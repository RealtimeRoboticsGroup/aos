// On-bus latency test for a running JPEG decoder service (nvjpeg_decoder or
// turbojpeg_decoder), for comparing decoder builds on an Orin with no
// cameras attached.
//
// Publishes real camera JPEGs on --channel at --rate_hz, rotating through
// the files matched by --frames, each stamped with its send time.  Watches
// <channel>/gray and identifies which input frame every output image
// actually contains by comparing its pixels with turbojpeg's decode of each
// input -- NOT by its timestamp, which a decoder can relabel.  Reports:
//
//   * pixel latency: output publish time minus the send time of the frame
//     whose pixels it carries;
//   * frames late: how many newer frames had already been sent when the
//     output was published (0 = the output is the newest frame);
//   * outputs whose timestamp does not match their pixels' frame, and
//     outputs that match no input (e.g. all-zero frames).
//
// The decoder's --skip applies, so with --skip=1 only every other sent
// frame comes back.  Stop the camera_reader for the channel first (on a
// bench with no cameras it is not running).

#include <glob.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/str_format.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/vision/gray_image_diff.h"
#include "frc/vision/vision_generated.h"
#include "turbojpeg.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");
ABSL_FLAG(
    std::string, channel, "/camera1",
    "Camera channel to publish on; results are read from <channel>/gray.");
ABSL_FLAG(double, rate_hz, 120.0, "Rate to publish frames at.");
ABSL_FLAG(double, seconds, 10.0, "How long to publish for.");
ABSL_FLAG(std::string, frames, "",
          "Glob of 1280x800 camera JPEGs to rotate through (at least 4 "
          "distinct frames).");

namespace frc::vision {
namespace {

// Compare every kStride-th pixel when identifying a frame.
constexpr size_t kStride = 61;

struct Frame {
  std::vector<uint8_t> jpeg;
  std::vector<uint8_t> gray;
  int width = 0;
  int height = 0;
};

std::vector<Frame> LoadFrames(const std::string &pattern) {
  glob_t matches;
  ABSL_CHECK_EQ(glob(pattern.c_str(), 0, nullptr, &matches), 0)
      << ": no files match " << pattern;
  tjhandle tj = tjInitDecompress();
  ABSL_CHECK(tj != nullptr);
  std::vector<Frame> frames;
  for (size_t i = 0; i < matches.gl_pathc; ++i) {
    std::ifstream in(matches.gl_pathv[i], std::ios::binary);
    Frame frame;
    frame.jpeg.assign(std::istreambuf_iterator<char>(in), {});
    int subsamp, colorspace;
    ABSL_CHECK_EQ(
        tjDecompressHeader3(tj, frame.jpeg.data(), frame.jpeg.size(),
                            &frame.width, &frame.height, &subsamp, &colorspace),
        0)
        << matches.gl_pathv[i] << ": " << tjGetErrorStr();
    frame.gray.resize(static_cast<size_t>(frame.width) * frame.height);
    ABSL_CHECK_EQ(tjDecompress2(tj, frame.jpeg.data(), frame.jpeg.size(),
                                frame.gray.data(), frame.width, 0, frame.height,
                                TJPF_GRAY, 0),
                  0)
        << matches.gl_pathv[i] << ": " << tjGetErrorStr();
    frames.push_back(std::move(frame));
  }
  globfree(&matches);
  tjDestroy(tj);
  ABSL_CHECK_GE(frames.size(), 4u) << ": need at least 4 frames";
  for (size_t i = 0; i < frames.size(); ++i) {
    ABSL_CHECK_EQ(frames[i].width, frames[0].width);
    ABSL_CHECK_EQ(frames[i].height, frames[0].height);
    for (size_t j = 0; j < i; ++j) {
      ABSL_CHECK_GT(MeanAbsDiff(frames[i].gray, frames[j].gray, kStride),
                    5 * kSameImage)
          << ": frames " << j << " and " << i
          << " are too similar to tell apart";
    }
  }
  return frames;
}

// One published input frame.
struct Sent {
  int64_t sequence;
  size_t frame_index;
  aos::monotonic_clock::time_point time;
  int64_t timestamp_ns;
};

// Outputs are never more than a few frames behind, so only the most recent
// sends are kept; memory stays flat however long the run is.
constexpr size_t kSendHistory = 1024;

double Percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0;
  std::sort(values.begin(), values.end());
  return values[std::min(values.size() - 1,
                         static_cast<size_t>(p * values.size()))];
}

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));
  aos::ShmEventLoop event_loop(&config.message());

  const std::vector<Frame> frames = LoadFrames(absl::GetFlag(FLAGS_frames));
  const std::string channel = absl::GetFlag(FLAGS_channel);
  aos::Sender<CameraImage> sender = event_loop.MakeSender<CameraImage>(channel);

  std::deque<Sent> sent;
  int64_t sends = 0;

  std::vector<double> latency_ms;
  std::map<int, int> frames_late_histogram;
  int unmatched = 0;
  int wrong_timestamp = 0;

  event_loop.MakeWatcher(channel + "/gray", [&](const CameraImage &image) {
    const aos::monotonic_clock::time_point published =
        event_loop.context().monotonic_event_time;
    if (image.data() == nullptr ||
        image.data()->size() != frames[0].gray.size()) {
      ++unmatched;
      return;
    }
    // Which input's pixels is this?
    size_t best = 0;
    double best_diff = 1e9;
    for (size_t i = 0; i < frames.size(); ++i) {
      const double diff = MeanAbsDiff(
          absl::MakeConstSpan(image.data()->data(), image.data()->size()),
          frames[i].gray, kStride);
      if (diff < best_diff) {
        best_diff = diff;
        best = i;
      }
    }
    if (best_diff >= kSameImage) {
      ++unmatched;
      return;
    }
    // The most recent send of that frame at or before the publish time.
    // Frames rotate through more files than any decoder is late by, so this
    // is the send the pixels came from.
    const Sent *source = nullptr;
    const Sent *newest = nullptr;
    for (auto it = sent.rbegin(); it != sent.rend(); ++it) {
      if (it->time > published) continue;
      if (newest == nullptr) newest = &*it;
      if (it->frame_index == best) {
        source = &*it;
        break;
      }
    }
    if (source == nullptr) {
      ++unmatched;
      return;
    }
    latency_ms.push_back(
        std::chrono::duration<double, std::milli>(published - source->time)
            .count());
    ++frames_late_histogram[newest->sequence - source->sequence];
    if (image.monotonic_timestamp_ns() != source->timestamp_ns) {
      ++wrong_timestamp;
    }
  });

  const std::chrono::nanoseconds period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / absl::GetFlag(FLAGS_rate_hz)));
  const int total_sends = static_cast<int>(absl::GetFlag(FLAGS_seconds) *
                                           absl::GetFlag(FLAGS_rate_hz));
  aos::TimerHandler *send_timer = event_loop.AddTimer([&]() {
    if (sends >= total_sends) {
      return;
    }
    const size_t index = sends % frames.size();
    const Frame &frame = frames[index];
    const aos::monotonic_clock::time_point now = event_loop.monotonic_now();
    auto builder = sender.MakeBuilder();
    auto data = builder.fbb()->CreateVector(frame.jpeg);
    CameraImage::Builder image(*builder.fbb());
    image.add_rows(frame.height);
    image.add_cols(frame.width);
    image.add_data(data);
    image.add_monotonic_timestamp_ns(now.time_since_epoch().count());
    image.add_format(ImageFormat::MJPEG);
    builder.CheckOk(builder.Send(image.Finish()));
    sent.push_back({sends, index, now, now.time_since_epoch().count()});
    if (sent.size() > kSendHistory) sent.pop_front();
    ++sends;
  });
  aos::TimerHandler *exit_timer =
      event_loop.AddTimer([&event_loop]() { event_loop.Exit(); });
  event_loop.OnRun([&]() {
    const aos::monotonic_clock::time_point now = event_loop.monotonic_now();
    send_timer->Schedule(now, period);
    // Leave time for the last frames to come back.
    exit_timer->Schedule(now + period * total_sends + std::chrono::seconds(1));
  });

  event_loop.Run();

  double sum = 0;
  for (double l : latency_ms) sum += l;
  ABSL_LOG(INFO) << absl::StrFormat(
      "%s at %.0f Hz: sent %d, got back %d matched + %d unmatched", channel,
      absl::GetFlag(FLAGS_rate_hz), sends, latency_ms.size(), unmatched);
  if (!latency_ms.empty()) {
    ABSL_LOG(INFO) << absl::StrFormat(
        "pixel latency ms: mean %.2f  p50 %.2f  p90 %.2f  p99 %.2f  max %.2f",
        sum / latency_ms.size(), Percentile(latency_ms, 0.5),
        Percentile(latency_ms, 0.9), Percentile(latency_ms, 0.99),
        Percentile(latency_ms, 1.0));
  }
  std::string histogram;
  for (const auto &[late, count] : frames_late_histogram) {
    absl::StrAppendFormat(&histogram, "  %d late x%d", late, count);
  }
  ABSL_LOG(INFO) << "frames late:" << histogram;
  ABSL_LOG(INFO) << "timestamp not matching pixels: " << wrong_timestamp;
  return 0;
}

}  // namespace
}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}
