// nvjpeg_decoder that also decodes every frame with turbojpeg and dies if
// the hardware output differs, so a bench or robot run turns any libnvjpeg
// misbehaviour into a crash instead of bad detections.  That includes the
// engine handing back a stale or delayed frame, whose pixels belong to an
// earlier JPEG and so do not match this one's CPU decode.  Swap it in for
// nvjpeg_decoder (executable_name in the config); it costs a CPU decode per
// frame, so it is not for matches.

#include <cstdlib>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "frc/vision/nvjpeg_decoder_service.h"
#include "turbojpeg.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "File path of aos configuration");

namespace frc::vision {
namespace {

// Mean absolute difference, in grey levels, under which the hardware and
// CPU decodes of one JPEG count as the same image.  They agree to well
// under 1; two different camera frames differ by 20+.
constexpr double kSameImage = 2.0;

double MeanAbsDiff(const uint8_t *a, const uint8_t *b, size_t size) {
  uint64_t sum = 0;
  for (size_t i = 0; i < size; ++i) {
    sum += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
  }
  return static_cast<double>(sum) / size;
}

class VerifyingNvjpegDecoder : public NvjpegDecoder {
 public:
  explicit VerifyingNvjpegDecoder(aos::EventLoop *event_loop)
      : NvjpegDecoder(event_loop), cpu_decoder_(tjInitDecompress()) {
    ABSL_CHECK(cpu_decoder_ != nullptr)
        << ": failed to create turbojpeg decoder";
    ABSL_LOG(INFO) << "Checking every frame against turbojpeg";
  }
  ~VerifyingNvjpegDecoder() override { tjDestroy(cpu_decoder_); }

 protected:
  void CheckDecode(const CameraImage &image, const uint8_t *gray, uint32_t rows,
                   uint32_t cols) override {
    const size_t size = static_cast<size_t>(rows) * cols;
    aos::ScopedNotRealtime nrt;
    if (cpu_gray_.size() < size) {
      cpu_gray_.resize(size);
    }
    if (tjDecompress2(cpu_decoder_, image.data()->data(), image.data()->size(),
                      cpu_gray_.data(), cols, 0, rows, TJPF_GRAY, 0) != 0) {
      // turbojpeg also fails on warnings, e.g. a corrupt frame the hardware
      // decoded anyway.  The two recover from those differently, so there
      // is nothing to compare against.
      ++skipped_;
      return;
    }
    const double diff = MeanAbsDiff(gray, cpu_gray_.data(), size);
    ABSL_CHECK_LT(diff, kSameImage)
        << ": hardware decode of a " << cols << "x" << rows
        << " frame differs from the CPU decode";
    ++checked_;
    ABSL_LOG_EVERY_N_SEC(INFO, 10)
        << "Checked " << checked_ << " frames against turbojpeg, skipped "
        << skipped_ << " that turbojpeg could not decode";
  }

 private:
  tjhandle cpu_decoder_;
  std::vector<uint8_t> cpu_gray_;
  uint64_t checked_ = 0;
  uint64_t skipped_ = 0;
};

}  // namespace

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  event_loop.SetRuntimeRealtimePriority(5);

  VerifyingNvjpegDecoder nvjpeg_decoder(&event_loop);

  event_loop.Run();

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  return frc::vision::Main();
}
