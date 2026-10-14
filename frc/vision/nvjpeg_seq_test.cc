// Hardware sequence test for NvJpegDecoderLib: decodes DIFFERENT JPEGs in
// order through ONE decoder instance (like the long-running service does)
// and checks every output against turbojpeg's decode of the same frame.
// Exposes the persistent-state bugs libnvjpeg has shown on other call
// sequences -- stale frames, a one-frame pipeline delay, never-written
// output, a mapping of the wrong buffer -- none of which a same-file-in-a-
// loop benchmark can see, and all of which report success.
//
// This test needs the real NVJPG engine, so it only runs on an Orin -- CI
// hosts are x86 and skip it as platform-incompatible.  To run it:
//
//   bazel build --config=arm64 //frc/vision:nvjpeg_seq_test
//   rsync -a bazel-bin/frc/vision/nvjpeg_seq_test{,.runfiles} orin:
//   ssh orin 'cd nvjpeg_seq_test.runfiles/$(ls nvjpeg_seq_test.runfiles | \
//       head -1) && ../../nvjpeg_seq_test'
//
// camera_{a,b,c}.jpg are real 1280x800 4:2:2 frames from the robot's UVC
// cameras (no Huffman tables, as the cameras send them).  seq_*.jpg are
// small flat gradients.

#include <sys/wait.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/testing/path.h"
#include "frc/vision/gray_image_diff.h"
#include "frc/vision/nvjpeg_decoder_lib.h"
#include "turbojpeg.h"

namespace frc::vision::testing {
namespace {

// Output buffer size: large enough for every test frame.
constexpr size_t kMaxPixels = 1280 * 800;

std::vector<uint8_t> ReadTestJpeg(const std::string &name) {
  const std::string path =
      aos::testing::ArtifactPath("frc/vision/testdata/" + name);
  std::ifstream file(path, std::ios::binary);
  ABSL_CHECK(file) << ": failed to open " << path;
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

// A decoded grayscale image.
struct Gray {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> pixels;
};

// The independent reference: turbojpeg's grayscale decode.
Gray TurboDecode(const std::vector<uint8_t> &jpeg) {
  tjhandle handle = tjInitDecompress();
  ABSL_CHECK(handle != nullptr);
  Gray gray;
  int subsamp, colorspace;
  ABSL_CHECK_EQ(
      tjDecompressHeader3(handle, jpeg.data(), jpeg.size(), &gray.width,
                          &gray.height, &subsamp, &colorspace),
      0)
      << tjGetErrorStr();
  gray.pixels.resize(static_cast<size_t>(gray.width) * gray.height);
  ABSL_CHECK_EQ(
      tjDecompress2(handle, jpeg.data(), jpeg.size(), gray.pixels.data(),
                    gray.width, 0, gray.height, TJPF_GRAY, 0),
      0)
      << tjGetErrorStr();
  tjDestroy(handle);
  return gray;
}

// Encodes a noisy test pattern as a JPEG with the given turbojpeg
// subsampling, for formats the cameras never send.
std::vector<uint8_t> EncodeTestPattern(int width, int height, int subsamp) {
  std::vector<uint8_t> rgb(static_cast<size_t>(width) * height * 3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      uint8_t *pixel = &rgb[(static_cast<size_t>(y) * width + x) * 3];
      pixel[0] = static_cast<uint8_t>(x * 3 + y);
      pixel[1] = static_cast<uint8_t>(x ^ y);
      pixel[2] = static_cast<uint8_t>(y * 5);
    }
  }
  tjhandle handle = tjInitCompress();
  ABSL_CHECK(handle != nullptr);
  unsigned char *jpeg = nullptr;
  unsigned long jpeg_size = 0;
  ABSL_CHECK_EQ(tjCompress2(handle, rgb.data(), width, 0, height, TJPF_RGB,
                            &jpeg, &jpeg_size, subsamp, 90, 0),
                0)
      << tjGetErrorStr();
  std::vector<uint8_t> result(jpeg, jpeg + jpeg_size);
  tjFree(jpeg);
  tjDestroy(handle);
  return result;
}

// The field failure mode this decoder sees in production: a frame whose
// leading USB payload (SOI marker + tables) was dropped, leaving the buffer
// starting at the SOF0 marker 0xff 0xc0.
std::vector<uint8_t> StripThroughSof0(std::vector<uint8_t> jpeg) {
  for (size_t i = 2; i + 1 < jpeg.size(); ++i) {
    if (jpeg[i] == 0xFF && jpeg[i + 1] == 0xC0) {
      jpeg.erase(jpeg.begin(), jpeg.begin() + i);
      return jpeg;
    }
  }
  ABSL_LOG(FATAL) << "test JPEG contains no SOF0 marker";
}

// One test frame: the JPEG and turbojpeg's decode of it.
struct Frame {
  explicit Frame(const std::string &name)
      : name(name), jpeg(ReadTestJpeg(name)), reference(TurboDecode(jpeg)) {}

  std::string name;
  std::vector<uint8_t> jpeg;
  Gray reference;
};

// Decodes |frame| through |decoder| and returns how far the output is from
// |frame|'s reference, or -1 if the decode failed.  The buffer is poisoned
// first so a decode that writes nothing cannot pass with a previous
// decode's pixels.
double DecodeAndDiff(NvJpegDecoderLib *decoder, const Frame &frame,
                     std::vector<uint8_t> *gray) {
  memset(gray->data(), 0xAA, gray->size());
  NvJpegDecoderLib::Result result;
  if (!decoder->DecodeToGray(frame.jpeg.data(), frame.jpeg.size(), gray->data(),
                             gray->size(), &result)) {
    return -1;
  }
  if (static_cast<int>(result.width) != frame.reference.width ||
      static_cast<int>(result.height) != frame.reference.height) {
    return -1;
  }
  return MeanAbsDiff(
      absl::MakeConstSpan(gray->data(), frame.reference.pixels.size()),
      frame.reference.pixels);
}

class NvJpegSeqTest : public ::testing::Test {
 protected:
  NvJpegSeqTest()
      : camera_{Frame("camera_a.jpg"), Frame("camera_b.jpg"),
                Frame("camera_c.jpg")},
        small_{Frame("seq_040.jpg"), Frame("seq_080.jpg"), Frame("seq_120.jpg"),
               Frame("seq_200.jpg")},
        gray_(kMaxPixels) {}

  // Decodes |frame| and expects it to match its own reference.
  void ExpectDecodesToItself(const Frame &frame) {
    const double diff = DecodeAndDiff(&decoder_, frame, &gray_);
    EXPECT_GE(diff, 0) << frame.name << ": " << decoder_.last_error();
    EXPECT_LT(diff, kSameImage) << frame.name;
  }

  NvJpegDecoderLib decoder_;
  const Frame camera_[3];
  const Frame small_[4];
  std::vector<uint8_t> gray_;
};

// The frames must differ from each other by far more than kSameImage, or
// the other tests could not tell a stale or delayed frame from the right
// one.
TEST_F(NvJpegSeqTest, TestFramesAreDistinguishable) {
  for (int i = 0; i < 3; ++i) {
    const Frame &a = camera_[i];
    const Frame &b = camera_[(i + 1) % 3];
    EXPECT_GT(MeanAbsDiff(a.reference.pixels, b.reference.pixels),
              5 * kSameImage)
        << a.name << " vs " << b.name;
  }
}

// Every decode returns the frame passed in -- not the previous one (the
// stream-mode surface readback's one-frame delay), not a stale one (the
// first-frame-only bug), not an unwritten buffer.  Run long enough to cycle
// the library's rotating output buffers several times.
TEST_F(NvJpegSeqTest, EveryDecodeReturnsTheCurrentFrame) {
  for (int i = 0; i < 60; ++i) {
    ExpectDecodesToItself(camera_[i % 3]);
  }
}

TEST_F(NvJpegSeqTest, SmallFramesReturnTheCurrentFrame) {
  for (int i = 0; i < 12; ++i) {
    ExpectDecodesToItself(small_[i % 4]);
  }
}

// A frame that lost its head off the camera (the production corrupt-frame
// signature) must fail with the reason readable through last_error(), and
// the next frame must decode correctly.
TEST_F(NvJpegSeqTest, CorruptFrameReportsReasonAndRecovers) {
  ExpectDecodesToItself(camera_[0]);

  const std::vector<uint8_t> corrupt = StripThroughSof0(camera_[1].jpeg);
  NvJpegDecoderLib::Result result;
  EXPECT_FALSE(decoder_.DecodeToGray(corrupt.data(), corrupt.size(),
                                     gray_.data(), gray_.size(), &result));
  EXPECT_NE(decoder_.last_error().find("Not a JPEG"), std::string_view::npos)
      << decoder_.last_error();

  ExpectDecodesToItself(camera_[2]);
  ExpectDecodesToItself(camera_[1]);
}

// A mid-stream dimension change must fail with the reason in last_error(),
// and the restarted stream must then decode correctly.
TEST_F(NvJpegSeqTest, DimensionChangeMidStreamFailsAndRecovers) {
  ExpectDecodesToItself(camera_[0]);

  NvJpegDecoderLib::Result result;
  EXPECT_FALSE(decoder_.DecodeToGray(small_[0].jpeg.data(),
                                     small_[0].jpeg.size(), gray_.data(),
                                     gray_.size(), &result));
  EXPECT_NE(decoder_.last_error().find("dimensions changed"),
            std::string_view::npos)
      << decoder_.last_error();

  ExpectDecodesToItself(camera_[1]);
  ExpectDecodesToItself(camera_[2]);
}

// ResetStream starts a clean stream (the service resets after a failed
// cross-check), and the new stream may use new dimensions.
TEST_F(NvJpegSeqTest, ResetStreamAllowsANewSize) {
  ExpectDecodesToItself(small_[0]);
  ExpectDecodesToItself(small_[1]);
  decoder_.ResetStream();
  ExpectDecodesToItself(camera_[0]);
  ExpectDecodesToItself(camera_[1]);
}

// Formats the cameras never send are rejected before they reach libnvjpeg,
// which segfaults on 4:4:4 instead of reporting an error.  The decoder
// keeps working afterwards.
TEST_F(NvJpegSeqTest, UnsupportedFormatsAreRejectedWithoutCrashing) {
  ExpectDecodesToItself(camera_[0]);
  for (const int subsamp : {TJSAMP_444, TJSAMP_420, TJSAMP_GRAY}) {
    const std::vector<uint8_t> jpeg = EncodeTestPattern(640, 400, subsamp);
    NvJpegDecoderLib::Result result;
    EXPECT_FALSE(decoder_.DecodeToGray(jpeg.data(), jpeg.size(), gray_.data(),
                                       gray_.size(), &result))
        << "subsampling " << subsamp;
    EXPECT_NE(decoder_.last_error().find("Unsupported JPEG format"),
              std::string_view::npos)
        << decoder_.last_error();
  }
  ExpectDecodesToItself(camera_[1]);
  ExpectDecodesToItself(camera_[2]);
}

// Production runs one decoder process per camera, all sharing the Orin's
// two NVJPG engines.  Each process must still get its own current frame.
TEST_F(NvJpegSeqTest, ConcurrentProcessesEachGetTheirOwnFrames) {
  constexpr int kProcesses = 4;
  constexpr int kDecodes = 300;
  std::vector<pid_t> children;
  for (int p = 0; p < kProcesses; ++p) {
    const pid_t pid = fork();
    ASSERT_GE(pid, 0);
    if (pid == 0) {
      // Child: its own decoder, frames in a per-process order.  Exit code =
      // number of bad decodes (capped), so no gtest state crosses the fork.
      NvJpegDecoderLib decoder;
      std::vector<uint8_t> gray(kMaxPixels);
      int bad = 0;
      for (int i = 0; i < kDecodes; ++i) {
        const double diff =
            DecodeAndDiff(&decoder, camera_[(i + p) % 3], &gray);
        if (diff < 0 || diff >= kSameImage) ++bad;
      }
      _exit(bad > 100 ? 100 : bad);
    }
    children.push_back(pid);
  }
  for (const pid_t pid : children) {
    int status = 0;
    ASSERT_EQ(waitpid(pid, &status, 0), pid);
    ASSERT_TRUE(WIFEXITED(status)) << "child crashed, status " << status;
    EXPECT_EQ(WEXITSTATUS(status), 0) << "bad decodes in child " << pid;
  }
}

}  // namespace
}  // namespace frc::vision::testing
