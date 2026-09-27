// Hardware sequence test for NvJpegDecoderLib: decodes DIFFERENT JPEGs in
// order through ONE decoder instance (like the long-running service does)
// and asserts each output's pixels come from the frame the pipelining
// contract promises.  Exposes persistent-state bugs that a
// same-file-in-a-loop benchmark cannot see, e.g. stale or never-written
// hardware output surfaces (the libnvjpeg raw-data-out bug that motivated
// the stream-mode + surface-readback design; see nvjpeg_decoder_lib.cc).
//
// This test needs the real NVJPG engine, so it only runs on an Orin -- CI
// hosts are x86 and skip it as platform-incompatible.  To run it:
//
//   bazel build --config=arm64 //frc/vision:nvjpeg_seq_test
//   rsync -a bazel-bin/frc/vision/nvjpeg_seq_test{,.runfiles} orin:
//   ssh orin 'cd nvjpeg_seq_test.runfiles/$(ls nvjpeg_seq_test.runfiles | \
//       head -1) && ../../nvjpeg_seq_test'
//
// The test frames are flat gradients with distinct mean brightness (40, 80,
// 120, 200), so WHICH frame a decode's pixels came from is identified by
// the output mean: a stale frame, a warmup zero frame (mean 0), or a
// never-written poisoned buffer (mean 170 = 0xAA) all classify differently
// and cannot masquerade as the expected image.

#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "gtest/gtest.h"

#include "aos/testing/path.h"
#include "frc/vision/nvjpeg_decoder_lib.h"

namespace frc::vision::testing {
namespace {

// Dimensions and per-file mean brightness of the checked-in test frames.
constexpr int kWidth = 300;
constexpr int kHeight = 200;
constexpr double kMeanTolerance = 8.0;

std::vector<uint8_t> ReadTestJpeg(const std::string &name) {
  const std::string path =
      aos::testing::ArtifactPath("frc/vision/testdata/" + name);
  std::ifstream file(path, std::ios::binary);
  ABSL_CHECK(file) << ": failed to open " << path;
  return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
}

// The field failure mode this decoder sees in production: a frame whose
// leading USB payload (SOI marker + tables) was dropped, leaving the buffer
// starting at the SOF0 marker 0xff 0xc0.
std::vector<uint8_t> StripThroughSof0(std::vector<uint8_t> jpeg) {
  for (size_t i = 0; i + 1 < jpeg.size(); ++i) {
    if (jpeg[i] == 0xFF && jpeg[i + 1] == 0xC0) {
      jpeg.erase(jpeg.begin(), jpeg.begin() + i);
      return jpeg;
    }
  }
  ABSL_LOG(FATAL) << "test JPEG contains no SOF0 marker";
}

class NvJpegSeqTest : public ::testing::Test {
 protected:
  NvJpegSeqTest()
      : frame_040_(ReadTestJpeg("seq_040.jpg")),
        frame_080_(ReadTestJpeg("seq_080.jpg")),
        frame_120_(ReadTestJpeg("seq_120.jpg")),
        frame_200_(ReadTestJpeg("seq_200.jpg")),
        frame_other_size_(ReadTestJpeg("seq_other_size.jpg")),
        gray_(static_cast<size_t>(kWidth) * kHeight) {}

  // Decodes |jpeg| through |decoder| expecting success and returns the mean
  // of the output pixels.  The buffer is poisoned first so a decode that
  // writes nothing yields mean 170 (0xAA) instead of passing stale pixels.
  double DecodeMean(NvJpegDecoderLib *decoder,
                    const std::vector<uint8_t> &jpeg) {
    memset(gray_.data(), 0xAA, gray_.size());
    NvJpegDecoderLib::Result result;
    EXPECT_TRUE(decoder->DecodeToGray(jpeg.data(), jpeg.size(), gray_.data(),
                                      gray_.size(), &result))
        << decoder->last_error();
    EXPECT_EQ(result.width, static_cast<uint32_t>(kWidth));
    EXPECT_EQ(result.height, static_cast<uint32_t>(kHeight));
    unsigned long long sum = 0;
    for (const uint8_t v : gray_) {
      sum += v;
    }
    return static_cast<double>(sum) / gray_.size();
  }

  // The second decode of a fresh stream is normally the pipeline's all-zero
  // warmup bubble, but the contract only promises "typically" -- accept the
  // submitted-previous frame too.
  static void ExpectWarmupOrMean(double mean, double expected) {
    EXPECT_TRUE(mean < kMeanTolerance ||
                std::abs(mean - expected) < kMeanTolerance)
        << "mean " << mean << " is neither warmup-zero nor " << expected;
  }

  NvJpegDecoderLib decoder_;
  const std::vector<uint8_t> frame_040_;
  const std::vector<uint8_t> frame_080_;
  const std::vector<uint8_t> frame_120_;
  const std::vector<uint8_t> frame_200_;
  const std::vector<uint8_t> frame_other_size_;
  std::vector<uint8_t> gray_;
};

// Steady-state pipelining: the first decode of a stream returns its own
// frame, the second is the warmup bubble (or the previous frame), and every
// decode after that returns the PREVIOUS submission's pixels.  A decoder
// with the raw-data-out persistent-state bug fails here on the third frame:
// the output would be stale (previous mean) or never written (mean 170).
TEST_F(NvJpegSeqTest, SequenceReturnsPreviousFrame) {
  EXPECT_NEAR(DecodeMean(&decoder_, frame_040_), 40, kMeanTolerance);
  ExpectWarmupOrMean(DecodeMean(&decoder_, frame_080_), 40);
  EXPECT_NEAR(DecodeMean(&decoder_, frame_120_), 80, kMeanTolerance);
  EXPECT_NEAR(DecodeMean(&decoder_, frame_200_), 120, kMeanTolerance);
  EXPECT_NEAR(DecodeMean(&decoder_, frame_040_), 200, kMeanTolerance);
  EXPECT_NEAR(DecodeMean(&decoder_, frame_080_), 40, kMeanTolerance);
}

// A frame that lost its head off the camera (the production corrupt-frame
// signature) must fail with the reason readable through last_error(), and
// the rebuilt stream must decode the next frame synchronously.
TEST_F(NvJpegSeqTest, CorruptFrameReportsReasonAndRecovers) {
  EXPECT_NEAR(DecodeMean(&decoder_, frame_040_), 40, kMeanTolerance);

  const std::vector<uint8_t> corrupt = StripThroughSof0(frame_120_);
  NvJpegDecoderLib::Result result;
  memset(gray_.data(), 0xAA, gray_.size());
  EXPECT_FALSE(decoder_.DecodeToGray(corrupt.data(), corrupt.size(),
                                     gray_.data(), gray_.size(), &result));
  EXPECT_NE(decoder_.last_error().find("Not a JPEG"), std::string_view::npos)
      << decoder_.last_error();

  // The failure rebuilt the stream: the next decode is synchronous and
  // returns its own frame again.
  EXPECT_NEAR(DecodeMean(&decoder_, frame_200_), 200, kMeanTolerance);
  ExpectWarmupOrMean(DecodeMean(&decoder_, frame_080_), 200);
  EXPECT_NEAR(DecodeMean(&decoder_, frame_040_), 80, kMeanTolerance);
}

// A mid-stream dimension change must fail (the output surface is sized for
// the stream) with the reason in last_error(), then recover on the next
// frame at the original dimensions.
TEST_F(NvJpegSeqTest, DimensionChangeMidStreamFailsAndRecovers) {
  EXPECT_NEAR(DecodeMean(&decoder_, frame_040_), 40, kMeanTolerance);

  NvJpegDecoderLib::Result result;
  memset(gray_.data(), 0xAA, gray_.size());
  EXPECT_FALSE(decoder_.DecodeToGray(frame_other_size_.data(),
                                     frame_other_size_.size(), gray_.data(),
                                     gray_.size(), &result));
  EXPECT_NE(decoder_.last_error().find("dimensions changed"),
            std::string_view::npos)
      << decoder_.last_error();

  EXPECT_NEAR(DecodeMean(&decoder_, frame_120_), 120, kMeanTolerance);
}

// The recreate-per-frame mode (the old --fresh flag): a brand-new decoder's
// first decode is synchronous and must return its own frame's pixels.
TEST_F(NvJpegSeqTest, FreshDecoderDecodesFirstFrameSynchronously) {
  const std::vector<uint8_t> *frames[] = {&frame_040_, &frame_080_, &frame_120_,
                                          &frame_200_};
  const double means[] = {40, 80, 120, 200};
  for (int i = 0; i < 4; ++i) {
    NvJpegDecoderLib fresh_decoder;
    EXPECT_NEAR(DecodeMean(&fresh_decoder, *frames[i]), means[i],
                kMeanTolerance);
  }
}

}  // namespace
}  // namespace frc::vision::testing
