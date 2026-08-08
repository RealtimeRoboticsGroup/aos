// Standalone benchmark for the turbojpeg CPU decode path.  Decodes a JPEG
// file N times with the exact per-frame call sequence turbojpeg_decoder.cc
// uses (tjDecompressHeader3 + tjDecompress2 straight to TJPF_GRAY, default
// flags) and reports per-frame decode time.  Counterpart to nvjpeg_bench for
// apples-to-apples decoder comparisons on the Orin.
//
// Usage: turbojpeg_bench <file.jpg> [iterations] [out.pgm]

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"

#include "turbojpeg.h"

int main(int argc, char **argv) {
  absl::InitializeLog();
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  if (argc < 2) {
    LOG(ERROR) << "usage: " << argv[0] << " <file.jpg> [iterations] [out.pgm]";
    return 1;
  }
  const int iterations = argc > 2 ? atoi(argv[2]) : 20;

  FILE *f = fopen(argv[1], "rb");
  if (f == nullptr) {
    LOG(ERROR) << "open " << argv[1] << ": " << strerror(errno);
    return 1;
  }
  fseek(f, 0, SEEK_END);
  const long file_size = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<unsigned char> jpeg(file_size);
  if (fread(jpeg.data(), 1, file_size, f) != static_cast<size_t>(file_size)) {
    LOG(ERROR) << "short read of " << argv[1];
    return 1;
  }
  fclose(f);

  tjhandle handle = tjInitDecompress();
  if (handle == nullptr) {
    LOG(ERROR) << "tjInitDecompress failed";
    return 1;
  }

  std::vector<unsigned char> gray;
  double total_us = 0;
  double min_us = 1e12;
  int width = 0, height = 0;

  for (int iter = 0; iter < iterations; ++iter) {
    const auto start = std::chrono::steady_clock::now();

    int subsamp = 0, colorspace = 0;
    if (tjDecompressHeader3(handle, jpeg.data(), jpeg.size(), &width, &height,
                            &subsamp, &colorspace) != 0) {
      LOG(ERROR) << "tjDecompressHeader3 failed: " << tjGetErrorStr();
      return 1;
    }
    gray.resize(static_cast<size_t>(width) * height);
    if (tjDecompress2(handle, jpeg.data(), jpeg.size(), gray.data(), width,
                      0 /* pitch */, height, TJPF_GRAY, 0) != 0) {
      LOG(ERROR) << "tjDecompress2 failed: " << tjGetErrorStr();
      return 1;
    }

    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    total_us += us;
    if (us < min_us) min_us = us;
    if (iter == 0) {
      LOG(INFO) << absl::StrFormat("image: %dx%d, subsamp=%d, colorspace=%d",
                                   width, height, subsamp, colorspace);
      LOG(INFO) << absl::StrFormat("first decode: %.1f us", us);
    }
  }

  LOG(INFO) << absl::StrFormat("%d iterations: mean %.1f us, min %.1f us",
                               iterations, total_us / iterations, min_us);

  if (argc > 3) {
    FILE *out = fopen(argv[3], "wb");
    if (out == nullptr) {
      LOG(ERROR) << "open " << argv[3] << ": " << strerror(errno);
      return 1;
    }
    fprintf(out, "P5\n%d %d\n255\n", width, height);
    fwrite(gray.data(), 1, gray.size(), out);
    fclose(out);
    LOG(INFO) << "wrote " << argv[3];
  }

  tjDestroy(handle);
  return 0;
}
