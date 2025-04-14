#include <iostream>

#include "Halide.h"

#define CHECK(x, message, ...)                                              \
  do {                                                                      \
    if (!(x)) {                                                             \
      fprintf(stderr, "assertion failed: " message ": %s\n", ##__VA_ARGS__, \
              #x);                                                          \
      abort();                                                              \
    }                                                                       \
  } while (0)

// This is a Halide "generator". This means it is a binary which generates
// ahead-of-time optimized functions as directed by command-line arguments.
// https://halide-lang.org/tutorials/tutorial_lesson_15_generators.html has an
// introduction to much of the magic in this file.

namespace frc::orin {

class ResizeNormalize : public Halide::Generator<ResizeNormalize> {
 public:
  GeneratorParam<int> input_rows{"input_rows", 0};
  GeneratorParam<int> input_cols{"input_cols", 0};
  GeneratorParam<int> input_start_row{"input_start_row", -1};
  GeneratorParam<int> input_start_col{"input_start_col", -1};

  GeneratorParam<int> output_rows{"output_rows", 0};
  GeneratorParam<int> output_cols{"output_cols", 0};
  GeneratorParam<int> output_stride{"output_stride", 0};

  Input<Buffer<uint8_t, 2>> input{"input"};
  Output<Buffer<float, 3>> output{"output"};

  Var col{"col"}, row{"row"}, channel{"channel"};

  // Everything is indexed as col, row, channel.
  void generate() {
    CHECK(input_rows > 0, "Must specify a input_rows");
    CHECK(input_cols > 0, "Must specify a input_cols");
    CHECK(input_start_row >= 0, "Must specify a input_start_row");
    CHECK(input_start_col >= 0, "Must specify a input_start_col");
    CHECK(output_rows > 0, "Must specify a output_rows");
    CHECK(output_cols > 0, "Must specify a output_cols");

    input.dim(0).set_stride(1);
    input.dim(0).set_extent(input_cols);
    input.dim(0).set_min(0);

    input.dim(1).set_stride(input_cols);
    input.dim(1).set_extent(input_rows);
    input.dim(1).set_min(0);

    Func normalized_avg("normalized_avg");

    Expr sum_u16 =
        cast<uint16_t>(input(col * 3 + 0 + input_start_col,
                                          row * 3 + 0 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 0 + input_start_col,
                                          row * 3 + 1 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 0 + input_start_col,
                                          row * 3 + 2 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 1 + input_start_col,
                                          row * 3 + 0 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 1 + input_start_col,
                                          row * 3 + 1 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 1 + input_start_col,
                                          row * 3 + 2 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 2 + input_start_col,
                                          row * 3 + 0 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 2 + input_start_col,
                                          row * 3 + 1 + input_start_row)) +
                     cast<uint16_t>(input(col * 3 + 2 + input_start_col,
                                          row * 3 + 2 + input_start_row));

    normalized_avg(col, row) = cast<float>(sum_u16) * (1.0f / (9.0f * 255.0f));

    output(col, row, channel) = normalized_avg(col, row);

    normalized_avg.compute_at(output, channel);

    output.reorder(channel, col, row);

    output.vectorize(col, 32);
    output.unroll(channel, 3);

    output.dim(0).set_stride(1);
    output.dim(0).set_extent(output_cols);
    output.dim(0).set_min(0);

    output.dim(1).set_stride(output_cols);
    output.dim(1).set_extent(output_rows);
    output.dim(1).set_min(0);

    output.dim(2).set_stride(output_stride);
    output.dim(2).set_extent(3);
    output.dim(2).set_min(0);
  }
};

}  // namespace frc::orin

HALIDE_REGISTER_GENERATOR(frc::orin::ResizeNormalize, resize_normalize)
