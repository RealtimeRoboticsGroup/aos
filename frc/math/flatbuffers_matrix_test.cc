#include "frc/math/flatbuffers_matrix.h"

#include "gtest/gtest.h"

#include "aos/json_to_flatbuffer.h"

namespace frc::testing {

class FlatbuffersMatrixTest : public ::testing::Test {
 protected:
  template <int Rows, int Cols,
            fbs::StorageOrder StorageOrder = fbs::StorageOrder::ColMajor>
  tl::expected<typename EigenMatrix<Rows, Cols, StorageOrder>::type,
               ConversionFailure>
  ToEigen(std::string_view json) {
    return frc::ToEigen<Rows, Cols, StorageOrder>(
        aos::FlatbufferDetachedBuffer<fbs::Matrix>(
            aos::JsonToFlatbuffer<fbs::Matrix>(json))
            .message());
  }
};

TEST_F(FlatbuffersMatrixTest, ReadWriteMatrix) {
  const Eigen::Matrix<double, 3, 4> expected{
      {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}};
  aos::fbs::Builder<fbs::MatrixStatic> builder;
  flatbuffers::FlatBufferBuilder fbb;
  ASSERT_TRUE(FromEigen(expected, builder.get()));
  fbb.Finish(FromEigen<3, 4>(expected, &fbb));
  EXPECT_EQ(
      "{ \"rows\": 3, \"cols\": 4, \"storage_order\": \"ColMajor\", \"data\": "
      "[ 0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11 ] }",
      aos::FlatbufferToJson(builder.AsFlatbufferSpan()));
  EXPECT_EQ(
      "{ \"rows\": 3, \"cols\": 4, \"storage_order\": \"ColMajor\", \"data\": "
      "[ 0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11 ] }",
      aos::FlatbufferToJson(
          aos::FlatbufferDetachedBuffer<fbs::Matrix>(fbb.Release())));

  const Eigen::Matrix<double, 3, 4> result =
      ToEigenOrDie<3, 4>(builder->AsFlatbuffer());
  EXPECT_EQ(expected, result);
}

TEST_F(FlatbuffersMatrixTest, ReadWriteMatrixRowMajor) {
  const Eigen::Matrix<double, 3, 4, Eigen::StorageOptions::RowMajor> expected{
      {0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}};
  aos::fbs::Builder<fbs::MatrixStatic> builder;
  ASSERT_TRUE(FromEigen(expected, builder.get()));
  EXPECT_EQ(
      "{ \"rows\": 3, \"cols\": 4, \"storage_order\": \"RowMajor\", \"data\": "
      "[ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 ] }",
      aos::FlatbufferToJson(builder.AsFlatbufferSpan()));

  const Eigen::Matrix<double, 3, 4, Eigen::StorageOptions::RowMajor> result =
      ToEigenOrDie<3, 4, fbs::StorageOrder::RowMajor>(builder->AsFlatbuffer());
  EXPECT_EQ(expected, result);
}

class FlatbuffersMatrixParamTest
    : public FlatbuffersMatrixTest,
      public ::testing::WithParamInterface<
          std::tuple<std::string, ConversionFailure>> {};
TEST_P(FlatbuffersMatrixParamTest, ConversionFailures) {
  auto result = this->ToEigen<3, 4>(std::get<0>(GetParam()));
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(std::get<1>(GetParam()), result.error());
}

INSTANTIATE_TEST_SUITE_P(
    ConversionFailureTests, FlatbuffersMatrixParamTest,
    ::testing::Values(
        std::make_tuple("{}", ConversionFailure{fbs::MatrixField::kRows,
                                                fbs::FieldError::kMissing}),
        std::make_tuple(R"json({"rows": 3})json",
                        ConversionFailure{fbs::MatrixField::kCols,
                                          fbs::FieldError::kMissing}),
        std::make_tuple(R"json({"rows": 3, "cols": 4})json",
                        ConversionFailure{fbs::MatrixField::kData,
                                          fbs::FieldError::kMissing}),
        std::make_tuple(
            R"json({"rows": 1, "cols": 4, "data": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]})json",
            ConversionFailure{fbs::MatrixField::kRows,
                              fbs::FieldError::kInconsistentWithTemplate}),
        std::make_tuple(
            R"json({"rows": 3, "cols": 7, "data": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]})json",
            ConversionFailure{fbs::MatrixField::kCols,
                              fbs::FieldError::kInconsistentWithTemplate}),
        std::make_tuple(R"json({"rows": 3, "cols": 4, "data": []})json",
                        ConversionFailure{
                            fbs::MatrixField::kData,
                            fbs::FieldError::kInconsistentWithTemplate}),
        std::make_tuple(
            R"json({"rows": 3, "cols": 4, "storage_order": "RowMajor", "data": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]})json",
            ConversionFailure{fbs::MatrixField::kStorageOrder,
                              fbs::FieldError::kInconsistentWithTemplate})));
}  // namespace frc::testing
