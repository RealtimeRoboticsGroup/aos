#include "frc/zeroing/absolute_encoder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "frc/zeroing/zeroing_test.h"

namespace frc::zeroing::testing {

using constants::AbsoluteEncoderZeroingConstants;

class AbsoluteEncoderZeroingTest : public ZeroingTest {
 protected:
  void MoveTo(PositionSensorSimulator *simulator,
              AbsoluteEncoderZeroingEstimator *estimator, double new_position) {
    simulator->MoveTo(new_position);
    flatbuffers::FlatBufferBuilder fbb;
    estimator->UpdateEstimate(
        *simulator->FillSensorValues<AbsolutePosition>(&fbb));
  }
};

// Makes sure that using an absolute encoder lets us zero without moving.
TEST_F(AbsoluteEncoderZeroingTest, TestAbsoluteEncoderZeroingWithoutMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double kMiddlePosition = 2.5;
  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;

  AbsoluteEncoderZeroingConstants constants{{},
                                            kSampleSize,
                                            index_diff,
                                            measured_absolute_position,
                                            kMiddlePosition,
                                            0.1,
                                            kMovingBufferSize,
                                            kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  AbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());
}

// Makes sure that we ignore a NAN if we get it, but will correctly zero
// afterwards.
TEST_F(AbsoluteEncoderZeroingTest, TestAbsoluteEncoderZeroingIgnoresNAN) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 2.1;
  double measured_absolute_position = 0.3 * index_diff;
  const double kMiddlePosition = 2.5;

  AbsoluteEncoderZeroingConstants constants{{},
                                            kSampleSize,
                                            index_diff,
                                            measured_absolute_position,
                                            kMiddlePosition,
                                            0.1,
                                            kMovingBufferSize,
                                            kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  AbsoluteEncoderZeroingEstimator estimator(constants);

  // We tolerate a couple NANs before we start.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN()));
  const auto sensor_values =
      flatbuffers::GetRoot<AbsolutePosition>(fbb.GetBufferPointer());
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos);
    ASSERT_FALSE(estimator.zeroed());
  }

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_TRUE(estimator.zeroed());
  EXPECT_DOUBLE_EQ(start_pos, estimator.offset());
}

// Makes sure that using an absolute encoder doesn't let us zero while moving.
TEST_F(AbsoluteEncoderZeroingTest, TestAbsoluteEncoderZeroingWithMovement) {
  const double index_diff = 1.0;
  PositionSensorSimulator sim(index_diff);

  const double start_pos = 10 * index_diff;
  double measured_absolute_position = 0.3 * index_diff;
  const double kMiddlePosition = 2.5;

  AbsoluteEncoderZeroingConstants constants{{},
                                            kSampleSize,
                                            index_diff,
                                            measured_absolute_position,
                                            kMiddlePosition,
                                            0.1,
                                            kMovingBufferSize,
                                            kIndexErrorFraction};

  sim.Initialize(start_pos, index_diff / 3.0, 0.0,
                 constants.measured_absolute_position);

  AbsoluteEncoderZeroingEstimator estimator(constants);

  for (size_t i = 0; i < kSampleSize + kMovingBufferSize - 1; ++i) {
    MoveTo(&sim, &estimator, start_pos + i * index_diff);
    ASSERT_FALSE(estimator.zeroed());
  }
  MoveTo(&sim, &estimator, start_pos + 10 * index_diff);

  MoveTo(&sim, &estimator, start_pos);
  ASSERT_FALSE(estimator.zeroed());
}

// Makes sure we detect an error if the ZeroingEstimator gets sent a NaN.
TEST_F(AbsoluteEncoderZeroingTest, TestAbsoluteEncoderZeroingWithNaN) {
  AbsoluteEncoderZeroingConstants constants{{},
                                            kSampleSize,
                                            1,
                                            0.3,
                                            1.0,
                                            0.1,
                                            kMovingBufferSize,
                                            kIndexErrorFraction};

  AbsoluteEncoderZeroingEstimator estimator(constants);

  flatbuffers::FlatBufferBuilder fbb;
  fbb.Finish(CreateAbsolutePosition(
      fbb, 0.0, ::std::numeric_limits<double>::quiet_NaN()));
  const auto sensor_values =
      flatbuffers::GetRoot<AbsolutePosition>(fbb.GetBufferPointer());
  for (size_t i = 0; i < kSampleSize - 1; ++i) {
    estimator.UpdateEstimate(*sensor_values);
  }
  ASSERT_FALSE(estimator.error());

  estimator.UpdateEstimate(*sensor_values);
  ASSERT_TRUE(estimator.error());

  flatbuffers::FlatBufferBuilder fbb2;
  fbb2.Finish(estimator.GetEstimatorState(&fbb2));

  const AbsoluteEncoderEstimatorState *state =
      flatbuffers::GetRoot<AbsoluteEncoderEstimatorState>(
          fbb2.GetBufferPointer());

  EXPECT_THAT(*state->errors(),
              ::testing::ElementsAre(ZeroingError::LOST_ABSOLUTE_ENCODER));
}

}  // namespace frc::zeroing::testing
