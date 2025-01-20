#include "frc/control_loops/position_sensor_sim.h"

#include <cmath>

namespace frc::control_loops {

/* Index pulse/segment Explanation:
 *
 * The index segments are labelled starting at zero and go up. Each index
 * segment is the space between the two bordering index pulses. The length of
 * each index segment is determined by the `index_diff` variable in the
 * constructor below.
 *
 * The index pulses are encountered when the mechanism moves from one index
 * segment to another.
 *
 *         index segment
 *               |
 *               V
 *
 * +--- 0---+--- 1---+--- 2---+--- 3---+--- 4---+--- 5---+--- 6---+
 *
 * |        |        |        |        |        |        |        |
 * 0        1        2        3        4        5        6        7
 *
 *                   A
 *                   |
 *              index pulse
 *
 *
 *
 * Absolute encoder explanation:
 *
 * If we were to graph the output of an absolute encoder that resets every 0.1
 * meters for example, it would looks something like the following. The y-axis
 * represents the output of the absolute encoder. The x-axis represents the
 * actual position of the robot's mechanism.
 *
 *          1 encoder segment
 *              +------+
 *
 *      |
 *  0.1 +      /|     /|     /|     /|     /|     /|     /|     /|
 *      |     / |    / |    / |    / |    / |    / |    / |    / |
 *      |    /  |   /  |   /  |   /  |   /  |   /  |   /  |   /  |
 *      |   /   |  /   |  /   |  /   |  /   |  /   |  /   |  /   |
 *      |  /    | /    | /    | /    | /    | /    | /    | /    |
 *      | /     |/     |/     |/     |/     |/     |/     |/     |
 *  0.0 ++------+------+------+------+------+------+------+------+----
 *      0.05   0.15   0.25   0.35   0.45   0.55   0.65   0.75   0.85
 *
 * An absolute encoder can be used to determine exactly where the mechanism in
 * question is within a certain segment. As long as you know a single combo of
 * absolute encoder reading and mechanism location you can extrapolate the
 * remainder of the graph.
 */

PositionSensorSimulator::PositionSensorSimulator(
    double distance_per_revolution, double single_turn_distance_per_revolution,
    unsigned int noise_seed)
    : lower_index_edge_(distance_per_revolution, noise_seed),
      upper_index_edge_(distance_per_revolution, noise_seed),
      distance_per_revolution_(distance_per_revolution),
      single_turn_distance_per_revolution_(
          single_turn_distance_per_revolution) {
  Initialize(0.0, 0.0);
}

void PositionSensorSimulator::Initialize(
    double start_position, double pot_noise_stddev, double known_index_position,
    double known_absolute_encoder_pos,
    double single_turn_known_absolute_encoder_pos) {
  InitializeHallEffectAndPosition(start_position, known_index_position,
                                  known_index_position);

  known_absolute_encoder_ = known_absolute_encoder_pos;
  single_turn_known_absolute_encoder_ = single_turn_known_absolute_encoder_pos;

  lower_index_edge_.mutable_pot_noise()->set_standard_deviation(
      pot_noise_stddev);
  upper_index_edge_.mutable_pot_noise()->set_standard_deviation(
      pot_noise_stddev);
}

void PositionSensorSimulator::InitializeHallEffectAndPosition(
    double start_position, double known_index_lower, double known_index_upper) {
  current_position_ = start_position;
  start_position_ = start_position;

  lower_index_edge_.Initialize(start_position, known_index_lower);
  upper_index_edge_.Initialize(start_position, known_index_upper);

  posedge_count_ = 0;
  negedge_count_ = 0;
  posedge_value_ = start_position;
  negedge_value_ = start_position;
}

void PositionSensorSimulator::InitializeRelativeEncoder() {
  current_position_ = 0.0;
}

void PositionSensorSimulator::MoveTo(double new_position) {
  {
    const int lower_start_segment = lower_index_edge_.current_index_segment();
    lower_index_edge_.MoveTo(new_position);
    const int lower_end_segment = lower_index_edge_.current_index_segment();
    if (lower_end_segment > lower_start_segment) {
      // Moving up past the lower edge.
      ++posedge_count_;
      posedge_value_ = lower_index_edge_.IndexPulsePosition();
    }
    if (lower_end_segment < lower_start_segment) {
      // Moved down.
      ++negedge_count_;
      negedge_value_ = lower_index_edge_.IndexPulsePosition();
    }
  }

  {
    const int upper_start_segment = upper_index_edge_.current_index_segment();
    upper_index_edge_.MoveTo(new_position);
    const int upper_end_segment = upper_index_edge_.current_index_segment();
    if (upper_end_segment > upper_start_segment) {
      // Moving up past the upper edge.
      ++negedge_count_;
      negedge_value_ = upper_index_edge_.IndexPulsePosition();
    }
    if (upper_end_segment < upper_start_segment) {
      // Moved down.
      ++posedge_count_;
      posedge_value_ = upper_index_edge_.IndexPulsePosition();
    }
  }

  current_position_ = new_position;
}

template <>
flatbuffers::Offset<IndexPosition>
PositionSensorSimulator::GetSensorValues<IndexPositionBuilder>(
    IndexPositionBuilder *builder) {
  builder->add_encoder(current_position_ - start_position_);

  const int index_count = lower_index_edge_.index_count();
  builder->add_index_pulses(index_count);
  if (index_count == 0) {
    builder->add_latched_encoder(0.0);
  } else {
    // Populate the latched encoder samples.
    builder->add_latched_encoder(lower_index_edge_.IndexPulsePosition() -
                                 start_position_);
  }
  return builder->Finish();
}

template <>
flatbuffers::Offset<PotAndIndexPosition>
PositionSensorSimulator::GetSensorValues<PotAndIndexPositionBuilder>(
    PotAndIndexPositionBuilder *builder) {
  builder->add_pot(lower_index_edge_.mutable_pot_noise()->AddNoiseToSample(
      current_position_));
  builder->add_encoder(current_position_ - start_position_);

  if (lower_index_edge_.index_count() == 0) {
    builder->add_latched_pot(0.0);
    builder->add_latched_encoder(0.0);
  } else {
    // Populate the latched pot/encoder samples.
    builder->add_latched_pot(lower_index_edge_.latched_pot());
    builder->add_latched_encoder(lower_index_edge_.IndexPulsePosition() -
                                 start_position_);
  }

  builder->add_index_pulses(lower_index_edge_.index_count());
  return builder->Finish();
}

template <>
flatbuffers::Offset<PotAndAbsolutePosition>
PositionSensorSimulator::GetSensorValues<PotAndAbsolutePositionBuilder>(
    PotAndAbsolutePositionBuilder *builder) {
  builder->add_pot(lower_index_edge_.mutable_pot_noise()->AddNoiseToSample(
      current_position_));
  builder->add_encoder(current_position_ - start_position_);
  builder->add_absolute_encoder(WrapAbsoluteEncoder());
  return builder->Finish();
}

template <>
flatbuffers::Offset<AbsolutePosition>
PositionSensorSimulator::GetSensorValues<AbsolutePositionBuilder>(
    AbsolutePositionBuilder *builder) {
  builder->add_encoder(current_position_ - start_position_);
  builder->add_absolute_encoder(WrapAbsoluteEncoder());
  return builder->Finish();
}

template <>
flatbuffers::Offset<AbsoluteAndAbsolutePosition>
PositionSensorSimulator::GetSensorValues<AbsoluteAndAbsolutePositionBuilder>(
    AbsoluteAndAbsolutePositionBuilder *builder) {
  builder->add_encoder(current_position_ - start_position_);
  builder->add_absolute_encoder(WrapAbsoluteEncoder());

  double single_turn_absolute_encoder =
      ::std::remainder(current_position_ + single_turn_known_absolute_encoder_,
                       single_turn_distance_per_revolution_);
  if (single_turn_absolute_encoder < 0) {
    single_turn_absolute_encoder += single_turn_distance_per_revolution_;
  }
  builder->add_single_turn_absolute_encoder(single_turn_absolute_encoder);
  return builder->Finish();
}

template <>
flatbuffers::Offset<HallEffectAndPosition>
PositionSensorSimulator::GetSensorValues<HallEffectAndPositionBuilder>(
    HallEffectAndPositionBuilder *builder) {
  builder->add_current(lower_index_edge_.current_index_segment() !=
                       upper_index_edge_.current_index_segment());
  builder->add_encoder(current_position_ - start_position_);

  builder->add_posedge_count(posedge_count_);
  builder->add_negedge_count(negedge_count_);
  builder->add_posedge_value(posedge_value_ - start_position_);
  builder->add_negedge_value(negedge_value_ - start_position_);
  return builder->Finish();
}

template <>
flatbuffers::Offset<RelativePosition>
PositionSensorSimulator::GetSensorValues<RelativePositionBuilder>(
    RelativePositionBuilder *builder) {
  builder->add_encoder(current_position_);
  return builder->Finish();
}

}  // namespace frc::control_loops
