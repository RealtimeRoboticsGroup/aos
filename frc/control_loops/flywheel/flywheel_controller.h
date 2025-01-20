#ifndef FRC_CONTROL_LOOPS_SHOOTER_FLYWHEEL_CONTROLLER_H_
#define FRC_CONTROL_LOOPS_SHOOTER_FLYWHEEL_CONTROLLER_H_

#include <memory>

#include "aos/time/time.h"
#include "frc/control_loops/control_loop.h"
#include "frc/control_loops/flywheel/flywheel_controller_status_generated.h"
#include "frc/control_loops/hybrid_state_feedback_loop.h"
#include "frc/control_loops/state_feedback_loop.h"

namespace frc::control_loops::flywheel {

class CurrentLimitedStateFeedbackController;

// Handles the velocity control of each flywheel.
class FlywheelController {
 public:
  FlywheelController(
      StateFeedbackLoop<3, 1, 1, double, StateFeedbackHybridPlant<3, 1, 1>,
                        HybridKalman<3, 1, 1>> &&loop,
      double bemf, double resistance);

  ~FlywheelController();

  // Sets the velocity goal in radians/sec
  void set_goal(double angular_velocity_goal);
  double goal() const { return last_goal_; }
  // Sets the current encoder position in radians
  void set_position(double current_position,
                    const aos::monotonic_clock::time_point position_timestamp);

  // Populates the status structure.
  flatbuffers::Offset<FlywheelControllerStatus> SetStatus(
      flatbuffers::FlatBufferBuilder *fbb);

  // Returns the control loop calculated voltage.
  double voltage() const;

  // Returns the expected battery current for the last U.
  double current() const;

  // Returns the instantaneous velocity.
  double velocity() const;

  // Executes the control loop for a cycle.
  void Update(bool disabled);

  double avg_angular_velocity() { return avg_angular_velocity_; }

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // The control loop.
  ::std::unique_ptr<CurrentLimitedStateFeedbackController> loop_;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 10;
  ::std::array<std::pair<double, ::aos::monotonic_clock::time_point>,
               kHistoryLength>
      history_;
  ptrdiff_t history_position_ = 0;

  // Average velocity logging.
  double avg_angular_velocity_ = 0;

  double last_goal_ = 0;

  bool first_ = true;

  DISALLOW_COPY_AND_ASSIGN(FlywheelController);
};

}  // namespace frc::control_loops::flywheel

#endif  // FRC_CONTROL_LOOPS_SHOOTER_FLYWHEEL_CONTROLLER_H_
