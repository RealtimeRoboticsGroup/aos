#ifndef Y2024_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2024_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/control_loop.h"
#include "frc/zeroing/absolute_encoder.h"
#include "frc/zeroing/pot_and_absolute_encoder.h"
#include "y2024_bot3/constants.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_output_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_position_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_status_generated.h"

namespace y2024_bot3::control_loops::superstructure {

class Superstructure
    : public ::frc::controls::ControlLoop<Goal, Position, Status, Output> {
 public:
  using AbsoluteEncoderSubsystem =
      ::frc::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc::zeroing::AbsoluteEncoderZeroingEstimator,
          ::frc::control_loops::AbsoluteEncoderProfiledJointStatus>;

  using PotAndAbsoluteEncoderSubsystem =
      ::frc::control_loops::StaticZeroingSingleDOFProfiledSubsystem<
          ::frc::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
          ::frc::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

  explicit Superstructure(::aos::EventLoop *event_loop,
                          const ::std::string &name = "/superstructure");

  double robot_velocity() const;

  inline const PotAndAbsoluteEncoderSubsystem &arm() const { return arm_; }

 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  frc::constants::ConstantsFetcher<Constants> constants_fetcher_;
  const Constants *robot_constants_;
  aos::Fetcher<frc::JoystickState> joystick_state_fetcher_;

  frc::Alliance alliance_ = frc::Alliance::kInvalid;

  PotAndAbsoluteEncoderSubsystem arm_;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace y2024_bot3::control_loops::superstructure

#endif  // Y2024_BOT3_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
