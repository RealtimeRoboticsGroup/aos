#ifndef Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/control_loop.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc/control_loops/drivetrain/drivetrain_can_position_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_status_generated.h"
#include "y2024_swerve/constants.h"
#include "y2024_swerve/constants/constants_generated.h"
#include "y2024_swerve/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024_swerve/control_loops/superstructure/superstructure_output_generated.h"
#include "y2024_swerve/control_loops/superstructure/superstructure_position_generated.h"
#include "y2024_swerve/control_loops/superstructure/superstructure_status_generated.h"

namespace y2024_swerve::control_loops::superstructure {

class Superstructure
    : public ::frc::controls::ControlLoop<Goal, Position, Status, Output> {
 public:

  explicit Superstructure(::aos::EventLoop *event_loop,
                          std::shared_ptr<const constants::Values> values,
                          const ::std::string &name = "/superstructure");
 protected:
  virtual void RunIteration(const Goal *unsafe_goal, const Position *position,
                            aos::Sender<Output>::Builder *output,
                            aos::Sender<Status>::Builder *status) override;

 private:
  std::shared_ptr<const constants::Values> values_;

  DISALLOW_COPY_AND_ASSIGN(Superstructure);
};

}  // namespace y2023_bot3::control_loops::superstructure

#endif  // Y2023_CONTROL_LOOPS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_