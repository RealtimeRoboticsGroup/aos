#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>

#include "absl/flags/flag.h"

#include "aos/actions/actions.h"
#include "aos/events/event_loop.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "aos/util/log_interval.h"
#include "frc/autonomous/base_autonomous_actor.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/drivetrain/localizer_generated.h"
#include "frc/control_loops/profiled_subsystem_generated.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem.h"
#include "frc/input/action_joystick_input.h"
#include "frc/input/driver_station_data.h"
#include "frc/input/drivetrain_input.h"
#include "frc/input/joystick_input.h"
#include "frc/input/redundant_joystick_data.h"
#include "frc/input/swerve_joystick_input.h"
#include "frc/zeroing/wrap.h"
#include "y2024_bot3/constants/constants_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_goal_generated.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_goal_static.h"
#include "y2024_bot3/control_loops/superstructure/superstructure_status_static.h"

using frc::CreateProfileParameters;
using frc::input::driver_station::ButtonLocation;
using frc::input::driver_station::ControlBit;
using frc::input::driver_station::JoystickAxis;
using frc::input::driver_station::POVLocation;
using Side = frc::control_loops::drivetrain::RobotSide;
using y2024_bot3::control_loops::superstructure::GoalStatic;
using y2024_bot3::control_loops::superstructure::Status;

namespace y2024_bot3::input::joysticks {

namespace swerve = frc::control_loops::swerve;

namespace superstructure = y2024_bot3::control_loops::superstructure;

const ButtonLocation kIntakePosition(2, 2);
const ButtonLocation kAmpPosition(2, 4);

const ButtonLocation kSpit(1, 13);
const ButtonLocation kSuck(2, 5);
const ButtonLocation kAmp(2, 8);

class Reader : public ::frc::input::SwerveJoystickInput {
 public:
  Reader(::aos::EventLoop *event_loop,
         const y2024_bot3::Constants *robot_constants)
      : ::frc::input::SwerveJoystickInput(event_loop,
                                             {.use_redundant_joysticks = true}),
        superstructure_goal_sender_(
            event_loop->MakeSender<GoalStatic>("/superstructure")),
        superstructure_status_fetcher_(
            event_loop->MakeFetcher<Status>("/superstructure")),
        robot_constants_(robot_constants) {
    CHECK(robot_constants_ != nullptr);
  }

  void AutoEnded() /*override*/ { AOS_LOG(INFO, "Auto ended.\n"); }

  void HandleTeleop(
      const ::frc::input::driver_station::Data &data) override {
    superstructure_status_fetcher_.Fetch();
    if (!superstructure_status_fetcher_.get()) {
      AOS_LOG(ERROR, "Got no superstructure status message.\n");
      return;
    }

    aos::Sender<superstructure::GoalStatic>::StaticBuilder
        superstructure_goal_builder =
            superstructure_goal_sender_.MakeStaticBuilder();

    if (data.IsPressed(kIntakePosition)) {
      superstructure_goal_builder->set_arm_position(
          superstructure::PivotGoal::INTAKE);
    } else if (data.IsPressed(kAmpPosition)) {
      superstructure_goal_builder->set_arm_position(
          superstructure::PivotGoal::AMP);
    } else {
      superstructure_goal_builder->set_arm_position(
          superstructure::PivotGoal::IDLE);
    }

    if (data.IsPressed(kSuck)) {
      superstructure_goal_builder->set_roller_goal(
          superstructure::IntakeGoal::INTAKE);
    } else if (data.IsPressed(kSpit)) {
      superstructure_goal_builder->set_roller_goal(
          superstructure::IntakeGoal::SPIT);
    } else if (data.IsPressed(kAmp)) {
      superstructure_goal_builder->set_roller_goal(
          superstructure::IntakeGoal::SCORE);
    } else {
      superstructure_goal_builder->set_roller_goal(
          superstructure::IntakeGoal::NONE);
    }

    superstructure_goal_builder.CheckOk(superstructure_goal_builder.Send());
  }

 private:
  ::aos::Sender<GoalStatic> superstructure_goal_sender_;
  ::aos::Fetcher<Status> superstructure_status_fetcher_;
  const y2024_bot3::Constants *robot_constants_;
};

}  // namespace y2024_bot3::input::joysticks

int main(int argc, char **argv) {
  ::aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");
  frc::constants::WaitForConstants<y2024_bot3::Constants>(&config.message());

  ::aos::ShmEventLoop constant_fetcher_event_loop(&config.message());
  frc::constants::ConstantsFetcher<y2024_bot3::Constants> constants_fetcher(
      &constant_fetcher_event_loop);
  const y2024_bot3::Constants *robot_constants = &constants_fetcher.constants();

  ::aos::ShmEventLoop event_loop(&config.message());
  ::y2024_bot3::input::joysticks::Reader reader(&event_loop, robot_constants);

  event_loop.Run();

  return 0;
}
