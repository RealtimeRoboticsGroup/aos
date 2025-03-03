#ifndef FRC_WPILIB_WPILIB_INTERFACE_H_
#define FRC_WPILIB_WPILIB_INTERFACE_H_

#include <cstdint>

#include "aos/events/event_loop.h"
#include "frc/input/robot_state_generated.h"

namespace frc::wpilib {

// Sends out a message on ::aos::robot_state.
flatbuffers::Offset<frc::RobotState> PopulateRobotState(
    aos::Sender<frc::RobotState>::Builder *builder, int32_t my_pid);

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_WPILIB_INTERFACE_H_
