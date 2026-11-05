#ifndef FRC_WPILIB_CAN_STATUS_H_
#define FRC_WPILIB_CAN_STATUS_H_

#include "ctre/phoenix/StatusCodes.h"

namespace frc::wpilib {

// Checks the result of setting up a CAN device, like changing a signal's rate
// or writing its configs.
//
// We need to keep running with a disconnected device, so this logs and returns
// false for the statuses which mean the device can't be reached.  Anything else
// means the code, the firmware, or the configuration is wrong, so we die to get
// that fixed rather than running with a device which is set up wrong.
//
// device_type and action are used for the messages, for example "failed to
// <action> on <device_type> <device_id>".  Returns true if status is OK.
bool CheckCANSetupStatus(ctre::phoenix::StatusCode status,
                         const char *device_type, int device_id,
                         const char *action);

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_CAN_STATUS_H_
