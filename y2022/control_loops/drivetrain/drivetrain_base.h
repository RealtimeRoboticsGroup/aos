#ifndef Y2022_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2022_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc/control_loops/drivetrain/drivetrain_config.h"

namespace y2022::control_loops::drivetrain {

const ::frc::control_loops::drivetrain::DrivetrainConfig<double> &
GetDrivetrainConfig();

}  // namespace y2022::control_loops::drivetrain

#endif  // Y2022_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
