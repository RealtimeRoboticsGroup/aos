#ifndef Y2019_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define Y2019_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "frc/control_loops/drivetrain/drivetrain_config.h"

namespace y2019::control_loops::drivetrain {

const ::frc::control_loops::drivetrain::DrivetrainConfig<double> &
GetDrivetrainConfig();

}  // namespace y2019::control_loops::drivetrain

#endif  // Y2019_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_BASE_H_
