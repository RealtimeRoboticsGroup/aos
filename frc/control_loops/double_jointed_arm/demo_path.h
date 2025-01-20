#ifndef FRC_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DEMO_PATH_H_
#define FRC_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DEMO_PATH_H_

#include <memory>

#include "frc/control_loops/double_jointed_arm/trajectory.h"

namespace frc::control_loops::arm {

::std::unique_ptr<Path> MakeDemoPath();
::std::unique_ptr<Path> MakeReversedDemoPath();

}  // namespace frc::control_loops::arm

#endif  // FRC_CONTROL_LOOPS_DOUBLE_JOINTED_ARM_DEMO_PATH_H_
