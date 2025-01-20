#include "frc/control_loops/profiled_subsystem.h"

namespace frc::control_loops::internal {

double UseUnlessZero(double target_value, double default_value) {
  if (target_value != 0.0) {
    return target_value;
  } else {
    return default_value;
  }
}

}  // namespace frc::control_loops::internal
