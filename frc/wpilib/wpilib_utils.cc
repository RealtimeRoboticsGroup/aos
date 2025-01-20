#include "frc/wpilib/wpilib_utils.h"

namespace frc::wpilib {

bool SafePotVoltageRange(::frc::constants::Range subsystem_range,
                         double potentiometer_offset,
                         ::std::function<double(double)> pot_translate_inverse,
                         bool reverse, double limit_buffer) {
  constexpr double kMinVoltage = 0.0;
  constexpr double kMaxVoltage = 5.0;
  double min_range_voltage =
      pot_translate_inverse(subsystem_range.lower_hard - potentiometer_offset);
  double max_range_voltage =
      pot_translate_inverse(subsystem_range.upper_hard - potentiometer_offset);
  if (reverse) {
    min_range_voltage *= -1;
    max_range_voltage *= -1;
  }
  return ((kMinVoltage + limit_buffer) < min_range_voltage &&
          min_range_voltage < (kMaxVoltage - limit_buffer) &&
          (kMinVoltage + limit_buffer) < max_range_voltage &&
          max_range_voltage < (kMaxVoltage - limit_buffer));
}
}  // namespace frc::wpilib
