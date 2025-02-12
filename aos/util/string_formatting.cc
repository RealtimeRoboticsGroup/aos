#include "aos/util/string_formatting.h"

#include <iomanip>
#include <sstream>

namespace aos::util {

std::string FormatFloat(double value, int precision) {
  // Use an ostringstream to apply fixed-point notation and set the precision
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  const std::string formatted_str = oss.str();

  // Return if the value has no decimal point, or has no trailing zeros.
  if (formatted_str.find('.') == std::string::npos ||
      formatted_str.back() != '0') {
    return formatted_str;
  }
  // Find the position of the last character that is not '0'
  const std::size_t last_non_zero = formatted_str.find_last_not_of('0');
  std::size_t last_included_character = last_non_zero;

  // If the character before the sequence of zeros is the decimal point
  if (formatted_str[last_non_zero] == '.') {
    // Keep one zero after the decimal point for clarity
    ++last_included_character;
  }

  // The string length is 1 greater than index of the last included character.
  return formatted_str.substr(0, last_included_character + 1);
}

}  // namespace aos::util