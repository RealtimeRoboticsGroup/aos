#ifndef AOS_UTIL_PRINT_TABLE_H_
#define AOS_UTIL_PRINT_TABLE_H_
#include <array>
#include <iomanip>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace aos::util {
// Generates a table of the specified strings, such that the columns are all
// spaced equally.
// Will use prefix for indentation.
template <size_t kColumns>
void PrintTable(std::ostream *os, std::string_view prefix,
                const std::vector<std::array<std::string, kColumns>> &table) {
  std::array<size_t, kColumns> widths;
  widths.fill(0);
  for (const auto &row : table) {
    for (size_t ii = 0; ii < kColumns; ++ii) {
      widths.at(ii) = std::max(widths.at(ii), row.at(ii).size());
    }
  }
  for (const auto &row : table) {
    *os << prefix << std::setfill(' ');
    for (size_t ii = 0; ii < widths.size(); ++ii) {
      *os << std::setw(widths.at(ii)) << row.at(ii);
      if (ii + 1 != widths.size()) {
        *os << " | ";
      }
    }
    *os << std::endl;
  }
}
}  // namespace aos::util
#endif  // AOS_UTIL_PRINT_TABLE_H_
