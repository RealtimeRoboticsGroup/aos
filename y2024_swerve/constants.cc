#include "y2024_swerve/constants.h"

#include <cstdint>

#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/network/team_number.h"

namespace y2024_swerve::constants {
Values MakeValues(uint16_t team) {
  LOG(INFO) << "creating a Constants for team: " << team;
  Values r;

  return r;
}

Values MakeValues() { return MakeValues(aos::network::GetTeamNumber()); }
}  // namespace y2024_swerve::constants
