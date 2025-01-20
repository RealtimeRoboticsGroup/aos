#include "frc/control_loops/team_number_test_environment.h"

#include "aos/network/team_number.h"

namespace frc::control_loops::testing {

void TeamNumberEnvironment::SetUp() {
  ::aos::network::OverrideTeamNumber(kTeamNumber);
}

}  // namespace frc::control_loops::testing
