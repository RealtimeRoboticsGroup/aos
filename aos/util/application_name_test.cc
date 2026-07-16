#include "aos/util/application_name.h"

#include <string>

#include "gtest/gtest.h"

namespace aos::testing {

TEST(ApplicationNameTest, GetProgramName) {
  std::string prog_name = GetProgramName();
  EXPECT_FALSE(prog_name.empty());
  EXPECT_NE(prog_name.find("application_name_test"), std::string::npos);
}

}  // namespace aos::testing
