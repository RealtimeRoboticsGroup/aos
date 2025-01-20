#include "aos/network/team_number.h"

#include "gtest/gtest.h"

namespace aos::network::testing {

using team_number_internal::ParsePiOrOrinTeamNumber;
using team_number_internal::ParseRoborioTeamNumber;

TEST(TeamNumberTest, Parse2015TeamNumber) {
  EXPECT_EQ(1323u, *ParseRoborioTeamNumber("roboRIO-1323"));

  EXPECT_EQ(1671u, ParseRoborioTeamNumber("roboRIO-1671"));

  EXPECT_FALSE(ParseRoborioTeamNumber("abc"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-8abc"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-"));
}

TEST(TeamNumberTest, Parse2016TeamNumber) {
  EXPECT_EQ(1323u, *ParseRoborioTeamNumber("roboRIO-1323-FRC"));

  EXPECT_EQ(1671u, *ParseRoborioTeamNumber("roboRIO-1671-FRC"));

  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-8abc-FRC"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-1671-FRC2"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO-1671-2FRC"));
  EXPECT_FALSE(ParseRoborioTeamNumber("roboRIO--FRC"));
}

TEST(HostnameParseTest, ParsePiOrOrinTeamNumber) {
  EXPECT_EQ(1323u, *ParsePiOrOrinTeamNumber("pi-1323-1"));
  EXPECT_EQ(1671u, *ParsePiOrOrinTeamNumber("pi-1671-22"));
  EXPECT_EQ(1671u, *ParsePiOrOrinTeamNumber("pi-1671-"));

  EXPECT_EQ(1323u, *ParsePiOrOrinTeamNumber("orin-1323-1"));
  EXPECT_EQ(1671u, *ParsePiOrOrinTeamNumber("orin-1671-22"));
  EXPECT_EQ(1671u, *ParsePiOrOrinTeamNumber("orin-1671-"));

  EXPECT_FALSE(ParsePiOrOrinTeamNumber("roboRIO-1323-FRC"));

  EXPECT_FALSE(ParseRoborioTeamNumber("pi"));
  EXPECT_FALSE(ParseRoborioTeamNumber("pi-"));
  EXPECT_FALSE(ParseRoborioTeamNumber("pi-1323"));
  EXPECT_FALSE(ParseRoborioTeamNumber("pi-118a-1"));
  EXPECT_FALSE(ParseRoborioTeamNumber("orin-118-1"));
}

TEST(HostnameParseTest, ParsePiOrOrinNumber) {
  EXPECT_EQ(1u, *ParsePiOrOrinNumber("pi-118-1"));
  EXPECT_EQ(22u, *ParsePiOrOrinNumber("pi-1671-22"));
  EXPECT_EQ(1u, *ParsePiOrOrinNumber("orin-118-1"));
  EXPECT_EQ(22u, *ParsePiOrOrinNumber("orin-1671-22"));

  EXPECT_FALSE(ParsePiOrOrinNumber("pi-1671-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("pi"));
  EXPECT_FALSE(ParsePiOrOrinNumber("pi-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("pi-118"));

  EXPECT_FALSE(ParsePiOrOrinNumber("orin-1671-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("orin"));
  EXPECT_FALSE(ParsePiOrOrinNumber("orin-"));
  EXPECT_FALSE(ParsePiOrOrinNumber("orin-118"));
}

TEST(HostnameParseTest, ParsePiOrOrin) {
  EXPECT_EQ("pi", *ParsePiOrOrin("pi-118-1"));
  EXPECT_EQ("pi", *ParsePiOrOrin("pi-1671-22"));
  EXPECT_EQ("pi", *ParsePiOrOrin("pi-1671-"));

  EXPECT_EQ("orin", *ParsePiOrOrin("orin-118-1"));
  EXPECT_EQ("orin", *ParsePiOrOrin("orin-1671-22"));
  EXPECT_EQ("orin", *ParsePiOrOrin("orin-1671-"));

  EXPECT_EQ("orin", *ParsePiOrOrin("imu-118-1"));

  EXPECT_FALSE(ParsePiOrOrin("roboRIO-118-FRC"));
  EXPECT_FALSE(ParsePiOrOrin("laptop"));
}

}  // namespace aos::network::testing
