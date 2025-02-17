#include "y2016/constants.h"

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <map>

#if __has_feature(address_sanitizer)
#include "sanitizer/lsan_interface.h"
#endif

#include "absl/base/call_once.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/network/team_number.h"
#include "aos/stl_mutex/stl_mutex.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace y2016::constants {

// ///// Mutual constants between robots. /////
const int Values::kZeroingSampleSize;

constexpr double Values::kDrivetrainEncoderRatio, Values::kShooterEncoderRatio,
    Values::kIntakeEncoderRatio, Values::kShoulderEncoderRatio,
    Values::kWristEncoderRatio, Values::kIntakePotRatio,
    Values::kShoulderPotRatio, Values::kWristPotRatio,
    Values::kIntakeEncoderIndexDifference,
    Values::kShoulderEncoderIndexDifference,
    Values::kWristEncoderIndexDifference;
constexpr ::frc::constants::Range Values::kIntakeRange, Values::kShoulderRange,
    Values::kWristRange;

namespace {
const uint16_t kCompTeamNumber = 971;
const uint16_t kPracticeTeamNumber = 9971;

// ///// Dynamic constants. /////

const Values *DoGetValuesForTeam(uint16_t team) {
  switch (team) {
    case 1:  // for tests
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
              0.0,
              {{},
               Values::kZeroingSampleSize,
               Values::kIntakeEncoderIndexDifference,
               0.0,
               0.3},
          },

          // Shoulder
          {
              0.0,
              {{},
               Values::kZeroingSampleSize,
               Values::kShoulderEncoderIndexDifference,
               0.0,
               0.3},
          },

          // Wrist
          {
              0.0,
              {{},
               Values::kZeroingSampleSize,
               Values::kWristEncoderIndexDifference,
               0.0,
               0.3},
          },

          0.0,
          "practice",
      };
      break;

    case kCompTeamNumber:
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
              // Value to add to the pot reading for the intake.
              -4.550531 + 150.40906362 * M_PI / 180.0 + 0.5098 - 0.0178 -
                  0.0725,
              {{},
               Values::kZeroingSampleSize,
               Values::kIntakeEncoderIndexDifference,
               // Location of an index pulse.
               0.018008,
               2.5},
          },

          // Shoulder
          {
              // Value to add to the pot reading for the shoulder.
              -2.86275657117,
              {{},
               Values::kZeroingSampleSize,
               Values::kShoulderEncoderIndexDifference,
               0.097312,
               2.5},
          },

          // Wrist
          {
              // Value to add to the pot reading for the wrist.
              3.2390714288298668 + -0.06138835 * M_PI / 180.0 + 0.0078 -
                  0.0548 - 0.0167 + 0.002 - 0.0026 - 0.1040 - 0.0035 - 0.0012 +
                  0.0166 - 0.017 + 0.148 + 0.004 + 0.024701 - 0.0741,
              {{},
               Values::kZeroingSampleSize,
               Values::kWristEncoderIndexDifference,
               0.000820,
               2.5},
          },

          0.0,
          "competition",
      };
      break;
    case kPracticeTeamNumber:
      return new Values{
          5.0,  // drivetrain max speed

          // Intake
          {
              // Hard stop is 160.0185751389329 degrees.
              -4.2193 + (160.0185751389329 * M_PI / 180.0 + 0.02 - 0.0235) +
                  0.0549 - 0.104 + 0.019 - 0.938 + 0.660 - 0.002 - 0.2081,
              {{},
               Values::kZeroingSampleSize,
               Values::kIntakeEncoderIndexDifference,
               0.332370,
               1.3},
          },

          // Shoulder (Now calibrated at 0)
          {
              -1.0016 - 0.0841 + 0.06138835 * M_PI / 180.0 + 1.07838 - 1.0441 +
                  0.0034 + 0.0065 - 0.0505,
              {{},
               Values::kZeroingSampleSize,
               Values::kShoulderEncoderIndexDifference,
               0.027180,
               1.3},
          },

          // Wrist
          {
              3.326328571170133 - 0.06138835 * M_PI / 180.0 - 0.177 + 0.0323 -
                  0.023 + 0.0488 + 0.0120 - 0.0005 - 0.0784 - 0.0010 - 0.080 +
                  0.1245,
              {{},
               Values::kZeroingSampleSize,
               Values::kWristEncoderIndexDifference,
               -0.263227,
               1.3},
          },

          0.011,
          "practice",
      };
      break;
    default:
      LOG(FATAL) << "unknown team: " << team;
  }
}

void DoGetValues(const Values **result) {
  uint16_t team = ::aos::network::GetTeamNumber();
  LOG(INFO) << "creating a Constants for team: " << team;
  *result = DoGetValuesForTeam(team);
  return;
}

}  // namespace

const Values &GetValues() {
  static absl::once_flag once;
  static const Values *result;
  absl::call_once(once, DoGetValues, &result);
  return *result;
}

const Values &GetValuesForTeam(uint16_t team_number) {
  static aos::stl_mutex mutex;
  std::unique_lock<aos::stl_mutex> locker(mutex);

  // IMPORTANT: This declaration has to stay after the mutex is locked to avoid
  // race conditions.
  static ::std::map<uint16_t, const Values *> values;

  if (values.count(team_number) == 0) {
    values[team_number] = DoGetValuesForTeam(team_number);
#if __has_feature(address_sanitizer)
    __lsan_ignore_object(values[team_number]);
#endif
  }
  return *values[team_number];
}

}  // namespace y2016::constants
