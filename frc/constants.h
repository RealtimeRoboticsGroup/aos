#ifndef FRC_CONSTANTS_H_
#define FRC_CONSTANTS_H_

#include <cstddef>

#include "frc/control_loops/control_loops_generated.h"
#include "frc/zeroing/constants_generated.h"

namespace frc::constants {

typedef frc::zeroing::HallEffectZeroingConstantsT HallEffectZeroingConstants;

typedef frc::zeroing::PotAndIndexPulseZeroingConstantsT
    PotAndIndexPulseZeroingConstants;

typedef frc::zeroing::EncoderPlusIndexZeroingConstantsT
    EncoderPlusIndexZeroingConstants;

typedef frc::zeroing::PotAndAbsoluteEncoderZeroingConstantsT
    PotAndAbsoluteEncoderZeroingConstants;

typedef frc::zeroing::RelativeEncoderZeroingConstantsT
    RelativeEncoderZeroingConstants;

typedef frc::zeroing::ContinuousAbsoluteEncoderZeroingConstantsT
    ContinuousAbsoluteEncoderZeroingConstants;

typedef frc::zeroing::AbsoluteEncoderZeroingConstantsT
    AbsoluteEncoderZeroingConstants;

typedef frc::zeroing::AbsoluteAndAbsoluteEncoderZeroingConstantsT
    AbsoluteAndAbsoluteEncoderZeroingConstants;

// Defines a range of motion for a subsystem.
// These are all absolute positions in scaled units.
struct Range {
  double lower_hard;
  double upper_hard;
  double lower;
  double upper;

  constexpr double middle() const { return (lower_hard + upper_hard) / 2.0; }
  constexpr double middle_soft() const { return (lower + upper) / 2.0; }

  constexpr double range() const { return upper_hard - lower_hard; }

  static Range FromFlatbuffer(const frc::Range *range) {
    return {.lower_hard = range->lower_hard(),
            .upper_hard = range->upper_hard(),
            .lower = range->lower(),
            .upper = range->upper()};
  }
};

}  // namespace frc::constants

#endif  // FRC_CONSTANTS_H_
