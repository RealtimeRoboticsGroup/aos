#ifndef FRC_ORIN_APRILTAG_TYPES_H_
#define FRC_ORIN_APRILTAG_TYPES_H_

namespace frc::apriltag {

// A size type for use in apriltag GPU code.
// This is 32 bits to consume less memory and be faster.  And, a 4 GB image will
// break so much other stuff that it isn't worth stressing.
typedef uint32_t apriltag_size_t;

}

#endif  // FRC_ORIN_APRILTAG_TYPES_H_
