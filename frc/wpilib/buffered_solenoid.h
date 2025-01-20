#ifndef FRC_WPILIB_BUFFERED_SOLENOID_H_
#define FRC_WPILIB_BUFFERED_SOLENOID_H_

namespace frc::wpilib {

class BufferedPcm;

// Handles sending values for a single solenoid to a BufferedPcm. Instances are
// created with BufferedPcm::MakeSolenoid.
class BufferedSolenoid {
 public:
  // Sets or unsets the solenoid.
  void Set(bool value);

 private:
  BufferedSolenoid(int number, BufferedPcm *pcm) : number_(number), pcm_(pcm) {}

  const int number_;
  BufferedPcm *const pcm_;

  friend class BufferedPcm;
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_BUFFERED_SOLENOID_H_
