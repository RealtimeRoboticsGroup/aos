#ifndef FRC_IMU_READER_IMU_H_
#define FRC_IMU_READER_IMU_H_

#include "aos/events/shm_event_loop.h"
#include "frc/wpilib/imu_batch_generated.h"

namespace frc::imu {

// Reads IMU packets from the kernel driver which reads them over spi
// from the Raspberry Pi Pico on the IMU board.
class Imu {
 public:
  // Constructs an IMU reader object.  encoder_scalar is in meters/count.
  Imu(aos::ShmEventLoop *event_loop, double encoder_scalar);
  ~Imu();

 private:
  flatbuffers::Offset<frc::ADIS16470DiagStat> PackDiagStat(
      flatbuffers::FlatBufferBuilder *fbb, uint16_t value);
  flatbuffers::Offset<frc::IMUValues> ProcessReading(
      flatbuffers::FlatBufferBuilder *fbb, absl::Span<uint8_t> buf);
  double ConvertValue32(absl::Span<const uint8_t> data, double lsb_per_output);
  double ConvertValue16(absl::Span<const uint8_t> data, double lsb_per_output);

  aos::ShmEventLoop *event_loop_;
  aos::Sender<frc::IMUValuesBatch> imu_sender_;
  int imu_fd_;

  uint failed_checksums_ = 0;

  double encoder_scalar_;
};

}  // namespace frc::imu

#endif  // FRC_IMU_READER_IMU_H_
