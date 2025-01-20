#ifndef FRC_IMU_FDCAN_DUAL_IMU_BLENDER_H_
#define FRC_IMU_FDCAN_DUAL_IMU_BLENDER_H_

#include "aos/events/event_loop.h"
#include "frc/imu_fdcan/dual_imu_blender_status_static.h"
#include "frc/imu_fdcan/dual_imu_generated.h"
#include "frc/wpilib/imu_batch_static.h"

namespace frc::imu_fdcan {

// Takes in the values from the dual_imu and creates an IMUValuesBatch. Will use
// the murata until we've hit saturation according to the tdk, then we will
// switch to using tdk IMU values.
class DualImuBlender {
 public:
  DualImuBlender(aos::EventLoop *event_loop);

  void HandleDualImu(const frc::imu::DualImu *dual_imu);

 private:
  aos::Sender<IMUValuesBatchStatic> imu_values_batch_sender_;
  aos::Sender<imu::DualImuBlenderStatusStatic> dual_imu_blender_status_sender_;
  int saturated_counter_ = 0;
  bool is_saturated_ = false;
};

}  // namespace frc::imu_fdcan

#endif  // FRC_IMU_FDCAN_DUAL_IMU_BLENDER_H_
