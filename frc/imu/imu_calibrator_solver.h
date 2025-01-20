#ifndef FRC_IMU_IMU_CALIBRATOR_SOLVER_H_
#define FRC_IMU_IMU_CALIBRATOR_SOLVER_H_

#include "frc/imu/imu_calibrator.h"

namespace frc::imu {

// Stores all the IMU data from a log so that we can feed it into the
// ImuCalibrator readily.
AllParameters<double> Solve(
    const std::vector<std::vector<RawImuReading>> &readings,
    const std::vector<ImuConfig<double>> &nominal_config);
}  // namespace frc::imu
#endif  // FRC_IMU_IMU_CALIBRATOR_SOLVER_H_
