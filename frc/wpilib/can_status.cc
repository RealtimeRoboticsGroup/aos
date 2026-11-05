#include "frc/wpilib/can_status.h"

#include "absl/log/absl_log.h"

#include "aos/logging/logging.h"

namespace frc::wpilib {
namespace {

// CTRE doesn't document which of these a missing device returns, so this list
// comes from the descriptions of each status.
// TODO: Confirm these by unplugging each kind of device from each bus.
bool IsDisconnected(ctre::phoenix::StatusCode status) {
  using ctre::phoenix::StatusCode;
  return status == StatusCode::EcuIsNotPresent ||
         status == StatusCode::RxTimeout || status == StatusCode::TxTimeout ||
         status == StatusCode::TxFailed ||
         status == StatusCode::CouldNotSendCanFrame ||
         status == StatusCode::CanMessageStale;
}

}  // namespace

bool CheckCANSetupStatus(ctre::phoenix::StatusCode status,
                         const char *device_type, int device_id,
                         const char *action) {
  if (status.IsOK()) {
    return true;
  }
  if (!IsDisconnected(status)) {
    ABSL_LOG(FATAL) << "Failed to " << action << " on " << device_type << " "
                    << device_id << ": " << status.GetName() << ": "
                    << status.GetDescription();
  }
  AOS_LOG(ERROR, "Failed to %s on %s %d, is it disconnected? %s: %s", action,
          device_type, device_id, status.GetName(), status.GetDescription());
  return false;
}

}  // namespace frc::wpilib
