#ifndef FRC_WPILIB_JOYSTICK_SENDER_HAL_H_
#define FRC_WPILIB_JOYSTICK_SENDER_HAL_H_

#include <wpi/Synchronization.h>

#include "aos/events/shm_event_loop.h"
#include "frc/input/joystick_state_static.h"

namespace frc::wpilib {

// JoystickSender, but reading the driver station straight from the HAL
// rather than through ahal's frc::DriverStation.  Use this in a process which
// links wpilibc, such as one running vendor libraries like Phoenix 6 which
// need it.  ahal and wpilibc can't share a process: both define
// frc::DriverStation, and members like GetStickAxis(int, int) mangle to the
// same symbols.
//
// This also skips wpilibc's frc::DriverStation, which allocates (it publishes
// FMSInfo to NetworkTables and returns strings), so it can run realtime.  The
// HAL calls made for each packet copy fixed-size structs out under a lock.
// HAL_ObserveUserProgramStarting allocates inside NI's netcomm library, so it
// and the rest of the one-time setup happen in the constructor, before the
// event loop goes realtime.
//
// Like JoystickSender, this blocks on the HAL's new-data event, so once the
// event loop is running it takes over its thread.  Give it an event loop of
// its own.
class JoystickSenderHal {
 public:
  JoystickSenderHal(::aos::ShmEventLoop *event_loop);
  ~JoystickSenderHal();

 private:
  // Refreshes the HAL's copy of the DS data and sends it.
  void SendJoystickState();

  aos::ShmEventLoop *event_loop_;
  aos::Sender<frc::JoystickStateStatic> joystick_state_sender_;
  const uint16_t team_id_;

  // Signaled by the HAL each time a packet arrives from the DS.
  wpi::Event new_data_event_{false, false};
};

}  // namespace frc::wpilib

#endif  // FRC_WPILIB_JOYSTICK_SENDER_HAL_H_
