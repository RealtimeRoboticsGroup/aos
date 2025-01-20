#ifndef FRC_CAN_LOGGER_CAN_LOGGER_H_
#define FRC_CAN_LOGGER_CAN_LOGGER_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>

#include <chrono>

#include "aos/events/shm_event_loop.h"
#include "aos/realtime.h"
#include "aos/scoped/scoped_fd.h"
#include "frc/can_logger/can_logging_generated.h"

namespace frc::can_logger {

// This class listens to all the traffic on a SocketCAN interface and sends it
// on the aos event loop so it can be logged with the aos logging
// infrastructure.
class CanLogger {
 public:
  static constexpr std::chrono::milliseconds kPollPeriod =
      std::chrono::milliseconds(100);

  CanLogger(aos::ShmEventLoop *event_loop,
            std::string_view channel_name = "/can",
            std::string_view interface_name = "can0");

  CanLogger(const CanLogger &) = delete;
  CanLogger &operator=(const CanLogger &) = delete;

  ~CanLogger() { shm_event_loop_->epoll()->DeleteFd(fd_.get()); }

 private:
  void Poll();

  // Read a CAN frame from the socket and send it on the event loop
  // Returns true if successful and false if the recieve buffer is empty.
  bool ReadFrame();

  aos::ShmEventLoop *shm_event_loop_;
  aos::ScopedFD fd_;
  aos::Sender<CanFrame> frames_sender_;
};

}  // namespace frc::can_logger

#endif  // FRC_CAN_LOGGER_CAN_LOGGER_H_
