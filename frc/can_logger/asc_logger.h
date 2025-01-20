#ifndef FRC_CAN_LOGGER_ASC_LOGGER_H_
#define FRC_CAN_LOGGER_ASC_LOGGER_H_

#include <iomanip>
#include <iostream>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/event_loop.h"
#include "frc/can_logger/can_logging_generated.h"

namespace frc::can_logger {

class AscLogger {
 public:
  AscLogger(aos::EventLoop *event_loop, const std::string &filename);

 private:
  void HandleFrame(const CanFrame &frame);

  // This implementation attempts to duplicate the output of can-utils/log2asc
  void WriteFrame(std::ostream &file, const CanFrame &frame);

  static void WriteHeader(std::ostream &file,
                          aos::realtime_clock::time_point start_time);

  std::optional<aos::realtime_clock::time_point> first_frame_realtime_;

  std::ofstream output_;

  aos::EventLoop *event_loop_;
};

}  // namespace frc::can_logger

#endif  // FRC_CAN_LOGGER_ASC_LOGGER_H_
