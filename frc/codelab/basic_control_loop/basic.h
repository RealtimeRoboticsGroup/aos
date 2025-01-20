#ifndef FRC_CODELAB_BASIC_H_
#define FRC_CODELAB_BASIC_H_

#include "aos/time/time.h"
#include "frc/codelab/basic_control_loop/basic_goal_generated.h"
#include "frc/codelab/basic_control_loop/basic_output_static.h"
#include "frc/codelab/basic_control_loop/basic_position_generated.h"
#include "frc/codelab/basic_control_loop/basic_status_static.h"
#include "frc/control_loops/control_loop.h"

namespace frc::codelab {

class Basic : public ::frc::controls::ControlLoop<Goal, Position, StatusStatic,
                                                  OutputStatic> {
 public:
  explicit Basic(::aos::EventLoop *event_loop,
                 const ::std::string &name = "/codelab");

 protected:
  void RunIteration(const Goal *goal, const Position *position,
                    aos::Sender<OutputStatic>::StaticBuilder *output,
                    aos::Sender<StatusStatic>::StaticBuilder *status) override;
};

}  // namespace frc::codelab

#endif  // FRC_CODELAB_BASIC_H_
