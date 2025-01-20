#include "frc/can_configuration_generated.h"
#include "frc/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc/wpilib/loop_output_handler.h"
#include "frc/wpilib/talonfx.h"

namespace frc::wpilib {

class CANDrivetrainWriter : public ::frc::wpilib::LoopOutputHandler<
                                ::frc::control_loops::drivetrain::Output> {
 public:
  CANDrivetrainWriter(::aos::EventLoop *event_loop);

  void set_talonfxs(std::vector<std::shared_ptr<TalonFX>> right_talonfxs,
                    std::vector<std::shared_ptr<TalonFX>> left_talonfxs);

  void HandleCANConfiguration(const CANConfiguration &configuration);

  static constexpr int kDrivetrainWriterPriority = 35;

 private:
  void WriteConfigs();

  void Write(const ::frc::control_loops::drivetrain::Output &output) override;

  void Stop() override;

  std::vector<std::shared_ptr<TalonFX>> right_talonfxs_;
  std::vector<std::shared_ptr<TalonFX>> left_talonfxs_;
};

}  // namespace frc::wpilib
