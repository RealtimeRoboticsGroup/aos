#include "absl/flags/flag.h"
#include "ctre/phoenix/cci/Diagnostics_CCI.h"

#include "aos/init.h"
#include "frc971/can_configuration_generated.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_can_position_static.h"
#include "frc971/control_loops/swerve/swerve_drivetrain_output_static.h"
#include "frc971/wpilib/can_sensor_reader.h"
#include "frc971/wpilib/loop_output_handler.h"
#include "frc971/wpilib/talonfx.h"
#include "y2024_swerve/constants.h"

using frc971::control_loops::swerve::SwerveModuleCanPositionStatic;
using frc971::wpilib::CANSensorReader;
using frc971::wpilib::TalonFX;
using frc971::wpilib::TalonFXParams;

ABSL_FLAG(bool, ctre_diag_server, false,
          "If true, enable the diagnostics server for interacting with "
          "devices on the CAN bus using Phoenix Tuner");

// Contains the objects for interacting with the hardware for a given swerve
// module, assuming that the module uses two TalonFX-based motor controllers and
// has a CTRE mag encoder on the rotation of the module.
struct SwerveModule {
  SwerveModule(TalonFXParams rotation_params, TalonFXParams translation_params,
               std::string canbus,
               std::vector<ctre::phoenix6::BaseStatusSignal *> *signals,
               double stator_current_limit, double supply_current_limit)
      : rotation(std::make_shared<TalonFX>(rotation_params, canbus, signals,
                                           stator_current_limit,
                                           supply_current_limit)),
        translation(std::make_shared<TalonFX>(translation_params, canbus,
                                              signals, stator_current_limit,
                                              supply_current_limit)) {}

  // Writes the requested torque currents from the module_output to the motors,
  // setting the maximum voltage of the motor outputs to the requested value.
  void WriteModule(
      const frc971::control_loops::swerve::SwerveModuleOutput *module_output,
      double max_voltage) {
    double rotation_current = 0.0;
    double translation_current = 0.0;

    if (module_output != nullptr) {
      rotation_current = module_output->rotation_current();
      translation_current = module_output->translation_current();
    }

    rotation->WriteCurrent(rotation_current, max_voltage);
    translation->WriteCurrent(translation_current, max_voltage);
  }

  struct ModuleGearRatios {
    double rotation;
    double translation;
  };

  // Populates a CAN-position message with the CAN-based devices (currently,
  // just the motors themselves).
  // Scales the motors' position values by the provided gear ratios.
  void PopulateCanPosition(
      frc971::control_loops::swerve::SwerveModuleCanPositionStatic
          *can_position,
      const ModuleGearRatios &ratios) {
    rotation->SerializePosition(can_position->add_rotation(), ratios.rotation);
    translation->SerializePosition(can_position->add_translation(),
                                   ratios.translation);
  }

  std::shared_ptr<TalonFX> rotation;
  std::shared_ptr<TalonFX> translation;
};

// Represents all the modules in a swerve drivetrain.
struct SwerveModules {
  void PopulateFalconsVector(std::vector<std::shared_ptr<TalonFX>> *falcons) {
    CHECK(falcons != nullptr);
    falcons->push_back(front_left->rotation);
    falcons->push_back(front_left->translation);

    falcons->push_back(front_right->rotation);
    falcons->push_back(front_right->translation);

    falcons->push_back(back_left->rotation);
    falcons->push_back(back_left->translation);

    falcons->push_back(back_right->rotation);
    falcons->push_back(back_right->translation);
  }

  std::shared_ptr<SwerveModule> front_left;
  std::shared_ptr<SwerveModule> front_right;
  std::shared_ptr<SwerveModule> back_left;
  std::shared_ptr<SwerveModule> back_right;
};

class DrivetrainWriter : public ::frc971::wpilib::LoopOutputHandler<
                             ::frc971::control_loops::swerve::Output> {
 public:
  DrivetrainWriter(::aos::EventLoop *event_loop, int drivetrain_writer_priority,
                   double max_voltage)
      : ::frc971::wpilib::LoopOutputHandler<
            ::frc971::control_loops::swerve::Output>(event_loop, "/drivetrain"),
        max_voltage_(max_voltage) {
    event_loop->SetRuntimeRealtimePriority(drivetrain_writer_priority);

    event_loop->OnRun([this]() { WriteConfigs(); });
  }

  void set_talonfxs(SwerveModules modules) { modules_ = std::move(modules); }

  void HandleCANConfiguration(const frc971::CANConfiguration &configuration) {
    for (auto module : {modules_.front_left, modules_.front_right,
                        modules_.back_left, modules_.back_right}) {
      module->rotation->PrintConfigs();
      module->translation->PrintConfigs();
    }
    if (configuration.reapply()) {
      WriteConfigs();
    }
  }

 private:
  void WriteConfigs() {
    for (auto module : {modules_.front_left, modules_.front_right,
                        modules_.back_left, modules_.back_right}) {
      module->rotation->WriteConfigs();
      module->translation->WriteConfigs();
    }
  }

  void Write(const ::frc971::control_loops::swerve::Output &output) {
    modules_.front_left->WriteModule(output.front_left_output(), max_voltage_);
    modules_.front_right->WriteModule(output.front_right_output(),
                                      max_voltage_);
    modules_.back_left->WriteModule(output.back_left_output(), max_voltage_);
    modules_.back_right->WriteModule(output.back_right_output(), max_voltage_);
  }

  void Stop() {
    AOS_LOG(WARNING, "drivetrain output too old\n");

    for (auto module : {modules_.front_left, modules_.front_right,
                        modules_.back_left, modules_.back_right}) {
      module->rotation->WriteCurrent(0, 0);
      module->translation->WriteCurrent(0, 0);
    }
  }

  SwerveModules modules_;

  double max_voltage_;
};

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig("aos_config.json");

  std::vector<aos::ShmEventLoop *> loops;

  std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry;
  std::vector<std::shared_ptr<TalonFX>> falcons;

  // TODO(max): Change the CanBus names with TalonFX software.
  SwerveModules modules{
      .front_left = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{6, true},
          frc971::wpilib::TalonFXParams{5, false}, "Drivetrain Bus",
          &signals_registry,
          y2024_swerve::constants::Values::kDrivetrainStatorCurrentLimit(),
          y2024_swerve::constants::Values::kDrivetrainSupplyCurrentLimit()),
      .front_right = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{3, true},
          frc971::wpilib::TalonFXParams{4, false}, "Drivetrain Bus",
          &signals_registry,
          y2024_swerve::constants::Values::kDrivetrainStatorCurrentLimit(),
          y2024_swerve::constants::Values::kDrivetrainSupplyCurrentLimit()),
      .back_left = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{7, true},
          frc971::wpilib::TalonFXParams{8, false}, "Drivetrain Bus",
          &signals_registry,
          y2024_swerve::constants::Values::kDrivetrainStatorCurrentLimit(),
          y2024_swerve::constants::Values::kDrivetrainSupplyCurrentLimit()),
      .back_right = std::make_shared<SwerveModule>(
          frc971::wpilib::TalonFXParams{2, true},
          frc971::wpilib::TalonFXParams{1, false}, "Drivetrain Bus",
          &signals_registry,
          y2024_swerve::constants::Values::kDrivetrainStatorCurrentLimit(),
          y2024_swerve::constants::Values::kDrivetrainSupplyCurrentLimit())};

  // Thread 1
  aos::ShmEventLoop can_sensor_reader_event_loop(&config.message());
  can_sensor_reader_event_loop.set_name("CANSensorReader");

  modules.PopulateFalconsVector(&falcons);
  falcons.emplace_back(std::make_shared<TalonFX>(9, false, "Drivetrain Bus",
                                                 &signals_registry, 10, 10));
  falcons.back()->WriteConfigs();

  aos::Sender<frc971::control_loops::swerve::CanPositionStatic>
      can_position_sender =
          can_sensor_reader_event_loop
              .MakeSender<frc971::control_loops::swerve::CanPositionStatic>(
                  "/drivetrain");

  CANSensorReader can_sensor_reader(
      &can_sensor_reader_event_loop, std::move(signals_registry), falcons,
      [falcons, modules,
       &can_position_sender](ctre::phoenix::StatusCode status) {
        // TODO(max): use status properly in the flatbuffer.
        (void)status;

        aos::Sender<frc971::control_loops::swerve::CanPositionStatic>::
            StaticBuilder builder = can_position_sender.MakeStaticBuilder();

        for (auto falcon : falcons) {
          falcon->RefreshNontimesyncedSignals();
        }

        const SwerveModule::ModuleGearRatios gear_ratios{
            .rotation = y2024_swerve::constants::Values::kRotationModuleRatio(),
            .translation =
                y2024_swerve::constants::Values::kTranslationModuleRatio()};
        modules.front_left->PopulateCanPosition(builder->add_front_left(),
                                                gear_ratios);
        modules.front_right->PopulateCanPosition(builder->add_front_right(),
                                                 gear_ratios);
        modules.back_left->PopulateCanPosition(builder->add_back_left(),
                                               gear_ratios);
        modules.back_right->PopulateCanPosition(builder->add_back_right(),
                                                gear_ratios);

        builder.CheckOk(builder.Send());
      });

  loops.push_back(&can_sensor_reader_event_loop);

  // Thread 2
  // Setup CAN
  if (!absl::GetFlag(FLAGS_ctre_diag_server)) {
    c_Phoenix_Diagnostics_SetSecondsToStart(-1);
    c_Phoenix_Diagnostics_Dispose();
  }

  ctre::phoenix::platform::can::CANComm_SetRxSchedPriority(
      y2024_swerve::constants::Values::kDrivetrainRxPriority, true,
      "Drivetrain Bus");
  ctre::phoenix::platform::can::CANComm_SetTxSchedPriority(
      y2024_swerve::constants::Values::kDrivetrainTxPriority, true,
      "Drivetrain Bus");

  aos::ShmEventLoop drivetrain_writer_event_loop(&config.message());
  drivetrain_writer_event_loop.set_name("DrivetrainWriter");

  DrivetrainWriter drivetrain_writer(
      &drivetrain_writer_event_loop,
      y2024_swerve::constants::Values::kDrivetrainWriterPriority, 12);

  drivetrain_writer.set_talonfxs(modules);

  loops.push_back(&drivetrain_writer_event_loop);

  std::vector<std::thread> threads;

  for (aos::ShmEventLoop *event_loop : loops) {
    threads.emplace_back([event_loop]() {
      LOG(INFO) << "Starting event loop " << event_loop->name();
      event_loop->Run();
    });
  }

  for (std::thread &thread : threads) {
    thread.join();
  }

  return 0;
};
