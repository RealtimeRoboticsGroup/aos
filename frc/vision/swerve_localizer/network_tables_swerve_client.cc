#include <sys/eventfd.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/die_if_null.h"
#include "absl/log/log.h"
#include "absl/strings/str_join.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/input/joystick_state_static.h"
#include "frc/input/robot_state_static.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/vision/swerve_localizer/chassis_speeds_static.h"
#include "frc/vision/swerve_localizer/pose2d_static.h"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleArrayTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/FloatTopic.h"
#include "networktables/IntegerTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include "networktables/StructTopic.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

ABSL_FLAG(std::string, server, "roborio", "");
ABSL_FLAG(std::string, chassis_speed_topic, "/DriveState/Speeds", "");
ABSL_FLAG(std::string, pose_topic, "/DriveState/Pose", "");
ABSL_FLAG(std::string, autonomous_topic,
          "/AdvantageKit/DriverStation/Autonomous", "");
ABSL_FLAG(std::string, alliance_station_topic,
          "/AdvantageKit/DriverStation/AllianceStation", "");
ABSL_FLAG(std::string, dsattached_topic,
          "/AdvantageKit/DriverStation/DSAttached", "");
ABSL_FLAG(std::string, emergency_stop_topic,
          "/AdvantageKit/DriverStation/EmergencyStop", "");
ABSL_FLAG(std::string, enabled_topic, "/AdvantageKit/DriverStation/Enabled",
          "");
ABSL_FLAG(std::string, event_name_topic,
          "/AdvantageKit/DriverStation/EventName", "");
ABSL_FLAG(std::string, fms_attached_topic,
          "/AdvantageKit/DriverStation/FMSAttached", "");
ABSL_FLAG(std::string, match_number_topic,
          "/AdvantageKit/DriverStation/MatchNumber", "");
ABSL_FLAG(std::string, match_time_topic,
          "/AdvantageKit/DriverStation/MatchTime", "");
ABSL_FLAG(std::string, match_type_topic,
          "/AdvantageKit/DriverStation/MatchType", "");
ABSL_FLAG(std::string, replay_number_topic,
          "/AdvantageKit/DriverStation/ReplayNumber", "");
ABSL_FLAG(std::string, test_topic, "/AdvantageKit/DriverStation/Test", "");
ABSL_FLAG(std::string, battery_voltage_topic,
          "/AdvantageKit/SystemStats/BatteryVoltage", "");

ABSL_FLAG(unsigned int, nt_min_log_level, 7,
          "Min log level to use for network tables.");
ABSL_FLAG(unsigned int, nt_max_log_level, UINT_MAX,
          "Max log level to use for network tables.");

namespace frc::vision::swerve_localizer {

class EventFd {
 public:
  EventFd() : fd_(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)) { CHECK_NE(fd_, -1); }
  ~EventFd() { close(fd_); }

  void Add(uint64_t i) { PCHECK(write(fd_, &i, sizeof(uint64_t)) == 8); }

  uint64_t Read() {
    uint64_t val;
    if (read(fd_, &val, sizeof(val)) == -1) {
      CHECK_EQ(errno, EAGAIN);
      return 0u;
    }
    return val;
  }

  int fd() { return fd_; }

 private:
  int fd_ = -1;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  EventFd speeds_eventfd;
  EventFd enabled_eventfd;
  aos::ShmEventLoop event_loop(&config.message());

  // TODO(austin): Set RT priority.  We want this to be higher priority than
  // apriltag detection.
  aos::Sender<ChassisSpeedsStatic> speeds_sender =
      event_loop.MakeSender<ChassisSpeedsStatic>("/drivetrain");
  aos::Sender<Pose2dStatic> pose_sender =
      event_loop.MakeSender<Pose2dStatic>("/drivetrain");
  aos::Sender<JoystickStateStatic> joystick_state_sender =
      event_loop.MakeSender<JoystickStateStatic>("/frc");
  aos::Sender<RobotStateStatic> robot_state_sender =
      event_loop.MakeSender<RobotStateStatic>("/frc");

  std::mutex connection_mutex;
  std::condition_variable connection_notify;

  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
  instance.SetServer(absl::GetFlag(FLAGS_server));
  instance.StartClient4("aos_swerve_client");

  instance.AddLogger(absl::GetFlag(FLAGS_nt_min_log_level),
                     absl::GetFlag(FLAGS_nt_max_log_level),
                     [](const nt::Event &event) {
                       const nt::LogMessage &log = *event.GetLogMessage();
                       std::cerr << log.filename << ":" << log.line << "("
                                 << log.level << "): " << log.message << "\n";
                     });

  instance.AddConnectionListener(
      /*whether to notify of existing connections*/ true,
      [&connection_mutex, &connection_notify](const nt::Event &event) {
        std::unique_lock<std::mutex> lock(connection_mutex);
        if (event.Is(nt::EventFlags::kConnected)) {
          VLOG(1) << "Connected!";
          connection_notify.notify_one();
        } else if (event.Is(nt::EventFlags::kDisconnected)) {
          VLOG(1) << "Disconnected!";
          connection_notify.notify_one();
        }
      });

  nt::StructTopic<frc::ChassisSpeeds> chassis_speed_topic;
  nt::StructTopic<frc::Pose2d> pose_topic;
  nt::StructSubscriber<frc::ChassisSpeeds> chassis_speed_subscriber;
  nt::StructSubscriber<frc::Pose2d> pose_subscriber;

  nt::BooleanTopic autonomous_topic;
  nt::BooleanSubscriber autonomous_subscriber;

  nt::IntegerTopic alliance_station_topic;
  nt::IntegerSubscriber alliance_station_subscriber;

  nt::BooleanTopic ds_attached_topic;
  nt::BooleanSubscriber ds_attached_subscriber;

  nt::BooleanTopic emergency_stop_topic;
  nt::BooleanSubscriber emergency_stop_subscriber;

  nt::BooleanTopic enabled_topic;
  nt::BooleanSubscriber enabled_subscriber;

  nt::StringTopic event_name_topic;
  nt::StringSubscriber event_name_subscriber;

  nt::BooleanTopic fms_attached_topic;
  nt::BooleanSubscriber fms_attached_subscriber;

  nt::IntegerTopic match_number_topic;
  nt::IntegerSubscriber match_number_subscriber;

  nt::IntegerTopic match_time_topic;
  nt::IntegerSubscriber match_time_subscriber;

  nt::IntegerTopic match_type_topic;
  nt::IntegerSubscriber match_type_subscriber;

  nt::IntegerTopic replay_number_topic;
  nt::IntegerSubscriber replay_number_subscriber;

  nt::BooleanTopic test_topic;
  nt::BooleanSubscriber test_subscriber;

  nt::DoubleTopic battery_voltage_topic;
  nt::DoubleSubscriber battery_voltage_subscriber;

  std::function<void()> publish_robot_state = [&]() {
    {
      aos::Sender<JoystickStateStatic>::StaticBuilder builder =
          joystick_state_sender.MakeStaticBuilder();
      builder->set_autonomous(autonomous_subscriber.Get());
      auto location = alliance_station_subscriber.GetAtomic();
      if (location.time != 0) {
        builder->set_location(location.value);
      }
      builder->set_ds_attached(ds_attached_subscriber.Get());
      builder->set_emergency_stop(emergency_stop_subscriber.Get());
      builder->set_enabled(enabled_subscriber.Get());

      {
        auto event_name_string = event_name_subscriber.GetAtomic();
        if (event_name_string.time != 0) {
          auto event_name = builder->add_event_name();
          CHECK(event_name->reserve(event_name_string.value.size() + 1));
          event_name->SetString(event_name_string.value);
        }
      }

      builder->set_fms_attached(fms_attached_subscriber.Get());

      auto match_number = match_number_subscriber.GetAtomic();
      if (match_number.time != 0) {
        builder->set_match_number(match_number.value);
      }

      auto match_time = match_time_subscriber.GetAtomic();
      if (match_time.time != 0) {
        builder->set_match_time(match_time.value);
      }
      auto match_type = match_type_subscriber.GetAtomic();
      if (match_type.time != 0) {
        builder->set_match_type(static_cast<MatchType>(match_type.value));
      }
      auto replay_number = replay_number_subscriber.GetAtomic();
      if (replay_number.time != 0) {
        builder->set_replay_number(replay_number.value);
      }
      builder->set_test_mode(test_subscriber.Get());

      builder.CheckOk(builder.Send());
    }

    {
      aos::Sender<RobotStateStatic>::StaticBuilder builder =
          robot_state_sender.MakeStaticBuilder();
      builder->set_voltage_battery(battery_voltage_subscriber.Get());
      builder.CheckOk(builder.Send());
    }
  };
  // /AdvantageKit/SystemStats/BatteryVoltage

  event_loop.epoll()->OnReadable(speeds_eventfd.fd(), [&]() {
    uint64_t events = speeds_eventfd.Read();
    // We just got poked and told there are values to read.  Go read them.
    {
      std::vector<nt::Timestamped<frc::ChassisSpeeds>> values =
          chassis_speed_subscriber.ReadQueue();
      for (const nt::Timestamped<frc::ChassisSpeeds> &value : values) {
        VLOG(1) << "At " << value.serverTime
                << " Got: " << value.value.vx.value() << ", "
                << value.value.vy.value() << ", " << value.value.omega.value();

        aos::Sender<ChassisSpeedsStatic>::StaticBuilder builder =
            speeds_sender.MakeStaticBuilder();
        builder->set_vx(value.value.vx.value());
        builder->set_vy(value.value.vy.value());
        builder->set_omega(value.value.omega.value());
        builder.CheckOk(builder.Send());
      }
    }

    {
      std::vector<nt::Timestamped<frc::Pose2d>> values =
          pose_subscriber.ReadQueue();
      for (const nt::Timestamped<frc::Pose2d> &value : values) {
        VLOG(1) << "At " << value.serverTime
                << " Got: " << value.value.X().value() << ", "
                << value.value.Y().value() << ", "
                << value.value.Rotation().Radians().value();

        aos::Sender<Pose2dStatic>::StaticBuilder builder =
            pose_sender.MakeStaticBuilder();
        builder->set_x(value.value.X().value());
        builder->set_y(value.value.Y().value());
        builder->set_theta(value.value.Rotation().Radians().value());
        builder.CheckOk(builder.Send());
      }
    }
    VLOG(1) << "Got " << events << " wakeups.";
  });

  event_loop.epoll()->OnReadable(enabled_eventfd.fd(), [&]() {
    uint64_t events = enabled_eventfd.Read();
    publish_robot_state();
    VLOG(1) << "Got " << events << " wakeups.";
  });

  {
    std::unique_lock<std::mutex> lock(connection_mutex);
    if (std::cv_status::timeout ==
        connection_notify.wait_for(lock, std::chrono::seconds(1))) {
      LOG(ERROR) << "Timed out connecting to " << absl::GetFlag(FLAGS_server);
      return 1;
    }

    CHECK(instance.IsConnected());

    chassis_speed_topic = instance.GetStructTopic<frc::ChassisSpeeds>(
        absl::GetFlag(FLAGS_chassis_speed_topic));
    chassis_speed_subscriber = chassis_speed_topic.Subscribe(
        frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s},
        {.pollStorage = 100, .periodic = 0.01, .keepDuplicates = true});

    pose_topic =
        instance.GetStructTopic<frc::Pose2d>(absl::GetFlag(FLAGS_pose_topic));
    pose_subscriber = pose_topic.Subscribe(
        frc::Pose2d{0_m, 0_m, 0_rad},
        {.pollStorage = 100, .periodic = 0.01, .keepDuplicates = true});

    autonomous_topic =
        instance.GetBooleanTopic(absl::GetFlag(FLAGS_autonomous_topic));
    autonomous_subscriber = autonomous_topic.Subscribe(
        false, {.pollStorage = 100, .keepDuplicates = true});

    alliance_station_topic =
        instance.GetIntegerTopic(absl::GetFlag(FLAGS_alliance_station_topic));
    alliance_station_subscriber = alliance_station_topic.Subscribe(-1);

    ds_attached_topic =
        instance.GetBooleanTopic(absl::GetFlag(FLAGS_dsattached_topic));
    ds_attached_subscriber = ds_attached_topic.Subscribe(false);

    emergency_stop_topic =
        instance.GetBooleanTopic(absl::GetFlag(FLAGS_emergency_stop_topic));
    emergency_stop_subscriber = emergency_stop_topic.Subscribe(false);

    enabled_topic =
        instance.GetBooleanTopic(absl::GetFlag(FLAGS_enabled_topic));
    enabled_subscriber = enabled_topic.Subscribe(
        false, {.pollStorage = 100, .keepDuplicates = true});

    event_name_topic =
        instance.GetStringTopic(absl::GetFlag(FLAGS_event_name_topic));
    event_name_subscriber = event_name_topic.Subscribe("");

    fms_attached_topic =
        instance.GetBooleanTopic(absl::GetFlag(FLAGS_fms_attached_topic));
    fms_attached_subscriber = fms_attached_topic.Subscribe(false);

    match_number_topic =
        instance.GetIntegerTopic(absl::GetFlag(FLAGS_match_number_topic));
    match_number_subscriber = match_number_topic.Subscribe(-1);

    match_time_topic =
        instance.GetIntegerTopic(absl::GetFlag(FLAGS_match_time_topic));
    match_time_subscriber = match_time_topic.Subscribe(-1);

    match_type_topic =
        instance.GetIntegerTopic(absl::GetFlag(FLAGS_match_type_topic));
    match_type_subscriber = match_type_topic.Subscribe(-1);

    replay_number_topic =
        instance.GetIntegerTopic(absl::GetFlag(FLAGS_replay_number_topic));
    replay_number_subscriber = replay_number_topic.Subscribe(-1);

    test_topic = instance.GetBooleanTopic(absl::GetFlag(FLAGS_test_topic));
    test_subscriber = test_topic.Subscribe(-1);

    battery_voltage_topic =
        instance.GetDoubleTopic(absl::GetFlag(FLAGS_battery_voltage_topic));
    battery_voltage_subscriber = battery_voltage_topic.Subscribe(
        0.0, {.pollStorage = 100, .keepDuplicates = true});

    instance.AddListener(chassis_speed_subscriber, nt::EventFlags::kValueAll,
                         [&speeds_eventfd](const nt::Event & /*event*/) {
                           // Poke the main thread.
                           speeds_eventfd.Add(1);
                         });
    instance.AddListener(pose_subscriber, nt::EventFlags::kValueAll,
                         [&speeds_eventfd](const nt::Event & /*event*/) {
                           // Poke the main thread.
                           speeds_eventfd.Add(1);
                         });

    instance.AddListener(enabled_subscriber, nt::EventFlags::kValueAll,
                         [&enabled_eventfd](const nt::Event & /*event*/) {
                           // Poke the main thread.
                           enabled_eventfd.Add(1);
                         });
  }

  aos::TimerHandler *enabled = event_loop.AddTimer(publish_robot_state);
  event_loop.OnRun([&]() {
    enabled->Schedule(event_loop.monotonic_now(),
                      std::chrono::milliseconds(20));
  });

  event_loop.Run();

  event_loop.epoll()->DeleteFd(speeds_eventfd.fd());
  event_loop.epoll()->DeleteFd(enabled_eventfd.fd());

  instance.StopClient();
  {
    std::unique_lock<std::mutex> lock(connection_mutex);
    if (std::cv_status::timeout ==
        connection_notify.wait_for(lock, std::chrono::seconds(1))) {
      LOG(ERROR) << "Timed out disconnecting from "
                 << absl::GetFlag(FLAGS_server);
      return 1;
    }
    CHECK(!instance.IsConnected());
  }

  return 0;
}

}  // namespace frc::vision::swerve_localizer

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return frc::vision::swerve_localizer::Main();
}
