#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/ip.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/die_if_null.h"
#include "absl/log/log.h"
#include "absl/strings/str_join.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/vision/events/udp.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc/input/joystick_state_static.h"
#include "frc/input/robot_state_static.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/vision/game_piece_locations_static.h"
#include "frc/vision/swerve_localizer/chassis_speeds_static.h"
#include "frc/vision/swerve_localizer/pose2d_static.h"
#include "frc/vision/swerve_localizer/udp_status_static.h"
#include "frc/vision/target_map_generated.h"
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
ABSL_FLAG(uint16_t, drive_state_port, 4647,
          "Port to listen for drivestate UDP messages on.");
ABSL_FLAG(uint16_t, pose_port, 4648, "Port to publish poses to");
ABSL_FLAG(uint16_t, game_piece_port, 4649, "Port to publish game pieces to");
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

std::string ResolveHostname(std::string_view host, int port) {
  struct sockaddr_storage result;
  memset(&result, 0, sizeof(result));
  struct addrinfo *addrinfo_result;
  struct sockaddr_in *t_addr = (struct sockaddr_in *)&result;
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  // We deliberately avoid AI_ADDRCONFIG here because it breaks running things
  // inside Bazel's test sandbox, which has no non-localhost IPv4 or IPv6
  // addresses. Also, it's not really helpful, because most systems will have
  // link-local addresses of both types with any interface that's up.
  hints.ai_flags = AI_PASSIVE | AI_V4MAPPED | AI_NUMERICSERV;
  int ret = getaddrinfo(host.empty() ? nullptr : std::string(host).c_str(),
                        std::to_string(port).c_str(), &hints, &addrinfo_result);
  if (ret == EAI_SYSTEM) {
    PLOG(FATAL) << "getaddrinfo failed to look up '" << host << "'";
  } else if (ret != 0) {
    LOG(FATAL) << "getaddrinfo failed to look up '" << host
               << "': " << gai_strerror(ret);
  }
  switch (addrinfo_result->ai_family) {
    case AF_INET:
      memcpy(t_addr, addrinfo_result->ai_addr, addrinfo_result->ai_addrlen);
      t_addr->sin_family = addrinfo_result->ai_family;
      t_addr->sin_port = htons(port);

      break;
    default:
      LOG(FATAL) << "Unsupported family";
  }

  // Now print it back out nicely.
  char host_string[NI_MAXHOST];
  char service_string[NI_MAXSERV];

  int error = getnameinfo((struct sockaddr *)&result,
                          addrinfo_result->ai_addrlen, host_string, NI_MAXHOST,
                          service_string, NI_MAXSERV, NI_NUMERICHOST);

  if (error) {
    LOG(ERROR) << "Reverse lookup failed ... " << gai_strerror(error);
  }

  LOG(INFO) << "remote:addr=" << host_string << ", port=" << service_string
            << ", family=" << addrinfo_result->ai_family;

  freeaddrinfo(addrinfo_result);

  return std::string(host_string);
}

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

class CoralForwarder {
 public:
  CoralForwarder(
      aos::EventLoop *event_loop, nt::NetworkTableInstance *instance,
      frc::constants::ConstantsFetcher<TargetMap> *target_map_fetcher)
      : event_loop_(event_loop),
        instance_(instance),
        target_map_fetcher_(target_map_fetcher),
        udp_server_(ResolveHostname(absl::GetFlag(FLAGS_server),
                                    absl::GetFlag(FLAGS_game_piece_port))),
        game_piece_socket_(udp_server_, absl::GetFlag(FLAGS_game_piece_port)),
        localizer_output_fetcher_(
            event_loop_->MakeFetcher<frc::controls::LocalizerOutput>(
                "/localizer")) {
    event_loop_->MakeWatcher(
        "/camera1/coral",
        [this](const frc::vision::GamePieceLocations &locations) {
          HandleGamePieceLocations(locations);
        });
  }

  size_t send_failure_count() const { return send_failure_count_; }

  void reset_send_failure_count() { send_failure_count_ = 0; }

 private:
  void HandleGamePieceLocations(
      const frc::vision::GamePieceLocations &locations) {
    auto offset = instance_->GetServerTimeOffset();
    if (!offset.has_value()) {
      VLOG(1) << "Not connected, ignoring";
      return;
    }

    localizer_output_fetcher_.Fetch();
    if (localizer_output_fetcher_.get() == nullptr) {
      return;
    }

    if (!locations.has_locations()) return;

    const double x = localizer_output_fetcher_->x();
    const double y = localizer_output_fetcher_->y();
    const double theta = localizer_output_fetcher_->theta();

    const Eigen::Affine3d robot_to_field =
        Eigen::Translation3d(Eigen::Vector3d(x, y, 0.0)) *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    const Eigen::Vector3d intake = robot_to_field * Eigen::Vector3d(-1.0, 0.0, 0.0);

    // 1: pick the best one.
    //
    // 2: Pack it and ship it.

    // confidence, x, y, width, height, time
    std::array<double, 6> game_piece_data{
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                event_loop_->context().realtime_event_time.time_since_epoch())
                .count() +
            offset.value())};

    if (locations.locations()->size() > 0) {
      const frc::vision::GamePieceLocation *best_location = nullptr;
      for (const frc::vision::GamePieceLocation *location :
           *locations.locations()) {
        if (best_location == nullptr) {
          best_location = location;
          continue;
        }

        if (std::hypot(best_location->x() - intake.x(),
                       best_location->y() - intake.y()) >
            std::hypot(location->x() - intake.x(),
                       location->y() - intake.y())) {
          best_location = location;
        }
      }

      game_piece_data[0] = best_location->confidence();
      game_piece_data[1] = best_location->x() +
                           target_map_fetcher_->constants().fieldlength() / 2.0;
      game_piece_data[2] = best_location->y() +
                           target_map_fetcher_->constants().fieldwidth() / 2.0;
      game_piece_data[3] = best_location->width();
      game_piece_data[4] = best_location->height();
    }

    if (game_piece_socket_.Send(
            reinterpret_cast<char *>(game_piece_data.data()),
            game_piece_data.size() * sizeof(double)) !=
        game_piece_data.size() * sizeof(double)) {
      ++send_failure_count_;
      VLOG(1) << "Send failure";
    }
  }

  aos::EventLoop *event_loop_;
  nt::NetworkTableInstance *instance_;
  frc::constants::ConstantsFetcher<TargetMap> *target_map_fetcher_;

  std::string udp_server_;
  aos::events::TXUdpSocket game_piece_socket_;

  aos::Fetcher<frc::controls::LocalizerOutput> localizer_output_fetcher_;

  size_t send_failure_count_ = 0;
};

int Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  EventFd enabled_eventfd;
  aos::events::RXUdpSocket drive_state_socket(
      absl::GetFlag(FLAGS_drive_state_port));

  frc::constants::WaitForConstants<frc::vision::TargetMap>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());
  event_loop.SetRuntimeRealtimePriority(1);

  frc::constants::ConstantsFetcher<TargetMap> target_map_fetcher(&event_loop);

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
  aos::Sender<UdpStatusStatic> udp_status_sender =
      event_loop.MakeSender<UdpStatusStatic>("/frc");

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

  std::string udp_server = ResolveHostname(absl::GetFlag(FLAGS_server),
                                           absl::GetFlag(FLAGS_pose_port));

  aos::events::TXUdpSocket pose_socket(udp_server,
                                       absl::GetFlag(FLAGS_pose_port));
  size_t send_failure_count = 0;

  event_loop.MakeWatcher(
      "/localizer",
      [&](const frc::controls::LocalizerOutput &localizer_output) {
        auto offset = instance.GetServerTimeOffset();
        if (!offset.has_value()) {
          VLOG(1) << "Not connected, ignoring";
          return;
        }

        std::array<double, 4> pose_data;

        pose_data[0] = localizer_output.x() +
                       target_map_fetcher.constants().fieldlength() / 2.0;
        pose_data[1] = localizer_output.y() +
                       target_map_fetcher.constants().fieldwidth() / 2.0;
        pose_data[2] = localizer_output.theta();
        pose_data[3] =
            std::chrono::duration_cast<std::chrono::microseconds>(
                event_loop.context().realtime_event_time.time_since_epoch())
                .count() +
            offset.value();

        if (pose_socket.Send(reinterpret_cast<char *>(pose_data.data()),
                         pose_data.size() * sizeof(double)) != pose_data.size() * sizeof(double)) {
          ++send_failure_count;
          VLOG(1) << "Send failure";
        }
      });

  CoralForwarder coral_forwarder(&event_loop, &instance, &target_map_fetcher);

  event_loop.epoll()->OnReadable(drive_state_socket.fd(), [&]() {
    std::array<uint8_t, 256> buffer;

    const int received_length =
        drive_state_socket.Recv(buffer.data(), buffer.size());
    CHECK_EQ(received_length % sizeof(double), 0u);

    std::span<const double> data(
        reinterpret_cast<const double *>(buffer.data()),
        received_length / sizeof(double));
    CHECK_EQ(data.size(), 7u);

    auto offset = instance.GetServerTimeOffset();
    if (!offset.has_value()) {
      VLOG(1) << "Not connected";
      return;
    }

    aos::realtime_clock::time_point publish_time(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(data[6])) -
        std::chrono::microseconds(offset.value()));
    VLOG(1) << "Published at " << publish_time << " now "
            << aos::realtime_clock::now() << " offset "
            << std::chrono::duration<double, std::milli>(
                   publish_time - aos::realtime_clock::now())
                   .count()
            << " ms";
    {
      aos::Sender<Pose2dStatic>::StaticBuilder builder =
          pose_sender.MakeStaticBuilder();
      builder->set_x(data[0]);
      builder->set_y(data[1]);
      builder->set_theta(data[2]);
      builder->set_age(std::chrono::duration<double>(
                           aos::realtime_clock::now() - publish_time)
                           .count());
      builder.CheckOk(builder.Send());
    }
    {
      aos::Sender<ChassisSpeedsStatic>::StaticBuilder builder =
          speeds_sender.MakeStaticBuilder();
      builder->set_vx(data[3]);
      builder->set_vy(data[4]);
      builder->set_omega(data[5]);
      builder->set_age(std::chrono::duration<double>(
                           aos::realtime_clock::now() - publish_time)
                           .count());
      builder.CheckOk(builder.Send());
    }
  });

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

    instance.AddListener(enabled_subscriber, nt::EventFlags::kValueAll,
                         [&enabled_eventfd](const nt::Event & /*event*/) {
                           // Poke the main thread.
                           enabled_eventfd.Add(1);
                         });
  }

  aos::TimerHandler *status_timer = event_loop.AddTimer([&]() {
    aos::Sender<UdpStatusStatic>::StaticBuilder builder =
        udp_status_sender.MakeStaticBuilder();
    auto faults = builder->add_faults();
    const size_t overall_send_failure_count =
        send_failure_count + coral_forwarder.send_failure_count();
    if (overall_send_failure_count > 0) {
      CHECK(faults->reserve(1));
      CHECK(faults->emplace_back(NetworkHealth::SEND_FAILURE));
      send_failure_count = 0;
      coral_forwarder.reset_send_failure_count();
    }
    builder.CheckOk(builder.Send());
  });
  aos::TimerHandler *enabled = event_loop.AddTimer(publish_robot_state);
  event_loop.OnRun([&]() {
    enabled->Schedule(event_loop.monotonic_now(),
                      std::chrono::milliseconds(20));
    status_timer->Schedule(event_loop.monotonic_now(),
                           std::chrono::milliseconds(1000));
  });

  event_loop.Run();

  event_loop.epoll()->DeleteFd(enabled_eventfd.fd());
  event_loop.epoll()->DeleteFd(drive_state_socket.fd());

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
