#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/log/die_if_null.h"
#include "absl/strings/str_join.h"

#include "aos/init.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "networktables/DoubleArrayTopic.h"
#include "networktables/FloatTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StructTopic.h"

ABSL_FLAG(bool, topics, false, "Prints out a list of topics available.");
ABSL_FLAG(std::string, float_topic, "", "");
ABSL_FLAG(std::string, server, "127.0.0.1", "");
ABSL_FLAG(double, float_val, 0.0, "");

ABSL_FLAG(std::string, topic, "", "");
ABSL_FLAG(std::string, type, "double[]", "");

ABSL_FLAG(unsigned int, nt_min_log_level, 7,
          "Min log level to use for network tables.");
ABSL_FLAG(unsigned int, nt_max_log_level, UINT_MAX,
          "Max log level to use for network tables.");

namespace frc::vision {

int Main() {
  std::mutex connection_mutex;
  std::condition_variable connection_notify;

  nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
  instance.SetServer(absl::GetFlag(FLAGS_server));
  instance.StartClient4("aos_cli_client");

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
          ABSL_VLOG(1) << "Connected!";
          connection_notify.notify_one();
        } else if (event.Is(nt::EventFlags::kDisconnected)) {
          ABSL_VLOG(1) << "Disconnected!";
          connection_notify.notify_one();
        }
      });
  std::unique_lock<std::mutex> lock(connection_mutex);
  if (std::cv_status::timeout ==
      connection_notify.wait_for(lock, std::chrono::seconds(1))) {
    ABSL_LOG(ERROR) << "Timed out connecting to "
                    << absl::GetFlag(FLAGS_server);
    return 1;
  }

  ABSL_CHECK(instance.IsConnected());

  if (absl::GetFlag(FLAGS_topics)) {
    std::vector<std::string_view> prefixes = {""};
    instance.AddListener(
        {prefixes.data(), prefixes.size()}, nt::EventFlags::kTopic,
        [&connection_mutex, &connection_notify](const nt::Event &event) {
          std::unique_lock<std::mutex> lock(connection_mutex);
          const nt::TopicInfo &topic = *ABSL_DIE_IF_NULL(event.GetTopicInfo());
          ABSL_VLOG(1) << "Found " << topic.name << " " << topic.type_str;
          connection_notify.notify_one();
        });
    if (std::cv_status::timeout ==
        connection_notify.wait_for(lock, std::chrono::seconds(1))) {
      ABSL_LOG(ERROR) << "Timed out finding topics on "
                      << absl::GetFlag(FLAGS_server);
      return 1;
    }
    const std::vector<nt::Topic> topics = instance.GetTopics();
    ABSL_VLOG(1) << "Found " << topics.size() << " topics.";
    for (const nt::Topic &topic : topics) {
      std::cout << "\"" << topic.GetName() << "\" \"" << topic.GetTypeString()
                << "\"\n";
    }
  }
  if (absl::GetFlag(FLAGS_float_topic).size() > 0) {
    nt::FloatTopic topic =
        instance.GetFloatTopic(absl::GetFlag(FLAGS_float_topic));
    nt::FloatSubscriber subscriber =
        topic.Subscribe(std::numeric_limits<double>::quiet_NaN());
    instance.Flush();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << subscriber.Get() << "\n";
    nt::FloatPublisher publisher = topic.Publish();
    publisher.Set(absl::GetFlag(FLAGS_float_val));
    instance.Flush();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::this_thread::sleep_for(std::chrono::seconds(100));
    std::cout << subscriber.Get() << "\n";
  }

  if (absl::GetFlag(FLAGS_topic).size() > 0) {
    if (absl::GetFlag(FLAGS_type) == "double[]") {
      nt::DoubleArrayTopic topic =
          instance.GetDoubleArrayTopic(absl::GetFlag(FLAGS_topic));
      nt::DoubleArraySubscriber subscriber = topic.Subscribe({});
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << absl::StrJoin(subscriber.Get(), ", ") << "\n";
      std::cout << topic.GetTypeString();
    } else if (absl::GetFlag(FLAGS_type) == "struct:ChassisSpeeds") {
      nt::StructTopic<frc::ChassisSpeeds> topic =
          instance.GetStructTopic<frc::ChassisSpeeds>(
              absl::GetFlag(FLAGS_topic));
      nt::StructSubscriber<frc::ChassisSpeeds> subscriber =
          topic.Subscribe(frc::ChassisSpeeds{0_mps, 0_mps, 0_rad_per_s});
      instance.AddListener(
          subscriber, nt::EventFlags::kValueAll,
          [&subscriber](const nt::Event & /*event*/) {
            std::vector<nt::Timestamped<frc::ChassisSpeeds>> values =
                subscriber.ReadQueue();
            for (const nt::Timestamped<frc::ChassisSpeeds> &value : values) {
              ABSL_LOG(INFO) << "At " << value.serverTime
                             << " Got: " << value.value.vx.value() << ", "
                             << value.value.vy.value() << ", "
                             << value.value.omega.value();
            }
          });
      std::this_thread::sleep_for(std::chrono::seconds(100));
    } else {
      ABSL_LOG(FATAL) << "Unsupported type " << absl::GetFlag(FLAGS_type);
    }
  }

  instance.StopClient();
  if (std::cv_status::timeout ==
      connection_notify.wait_for(lock, std::chrono::seconds(1))) {
    ABSL_LOG(ERROR) << "Timed out disconnecting from "
                    << absl::GetFlag(FLAGS_server);
    return 1;
  }
  ABSL_CHECK(!instance.IsConnected());

  return 0;
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  return frc::vision::Main();
}
