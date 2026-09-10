#include <ranges>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/die_if_null.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/network/demo/variable_size_message_static.h"

ABSL_FLAG(int32_t, sleep_us, 10000, "Time to sleep between messages");
ABSL_FLAG(int32_t, message_size, 10000,
          "Number of bytes to stuff into the message.");
ABSL_FLAG(int32_t, num_messages_per_sleep, 10,
          "Number of messages to send in a single wakeup.");
ABSL_FLAG(std::string, config, "config.json", "Path to the config.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  aos::EventLoop::SetDefaultVersionString("ping_version");

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());
  aos::Sender<aos::examples::VariableSizeMessageStatic> sender =
      event_loop.MakeSender<aos::examples::VariableSizeMessageStatic>("/test");

  const int32_t message_size = absl::GetFlag(FLAGS_message_size);
  uint64_t value = 0;
  const int num_messages_per_sleep =
      absl::GetFlag(FLAGS_num_messages_per_sleep);

  aos::TimerHandler *timer_handle = event_loop.AddTimer(
      [num_messages_per_sleep, &sender, message_size, &value] {
        for ([[maybe_unused]] int _ :
             std::views::iota(0, num_messages_per_sleep)) {
          aos::Sender<aos::examples::VariableSizeMessageStatic>::StaticBuilder
              builder = sender.MakeStaticBuilder();
          builder->set_value(value);
          auto data = ABSL_DIE_IF_NULL(builder->add_data());
          ABSL_CHECK(data->reserve(message_size));
          data->resize(message_size);
          builder.CheckOk(builder.Send());

          ++value;
        }
        VLOG(1) << "Sent " << num_messages_per_sleep << " messages.";
      });
  timer_handle->set_name("sender");

  event_loop.OnRun([timer_handle, &event_loop] {
    timer_handle->Schedule(
        event_loop.monotonic_now(),
        std::chrono::microseconds(absl::GetFlag(FLAGS_sleep_us)));
  });

  event_loop.SetRuntimeRealtimePriority(5);
  event_loop.Run();

  return 0;
}
