#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/input/joystick_state_generated.h"
#include "frc/vision/camera_settings_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

ABSL_FLAG(uint32_t, red_exposure, 15, "Exposure on red");
ABSL_FLAG(uint32_t, blue_exposure, 20, "Exposure on blue");

namespace frc::vision {

void Main() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  aos::Fetcher<JoystickState> joystick_state_fetcher =
      event_loop.MakeFetcher<JoystickState>("/frc");

  aos::Sender<CameraStreamSettings> camera0_sender =
      event_loop.MakeSender<CameraStreamSettings>("/camera0");
  aos::Sender<CameraStreamSettings> camera1_sender =
      event_loop.MakeSender<CameraStreamSettings>("/camera1");
  aos::Sender<CameraStreamSettings> camera2_sender =
      event_loop.MakeSender<CameraStreamSettings>("/camera2");
  aos::Sender<CameraStreamSettings> camera3_sender =
      event_loop.MakeSender<CameraStreamSettings>("/camera3");

  aos::TimerHandler *exposure_timer = event_loop.AddTimer([&]() {
    joystick_state_fetcher.Fetch();
    if (joystick_state_fetcher.get() == nullptr) {
      return;
    }

    uint32_t exposure;
    switch (joystick_state_fetcher->alliance()) {
      case Alliance::kRed:
        exposure = absl::GetFlag(FLAGS_red_exposure);
        break;
      case Alliance::kBlue:
        exposure = absl::GetFlag(FLAGS_blue_exposure);
        break;
      case Alliance::kInvalid:
        return;
    };

    for (aos::Sender<CameraStreamSettings> *sender :
         {&camera0_sender, &camera1_sender, &camera2_sender, &camera3_sender}) {
      aos::Sender<CameraStreamSettings>::Builder builder = sender->MakeBuilder();
      auto stream_settings_builder =
          builder.MakeBuilder<CameraStreamSettings>();
      stream_settings_builder.add_exposure_100us(exposure);
      builder.CheckOk(builder.Send(stream_settings_builder.Finish()));
    }
  });

  event_loop.OnRun([&]() {
    exposure_timer->Schedule(event_loop.monotonic_now(),
                             std::chrono::seconds(1));
  });

  event_loop.Run();
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::Main();
}
