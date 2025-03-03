#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "frc/input/joystick_state_generated.h"

// Takes in log file and gives back the Match Type and Match #
int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::logger::LogReader reader(
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv)));
  reader.Register();
  const aos::Node *roborio =
      aos::configuration::GetNode(reader.configuration(), "roborio");

  std::unique_ptr<aos::EventLoop> event_loop =
      reader.event_loop_factory()->MakeEventLoop("roborio", roborio);

  frc::MatchType match_type = frc::MatchType::kNone;
  int match_number = 0;

  auto joystick_state_fetcher =
      event_loop->MakeFetcher<frc::JoystickState>("/roborio/frc");

  event_loop->AddPhasedLoop(
      [&](int) {
        // Fetch joystick state if null then don't give type and number
        if (!joystick_state_fetcher.Fetch()) {
          return;
        }
        match_type = joystick_state_fetcher->match_type();
        match_number = joystick_state_fetcher->match_number();
        // Exits if the match type isn't kNone if match type isn't kNone the
        // match has been found
        if (match_type != frc::MatchType::kNone) {
          reader.event_loop_factory()->Exit();
          return;
        }
      },
      std::chrono::seconds(1));

  reader.event_loop_factory()->Run();

  LOG(INFO) << "Match Type: " << frc::EnumNameMatchType(match_type);
  LOG(INFO) << "Match #: " << match_number;
}
