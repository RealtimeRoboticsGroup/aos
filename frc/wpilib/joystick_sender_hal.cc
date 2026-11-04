#include "frc/wpilib/joystick_sender_hal.h"

#include <hal/DriverStation.h>

#include <algorithm>
#include <cstring>
#include <string_view>

#include "absl/log/check.h"
#include <wpi/Synchronization.h>

#include "aos/logging/logging.h"
#include "aos/network/team_number.h"
#include "frc/input/driver_station_data.h"

namespace frc::wpilib {

using ::frc::input::driver_station::JoystickAxis;
using ::frc::input::driver_station::JoystickFeature;

JoystickSenderHal::JoystickSenderHal(::aos::ShmEventLoop *event_loop)
    : event_loop_(event_loop),
      joystick_state_sender_(
          event_loop_->MakeSender<frc::JoystickStateStatic>("/frc")),
      team_id_(aos::network::GetTeamNumber()) {
  event_loop_->SetRuntimeRealtimePriority(28);
  event_loop->set_name("joystick_sender");

  // These run here, before the event loop goes realtime, because
  // HAL_ObserveUserProgramStarting allocates inside NI's netcomm library.
  //
  // The HAL_Observe* functions let the Driver Station know that the robot code
  // is alive.  This drives the Robot Code light on the DS, and the DS won't
  // enable a robot it thinks has no code.
  HAL_ObserveUserProgramStarting();
  HAL_ProvideNewDataEventHandle(new_data_event_.GetHandle());

  event_loop_->OnRun([this]() {
    // TODO(Brian): Fix the potential deadlock when stopping here (condition
    // variable / mutex needs to get exposed all the way out or something).
    while (event_loop_->is_running()) {
      wpi::WaitForObject(new_data_event_.GetHandle());
      SendJoystickState();
    }
  });
}

JoystickSenderHal::~JoystickSenderHal() {
  HAL_RemoveNewDataEventHandle(new_data_event_.GetHandle());
}

void JoystickSenderHal::SendJoystickState() {
  // Pull the new packet into the HAL's cache.  This is what wpilibc's
  // DriverStation::RefreshData does, minus the NetworkTables publishing, and
  // it keeps anything in the process reading frc::DriverStation up to date.
  HAL_RefreshDSData();

  HAL_ControlWord control_word;
  HAL_GetControlWord(&control_word);

  // Keep feeding the DS's watchdog.
  if (!control_word.enabled) {
    HAL_ObserveUserProgramDisabled();
  } else if (control_word.autonomous) {
    HAL_ObserveUserProgramAutonomous();
  } else if (control_word.test) {
    HAL_ObserveUserProgramTest();
  } else {
    HAL_ObserveUserProgramTeleop();
  }

  auto builder = joystick_state_sender_.MakeStaticBuilder();
  frc::JoystickStateStatic *joystick_state = builder.get();

  auto *joysticks = joystick_state->add_joysticks();
  for (int i = 0; i < JoystickFeature::kJoysticks; ++i) {
    HAL_JoystickAxes axes;
    HAL_GetJoystickAxes(i, &axes);
    HAL_JoystickPOVs povs;
    HAL_GetJoystickPOVs(i, &povs);
    HAL_JoystickButtons buttons;
    HAL_GetJoystickButtons(i, &buttons);

    frc::JoystickStatic *joystick = joysticks->emplace_back();
    ABSL_CHECK(joystick != nullptr);

    joystick->set_buttons(buttons.buttons);

    // Axes the joystick doesn't have read as 0.
    auto *axis = joystick->add_axis();
    for (int j = 0; j < JoystickAxis::kAxes; ++j) {
      ABSL_CHECK(axis->emplace_back(j < axes.count ? axes.axes[j] : 0.0));
    }

    if (povs.count > 0) {
      joystick->set_pov(povs.povs[0]);
    }
  }

  HAL_MatchInfo match_info;
  HAL_GetMatchInfo(&match_info);

  const std::string_view game_data(
      reinterpret_cast<const char *>(match_info.gameSpecificMessage),
      std::min<size_t>(match_info.gameSpecificMessageSize,
                       sizeof(match_info.gameSpecificMessage)));
  if (game_data.size() >= 2u) {
    joystick_state->set_switch_left(game_data[0] == 'L' || game_data[0] == 'l');
    joystick_state->set_scale_left(game_data[1] == 'L' || game_data[1] == 'l');
  }
  aos::fbs::SetStringOrDie(joystick_state->add_game_data(), game_data);

  aos::fbs::SetStringOrDie(
      joystick_state->add_event_name(),
      std::string_view(
          match_info.eventName,
          strnlen(match_info.eventName, sizeof(match_info.eventName))));

  joystick_state->set_test_mode(control_word.test);
  joystick_state->set_fms_attached(control_word.fmsAttached);
  joystick_state->set_ds_attached(control_word.dsAttached);
  joystick_state->set_enabled(control_word.enabled);
  joystick_state->set_autonomous(control_word.autonomous);
  joystick_state->set_emergency_stop(control_word.eStop);

  int32_t status = 0;
  switch (HAL_GetAllianceStation(&status)) {
    case HAL_AllianceStationID_kRed1:
      joystick_state->set_alliance(frc::Alliance::kRed);
      joystick_state->set_location(1);
      break;
    case HAL_AllianceStationID_kRed2:
      joystick_state->set_alliance(frc::Alliance::kRed);
      joystick_state->set_location(2);
      break;
    case HAL_AllianceStationID_kRed3:
      joystick_state->set_alliance(frc::Alliance::kRed);
      joystick_state->set_location(3);
      break;
    case HAL_AllianceStationID_kBlue1:
      joystick_state->set_alliance(frc::Alliance::kBlue);
      joystick_state->set_location(1);
      break;
    case HAL_AllianceStationID_kBlue2:
      joystick_state->set_alliance(frc::Alliance::kBlue);
      joystick_state->set_location(2);
      break;
    case HAL_AllianceStationID_kBlue3:
      joystick_state->set_alliance(frc::Alliance::kBlue);
      joystick_state->set_location(3);
      break;
    default:
      joystick_state->set_alliance(frc::Alliance::kInvalid);
      joystick_state->set_location(0);
      break;
  }

  joystick_state->set_team_id(team_id_);
  joystick_state->set_match_number(match_info.matchNumber);
  joystick_state->set_replay_number(match_info.replayNumber);
  joystick_state->set_match_time(static_cast<int>(HAL_GetMatchTime(&status)));

  switch (match_info.matchType) {
    case HAL_kMatchType_none:
      joystick_state->set_match_type(frc::MatchType::kNone);
      break;
    case HAL_kMatchType_practice:
      joystick_state->set_match_type(frc::MatchType::kPractice);
      break;
    case HAL_kMatchType_qualification:
      joystick_state->set_match_type(frc::MatchType::kQualification);
      break;
    case HAL_kMatchType_elimination:
      joystick_state->set_match_type(frc::MatchType::kElimination);
      break;
  }

  if (builder.Send() != aos::RawSender::Error::kOk) {
    AOS_LOG(WARNING, "sending joystick_state failed\n");
  }
}

}  // namespace frc::wpilib
