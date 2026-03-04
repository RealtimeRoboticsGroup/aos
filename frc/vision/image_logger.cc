#include <sys/resource.h>
#include <sys/time.h>

#include <filesystem>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_writer.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/logging/log_namer.h"
#include "aos/util/filesystem_generated.h"
#include "frc/input/joystick_state_generated.h"

// The base name for log files
constexpr char kLogBaseName[] = "image_log";

// Declare the logging_folder flag which is defined in aos/logging/log_namer.cc
ABSL_DECLARE_FLAG(std::string, logging_folder);

ABSL_FLAG(std::string, config, "aos_config.json", "Config file to use.");
ABSL_FLAG(double, rotate_every, 0.0,
          "If set, rotate the logger after this many seconds");
ABSL_DECLARE_FLAG(int32_t, flush_size);
ABSL_FLAG(double, disabled_time, 5.0,
          "Continue logging if disabled for this amount of time or less");
ABSL_FLAG(bool, direct, false,
          "If true, write using O_DIRECT and write 512 byte aligned blocks "
          "whenever possible.");
ABSL_FLAG(bool, always_log, false,
          "If true, ignore the disabled signal and log all the time.");

std::unique_ptr<aos::logger::MultiNodeFilesLogNamer> MakeLogNamer(
    aos::EventLoop *event_loop, std::optional<std::string> log_name) {
  if (!log_name.has_value()) {
    return nullptr;
  }

  return std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
      event_loop,
      std::make_unique<aos::logger::RenamableFileBackend>(
          absl::StrCat(log_name.value(), "/"), absl::GetFlag(FLAGS_direct)));
}

struct LogName {
  std::string MakeName() const {
    if (match_number > 0 && replay_number > 0 &&
        (match_type == frc::MatchType::kPractice ||
         match_type == frc::MatchType::kQualification ||
         match_type == frc::MatchType::kElimination)) {
      std::string match_type_string =
          (match_type == frc::MatchType::kQualification
               ? "q"
               : (match_type == frc::MatchType::kElimination ? "e" : "p"));
      std::string event_name_prepend;

      if (!event_name.empty()) {
        event_name_prepend = absl::StrCat(event_name, "-");
      }

      if (replay_number == 1) {
        return absl::StrCat(log_name, "-", event_name_prepend,
                            match_type_string, match_number, "/");
      } else {
        return absl::StrCat(log_name, "-", event_name_prepend,
                            match_type_string, match_number, "-r",
                            replay_number, "/");
      }
    }
    return absl::StrCat(log_name, "/");
  }

  void Reset(std::string new_log_name) {
    match_number = -1;
    replay_number = -1;
    match_type = frc::MatchType::kNone;
    log_name = new_log_name;
    event_name = "";
  }

  std::string log_name;
  int match_number = -1;
  int replay_number = -1;
  frc::MatchType match_type = frc::MatchType::kNone;
  std::string event_name;

  bool Update(const frc::JoystickState &joystick_state) {
    bool update = false;
    if (joystick_state.match_number() != match_number) {
      update = true;
    }
    match_number = joystick_state.match_number();

    if (joystick_state.replay_number() != replay_number) {
      update = true;
    }
    replay_number = joystick_state.replay_number();

    if (joystick_state.match_type() != match_type &&
        (match_type == frc::MatchType::kPractice ||
         match_type == frc::MatchType::kQualification ||
         match_type == frc::MatchType::kElimination)) {
      update = true;
    }
    match_type = joystick_state.match_type();

    if (joystick_state.has_event_name() &&
        joystick_state.event_name()->string_view() != event_name) {
      event_name = joystick_state.event_name()->string_view();
      update = true;
    }

    return update;
  }
};

int main(int argc, char *argv[]) {
  absl::SetProgramUsageMessage(
      "This program provides a simple logger binary that logs all SHMEM data "
      "directly to a file specified at the command line when the robot is "
      "enabled and for a bit of time after.");
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  aos::Fetcher<aos::util::FilesystemStatus> filesystem_status =
      event_loop.MakeFetcher<aos::util::FilesystemStatus>("/aos");

  bool logging = false;
  bool enabled = false;
  aos::monotonic_clock::time_point last_disable_time =
      aos::monotonic_clock::min_time;
  aos::monotonic_clock::time_point last_rotation_time =
      event_loop.monotonic_now();
  aos::logger::Logger logger(&event_loop);

  if (absl::GetFlag(FLAGS_rotate_every) != 0.0) {
    logger.set_on_logged_period([&](aos::monotonic_clock::time_point) {
      const auto now = event_loop.monotonic_now();
      if (logging &&
          now > last_rotation_time + std::chrono::duration<double>(
                                         absl::GetFlag(FLAGS_rotate_every))) {
        logger.Rotate();
        last_rotation_time = now;
      }
    });
  }

  LOG(INFO) << "Starting image_logger; will wait on joystick enabled to start "
               "logging";
  event_loop.OnRun([]() {
    errno = 0;
    setpriority(PRIO_PROCESS, 0, -20);
    PCHECK(errno == 0) << ": Renicing to -20 failed.";
  });

  LogName log_name_accumulator;
  aos::logger::MultiNodeFilesLogNamer *current_log_namer = nullptr;

  event_loop.MakeWatcher("/frc", [&](const frc::JoystickState &joystick_state) {
    const auto timestamp = event_loop.context().monotonic_event_time;
    filesystem_status.Fetch();
    bool joystick_state_enabled = joystick_state.enabled();
    if (absl::GetFlag(FLAGS_always_log)) {
      joystick_state_enabled = true;
    }

    // Store the last time we got disabled
    if (enabled && !joystick_state_enabled) {
      last_disable_time = timestamp;
    }
    enabled = joystick_state_enabled;

    bool enough_space = true;

    if (filesystem_status.get() != nullptr) {
      enough_space = false;
      for (const aos::util::Filesystem *fs :
           *filesystem_status->filesystems()) {
        CHECK(fs->has_path());
        if (fs->path()->string_view() == "/") {
          if (fs->free_space() > 50ull * 1024ull * 1024ull * 1024ull) {
            enough_space = true;
          }
        }
      }
    }

    const bool should_be_logging =
        (enabled || timestamp < last_disable_time +
                                    std::chrono::duration<double>(
                                        absl::GetFlag(FLAGS_disabled_time))) &&
        enough_space;

    if (!logging && should_be_logging) {
      std::optional<std::string> log_name =
          aos::logging::MaybeGetLogName(kLogBaseName);
      auto log_namer = MakeLogNamer(&event_loop, log_name);
      if (log_namer == nullptr) {
        return;
      }
      current_log_namer = log_namer.get();

      // Start logging if we just got enabled
      LOG(INFO) << "Starting logging to " << log_namer->base_name();
      logger.StartLogging(std::move(log_namer));
      logging = true;
      last_rotation_time = event_loop.monotonic_now();
      log_name_accumulator.Reset(log_name.value());
    } else if (logging && !should_be_logging) {
      // Stop logging if we've been disabled for a non-negligible amount of
      // time
      LOG(INFO) << "Stopping logging";
      aos::CheckExpected(logger.StopLogging(event_loop.monotonic_now()));
      logging = false;
      current_log_namer = nullptr;
    }

    // It is pretty cheap to rename, rename if the name gets "better" than
    // before.
    if (logging) {
      if (log_name_accumulator.Update(joystick_state)) {
        std::string new_base_name = log_name_accumulator.MakeName();
        current_log_namer->set_base_name(new_base_name);

        // Update the -current symlink to point to the new directory
        // FLAGS_logging_folder contains the directory where logs are stored
        // (set by MaybeGetLogName)
        std::string folder = absl::GetFlag(FLAGS_logging_folder);

        // Extract the directory name without the path for the symlink target
        std::filesystem::path new_path(new_base_name);
        std::string target = new_path.parent_path().filename().string();

        // Update the symlink to point to the new directory
        aos::logging::UpdateCurrentSymlink(folder, kLogBaseName, target);
      }
    }
  });

  event_loop.Run();

  LOG(INFO) << "Shutting down";

  return 0;
}
