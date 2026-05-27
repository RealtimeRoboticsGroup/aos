#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/escaping.h"

#include "aos/configuration.h"
#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "aos/sha256.h"
#include "aos/util/file.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");

ABSL_FLAG(std::string, path, "/tmp/images/", "The directory to write into");
ABSL_FLAG(std::string, channel, "/camera0", "The channel to save images from");

absl::optional<int> CameraNumber(absl::string_view sv) {
  // Iterate forward through the string view
  for (char c : sv) {
    if (std::isdigit(c)) {
      // Found the first digit, return it immediately
      return c - '0';
    }
  }
  // No digit was found after checking the entire string
  return absl::nullopt;
}

class Nonblocking {
 public:
  Nonblocking(int fd) {
    fd_ = fd;
    int flags_ = fcntl(fd_, F_GETFL, 0);
    PCHECK(flags_ != -1);
    PCHECK(fcntl(fd_, F_SETFL, flags_ | O_NONBLOCK) != -1);
  }

  ~Nonblocking() { PCHECK(fcntl(fd_, F_SETFL, flags_) != -1); }

 private:
  int fd_;
  int flags_;
};

class TerminalRawMode {
 public:
  explicit TerminalRawMode(int fd) : fd_(fd) {
    // Check if the file descriptor is associated with a terminal
    CHECK(isatty(fd_)) << ": Only supported on terminals.";

    // Get current terminal attributes
    PCHECK(tcgetattr(fd_, &original_termios_) != -1);

    // Copy attributes and modify for raw mode
    struct termios raw = original_termios_;
    // ICANON: Disable canonical mode (line buffering)
    // ECHO: Disable echoing typed characters back to the terminal
    raw.c_lflag &= ~(ICANON | ECHO);

    // Set VMIN and VTIME for character-by-character input
    raw.c_cc[VMIN] = 1;   // Read returns after 1 character is available
    raw.c_cc[VTIME] = 0;  // No timeout (wait indefinitely for a character)

    // Apply the modified attributes immediately
    CHECK(tcsetattr(fd_, TCSAFLUSH, &raw) != -1);
  }

  ~TerminalRawMode() {
    PCHECK(tcsetattr(fd_, TCSAFLUSH, &original_termios_) != -1);
  }

  TerminalRawMode(const TerminalRawMode &) = delete;
  TerminalRawMode &operator=(const TerminalRawMode &) = delete;
  TerminalRawMode(TerminalRawMode &&) = delete;
  TerminalRawMode &operator=(TerminalRawMode &&) = delete;

 private:
  int fd_;                           // The file descriptor being managed
  struct termios original_termios_;  // Stores the original terminal settings
};

class ImageDump {
 public:
  ImageDump(aos::ShmEventLoop *event_loop)
      : event_loop_(event_loop),
        image_fetcher_(event_loop_->MakeFetcher<frc::vision::CameraImage>(
            absl::GetFlag(FLAGS_channel))),
        camera_number_(CameraNumber(absl::GetFlag(FLAGS_channel)).value()),
        raw_(STDIN_FILENO),
        nonblocking_(STDIN_FILENO) {
    event_loop_->epoll()->OnReadable(STDIN_FILENO, [this]() {
      ssize_t bytes_read;
      char read_buffer[1];
      while (true) {
        bytes_read = read(STDIN_FILENO, read_buffer, sizeof(read_buffer));

        if (bytes_read == -1) {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
          }
          PCHECK(bytes_read != -1);
        } else if (bytes_read == 0) {
          // End Of File (EOF) detected on stdin (e.g., Ctrl+D pressed)
          event_loop_->Exit();
          return;
        }

        // Process the characters read
        char terminated_buffer[2];
        terminated_buffer[0] = read_buffer[0];
        terminated_buffer[1] = '\0';
        switch (read_buffer[0]) {
          case 'q':
          case 'Q':
            event_loop_->Exit();
            return;
          case 's':
          case 'S':
          case 'w':
          case 'W':
          case ' ':
            MaybeLogLastImage();
            break;
          default:
            LOG(INFO) << "Unhandled character '"
                      << absl::CEscape(terminated_buffer) << "'";
            break;
        }
      }
    });
  }

  ~ImageDump() { event_loop_->epoll()->DeleteFd(STDIN_FILENO); }

  void MaybeLogLastImage() {
    if (!image_fetcher_.Fetch()) {
      LOG(ERROR) << "No new images";
      return;
    }
    LogImage(*image_fetcher_);
  }

  void LogImage(const frc::vision::CameraImage &image) {
    CHECK(image.format() == frc::vision::ImageFormat::MJPEG);
    CHECK(image.has_data());
    std::string_view image_data(
        reinterpret_cast<const char *>(image.data()->data()),
        image.data()->size());
    std::string sha256 = aos::Sha256(image_data);

    std::string path =
        absl::StrCat(absl::GetFlag(FLAGS_path), "/", sha256.substr(0, 2), "/",
                     sha256, "-", camera_number_, ".jpg");
    LOG(INFO) << "Writing " << path;

    CHECK(aos::util::MkdirPIfSpace(path, 0755));
    aos::util::WriteStringToFileOrDie(path, image_data);
  }

 private:
  aos::ShmEventLoop *event_loop_;
  aos::Fetcher<frc::vision::CameraImage> image_fetcher_;
  int camera_number_;

  TerminalRawMode raw_;
  Nonblocking nonblocking_;
};

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  aos::ShmEventLoop event_loop(&config.message());

  ImageDump image_dump(&event_loop);

  LOG(INFO) << "Logging images from " << absl::GetFlag(FLAGS_channel);
  LOG(INFO) << "Press 's' to save an image";

  event_loop.Run();

  LOG(INFO) << "Exiting";
}
