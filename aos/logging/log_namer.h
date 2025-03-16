#ifndef AOS_LOGGING_LOG_NAMER_H_
#define AOS_LOGGING_LOG_NAMER_H_

#include <optional>
#include <string>
#include <string_view>

namespace aos::logging {
// Returns the correct filename to log to, blocking until the usb drive
// filesystem mounts, incrementing the number on the end of the filename, and
// setting up a symlink at basename-current.
//
// basename is the prefix to use for the logs within the usb drive. E.g., on a
// typical roborio setup, calling GetLogName("abc") will return a filename of
// the form "/media/sda1/abc-123" and setup a symlink pointing to it at
// "/media/sda1/abc-current".
//
// This function sets FLAGS_logging_folder to the directory where logs are
// stored (e.g., "/media/sda1/"). This flag can then be used by other functions
// that need to know where logs are stored.
std::string GetLogName(const char *basename);

// A nonblocking variant of GetLogName that you can poll instead of blocking for
// the usb drive.
std::optional<std::string> MaybeGetLogName(const char *basename);

// Updates the symlink at basename-current to point to the target directory.
// This is useful when the log directory is renamed after initial creation.
// folder is the parent directory where the symlink will be created.
// basename is the prefix used for the logs.
// target is the directory name that the symlink should point to.
void UpdateCurrentSymlink(std::string_view folder, std::string_view basename,
                          std::string_view target);

}  // namespace aos::logging

#endif  // AOS_LOGGING_LOG_NAMER_H_
