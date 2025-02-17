#include <fcntl.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <chrono>
#include <csignal>
#include <filesystem>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/containers/resizeable_buffer.h"
#include "aos/events/logging/log_backend.h"
#include "aos/init.h"
#include "aos/realtime.h"
#include "aos/time/time.h"

namespace chrono = std::chrono;

ABSL_FLAG(std::string, file, "/media/sda1/foo", "File to write to.");

ABSL_FLAG(uint32_t, write_size, 4096, "Size of hunk to write");
ABSL_FLAG(bool, cleanup, true, "If true, delete the created file");
ABSL_FLAG(int32_t, nice, -20,
          "Priority to nice to. Set to 0 to not change the priority.");
ABSL_FLAG(bool, sync, false,
          "If true, sync the file after each written block.");
ABSL_FLAG(bool, writev, false, "If true, use writev.");
ABSL_FLAG(bool, direct, false, "If true, O_DIRECT.");
ABSL_FLAG(uint32_t, chunks, 1, "Chunks to write using writev.");
ABSL_FLAG(uint32_t, chunk_size, 512, "Chunk size to write using writev.");
ABSL_FLAG(uint64_t, overall_size, 0,
          "If nonzero, write this many bytes and then stop.  Must be a "
          "multiple of --write_size");
ABSL_FLAG(bool, rate_limit, false,
          "If true, kick off writes every 100ms to mimic logger write "
          "patterns more correctly.");
ABSL_FLAG(double, write_bandwidth, 120.0,
          "Write speed in MB/s to simulate. This is only used when "
          "--rate_limit is specified.");

void trap_sig(int signum) { exit(signum); }

aos::monotonic_clock::time_point start_time = aos::monotonic_clock::min_time;
std::atomic<size_t> written_data = 0;

void Cleanup() {
  LOG(INFO) << "Overall average write speed: "
            << ((written_data) /
                chrono::duration<double>(aos::monotonic_clock::now() -
                                         start_time)
                    .count() /
                1024. / 1024.)
            << " MB/s for " << static_cast<double>(written_data) / 1024. / 1024.
            << "MB";

  // Delete FLAGS_file at shutdown
  PCHECK(std::filesystem::remove(absl::GetFlag(FLAGS_file)) != 0)
      << "Failed to cleanup file";
}

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  // c++ needs bash's trap <fcn> EXIT
  // instead we get this mess :(
  if (absl::GetFlag(FLAGS_cleanup)) {
    std::signal(SIGINT, trap_sig);
    std::signal(SIGTERM, trap_sig);
    std::signal(SIGKILL, trap_sig);
    std::signal(SIGHUP, trap_sig);
    std::atexit(Cleanup);
  }
  aos::AllocatorResizeableBuffer<
      aos::AlignedReallocator<aos::logger::FileHandler::kSector>>
      data;

  {
    // We want uncompressible data.  The easiest way to do this is to grab a
    // good sized block from /dev/random, and then reuse it.
    int random_fd = open("/dev/random", O_RDONLY | O_CLOEXEC);
    PCHECK(random_fd != -1) << ": Failed to open /dev/random";
    data.resize(absl::GetFlag(FLAGS_write_size));
    size_t written = 0;
    while (written < data.size()) {
      const size_t result =
          read(random_fd, data.data() + written, data.size() - written);
      PCHECK(result > 0);
      written += result;
    }

    PCHECK(close(random_fd) == 0);
  }

  std::vector<struct iovec> iovec;
  if (absl::GetFlag(FLAGS_writev)) {
    const size_t chunks = absl::GetFlag(FLAGS_chunks);
    const size_t chunk_size = absl::GetFlag(FLAGS_chunk_size);
    iovec.resize(chunks);
    CHECK_LE(chunks * chunk_size, absl::GetFlag(FLAGS_write_size));

    for (size_t i = 0; i < chunks; ++i) {
      iovec[i].iov_base = &data.at(i * chunk_size);
      iovec[i].iov_len = chunk_size;
    }
    iovec[chunks - 1].iov_base = &data.at((chunks - 1) * chunk_size);
    iovec[chunks - 1].iov_len = data.size() - (chunks - 1) * chunk_size;
  }

  int fd = open(absl::GetFlag(FLAGS_file).c_str(),
                O_RDWR | O_CLOEXEC | O_CREAT |
                    (absl::GetFlag(FLAGS_direct) ? O_DIRECT : 0),
                0774);
  PCHECK(fd != -1);

  start_time = aos::monotonic_clock::now();
  aos::monotonic_clock::time_point last_print_time = start_time;
  aos::monotonic_clock::time_point cycle_start_time = start_time;
  size_t last_written_data = 0;
  written_data = 0;
  // Track how much data we write per cycle. When --rate_limit is specified,
  // --write_bandwidth is the amount of data we want to write per second, and we
  // want to write it in cycles of 100ms to simulate the logger.
  size_t cycle_written_data = 0;
  size_t data_per_cycle = std::numeric_limits<size_t>::max();
  if (absl::GetFlag(FLAGS_rate_limit)) {
    data_per_cycle = static_cast<size_t>(
        (absl::GetFlag(FLAGS_write_bandwidth) * 1024 * 1024) / 10);
  }

  if (absl::GetFlag(FLAGS_nice) != 0) {
    PCHECK(-1 != setpriority(PRIO_PROCESS, 0, absl::GetFlag(FLAGS_nice)))
        << ": Renicing to " << absl::GetFlag(FLAGS_nice) << " failed";
  }

  while (true) {
    // Bail if we have written our limit.
    if (written_data >= absl::GetFlag(FLAGS_overall_size) &&
        absl::GetFlag(FLAGS_overall_size) != 0) {
      break;
    }

    if (absl::GetFlag(FLAGS_writev)) {
      PCHECK(writev(fd, iovec.data(), iovec.size()) ==
             static_cast<ssize_t>(data.size()))
          << ": Failed after "
          << chrono::duration<double>(aos::monotonic_clock::now() - start_time)
                 .count();
    } else {
      PCHECK(write(fd, data.data(), data.size()) ==
             static_cast<ssize_t>(data.size()))
          << ": Failed after "
          << chrono::duration<double>(aos::monotonic_clock::now() - start_time)
                 .count();
    }

    // Trigger a flush if asked.
    if (absl::GetFlag(FLAGS_sync)) {
      const aos::monotonic_clock::time_point monotonic_now =
          aos::monotonic_clock::now();
      sync_file_range(fd, written_data, data.size(), SYNC_FILE_RANGE_WRITE);

      // Now, blocking flush the previous page so we don't get too far ahead.
      // This is Linus' recommendation.
      if (written_data > 0) {
        sync_file_range(fd, written_data - data.size(), data.size(),
                        SYNC_FILE_RANGE_WAIT_BEFORE | SYNC_FILE_RANGE_WRITE |
                            SYNC_FILE_RANGE_WAIT_AFTER);
        posix_fadvise(fd, written_data - data.size(), data.size(),
                      POSIX_FADV_DONTNEED);
      }
      VLOG(1) << "Took "
              << chrono::duration<double>(aos::monotonic_clock::now() -
                                          monotonic_now)
                     .count();
    }

    written_data += data.size();
    cycle_written_data += data.size();

    // Simulate the logger by writing the specified amount of data in periods of
    // 100ms.
    bool reset_cycle = false;
    if (cycle_written_data > data_per_cycle &&
        absl::GetFlag(FLAGS_rate_limit)) {
      // Check how much data we should have already written based on
      // --write_bandwidth.
      const size_t current_target =
          absl::GetFlag(FLAGS_write_bandwidth) * 1024 * 1024 *
          chrono::duration<double>(aos::monotonic_clock::now() - start_time)
              .count();
      const bool caught_up = written_data > current_target;
      if (caught_up) {
        // If we're on track, sleep for the rest of this cycle, as long as we
        // didn't use up all the cycle time writing.
        const aos::monotonic_clock::time_point monotonic_now =
            aos::monotonic_clock::now();
        const auto sleep_duration =
            (cycle_start_time + chrono::milliseconds(100)) - monotonic_now;
        if (sleep_duration.count() > 0) {
          VLOG(2) << "Sleeping for " << sleep_duration.count();
          std::this_thread::sleep_for(sleep_duration);
        } else {
          LOG(WARNING) << "It took longer than 100ms to write "
                       << data_per_cycle << " bytes.";
        }
        reset_cycle = true;
      } else {
        // If we aren't on track, don't sleep.
        LOG(WARNING) << "Still catching up to target write rate.";
      }
      // Either way, reset the data we're counting for this "cycle". If we're
      // still behind, let's check again after writing another data_per_cycle
      // bytes.
      cycle_written_data = 0;
    }

    const aos::monotonic_clock::time_point monotonic_now =
        aos::monotonic_clock::now();
    // Print out MB/s once it has been at least 1 second since last time.
    if (monotonic_now > last_print_time + chrono::seconds(1)) {
      LOG(INFO)
          << ((written_data - last_written_data) /
              chrono::duration<double>(monotonic_now - last_print_time)
                  .count() /
              1024. / 1024.)
          << " MB/s, average of "
          << (written_data /
              chrono::duration<double>(monotonic_now - start_time).count() /
              1024. / 1024.)
          << " MB/s for " << static_cast<double>(written_data) / 1024. / 1024.
          << "MB";
      last_print_time = monotonic_now;
      last_written_data = written_data;
    }

    // Do this at the end so that we're setting the next cycle start time as
    // accurately as possible.
    if (reset_cycle) {
      cycle_start_time = monotonic_now;
      VLOG(1) << cycle_start_time;
    }
  }

  return 0;
}
