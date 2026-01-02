#include "aos/ipc_lib/lockless_queue_test_utils.h"

#include "absl/flags/flag.h"

#include "aos/ipc_lib/signalfd.h"

ABSL_FLAG(uint32_t, start_core_index, 0, "The core to start pinning on");

namespace aos::ipc_lib::testing {

LocklessQueueTest::PinForTest::PinForTest() {
  CpuSet cpus = GetCurrentThreadAffinity();
  old_ = cpus;
  int number_found = 0;
  for (int i = 0; i < static_cast<int>(CpuSet::kSize); ++i) {
    // We don't want to exclude cores, but start at a different spot in the core
    // index.  This makes it so on a box with a reasonable set of cores
    // available, the test variants won't all end up on core's 0 and 1.
    const int actual_i =
        (i + absl::GetFlag(FLAGS_start_core_index)) % CpuSet::kSize;
    if (cpus.IsSet(actual_i)) {
      if (number_found < 2) {
        ++number_found;
      } else {
        cpus.Clear(actual_i);
      }
    }
  }
  SetCurrentThreadAffinity(cpus);
}

LocklessQueueTest::LocklessQueueTest() {
  config_.num_watchers = 10;
  config_.num_senders = 100;
  config_.num_pinners = 5;
  config_.queue_size = 10000;
  // Exercise the alignment code.  This would throw off alignment.
  config_.message_data_size = 101;

  // Since our backing store is an array of uint64_t for alignment purposes,
  // normalize by the size.
  memory_.resize(LocklessQueueMemorySize(config_) / sizeof(uint64_t));

  Reset();
}

void LocklessQueueTest::RunUntilWakeup(Event *ready, int priority) {
  internal::EPoll epoll;
  SignalFd signalfd({kWakeupSignal});

  epoll.OnReadable(signalfd.fd(), [&signalfd, &epoll]() {
    signalfd_siginfo result = signalfd.Read();

    fprintf(stderr, "Got signal: %d\n", result.ssi_signo);
    epoll.Quit();
  });

  {
    // Register to be woken up *after* the signalfd is catching the signals.
    LocklessQueueWatcher watcher =
        LocklessQueueWatcher::Make(queue(), priority).value();

    // And signal we are now ready.
    ready->Set();

    epoll.Run();

    // Cleanup, ensuring the watcher is destroyed before the signalfd.
  }
  epoll.DeleteFd(signalfd.fd());
}
}  // namespace aos::ipc_lib::testing
