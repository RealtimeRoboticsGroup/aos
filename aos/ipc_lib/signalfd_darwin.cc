#include <signal.h>

#include <cstring>

#include "absl/log/absl_check.h"

#include "aos/ipc_lib/signalfd.h"

namespace aos::ipc_lib {

SignalFd::SignalFd(::std::initializer_list<unsigned int> signal_list) {
  signals_.reserve(signal_list.size());
  for (unsigned int signal : signal_list) {
    SignalState state;
    state.signal = signal;

    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;

    if (sigaction(signal, &sa, &state.old_sa) == 0) {
      signals_.push_back(state);
    }
  }
}

SignalFd::~SignalFd() {
  for (const auto &state : signals_) {
    if (state.restore) {
      sigaction(state.signal, &state.old_sa, nullptr);
    }
  }
}

void SignalFd::LeaveSignalBlocked(unsigned int signal) {
  for (auto &state : signals_) {
    if (state.signal == signal) {
      state.restore = false;
    }
  }
}

}  // namespace aos::ipc_lib
