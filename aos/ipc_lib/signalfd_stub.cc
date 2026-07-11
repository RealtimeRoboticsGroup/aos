#include "aos/ipc_lib/signalfd.h"

#if !defined(__linux__) && !defined(__APPLE__)
namespace aos::ipc_lib {

SignalFd::SignalFd(::std::initializer_list<unsigned int> /*signal_list*/) {}
SignalFd::~SignalFd() {}

void SignalFd::LeaveSignalBlocked(unsigned int /*signal*/) {}

}  // namespace aos::ipc_lib
#endif
