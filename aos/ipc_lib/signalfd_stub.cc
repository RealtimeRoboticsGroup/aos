#include "aos/ipc_lib/signalfd.h"

namespace aos::ipc_lib {

#ifndef __linux__
SignalFd::SignalFd(::std::initializer_list<unsigned int> /*signal_list*/) {}
SignalFd::~SignalFd() {}
#endif

}  // namespace aos::ipc_lib
