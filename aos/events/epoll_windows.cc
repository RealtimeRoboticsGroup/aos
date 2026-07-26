#include "aos/events/epoll.h"

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::internal {

TimerFd::TimerFd() { ABSL_LOG(FATAL) << "TimerFd not implemented on Windows"; }

TimerFd::~TimerFd() {}

void TimerFd::SetTime(monotonic_clock::time_point /*start*/,
                      monotonic_clock::duration /*interval*/) {
  ABSL_LOG(FATAL) << "TimerFd::SetTime not implemented on Windows";
}

uint64_t TimerFd::Read() {
  ABSL_LOG(FATAL) << "TimerFd::Read not implemented on Windows";
  return 0;
}

EPoll::EPoll() { ABSL_LOG(FATAL) << "EPoll not implemented on Windows"; }

EPoll::~EPoll() {}

void EPoll::Run() {
  ABSL_LOG(FATAL) << "EPoll::Run not implemented on Windows";
}

bool EPoll::Poll(bool /*block*/) {
  ABSL_LOG(FATAL) << "EPoll::Poll not implemented on Windows";
  return false;
}

void EPoll::Quit() {
  ABSL_LOG(FATAL) << "EPoll::Quit not implemented on Windows";
}

void EPoll::BeforeWait(std::function<void()> /*function*/) {
  ABSL_LOG(FATAL) << "EPoll::BeforeWait not implemented on Windows";
}

void EPoll::OnReadable(int /*fd*/, ::std::function<void()> /*function*/) {
  ABSL_LOG(FATAL) << "EPoll::OnReadable not implemented on Windows";
}

void EPoll::OnError(int /*fd*/, ::std::function<void()> /*function*/) {
  ABSL_LOG(FATAL) << "EPoll::OnError not implemented on Windows";
}

void EPoll::OnWriteable(int /*fd*/, ::std::function<void()> /*function*/) {
  ABSL_LOG(FATAL) << "EPoll::OnWriteable not implemented on Windows";
}

void EPoll::OnEvents(int /*fd*/, ::std::function<void(uint32_t)> /*function*/) {
  ABSL_LOG(FATAL) << "EPoll::OnEvents not implemented on Windows";
}

void EPoll::DeleteFd(int /*fd*/) {
  ABSL_LOG(FATAL) << "EPoll::DeleteFd not implemented on Windows";
}

void EPoll::ForgetClosedFd(int /*fd*/) {
  ABSL_LOG(FATAL) << "EPoll::ForgetClosedFd not implemented on Windows";
}

void EPoll::SetEvents(int /*fd*/, uint32_t /*events*/) {
  ABSL_LOG(FATAL) << "EPoll::SetEvents not implemented on Windows";
}

void EPoll::InOutEventData::DoCallbacks(uint32_t /*events*/) {
  ABSL_LOG(FATAL)
      << "EPoll::InOutEventData::DoCallbacks not implemented on Windows";
}

void EPoll::EnableEvents(int /*fd*/, uint32_t /*events*/) {
  ABSL_LOG(FATAL) << "EPoll::EnableEvents not implemented on Windows";
}

void EPoll::DisableEvents(int /*fd*/, uint32_t /*events*/) {
  ABSL_LOG(FATAL) << "EPoll::DisableEvents not implemented on Windows";
}

EPoll::EventData *EPoll::GetEventData(int /*fd*/) {
  ABSL_LOG(FATAL) << "EPoll::GetEventData not implemented on Windows";
  return nullptr;
}

void EPoll::DoEpollCtl(EventData * /*event_data*/, uint32_t /*new_events*/) {
  ABSL_LOG(FATAL) << "EPoll::DoEpollCtl not implemented on Windows";
}

void EPoll::DeleteFdFromEpoll(int /*fd*/) {
  ABSL_LOG(FATAL) << "EPoll::DeleteFdFromEpoll not implemented on Windows";
}

}  // namespace aos::internal
