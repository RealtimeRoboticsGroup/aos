#include "aos/events/epoll.h"

#include <sys/socket.h>
#include <sys/stat.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

namespace aos::internal {

void EPoll::BeforeWait(std::function<void()> function) {
  before_epoll_wait_functions_.emplace_back(std::move(function));
}

void EPoll::Run() {
  run_ = true;
  // As long as run_ is true or we still have events to process, keep polling.
  while (true) {
    // If we ran out of events and Quit() was called, quit
    if (!Poll(run_)) {
      if (!run_) {
        return;
      }
    }
  }
}

void EPoll::Quit() {
  // Shortcut to break us out of infinite loops. We might write more than once
  // to the pipe, but we'll stop once the first is read on the other end.
  if (!run_) {
    return;
  }
  ABSL_PCHECK(write(quit_signal_fd_, "q", 1) == 1);
}

void EPoll::OnReadable(int fd, ::std::function<void()> function) {
  EventData *event_data = GetEventData(fd);
  if (event_data == nullptr) {
    fns_.emplace_back(std::make_unique<InOutEventData>(fd));
    event_data = fns_.back().get();
  } else {
    ABSL_CHECK(!static_cast<InOutEventData *>(event_data)->in_fn)
        << ": Duplicate in functions for " << fd;
  }
  static_cast<InOutEventData *>(event_data)->in_fn = ::std::move(function);
  DoEpollCtl(event_data, event_data->events | kInEvents);
}

void EPoll::OnError(int fd, ::std::function<void()> function) {
  EventData *event_data = GetEventData(fd);
  if (event_data == nullptr) {
    fns_.emplace_back(std::make_unique<InOutEventData>(fd));
    event_data = fns_.back().get();
  } else {
    ABSL_CHECK(!static_cast<InOutEventData *>(event_data)->err_fn)
        << ": Duplicate error functions for " << fd;
  }
  static_cast<InOutEventData *>(event_data)->err_fn = ::std::move(function);
  DoEpollCtl(event_data, event_data->events | kErrorEvents);
}

void EPoll::OnWritable(int fd, ::std::function<void()> function) {
  EventData *event_data = GetEventData(fd);
  if (event_data == nullptr) {
    fns_.emplace_back(std::make_unique<InOutEventData>(fd));
    event_data = fns_.back().get();
  } else {
    ABSL_CHECK(!static_cast<InOutEventData *>(event_data)->out_fn)
        << ": Duplicate out functions for " << fd;
  }
  static_cast<InOutEventData *>(event_data)->out_fn = ::std::move(function);
  DoEpollCtl(event_data, event_data->events | kOutEvents);
}

void EPoll::OnEvents(int fd, ::std::function<void(uint32_t)> function) {
  if (GetEventData(fd) != nullptr) {
    ABSL_LOG(FATAL) << "May not replace OnEvents handlers";
  }
  fns_.emplace_back(std::make_unique<SingleEventData>(fd));
  static_cast<SingleEventData *>(fns_.back().get())->fn = std::move(function);
}

void EPoll::ForgetClosedFd(int fd) {
  auto element = fns_.begin();
  while (fns_.end() != element) {
    if (element->get()->fd == fd) {
      fns_.erase(element);
      return;
    }
    ++element;
  }
  ABSL_LOG(FATAL) << "fd " << fd << " not found";
}

void EPoll::SetEvents(int fd, uint32_t events) {
  EventData *event_data;
  ABSL_CHECK((event_data = GetEventData(fd)) != nullptr);
  DoEpollCtl(event_data, events);
}

// Removes fd from the event loop.
void EPoll::DeleteFd(int fd) {
  auto element = fns_.begin();
  while (fns_.end() != element) {
    if (element->get()->fd == fd) {
      DoEpollCtl(element->get(), 0);
      fns_.erase(element);
      return;
    }
    ++element;
  }
  ABSL_LOG(FATAL) << "fd " << fd << " not found";
}

namespace {
bool IsSocket(int fd) {
  struct stat st;
  if (fstat(fd, &st) == -1) {
    return false;
  }
  return static_cast<bool>(S_ISSOCK(st.st_mode));
}

::std::string GetSocketErrorStr(int fd) {
  ::std::string error_str;
  if (IsSocket(fd)) {
    int error = 0;
    socklen_t errlen = sizeof(error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (void *)&error, &errlen) == 0) {
      if (error) {
        error_str = "Socket error: " + ::std::string(strerror(error));
      }
    }
  }
  return error_str;
}
}  // namespace

void EPoll::InOutEventData::DoCallbacks(uint32_t events) {
  if (events & kInEvents) {
    ABSL_CHECK(in_fn)
        << ": No handler registered for input events on descriptor " << fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    in_fn();
  }
  if (events & kOutEvents) {
    ABSL_CHECK(out_fn)
        << ": No handler registered for output events on descriptor " << fd
        << ". Received events = 0x" << std::hex << events << std::dec;
    out_fn();
  }
  if (events & kErrorEvents) {
    ABSL_CHECK(err_fn)
        << ": No handler registered for error events on descriptor " << fd
        << ". Received events = 0x" << std::hex << events << std::dec << ". "
        << GetSocketErrorStr(fd);
    err_fn();
  }
}

void EPoll::EnableEvents(int fd, uint32_t events) {
  EventData *const event_data = GetEventData(fd);
  ABSL_CHECK(event_data != nullptr);
  DoEpollCtl(event_data, event_data->events | events);
}

void EPoll::DisableEvents(int fd, uint32_t events) {
  EventData *const event_data = GetEventData(fd);
  ABSL_CHECK(event_data != nullptr);
  DoEpollCtl(event_data, event_data->events & ~events);
}

EPoll::EventData *EPoll::GetEventData(int fd) {
  const auto iterator = std::find_if(
      fns_.begin(), fns_.end(),
      [fd](const std::unique_ptr<EventData> &data) { return data->fd == fd; });
  if (iterator == fns_.end()) {
    return nullptr;
  }
  return iterator->get();
}

}  // namespace aos::internal
