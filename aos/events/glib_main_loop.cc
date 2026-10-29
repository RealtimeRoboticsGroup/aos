#include "aos/events/glib_main_loop.h"

#include <fcntl.h>

#include <algorithm>
#include <cerrno>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"

#include "aos/events/shm_event_loop.h"

namespace aos {
namespace {

// Aio delivers the four bits named in file_descriptor.h and nothing else, a
// hangup included: it arrives as kErrorEvent.
gint AioToGio(uint32_t events) {
  gint result = 0;
  if (events & kReadableEvent) {
    result |= G_IO_IN;
    events &= ~kReadableEvent;
  }
  if (events & kWritableEvent) {
    result |= G_IO_OUT;
    events &= ~kWritableEvent;
  }
  if (events & kPriorityEvent) {
    result |= G_IO_PRI;
    events &= ~kPriorityEvent;
  }
  if (events & kErrorEvent) {
    result |= G_IO_ERR;
    events &= ~kErrorEvent;
  }
  ABSL_CHECK_EQ(events, 0u) << ": Unhandled Aio event bits";
  return result;
}

// G_IO_HUP and G_IO_ERR both ask for kErrorEvent, the bit Aio reports either
// on.  A mask of nothing but them still keeps the fd registered, where a mask
// of 0 would remove it.
uint32_t GioToAio(gint gio) {
  uint32_t result = 0;
  if (gio & G_IO_IN) {
    result |= kReadableEvent;
    gio &= ~G_IO_IN;
  }
  if (gio & G_IO_OUT) {
    result |= kWritableEvent;
    gio &= ~G_IO_OUT;
  }
  if (gio & G_IO_PRI) {
    result |= kPriorityEvent;
    gio &= ~G_IO_PRI;
  }
  if (gio & (G_IO_HUP | G_IO_ERR)) {
    result |= kErrorEvent;
    gio &= ~(G_IO_HUP | G_IO_ERR);
  }
  ABSL_CHECK_EQ(gio, 0) << ": Unhandled gio bits";
  return result;
}

}  // namespace

GlibMainLoop::GlibMainLoop(ShmEventLoop *event_loop)
    : GlibMainLoop(event_loop, event_loop->aio(),
                   [event_loop]() { event_loop->Exit(); }) {}

GlibMainLoop::GlibMainLoop(EventLoop *event_loop, Aio *aio,
                           std::function<void()> exit_handler)
    : event_loop_(event_loop),
      aio_(aio),
      exit_handler_(std::move(exit_handler)),
      timeout_timer_(event_loop->AddTimer([]() {
        // Don't need to do anything, just need to get the event loop to break
        // out of the kernel and call BeforeWait again.
      })),
      g_main_context_(g_main_context_ref(g_main_context_default())),
      g_main_loop_(g_main_loop_new(g_main_context_, true)) {
  event_loop_->OnRun([this]() {
    ABSL_CHECK(!acquired_context_);
    ABSL_CHECK(g_main_context_acquire(g_main_context_))
        << ": The EventLoop thread must own the context";
    acquired_context_ = true;
  });
  before_wait_hook_ = aio_->BeforeWait([this]() { BeforeWait(); });
}

GlibMainLoop::~GlibMainLoop() {
  ABSL_CHECK_EQ(children_, 0) << ": Failed to destroy all children";
  aio_->RemoveBeforeWait(before_wait_hook_);
  RemoveAllFds();
  if (acquired_context_) {
    g_main_context_release(g_main_context_);
  }
  g_main_loop_unref(g_main_loop_);
  g_main_context_unref(g_main_context_);
}

void GlibMainLoop::RemoveAllFds() {
  while (true) {
    const auto to_remove = added_fds_.begin();
    if (to_remove == added_fds_.end()) {
      break;
    }
    aio_->DeleteFd(*to_remove);
    added_fds_.erase(to_remove);
  }
}

void GlibMainLoop::BeforeWait() {
  if (!g_main_loop_is_running(g_main_loop_)) {
    // glib will never quiesce its FDs, so the best we can do is just skip it
    // once it's done and shut down our event loop. We have to remove all of its
    // FDs first so other event sources can quiesce.
    ABSL_VLOG(1) << "g_main_loop_is_running = false";
    RemoveAllFds();
    exit_handler_();
    return;
  }
  if (!aio_->should_run()) {
    // Give glib one more round of dispatching.
    ABSL_VLOG(1) << "Aio::should_run = false";
    g_main_loop_quit(g_main_loop_);
  }

  if (!gpoll_fds_.empty()) {
    // Tell glib about any events we received on the FDs it asked about.
    if (g_main_context_check(g_main_context_, last_query_max_priority_,
                             gpoll_fds_.data(), gpoll_fds_.size())) {
      ABSL_VLOG(1) << "g_main_context_dispatch";
      // We have some glib events now, dispatch them now.
      g_main_context_dispatch(g_main_context_);
    }
  }

  // Call prepare to check for any other events that are ready to be dispatched.
  // g_main_context_iterate ignores the return value, so we're going to do that
  // too.
  g_main_context_prepare(g_main_context_, &last_query_max_priority_);

  gint timeout_ms;
  while (true) {
    const gint number_new_fds =
        g_main_context_query(g_main_context_, last_query_max_priority_,
                             &timeout_ms, gpoll_fds_.data(), gpoll_fds_.size());
    if (static_cast<size_t>(number_new_fds) <= gpoll_fds_.size()) {
      // They all fit, resize to drop any stale entries and then we're done.
      gpoll_fds_.resize(number_new_fds);
      ABSL_VLOG(1) << "glib gave " << number_new_fds;
      break;
    }
    // Need more space, we know how much so try again.
    gpoll_fds_.resize(number_new_fds);
  }

  for (GPollFD gpoll_fd : gpoll_fds_) {
    // API docs are a bit unclear, but this shouldn't ever happen I think?
    ABSL_CHECK_EQ(gpoll_fd.revents, 0) << ": what does this mean?";

    if (added_fds_.count(gpoll_fd.fd) == 0) {
      ABSL_VLOG(1) << "Add to ShmEventLoop: " << gpoll_fd.fd;
      aio_->OnEvents(gpoll_fd.fd, [this, fd = gpoll_fd.fd](uint32_t events) {
        ABSL_VLOG(1) << "glib " << fd << " triggered: " << std::hex << events;
        const auto iterator = std::find_if(
            gpoll_fds_.begin(), gpoll_fds_.end(),
            [fd](const GPollFD &candidate) { return candidate.fd == fd; });
        ABSL_CHECK(iterator != gpoll_fds_.end())
            << ": Lost GPollFD for " << fd
            << " but still registered with the Aio";
        iterator->revents |= AioToGio(events);
      });
      added_fds_.insert(gpoll_fd.fd);
    }
    aio_->SetEvents(gpoll_fd.fd, GioToAio(gpoll_fd.events));
  }
  for (auto added = added_fds_.begin(); added != added_fds_.end();) {
    const FileDescriptor fd = *added;
    if (std::any_of(
            gpoll_fds_.begin(), gpoll_fds_.end(),
            [fd](const GPollFD &candidate) { return candidate.fd == fd; })) {
      ++added;
      continue;
    }
    ABSL_VLOG(1) << "Remove from ShmEventLoop: " << fd;
    // glib has stopped asking about fd, and whoever owned it may have closed
    // it since, which poll(2) never minds.  The Aio does, so it is told
    // which.  A number that was closed and already reused by the time this
    // runs looks open, and DeleteFd() of it dies.
    if (fcntl(fd, F_GETFD) == -1 && errno == EBADF) {
      aio_->ForgetClosedFd(fd);
    } else {
      aio_->DeleteFd(fd);
    }
    added = added_fds_.erase(added);
  }
  ABSL_CHECK_EQ(added_fds_.size(), gpoll_fds_.size());
  ABSL_VLOG(1) << "Timeout: " << timeout_ms;
  if (timeout_ms == -1) {
    timeout_timer_->Disable();
  } else {
    timeout_timer_->Schedule(event_loop_->monotonic_now() +
                             std::chrono::milliseconds(timeout_ms));
  }
}

}  // namespace aos
