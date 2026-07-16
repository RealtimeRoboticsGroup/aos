#ifndef AOS_EVENTS_AIO_INTERNAL_H_
#define AOS_EVENTS_AIO_INTERNAL_H_

#include <functional>
#include <memory>
#include <span>

#include "aos/events/aio.h"

namespace aos {

struct Aio::TimerState {
  virtual ~TimerState() = default;

  virtual void Initialize() = 0;
  virtual void Schedule(aos::monotonic_clock::time_point deadline,
                        aos::monotonic_clock::duration interval,
                        CompletionCallback callback, void *context) = 0;
  virtual void Cancel(bool reap) = 0;

  Aio *aio = nullptr;
  AsyncRequest request;
  aos::monotonic_clock::duration interval = aos::monotonic_clock::zero();
  aos::monotonic_clock::time_point deadline = aos::monotonic_clock::epoch();
  CompletionCallback user_callback = nullptr;
  void *user_context = nullptr;

  // Intrusive doubly-linked list pointers for active timers
  Aio::TimerState *prev_active = nullptr;
  Aio::TimerState *next_active = nullptr;
  bool is_active = false;
};

struct Aio::Impl {
  virtual ~Impl() = default;

  virtual std::unique_ptr<Aio::TimerState> MakeTimerState() = 0;

  virtual void Run() = 0;
  virtual bool should_run() const = 0;
  virtual bool Poll(bool block) = 0;
  virtual void Quit() = 0;

  virtual void AsyncRead(int fd, std::span<char> buffer,
                         AsyncRequest *request) = 0;
  virtual void AsyncWrite(int fd, std::span<const char> buffer,
                          AsyncRequest *request) = 0;
  virtual void Cancel(AsyncRequest *request) = 0;
  virtual void BeforeWait(std::function<void()> function) = 0;

  // Legacy Readiness Hooks.
  virtual void OnReadable(int fd, std::function<void()> callback) = 0;
  virtual void OnError(int fd, std::function<void()> callback) = 0;
  virtual void OnWritable(int fd, std::function<void()> callback) = 0;
  virtual void OnEvents(int fd, std::function<void(uint32_t)> callback) = 0;
  virtual void DeleteFd(int fd) = 0;
  virtual void ForgetClosedFd(int fd) = 0;
  virtual void EnableWritable(int fd) = 0;
  virtual void DisableWritable(int fd) = 0;
  virtual void SetEvents(int fd, uint32_t events) = 0;

  // Registers a SignalFd for wakeups.
  virtual void RegisterSignalFd(ipc_lib::SignalFd *sfd,
                                std::function<void()> callback) = 0;
  virtual void UnregisterSignalFd(ipc_lib::SignalFd *sfd) = 0;
  virtual void ConsumeSignalFd(ipc_lib::SignalFd *sfd) = 0;
};

}  // namespace aos

#endif  // AOS_EVENTS_AIO_INTERNAL_H_
