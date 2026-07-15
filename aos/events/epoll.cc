#include "aos/events/epoll.h"

#include "aos/events/aio.h"

namespace aos {

EPoll::EPoll() : aio_(std::make_unique<Aio>()) {}

EPoll::~EPoll() = default;

void EPoll::Run() { aio_->Run(); }

// The Aio's answer rather than a flag of our own: EPoll is a shim over Aio
// now, and a second copy of "are we running" would be a second thing to keep
// in step.  It would also be subtly wrong -- Aio's should_run() folds in a
// Quit() that landed concurrently with Run() (quit_requested), which a plain
// bool set in Run()/Quit() cannot see.
bool EPoll::should_run() const { return aio_->should_run(); }

bool EPoll::Poll(bool block) { return aio_->Poll(block); }

void EPoll::Quit() { aio_->Quit(); }

void EPoll::BeforeWait(std::function<void()> function) {
  aio_->BeforeWait(std::move(function));
}

void EPoll::OnReadable(int fd, ::std::function<void()> function) {
  aio_->OnReadable(fd, std::move(function));
}

void EPoll::OnError(int fd, ::std::function<void()> function) {
  aio_->OnError(fd, std::move(function));
}

void EPoll::OnWritable(int fd, ::std::function<void()> function) {
  aio_->OnWritable(fd, std::move(function));
}

void EPoll::OnEvents(int fd, ::std::function<void(uint32_t)> function) {
  aio_->OnEvents(fd, std::move(function));
}

void EPoll::DeleteFd(int fd) { aio_->DeleteFd(fd); }

void EPoll::ForgetClosedFd(int fd) { aio_->ForgetClosedFd(fd); }

void EPoll::EnableWritable(int fd) { aio_->EnableWritable(fd); }

void EPoll::DisableWritable(int fd) { aio_->DisableWritable(fd); }

void EPoll::SetEvents(int fd, uint32_t events) { aio_->SetEvents(fd, events); }

}  // namespace aos
