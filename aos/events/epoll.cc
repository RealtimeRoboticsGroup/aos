#include "aos/events/epoll.h"

#include "aos/events/aio.h"

namespace aos {

EPoll::EPoll() : owned_aio_(std::make_unique<Aio>()), aio_(owned_aio_.get()) {}

EPoll::EPoll(Aio *aio) : aio_(aio) {}

EPoll::~EPoll() = default;

void EPoll::Run() {
  run_ = true;
  aio_->Run();
}

bool EPoll::Poll(bool block) { return aio_->Poll(block); }

void EPoll::Quit() {
  run_ = false;
  aio_->Quit();
}

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
