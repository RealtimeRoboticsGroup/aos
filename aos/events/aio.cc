#include "aos/events/aio.h"

#include "aos/events/aio_internal.h"

namespace aos {

Aio::~Aio() = default;

void Aio::Run() { impl_->Run(); }

bool Aio::Poll(bool block) { return impl_->Poll(block); }

void Aio::Quit() { impl_->Quit(); }

bool Aio::should_run() const { return impl_->should_run(); }

void Aio::AsyncRead(int fd, std::span<char> buffer, AsyncRequest *request) {
  impl_->AsyncRead(fd, buffer, request);
}

void Aio::AsyncWrite(int fd, std::span<const char> buffer,
                     AsyncRequest *request) {
  impl_->AsyncWrite(fd, buffer, request);
}

void Aio::Cancel(AsyncRequest *request) { impl_->Cancel(request); }

void Aio::BeforeWait(std::function<void()> function) {
  impl_->BeforeWait(std::move(function));
}

void Aio::OnReadable(int fd, std::function<void()> callback) {
  impl_->OnReadable(fd, std::move(callback));
}

void Aio::OnError(int fd, std::function<void()> callback) {
  impl_->OnError(fd, std::move(callback));
}

void Aio::OnWritable(int fd, std::function<void()> callback) {
  impl_->OnWritable(fd, std::move(callback));
}

void Aio::OnEvents(int fd, std::function<void(uint32_t)> callback) {
  impl_->OnEvents(fd, std::move(callback));
}

void Aio::DeleteFd(int fd) { impl_->DeleteFd(fd); }

void Aio::ForgetClosedFd(int fd) { impl_->ForgetClosedFd(fd); }

void Aio::EnableWritable(int fd) { impl_->EnableWritable(fd); }

void Aio::DisableWritable(int fd) { impl_->DisableWritable(fd); }

void Aio::SetEvents(int fd, uint32_t events) { impl_->SetEvents(fd, events); }

void Aio::RegisterSignalFd(ipc_lib::SignalFd *sfd,
                           std::function<void()> callback) {
  impl_->RegisterSignalFd(sfd, std::move(callback));
}

void Aio::UnregisterSignalFd(ipc_lib::SignalFd *sfd) {
  impl_->UnregisterSignalFd(sfd);
}

void Aio::ConsumeSignalFd(ipc_lib::SignalFd *sfd) {
  impl_->ConsumeSignalFd(sfd);
}

Aio::Timer::Timer(Aio *aio) : state_(aio->impl_->MakeTimerState()) {
  state_->aio = aio;
  state_->Initialize();
}

Aio::Timer::~Timer() = default;

void Aio::Timer::Schedule(aos::monotonic_clock::time_point deadline,
                          aos::monotonic_clock::duration interval,
                          CompletionCallback callback, void *context) {
  state_->Schedule(deadline, interval, callback, context);
}

void Aio::Timer::Cancel() { state_->Cancel(false); }

}  // namespace aos
