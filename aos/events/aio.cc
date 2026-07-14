#include "aos/events/aio.h"

#include "aos/events/aio_internal.h"

namespace aos {

Aio::~Aio() = default;

void Aio::Run() { impl_->Run(); }

bool Aio::Poll(bool block) { return impl_->Poll(block); }

void Aio::Quit() { impl_->Quit(); }

void Aio::AsyncRead(FileDescriptor fd, std::span<char> buffer,
                    AsyncRequest *request) {
  impl_->AsyncRead(fd, buffer, request);
}

void Aio::AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                     AsyncRequest *request) {
  impl_->AsyncWrite(fd, buffer, request);
}

void Aio::Cancel(AsyncRequest *request) { impl_->Cancel(request); }

void Aio::BeforeWait(std::function<void()> function) {
  impl_->BeforeWait(std::move(function));
}

void Aio::OnReadable(FileDescriptor fd, std::function<void()> callback) {
  impl_->OnReadable(fd, std::move(callback));
}

void Aio::OnError(FileDescriptor fd, std::function<void()> callback) {
  impl_->OnError(fd, std::move(callback));
}

void Aio::OnWritable(FileDescriptor fd, std::function<void()> callback) {
  impl_->OnWritable(fd, std::move(callback));
}

void Aio::OnEvents(FileDescriptor fd, std::function<void(uint32_t)> callback) {
  impl_->OnEvents(fd, std::move(callback));
}

void Aio::DeleteFd(FileDescriptor fd) { impl_->DeleteFd(fd); }

void Aio::ForgetClosedFd(FileDescriptor fd) { impl_->ForgetClosedFd(fd); }

void Aio::EnableWritable(FileDescriptor fd) { impl_->EnableWritable(fd); }

void Aio::DisableWritable(FileDescriptor fd) { impl_->DisableWritable(fd); }

void Aio::SetEvents(FileDescriptor fd, uint32_t events) {
  impl_->SetEvents(fd, events);
}

void Aio::RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                       std::function<void()> callback) {
  impl_->RegisterThreadSignalReceiver(receiver, std::move(callback));
}

void Aio::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  impl_->UnregisterThreadSignalReceiver(receiver);
}

void Aio::ConsumeThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver) {
  impl_->ConsumeThreadSignalReceiver(receiver);
}

Aio::Timer::Timer(Aio *aio) : state_(aio->impl_->MakeTimerState()) {
  state_->aio = aio;
  state_->Initialize();
}

Aio::Timer::~Timer() {
  if (state_ != nullptr) {
    Aio *aio = state_->aio;
    aio->impl_->DestroyTimerState(std::move(state_));
  }
}

void Aio::Timer::Schedule(aos::monotonic_clock::time_point deadline,
                          CompletionCallback callback, void *context) {
  state_->Schedule(deadline, callback, context);
}

void Aio::Timer::Cancel() { state_->Cancel(false); }

}  // namespace aos
