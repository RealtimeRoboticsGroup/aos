#include "aos/events/aio.h"

#include "absl/log/absl_check.h"

#include "aos/events/aio_internal.h"
#include "aos/realtime.h"

namespace aos {

void Aio::Impl::CheckNoRawRequestsInFlightOnFork() const {
  ABSL_CHECK(!HasRawRequestsInFlight())
      << ": a forked child touched an Aio with caller-submitted "
         "AsyncRead/AsyncWrite requests in flight at the fork.  A request "
         "is in flight until its callback has run: let it complete, or "
         "Cancel() it, and Poll() until the callback has been delivered "
         "before forking, or keep the child away from this Aio.";
}

Aio::~Aio() = default;

// The calls below that register, replace or remove something allocate or
// free on every backend, if only sometimes -- a freelist miss, a
// std::function capture, a registration table growing.  Something that only
// sometimes allocates only sometimes trips the realtime malloc hook, a
// data-dependent crash, so they are all illegal under realtime outright and
// say so here, once, for every backend.

void Aio::Run() { impl_->Run(); }

bool Aio::Poll(bool block) { return impl_->Poll(block); }

void Aio::Quit() { impl_->Quit(); }

bool Aio::should_run() const { return impl_->should_run(); }

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
  aos::CheckNotRealtime();
  impl_->BeforeWait(std::move(function));
}

void Aio::OnReadable(FileDescriptor fd, std::function<void()> callback) {
  aos::CheckNotRealtime();
  impl_->OnReadable(fd, std::move(callback));
}

void Aio::OnError(FileDescriptor fd, std::function<void()> callback) {
  aos::CheckNotRealtime();
  impl_->OnError(fd, std::move(callback));
}

void Aio::OnWritable(FileDescriptor fd, std::function<void()> callback) {
  aos::CheckNotRealtime();
  impl_->OnWritable(fd, std::move(callback));
}

void Aio::OnEvents(FileDescriptor fd, std::function<void(uint32_t)> callback) {
  aos::CheckNotRealtime();
  impl_->OnEvents(fd, std::move(callback));
}

void Aio::DeleteFd(FileDescriptor fd) {
  aos::CheckNotRealtime();
  impl_->DeleteFd(fd);
}

void Aio::ForgetClosedFd(FileDescriptor fd) {
  aos::CheckNotRealtime();
  impl_->ForgetClosedFd(fd);
}

void Aio::EnableWritable(FileDescriptor fd) { impl_->EnableWritable(fd); }

void Aio::DisableWritable(FileDescriptor fd) { impl_->DisableWritable(fd); }

void Aio::SetEvents(FileDescriptor fd, uint32_t events) {
  impl_->SetEvents(fd, events);
}

void Aio::RegisterThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver,
                                       std::function<void()> callback) {
  aos::CheckNotRealtime();
  impl_->RegisterThreadSignalReceiver(receiver, std::move(callback));
}

void Aio::UnregisterThreadSignalReceiver(
    ipc_lib::ThreadSignalReceiver *receiver) {
  aos::CheckNotRealtime();
  impl_->UnregisterThreadSignalReceiver(receiver);
}

void Aio::ConsumeThreadSignalReceiver(ipc_lib::ThreadSignalReceiver *receiver) {
  impl_->ConsumeThreadSignalReceiver(receiver);
}

Aio::Timer::Timer(Aio *aio) {
  aos::CheckNotRealtime();
  state_ = aio->impl_->MakeTimerState();
  state_->aio = aio;
  state_->Initialize();
}

Aio::Timer::~Timer() {
  aos::CheckNotRealtime();
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
