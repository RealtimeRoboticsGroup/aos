#ifndef AOS_EVENTS_AIO_INTERNAL_H_
#define AOS_EVENTS_AIO_INTERNAL_H_

#include <functional>
#include <memory>
#include <span>

#include "absl/log/absl_check.h"

#include "aos/events/aio.h"

namespace aos {

// Intrusive singly-linked LIFO stack.  The link lives inside the node;
// Traits names it:
//
//   struct Traits { static Node *&next(Node *node) { return node->next_; } };
//
// (a traits struct rather than a member pointer, so links can live inside
// unions or nested private types).  Nothing allocates or frees -- every
// operation is pointer manipulation, safe on RT threads.  A node may only
// be on one stack at a time; the caller tracks membership where it isn't
// structurally obvious.
template <typename Node, typename Traits>
class IntrusiveStack {
 public:
  bool empty() const { return head_ == nullptr; }

  void Push(Node *node) {
    Traits::next(node) = head_;
    head_ = node;
  }

  // Pops and returns the most recently pushed node, or nullptr when empty.
  Node *Pop() {
    Node *node = head_;
    if (node != nullptr) {
      head_ = Traits::next(node);
      Traits::next(node) = nullptr;
    }
    return node;
  }

  // Removes node if present.  Returns whether it was.  O(n).
  bool Remove(Node *node) {
    for (Node **link = &head_; *link != nullptr; link = &Traits::next(*link)) {
      if (*link == node) {
        *link = Traits::next(node);
        Traits::next(node) = nullptr;
        return true;
      }
    }
    return false;
  }

  // Moves every node matching pred onto dst, preserving the survivors'
  // order.  O(n), pointer manipulation only.
  template <typename Pred>
  void MoveMatchingTo(IntrusiveStack *dst, Pred pred) {
    Node **link = &head_;
    while (*link != nullptr) {
      Node *node = *link;
      if (pred(node)) {
        *link = Traits::next(node);
        dst->Push(node);
      } else {
        link = &Traits::next(node);
      }
    }
  }

 private:
  Node *head_ = nullptr;
};

// Intrusive doubly-linked FIFO with O(1) removal from anywhere in the list.
// Traits names both links:
//
//   struct Traits {
//     static Node *&next(Node *node);
//     static Node *&prev(Node *node);
//   };
//
// The link fields may hold unrelated data while a node is unlinked (see
// AioState::link, which lives in a union), so membership cannot be inferred
// from the links themselves -- the caller tracks it.  What the class does
// verify: Remove() CHECKs that a node with a null prev/next really is the
// head/tail, turning "unlinked a node that was never on this list" (or a
// corrupted splice) into a loud crash instead of silent list corruption.
// Nothing allocates or frees; all operations are RT-safe.
template <typename Node, typename Traits>
class IntrusiveDoublyLinkedList {
 public:
  bool empty() const { return head_ == nullptr; }
  Node *front() const { return head_; }
  static Node *Next(Node *node) { return Traits::next(node); }

  void PushFront(Node *node) {
    Traits::prev(node) = nullptr;
    Traits::next(node) = head_;
    if (head_ != nullptr) {
      Traits::prev(head_) = node;
    } else {
      tail_ = node;
    }
    head_ = node;
  }

  void PushBack(Node *node) {
    Traits::next(node) = nullptr;
    Traits::prev(node) = tail_;
    if (tail_ != nullptr) {
      Traits::next(tail_) = node;
    } else {
      head_ = node;
    }
    tail_ = node;
  }

  void Remove(Node *node) {
    Node *prev = Traits::prev(node);
    Node *next = Traits::next(node);
    if (prev != nullptr) {
      Traits::next(prev) = next;
    } else {
      ABSL_CHECK_EQ(head_, node)
          << ": removing a node that is not on this list";
      head_ = next;
    }
    if (next != nullptr) {
      Traits::prev(next) = prev;
    } else {
      ABSL_CHECK_EQ(tail_, node)
          << ": removing a node that is not on this list";
      tail_ = prev;
    }
    Traits::prev(node) = nullptr;
    Traits::next(node) = nullptr;
  }

  // Pops and returns the front node, or nullptr when empty.
  Node *PopFront() {
    Node *node = head_;
    if (node != nullptr) {
      Remove(node);
    }
    return node;
  }

 private:
  Node *head_ = nullptr;
  Node *tail_ = nullptr;
};

struct Aio::TimerState {
  virtual ~TimerState() = default;

  virtual void Initialize() = 0;
  virtual void Schedule(aos::monotonic_clock::time_point deadline,
                        CompletionCallback callback, void *context) = 0;
  virtual void Cancel(bool reap) = 0;

  Aio *aio = nullptr;
  AsyncRequest request;
  aos::monotonic_clock::time_point deadline = aos::monotonic_clock::epoch();
  CompletionCallback user_callback = nullptr;
  void *user_context = nullptr;

  // Intrusive doubly-linked list pointers for active timers
  Aio::TimerState *prev_active = nullptr;
  Aio::TimerState *next_active = nullptr;
  bool is_active = false;

  // Set while an asynchronous cancellation is in flight (io_uring backend).
  // Only teardown does that now: IoUringImpl::DestroyTimerState() cancels the
  // multishot poll the timer holds on its timerfd and parks the state as an
  // orphan until that cancel's completion drains.  It is what makes the
  // resulting error completion expected rather than fatal, and what stops the
  // poll from being re-armed in the meantime.  Scheduling and cancelling the
  // *timer* never set this -- both are a plain timerfd_settime(2).
  bool canceling = false;
};

struct Aio::Impl {
  virtual ~Impl() = default;

  virtual std::unique_ptr<Aio::TimerState> MakeTimerState() = 0;

  // Disposes of a timer's state on Timer destruction.  The default --
  // destroy it now, which runs the backend's synchronous teardown -- is
  // right for backends whose cancels resolve inline.  The io_uring backend
  // overrides this to *orphan* the state instead: destruction submits an async
  // cancel and returns immediately, and the state is recycled once its last
  // kernel completion has drained.
  virtual void DestroyTimerState(std::unique_ptr<Aio::TimerState> state) {
    state.reset();
  }

  virtual void Run() = 0;
  virtual bool Poll(bool block) = 0;
  virtual void Quit() = 0;

  virtual void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                         AsyncRequest *request) = 0;
  virtual void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                          AsyncRequest *request) = 0;
  virtual void Cancel(AsyncRequest *request) = 0;
  virtual void BeforeWait(std::function<void()> function) = 0;

  // Legacy Readiness Hooks.
  virtual void OnReadable(FileDescriptor fd,
                          std::function<void()> callback) = 0;
  virtual void OnError(FileDescriptor fd, std::function<void()> callback) = 0;
  virtual void OnWritable(FileDescriptor fd,
                          std::function<void()> callback) = 0;
  virtual void OnEvents(FileDescriptor fd,
                        std::function<void(uint32_t)> callback) = 0;
  virtual void DeleteFd(FileDescriptor fd) = 0;
  virtual void ForgetClosedFd(FileDescriptor fd) = 0;
  virtual void EnableWritable(FileDescriptor fd) = 0;
  virtual void DisableWritable(FileDescriptor fd) = 0;
  virtual void SetEvents(FileDescriptor fd, uint32_t events) = 0;

  // Registers a ThreadSignalReceiver for wakeups.
  virtual void RegisterThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver,
      std::function<void()> callback) = 0;
  virtual void UnregisterThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) = 0;
  virtual void ConsumeThreadSignalReceiver(
      ipc_lib::ThreadSignalReceiver *receiver) = 0;
};

}  // namespace aos

#endif  // AOS_EVENTS_AIO_INTERNAL_H_
