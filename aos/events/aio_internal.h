#ifndef AOS_EVENTS_AIO_INTERNAL_H_
#define AOS_EVENTS_AIO_INTERNAL_H_

#include <algorithm>
#include <functional>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include "absl/log/absl_check.h"

#include "aos/events/aio.h"
#include "aos/realtime.h"

namespace aos {

namespace internal {
// What every backend says when a request is armed again before its callback
// has run -- see AsyncRequest::done.  One string, so a death test matches it
// on all of them.
inline constexpr char kRequestStillInFlight[] =
    ": AsyncRead()/AsyncWrite() on a request that is still in flight; wait "
    "for its callback (a Cancel()ed request's too) before re-arming it";
}  // namespace internal

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
  IntrusiveStack() = default;
  // A copy would share nodes with the original, so a push onto either one
  // would corrupt the other.
  IntrusiveStack(const IntrusiveStack &) = delete;
  IntrusiveStack &operator=(const IntrusiveStack &) = delete;

  bool empty() const { return head_ == nullptr; }

  void Push(Node *node) {
    Traits::next(node) = head_;
    head_ = node;
  }

  // Visits every node.  Does not tolerate the callback unlinking a node.
  template <typename Fn>
  void ForEach(Fn fn) const {
    for (Node *node = head_; node != nullptr; node = Traits::next(node)) {
      fn(node);
    }
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

// Intrusive singly-linked FIFO.  Same Traits as IntrusiveStack -- one `next`
// link inside the node -- plus a tail pointer, so what comes out is in the
// order it went in.  Use it where that order is observable (a queue of
// completions a caller is driving) and IntrusiveStack where it is not.
// Nothing allocates or frees; a node may only be on one list at a time.
template <typename Node, typename Traits>
class IntrusiveFifo {
 public:
  IntrusiveFifo() = default;
  // A copy would share nodes with the original, so a push onto either one
  // would corrupt the other.
  IntrusiveFifo(const IntrusiveFifo &) = delete;
  IntrusiveFifo &operator=(const IntrusiveFifo &) = delete;

  bool empty() const { return head_ == nullptr; }

  void PushBack(Node *node) {
    Traits::next(node) = nullptr;
    if (tail_ != nullptr) {
      Traits::next(tail_) = node;
    } else {
      head_ = node;
    }
    tail_ = node;
  }

  // Visits every node, oldest first.  Does not tolerate the callback
  // unlinking a node.
  template <typename Fn>
  void ForEach(Fn fn) const {
    for (Node *node = head_; node != nullptr; node = Traits::next(node)) {
      fn(node);
    }
  }

  // Pops and returns the least recently pushed node, or nullptr when empty.
  Node *PopFront() {
    Node *node = head_;
    if (node != nullptr) {
      head_ = Traits::next(node);
      if (head_ == nullptr) {
        tail_ = nullptr;
      }
      Traits::next(node) = nullptr;
    }
    return node;
  }

 private:
  Node *head_ = nullptr;
  Node *tail_ = nullptr;
};

// An IntrusiveStack which owns what is on it, deleting whatever is left when
// it goes away.  Use it where the stack is the owner -- a free pool, a list of
// orphaned states -- and plain IntrusiveStack where the nodes belong to
// someone else, so the type says which it is.
//
// Where teardown order matters, keep calling Clear() explicitly at the point
// it has to happen; the destructor is then a backstop rather than the plan.
// That is the whole point: forgetting to drain one stops being a leak.
template <typename Node, typename Traits>
class OwningIntrusiveStack : public IntrusiveStack<Node, Traits> {
 public:
  OwningIntrusiveStack() = default;
  // Copying one would double-free; nothing needs to move one yet.
  OwningIntrusiveStack(const OwningIntrusiveStack &) = delete;
  OwningIntrusiveStack &operator=(const OwningIntrusiveStack &) = delete;
  ~OwningIntrusiveStack() { Clear(); }

  // Deletes everything on the stack.  Allocator work, so not for RT threads.
  void Clear() {
    while (Node *node = this->Pop()) {
      delete node;
    }
  }
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
  IntrusiveDoublyLinkedList() = default;
  // A copy would share nodes with the original, so a push onto either one
  // would corrupt the other.
  IntrusiveDoublyLinkedList(const IntrusiveDoublyLinkedList &) = delete;
  IntrusiveDoublyLinkedList &operator=(const IntrusiveDoublyLinkedList &) =
      delete;

  bool empty() const { return head_ == nullptr; }
  // Maintained by the mutators below rather than counted on demand.  A caller
  // that needs the length has it in O(1), and cannot get it out of step with
  // the list by keeping its own tally beside it.
  size_t size() const { return size_; }
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
    ++size_;
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
    ++size_;
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
    --size_;
  }

  // Visits every node, front to back.  Does not tolerate the callback
  // unlinking a node.
  template <typename Fn>
  void ForEach(Fn fn) const {
    for (Node *node = head_; node != nullptr; node = Traits::next(node)) {
      fn(node);
    }
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
  size_t size_ = 0;
};

struct Aio::TimerState {
  TimerState() = default;
  TimerState(const TimerState &) = delete;
  TimerState &operator=(const TimerState &) = delete;
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

// One Aio's before-wait functions, run in the order they were added.  Shared
// by every backend so that the rules below cannot drift apart.
//
// Neither adding nor removing is allowed from inside a before-wait function:
// either can reallocate or shift the vector while Run() is executing an
// element of it.  That is deterministically illegal rather than sometimes
// corrupting.
class BeforeWaitFunctions {
 public:
  Aio::BeforeWaitId Add(std::function<void()> function) {
    ABSL_CHECK(!running_)
        << ": BeforeWait() may not be called from a before-wait function";
    // Unique in the process rather than per Aio, so that an id handed to the
    // wrong Aio dies in Remove() instead of removing that Aio's function.
    // Not starting at 0, so a zero-initialized id never names a function.
    static std::atomic<Aio::BeforeWaitId> next_id{1868};
    const Aio::BeforeWaitId id =
        next_id.fetch_add(1, std::memory_order_relaxed);
    functions_.emplace_back(id, std::move(function));
    return id;
  }

  void Remove(Aio::BeforeWaitId id) {
    ABSL_CHECK(!running_) << ": RemoveBeforeWait() may not be called from a "
                             "before-wait function";
    auto it = std::find_if(
        functions_.begin(), functions_.end(),
        [id](const auto &function) { return function.first == id; });
    ABSL_CHECK(it != functions_.end())
        << ": RemoveBeforeWait() of a function this Aio does not have";
    // Out of the vector before erasing, so that its destructor runs once the
    // vector is whole again.  Its captures can own something whose
    // destructor adds or removes a before-wait function on this Aio, a
    // borrowed ShmEventLoop or a GlibMainLoop say, and that would otherwise
    // run on a half-shifted vector.
    std::function<void()> removed = std::move(it->second);
    functions_.erase(it);
  }

  void Run() {
    running_ = true;
    for (const auto &function : functions_) {
      function.second();
    }
    running_ = false;
  }

  // True while Run() is calling them.
  bool running() const { return running_; }

  bool empty() const { return functions_.empty(); }

 private:
  std::vector<std::pair<Aio::BeforeWaitId, std::function<void()>>> functions_;
  bool running_ = false;
};

struct Aio::Impl {
  Impl() = default;
  Impl(const Impl &) = delete;
  Impl &operator=(const Impl &) = delete;
  virtual ~Impl() = default;

  // --- fork() contract -------------------------------------------------
  //
  // Every backend has to rebuild its kernel-side state in a forked child:
  // an io_uring ring, an epoll instance, and a kqueue are all invalid
  // there.  The mechanism is per-backend; what lives here is the part they
  // must not get subtly different from each other.

  // Whether a caller-submitted AsyncRead/AsyncWrite is in flight, which
  // lasts until its callback has run: resolved but not yet delivered counts.
  // A backend's own internal reads don't count -- the wakeup eventfd read
  // goes through the public AsyncRead() on three of the four backends and
  // is reconstructed rather than lost, so counting it would refuse every
  // fork.
  virtual bool HasRawRequestsInFlight() const = 0;

  // Dies if this loop has raw I/O in flight.  Called by each backend at the
  // top of its reconstruction, before it has torn anything down.
  //
  // Every answer here is a bug, which is why this refuses to pick one:
  // io_uring's child cannot complete the request (it was submitted to the
  // inherited ring, and there is no registry to re-arm raw requests from),
  // while epoll's and kqueue's registrations survive the fork, so
  // reconstruction would re-arm them and repeat a read or write the parent
  // is also still doing.  Shared rather than copied per backend so the rule
  // and the message it dies with cannot drift apart, and so a new backend
  // inherits both by implementing one predicate.
  void CheckNoRawRequestsInFlightOnFork() const;

  virtual std::unique_ptr<Aio::TimerState> MakeTimerState() = 0;

  // Disposes of a timer's state on Timer destruction.  The default --
  // destroy it now, which runs the backend's synchronous teardown -- is
  // right for backends whose cancels resolve inline.  The io_uring backend
  // overrides this to *orphan* the state instead: destruction submits an async
  // cancel and returns immediately, and the state is recycled once its last
  // kernel completion has drained.
  //
  // Illegal under RT on every backend, since this can free;
  // Aio::Timer::~Timer() checks that before calling this.
  virtual void DestroyTimerState(std::unique_ptr<Aio::TimerState> state) {
    state.reset();
  }

  virtual void Run() = 0;
  virtual bool should_run() const = 0;
  virtual bool Poll(bool block) = 0;
  virtual void Quit() = 0;

  virtual void AsyncRead(FileDescriptor fd, std::span<char> buffer,
                         AsyncRequest *request) = 0;
  virtual void AsyncWrite(FileDescriptor fd, std::span<const char> buffer,
                          AsyncRequest *request) = 0;
  virtual void Cancel(AsyncRequest *request) = 0;
  virtual Aio::BeforeWaitId BeforeWait(std::function<void()> function) = 0;
  virtual void RemoveBeforeWait(Aio::BeforeWaitId id) = 0;

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
