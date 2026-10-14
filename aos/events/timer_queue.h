#ifndef AOS_EVENTS_TIMER_QUEUE_H_
#define AOS_EVENTS_TIMER_QUEUE_H_

#include <cstdint>

#include "aos/events/intrusive_rb_tree.h"
#include "aos/time/time.h"

namespace aos {

// The armed timers, in the order they must fire.
//
// A thin layer over IntrusiveRbTree: it supplies the (deadline, sequence)
// ordering and stamps the sequence, which is what makes equal deadlines FIFO
// rather than arbitrary -- see the comment above IntrusiveRbTree for why that
// order is load-bearing across backends.
//
// Traits supplies the tree's link accessors (left/right/parent/red) plus:
//   static uint64_t &sequence(Node *node);
//   static aos::monotonic_clock::time_point deadline(const Node *node);
template <typename Node, typename Traits>
class TimerQueue {
 private:
  // Turns the timer ordering into the plain Less() the tree wants.
  struct TreeTraits : Traits {
    static bool Less(Node *a, Node *b) {
      const aos::monotonic_clock::time_point a_deadline = Traits::deadline(a);
      const aos::monotonic_clock::time_point b_deadline = Traits::deadline(b);
      if (a_deadline != b_deadline) {
        return a_deadline < b_deadline;
      }
      return Traits::sequence(a) < Traits::sequence(b);
    }
  };

 public:
  bool empty() const { return tree_.empty(); }
  Node *front() const { return tree_.front(); }
  static Node *Next(Node *node) {
    return IntrusiveRbTree<Node, TreeTraits>::Next(node);
  }

  void Insert(Node *node) {
    // Stamped here rather than by the caller, so the FIFO rule cannot be got
    // wrong from outside.  64 bits of arming outlasts any machine this runs
    // on, so wrapping is not a case worth handling.
    Traits::sequence(node) = next_sequence_++;
    tree_.Insert(node);
  }

  void Remove(Node *node) { tree_.Remove(node); }

 private:
  IntrusiveRbTree<Node, TreeTraits> tree_;
  uint64_t next_sequence_ = 0;
};

}  // namespace aos

#endif  // AOS_EVENTS_TIMER_QUEUE_H_
