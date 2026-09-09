#include "aos/events/aio_internal.h"

#include <vector>

#include "gtest/gtest.h"

namespace aos::testing {
namespace {

// One node type for every container, so a test can move a node between
// them the way the backends do (a registration is on the tree, then the
// retired stack, then the free list) without a link field going unused.
struct Node {
  explicit Node(int value_in) : value(value_in) {}
  int value;
  Node *next = nullptr;
  Node *prev = nullptr;
};

struct NextTraits {
  static Node *&next(Node *node) { return node->next; }
};

struct BothTraits {
  static Node *&next(Node *node) { return node->next; }
  static Node *&prev(Node *node) { return node->prev; }
};

template <typename Container>
std::vector<int> Values(const Container &container) {
  std::vector<int> result;
  container.ForEach([&result](Node *node) { result.push_back(node->value); });
  return result;
}

TEST(IntrusiveStackTest, PopsInReverseOrderOfPush) {
  IntrusiveStack<Node, NextTraits> stack;
  EXPECT_TRUE(stack.empty());
  EXPECT_EQ(stack.Pop(), nullptr);

  Node a(1), b(2), c(3);
  stack.Push(&a);
  stack.Push(&b);
  stack.Push(&c);
  EXPECT_FALSE(stack.empty());
  EXPECT_EQ(Values(stack), (std::vector<int>{3, 2, 1}));

  EXPECT_EQ(stack.Pop(), &c);
  EXPECT_EQ(stack.Pop(), &b);
  EXPECT_EQ(stack.Pop(), &a);
  EXPECT_EQ(stack.Pop(), nullptr);
  EXPECT_TRUE(stack.empty());
  // A popped node's link is cleared, so it can go straight onto another
  // list.  c's pointed at b before it was popped.
  EXPECT_EQ(c.next, nullptr);
}

TEST(IntrusiveStackTest, RemoveUnlinksFromAnywhere) {
  IntrusiveStack<Node, NextTraits> stack;
  Node a(1), b(2), c(3), stranger(4);
  stack.Push(&a);
  stack.Push(&b);
  stack.Push(&c);

  // The middle, the head, then the only one left; and one never pushed.
  EXPECT_TRUE(stack.Remove(&b));
  EXPECT_EQ(b.next, nullptr);
  EXPECT_EQ(Values(stack), (std::vector<int>{3, 1}));
  EXPECT_TRUE(stack.Remove(&c));
  EXPECT_EQ(Values(stack), (std::vector<int>{1}));
  EXPECT_FALSE(stack.Remove(&stranger));
  EXPECT_TRUE(stack.Remove(&a));
  EXPECT_TRUE(stack.empty());
  EXPECT_FALSE(stack.Remove(&a));
}

// The matching nodes come off, and the survivors keep their order.
TEST(IntrusiveStackTest, MoveMatchingToKeepsSurvivorsInOrder) {
  IntrusiveStack<Node, NextTraits> source;
  IntrusiveStack<Node, NextTraits> destination;
  Node nodes[] = {Node(1), Node(2), Node(3), Node(4), Node(5)};
  for (Node &node : nodes) {
    source.Push(&node);
  }
  ASSERT_EQ(Values(source), (std::vector<int>{5, 4, 3, 2, 1}));

  source.MoveMatchingTo(&destination,
                        [](Node *node) { return node->value % 2 == 0; });
  EXPECT_EQ(Values(source), (std::vector<int>{5, 3, 1}));
  // Pushed onto destination one at a time, so they arrive reversed.
  EXPECT_EQ(Values(destination), (std::vector<int>{2, 4}));

  // Nothing matches: nothing moves.
  source.MoveMatchingTo(&destination, [](Node *) { return false; });
  EXPECT_EQ(Values(source), (std::vector<int>{5, 3, 1}));
  // Everything matches: source empties.
  source.MoveMatchingTo(&destination, [](Node *) { return true; });
  EXPECT_TRUE(source.empty());
  EXPECT_EQ(Values(destination), (std::vector<int>{1, 3, 5, 2, 4}));
}

TEST(IntrusiveFifoTest, PopsInOrderOfPush) {
  IntrusiveFifo<Node, NextTraits> fifo;
  EXPECT_TRUE(fifo.empty());
  EXPECT_EQ(fifo.PopFront(), nullptr);

  Node a(1), b(2), c(3);
  fifo.PushBack(&a);
  fifo.PushBack(&b);
  fifo.PushBack(&c);
  EXPECT_FALSE(fifo.empty());
  EXPECT_EQ(Values(fifo), (std::vector<int>{1, 2, 3}));

  EXPECT_EQ(fifo.PopFront(), &a);
  EXPECT_EQ(a.next, nullptr);
  EXPECT_EQ(Values(fifo), (std::vector<int>{2, 3}));
  EXPECT_EQ(fifo.PopFront(), &b);
  EXPECT_EQ(fifo.PopFront(), &c);
  EXPECT_EQ(fifo.PopFront(), nullptr);
  EXPECT_TRUE(fifo.empty());
}

// Emptying the list and refilling it exercises the tail pointer: a FIFO
// whose tail was left dangling after the last pop would link the next push
// onto a node that is no longer on the list.
TEST(IntrusiveFifoTest, RefillsAfterEmptying) {
  IntrusiveFifo<Node, NextTraits> fifo;
  Node a(1), b(2), c(3);
  fifo.PushBack(&a);
  EXPECT_EQ(fifo.PopFront(), &a);
  ASSERT_TRUE(fifo.empty());

  fifo.PushBack(&b);
  fifo.PushBack(&c);
  EXPECT_EQ(Values(fifo), (std::vector<int>{2, 3}));
  // The popped node stayed unlinked.
  EXPECT_EQ(a.next, nullptr);
  EXPECT_EQ(fifo.PopFront(), &b);
  EXPECT_EQ(fifo.PopFront(), &c);
  EXPECT_TRUE(fifo.empty());
}

// A node may only be on one list at a time, and the backends move states
// between a stack and a FIFO through the same link.
TEST(IntrusiveFifoTest, ShareALinkWithAStack) {
  IntrusiveFifo<Node, NextTraits> fifo;
  IntrusiveStack<Node, NextTraits> stack;
  Node a(1), b(2);
  fifo.PushBack(&a);
  fifo.PushBack(&b);
  stack.Push(fifo.PopFront());
  stack.Push(fifo.PopFront());
  EXPECT_TRUE(fifo.empty());
  EXPECT_EQ(Values(stack), (std::vector<int>{2, 1}));
  fifo.PushBack(stack.Pop());
  fifo.PushBack(stack.Pop());
  EXPECT_EQ(Values(fifo), (std::vector<int>{2, 1}));
}

TEST(IntrusiveDoublyLinkedListTest, PushesAtBothEnds) {
  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  EXPECT_TRUE(list.empty());
  EXPECT_EQ(list.front(), nullptr);
  EXPECT_EQ(list.PopFront(), nullptr);

  Node a(1), b(2), c(3);
  list.PushBack(&b);
  list.PushFront(&a);
  list.PushBack(&c);
  EXPECT_EQ(list.front(), &a);
  EXPECT_EQ(Values(list), (std::vector<int>{1, 2, 3}));
  using List = IntrusiveDoublyLinkedList<Node, BothTraits>;
  EXPECT_EQ(List::Next(&a), &b);
  EXPECT_EQ(List::Next(&c), nullptr);

  EXPECT_EQ(list.PopFront(), &a);
  EXPECT_EQ(a.next, nullptr);
  EXPECT_EQ(a.prev, nullptr);
  EXPECT_EQ(list.PopFront(), &b);
  EXPECT_EQ(list.PopFront(), &c);
  EXPECT_TRUE(list.empty());
}

// PushFront() onto an empty list has to set the tail too, or the next
// PushBack() links onto nothing.
TEST(IntrusiveDoublyLinkedListTest, PushesFrontOntoEmpty) {
  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  Node a(1), b(2);
  list.PushFront(&a);
  EXPECT_EQ(list.front(), &a);
  EXPECT_EQ(a.next, nullptr);
  EXPECT_EQ(a.prev, nullptr);
  list.PushBack(&b);
  EXPECT_EQ(Values(list), (std::vector<int>{1, 2}));
  EXPECT_EQ(b.prev, &a);

  EXPECT_EQ(list.PopFront(), &a);
  EXPECT_EQ(list.PopFront(), &b);
  EXPECT_TRUE(list.empty());
}

// size() has to track every path in and out, including rotating the front to
// the back (PopFront() then PushBack()) and a Remove() from the middle.  The
// whole point of it living on the list is that a caller cannot get it out of
// step.
TEST(IntrusiveDoublyLinkedListTest, TracksItsOwnSize) {
  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  EXPECT_EQ(list.size(), 0u);

  Node a(1), b(2), c(3);
  list.PushBack(&a);
  EXPECT_EQ(list.size(), 1u);
  list.PushFront(&b);
  list.PushBack(&c);
  EXPECT_EQ(list.size(), 3u);

  // Rotating leaves the size alone, and a full cycle of them leaves the
  // order alone too.
  const std::vector<int> before = Values(list);
  for (int i = 0; i < 3; ++i) {
    list.PushBack(list.PopFront());
    EXPECT_EQ(list.size(), 3u);
  }
  EXPECT_EQ(Values(list), before);

  list.Remove(&a);
  EXPECT_EQ(list.size(), 2u);
  EXPECT_EQ(list.PopFront(), &b);
  EXPECT_EQ(list.size(), 1u);
  EXPECT_NE(list.PopFront(), nullptr);
  EXPECT_EQ(list.size(), 0u);
  EXPECT_TRUE(list.empty());

  // Popping an empty list must not run it below zero.
  EXPECT_EQ(list.PopFront(), nullptr);
  EXPECT_EQ(list.size(), 0u);
}

TEST(IntrusiveDoublyLinkedListTest, RemovesFromAnywhere) {
  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  Node a(1), b(2), c(3), d(4);
  list.PushBack(&a);
  list.PushBack(&b);
  list.PushBack(&c);
  list.PushBack(&d);

  // The middle, the tail, the head, then the last one standing.
  list.Remove(&b);
  EXPECT_EQ(Values(list), (std::vector<int>{1, 3, 4}));
  list.Remove(&d);
  EXPECT_EQ(Values(list), (std::vector<int>{1, 3}));
  list.Remove(&a);
  EXPECT_EQ(Values(list), (std::vector<int>{3}));
  EXPECT_EQ(list.front(), &c);
  list.Remove(&c);
  EXPECT_TRUE(list.empty());

  // Every removed node comes out fully unlinked.
  for (Node *node : {&a, &b, &c, &d}) {
    EXPECT_EQ(node->next, nullptr);
    EXPECT_EQ(node->prev, nullptr);
  }
  // And the list is usable again.
  list.PushBack(&b);
  EXPECT_EQ(Values(list), (std::vector<int>{2}));
}

// Rotating the front to the back, with removals in between, keeps every node
// reachable and in order.
TEST(IntrusiveDoublyLinkedListTest, RotatesWithRemovals) {
  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  Node a(1), b(2), c(3);
  list.PushBack(&a);
  list.PushBack(&b);
  list.PushBack(&c);

  list.PushBack(list.PopFront());
  EXPECT_EQ(Values(list), (std::vector<int>{2, 3, 1}));
  list.Remove(&c);
  list.PushBack(list.PopFront());
  EXPECT_EQ(Values(list), (std::vector<int>{1, 2}));
  list.PushBack(list.PopFront());
  EXPECT_EQ(Values(list), (std::vector<int>{2, 1}));
}

TEST(IntrusiveDoublyLinkedListDeathTest, RemovingAStrangerDies) {
  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  Node a(1), stranger(2);
  list.PushBack(&a);
  EXPECT_DEATH(list.Remove(&stranger), "not on this list");
}

struct CountedNode {
  explicit CountedNode(int *live_in) : live(live_in) { ++*live; }
  ~CountedNode() { --*live; }
  int *live;
  CountedNode *next = nullptr;
};

struct CountedTraits {
  static CountedNode *&next(CountedNode *node) { return node->next; }
};

// A node's links may hold unrelated data while it is on no list (AioState's
// link is in a union), so a push sets every link it owns rather than trusting
// what is there.  Each node starts out pointing at a node that is on no list.
TEST(IntrusiveDoublyLinkedListTest, PushOverwritesStaleLinks) {
  Node garbage(99), a(1), b(2), c(3);
  for (Node *node : {&a, &b, &c}) {
    node->next = &garbage;
    node->prev = &garbage;
  }

  IntrusiveDoublyLinkedList<Node, BothTraits> list;
  list.PushBack(&a);
  list.PushFront(&b);
  EXPECT_EQ(Values(list), (std::vector<int>{2, 1}));
  EXPECT_EQ(b.prev, nullptr);
  EXPECT_EQ(a.next, nullptr);

  IntrusiveFifo<Node, NextTraits> fifo;
  fifo.PushBack(&c);
  EXPECT_EQ(Values(fifo), (std::vector<int>{3}));
}

TEST(OwningIntrusiveStackTest, DeletesWhatIsLeftOnIt) {
  int live = 0;
  {
    OwningIntrusiveStack<CountedNode, CountedTraits> stack;
    stack.Push(new CountedNode(&live));
    stack.Push(new CountedNode(&live));
    stack.Push(new CountedNode(&live));
    EXPECT_EQ(live, 3);

    // A node popped off is the caller's again, and the stack does not touch
    // it after that.
    CountedNode *taken = stack.Pop();
    EXPECT_EQ(live, 3);

    stack.Clear();
    EXPECT_EQ(live, 1);
    EXPECT_TRUE(stack.empty());

    stack.Push(taken);
  }
  EXPECT_EQ(live, 0) << "the destructor did not delete what was pushed back";
}

}  // namespace
}  // namespace aos::testing
