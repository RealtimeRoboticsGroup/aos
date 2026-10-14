#include "aos/events/timer_queue.h"

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace aos::testing {
namespace {

// A stand-in for the backend's timer state, carrying just what the queue
// touches.  `name` is only so a failure says which timer came back wrong.
struct TestTimer {
  explicit TestTimer(std::string name, int deadline_ms)
      : name(std::move(name)),
        deadline(aos::monotonic_clock::epoch() +
                 std::chrono::milliseconds(deadline_ms)) {}

  std::string name;
  aos::monotonic_clock::time_point deadline;
  TestTimer *left = nullptr;
  TestTimer *right = nullptr;
  TestTimer *parent = nullptr;
  bool red = false;
  uint64_t sequence = 0;
};

struct TestTraits {
  static TestTimer *&left(TestTimer *timer) { return timer->left; }
  static TestTimer *&right(TestTimer *timer) { return timer->right; }
  static TestTimer *&parent(TestTimer *timer) { return timer->parent; }
  static bool &red(TestTimer *timer) { return timer->red; }
  static uint64_t &sequence(TestTimer *timer) { return timer->sequence; }
  static aos::monotonic_clock::time_point deadline(const TestTimer *timer) {
    return timer->deadline;
  }
};

using Queue = TimerQueue<TestTimer, TestTraits>;

// The tree is only correct if it is still a red-black tree, which the ordering
// checks below cannot see.  Walks it and returns the black height, failing the
// test on any violation: parent links that do not match, a red node with a red
// child, or two root-to-leaf paths with different black counts.  A null leaf
// counts as black.
int CheckRedBlack(const TestTimer *node, const TestTimer *parent,
                  const TestTimer *lower, const TestTimer *upper) {
  if (node == nullptr) {
    return 1;
  }
  EXPECT_EQ(node->parent, parent)
      << ": " << node->name << " has the wrong parent link";
  // Binary search order, on the (deadline, sequence) key the queue sorts by.
  if (lower != nullptr) {
    EXPECT_TRUE(std::make_pair(lower->deadline, lower->sequence) <
                std::make_pair(node->deadline, node->sequence))
        << ": " << node->name << " is out of order under " << lower->name;
  }
  if (upper != nullptr) {
    EXPECT_TRUE(std::make_pair(node->deadline, node->sequence) <
                std::make_pair(upper->deadline, upper->sequence))
        << ": " << node->name << " is out of order under " << upper->name;
  }
  if (node->red) {
    EXPECT_FALSE(node->left != nullptr && node->left->red)
        << ": red " << node->name << " has a red left child";
    EXPECT_FALSE(node->right != nullptr && node->right->red)
        << ": red " << node->name << " has a red right child";
  }
  const int left = CheckRedBlack(node->left, node, lower, node);
  const int right = CheckRedBlack(node->right, node, node, upper);
  EXPECT_EQ(left, right) << ": black height differs under " << node->name;
  return left + (node->red ? 0 : 1);
}

// Reaches the root through front(), so the test needs no access to internals.
void CheckInvariants(const Queue &queue) {
  const TestTimer *root = queue.front();
  if (root == nullptr) {
    return;
  }
  while (root->parent != nullptr) {
    root = root->parent;
  }
  EXPECT_FALSE(root->red) << ": the root must be black";
  CheckRedBlack(root, nullptr, nullptr, nullptr);
}

// The order the queue would dispatch in, without consuming it.
std::vector<std::string> Order(const Queue &queue) {
  std::vector<std::string> result;
  for (TestTimer *timer = queue.front(); timer != nullptr;
       timer = Queue::Next(timer)) {
    result.push_back(timer->name);
  }
  return result;
}

// Draining is what a backend actually does: take the front, dispatch it, take
// the new front.  Re-checks the tree after every removal, since that is the
// operation with the case analysis worth doubting.
std::vector<std::string> Drain(Queue *queue) {
  std::vector<std::string> result;
  while (!queue->empty()) {
    TestTimer *const timer = queue->front();
    result.push_back(timer->name);
    queue->Remove(timer);
    CheckInvariants(*queue);
  }
  return result;
}

TEST(TimerQueueTest, EmptyHasNoFront) {
  Queue queue;
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.front(), nullptr);
}

TEST(TimerQueueTest, OrdersByDeadline) {
  Queue queue;
  TestTimer late("late", 30), early("early", 10), middle("middle", 20);
  // Inserted in an order unrelated to the deadlines.
  queue.Insert(&late);
  queue.Insert(&early);
  queue.Insert(&middle);

  EXPECT_EQ(Order(queue),
            (std::vector<std::string>{"early", "middle", "late"}));
}

// The property the kqueue backend exists to restore: timers due at the same
// instant dispatch in the order they were scheduled, not the reverse.
TEST(TimerQueueTest, IsFifoAmongEqualDeadlines) {
  Queue queue;
  TestTimer first("first", 10), second("second", 10), third("third", 10);
  queue.Insert(&first);
  queue.Insert(&second);
  queue.Insert(&third);

  EXPECT_EQ(Order(queue),
            (std::vector<std::string>{"first", "second", "third"}));
}

// Equal deadlines interleaved with distinct ones: deadline order wins, and
// insertion order breaks every tie within it.
TEST(TimerQueueTest, BreaksTiesWithinDeadlineOrder) {
  Queue queue;
  TestTimer a("a", 20), b("b", 10), c("c", 20), d("d", 10), e("e", 30);
  for (TestTimer *timer : {&a, &b, &c, &d, &e}) {
    queue.Insert(timer);
  }

  EXPECT_EQ(Order(queue), (std::vector<std::string>{"b", "d", "a", "c", "e"}));
}

TEST(TimerQueueTest, InsertingEarlierMovesTheFront) {
  Queue queue;
  TestTimer later("later", 20);
  queue.Insert(&later);
  EXPECT_EQ(queue.front(), &later);

  TestTimer sooner("sooner", 10);
  queue.Insert(&sooner);
  EXPECT_EQ(queue.front(), &sooner);
}

TEST(TimerQueueTest, RemovesFromEveryPosition) {
  Queue queue;
  TestTimer a("a", 10), b("b", 20), c("c", 30);
  for (TestTimer *timer : {&a, &b, &c}) {
    queue.Insert(timer);
  }

  queue.Remove(&b);  // middle
  EXPECT_EQ(Order(queue), (std::vector<std::string>{"a", "c"}));
  queue.Remove(&a);  // front
  EXPECT_EQ(Order(queue), (std::vector<std::string>{"c"}));
  queue.Remove(&c);  // back, and last
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.front(), nullptr);
}

// Removing the last timer while an earlier one is still armed.
TEST(TimerQueueTest, RemovesLastWithAPredecessor) {
  Queue queue;
  TestTimer a("a", 10), b("b", 20);
  queue.Insert(&a);
  queue.Insert(&b);

  queue.Remove(&b);
  EXPECT_EQ(Order(queue), (std::vector<std::string>{"a"}));

  // Appends through tail_, so a stale one shows up here.
  TestTimer c("c", 30);
  queue.Insert(&c);
  EXPECT_EQ(Order(queue), (std::vector<std::string>{"a", "c"}));
  EXPECT_EQ(Drain(&queue), (std::vector<std::string>{"a", "c"}));
}

// Removals interleaved with insertions, checking the tree after every single
// operation.  This is the test that actually exercises the rebalancing: the
// delete fixup's case analysis (red sibling, both nephews black, near nephew
// red) only comes up in trees deep enough to have them, so the queue is kept
// large and the deadlines drawn from a small set so ties are everywhere.
TEST(TimerQueueTest, SurvivesInterleavedInsertAndRemove) {
  std::mt19937 generator(1);
  std::vector<std::unique_ptr<TestTimer>> timers;
  std::vector<TestTimer *> on_queue;
  Queue queue;

  for (int step = 0; step < 20000; ++step) {
    // Biased towards inserting until the queue is big, then evenly, so it
    // spends most of its life deep rather than nearly empty.
    const bool insert =
        on_queue.empty() || (on_queue.size() < 200 ? (generator() % 4 != 0)
                                                   : (generator() % 2 == 0));
    if (insert) {
      timers.push_back(std::make_unique<TestTimer>(
          std::to_string(step), 1 + static_cast<int>(generator() % 8)));
      queue.Insert(timers.back().get());
      on_queue.push_back(timers.back().get());
    } else {
      const size_t index = generator() % on_queue.size();
      queue.Remove(on_queue[index]);
      on_queue.erase(on_queue.begin() + index);
    }

    ASSERT_NO_FATAL_FAILURE(CheckInvariants(queue)) << ": step " << step;

    // Still sorted, still holding exactly what we put on it, and front() still
    // agrees with the walk.
    size_t seen = 0;
    TestTimer *previous = nullptr;
    for (TestTimer *timer = queue.front(); timer != nullptr;
         timer = Queue::Next(timer)) {
      if (previous != nullptr) {
        ASSERT_LE(previous->deadline, timer->deadline) << ": step " << step;
        if (previous->deadline == timer->deadline) {
          ASSERT_LT(previous->sequence, timer->sequence) << ": step " << step;
        }
      }
      previous = timer;
      ++seen;
    }
    ASSERT_EQ(seen, on_queue.size()) << ": step " << step;
  }

  // And it unwinds cleanly from a deep state.
  while (!on_queue.empty()) {
    queue.Remove(on_queue.back());
    on_queue.pop_back();
    ASSERT_NO_FATAL_FAILURE(CheckInvariants(queue));
  }
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(queue.front(), nullptr);
}

// Ascending and descending insertion are the two shapes that would degenerate
// an unbalanced tree into a list.  Depth is bounded by 2*log2(n+1).
TEST(TimerQueueTest, StaysBalancedOnSortedInput) {
  for (const bool ascending : {true, false}) {
    constexpr int kCount = 1023;
    std::vector<std::unique_ptr<TestTimer>> timers;
    Queue queue;
    for (int i = 0; i < kCount; ++i) {
      const int deadline = ascending ? i : kCount - i;
      timers.push_back(
          std::make_unique<TestTimer>(std::to_string(i), deadline));
      queue.Insert(timers.back().get());
    }
    ASSERT_NO_FATAL_FAILURE(CheckInvariants(queue));

    // Deepest root-to-leaf path, measured by walking up from every node.
    int deepest = 0;
    for (const auto &timer : timers) {
      int depth = 0;
      for (const TestTimer *n = timer.get(); n != nullptr; n = n->parent) {
        ++depth;
      }
      deepest = std::max(deepest, depth);
    }
    // 2*log2(1024) = 20.
    EXPECT_LE(deepest, 20) << ": ascending=" << ascending;
  }
}

TEST(TimerQueueDeathTest, RefusesToInsertATimerAlreadyOnTheQueue) {
  Queue queue;
  TestTimer a("a", 10);
  queue.Insert(&a);
  // The sole-element case, where both links are null and only head_ tells.
  EXPECT_DEATH(queue.Insert(&a), "already in this tree");

  TestTimer b("b", 20);
  queue.Insert(&b);
  EXPECT_DEATH(queue.Insert(&a), "already in a tree");
}

TEST(TimerQueueDeathTest, RefusesToRemoveATimerThatIsNotOnTheQueue) {
  Queue queue;
  TestTimer a("a", 10), stranger("stranger", 10);
  queue.Insert(&a);
  EXPECT_DEATH(queue.Remove(&stranger), "not in this tree");
}

// Removing must leave the node clean enough to go straight back on, which is
// what rescheduling a timer does.
TEST(TimerQueueTest, ReinsertsAfterRemoval) {
  Queue queue;
  TestTimer a("a", 10), b("b", 20);
  queue.Insert(&a);
  queue.Insert(&b);

  queue.Remove(&a);
  a.deadline = aos::monotonic_clock::epoch() + std::chrono::milliseconds(30);
  queue.Insert(&a);

  EXPECT_EQ(Order(queue), (std::vector<std::string>{"b", "a"}));
}

// Emptying and refilling has to leave no stale head_/tail_ behind.
TEST(TimerQueueTest, SurvivesEmptying) {
  Queue queue;
  TestTimer a("a", 10);
  queue.Insert(&a);
  queue.Remove(&a);
  EXPECT_TRUE(queue.empty());

  TestTimer b("b", 20), c("c", 5);
  queue.Insert(&b);
  queue.Insert(&c);
  EXPECT_EQ(Order(queue), (std::vector<std::string>{"c", "b"}));
}

TEST(TimerQueueTest, DrainsInDispatchOrder) {
  Queue queue;
  TestTimer a("a", 20), b("b", 10), c("c", 20);
  for (TestTimer *timer : {&a, &b, &c}) {
    queue.Insert(timer);
  }

  EXPECT_EQ(Drain(&queue), (std::vector<std::string>{"b", "a", "c"}));
  EXPECT_TRUE(queue.empty());
}

// Randomized cross-check against a sort with the same key, over deadlines
// chosen from a small set so ties are common.
TEST(TimerQueueTest, MatchesAStableSortOnRandomInput) {
  std::mt19937 generator(0);
  std::uniform_int_distribution<int> deadlines(1, 5);

  for (int trial = 0; trial < 200; ++trial) {
    std::vector<std::unique_ptr<TestTimer>> timers;
    std::vector<std::pair<int, size_t>> expected;
    Queue queue;
    const int count = 1 + static_cast<int>(generator() % 12);
    for (int i = 0; i < count; ++i) {
      const int deadline = deadlines(generator);
      timers.push_back(
          std::make_unique<TestTimer>(std::to_string(i), deadline));
      expected.emplace_back(deadline, i);
      queue.Insert(timers.back().get());
    }

    // Stable, so equal deadlines stay in insertion order -- the queue's rule.
    std::stable_sort(
        expected.begin(), expected.end(),
        [](const auto &a, const auto &b) { return a.first < b.first; });
    std::vector<std::string> expected_names;
    for (const auto &pair : expected) {
      expected_names.push_back(std::to_string(pair.second));
    }

    EXPECT_EQ(Drain(&queue), expected_names) << ": trial " << trial;
  }
}

// Removing a timer that is not the front, from the middle of a run of equal
// deadlines, must not disturb the rest of that run.
TEST(TimerQueueTest, RemovalKeepsTheRestOfATieIntact) {
  Queue queue;
  TestTimer a("a", 10), b("b", 10), c("c", 10), d("d", 10);
  for (TestTimer *timer : {&a, &b, &c, &d}) {
    queue.Insert(timer);
  }

  queue.Remove(&c);
  EXPECT_EQ(Order(queue), (std::vector<std::string>{"a", "b", "d"}));
}

}  // namespace
}  // namespace aos::testing
