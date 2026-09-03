#include "aos/events/intrusive_rb_tree.h"

#include <algorithm>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace aos::testing {
namespace {

// A node keyed on a plain int, deliberately not a timer: the tree takes its
// ordering from Traits, and this is the test that it really does.
struct TestNode {
  explicit TestNode(int key) : key(key) {}

  int key;
  TestNode *left = nullptr;
  TestNode *right = nullptr;
  TestNode *parent = nullptr;
  bool red = false;
};

struct TestTraits {
  static TestNode *&left(TestNode *node) { return node->left; }
  static TestNode *&right(TestNode *node) { return node->right; }
  static TestNode *&parent(TestNode *node) { return node->parent; }
  static bool &red(TestNode *node) { return node->red; }
  static bool Less(TestNode *a, TestNode *b) { return a->key < b->key; }
  static int Compare(const TestNode *node, int key) {
    if (node->key < key) return -1;
    if (key < node->key) return 1;
    return 0;
  }
};

using Tree = IntrusiveRbTree<TestNode, TestTraits>;

// The ordering checks below cannot see whether this is still a red-black
// tree, and an unbalanced one still answers every query correctly -- it just
// degrades to a list.  So walk it: parent links must match, a red node may
// not have a red child, every root-to-leaf path must have the same number of
// black nodes, and the keys must respect the search order.  Returns the black
// height so the caller can compare siblings; a null leaf counts as black.
int CheckRedBlack(const TestNode *node, const TestNode *parent,
                  const TestNode *lower, const TestNode *upper) {
  if (node == nullptr) {
    return 1;
  }
  EXPECT_EQ(node->parent, parent)
      << ": node " << node->key << " has the wrong parent link";
  if (lower != nullptr) {
    EXPECT_LT(lower->key, node->key)
        << ": node " << node->key << " is out of order under " << lower->key;
  }
  if (upper != nullptr) {
    EXPECT_LT(node->key, upper->key)
        << ": node " << node->key << " is out of order under " << upper->key;
  }
  if (node->red) {
    EXPECT_FALSE(node->left != nullptr && node->left->red)
        << ": red node " << node->key << " has a red left child";
    EXPECT_FALSE(node->right != nullptr && node->right->red)
        << ": red node " << node->key << " has a red right child";
  }
  const int left = CheckRedBlack(node->left, node, lower, node);
  const int right = CheckRedBlack(node->right, node, node, upper);
  EXPECT_EQ(left, right) << ": black height differs under " << node->key;
  return left + (node->red ? 0 : 1);
}

// The tree does not hand out its root -- nothing but the tree itself has any
// business with it -- so walk up to it from a node the tree does hand out.
const TestNode *RootOf(const TestNode *node) {
  if (node == nullptr) {
    return nullptr;
  }
  while (node->parent != nullptr) {
    node = node->parent;
  }
  return node;
}

// Everything the tree holds, in iteration order.
std::vector<int> Keys(const Tree &tree) {
  std::vector<int> keys;
  for (TestNode *node : tree) {
    keys.push_back(node->key);
  }
  return keys;
}

// Walks the whole tree through the public interface and checks it against
// what it should contain, then checks the shape.  front() and Next() are
// checked to agree with begin()/end(), since callers use both.
void ExpectHolds(const Tree &tree, std::vector<int> expected) {
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(Keys(tree), expected);

  std::vector<int> walked;
  for (TestNode *node = tree.front(); node != nullptr;
       node = Tree::Next(node)) {
    walked.push_back(node->key);
  }
  EXPECT_EQ(walked, expected)
      << ": front()/Next() disagrees with begin()/end()";

  EXPECT_EQ(tree.empty(), expected.empty());
  if (expected.empty()) {
    EXPECT_EQ(tree.front(), nullptr);
  } else {
    ASSERT_NE(tree.front(), nullptr);
    EXPECT_EQ(tree.front()->key, expected.front());
  }

  // Every key present must be findable, and Find() must agree with the node
  // iteration handed back.
  for (int key : expected) {
    TestNode *found = tree.Find(key);
    ASSERT_NE(found, nullptr) << ": Find(" << key << ") missed";
    EXPECT_EQ(found->key, key);
  }
}

// Builds nodes that stay alive for the caller's whole test.
class NodePool {
 public:
  TestNode *Make(int key) {
    nodes_.push_back(std::make_unique<TestNode>(key));
    return nodes_.back().get();
  }

 private:
  std::vector<std::unique_ptr<TestNode>> nodes_;
};

TEST(IntrusiveRbTreeTest, EmptyHasNoFront) {
  Tree tree;
  EXPECT_TRUE(tree.empty());
  EXPECT_EQ(tree.front(), nullptr);
  EXPECT_EQ(tree.begin(), tree.end());
  EXPECT_EQ(tree.Find(0), nullptr);
}

TEST(IntrusiveRbTreeTest, HoldsOneNode) {
  NodePool pool;
  Tree tree;
  TestNode *only = pool.Make(7);
  tree.Insert(only);

  ExpectHolds(tree, {7});
  EXPECT_EQ(tree.front(), only);
  EXPECT_EQ(Tree::Next(only), nullptr) << ": Next() past the last node";
}

TEST(IntrusiveRbTreeTest, OrdersByTheTraitsKey) {
  NodePool pool;
  Tree tree;
  for (int key : {5, 1, 9, 3, 7}) {
    tree.Insert(pool.Make(key));
  }
  ExpectHolds(tree, {1, 3, 5, 7, 9});
  CheckRedBlack(RootOf(tree.front()), nullptr, nullptr, nullptr);
}

TEST(IntrusiveRbTreeTest, FindMissesWhatIsNotThere) {
  NodePool pool;
  Tree tree;
  for (int key : {10, 20, 30}) {
    tree.Insert(pool.Make(key));
  }
  EXPECT_EQ(tree.Find(15), nullptr);
  EXPECT_EQ(tree.Find(0), nullptr);
  EXPECT_EQ(tree.Find(40), nullptr);
  ASSERT_NE(tree.Find(20), nullptr);
  EXPECT_EQ(tree.Find(20)->key, 20);
}

TEST(IntrusiveRbTreeTest, FindStopsSeeingARemovedNode) {
  NodePool pool;
  Tree tree;
  TestNode *middle = pool.Make(20);
  tree.Insert(pool.Make(10));
  tree.Insert(middle);
  tree.Insert(pool.Make(30));

  ASSERT_NE(tree.Find(20), nullptr);
  tree.Remove(middle);
  EXPECT_EQ(tree.Find(20), nullptr);
  ExpectHolds(tree, {10, 30});
}

TEST(IntrusiveRbTreeTest, RemovingTheFrontMovesIt) {
  NodePool pool;
  Tree tree;
  TestNode *first = pool.Make(1);
  TestNode *second = pool.Make(2);
  tree.Insert(first);
  tree.Insert(second);

  EXPECT_EQ(tree.front(), first);
  tree.Remove(first);
  EXPECT_EQ(tree.front(), second);
  ExpectHolds(tree, {2});
}

TEST(IntrusiveRbTreeTest, InsertingSmallerMovesTheFront) {
  NodePool pool;
  Tree tree;
  tree.Insert(pool.Make(5));
  ASSERT_NE(tree.front(), nullptr);
  EXPECT_EQ(tree.front()->key, 5);

  tree.Insert(pool.Make(3));
  EXPECT_EQ(tree.front()->key, 3);
  tree.Insert(pool.Make(9));
  EXPECT_EQ(tree.front()->key, 3) << ": a larger key must not move the front";
}

// The three shapes CLRS deletion has to tell apart: no child, one child, and
// two children (where a successor is spliced in).  Removing each in turn from
// the same tree walks all of them.
TEST(IntrusiveRbTreeTest, RemovesFromEveryPosition) {
  NodePool pool;
  std::vector<int> keys = {50, 25, 75, 10, 30, 60, 90, 5, 15, 27, 35};
  for (size_t i = 0; i < keys.size(); ++i) {
    Tree tree;
    std::vector<TestNode *> nodes;
    for (int key : keys) {
      nodes.push_back(pool.Make(key));
      tree.Insert(nodes.back());
    }

    tree.Remove(nodes[i]);
    std::vector<int> expected = keys;
    expected.erase(expected.begin() + i);
    ExpectHolds(tree, expected);

    // Drain, so nothing is left pointing into the pool's next iteration.
    while (TestNode *node = tree.front()) {
      tree.Remove(node);
    }
  }
}

TEST(IntrusiveRbTreeTest, RemoveClearsTheLinksSoANodeCanGoBackIn) {
  NodePool pool;
  Tree tree;
  TestNode *node = pool.Make(2);
  tree.Insert(pool.Make(1));
  tree.Insert(node);
  tree.Insert(pool.Make(3));

  tree.Remove(node);
  EXPECT_EQ(node->left, nullptr);
  EXPECT_EQ(node->right, nullptr);
  EXPECT_EQ(node->parent, nullptr);
  EXPECT_FALSE(node->red);

  // Which is exactly what Insert()'s CHECKs demand, so this must work.
  tree.Insert(node);
  ExpectHolds(tree, {1, 2, 3});
}

TEST(IntrusiveRbTreeTest, SurvivesEmptying) {
  NodePool pool;
  Tree tree;
  std::vector<TestNode *> nodes;
  for (int key : {4, 2, 6, 1, 3, 5, 7}) {
    nodes.push_back(pool.Make(key));
    tree.Insert(nodes.back());
  }
  for (TestNode *node : nodes) {
    tree.Remove(node);
  }
  EXPECT_TRUE(tree.empty());
  EXPECT_EQ(tree.front(), nullptr);
  EXPECT_EQ(tree.begin(), tree.end());
  EXPECT_EQ(tree.Find(4), nullptr);
}

// Sorted input is what turns an unbalanced tree into a linked list, so the
// black-height check is the point of this one.
TEST(IntrusiveRbTreeTest, StaysBalancedOnSortedInput) {
  for (bool ascending : {true, false}) {
    NodePool pool;
    Tree tree;
    std::vector<int> keys;
    for (int i = 0; i < 128; ++i) {
      keys.push_back(ascending ? i : 127 - i);
    }
    for (int key : keys) {
      tree.Insert(pool.Make(key));
    }
    ExpectHolds(tree, keys);
    CheckRedBlack(RootOf(tree.front()), nullptr, nullptr, nullptr);
  }
}

TEST(IntrusiveRbTreeTest, MatchesAStdSetOnRandomInput) {
  NodePool pool;
  Tree tree;
  std::mt19937 gen(1234);
  constexpr int kKeySpace = 200;
  std::uniform_int_distribution<int> key_dist(0, kKeySpace - 1);

  std::set<int> model;
  std::vector<TestNode *> live;

  for (int i = 0; i < 2000; ++i) {
    // Insert unless the tree is full -- the order the tree documents is
    // total, so the model must never hold a key twice, and once every key is
    // taken the only legal move is a removal.
    const bool insert =
        live.empty() ||
        (static_cast<int>(model.size()) < kKeySpace && (gen() % 3) != 0);
    if (insert) {
      // Probe upward from a random key rather than redrawing, which would
      // spin for as long as the tree stays nearly full.
      int key = key_dist(gen);
      while (model.count(key) != 0) {
        key = (key + 1) % kKeySpace;
      }
      TestNode *node = pool.Make(key);
      tree.Insert(node);
      live.push_back(node);
      model.insert(key);
    } else {
      const size_t index = gen() % live.size();
      TestNode *node = live[index];
      model.erase(node->key);
      tree.Remove(node);
      live.erase(live.begin() + index);
    }

    if (i % 100 == 0) {
      CheckRedBlack(RootOf(tree.front()), nullptr, nullptr, nullptr);
      EXPECT_EQ(Keys(tree), std::vector<int>(model.begin(), model.end()));
    }
  }

  ExpectHolds(tree, std::vector<int>(model.begin(), model.end()));
}

TEST(IntrusiveRbTreeDeathTest, RefusesToInsertANodeAlreadyInATree) {
  NodePool pool;
  Tree tree;
  TestNode *first = pool.Make(1);
  TestNode *second = pool.Make(2);
  tree.Insert(first);

  // The whole tree: no links set, so only the root check can catch it.
  EXPECT_DEATH(tree.Insert(first), "already in this tree");

  tree.Insert(second);
  // Now linked to a parent, which the link checks catch first.
  EXPECT_DEATH(tree.Insert(second), "already in a tree");
}

TEST(IntrusiveRbTreeDeathTest, RefusesToRemoveANodeThatIsNotInTheTree) {
  NodePool pool;
  Tree tree;
  tree.Insert(pool.Make(1));
  TestNode *stranger = pool.Make(2);
  EXPECT_DEATH(tree.Remove(stranger), "not in this tree");
}

}  // namespace
}  // namespace aos::testing
