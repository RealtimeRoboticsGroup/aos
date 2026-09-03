#ifndef AOS_EVENTS_INTRUSIVE_RB_TREE_H_
#define AOS_EVENTS_INTRUSIVE_RB_TREE_H_

#include <cstdint>

#include "absl/log/absl_check.h"

namespace aos {

// An ordered set whose links live on the elements themselves.
//
// Intrusive and allocation-free: inserting costs nothing but pointer writes,
// so a real-time thread can add to one.  That is the whole reason this exists
// rather than a std::set or a sorted std::vector -- both allocate, and the
// vector reallocates, on exactly the paths that must not.
//
// O(log n) to insert and to remove, O(1) to ask for the smallest element,
// with no case that degrades.  A sorted list is the obvious alternative and
// is O(1) at whichever end it searches from -- but only that end, and a
// caller that inserts at both ends pays O(n) for half its inserts.  A tree
// has no bad end to search from.
//
// Traits supplies the links:
//   static Node *&left(Node *node);
//   static Node *&right(Node *node);
//   static Node *&parent(Node *node);
//   static bool &red(Node *node);
//
// and the ordering:
//   static bool Less(Node *a, Node *b);
//   static int Compare(const Node *node, const Key &key);   // for Find()
//
// Less() must be a strict weak ordering, and a total one: two nodes that
// compare equal in both directions are not handled, so a caller whose natural
// key can tie needs a tiebreaker in it -- a set keyed on deadlines wants a
// sequence number beside them.
//
// left/right/parent/red belong to this class -- nothing else may read or
// write them while a node is in the tree.  A node's key must not change while
// it is in one either: the position is fixed at Insert() and never revisited.
// Changing a key means Remove(), then write it, then Insert().
template <typename Node, typename Traits>
class IntrusiveRbTree {
 public:
  bool empty() const { return root_ == nullptr; }

  // The smallest element, or nullptr when the tree is empty.
  Node *front() const { return min_; }

  // The next node in sort order, for a caller that needs to see more than the
  // front.
  static Node *Next(Node *node) {
    if (Traits::right(node) != nullptr) {
      return Leftmost(Traits::right(node));
    }
    Node *parent = Traits::parent(node);
    while (parent != nullptr && node == Traits::right(parent)) {
      node = parent;
      parent = Traits::parent(parent);
    }
    return parent;
  }

  // The node whose key compares equal to `key`, or nullptr.  Traits::Compare
  // returns <0 when the node sorts before the key, >0 after, 0 on a match --
  // so a user keyed on something cheap (an fd) can look one up without
  // building a whole Node to compare against.
  template <typename Key>
  Node *Find(const Key &key) const {
    Node *node = root_;
    while (node != nullptr) {
      const int direction = Traits::Compare(node, key);
      if (direction == 0) {
        return node;
      }
      node = direction < 0 ? Traits::right(node) : Traits::left(node);
    }
    return nullptr;
  }

  // Forward iteration in sort order, so callers that have to visit every node
  // can range-for.  O(1) amortised per step, no allocation.  Invalidated by
  // Remove() of the node it is sitting on, like any intrusive container.
  class iterator {
   public:
    explicit iterator(Node *node) : node_(node) {}
    Node *operator*() const { return node_; }
    iterator &operator++() {
      node_ = IntrusiveRbTree::Next(node_);
      return *this;
    }
    bool operator!=(const iterator &other) const {
      return node_ != other.node_;
    }
    bool operator==(const iterator &other) const {
      return node_ == other.node_;
    }

   private:
    Node *node_ = nullptr;
  };

  iterator begin() const { return iterator(min_); }
  iterator end() const { return iterator(nullptr); }

  // Adds node, which must not already be in a tree.
  void Insert(Node *node) {
    ABSL_CHECK(Traits::left(node) == nullptr)
        << ": inserting a node that is already in a tree";
    ABSL_CHECK(Traits::right(node) == nullptr)
        << ": inserting a node that is already in a tree";
    ABSL_CHECK(Traits::parent(node) == nullptr)
        << ": inserting a node that is already in a tree";
    // The three above all pass for a node that is the whole tree.
    ABSL_CHECK(root_ != node)
        << ": inserting a node that is already in this tree";

    Node *parent = nullptr;
    Node **link = &root_;
    while (*link != nullptr) {
      parent = *link;
      link =
          Less(node, parent) ? &Traits::left(parent) : &Traits::right(parent);
    }
    *link = node;
    Traits::parent(node) = parent;
    Traits::red(node) = true;

    if (min_ == nullptr || Less(node, min_)) {
      min_ = node;
    }
    InsertFixup(node);
  }

  // Removes node, which must be in this tree.
  void Remove(Node *node) {
    ABSL_CHECK(node == root_ || Traits::parent(node) != nullptr)
        << ": removing a node that is not in this tree";

    if (node == min_) {
      min_ = Next(node);
    }

    // CLRS deletion with nullptr for the leaves.  `child` takes the spliced
    // node's place and may be null, so its parent is tracked alongside -- that
    // is what the fixup needs and cannot ask a null child for.
    Node *spliced = node;
    bool spliced_was_red = Traits::red(spliced);
    Node *child = nullptr;
    Node *child_parent = nullptr;

    if (Traits::left(node) == nullptr) {
      child = Traits::right(node);
      child_parent = Traits::parent(node);
      Transplant(node, child);
    } else if (Traits::right(node) == nullptr) {
      child = Traits::left(node);
      child_parent = Traits::parent(node);
      Transplant(node, child);
    } else {
      spliced = Leftmost(Traits::right(node));
      spliced_was_red = Traits::red(spliced);
      child = Traits::right(spliced);
      if (Traits::parent(spliced) == node) {
        child_parent = spliced;
      } else {
        child_parent = Traits::parent(spliced);
        Transplant(spliced, child);
        Traits::right(spliced) = Traits::right(node);
        Traits::parent(Traits::right(spliced)) = spliced;
      }
      Transplant(node, spliced);
      Traits::left(spliced) = Traits::left(node);
      Traits::parent(Traits::left(spliced)) = spliced;
      Traits::red(spliced) = Traits::red(node);
    }

    if (!spliced_was_red) {
      RemoveFixup(child, child_parent);
    }

    Traits::left(node) = nullptr;
    Traits::right(node) = nullptr;
    Traits::parent(node) = nullptr;
    Traits::red(node) = false;
  }

 private:
  // Non-const because a Traits::Less() may reach accessors that hand back
  // writable references; nothing here modifies either node.
  static bool Less(Node *a, Node *b) { return Traits::Less(a, b); }

  static Node *Leftmost(Node *node) {
    while (Traits::left(node) != nullptr) {
      node = Traits::left(node);
    }
    return node;
  }

  // A null leaf counts as black.  That is what lets the fixups below do their
  // case analysis without a sentinel node to stand in for the leaves.
  static bool IsRed(Node *node) { return node != nullptr && Traits::red(node); }
  static void Paint(Node *node, bool red) {
    if (node != nullptr) {
      Traits::red(node) = red;
    }
  }

  void RotateLeft(Node *node) {
    Node *const pivot = Traits::right(node);
    Traits::right(node) = Traits::left(pivot);
    if (Traits::left(pivot) != nullptr) {
      Traits::parent(Traits::left(pivot)) = node;
    }
    Traits::parent(pivot) = Traits::parent(node);
    if (Traits::parent(node) == nullptr) {
      root_ = pivot;
    } else if (node == Traits::left(Traits::parent(node))) {
      Traits::left(Traits::parent(node)) = pivot;
    } else {
      Traits::right(Traits::parent(node)) = pivot;
    }
    Traits::left(pivot) = node;
    Traits::parent(node) = pivot;
  }

  void RotateRight(Node *node) {
    Node *const pivot = Traits::left(node);
    Traits::left(node) = Traits::right(pivot);
    if (Traits::right(pivot) != nullptr) {
      Traits::parent(Traits::right(pivot)) = node;
    }
    Traits::parent(pivot) = Traits::parent(node);
    if (Traits::parent(node) == nullptr) {
      root_ = pivot;
    } else if (node == Traits::right(Traits::parent(node))) {
      Traits::right(Traits::parent(node)) = pivot;
    } else {
      Traits::left(Traits::parent(node)) = pivot;
    }
    Traits::right(pivot) = node;
    Traits::parent(node) = pivot;
  }

  void InsertFixup(Node *node) {
    // A red node with a red parent is the only violation Insert() can make.
    while (IsRed(Traits::parent(node))) {
      Node *parent = Traits::parent(node);
      // A red parent is never the root, so a grandparent exists.
      Node *const grandparent = Traits::parent(parent);
      if (parent == Traits::left(grandparent)) {
        Node *const uncle = Traits::right(grandparent);
        if (IsRed(uncle)) {
          Traits::red(parent) = false;
          Paint(uncle, false);
          Traits::red(grandparent) = true;
          node = grandparent;
          continue;
        }
        if (node == Traits::right(parent)) {
          node = parent;
          RotateLeft(node);
          parent = Traits::parent(node);
        }
        Traits::red(parent) = false;
        Traits::red(Traits::parent(parent)) = true;
        RotateRight(Traits::parent(parent));
      } else {
        Node *const uncle = Traits::left(grandparent);
        if (IsRed(uncle)) {
          Traits::red(parent) = false;
          Paint(uncle, false);
          Traits::red(grandparent) = true;
          node = grandparent;
          continue;
        }
        if (node == Traits::left(parent)) {
          node = parent;
          RotateRight(node);
          parent = Traits::parent(node);
        }
        Traits::red(parent) = false;
        Traits::red(Traits::parent(parent)) = true;
        RotateLeft(Traits::parent(parent));
      }
    }
    Traits::red(root_) = false;
  }

  void Transplant(Node *node, Node *replacement) {
    Node *const parent = Traits::parent(node);
    if (parent == nullptr) {
      root_ = replacement;
    } else if (node == Traits::left(parent)) {
      Traits::left(parent) = replacement;
    } else {
      Traits::right(parent) = replacement;
    }
    if (replacement != nullptr) {
      Traits::parent(replacement) = parent;
    }
  }

  // `node` carries one extra black; push it up the tree until something can
  // absorb it.  `parent` comes in separately because node may be a null leaf.
  void RemoveFixup(Node *node, Node *parent) {
    while (node != root_ && !IsRed(node)) {
      if (node == Traits::left(parent)) {
        // Never null: node's side is a black short, so the other side has at
        // least one to spare.
        Node *sibling = Traits::right(parent);
        if (IsRed(sibling)) {
          Traits::red(sibling) = false;
          Traits::red(parent) = true;
          RotateLeft(parent);
          sibling = Traits::right(parent);
        }
        if (!IsRed(Traits::left(sibling)) && !IsRed(Traits::right(sibling))) {
          Traits::red(sibling) = true;
          node = parent;
          parent = Traits::parent(node);
          continue;
        }
        if (!IsRed(Traits::right(sibling))) {
          Paint(Traits::left(sibling), false);
          Traits::red(sibling) = true;
          RotateRight(sibling);
          sibling = Traits::right(parent);
        }
        Traits::red(sibling) = Traits::red(parent);
        Traits::red(parent) = false;
        Paint(Traits::right(sibling), false);
        RotateLeft(parent);
        node = root_;
        parent = nullptr;
      } else {
        Node *sibling = Traits::left(parent);
        if (IsRed(sibling)) {
          Traits::red(sibling) = false;
          Traits::red(parent) = true;
          RotateRight(parent);
          sibling = Traits::left(parent);
        }
        if (!IsRed(Traits::left(sibling)) && !IsRed(Traits::right(sibling))) {
          Traits::red(sibling) = true;
          node = parent;
          parent = Traits::parent(node);
          continue;
        }
        if (!IsRed(Traits::left(sibling))) {
          Paint(Traits::right(sibling), false);
          Traits::red(sibling) = true;
          RotateLeft(sibling);
          sibling = Traits::left(parent);
        }
        Traits::red(sibling) = Traits::red(parent);
        Traits::red(parent) = false;
        Paint(Traits::left(sibling), false);
        RotateRight(parent);
        node = root_;
        parent = nullptr;
      }
    }
    Paint(node, false);
  }

  Node *root_ = nullptr;
  // front(), kept on the side so asking is O(1) rather than a walk down the
  // left spine.
  Node *min_ = nullptr;
};

}  // namespace aos

#endif  // AOS_EVENTS_INTRUSIVE_RB_TREE_H_
