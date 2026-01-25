#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Vantage-point tree (VP-tree)
//
// A VP-tree is a metric tree that supports efficient k-nearest-neighbor queries
// in any space with a valid metric distance function.
//
// We use this for mining analytics (novelty/outlier detection) where we often
// need to find kNN repeatedly over thousands of MineRecords.
//
// Design goals:
// - Header-only, no external dependencies.
// - Deterministic construction (tie-break on index).
// - Deterministic kNN results when distances tie.
// -----------------------------------------------------------------------------

template <typename DistFn>
class VPTree {
public:
  // `items` are integer IDs understood by the provided `dist` function.
  explicit VPTree(std::vector<int> items, DistFn dist)
      : m_dist(std::move(dist))
  {
    m_nodes.reserve(items.size());
    m_root = build(items);
  }

  bool empty() const { return m_root < 0; }

  // Return up to k nearest neighbors of `target` as (distance, id) pairs sorted
  // by ascending distance then ascending id.
  std::vector<std::pair<double, int>> kNearest(int target, int k) const
  {
    std::vector<std::pair<double, int>> out;
    if (k <= 0) return out;
    if (m_root < 0) return out;

    std::priority_queue<std::pair<double, int>> heap; // max-heap: farthest at top
    double tau = std::numeric_limits<double>::infinity();
    search(m_root, target, k, heap, tau);

    out.reserve(heap.size());
    while (!heap.empty()) {
      out.push_back(heap.top());
      heap.pop();
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
      if (a.first < b.first) return true;
      if (a.first > b.first) return false;
      return a.second < b.second;
    });
    return out;
  }

private:
  struct Node {
    int vp = -1;
    double threshold = 0.0;
    int left = -1;
    int right = -1;
  };

  DistFn m_dist;
  std::vector<Node> m_nodes;
  int m_root = -1;

  int build(std::vector<int>& items)
  {
    if (items.empty()) return -1;

    const int nodeId = static_cast<int>(m_nodes.size());
    m_nodes.push_back(Node{});

    Node& n = m_nodes.back();

    // Deterministic choice of vantage point: take the last element.
    n.vp = items.back();
    items.pop_back();

    if (items.empty()) {
      n.threshold = 0.0;
      n.left = -1;
      n.right = -1;
      return nodeId;
    }

    // Compute distances to the vantage point.
    std::vector<std::pair<double, int>> dists;
    dists.reserve(items.size());
    for (int id : items) {
      dists.emplace_back(m_dist(n.vp, id), id);
    }

    // Deterministic median split: sort with tie-break on id.
    std::sort(dists.begin(), dists.end(), [](const auto& a, const auto& b) {
      if (a.first < b.first) return true;
      if (a.first > b.first) return false;
      return a.second < b.second;
    });

    const std::size_t median = dists.size() / 2;
    n.threshold = dists[median].first;

    std::vector<int> inner;
    std::vector<int> outer;
    inner.reserve(median);
    outer.reserve(dists.size() - median);

    for (std::size_t i = 0; i < dists.size(); ++i) {
      if (i < median) {
        inner.push_back(dists[i].second);
      } else {
        outer.push_back(dists[i].second);
      }
    }

    n.left = build(inner);
    n.right = build(outer);
    return nodeId;
  }

  static bool BetterNeighbor(const std::pair<double, int>& a, const std::pair<double, int>& b)
  {
    // Return true if a is strictly better (closer, or tie with smaller id).
    if (a.first < b.first) return true;
    if (a.first > b.first) return false;
    return a.second < b.second;
  }

  void maybeAddNeighbor(std::priority_queue<std::pair<double, int>>& heap,
                        int k,
                        const std::pair<double, int>& cand,
                        double& tau) const
  {
    if (static_cast<int>(heap.size()) < k) {
      heap.push(cand);
      if (static_cast<int>(heap.size()) == k) tau = heap.top().first;
      return;
    }

    const std::pair<double, int> worst = heap.top(); // farthest
    if (BetterNeighbor(cand, worst)) {
      heap.pop();
      heap.push(cand);
      tau = heap.top().first;
    }
  }

  void search(int nodeId,
              int target,
              int k,
              std::priority_queue<std::pair<double, int>>& heap,
              double& tau) const
  {
    if (nodeId < 0) return;
    const Node& n = m_nodes[static_cast<std::size_t>(nodeId)];

    const double dist = m_dist(target, n.vp);

    if (n.vp != target) {
      maybeAddNeighbor(heap, k, {dist, n.vp}, tau);
    }

    // Leaf?
    if (n.left < 0 && n.right < 0) return;

    const bool searchInnerFirst = dist < n.threshold;

    if (searchInnerFirst) {
      if (n.left >= 0) search(n.left, target, k, heap, tau);
      if (n.right >= 0 && dist + tau >= n.threshold) search(n.right, target, k, heap, tau);
    } else {
      if (n.right >= 0) search(n.right, target, k, heap, tau);
      if (n.left >= 0 && dist - tau <= n.threshold) search(n.left, target, k, heap, tau);
    }
  }
};

} // namespace isocity
