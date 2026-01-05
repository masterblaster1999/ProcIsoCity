#include "isocity/GfxPacker.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>

namespace isocity {

namespace {

struct Rect {
  int x = 0;
  int y = 0;
  int w = 0;
  int h = 0;
};

inline bool Intersects(const Rect& a, const Rect& b)
{
  if (a.x >= b.x + b.w) return false;
  if (a.x + a.w <= b.x) return false;
  if (a.y >= b.y + b.h) return false;
  if (a.y + a.h <= b.y) return false;
  return true;
}

inline bool Contains(const Rect& a, const Rect& b)
{
  return b.x >= a.x && b.y >= a.y && (b.x + b.w) <= (a.x + a.w) && (b.y + b.h) <= (a.y + a.h);
}

bool SplitFreeNode(const Rect& freeNode, const Rect& usedNode, std::vector<Rect>& outNew)
{
  if (!Intersects(freeNode, usedNode)) return false;

  const int freeRight = freeNode.x + freeNode.w;
  const int freeBottom = freeNode.y + freeNode.h;
  const int usedRight = usedNode.x + usedNode.w;
  const int usedBottom = usedNode.y + usedNode.h;

  // New node at the left side of the used node.
  if (usedNode.x > freeNode.x) {
    Rect r;
    r.x = freeNode.x;
    r.y = freeNode.y;
    r.w = usedNode.x - freeNode.x;
    r.h = freeNode.h;
    if (r.w > 0 && r.h > 0) outNew.push_back(r);
  }

  // New node at the right side of the used node.
  if (usedRight < freeRight) {
    Rect r;
    r.x = usedRight;
    r.y = freeNode.y;
    r.w = freeRight - usedRight;
    r.h = freeNode.h;
    if (r.w > 0 && r.h > 0) outNew.push_back(r);
  }

  // New node at the top side of the used node.
  if (usedNode.y > freeNode.y) {
    Rect r;
    r.x = freeNode.x;
    r.y = freeNode.y;
    r.w = freeNode.w;
    r.h = usedNode.y - freeNode.y;
    if (r.w > 0 && r.h > 0) outNew.push_back(r);
  }

  // New node at the bottom side of the used node.
  if (usedBottom < freeBottom) {
    Rect r;
    r.x = freeNode.x;
    r.y = usedBottom;
    r.w = freeNode.w;
    r.h = freeBottom - usedBottom;
    if (r.w > 0 && r.h > 0) outNew.push_back(r);
  }

  return true;
}

void PruneFreeList(std::vector<Rect>& ioFree)
{
  // Remove any free rect that is fully contained in another.
  for (std::size_t i = 0; i < ioFree.size(); ++i) {
    for (std::size_t j = i + 1; j < ioFree.size(); /* increment inside */) {
      if (Contains(ioFree[i], ioFree[j])) {
        ioFree.erase(ioFree.begin() + static_cast<std::ptrdiff_t>(j));
        continue;
      }
      if (Contains(ioFree[j], ioFree[i])) {
        ioFree.erase(ioFree.begin() + static_cast<std::ptrdiff_t>(i));
        --i;
        break;
      }
      ++j;
    }
  }
}

bool FindPositionBestShortSideFit(const std::vector<Rect>& freeRects, int w, int h, Rect& outNode)
{
  int bestShort = std::numeric_limits<int>::max();
  int bestLong = std::numeric_limits<int>::max();
  bool found = false;

  for (const Rect& r : freeRects) {
    if (w <= r.w && h <= r.h) {
      const int leftoverHoriz = r.w - w;
      const int leftoverVert = r.h - h;
      const int shortSideFit = std::min(leftoverHoriz, leftoverVert);
      const int longSideFit = std::max(leftoverHoriz, leftoverVert);
      if (shortSideFit < bestShort || (shortSideFit == bestShort && longSideFit < bestLong)) {
        outNode = Rect{r.x, r.y, w, h};
        bestShort = shortSideFit;
        bestLong = longSideFit;
        found = true;
      }
    }
  }

  return found;
}

} // namespace

bool PackMaxRects(int binW, int binH, std::vector<GfxPackRect>& ioRects, std::string& outError)
{
  outError.clear();

  if (binW <= 0 || binH <= 0) {
    outError = "invalid bin size";
    return false;
  }

  // Validate sizes.
  int maxW = 0;
  int maxH = 0;
  for (const auto& r : ioRects) {
    if (r.w <= 0 || r.h <= 0) {
      outError = "invalid rect size";
      return false;
    }
    maxW = std::max(maxW, r.w);
    maxH = std::max(maxH, r.h);
  }
  if (maxW > binW || maxH > binH) {
    outError = "rect does not fit in bin";
    return false;
  }

  // Deterministic insertion order: bigger area first, then bigger max dimension, then id.
  std::vector<std::size_t> order;
  order.reserve(ioRects.size());
  for (std::size_t i = 0; i < ioRects.size(); ++i) order.push_back(i);
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    const auto& ra = ioRects[a];
    const auto& rb = ioRects[b];
    const long long aa = static_cast<long long>(ra.w) * static_cast<long long>(ra.h);
    const long long ab = static_cast<long long>(rb.w) * static_cast<long long>(rb.h);
    if (aa != ab) return aa > ab;
    const int ma = std::max(ra.w, ra.h);
    const int mb = std::max(rb.w, rb.h);
    if (ma != mb) return ma > mb;
    return ra.id < rb.id;
  });

  std::vector<Rect> freeRects;
  freeRects.push_back(Rect{0, 0, binW, binH});

  for (std::size_t oi = 0; oi < order.size(); ++oi) {
    const std::size_t idx = order[oi];
    const int w = ioRects[idx].w;
    const int h = ioRects[idx].h;

    Rect node;
    if (!FindPositionBestShortSideFit(freeRects, w, h, node)) {
      outError = "failed to pack rect id=" + std::to_string(ioRects[idx].id);
      return false;
    }

    // Place it.
    ioRects[idx].x = node.x;
    ioRects[idx].y = node.y;

    // Split any free rects that intersect this placement.
    std::vector<Rect> newFree;
    newFree.reserve(freeRects.size() * 2);

    for (std::size_t i = 0; i < freeRects.size(); ++i) {
      const Rect fr = freeRects[i];
      if (!Intersects(fr, node)) {
        newFree.push_back(fr);
        continue;
      }
      std::vector<Rect> splits;
      SplitFreeNode(fr, node, splits);
      for (const Rect& s : splits) newFree.push_back(s);
    }

    freeRects.swap(newFree);
    PruneFreeList(freeRects);
  }

  return true;
}

bool PackShelf(int binW, std::vector<GfxPackRect>& ioRects, int& outUsedH, std::string& outError)
{
  outError.clear();
  outUsedH = 0;

  if (binW <= 0) {
    outError = "invalid binW";
    return false;
  }

  // Deterministic insertion order: taller first, then wider, then id.
  std::vector<std::size_t> order;
  order.reserve(ioRects.size());
  for (std::size_t i = 0; i < ioRects.size(); ++i) order.push_back(i);
  std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    const auto& ra = ioRects[a];
    const auto& rb = ioRects[b];
    if (ra.h != rb.h) return ra.h > rb.h;
    if (ra.w != rb.w) return ra.w > rb.w;
    return ra.id < rb.id;
  });

  int x = 0;
  int y = 0;
  int rowH = 0;

  for (std::size_t oi = 0; oi < order.size(); ++oi) {
    const std::size_t idx = order[oi];
    const int w = ioRects[idx].w;
    const int h = ioRects[idx].h;
    if (w > binW) {
      outError = "rect wider than bin";
      return false;
    }

    if (x + w > binW) {
      x = 0;
      y += rowH;
      rowH = 0;
    }

    ioRects[idx].x = x;
    ioRects[idx].y = y;

    x += w;
    rowH = std::max(rowH, h);
    outUsedH = std::max(outUsedH, y + rowH);
  }

  return true;
}

} // namespace isocity
