#include "isocity/Contours.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace isocity {

namespace {

struct Key {
  std::int64_t x = 0;
  std::int64_t y = 0;

  bool operator==(const Key& o) const { return x == o.x && y == o.y; }
  bool operator!=(const Key& o) const { return !(*this == o); }
};

struct KeyHash {
  std::size_t operator()(const Key& k) const noexcept
  {
    // 64-bit mix to size_t.
    std::uint64_t h = static_cast<std::uint64_t>(k.x) * 0x9E3779B185EBCA87ull;
    h ^= static_cast<std::uint64_t>(k.y) + 0x9E3779B185EBCA87ull + (h << 6) + (h >> 2);
    return static_cast<std::size_t>(h);
  }
};

inline bool KeyLess(const Key& a, const Key& b)
{
  if (a.y != b.y) return a.y < b.y;
  return a.x < b.x;
}

struct Segment {
  Key a;
  Key b;
};

inline double Clamp01(double v)
{
  if (v < 0.0) return 0.0;
  if (v > 1.0) return 1.0;
  return v;
}

inline double Lerp(double a, double b, double t) { return a + (b - a) * t; }

inline FPoint Interp(const FPoint& p0, double v0, const FPoint& p1, double v1, double level)
{
  const double den = (v1 - v0);
  double t = 0.5;
  if (std::abs(den) > 1e-12) {
    t = (level - v0) / den;
  }
  t = Clamp01(t);
  return FPoint{Lerp(p0.x, p1.x, t), Lerp(p0.y, p1.y, t)};
}

inline double DistSq(const FPoint& a, const FPoint& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline double DistPointSegSq(const FPoint& p, const FPoint& a, const FPoint& b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double wx = p.x - a.x;
  const double wy = p.y - a.y;

  const double vv = vx * vx + vy * vy;
  if (vv <= 1e-18) return DistSq(p, a);

  double t = (wx * vx + wy * vy) / vv;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;

  const FPoint proj{a.x + vx * t, a.y + vy * t};
  return DistSq(p, proj);
}

std::vector<FPoint> SimplifyDouglasPeuckerOpen(const std::vector<FPoint>& in, double tol)
{
  if (in.size() <= 2 || tol <= 0.0) return in;

  const double tolSq = tol * tol;
  const int n = static_cast<int>(in.size());

  std::vector<std::uint8_t> keep(static_cast<std::size_t>(n), std::uint8_t{0});
  keep[0] = 1;
  keep[n - 1] = 1;

  struct Range {
    int a;
    int b;
  };

  std::vector<Range> stack;
  stack.push_back({0, n - 1});

  while (!stack.empty()) {
    const Range r = stack.back();
    stack.pop_back();

    if (r.b <= r.a + 1) continue;

    double best = -1.0;
    int bestIdx = -1;
    for (int i = r.a + 1; i < r.b; ++i) {
      const double d = DistPointSegSq(in[static_cast<std::size_t>(i)], in[static_cast<std::size_t>(r.a)], in[static_cast<std::size_t>(r.b)]);
      if (d > best) {
        best = d;
        bestIdx = i;
      }
    }

    if (bestIdx >= 0 && best > tolSq) {
      keep[static_cast<std::size_t>(bestIdx)] = 1;
      stack.push_back({r.a, bestIdx});
      stack.push_back({bestIdx, r.b});
    }
  }

  std::vector<FPoint> out;
  out.reserve(in.size());
  for (int i = 0; i < n; ++i) {
    if (keep[static_cast<std::size_t>(i)] != 0) out.push_back(in[static_cast<std::size_t>(i)]);
  }
  if (out.size() < 2) return in;
  return out;
}

std::vector<FPoint> SimplifyDouglasPeuckerClosed(const std::vector<FPoint>& in, double tol)
{
  if (in.size() <= 3 || tol <= 0.0) return in;

  // Expect a closed ring: last == first.
  if (!(in.front() == in.back())) return SimplifyDouglasPeuckerOpen(in, tol);

  // Remove the duplicate closing point.
  std::vector<FPoint> ring(in.begin(), in.end() - 1);
  if (ring.size() < 3) return in;

  // Pick a deterministic cut point (lexicographically minimal) so we can run the open simplifier.
  std::size_t cut = 0;
  for (std::size_t i = 1; i < ring.size(); ++i) {
    const FPoint& p = ring[i];
    const FPoint& q = ring[cut];
    if (p.y < q.y || (p.y == q.y && p.x < q.x)) cut = i;
  }

  std::vector<FPoint> open;
  open.reserve(ring.size() + 1);
  for (std::size_t i = 0; i < ring.size(); ++i) {
    open.push_back(ring[(cut + i) % ring.size()]);
  }
  // Open polyline endpoints are adjacent in the original ring; that's OK.
  open.push_back(open.front());

  std::vector<FPoint> simp = SimplifyDouglasPeuckerOpen(open, tol);
  if (simp.size() >= 2 && simp.front() == simp.back()) {
    // Ensure closure.
    return simp;
  }

  // Fall back: re-close.
  if (!simp.empty()) simp.push_back(simp.front());
  return simp;
}

} // namespace

std::vector<double> BuildCornerHeightGrid(const World& world, double heightScale)
{
  const int w = world.width();
  const int h = world.height();

  const int cw = w + 1;
  const int ch = h + 1;

  std::vector<double> out;
  out.assign(static_cast<std::size_t>(cw) * static_cast<std::size_t>(ch), 0.0);

  for (int cy = 0; cy < ch; ++cy) {
    for (int cx = 0; cx < cw; ++cx) {
      double sum = 0.0;
      int count = 0;

      // Adjacent tiles touching this corner: (cx-1,cy-1), (cx,cy-1), (cx-1,cy), (cx,cy)
      for (int dy = -1; dy <= 0; ++dy) {
        for (int dx = -1; dx <= 0; ++dx) {
          const int tx = cx + dx;
          const int ty = cy + dy;
          if (tx < 0 || ty < 0 || tx >= w || ty >= h) continue;
          sum += static_cast<double>(world.at(tx, ty).height) * heightScale;
          count++;
        }
      }

      if (count <= 0) count = 1;
      out[static_cast<std::size_t>(cy) * static_cast<std::size_t>(cw) + static_cast<std::size_t>(cx)] = sum / static_cast<double>(count);
    }
  }

  return out;
}

std::vector<ContourLevel> ExtractContours(const std::vector<double>& cornerValues,
                                         int cornerW,
                                         int cornerH,
                                         const std::vector<double>& levels,
                                         const ContourConfig& cfg,
                                         std::string* outError)
{
  if (outError) outError->clear();

  if (cornerW <= 1 || cornerH <= 1) {
    if (outError) *outError = "corner grid must be at least 2x2";
    return {};
  }
  if (static_cast<int>(cornerValues.size()) != cornerW * cornerH) {
    if (outError) *outError = "cornerValues size mismatch";
    return {};
  }

  const int cellW = cornerW - 1;
  const int cellH = cornerH - 1;

  std::vector<ContourLevel> out;
  out.reserve(levels.size());

  const double q = (cfg.quantize > 0.0) ? cfg.quantize : 1e-6;
  const double invQ = 1.0 / q;

  auto QuantizeKey = [&](const FPoint& p) -> Key {
    const std::int64_t qx = static_cast<std::int64_t>(std::llround(p.x * invQ));
    const std::int64_t qy = static_cast<std::int64_t>(std::llround(p.y * invQ));
    return Key{qx, qy};
  };

  auto KeyToPoint = [&](const Key& k) -> FPoint {
    return FPoint{static_cast<double>(k.x) * q, static_cast<double>(k.y) * q};
  };

  auto Corner = [&](int x, int y) -> double {
    return cornerValues[static_cast<std::size_t>(y) * static_cast<std::size_t>(cornerW) + static_cast<std::size_t>(x)];
  };

  // Pre-allocate per-level storage (keeps deterministic ordering).
  for (double level : levels) {
    ContourLevel cl;
    cl.level = level;

    // First, collect raw line segments (unordered).
    std::vector<Segment> segments;
    segments.reserve(static_cast<std::size_t>(cellW) * static_cast<std::size_t>(cellH) / 2u);

    std::unordered_map<Key, std::vector<int>, KeyHash> adj;
    adj.reserve(static_cast<std::size_t>(cellW) * static_cast<std::size_t>(cellH));

    // Marching squares over each cell.
    for (int y = 0; y < cellH; ++y) {
      for (int x = 0; x < cellW; ++x) {
        const double v0 = Corner(x, y);         // TL
        const double v1 = Corner(x + 1, y);     // TR
        const double v2 = Corner(x + 1, y + 1); // BR
        const double v3 = Corner(x, y + 1);     // BL

        // Bitmask of corners above level.
        int c = 0;
        if (v0 > level) c |= 1;
        if (v1 > level) c |= 2;
        if (v2 > level) c |= 4;
        if (v3 > level) c |= 8;

        if (c == 0 || c == 15) continue; // no crossings

        const FPoint p0{static_cast<double>(x), static_cast<double>(y)};
        const FPoint p1{static_cast<double>(x + 1), static_cast<double>(y)};
        const FPoint p2{static_cast<double>(x + 1), static_cast<double>(y + 1)};
        const FPoint p3{static_cast<double>(x), static_cast<double>(y + 1)};

        auto EdgePt = [&](int e) -> FPoint {
          switch (e) {
          case 0: // top TL->TR
            return Interp(p0, v0, p1, v1, level);
          case 1: // right TR->BR
            return Interp(p1, v1, p2, v2, level);
          case 2: // bottom BR->BL
            return Interp(p2, v2, p3, v3, level);
          case 3: // left BL->TL
            return Interp(p3, v3, p0, v0, level);
          default:
            return FPoint{0, 0};
          }
        };

        auto AddSeg = [&](int ea, int eb) {
          const FPoint a = EdgePt(ea);
          const FPoint b = EdgePt(eb);
          const Key ka = QuantizeKey(a);
          const Key kb = QuantizeKey(b);
          if (ka == kb) return; // degenerate

          const int idx = static_cast<int>(segments.size());
          segments.push_back(Segment{ka, kb});
          adj[ka].push_back(idx);
          adj[kb].push_back(idx);
        };

        switch (c) {
        case 1:  AddSeg(3, 0); break;
        case 2:  AddSeg(0, 1); break;
        case 3:  AddSeg(3, 1); break;
        case 4:  AddSeg(1, 2); break;
        case 5: {
          const double center = 0.25 * (v0 + v1 + v2 + v3);
          if (cfg.useAsymptoticDecider ? (center > level) : true) {
            // Connect around low corners (TR,BL): top->right and bottom->left.
            AddSeg(0, 1);
            AddSeg(2, 3);
          } else {
            // Connect around high corners (TL,BR): left->top and right->bottom.
            AddSeg(3, 0);
            AddSeg(1, 2);
          }
        } break;
        case 6:  AddSeg(0, 2); break;
        case 7:  AddSeg(3, 2); break;
        case 8:  AddSeg(2, 3); break;
        case 9:  AddSeg(0, 2); break;
        case 10: {
          const double center = 0.25 * (v0 + v1 + v2 + v3);
          if (cfg.useAsymptoticDecider ? (center > level) : true) {
            // Connect around low corners (TL,BR): top->left and right->bottom.
            AddSeg(0, 3);
            AddSeg(1, 2);
          } else {
            // Connect around high corners (TR,BL): top->right and bottom->left.
            AddSeg(0, 1);
            AddSeg(2, 3);
          }
        } break;
        case 11: AddSeg(1, 2); break;
        case 12: AddSeg(3, 1); break;
        case 13: AddSeg(0, 1); break;
        case 14: AddSeg(3, 0); break;
        default:
          break;
        }
      }
    }

    if (segments.empty()) {
      out.push_back(std::move(cl));
      continue;
    }

    // Trace polylines.
    std::vector<std::uint8_t> used(segments.size(), std::uint8_t{0});

    // Gather keys for deterministic traversal.
    std::vector<Key> keys;
    keys.reserve(adj.size());
    for (const auto& kv : adj) keys.push_back(kv.first);
    std::sort(keys.begin(), keys.end(), KeyLess);

    auto NextSegAt = [&](const Key& k) -> int {
      auto it = adj.find(k);
      if (it == adj.end()) return -1;
      const std::vector<int>& lst = it->second;
      int best = -1;
      for (int si : lst) {
        if (si < 0 || si >= static_cast<int>(used.size())) continue;
        if (used[static_cast<std::size_t>(si)] != 0) continue;
        if (best < 0 || si < best) best = si;
      }
      return best;
    };

    auto Other = [&](int segIdx, const Key& here) -> Key {
      const Segment& s = segments[static_cast<std::size_t>(segIdx)];
      return (s.a == here) ? s.b : s.a;
    };

    auto TraceFrom = [&](const Key& start) -> ContourPolyline {
      ContourPolyline poly;
      poly.closed = false;

      // Starting key.
      Key cur = start;
      poly.pts.push_back(KeyToPoint(cur));

      int prevSeg = -1;

      while (true) {
        auto it = adj.find(cur);
        if (it == adj.end()) break;

        // Choose the next unused segment. If there are multiple choices, pick the
        // smallest index that isn't the one we just came from.
        int next = -1;
        for (int si : it->second) {
          if (si < 0 || si >= static_cast<int>(used.size())) continue;
          if (used[static_cast<std::size_t>(si)] != 0) continue;
          if (prevSeg >= 0 && si == prevSeg) continue;
          if (next < 0 || si < next) next = si;
        }

        if (next < 0) {
          // If we couldn't find a segment excluding prevSeg, we might be at the
          // start of a loop where the only remaining segment is prevSeg.
          next = NextSegAt(cur);
        }

        if (next < 0) break;

        used[static_cast<std::size_t>(next)] = 1;
        const Key nxt = Other(next, cur);
        poly.pts.push_back(KeyToPoint(nxt));

        prevSeg = next;
        cur = nxt;

        if (cur == start) {
          poly.closed = true;
          break;
        }
      }

      // Ensure explicit closure point when closed.
      if (poly.closed) {
        if (poly.pts.empty() || !(poly.pts.front() == poly.pts.back())) poly.pts.push_back(poly.pts.front());
      }

      return poly;
    };

    // First, start traces at degree-1 endpoints (open contours).
    for (const Key& k : keys) {
      const auto it = adj.find(k);
      if (it == adj.end()) continue;
      const int deg = static_cast<int>(it->second.size());
      if (deg != 1) continue;

      if (NextSegAt(k) < 0) continue;
      ContourPolyline p = TraceFrom(k);
      if (cfg.simplifyTolerance > 0.0) {
        p.pts = SimplifyDouglasPeuckerOpen(p.pts, cfg.simplifyTolerance);
      }
      if (static_cast<int>(p.pts.size()) >= cfg.minPoints) cl.lines.push_back(std::move(p));
    }

    // Then, any remaining segments are loops (or more complex graphs). Trace them deterministically.
    for (const Key& k : keys) {
      while (NextSegAt(k) >= 0) {
        ContourPolyline p = TraceFrom(k);
        if (cfg.simplifyTolerance > 0.0) {
          p.pts = p.closed ? SimplifyDouglasPeuckerClosed(p.pts, cfg.simplifyTolerance)
                           : SimplifyDouglasPeuckerOpen(p.pts, cfg.simplifyTolerance);
        }
        if (static_cast<int>(p.pts.size()) >= cfg.minPoints) cl.lines.push_back(std::move(p));
      }
    }

    out.push_back(std::move(cl));
  }

  return out;
}

} // namespace isocity
