#include "isocity/EditHistory.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>

namespace {

// Local road auto-tiling fixup.
//
// World stores the road connection mask in the low 4 bits of Tile::variation.
// Historically EditHistory called World::recomputeRoadMasks() after undo/redo.
// That is correct but O(map) per command.
//
// To keep undo/redo fast, we update masks only around tiles that changed in a
// way that could affect road connectivity.
inline void ApplyRoadMaskLocal(isocity::World& world, int x, int y)
{
  using namespace isocity;
  if (!world.inBounds(x, y)) return;

  Tile& t = world.at(x, y);
  if (t.overlay != Overlay::Road) return;

  // Bit layout matches World::computeRoadMask():
  //  bit0: (x, y-1)
  //  bit1: (x+1, y)
  //  bit2: (x, y+1)
  //  bit3: (x-1, y)
  std::uint8_t m = 0;
  if (world.inBounds(x, y - 1) && world.at(x, y - 1).overlay == Overlay::Road) m |= 1u << 0;
  if (world.inBounds(x + 1, y) && world.at(x + 1, y).overlay == Overlay::Road) m |= 1u << 1;
  if (world.inBounds(x, y + 1) && world.at(x, y + 1).overlay == Overlay::Road) m |= 1u << 2;
  if (world.inBounds(x - 1, y) && world.at(x - 1, y).overlay == Overlay::Road) m |= 1u << 3;

  // Preserve upper bits for stable per-tile lighting variation.
  t.variation = static_cast<std::uint8_t>((t.variation & 0xF0u) | (m & 0x0Fu));
}

inline void UpdateRoadMasksAroundLocal(isocity::World& world, int x, int y)
{
  ApplyRoadMaskLocal(world, x, y);
  ApplyRoadMaskLocal(world, x, y - 1);
  ApplyRoadMaskLocal(world, x + 1, y);
  ApplyRoadMaskLocal(world, x, y + 1);
  ApplyRoadMaskLocal(world, x - 1, y);
}

} // namespace

namespace isocity {

void EditHistory::clear()
{
  m_strokeActive = false;
  m_strokeW = 0;
  m_strokeH = 0;
  m_moneyBefore = 0;
  m_visited.clear();
  m_indices.clear();
  m_before.clear();
  m_undo.clear();
  m_redo.clear();
}

bool EditHistory::tilesEqual(const Tile& a, const Tile& b)
{
  return a.terrain == b.terrain && a.overlay == b.overlay && a.height == b.height && a.variation == b.variation &&
         a.level == b.level && a.occupants == b.occupants && a.district == b.district;
}

void EditHistory::beginStroke(const World& world)
{
  m_strokeActive = true;
  m_strokeW = world.width();
  m_strokeH = world.height();
  m_moneyBefore = world.stats().money;

  const std::size_t n = static_cast<std::size_t>(std::max(0, m_strokeW) * std::max(0, m_strokeH));
  m_visited.assign(n, 0);
  m_indices.clear();
  m_before.clear();
}

void EditHistory::noteTilePreEdit(const World& world, int x, int y)
{
  if (!m_strokeActive) return;
  if (world.width() != m_strokeW || world.height() != m_strokeH) return;
  if (!world.inBounds(x, y)) return;

  const int idx = y * m_strokeW + x;
  if (idx < 0) return;
  const std::size_t uidx = static_cast<std::size_t>(idx);
  if (uidx >= m_visited.size()) return;

  if (m_visited[uidx]) return;
  m_visited[uidx] = 1;

  m_indices.push_back(idx);
  m_before.push_back(world.at(x, y));
}

void EditHistory::endStroke(World& world)
{
  (void)endStroke(world, nullptr);
}

bool EditHistory::endStroke(World& world, Command* outCmd)
{
  if (outCmd) *outCmd = Command{};

  if (!m_strokeActive) return false;
  m_strokeActive = false;

  if (world.width() != m_strokeW || world.height() != m_strokeH) {
    // World resized; discard.
    m_indices.clear();
    m_before.clear();
    m_visited.clear();
    return false;
  }

  Command cmd;
  cmd.moneyDelta = world.stats().money - m_moneyBefore;

  cmd.tiles.reserve(m_indices.size());
  for (std::size_t i = 0; i < m_indices.size(); ++i) {
    const int idx = m_indices[i];
    const int x = idx % m_strokeW;
    const int y = idx / m_strokeW;
    if (!world.inBounds(x, y)) continue;

    const Tile& before = m_before[i];
    const Tile after = world.at(x, y);
    if (tilesEqual(before, after)) continue;
    cmd.tiles.push_back(TileChange{x, y, before, after});
  }

  m_indices.clear();
  m_before.clear();
  m_visited.clear();

  if (cmd.tiles.empty() && cmd.moneyDelta == 0) return false;

  if (outCmd) *outCmd = cmd;

  m_undo.push_back(std::move(cmd));
  m_redo.clear();

  // Cap history to avoid unbounded memory.
  if (m_undo.size() > m_maxCommands) {
    const std::size_t extra = m_undo.size() - m_maxCommands;
    m_undo.erase(m_undo.begin(), m_undo.begin() + static_cast<std::ptrdiff_t>(extra));
  }

  return true;
}

bool EditHistory::undo(World& world)
{
  return undo(world, nullptr);
}

bool EditHistory::undo(World& world, Command* outCmd)
{
  if (outCmd) *outCmd = Command{};
  if (m_undo.empty()) return false;

  Command cmd = std::move(m_undo.back());
  m_undo.pop_back();

  // Track which locations might affect road auto-tiling.
  std::vector<std::pair<int, int>> roadTouches;
  roadTouches.reserve(cmd.tiles.size());
  for (const TileChange& c : cmd.tiles) {
    if (c.before.overlay == Overlay::Road || c.after.overlay == Overlay::Road) {
      roadTouches.emplace_back(c.x, c.y);
    }
  }

  for (const TileChange& c : cmd.tiles) {
    if (!world.inBounds(c.x, c.y)) continue;
    world.at(c.x, c.y) = c.before;
  }

  // Keep road auto-tiling masks consistent without a full recompute.
  for (const auto& p : roadTouches) {
    UpdateRoadMasksAroundLocal(world, p.first, p.second);
  }

  world.stats().money -= cmd.moneyDelta; // reverse

  if (outCmd) *outCmd = cmd;

  m_redo.push_back(std::move(cmd));
  return true;
}

bool EditHistory::redo(World& world)
{
  return redo(world, nullptr);
}

bool EditHistory::redo(World& world, Command* outCmd)
{
  if (outCmd) *outCmd = Command{};
  if (m_redo.empty()) return false;

  Command cmd = std::move(m_redo.back());
  m_redo.pop_back();

  std::vector<std::pair<int, int>> roadTouches;
  roadTouches.reserve(cmd.tiles.size());
  for (const TileChange& c : cmd.tiles) {
    if (c.before.overlay == Overlay::Road || c.after.overlay == Overlay::Road) {
      roadTouches.emplace_back(c.x, c.y);
    }
  }

  for (const TileChange& c : cmd.tiles) {
    if (!world.inBounds(c.x, c.y)) continue;
    world.at(c.x, c.y) = c.after;
  }

  for (const auto& p : roadTouches) {
    UpdateRoadMasksAroundLocal(world, p.first, p.second);
  }

  world.stats().money += cmd.moneyDelta;

  if (outCmd) *outCmd = cmd;

  m_undo.push_back(std::move(cmd));
  return true;
}

} // namespace isocity
