#include "isocity/EditHistory.hpp"

#include <algorithm>
#include <utility>

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
         a.level == b.level && a.occupants == b.occupants;
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
  if (!m_strokeActive) return;
  m_strokeActive = false;

  if (world.width() != m_strokeW || world.height() != m_strokeH) {
    // World resized; discard.
    m_indices.clear();
    m_before.clear();
    m_visited.clear();
    return;
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

  if (cmd.tiles.empty() && cmd.moneyDelta == 0) return;

  m_undo.push_back(std::move(cmd));
  m_redo.clear();

  // Cap history to avoid unbounded memory.
  if (m_undo.size() > m_maxCommands) {
    const std::size_t extra = m_undo.size() - m_maxCommands;
    m_undo.erase(m_undo.begin(), m_undo.begin() + static_cast<std::ptrdiff_t>(extra));
  }
}

bool EditHistory::undo(World& world)
{
  if (m_undo.empty()) return false;

  Command cmd = std::move(m_undo.back());
  m_undo.pop_back();

  for (const TileChange& c : cmd.tiles) {
    if (!world.inBounds(c.x, c.y)) continue;
    world.at(c.x, c.y) = c.before;
  }

  // Road auto-tiling masks depend on local connectivity; keep them consistent.
  world.recomputeRoadMasks();

  world.stats().money -= cmd.moneyDelta; // reverse

  m_redo.push_back(std::move(cmd));
  return true;
}

bool EditHistory::redo(World& world)
{
  if (m_redo.empty()) return false;

  Command cmd = std::move(m_redo.back());
  m_redo.pop_back();

  for (const TileChange& c : cmd.tiles) {
    if (!world.inBounds(c.x, c.y)) continue;
    world.at(c.x, c.y) = c.after;
  }

  world.recomputeRoadMasks();

  world.stats().money += cmd.moneyDelta;

  m_undo.push_back(std::move(cmd));
  return true;
}

} // namespace isocity
