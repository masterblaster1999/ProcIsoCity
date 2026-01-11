#pragma once

#include "isocity/World.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace isocity {

// Lightweight undo/redo history for map editing.
//
// Captures per-stroke tile diffs (before/after) plus the money delta caused
// by tool costs. It does not rewind simulation time; it simply applies tile
// edits and refunds/spends the recorded money delta.
//
// Notes:
// - Undo/redo performs a small local fixup for road auto-tiling masks (stored in
//   the low bits of Tile::variation) around any tiles that touched roads in the
//   command. This avoids an O(map) full recompute per undo/redo.
class EditHistory {
public:
  struct TileChange {
    int x = 0;
    int y = 0;
    Tile before{};
    Tile after{};
  };

  struct Command {
    std::vector<TileChange> tiles;
    int moneyDelta = 0; // afterMoney - beforeMoney for the stroke
  };

  void clear();

  // Stroke lifecycle.
  void beginStroke(const World& world);
  void noteTilePreEdit(const World& world, int x, int y);
  // Returns true and optionally outputs the committed command if a command was
  // recorded (i.e., there were tile changes and/or a money delta).
  bool endStroke(World& world, Command* outCmd);
  // Back-compat wrapper (ignores the committed command).
  void endStroke(World& world);
  bool strokeActive() const { return m_strokeActive; }

  // Undo/redo.
  bool canUndo() const { return !m_undo.empty(); }
  bool canRedo() const { return !m_redo.empty(); }
  // Optional outCmd allows callers (e.g. deterministic replay capture) to
  // record exactly what was applied.
  bool undo(World& world, Command* outCmd);
  bool redo(World& world, Command* outCmd);
  bool undo(World& world);
  bool redo(World& world);

  std::size_t undoSize() const { return m_undo.size(); }
  std::size_t redoSize() const { return m_redo.size(); }

private:
  static bool tilesEqual(const Tile& a, const Tile& b);

  // Pending stroke data.
  bool m_strokeActive = false;
  int m_strokeW = 0;
  int m_strokeH = 0;
  int m_moneyBefore = 0;
  std::vector<std::uint8_t> m_visited; // size = w*h
  std::vector<int> m_indices;
  std::vector<Tile> m_before;

  std::vector<Command> m_undo;
  std::vector<Command> m_redo;

  std::size_t m_maxCommands = 64;
};

} // namespace isocity
