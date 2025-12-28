#pragma once

namespace isocity {

// Simple integer tile coordinate.
//
// Kept raylib-free so it can be used by the headless core (tests, sim, save/load).
struct Point {
  int x = 0;
  int y = 0;
};

} // namespace isocity
