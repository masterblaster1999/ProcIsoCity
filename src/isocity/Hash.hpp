#pragma once

#include <cstdint>

namespace isocity {

class World;
struct Stats;

// Stable, cross-platform (endianness-independent) 64-bit hashes for core state.
//
// Intended uses:
//  - deterministic regression tests ("same seed + same steps => same hash")
//  - headless tooling/CI to compare simulation outputs
//  - debugging save/load round-trips
//
// NOTE: The exact hash values are not a public API contract; they may change if
// the underlying serialized state changes. Tests should generally compare two
// runs of the same build rather than hard-coding constants.

std::uint64_t HashStats(const Stats& stats);

// Hash the full world tile grid plus (optionally) the current Stats snapshot.
std::uint64_t HashWorld(const World& world, bool includeStats = true);

} // namespace isocity
