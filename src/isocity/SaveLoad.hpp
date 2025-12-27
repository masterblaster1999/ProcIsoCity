#pragma once

#include "isocity/World.hpp"

#include <string>

namespace isocity {

// Simple binary save/load for the current world state.
// File format is versioned so you can extend it later.

bool SaveWorldBinary(const World& world, const std::string& path, std::string& outError);
bool LoadWorldBinary(World& outWorld, const std::string& path, std::string& outError);

} // namespace isocity
