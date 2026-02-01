#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Deterministic "city newspaper" generated from per-day Stats snapshots.
// The Chronicle is intended for headless exports (dossiers, regression tests,
// CLI analysis) so it avoids any UI dependencies.

enum class ChronicleTone : std::uint8_t {
  Good = 0,
  Neutral = 1,
  Bad = 2,
  Alert = 3,
};

const char* ChronicleToneName(ChronicleTone t);

struct ChronicleEntry {
  int day = 0;
  ChronicleTone tone = ChronicleTone::Neutral;

  // 0..100 (EMA-smoothed).
  float mayorRating = 50.0f;

  // Story salience (for sorting/debugging).
  float score = 0.0f;

  std::vector<std::string> tags;
  std::string headline;
  std::string body;
  std::string tip;
};

struct Chronicle {
  int version = 1;
  std::uint64_t seed = 0;
  std::string cityName;

  // size == kDistrictCount
  std::vector<std::string> districtNames;

  std::vector<ChronicleEntry> entries;
};

// Generate a deterministic chronicle from Stats snapshots.
//
// ticks: Stats snapshots after each simulated day.
// If ticks is empty, a single entry is generated from world.stats().
Chronicle GenerateCityChronicle(const World& world, const std::vector<Stats>& ticks);

bool WriteCityChronicleJson(const std::string& path, const Chronicle& chronicle, std::string& outError);
bool WriteCityChronicleMarkdown(const std::string& path, const Chronicle& chronicle, std::string& outError);

} // namespace isocity
