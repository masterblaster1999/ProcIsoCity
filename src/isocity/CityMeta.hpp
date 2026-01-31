#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace isocity {

// Version for the CityMeta JSON schema.
//
// CityMeta stores *game-layer* state that is not part of the World binary save:
//  - City Report time-series samples
//  - City News feed
//  - City Challenges progress/log
//
// Kept as a separate JSON sidecar so the core World format remains compact and
// forward-compatible.
constexpr int kCityMetaVersion = 1;

// Small time-series sample used by the in-game City Report panel.
// Stored in the game layer, derived from World::Stats after each sim tick.
struct CityHistorySample {
  int day = 0;

  int population = 0;
  int money = 0;

  float happiness = 0.0f;
  float demandResidential = 0.0f;
  float demandCommercial = 0.0f;
  float demandIndustrial = 0.0f;

  float avgLandValue = 0.0f;
  float avgTaxPerCapita = 0.0f;

  int income = 0;
  int expenses = 0;
  int taxRevenue = 0;
  int maintenanceCost = 0;

  int commuters = 0;
  float avgCommute = 0.0f;         // road steps
  float avgCommuteTime = 0.0f;     // street-step equivalent travel time
  float trafficCongestion = 0.0f;  // 0..1

  float goodsSatisfaction = 1.0f;  // 0..1
};

// SimCity-style city news / advisor feed entry.
enum class CityNewsTone : std::uint8_t { Good = 0, Neutral = 1, Bad = 2, Alert = 3 };

struct CityNewsEntry {
  int day = 0;
  CityNewsTone tone = CityNewsTone::Neutral;

  // Mayor rating (0..100), exponentially smoothed so it doesn't jitter day-to-day.
  float mayorRating = 50.0f;

  std::string headline;
  std::string body;
};

// Optional goal/challenge that nudges sandbox play toward short-term objectives.
enum class CityChallengeKind : std::uint8_t {
  GrowPopulation = 0,
  BuildParks,
  ReduceCongestion,
  ImproveGoods,
  ImproveServices,
  BalanceBudget,
  RestoreOutsideConnection,
};

enum class CityChallengeStatus : std::uint8_t { Active = 0, Completed = 1, Failed = 2, Canceled = 3 };

struct CityChallenge {
  std::uint32_t id = 0;
  CityChallengeKind kind = CityChallengeKind::GrowPopulation;
  CityChallengeStatus status = CityChallengeStatus::Active;

  int dayIssued = 0;
  int dayDeadline = 0; // inclusive

  int rewardMoney = 0;

  // Generic parameters used by each kind.
  int startInt = 0;
  int targetInt = 0;
  int stateInt = 0;
  float startF = 0.0f;
  float targetF = 0.0f;

  std::string title;
  std::string description;
};

struct CityChallengeLogEntry {
  int day = 0;
  CityChallengeStatus status = CityChallengeStatus::Completed;
  int rewardMoney = 0;
  std::string title;
};

// Sidecar JSON persisted next to a save file.
struct CityMeta {
  int version = kCityMetaVersion;
  std::uint64_t seed = 0;
  int width = 0;
  int height = 0;
  int day = 0;

  // City Report (graphs).
  int historyMax = 240;
  std::vector<CityHistorySample> history;

  // City News.
  int newsMax = 120;
  int newsSelection = 0;
  int newsFirst = 0;
  float mayorRatingEma = 50.0f;
  float mayorRatingPrev = 50.0f;
  std::deque<CityNewsEntry> news;

  // City Challenges.
  int challengeTargetActive = 3;
  int challengeRerolls = 0;
  std::uint32_t challengeNextId = 1;
  int challengeLastProcessedDay = -1;
  int challengeSelection = 0;
  int challengeFirst = 0;
  std::vector<CityChallenge> challenges;
  std::deque<CityChallengeLogEntry> challengeLog;

  // Daily addendum injected into City News (challenge completions/failures).
  std::map<int, std::string> newsAddendum;
};

// Serialize/deserialize CityMeta as a JSON string.
bool SerializeCityMetaJson(const CityMeta& meta, std::string& outJson, std::string& outError);
bool DeserializeCityMetaJson(CityMeta& outMeta, const std::string& json, std::string& outError);

// Resolve the JSON sidecar path for a given save file path.
// Example: "isocity_save_slot2.bin" -> "isocity_save_slot2.meta.json".
std::string CityMetaPathForSavePath(const std::string& savePath);

// Save/load helpers used by the interactive app (best-effort, does not affect the main save).
bool SaveCityMetaForSavePath(const std::string& savePath, const CityMeta& meta, std::string& outError);
bool LoadCityMetaJsonFile(const std::string& path, CityMeta& outMeta, std::string& outError);

} // namespace isocity
