#pragma once

#include "isocity/Sim.hpp"
#include "isocity/World.hpp"

#include <cstdint>
#include <atomic>
#include <limits>
#include <string>
#include <vector>

namespace isocity {

// Lightweight "policy search" utilities.
//
// The simulation already exposes a handful of global levers (taxes + maintenance).
// This module provides a deterministic way to explore that policy space headlessly
// by repeatedly simulating N days from the same baseline world.
//
// Important notes:
//  - This is NOT a perfect optimizer. It is intended for tooling / analysis.
//  - Results are deterministic for a given baseline world + search config.
//  - The simulator contains a small amount of pseudo-randomness (e.g. upgrades),
//    but it is seeded deterministically from the world seed + day.

// Search method.
//  - Exhaustive: enumerate every candidate in the given ranges (guarded by maxExhaustiveCandidates).
//  - CEM: cross-entropy method style iterative sampling (fast for large spaces).
enum class PolicyOptMethod : std::uint8_t {
  Exhaustive = 0,
  CEM = 1,
};

// Subset of SimConfig that we allow the optimizer to modify.
struct PolicyCandidate {
  int taxResidential = 1;
  int taxCommercial = 2;
  int taxIndustrial = 2;

  int maintenanceRoad = 1;
  int maintenancePark = 1;
};

struct PolicySearchSpace {
  int taxResMin = 0;
  int taxResMax = 6;

  int taxComMin = 0;
  int taxComMax = 8;

  int taxIndMin = 0;
  int taxIndMax = 8;

  int maintRoadMin = 0;
  int maintRoadMax = 4;

  int maintParkMin = 0;
  int maintParkMax = 4;
};

struct PolicyObjective {
  // Score is a linear combination of simple end-of-horizon metrics:
  //
  //  moneyDelta: moneyEnd - moneyStart
  //  populationEnd
  //  happyPop: avgHappiness * populationEnd
  //  unemployed: max(0, populationEnd - employedEnd)
  //  congestionPop: trafficCongestionEnd * populationEnd
  //
  // score =
  //   wMoneyDelta * moneyDelta +
  //   wPopulation * populationEnd +
  //   wHappyPop * happyPop -
  //   wUnemployed * unemployed -
  //   wCongestionPop * congestionPop
  //
  // Units:
  //  - moneyDelta is in dollars
  //  - populationEnd, happyPop, unemployed are in "people"
  //  - congestionPop is in "people-equivalent" (0..pop)
  double wMoneyDelta = 1.0;
  double wPopulation = 0.0;
  double wHappyPop = 0.0;
  double wUnemployed = 0.0;
  double wCongestionPop = 0.0;

  // Hard constraints (violations yield -inf score).
  double minHappiness = 0.0; // final happiness (0..1)
  int minMoneyEnd = std::numeric_limits<int>::min();
};

// Extra metrics recorded during evaluation.
// These are meant for reporting / debugging, not for perfect modeling.
struct PolicyEvalMetrics {
  int daysSimulated = 0;

  int moneyStart = 0;
  int moneyEnd = 0;
  int moneyDelta = 0;

  int populationEnd = 0;
  int employedEnd = 0;
  int jobsCapacityAccessibleEnd = 0;

  float happinessEnd = 0.0f;     // final
  float avgHappiness = 0.0f;     // averaged over eval horizon (or equals happinessEnd if daysSimulated==0)
  float demandResidentialEnd = 0.0f;
  float avgLandValueEnd = 0.0f;

  float avgCommuteTimeEnd = 0.0f;
  float trafficCongestionEnd = 0.0f;

  double avgNetPerDay = 0.0; // mean(income-expenses) over eval horizon (0 when daysSimulated==0)
};

struct PolicyEvalResult {
  PolicyCandidate policy{};
  PolicyEvalMetrics metrics{};
  double score = -std::numeric_limits<double>::infinity();
};

struct PolicyDistribution {
  double meanTaxResidential = 0.0;
  double stdTaxResidential = 0.0;

  double meanTaxCommercial = 0.0;
  double stdTaxCommercial = 0.0;

  double meanTaxIndustrial = 0.0;
  double stdTaxIndustrial = 0.0;

  double meanMaintRoad = 0.0;
  double stdMaintRoad = 0.0;

  double meanMaintPark = 0.0;
  double stdMaintPark = 0.0;
};

struct PolicyOptimizerConfig {
  PolicyOptMethod method = PolicyOptMethod::CEM;

  // Evaluation horizon for each candidate (simulation steps).
  int evalDays = 60;

  // CEM settings (ignored for Exhaustive).
  int iterations = 25;
  int population = 64;
  int elites = 8;
  float exploreProb = 0.10f; // per-sample uniform exploration probability

  std::uint64_t rngSeed = 1;

  // Threading: 0 means "auto".
  int threads = 0;

  // Exhaustive guard.
  std::uint64_t maxExhaustiveCandidates = 500000;

  // How many top candidates to retain (sorted descending score) in the final result.
  int topK = 32;

  PolicyObjective objective{};
};

struct PolicyOptimizationResult {
  PolicyEvalResult best{};
  PolicyOptMethod methodUsed = PolicyOptMethod::CEM;
  std::vector<PolicyEvalResult> top; // best candidates (descending score)

  int candidatesEvaluated = 0;
  int iterationsCompleted = 0;

  // For CEM runs, a compact trace for plotting/debug.
  std::vector<PolicyEvalResult> bestByIteration;
  std::vector<PolicyDistribution> distByIteration;
};


// Optional progress reporting for long-running optimization calls.
//
// When non-null, OptimizePolicies() will update these atomics as it evaluates candidates.
// The intent is for interactive UIs to display approximate progress without needing
// a heavy callback or logging system.
struct PolicyOptProgress {
  std::atomic<int> iterationsTotal{0};      // total planned iterations (1 for exhaustive)
  std::atomic<int> iterationsCompleted{0};  // completed iterations
  std::atomic<int> candidatesEvaluated{0};  // total candidates evaluated so far
  std::atomic<bool> exhaustive{false};      // true when methodUsed==Exhaustive
  std::atomic<bool> done{false};            // set to true when the call returns
};


// Extract the editable policy subset from a SimConfig.
PolicyCandidate ExtractPolicyFromSimConfig(const SimConfig& cfg);

// Apply the editable policy subset onto an existing SimConfig.
void ApplyPolicyToSimConfig(const PolicyCandidate& p, SimConfig& cfg);

// Evaluate a single candidate by simulating cfg.evalDays days from baseWorld.
PolicyEvalResult EvaluatePolicyCandidate(const World& baseWorld, const SimConfig& baseSimCfg, const PolicyCandidate& cand,
                                        const PolicyOptimizerConfig& cfg);

// Optimize policy parameters over the given search space.
PolicyOptimizationResult OptimizePolicies(const World& baseWorld, const SimConfig& baseSimCfg, const PolicySearchSpace& space,
                                         const PolicyOptimizerConfig& cfg, PolicyOptProgress* progress = nullptr);

} // namespace isocity
