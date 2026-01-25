#pragma once

#include "isocity/World.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Mine traces: per-day time-series metrics derived from the simulator Stats.
//
// This is primarily used by the mine gallery exporter to add sparkline
// visualizations and to export a compact traces.json artifact for offline
// analysis.
// -----------------------------------------------------------------------------

enum class MineTraceMetric : std::uint8_t {
  Population = 0,
  Happiness,
  Money,
  AvgLandValue,
  TrafficCongestion,
  GoodsSatisfaction,
  ServicesOverallSatisfaction,
  TransitModeShare,
  AvgCommuteTime,
  EconomyIndex,
  TradeMarketIndex,
};

const char* MineTraceMetricName(MineTraceMetric m);

// Parse a metric name (case-insensitive).
// Accepts common aliases (e.g. "pop" => population).
bool ParseMineTraceMetric(const std::string& s, MineTraceMetric& out);

// Convenience: parse comma-separated list.
//
// - Empty input clears outMetrics and returns true.
// - On error, returns false and writes a human-readable message to outError.
bool ParseMineTraceMetricList(const std::string& csv,
                             std::vector<MineTraceMetric>& outMetrics,
                             std::string* outError = nullptr);

// Default metric set for trace exports when the user doesn't specify.
std::vector<MineTraceMetric> DefaultMineTraceMetrics();

// Extract a scalar value from a Stats snapshot.
//
// Returned value is always finite for built-in metrics (uses 0 on non-finite).
double MineTraceMetricValue(const Stats& s, MineTraceMetric m);

} // namespace isocity
