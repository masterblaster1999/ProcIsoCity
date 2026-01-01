#pragma once

#include "isocity/PolicyOptimizer.hpp"

#include <iosfwd>
#include <string>

namespace isocity {

// Serialize a PolicyOptimizationResult to JSON (single document).
//
// baseline (optional):
//   If non-null, writes a "baseline" field for comparison.
//   Typically this is the evaluation of the save's current policy.
//
// includeTop:
//   If true, includes the "top" array (can be large if topK is large).
bool WritePolicyOptimizationJson(std::ostream& os, const PolicyOptimizationResult& r, const PolicyOptimizerConfig& cfg,
                                const PolicySearchSpace& space, const PolicyEvalResult* baseline = nullptr, bool includeTop = true,
                                std::string* outError = nullptr);

bool ExportPolicyOptimizationJson(const std::string& path, const PolicyOptimizationResult& r, const PolicyOptimizerConfig& cfg,
                                 const PolicySearchSpace& space, const PolicyEvalResult* baseline = nullptr, bool includeTop = true,
                                 std::string* outError = nullptr);

// Export the top candidates table to CSV.
bool ExportPolicyOptimizationTopCsv(const std::string& path, const PolicyOptimizationResult& r, std::string* outError = nullptr);

// Export the CEM iteration trace to CSV.
// If r.bestByIteration is empty, writes only the header.
bool ExportPolicyOptimizationTraceCsv(const std::string& path, const PolicyOptimizationResult& r, std::string* outError = nullptr);

} // namespace isocity
