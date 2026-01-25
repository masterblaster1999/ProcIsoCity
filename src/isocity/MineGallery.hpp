#pragma once

#include "isocity/Export.hpp"
#include "isocity/MineClustering.hpp"
#include "isocity/MineEmbedding.hpp"
#include "isocity/MineNeighbors.hpp"
#include "isocity/MineTraces.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SeedMiner.hpp"
#include "isocity/Sim.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace isocity {

// -----------------------------------------------------------------------------
// Mine gallery exporter
//
// The seed miner (`proc_isocity_mine`) is great for discovering interesting
// procedural worlds, but a CSV/JSON list can be hard to triage quickly.
//
// A "mine gallery" is a lightweight, *multi-seed* export bundle that renders
// one or more map layers for a selected set of MineRecords and writes a simple
// offline-friendly HTML index page.
//
// This intentionally lives in isocity_core (no raylib dependency) so both
// headless tools and the interactive game can reuse it.
// -----------------------------------------------------------------------------

struct MineGalleryProgress {
  int index = 0; // 0-based entry index within the selected list
  int total = 0;
  std::uint64_t seed = 0;
  std::string stage;
};

using MineGalleryProgressFn = std::function<void(const MineGalleryProgress&)>;

struct MineGalleryConfig {
  // Output directory (required).
  std::filesystem::path outDir;

  // Image format extension used for thumbnails ("png" or "ppm").
  // Recommended: "png".
  std::string format = "png";

  // Nearest-neighbor upscale for one-pixel-per-tile map renders.
  int exportScale = 3;

  // Layers rendered for each seed.
  // If empty, Overlay is used.
  std::vector<ExportLayer> layers = {ExportLayer::Overlay};

  // Write a single contact sheet image (grid of the primary layer).
  bool writeContactSheet = true;
  int contactSheetCols = 6;
  int contactSheetPaddingPx = 2;

  // Write a small JSON manifest of entries.
  bool writeJson = true;

  // Write an offline HTML index.
  bool writeHtml = true;

  // Compute a k-medoids clustering over the selected seeds and annotate the
  // gallery (HTML + JSON + embedding) with cluster ids.
  bool writeClusters = false;

  // Configuration for clustering (only used when writeClusters==true).
  MineClusteringConfig clusteringCfg{};

  // Write a 2D embedding plot (interactive canvas) into the HTML index.
  // This helps visually cluster the selected seeds by KPI/layout similarity.
  bool writeEmbeddingPlot = false;

  // Configuration for the embedding distance space.
  MineEmbeddingConfig embeddingCfg{};

  // Compute a k-nearest-neighbors graph over the selected seeds and embed it
  // into the gallery outputs. This is useful for "similar city" navigation.
  bool writeNeighbors = false;

  // Configuration for the neighbor distance space.
  MineNeighborsConfig neighborsCfg{};

  // Write per-day KPI traces for the selected seeds.
  //
  // When enabled, the gallery exporter writes a compact traces.json and
  // embeds interactive sparkline controls into the HTML index.
  bool writeTraces = false;

  // Metrics recorded for traces.
  // If empty, DefaultMineTraceMetrics() is used.
  std::vector<MineTraceMetric> traceMetrics;
};

struct MineGalleryResult {
  std::filesystem::path outDir;
  std::filesystem::path indexHtml;
  std::filesystem::path jsonManifest;
  std::filesystem::path contactSheet;

  // Optional: written when writeEmbeddingPlot==true.
  std::filesystem::path embeddingJson;

  // Optional: written when writeNeighbors==true.
  std::filesystem::path neighborsJson;

  // Optional: written when writeTraces==true.
  std::filesystem::path tracesJson;
};

// Generate a mine gallery for the given records.
//
// - `recs` holds the mined summary records.
// - `selectedIndices` are indices into `recs` describing which seeds to export
//   (typically the output of SelectTopIndices/SelectTopParetoIndices/...)
// - `days` is the number of simulation days to run to reproduce the final world
//   state for renders.
//
// Returns true on success; on failure, outErr contains a human-readable error.
bool WriteMineGallery(const MineGalleryConfig& cfg,
                      const std::vector<MineRecord>& recs,
                      const std::vector<int>& selectedIndices,
                      const ProcGenConfig& procCfg,
                      const SimConfig& simCfg,
                      int days,
                      MineGalleryResult* outRes,
                      std::string& outErr,
                      const MineGalleryProgressFn& progress = MineGalleryProgressFn());

} // namespace isocity
