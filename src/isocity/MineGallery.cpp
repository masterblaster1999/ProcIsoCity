#include "isocity/MineGallery.hpp"

#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Json.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

static std::string Hex16NoPrefix(std::uint64_t v)
{
  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static bool WriteTextFile(const std::filesystem::path& path, const std::string& text, std::string& outErr)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outErr = "Failed to write: " + path.string();
    return false;
  }
  f.write(text.data(), static_cast<std::streamsize>(text.size()));
  if (!f) {
    outErr = "Failed while writing: " + path.string();
    return false;
  }
  return true;
}

static bool EnsureDir(const std::filesystem::path& p, std::string& outErr)
{
  std::error_code ec;
  if (p.empty()) {
    outErr = "Empty output directory";
    return false;
  }
  std::filesystem::create_directories(p, ec);
  if (ec) {
    outErr = "Failed to create directory: " + p.string();
    return false;
  }
  return true;
}

static bool WriteImageWithExt(const std::filesystem::path& path, const PpmImage& img, std::string& outErr)
{
  const std::string ext = path.extension().string();
  if (ext == ".png") {
    return WritePng(path.string(), img, outErr);
  }
  if (ext == ".ppm" || ext == ".pnm") {
    return WritePpm(path.string(), img, outErr);
  }
  // Default: try PNG.
  return WritePng(path.string(), img, outErr);
}

static bool LayerNeedsTraffic(const ExportLayer l)
{
  switch (l) {
  case ExportLayer::Traffic:
  case ExportLayer::GoodsTraffic:
  case ExportLayer::Noise:
  case ExportLayer::HeatIsland:
  case ExportLayer::LandValue:
    return true;
  default: break;
  }
  return false;
}

static bool LayerNeedsGoods(const ExportLayer l)
{
  switch (l) {
  case ExportLayer::GoodsTraffic:
  case ExportLayer::GoodsFill:
  case ExportLayer::Noise:
  case ExportLayer::HeatIsland:
    return true;
  default: break;
  }
  return false;
}

static bool LayerNeedsLandValue(const ExportLayer l)
{
  return l == ExportLayer::LandValue;
}

static PpmImage MakeBlankPpm(int w, int h, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  PpmImage out;
  out.width = std::max(0, w);
  out.height = std::max(0, h);
  out.rgb.resize(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 3u, 0);
  for (int y = 0; y < out.height; ++y) {
    for (int x = 0; x < out.width; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x)) * 3u;
      out.rgb[idx + 0] = r;
      out.rgb[idx + 1] = g;
      out.rgb[idx + 2] = b;
    }
  }
  return out;
}

static void BlitPpm(const PpmImage& src, PpmImage& dst, int dstX, int dstY)
{
  if (src.width <= 0 || src.height <= 0) return;
  if (dst.width <= 0 || dst.height <= 0) return;

  for (int y = 0; y < src.height; ++y) {
    const int ty = dstY + y;
    if (ty < 0 || ty >= dst.height) continue;
    for (int x = 0; x < src.width; ++x) {
      const int tx = dstX + x;
      if (tx < 0 || tx >= dst.width) continue;

      const std::size_t sidx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 3u;
      const std::size_t didx = (static_cast<std::size_t>(ty) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(tx)) * 3u;
      dst.rgb[didx + 0] = src.rgb[sidx + 0];
      dst.rgb[didx + 1] = src.rgb[sidx + 1];
      dst.rgb[didx + 2] = src.rgb[sidx + 2];
    }
  }
}

static void AddJsonKV(JsonValue& o, const char* key, JsonValue v)
{
  o.objectValue.emplace_back(key, std::move(v));
}

static std::string EscapeHtml(const std::string& s)
{
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
    case '&': out += "&amp;"; break;
    case '<': out += "&lt;"; break;
    case '>': out += "&gt;"; break;
    case '"': out += "&quot;"; break;
    case '\'': out += "&#39;"; break;
    default: out.push_back(c); break;
    }
  }
  return out;
}

} // namespace


bool WriteMineGallery(const MineGalleryConfig& cfg,
                      const std::vector<MineRecord>& recs,
                      const std::vector<int>& selectedIndices,
                      const ProcGenConfig& procCfg,
                      const SimConfig& simCfg,
                      int days,
                      MineGalleryResult* outRes,
                      std::string& outErr,
                      const MineGalleryProgressFn& progress)
{
  outErr.clear();

  if (selectedIndices.empty()) {
    outErr = "No selected seeds to export";
    return false;
  }
  if (days < 0) {
    outErr = "Invalid days";
    return false;
  }
  if (cfg.exportScale <= 0) {
    outErr = "Invalid exportScale";
    return false;
  }

  MineGalleryConfig localCfg = cfg;
  if (localCfg.layers.empty()) {
    localCfg.layers.push_back(ExportLayer::Overlay);
  }
  if (localCfg.format.empty()) localCfg.format = "png";
  if (localCfg.contactSheetCols <= 0) localCfg.contactSheetCols = 6;
  if (localCfg.contactSheetPaddingPx < 0) localCfg.contactSheetPaddingPx = 0;

  // Output directory structure.
  if (!EnsureDir(localCfg.outDir, outErr)) return false;
  const std::filesystem::path thumbsDir = localCfg.outDir / "thumbs";
  if (!EnsureDir(thumbsDir, outErr)) return false;

  // Precompute whether we need derived overlays for requested layers.
  bool needTraffic = false;
  bool needGoods = false;
  bool needLandValue = false;
  for (ExportLayer l : localCfg.layers) {
    needTraffic = needTraffic || LayerNeedsTraffic(l);
    needGoods = needGoods || LayerNeedsGoods(l);
    needLandValue = needLandValue || LayerNeedsLandValue(l);
  }

  // Prepare contact sheet (primary layer only).
  const ExportLayer primaryLayer = localCfg.layers.front();
  PpmImage contactSheet;
  int thumbW = 0;
  int thumbH = 0;
  int sheetRows = 0;

  // JSON manifest root.
  JsonValue manifest = JsonValue::MakeObject();
  AddJsonKV(manifest, "type", JsonValue::MakeString("procisocity_mine_gallery"));
  AddJsonKV(manifest, "version", JsonValue::MakeNumber(1));
  {
    JsonValue cfgObj = JsonValue::MakeObject();
    AddJsonKV(cfgObj, "format", JsonValue::MakeString(localCfg.format));
    AddJsonKV(cfgObj, "exportScale", JsonValue::MakeNumber(static_cast<double>(localCfg.exportScale)));
    JsonValue layersArr = JsonValue::MakeArray();
    for (ExportLayer l : localCfg.layers) {
      layersArr.arrayValue.push_back(JsonValue::MakeString(ExportLayerName(l)));
    }
    AddJsonKV(cfgObj, "layers", std::move(layersArr));
    AddJsonKV(cfgObj, "contactSheet", JsonValue::MakeBool(localCfg.writeContactSheet));
    AddJsonKV(cfgObj, "contactSheetCols", JsonValue::MakeNumber(static_cast<double>(localCfg.contactSheetCols)));
    AddJsonKV(cfgObj, "contactSheetPaddingPx", JsonValue::MakeNumber(static_cast<double>(localCfg.contactSheetPaddingPx)));

    // Optional clustering.
    AddJsonKV(cfgObj, "clusters", JsonValue::MakeBool(localCfg.writeClusters));
    if (localCfg.writeClusters) {
      JsonValue c = JsonValue::MakeObject();
      AddJsonKV(c, "k", JsonValue::MakeNumber(static_cast<double>(localCfg.clusteringCfg.k)));
      AddJsonKV(c, "space", JsonValue::MakeString(MineDiversityModeName(localCfg.clusteringCfg.space)));
      AddJsonKV(c, "layoutWeight", JsonValue::MakeNumber(localCfg.clusteringCfg.layoutWeight));
      AddJsonKV(c, "robustScaling", JsonValue::MakeBool(localCfg.clusteringCfg.robustScaling));
      AddJsonKV(c, "maxIters", JsonValue::MakeNumber(static_cast<double>(localCfg.clusteringCfg.maxIters)));
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : localCfg.clusteringCfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      AddJsonKV(c, "metrics", std::move(ms));
      AddJsonKV(cfgObj, "clustering", std::move(c));
    }

    // Optional embedding plot.
    AddJsonKV(cfgObj, "embeddingPlot", JsonValue::MakeBool(localCfg.writeEmbeddingPlot));
    if (localCfg.writeEmbeddingPlot) {
      JsonValue e = JsonValue::MakeObject();
      AddJsonKV(e, "space", JsonValue::MakeString(MineDiversityModeName(localCfg.embeddingCfg.space)));
      AddJsonKV(e, "layoutWeight", JsonValue::MakeNumber(localCfg.embeddingCfg.layoutWeight));
      AddJsonKV(e, "robustScaling", JsonValue::MakeBool(localCfg.embeddingCfg.robustScaling));
      AddJsonKV(e, "powerIters", JsonValue::MakeNumber(static_cast<double>(localCfg.embeddingCfg.powerIters)));
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : localCfg.embeddingCfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      AddJsonKV(e, "metrics", std::move(ms));
      AddJsonKV(cfgObj, "embedding", std::move(e));
    }

    // Optional neighbor graph.
    AddJsonKV(cfgObj, "neighbors", JsonValue::MakeBool(localCfg.writeNeighbors));
    if (localCfg.writeNeighbors) {
      JsonValue n = JsonValue::MakeObject();
      AddJsonKV(n, "k", JsonValue::MakeNumber(static_cast<double>(localCfg.neighborsCfg.k)));
      AddJsonKV(n, "space", JsonValue::MakeString(MineDiversityModeName(localCfg.neighborsCfg.space)));
      AddJsonKV(n, "layoutWeight", JsonValue::MakeNumber(localCfg.neighborsCfg.layoutWeight));
      AddJsonKV(n, "robustScaling", JsonValue::MakeBool(localCfg.neighborsCfg.robustScaling));
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : localCfg.neighborsCfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      AddJsonKV(n, "metrics", std::move(ms));
      AddJsonKV(cfgObj, "neighborGraph", std::move(n));
    }

    // Optional per-day KPI traces.
    AddJsonKV(cfgObj, "traces", JsonValue::MakeBool(localCfg.writeTraces));
    if (localCfg.writeTraces) {
      const std::vector<MineTraceMetric> tms =
          localCfg.traceMetrics.empty() ? DefaultMineTraceMetrics() : localCfg.traceMetrics;
      JsonValue ms = JsonValue::MakeArray();
      for (MineTraceMetric m : tms) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineTraceMetricName(m)));
      }
      AddJsonKV(cfgObj, "traceMetrics", std::move(ms));
      // Traces are recorded for the same `days` used to reproduce the renders.
      AddJsonKV(cfgObj, "traceDays", JsonValue::MakeNumber(static_cast<double>(days)));
      AddJsonKV(cfgObj, "traceIncludeDay0", JsonValue::MakeBool(true));
    }
    AddJsonKV(manifest, "config", std::move(cfgObj));
  }

  JsonValue entries = JsonValue::MakeArray();

  // Optional embedding artifact.
  std::filesystem::path embeddingPath;
  std::string embeddingJsonInline;
  MineEmbeddingResult embeddingRes;

  // Optional neighbor graph artifact.
  std::filesystem::path neighborsPath;
  MineNeighborsResult neighborsRes;

  // Optional per-day trace artifact.
  std::filesystem::path tracesPath;
  std::string tracesJsonInline;
  JsonValue tracesObj;
  JsonValue tracesSeries;
  std::vector<MineTraceMetric> traceMetrics;
  std::vector<double> traceMin;
  std::vector<double> traceMax;

  // Optional clustering artifact.
  MineClusteringResult clusterRes;
  std::vector<int> entryClusters;
  std::string clusterJsonInline;

  // Build thumbnails.
  const int total = static_cast<int>(selectedIndices.size());

  // Optional clustering over the selected set.
  JsonValue clustersObj;
  if (localCfg.writeClusters && total > 0) {
    clusterRes = ComputeMineClusteringKMedoids(recs, selectedIndices, localCfg.clusteringCfg);

    entryClusters.assign(static_cast<std::size_t>(total), -1);
    if (clusterRes.ok && clusterRes.assignment.size() == selectedIndices.size()) {
      entryClusters = clusterRes.assignment;
    }

    clustersObj = JsonValue::MakeObject();
    AddJsonKV(clustersObj, "type", JsonValue::MakeString("procisocity_mine_clusters"));
    AddJsonKV(clustersObj, "version", JsonValue::MakeNumber(1));
    AddJsonKV(clustersObj, "ok", JsonValue::MakeBool(clusterRes.ok));
    if (!clusterRes.warning.empty()) {
      AddJsonKV(clustersObj, "warning", JsonValue::MakeString(clusterRes.warning));
    }
    AddJsonKV(clustersObj, "k", JsonValue::MakeNumber(static_cast<double>(clusterRes.cfg.k)));
    AddJsonKV(clustersObj, "space", JsonValue::MakeString(MineDiversityModeName(clusterRes.cfg.space)));
    AddJsonKV(clustersObj, "layoutWeight", JsonValue::MakeNumber(clusterRes.cfg.layoutWeight));
    AddJsonKV(clustersObj, "robustScaling", JsonValue::MakeBool(clusterRes.cfg.robustScaling));
    AddJsonKV(clustersObj, "maxIters", JsonValue::MakeNumber(static_cast<double>(clusterRes.cfg.maxIters)));
    {
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : clusterRes.cfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      AddJsonKV(clustersObj, "metrics", std::move(ms));
    }
    AddJsonKV(clustersObj, "totalCost", JsonValue::MakeNumber(clusterRes.totalCost));
    AddJsonKV(clustersObj, "avgSilhouette", JsonValue::MakeNumber(clusterRes.avgSilhouette));

    {
      JsonValue sz = JsonValue::MakeArray();
      for (int s : clusterRes.clusterSizes) {
        sz.arrayValue.push_back(JsonValue::MakeNumber(static_cast<double>(s)));
      }
      AddJsonKV(clustersObj, "clusterSizes", std::move(sz));
    }

    {
      JsonValue meds = JsonValue::MakeArray();
      for (int c = 0; c < clusterRes.cfg.k; ++c) {
        const int e = (c >= 0 && static_cast<std::size_t>(c) < clusterRes.medoidEntry.size()) ? clusterRes.medoidEntry[static_cast<std::size_t>(c)] : -1;
        const int ridx = (c >= 0 && static_cast<std::size_t>(c) < clusterRes.medoidRecIndex.size()) ? clusterRes.medoidRecIndex[static_cast<std::size_t>(c)] : -1;
        if (e < 0 || ridx < 0 || static_cast<std::size_t>(ridx) >= recs.size()) continue;
        const MineRecord& r = recs[static_cast<std::size_t>(ridx)];

        JsonValue m = JsonValue::MakeObject();
        AddJsonKV(m, "cluster", JsonValue::MakeNumber(static_cast<double>(c)));
        AddJsonKV(m, "entry", JsonValue::MakeNumber(static_cast<double>(e)));
        AddJsonKV(m, "rank", JsonValue::MakeNumber(static_cast<double>(e + 1)));
        AddJsonKV(m, "recIndex", JsonValue::MakeNumber(static_cast<double>(ridx)));
        AddJsonKV(m, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
        AddJsonKV(m, "seedHex", JsonValue::MakeString(HexU64(r.seed)));
        AddJsonKV(m, "score", JsonValue::MakeNumber(r.score));
        meds.arrayValue.push_back(std::move(m));
      }
      AddJsonKV(clustersObj, "medoids", std::move(meds));
    }

    // Inline (minified) JSON for the HTML controls.
    {
      JsonWriteOptions opt;
      opt.pretty = false;
      opt.sortKeys = false;
      clusterJsonInline = JsonStringify(clustersObj, opt);
    }

    // Also include in the gallery manifest.
    AddJsonKV(manifest, "clusters", clustersObj);
  }

  // Optional kNN neighbor graph over the selected set.
  // This is useful for exploration: jump to similar cities from a given seed.
  JsonValue neighborsObj;
  if (localCfg.writeNeighbors && total > 0) {
    neighborsRes = ComputeMineNeighborsKNN(recs, selectedIndices, localCfg.neighborsCfg);

    neighborsObj = JsonValue::MakeObject();
    AddJsonKV(neighborsObj, "type", JsonValue::MakeString("procisocity_mine_neighbors"));
    AddJsonKV(neighborsObj, "version", JsonValue::MakeNumber(1));
    AddJsonKV(neighborsObj, "ok", JsonValue::MakeBool(neighborsRes.ok));
    if (!neighborsRes.warning.empty()) {
      AddJsonKV(neighborsObj, "warning", JsonValue::MakeString(neighborsRes.warning));
    }
    AddJsonKV(neighborsObj, "k", JsonValue::MakeNumber(static_cast<double>(neighborsRes.cfg.k)));
    AddJsonKV(neighborsObj, "space", JsonValue::MakeString(MineDiversityModeName(neighborsRes.cfg.space)));
    AddJsonKV(neighborsObj, "layoutWeight", JsonValue::MakeNumber(neighborsRes.cfg.layoutWeight));
    AddJsonKV(neighborsObj, "robustScaling", JsonValue::MakeBool(neighborsRes.cfg.robustScaling));
    {
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : neighborsRes.cfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      AddJsonKV(neighborsObj, "metrics", std::move(ms));
    }

    // Store a compact neighbor graph as entry-index neighbors.
    JsonValue graph = JsonValue::MakeArray();
    graph.arrayValue.reserve(static_cast<std::size_t>(total));
    for (int i = 0; i < total; ++i) {
      JsonValue row = JsonValue::MakeArray();
      if (neighborsRes.ok && static_cast<std::size_t>(i) < neighborsRes.neighbors.size() &&
          static_cast<std::size_t>(i) < neighborsRes.distances.size()) {
        const auto& ns = neighborsRes.neighbors[static_cast<std::size_t>(i)];
        const auto& ds = neighborsRes.distances[static_cast<std::size_t>(i)];
        const std::size_t m = std::min(ns.size(), ds.size());
        row.arrayValue.reserve(m);
        for (std::size_t j = 0; j < m; ++j) {
          JsonValue e = JsonValue::MakeObject();
          AddJsonKV(e, "entry", JsonValue::MakeNumber(static_cast<double>(ns[j])));
          AddJsonKV(e, "rank", JsonValue::MakeNumber(static_cast<double>(ns[j] + 1)));
          AddJsonKV(e, "dist", JsonValue::MakeNumber(ds[j]));
          row.arrayValue.push_back(std::move(e));
        }
      }
      graph.arrayValue.push_back(std::move(row));
    }
    AddJsonKV(neighborsObj, "graph", std::move(graph));

    // Write neighbors.json for tooling / postprocessing.
    neighborsPath = localCfg.outDir / "neighbors.json";
    {
      std::string jerr;
      JsonWriteOptions opt;
      opt.pretty = true;
      opt.indent = 2;
      opt.sortKeys = false;
      if (!WriteJsonFile(neighborsPath.string(), neighborsObj, jerr, opt)) {
        outErr = "Failed to write neighbors.json: " + jerr;
        return false;
      }
      AddJsonKV(manifest, "neighbors", JsonValue::MakeString(neighborsPath.filename().generic_string()));
    }
  }

  // Optional per-day KPI traces (computed while simulating each selected seed).
  // We write traces.json for tooling, and also embed a minified copy into the
  // HTML index so it works offline via file:// without CORS fetches.
  if (localCfg.writeTraces && total > 0) {
    traceMetrics = localCfg.traceMetrics.empty() ? DefaultMineTraceMetrics() : localCfg.traceMetrics;
    if (traceMetrics.empty()) {
      // Nothing to record.
      localCfg.writeTraces = false;
    } else {
      traceMin.assign(traceMetrics.size(), std::numeric_limits<double>::infinity());
      traceMax.assign(traceMetrics.size(), -std::numeric_limits<double>::infinity());
      tracesObj = JsonValue::MakeObject();
      tracesSeries = JsonValue::MakeArray();
      tracesSeries.arrayValue.reserve(static_cast<std::size_t>(total));
      AddJsonKV(tracesObj, "type", JsonValue::MakeString("procisocity_mine_traces"));
      AddJsonKV(tracesObj, "version", JsonValue::MakeNumber(1));
      AddJsonKV(tracesObj, "days", JsonValue::MakeNumber(static_cast<double>(days)));
      AddJsonKV(tracesObj, "includeDay0", JsonValue::MakeBool(true));
      {
        JsonValue ms = JsonValue::MakeArray();
        for (MineTraceMetric m : traceMetrics) {
          ms.arrayValue.push_back(JsonValue::MakeString(MineTraceMetricName(m)));
        }
        AddJsonKV(tracesObj, "metrics", std::move(ms));
      }
    }
  }

  for (int entryIndex = 0; entryIndex < total; ++entryIndex) {
    const int recIndex = selectedIndices[static_cast<std::size_t>(entryIndex)];
    if (recIndex < 0 || static_cast<std::size_t>(recIndex) >= recs.size()) {
      outErr = "Selected index out of range";
      return false;
    }

    const MineRecord& r = recs[static_cast<std::size_t>(recIndex)];

    if (progress) {
      MineGalleryProgress p;
      p.index = entryIndex;
      p.total = total;
      p.seed = r.seed;
      p.stage = "simulate";
      progress(p);
    }

    World world = GenerateWorld(r.w, r.h, r.seed, procCfg);
    Simulator sim(simCfg);
    sim.resetTimer();

    // Optional time-series traces (record a small set of scalar KPI metrics
    // across the simulation horizon).
    std::vector<std::vector<double>> traceValues;
    if (localCfg.writeTraces) {
      traceValues.resize(traceMetrics.size());
      for (auto& v : traceValues) {
        v.reserve(static_cast<std::size_t>(days) + 1u);
      }
    }

    auto recordTraceSample = [&]() {
      if (!localCfg.writeTraces) return;
      const Stats& s = world.stats();
      for (std::size_t mi = 0; mi < traceMetrics.size(); ++mi) {
        const double val = MineTraceMetricValue(s, traceMetrics[mi]);
        traceValues[mi].push_back(val);
        if (val < traceMin[mi]) traceMin[mi] = val;
        if (val > traceMax[mi]) traceMax[mi] = val;
      }
    };

    // Day 0 sample.
    sim.refreshDerivedStats(world);
    recordTraceSample();

    for (int d = 0; d < days; ++d) {
      sim.stepOnce(world);
      recordTraceSample();
    }

    // Ensure derived stats are fresh for overlay exports.
    sim.refreshDerivedStats(world);

    // Append this entry's trace series to the global traces object.
    if (localCfg.writeTraces) {
      JsonValue te = JsonValue::MakeObject();
      AddJsonKV(te, "rank", JsonValue::MakeNumber(static_cast<double>(entryIndex + 1)));
      AddJsonKV(te, "entry", JsonValue::MakeNumber(static_cast<double>(entryIndex)));
      AddJsonKV(te, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
      AddJsonKV(te, "seedHex", JsonValue::MakeString(HexU64(r.seed)));
      JsonValue vals = JsonValue::MakeArray();
      vals.arrayValue.reserve(traceValues.size());
      for (const auto& series : traceValues) {
        JsonValue arr = JsonValue::MakeArray();
        arr.arrayValue.reserve(series.size());
        for (double v : series) {
          arr.arrayValue.push_back(JsonValue::MakeNumber(v));
        }
        vals.arrayValue.push_back(std::move(arr));
      }
      AddJsonKV(te, "values", std::move(vals));
      tracesSeries.arrayValue.push_back(std::move(te));
    }

    std::vector<std::uint8_t> roadToEdgeMask;
    const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
    if (sim.config().requireOutsideConnection) {
      ComputeRoadsConnectedToEdge(world, roadToEdgeMask);
      roadToEdgePtr = &roadToEdgeMask;
    }

    TrafficResult trafficRes;
    GoodsResult goodsRes;
    LandValueResult landValueRes;
    bool haveTraffic = false;
    bool haveGoods = false;
    bool haveLandValue = false;

    if (needTraffic) {
      if (progress) {
        MineGalleryProgress p;
        p.index = entryIndex;
        p.total = total;
        p.seed = r.seed;
        p.stage = "compute_traffic";
        progress(p);
      }

      float employedShare = 0.0f;
      if (world.stats().population > 0) {
        employedShare = static_cast<float>(world.stats().employed) / static_cast<float>(world.stats().population);
      }
      const float carShare = employedShare * (1.0f - std::clamp(world.stats().transitModeShare, 0.0f, 1.0f));

      TrafficConfig tc{};
      tc.requireOutsideConnection = sim.config().requireOutsideConnection;
      tc.congestionAwareRouting = sim.trafficModel().congestionAwareRouting;
      tc.congestionIterations = sim.trafficModel().congestionIterations;
      tc.congestionAlpha = sim.trafficModel().congestionAlpha;
      tc.congestionBeta = sim.trafficModel().congestionBeta;
      tc.congestionCapacityScale = sim.trafficModel().congestionCapacityScale;
      tc.congestionRatioClamp = sim.trafficModel().congestionRatioClamp;
      tc.capacityAwareJobs = sim.trafficModel().capacityAwareJobs;
      tc.jobAssignmentIterations = sim.trafficModel().jobAssignmentIterations;
      tc.jobPenaltyBaseMilli = sim.trafficModel().jobPenaltyBaseMilli;
      trafficRes = ComputeCommuteTraffic(world, tc, carShare, roadToEdgePtr);
      haveTraffic = true;
    }

    if (needGoods) {
      if (progress) {
        MineGalleryProgress p;
        p.index = entryIndex;
        p.total = total;
        p.seed = r.seed;
        p.stage = "compute_goods";
        progress(p);
      }

      GoodsConfig gc{};
      gc.requireOutsideConnection = sim.config().requireOutsideConnection;
      gc.allowImports = true;
      gc.allowExports = true;
      gc.importCapacityPct = std::clamp(world.stats().tradeImportCapacityPct, 0, 100);
      gc.exportCapacityPct = std::clamp(world.stats().tradeExportCapacityPct, 0, 100);
      goodsRes = ComputeGoodsFlow(world, gc, roadToEdgePtr);
      haveGoods = true;
    }

    if (needLandValue) {
      if (progress) {
        MineGalleryProgress p;
        p.index = entryIndex;
        p.total = total;
        p.seed = r.seed;
        p.stage = "compute_land_value";
        progress(p);
      }

      LandValueConfig lc{};
      lc.requireOutsideConnection = sim.config().requireOutsideConnection;
      landValueRes = ComputeLandValue(world, lc, haveTraffic ? &trafficRes : nullptr, roadToEdgePtr);
      haveLandValue = true;
    }

    const std::string seedHex = Hex16NoPrefix(r.seed);

    JsonValue entry = JsonValue::MakeObject();
    AddJsonKV(entry, "rank", JsonValue::MakeNumber(static_cast<double>(entryIndex + 1)));
    AddJsonKV(entry, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
    AddJsonKV(entry, "seedHex", JsonValue::MakeString(HexU64(r.seed)));
    AddJsonKV(entry, "record", MineRecordToJson(r));

    if (!entryClusters.empty() && static_cast<std::size_t>(entryIndex) < entryClusters.size()) {
      AddJsonKV(entry, "cluster", JsonValue::MakeNumber(static_cast<double>(entryClusters[static_cast<std::size_t>(entryIndex)])));
    }

    // Attach a small neighbor list for local navigation (optional).
    if (localCfg.writeNeighbors && neighborsRes.ok && static_cast<std::size_t>(entryIndex) < neighborsRes.neighbors.size() &&
        static_cast<std::size_t>(entryIndex) < neighborsRes.distances.size()) {
      JsonValue nn = JsonValue::MakeArray();
      const auto& ns = neighborsRes.neighbors[static_cast<std::size_t>(entryIndex)];
      const auto& ds = neighborsRes.distances[static_cast<std::size_t>(entryIndex)];
      const std::size_t m = std::min(ns.size(), ds.size());
      nn.arrayValue.reserve(m);
      for (std::size_t j = 0; j < m; ++j) {
        const int nEntry = ns[j];
        if (nEntry < 0 || nEntry >= total) continue;
        const int nRecIndex = selectedIndices[static_cast<std::size_t>(nEntry)];
        if (nRecIndex < 0 || static_cast<std::size_t>(nRecIndex) >= recs.size()) continue;
        const MineRecord& nr = recs[static_cast<std::size_t>(nRecIndex)];

        JsonValue o = JsonValue::MakeObject();
        AddJsonKV(o, "rank", JsonValue::MakeNumber(static_cast<double>(nEntry + 1)));
        AddJsonKV(o, "seed", JsonValue::MakeNumber(static_cast<double>(nr.seed)));
        AddJsonKV(o, "seedHex", JsonValue::MakeString(HexU64(nr.seed)));
        AddJsonKV(o, "dist", JsonValue::MakeNumber(ds[j]));
        nn.arrayValue.push_back(std::move(o));
      }
      AddJsonKV(entry, "neighbors", std::move(nn));
    }

    JsonValue imgs = JsonValue::MakeObject();

    for (ExportLayer layer : localCfg.layers) {
      if (progress) {
        MineGalleryProgress p;
        p.index = entryIndex;
        p.total = total;
        p.seed = r.seed;
        p.stage = std::string("render_") + ExportLayerName(layer);
        progress(p);
      }

      PpmImage img = RenderPpmLayer(world, layer,
                                   haveLandValue ? &landValueRes : nullptr,
                                   haveTraffic ? &trafficRes : nullptr,
                                   haveGoods ? &goodsRes : nullptr);
      img = ScaleNearest(img, localCfg.exportScale);

      const std::filesystem::path rel = std::filesystem::path("thumbs") /
          ("rank_" + std::to_string(entryIndex + 1) + "_" + seedHex + "_" + ExportLayerName(layer) + "." + localCfg.format);

      const std::filesystem::path abs = localCfg.outDir / rel;

      std::string werr;
      if (!WriteImageWithExt(abs, img, werr)) {
        outErr = "Failed to write thumbnail: " + abs.string() + " (" + werr + ")";
        return false;
      }

      AddJsonKV(imgs, ExportLayerName(layer), JsonValue::MakeString(rel.generic_string()));

      // Lazily allocate contact sheet once we know thumb dims.
      if (localCfg.writeContactSheet && layer == primaryLayer) {
        if (thumbW == 0 || thumbH == 0) {
          thumbW = img.width;
          thumbH = img.height;
          if (thumbW <= 0 || thumbH <= 0) {
            outErr = "Invalid thumbnail dimensions";
            return false;
          }
          sheetRows = (total + localCfg.contactSheetCols - 1) / localCfg.contactSheetCols;
          const int sheetW = localCfg.contactSheetCols * thumbW + (localCfg.contactSheetCols - 1) * localCfg.contactSheetPaddingPx;
          const int sheetH = sheetRows * thumbH + (sheetRows - 1) * localCfg.contactSheetPaddingPx;
          contactSheet = MakeBlankPpm(sheetW, sheetH, 18, 18, 18);
        }

        const int col = entryIndex % localCfg.contactSheetCols;
        const int row = entryIndex / localCfg.contactSheetCols;
        const int dstX = col * (thumbW + localCfg.contactSheetPaddingPx);
        const int dstY = row * (thumbH + localCfg.contactSheetPaddingPx);
        BlitPpm(img, contactSheet, dstX, dstY);
      }
    }

    AddJsonKV(entry, "images", std::move(imgs));
    entries.arrayValue.push_back(std::move(entry));
  }

  AddJsonKV(manifest, "entries", std::move(entries));

  // Finalize/write traces.json (optional).
  if (localCfg.writeTraces && !traceMetrics.empty()) {
    // Min/max ranges per metric (for consistent scaling across cards).
    JsonValue mins = JsonValue::MakeArray();
    JsonValue maxs = JsonValue::MakeArray();
    mins.arrayValue.reserve(traceMetrics.size());
    maxs.arrayValue.reserve(traceMetrics.size());
    for (std::size_t mi = 0; mi < traceMetrics.size(); ++mi) {
      double lo = traceMin[mi];
      double hi = traceMax[mi];
      if (!std::isfinite(lo) || !std::isfinite(hi)) {
        lo = 0.0;
        hi = 0.0;
      }
      if (hi < lo) std::swap(lo, hi);
      mins.arrayValue.push_back(JsonValue::MakeNumber(lo));
      maxs.arrayValue.push_back(JsonValue::MakeNumber(hi));
    }
    AddJsonKV(tracesObj, "min", std::move(mins));
    AddJsonKV(tracesObj, "max", std::move(maxs));
    AddJsonKV(tracesObj, "series", std::move(tracesSeries));

    // Write traces.json for downstream tools (and keep HTML smaller).
    tracesPath = localCfg.outDir / "traces.json";
    {
      std::string jerr;
      JsonWriteOptions opt;
      opt.pretty = true;
      opt.indent = 2;
      opt.sortKeys = false;
      if (!WriteJsonFile(tracesPath.string(), tracesObj, jerr, opt)) {
        outErr = "Failed to write traces.json: " + jerr;
        return false;
      }
      AddJsonKV(manifest, "traces", JsonValue::MakeString(tracesPath.filename().generic_string()));
    }

    // Inline (minified) JSON for the HTML sparkline controls.
    {
      JsonWriteOptions opt;
      opt.pretty = false;
      opt.sortKeys = false;
      tracesJsonInline = JsonStringify(tracesObj, opt);
    }
  }

  // Compute a 2D embedding for the selected seeds (optional).
  if (localCfg.writeEmbeddingPlot) {
    embeddingRes = ComputeMineEmbeddingMDS(recs, selectedIndices, localCfg.embeddingCfg);

    JsonValue emb = JsonValue::MakeObject();
    AddJsonKV(emb, "type", JsonValue::MakeString("procisocity_mine_embedding"));
    AddJsonKV(emb, "version", JsonValue::MakeNumber(1));
    AddJsonKV(emb, "ok", JsonValue::MakeBool(embeddingRes.ok));
    if (!embeddingRes.warning.empty()) {
      AddJsonKV(emb, "warning", JsonValue::MakeString(embeddingRes.warning));
    }
    AddJsonKV(emb, "space", JsonValue::MakeString(MineDiversityModeName(embeddingRes.cfg.space)));
    AddJsonKV(emb, "layoutWeight", JsonValue::MakeNumber(embeddingRes.cfg.layoutWeight));
    AddJsonKV(emb, "robustScaling", JsonValue::MakeBool(embeddingRes.cfg.robustScaling));
    AddJsonKV(emb, "powerIters", JsonValue::MakeNumber(static_cast<double>(embeddingRes.cfg.powerIters)));
    AddJsonKV(emb, "eigen1", JsonValue::MakeNumber(embeddingRes.eigen1));
    AddJsonKV(emb, "eigen2", JsonValue::MakeNumber(embeddingRes.eigen2));
    {
      JsonValue ms = JsonValue::MakeArray();
      for (MineMetric m : embeddingRes.cfg.metrics) {
        ms.arrayValue.push_back(JsonValue::MakeString(MineMetricName(m)));
      }
      AddJsonKV(emb, "metrics", std::move(ms));
    }

    if (localCfg.writeClusters && clustersObj.isObject()) {
      // Duplicate cluster summary into embedding.json for self-contained tooling.
      AddJsonKV(emb, "clusters", clustersObj);
    }

    JsonValue pts = JsonValue::MakeArray();
    pts.arrayValue.reserve(selectedIndices.size());

    for (int entryIndex = 0; entryIndex < total; ++entryIndex) {
      const int recIndex = selectedIndices[static_cast<std::size_t>(entryIndex)];
      const MineRecord& r = recs[static_cast<std::size_t>(recIndex)];
      const std::string seedHex = Hex16NoPrefix(r.seed);

      double x = 0.0;
      double y = 0.0;
      if (embeddingRes.ok && embeddingRes.points.size() == selectedIndices.size()) {
        x = embeddingRes.points[static_cast<std::size_t>(entryIndex)].x;
        y = embeddingRes.points[static_cast<std::size_t>(entryIndex)].y;
      }

      const std::string imgRel = (std::filesystem::path("thumbs") /
          ("rank_" + std::to_string(entryIndex + 1) + "_" + seedHex + "_" + ExportLayerName(primaryLayer) + "." + localCfg.format)).generic_string();

      JsonValue p = JsonValue::MakeObject();
      AddJsonKV(p, "rank", JsonValue::MakeNumber(static_cast<double>(entryIndex + 1)));
      AddJsonKV(p, "recIndex", JsonValue::MakeNumber(static_cast<double>(recIndex)));
      AddJsonKV(p, "seed", JsonValue::MakeNumber(static_cast<double>(r.seed)));
      AddJsonKV(p, "seedHex", JsonValue::MakeString(HexU64(r.seed)));
      AddJsonKV(p, "score", JsonValue::MakeNumber(r.score));
      AddJsonKV(p, "population", JsonValue::MakeNumber(static_cast<double>(r.stats.population)));
      AddJsonKV(p, "happiness", JsonValue::MakeNumber(r.stats.happiness));
      AddJsonKV(p, "congestion", JsonValue::MakeNumber(r.stats.trafficCongestion));
      if (r.outlierLof > 0.0) {
        AddJsonKV(p, "lof", JsonValue::MakeNumber(r.outlierLof));
        AddJsonKV(p, "novelty", JsonValue::MakeNumber(r.novelty));
      }
      AddJsonKV(p, "phash", JsonValue::MakeString(HexU64(r.overlayPHash)));
      if (!entryClusters.empty() && static_cast<std::size_t>(entryIndex) < entryClusters.size()) {
        AddJsonKV(p, "cluster", JsonValue::MakeNumber(static_cast<double>(entryClusters[static_cast<std::size_t>(entryIndex)])));
      }
      AddJsonKV(p, "x", JsonValue::MakeNumber(x));
      AddJsonKV(p, "y", JsonValue::MakeNumber(y));
      AddJsonKV(p, "img", JsonValue::MakeString(imgRel));
      pts.arrayValue.push_back(std::move(p));
    }

    AddJsonKV(emb, "points", std::move(pts));

    // Write embedding.json (for re-use outside the HTML, and to keep HTML simpler).
    embeddingPath = localCfg.outDir / "embedding.json";
    {
      std::string jerr;
      JsonWriteOptions opt;
      opt.pretty = true;
      opt.indent = 2;
      opt.sortKeys = false;
      if (!WriteJsonFile(embeddingPath.string(), emb, jerr, opt)) {
        outErr = "Failed to write embedding.json: " + jerr;
        return false;
      }
      AddJsonKV(manifest, "embedding", JsonValue::MakeString(embeddingPath.filename().generic_string()));
    }

    // Inline (minified) JSON for the HTML plot.
    {
      JsonWriteOptions opt;
      opt.pretty = false;
      opt.sortKeys = false;
      embeddingJsonInline = JsonStringify(emb, opt);
    }
  }

  // Write contact sheet.
  std::filesystem::path contactSheetPath;
  if (localCfg.writeContactSheet && contactSheet.width > 0 && contactSheet.height > 0) {
    contactSheetPath = localCfg.outDir / (std::string("contact_sheet_") + ExportLayerName(primaryLayer) + "." + localCfg.format);
    std::string werr;
    if (!WriteImageWithExt(contactSheetPath, contactSheet, werr)) {
      outErr = "Failed to write contact sheet: " + contactSheetPath.string() + " (" + werr + ")";
      return false;
    }
    AddJsonKV(manifest, "contactSheet", JsonValue::MakeString(contactSheetPath.filename().generic_string()));
  }

  // Write JSON manifest.
  std::filesystem::path manifestPath;
  if (localCfg.writeJson) {
    manifestPath = localCfg.outDir / "gallery.json";
    std::string jerr;
    JsonWriteOptions opt;
    opt.pretty = true;
    opt.indent = 2;
    opt.sortKeys = false;
    if (!WriteJsonFile(manifestPath.string(), manifest, jerr, opt)) {
      outErr = "Failed to write gallery.json: " + jerr;
      return false;
    }
  }

  // Write HTML index.
  std::filesystem::path indexPath;
  if (localCfg.writeHtml) {
    indexPath = localCfg.outDir / "index.html";

    std::ostringstream html;
    html << "<!doctype html>\n";
    html << "<html><head><meta charset='utf-8'>\n";
    html << "<meta name='viewport' content='width=device-width, initial-scale=1'>\n";
    html << "<title>ProcIsoCity Mine Gallery</title>\n";
    html << "<style>\n";
    html << "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:16px;background:#111;color:#eee;}\n";
    html << "a{color:#8bd;}\n";
    html << ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px;}\n";
    html << ".card{background:#1b1b1b;border:1px solid #333;border-radius:10px;padding:10px;}\n";
    html << ".card img{width:100%;height:auto;image-rendering:pixelated;border-radius:8px;border:1px solid #2a2a2a;}\n";
    html << ".meta{font-size:13px;line-height:1.35;margin-top:8px;color:#ddd;}\n";
    html << ".meta code{color:#ddd;}\n";
    html << ".chips{margin-top:6px;display:flex;flex-wrap:wrap;gap:6px;}\n";
    html << ".chip{font-size:12px;padding:2px 8px;border-radius:999px;background:#242424;border:1px solid #333;}\n";
    html << ".clusterBar{display:flex;flex-wrap:wrap;gap:8px;margin:10px 0 14px 0;}\n";
    html << ".clusterItem{display:flex;align-items:center;gap:6px;padding:4px 10px;border-radius:999px;background:#202020;border:1px solid #333;font-size:12px;}\n";
    html << ".clusterSwatch{width:12px;height:12px;border-radius:3px;border:1px solid #333;display:inline-block;}\n";
    html << ".clusterTools{display:flex;gap:8px;margin:0 0 8px 0;}\n";
    html << ".clusterTools button{background:#222;color:#eee;border:1px solid #444;border-radius:8px;padding:6px 10px;cursor:pointer;}\n";
    html << ".clusterTools button:hover{border-color:#666;}\n";
    html << ".embedWrap{position:relative;max-width:1100px;margin:14px 0;}\n";
    html << "#embed_canvas{width:100%;height:520px;max-width:1100px;background:#0d0d0d;border:1px solid #333;border-radius:10px;}\n";
    html << "#embed_tooltip{position:absolute;display:none;pointer-events:none;background:rgba(0,0,0,0.88);border:1px solid #333;border-radius:8px;padding:8px;font-size:12px;line-height:1.35;color:#eee;z-index:10;}\n";
    html << "#embed_tooltip code{color:#ddd;}\n";
    html << ".traceTools{display:flex;flex-wrap:wrap;align-items:center;gap:10px;margin:10px 0 14px 0;}\n";
    html << ".traceTools label{display:flex;align-items:center;gap:8px;font-size:13px;}\n";
    html << ".traceTools select{background:#222;color:#eee;border:1px solid #444;border-radius:8px;padding:6px 10px;}\n";
    html << ".traceRange{font-size:12px;color:#bbb;}\n";
    html << ".spark{width:100%;height:56px;margin-top:8px;background:#0d0d0d;border:1px solid #2a2a2a;border-radius:8px;}\n";
    html << "</style></head><body>\n";
    html << "<h1>ProcIsoCity Mine Gallery</h1>\n";

    html << "<p>Seeds exported: <b>" << total << "</b>. Primary layer: <code>" << EscapeHtml(ExportLayerName(primaryLayer)) << "</code>.";
    if (!manifestPath.empty()) {
      html << " Manifest: <a href='gallery.json'>gallery.json</a>.";
    }
    if (!neighborsPath.empty()) {
      html << " Neighbors: <a href='" << EscapeHtml(neighborsPath.filename().generic_string()) << "'>neighbors.json</a>.";
    }
    if (!tracesPath.empty()) {
      html << " Traces: <a href='" << EscapeHtml(tracesPath.filename().generic_string()) << "'>traces.json</a>.";
    }
    html << "</p>\n";

    if (!contactSheetPath.empty()) {
      html << "<p><a href='" << EscapeHtml(contactSheetPath.filename().generic_string()) << "'>";
      html << "<img src='" << EscapeHtml(contactSheetPath.filename().generic_string()) << "' alt='contact sheet' style='max-width:100%;height:auto;border:1px solid #333;border-radius:10px;image-rendering:pixelated'></a></p>\n";
    }

    // Optional per-day KPI traces (sparklines).
    if (localCfg.writeTraces && !tracesJsonInline.empty()) {
      html << "<h2>Traces</h2>\n";
      html << "<p>Per-day KPI sparklines from <b>day 0</b> to <b>day " << days << "</b>. "
              "Choose a metric to render in each card (global scaling across the selected seeds).</p>\n";

      html << "<div class='traceTools'>";
      html << "<label>Metric <select id='trace_metric'></select></label>";
      html << "<span id='trace_range' class='traceRange'></span>";
      html << "</div>\n";

      html << "<script id='trace_data' type='application/json'>\n";
      html << tracesJsonInline;
      html << "\n</script>\n";
    }

    // Optional cluster controls.
    if (localCfg.writeClusters && !clusterJsonInline.empty()) {
      html << "<h2>Clusters</h2>\n";
      html << "<p>Deterministic <b>k-medoids</b> clustering in <code>" << EscapeHtml(MineDiversityModeName(clusterRes.cfg.space)) << "</code> space";
      html << " (k=" << clusterRes.cfg.k << ")";
      html << " · avg silhouette=" << std::fixed << std::setprecision(3) << clusterRes.avgSilhouette;
      html << " · cost=" << std::setprecision(3) << clusterRes.totalCost;
      html << ". Toggle clusters to filter both the gallery cards and the embedding (if enabled).";
      html << "</p>\n";
      if (!clusterRes.warning.empty()) {
        html << "<p><b>Note:</b> " << EscapeHtml(clusterRes.warning) << "</p>\n";
      }
      html << "<div id='cluster_tools' class='clusterTools'></div>\n";
      html << "<div id='cluster_bar' class='clusterBar'></div>\n";
      html << "<script id='cluster_data' type='application/json'>\n";
      html << clusterJsonInline << "\n";
      html << "</script>\n";
    }

    // Optional interactive embedding.
    if (localCfg.writeEmbeddingPlot && !embeddingJsonInline.empty()) {
      html << "<h2>Embedding map</h2>\n";
      html << "<p>2D embedding via <b>classical MDS</b> over <code>" << EscapeHtml(MineDiversityModeName(localCfg.embeddingCfg.space)) << "</code> distance. ";
      html << "Hover for details; click a point to open the primary-layer thumbnail.</p>\n";
      if (!embeddingRes.warning.empty()) {
        html << "<p><b>Note:</b> " << EscapeHtml(embeddingRes.warning) << "</p>\n";
      }

      html << "<div class='embedWrap'>\n";
      html << "<canvas id='embed_canvas'></canvas>\n";
      html << "<div id='embed_tooltip'></div>\n";
      html << "</div>\n";

      html << "<script id='embed_data' type='application/json'>\n";
      html << embeddingJsonInline << "\n";
      html << "</script>\n";

      html << "<script>\n";
      html << R"EMBEDJS(
(function(){
  const canvas = document.getElementById('embed_canvas');
  const tip = document.getElementById('embed_tooltip');
  const dataEl = document.getElementById('embed_data');
  if (!canvas || !dataEl) return;

  let data = null;
  try { data = JSON.parse(dataEl.textContent || 'null'); } catch (e) { console.error(e); return; }
  const pts = (data && data.points) ? data.points : [];
  if (!Array.isArray(pts) || pts.length === 0) {
    const ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.fillStyle = '#eee';
      ctx.font = '14px system-ui, sans-serif';
      ctx.fillText('No embedding points', 16, 24);
    }
    return;
  }

  // Range helpers.
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minS = Infinity, maxS = -Infinity;
  for (const p of pts) {
    const x = Number(p.x) || 0;
    const y = Number(p.y) || 0;
    const s = Number(p.score) || 0;
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (s < minS) minS = s; if (s > maxS) maxS = s;
  }

  function clamp01(t){ return t < 0 ? 0 : (t > 1 ? 1 : t); }
  function scoreT(s){
    if (!(maxS > minS)) return 0.5;
    return clamp01((s - minS) / (maxS - minS));
  }

  // Cluster palette + filter (optional).
  function hslToRgb(h, s, l){
    h = ((h % 360) + 360) % 360;
    s = clamp01(s);
    l = clamp01(l);
    const c = (1 - Math.abs(2*l - 1)) * s;
    const hp = h / 60;
    const x = c * (1 - Math.abs((hp % 2) - 1));
    let r=0,g=0,b=0;
    if (0 <= hp && hp < 1) { r=c; g=x; b=0; }
    else if (1 <= hp && hp < 2) { r=x; g=c; b=0; }
    else if (2 <= hp && hp < 3) { r=0; g=c; b=x; }
    else if (3 <= hp && hp < 4) { r=0; g=x; b=c; }
    else if (4 <= hp && hp < 5) { r=x; g=0; b=c; }
    else { r=c; g=0; b=x; }
    const m = l - c/2;
    return [Math.round(255*(r+m)), Math.round(255*(g+m)), Math.round(255*(b+m))];
  }

  function clusterColor(cid, a){
    a = (a == null) ? 1 : a;
    cid = Number(cid);
    if (!isFinite(cid) || cid < 0) return `rgba(160,160,160,${a})`;
    const g = 0.61803398875; // golden ratio conjugate
    const hue = ((cid * g) % 1) * 360;
    const rgb = hslToRgb(hue, 0.65, 0.55);
    return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${a})`;
  }

  if (!window.__procisocity_clusterColor) {
    window.__procisocity_clusterColor = clusterColor;
  }

  let enabledClusters = null;

  function isVisible(p){
    if (!enabledClusters) return true;
    const cid = Number(p.cluster);
    if (!isFinite(cid) || cid < 0) return true;
    return enabledClusters.has(cid);
  }

  window.addEventListener('procisocity_cluster_filter', (ev) => {
    const arr = ev && ev.detail && ev.detail.enabled;
    if (Array.isArray(arr)) {
      enabledClusters = new Set(arr.map(x => Number(x)).filter(x => isFinite(x) && x >= 0));
    } else {
      enabledClusters = null;
    }
    draw(hovered);
  });

  function resize(){
    const dpr = window.devicePixelRatio || 1;
    const cssW = Math.max(1, canvas.clientWidth || 900);
    const cssH = Math.max(1, canvas.clientHeight || 520);
    canvas.width = Math.floor(cssW * dpr);
    canvas.height = Math.floor(cssH * dpr);
    const ctx = canvas.getContext('2d');
    if (ctx) ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw();
  }

  function project(p, w, h){
    const pad = 24;
    const rx = (maxX - minX);
    const ry = (maxY - minY);
    const dx = (rx > 1e-12) ? ((Number(p.x) - minX) / rx) : 0.5;
    const dy = (ry > 1e-12) ? ((Number(p.y) - minY) / ry) : 0.5;
    const sx = pad + dx * (w - 2*pad);
    const sy = pad + (1.0 - dy) * (h - 2*pad);
    p._sx = sx;
    p._sy = sy;
  }

  function draw(highlight){
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const w = canvas.clientWidth || 900;
    const h = canvas.clientHeight || 520;
    ctx.clearRect(0, 0, w, h);

    // Grid.
    ctx.strokeStyle = 'rgba(255,255,255,0.07)';
    ctx.lineWidth = 1;
    for (let i = 1; i <= 4; ++i) {
      const gx = (w * i) / 5;
      const gy = (h * i) / 5;
      ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, h); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(w, gy); ctx.stroke();
    }

    // Points.
    for (const p of pts) project(p, w, h);
    for (let i = 0; i < pts.length; ++i) {
      const p = pts[i];
      if (!isVisible(p)) continue;
      const t = scoreT(Number(p.score) || 0);
      const r = 3 + 4 * t;
      const a = 0.25 + 0.75 * t;
      const cid = (p.cluster != null) ? Number(p.cluster) : -1;
      const colFn = window.__procisocity_clusterColor || clusterColor;
      ctx.fillStyle = colFn(cid, Number.isFinite(a) ? a : 0.5);
      ctx.beginPath();
      ctx.arc(p._sx, p._sy, r, 0, Math.PI * 2);
      ctx.fill();
    }

    if (highlight != null && highlight >= 0 && highlight < pts.length) {
      const p = pts[highlight];
      if (isVisible(p)) {
        ctx.strokeStyle = 'rgba(255,255,255,0.9)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p._sx, p._sy, 10, 0, Math.PI * 2);
        ctx.stroke();
      }
    }
  }

  function pick(mx, my){
    let best = -1;
    let bestD = 1e18;
    for (let i = 0; i < pts.length; ++i) {
      const p = pts[i];
      if (!isVisible(p)) continue;
      const dx = mx - p._sx;
      const dy = my - p._sy;
      const d = dx*dx + dy*dy;
      if (d < bestD) { bestD = d; best = i; }
    }
    return (bestD <= 12*12) ? best : -1;
  }

  function fmt(n, digits){
    const x = Number(n);
    if (!isFinite(x)) return 'nan';
    return x.toFixed(digits);
  }

  let hovered = -1;

  canvas.addEventListener('mousemove', (ev) => {
    const rect = canvas.getBoundingClientRect();
    const mx = ev.clientX - rect.left;
    const my = ev.clientY - rect.top;
    const idx = pick(mx, my);
    if (idx !== hovered) {
      hovered = idx;
      draw(hovered);
    }

    if (idx >= 0) {
      const p = pts[idx];
      tip.style.display = 'block';
      tip.style.left = (mx + 14) + 'px';
      tip.style.top = (my + 14) + 'px';
      const hasLof = (p.lof != null);
      const hasCl = (p.cluster != null);
      tip.innerHTML = `<b>#${p.rank}</b> seed <code>${p.seedHex}</code><br>` +
        `score=${fmt(p.score, 2)} pop=${p.population} happy=${fmt(p.happiness, 3)} cong=${fmt(p.congestion, 3)}` +
        (hasLof ? `<br>lof=${fmt(p.lof, 3)} nov=${fmt(p.novelty, 3)}` : '') +
        (hasCl ? `<br>cluster=${p.cluster}` : '') +
        `<br>pHash=<code>${p.phash}</code>`;
    } else {
      tip.style.display = 'none';
    }
  });

  canvas.addEventListener('mouseleave', () => {
    hovered = -1;
    tip.style.display = 'none';
    draw(-1);
  });

  canvas.addEventListener('click', () => {
    if (hovered >= 0) {
      const p = pts[hovered];
      if (p.img) window.location.href = p.img;
    }
  });

  window.addEventListener('resize', resize);
  resize();
})();
)EMBEDJS";
      html << "\n</script>\n";
    }

    html << "<div class='grid'>\n";
    for (int entryIndex = 0; entryIndex < total; ++entryIndex) {
      const int recIndex = selectedIndices[static_cast<std::size_t>(entryIndex)];
      const MineRecord& r = recs[static_cast<std::size_t>(recIndex)];
      const std::string seedHex = Hex16NoPrefix(r.seed);

      const std::string imgRel = (std::filesystem::path("thumbs") /
          ("rank_" + std::to_string(entryIndex + 1) + "_" + seedHex + "_" + ExportLayerName(primaryLayer) + "." + localCfg.format)).generic_string();

      int cl = -1;
      if (!entryClusters.empty() && static_cast<std::size_t>(entryIndex) < entryClusters.size()) {
        cl = entryClusters[static_cast<std::size_t>(entryIndex)];
      }
      if (localCfg.writeClusters) {
        html << "<div class='card' data-cluster='" << cl << "'>\n";
      } else {
        html << "<div class='card'>\n";
      }
      html << "<a href='" << EscapeHtml(imgRel) << "'><img src='" << EscapeHtml(imgRel) << "' alt='seed " << r.seed << "'></a>\n";
      html << "<div class='meta'>";
      html << "<b>#" << (entryIndex + 1) << "</b> seed=<code>" << r.seed << "</code> (<code>" << EscapeHtml(HexU64(r.seed)) << "</code>)<br>";
      html << "score=" << std::fixed << std::setprecision(2) << r.score;
      html << " · pop=" << r.stats.population;
      html << " · happy=" << std::setprecision(3) << r.stats.happiness;
      html << " · cong=" << std::setprecision(3) << r.stats.trafficCongestion;
      if (localCfg.writeClusters && cl >= 0) {
        html << " · cl=" << cl;
      }
      if (r.outlierLof > 0.0) {
        html << " · lof=" << std::setprecision(3) << r.outlierLof;
        html << " · nov=" << std::setprecision(3) << r.novelty;
      }
      html << "<br>phash=<code>" << EscapeHtml(HexU64(r.overlayPHash)) << "</code>";
      html << "</div>\n";

      // Per-day trace sparkline canvas (optional).
      if (localCfg.writeTraces && !tracesJsonInline.empty()) {
        html << "<canvas class='spark' data-entry='" << entryIndex << "'></canvas>\n";
      }

      if (localCfg.layers.size() > 1) {
        html << "<div class='chips'>";
        for (ExportLayer l : localCfg.layers) {
          const std::string rel = (std::filesystem::path("thumbs") /
              ("rank_" + std::to_string(entryIndex + 1) + "_" + seedHex + "_" + ExportLayerName(l) + "." + localCfg.format)).generic_string();
          html << "<a class='chip' href='" << EscapeHtml(rel) << "'>" << EscapeHtml(ExportLayerName(l)) << "</a>";
        }
        html << "</div>\n";
      }

      // Nearest neighbors (optional).
      if (localCfg.writeNeighbors && neighborsRes.ok && static_cast<std::size_t>(entryIndex) < neighborsRes.neighbors.size() &&
          static_cast<std::size_t>(entryIndex) < neighborsRes.distances.size()) {
        const auto& ns = neighborsRes.neighbors[static_cast<std::size_t>(entryIndex)];
        const auto& ds = neighborsRes.distances[static_cast<std::size_t>(entryIndex)];
        const std::size_t m = std::min(ns.size(), ds.size());
        if (m > 0) {
          html << "<div class='chips'>";
          html << "<span class='chip' title='k-nearest neighbors in " << EscapeHtml(MineDiversityModeName(neighborsRes.cfg.space)) << " space'>nn</span>";
          for (std::size_t j = 0; j < m; ++j) {
            const int nEntry = ns[j];
            if (nEntry < 0 || nEntry >= total) continue;
            const int nRecIndex = selectedIndices[static_cast<std::size_t>(nEntry)];
            if (nRecIndex < 0 || static_cast<std::size_t>(nRecIndex) >= recs.size()) continue;
            const MineRecord& nr = recs[static_cast<std::size_t>(nRecIndex)];
            const std::string nSeedHex = Hex16NoPrefix(nr.seed);
            const std::string nImgRel = (std::filesystem::path("thumbs") /
                ("rank_" + std::to_string(nEntry + 1) + "_" + nSeedHex + "_" + ExportLayerName(primaryLayer) + "." + localCfg.format)).generic_string();
            std::ostringstream tit;
            tit << "dist=" << std::fixed << std::setprecision(3) << ds[j] << " seed=" << HexU64(nr.seed);
            html << "<a class='chip' href='" << EscapeHtml(nImgRel) << "' title='" << EscapeHtml(tit.str()) << "'>#" << (nEntry + 1) << "</a>";
          }
          html << "</div>\n";
        }
      }
      html << "</div>\n";
    }
    html << "</div>\n";

    // Trace sparkline renderer (optional).
    if (localCfg.writeTraces && !tracesJsonInline.empty()) {
      html << "<script>\n";
      html << R"TRACEJS(
(function(){
  const dataEl = document.getElementById('trace_data');
  const selectEl = document.getElementById('trace_metric');
  const rangeEl = document.getElementById('trace_range');
  if (!dataEl || !selectEl) return;

  let data = null;
  try { data = JSON.parse(dataEl.textContent || 'null'); } catch (e) { console.error(e); return; }
  if (!data) return;

  const metrics = Array.isArray(data.metrics) ? data.metrics : [];
  const series = Array.isArray(data.series) ? data.series : [];
  const mins = Array.isArray(data.min) ? data.min : [];
  const maxs = Array.isArray(data.max) ? data.max : [];

  // Populate metric selector.
  while (selectEl.firstChild) selectEl.removeChild(selectEl.firstChild);
  for (let i = 0; i < metrics.length; ++i) {
    const opt = document.createElement('option');
    opt.value = String(i);
    opt.textContent = String(metrics[i]);
    selectEl.appendChild(opt);
  }

  const canvases = Array.from(document.querySelectorAll('canvas.spark'));
  if (canvases.length === 0) return;

  function fmtNum(v){
    if (!isFinite(v)) return 'n/a';
    const av = Math.abs(v);
    if (av >= 1e6) return (v/1e6).toFixed(2) + 'M';
    if (av >= 1e3) return (v/1e3).toFixed(2) + 'k';
    if (av >= 100) return v.toFixed(1);
    return v.toFixed(2);
  }

  function clamp01(t){ return t < 0 ? 0 : (t > 1 ? 1 : t); }

  function resizeCanvas(canvas){
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.floor(canvas.clientWidth * dpr));
    const h = Math.max(1, Math.floor(canvas.clientHeight * dpr));
    if (canvas.width !== w) canvas.width = w;
    if (canvas.height !== h) canvas.height = h;
    return {w, h, dpr};
  }

  function drawAll(){
    const idx = Math.max(0, Math.min(metrics.length - 1, Number(selectEl.value) || 0));
    const lo = Number(mins[idx]);
    const hi = Number(maxs[idx]);
    const den = (isFinite(lo) && isFinite(hi) && hi > lo) ? (hi - lo) : 0;

    if (rangeEl) {
      rangeEl.textContent = (den > 0) ? (`range: ${fmtNum(lo)} .. ${fmtNum(hi)}`) : '';
    }

    for (const canvas of canvases) {
      const entry = Number(canvas.getAttribute('data-entry')) || 0;
      const e = (entry >= 0 && entry < series.length) ? series[entry] : null;
      const vlist = e && Array.isArray(e.values) ? e.values[idx] : null;
      if (!Array.isArray(vlist) || vlist.length < 2) continue;

      const sz = resizeCanvas(canvas);
      const ctx = canvas.getContext('2d');
      if (!ctx) continue;
      ctx.clearRect(0, 0, sz.w, sz.h);

      // Baseline.
      ctx.strokeStyle = 'rgba(255,255,255,0.10)';
      ctx.lineWidth = Math.max(1, sz.dpr);
      ctx.beginPath();
      ctx.moveTo(0, sz.h - 0.5);
      ctx.lineTo(sz.w, sz.h - 0.5);
      ctx.stroke();

      ctx.strokeStyle = 'rgba(130,200,255,0.95)';
      ctx.lineWidth = Math.max(1.25 * sz.dpr, 1);
      ctx.beginPath();
      const n = vlist.length;
      for (let i = 0; i < n; ++i) {
        const x = (n === 1) ? 0 : (i / (n - 1)) * (sz.w - 1);
        let v = Number(vlist[i]);
        if (!isFinite(v)) v = lo;
        const u = (den > 0) ? clamp01((v - lo) / den) : 0.5;
        const y = (1 - u) * (sz.h - 1);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Last point dot.
      ctx.fillStyle = 'rgba(255,255,255,0.85)';
      const last = Number(vlist[n - 1]);
      const uLast = (den > 0 && isFinite(last)) ? clamp01((last - lo) / den) : 0.5;
      const xLast = sz.w - 1;
      const yLast = (1 - uLast) * (sz.h - 1);
      ctx.beginPath();
      ctx.arc(xLast, yLast, Math.max(1.6 * sz.dpr, 1.6), 0, Math.PI * 2);
      ctx.fill();
    }
  }

  selectEl.addEventListener('change', drawAll);
  window.addEventListener('resize', drawAll);
  if (!selectEl.value) selectEl.value = '0';
  drawAll();
})();
)TRACEJS";
      html << "\n</script>\n";
    }

    // Cluster filter / toggles (cards + embedding).
    if (localCfg.writeClusters && !clusterJsonInline.empty()) {
      html << "<script>\n";
      html << R"CLJS(
(function(){
  const dataEl = document.getElementById('cluster_data');
  const toolsEl = document.getElementById('cluster_tools');
  const barEl = document.getElementById('cluster_bar');
  if (!dataEl || !barEl) return;

  let data = null;
  try { data = JSON.parse(dataEl.textContent || 'null'); } catch (e) { console.error(e); return; }
  if (!data || !data.ok) return;
  const k = Number(data.k) || 0;
  const sizes = Array.isArray(data.clusterSizes) ? data.clusterSizes : [];
  if (k <= 0) return;

  // Palette (shared with embedding map if present).
  function clamp01(t){ return t < 0 ? 0 : (t > 1 ? 1 : t); }
  function hslToRgb(h, s, l){
    h = ((h % 360) + 360) % 360;
    s = clamp01(s);
    l = clamp01(l);
    const c = (1 - Math.abs(2*l - 1)) * s;
    const hp = h / 60;
    const x = c * (1 - Math.abs((hp % 2) - 1));
    let r=0,g=0,b=0;
    if (0 <= hp && hp < 1) { r=c; g=x; b=0; }
    else if (1 <= hp && hp < 2) { r=x; g=c; b=0; }
    else if (2 <= hp && hp < 3) { r=0; g=c; b=x; }
    else if (3 <= hp && hp < 4) { r=0; g=x; b=c; }
    else if (4 <= hp && hp < 5) { r=x; g=0; b=c; }
    else { r=c; g=0; b=x; }
    const m = l - c/2;
    return [Math.round(255*(r+m)), Math.round(255*(g+m)), Math.round(255*(b+m))];
  }

  function localClusterColor(cid, a){
    a = (a == null) ? 1 : a;
    cid = Number(cid);
    if (!isFinite(cid) || cid < 0) return `rgba(160,160,160,${a})`;
    const g = 0.61803398875;
    const hue = ((cid * g) % 1) * 360;
    const rgb = hslToRgb(hue, 0.65, 0.55);
    return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${a})`;
  }

  const colorFn = window.__procisocity_clusterColor || localClusterColor;
  if (!window.__procisocity_clusterColor) window.__procisocity_clusterColor = colorFn;

  const enabled = new Set();
  for (let i = 0; i < k; ++i) enabled.add(i);

  function emit(){
    try {
      window.dispatchEvent(new CustomEvent('procisocity_cluster_filter', {detail:{enabled:Array.from(enabled)}}));
    } catch (e) {}
  }

  const cards = Array.from(document.querySelectorAll('.card[data-cluster]'));
  function apply(){
    for (const card of cards) {
      const cid = Number(card.getAttribute('data-cluster'));
      const vis = (!isFinite(cid) || cid < 0) || enabled.has(cid);
      card.style.display = vis ? '' : 'none';
    }
    emit();
  }

  function rebuild(){
    barEl.innerHTML = '';
    for (let cid = 0; cid < k; ++cid) {
      const lab = document.createElement('label');
      lab.className = 'clusterItem';

      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = true;
      cb.addEventListener('change', () => {
        if (cb.checked) enabled.add(cid);
        else enabled.delete(cid);
        apply();
      });

      const sw = document.createElement('span');
      sw.className = 'clusterSwatch';
      sw.style.background = colorFn(cid, 1);

      const sz = (cid < sizes.length) ? (Number(sizes[cid]) || 0) : 0;
      const txt = document.createElement('span');
      txt.textContent = `c${cid} (${sz})`;

      lab.appendChild(cb);
      lab.appendChild(sw);
      lab.appendChild(txt);
      barEl.appendChild(lab);
    }
  }

  if (toolsEl) {
    toolsEl.innerHTML = '';
    const mkBtn = (name, fn) => {
      const b = document.createElement('button');
      b.textContent = name;
      b.addEventListener('click', (ev) => { ev.preventDefault(); fn(); });
      return b;
    };

    toolsEl.appendChild(mkBtn('All', () => {
      enabled.clear();
      for (let i = 0; i < k; ++i) enabled.add(i);
      const cbs = barEl.querySelectorAll('input[type=checkbox]');
      for (const cb of cbs) cb.checked = true;
      apply();
    }));

    toolsEl.appendChild(mkBtn('None', () => {
      enabled.clear();
      const cbs = barEl.querySelectorAll('input[type=checkbox]');
      for (const cb of cbs) cb.checked = false;
      apply();
    }));

    toolsEl.appendChild(mkBtn('Invert', () => {
      for (let i = 0; i < k; ++i) {
        if (enabled.has(i)) enabled.delete(i); else enabled.add(i);
      }
      const cbs = barEl.querySelectorAll('input[type=checkbox]');
      let idx = 0;
      for (const cb of cbs) {
        cb.checked = enabled.has(idx);
        idx++;
      }
      apply();
    }));
  }

  rebuild();
  apply();
})();
)CLJS";
      html << "\n</script>\n";
    }

    html << "</body></html>\n";

    if (!WriteTextFile(indexPath, html.str(), outErr)) {
      return false;
    }
  }

  if (outRes) {
    outRes->outDir = localCfg.outDir;
    outRes->indexHtml = indexPath;
    outRes->jsonManifest = manifestPath;
    outRes->contactSheet = contactSheetPath;
    outRes->embeddingJson = embeddingPath;
    outRes->neighborsJson = neighborsPath;
    outRes->tracesJson = tracesPath;
  }

  return true;
}

} // namespace isocity
