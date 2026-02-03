#include "isocity/Dossier.hpp"

#include "isocity/Chronicle.hpp"

#include "isocity/ConfigIO.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/LandUseMix.hpp"
#include "isocity/NoisePollution.hpp"
#include "isocity/HeatIsland.hpp"
#include "isocity/AirPollution.hpp"
#include "isocity/RunoffPollution.hpp"
#include "isocity/RunoffMitigation.hpp"
#include "isocity/SolarPotential.hpp"
#include "isocity/SkyView.hpp"
#include "isocity/EnergyModel.hpp"
#include "isocity/CarbonModel.hpp"
#include "isocity/CrimeModel.hpp"
#include "isocity/TrafficSafety.hpp"
#include "isocity/TransitAccessibility.hpp"
#include "isocity/FireRisk.hpp"
#include "isocity/Walkability.hpp"
#include "isocity/RoadHealth.hpp"
#include "isocity/Livability.hpp"
#include "isocity/HotspotAnalysis.hpp"
#include "isocity/JobOpportunity.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/StatsCsv.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>

namespace isocity {

namespace {

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

bool ParseJsonObjectText(const std::string& text, JsonValue& outObj, std::string& outErr)
{
  JsonValue v;
  if (!ParseJson(text, v, outErr)) return false;
  if (!v.isObject()) {
    outErr = "expected JSON object";
    return false;
  }
  outObj = std::move(v);
  outErr.clear();
  return true;
}

bool EnsureDir(const std::filesystem::path& dir)
{
  if (dir.empty()) return true;
  try {
    std::filesystem::create_directories(dir);
  } catch (...) {
    return false;
  }
  return true;
}

bool EnsureParentDir(const std::filesystem::path& file)
{
  try {
    const std::filesystem::path parent = file.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

float InferCoastalSeaLevel(const World& world)
{
  // Infer sea level by looking at edge-connected "ocean" water tiles.
  // This avoids inland lakes artificially raising the threshold.
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0.35f;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<std::uint8_t> visited(n, std::uint8_t{0});
  std::vector<std::size_t> stack;
  stack.reserve(static_cast<std::size_t>(w + h) * 2u);

  auto idx = [&](int x, int y) {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
  };

  auto pushIfOcean = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= w || y >= h) return;
    const std::size_t i = idx(x, y);
    if (visited[i]) return;
    const Tile& t = world.at(x, y);
    if (t.terrain != Terrain::Water) return;
    visited[i] = 1;
    stack.push_back(i);
  };

  // Seed with edge water tiles.
  for (int x = 0; x < w; ++x) {
    pushIfOcean(x, 0);
    pushIfOcean(x, h - 1);
  }
  for (int y = 0; y < h; ++y) {
    pushIfOcean(0, y);
    pushIfOcean(w - 1, y);
  }

  const bool anyEdgeWater = !stack.empty();
  float seaLevel = 0.0f;

  // Flood-fill ocean region.
  while (!stack.empty()) {
    const std::size_t i = stack.back();
    stack.pop_back();

    const int x = static_cast<int>(i % static_cast<std::size_t>(w));
    const int y = static_cast<int>(i / static_cast<std::size_t>(w));
    const Tile& t = world.at(x, y);
    seaLevel = std::max(seaLevel, t.height);

    if (x > 0) pushIfOcean(x - 1, y);
    if (x + 1 < w) pushIfOcean(x + 1, y);
    if (y > 0) pushIfOcean(x, y - 1);
    if (y + 1 < h) pushIfOcean(x, y + 1);
  }

  if (anyEdgeWater) return seaLevel;

  // Fallback: if no edge water, use max water height (lake) or a default.
  bool anyWater = false;
  float maxWaterH = 0.0f;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      if (t.terrain == Terrain::Water) {
        anyWater = true;
        maxWaterH = std::max(maxWaterH, t.height);
      }
    }
  }

  return anyWater ? maxWaterH : 0.35f;
}

void BuildHeightFieldAndDrainMask(const World& world, std::vector<float>& heights, std::vector<std::uint8_t>& drainMask)
{
  const int w = world.width();
  const int h = world.height();
  const std::size_t n =
      static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

  heights.assign(n, 0.0f);
  drainMask.assign(n, 0);
  if (w <= 0 || h <= 0) return;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      heights[i] = t.height;
      if (t.terrain == Terrain::Water) {
        drainMask[i] = 1;
      }
    }
  }
}

bool WriteCsvHeader(std::ostream& os)
{
  return WriteStatsCsvHeader(os);
}

bool WriteCsvRow(std::ostream& os, const Stats& s)
{
  return WriteStatsCsvRow(os, s);
}

bool WriteSummaryJson(const std::filesystem::path& outPath, const World& world,
                      std::uint64_t hash, const ProcGenConfig& procCfg,
                      const SimConfig& simCfg, const std::vector<Stats>& ticks,
                      const std::vector<ExportLayer>& layers2d,
                      const std::vector<ExportLayer>& layersIso,
                      bool include3d,
                      const std::string& imageExt,
                      int exportScale,
                      std::string& outErr)
{
  auto add = [](JsonValue& obj, const std::string& k, JsonValue v) {
    obj.objectValue.emplace_back(k, std::move(v));
  };

  JsonValue root = JsonValue::MakeObject();
  add(root, "tool", JsonValue::MakeString("proc_isocity_dossier"));
  add(root, "hash", JsonValue::MakeString(HexU64(hash)));
  add(root, "seed", JsonValue::MakeNumber(static_cast<double>(world.seed())));
  add(root, "width", JsonValue::MakeNumber(static_cast<double>(world.width())));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(world.height())));

  {
    std::string err;
    JsonValue procObj;
    if (!ParseJsonObjectText(ProcGenConfigToJson(procCfg, 2), procObj, err)) {
      outErr = "Failed to serialize ProcGenConfig to JSON: " + err;
      return false;
    }
    add(root, "proc", std::move(procObj));
  }

  {
    std::string err;
    JsonValue simObj;
    if (!ParseJsonObjectText(SimConfigToJson(simCfg, 2), simObj, err)) {
      outErr = "Failed to serialize SimConfig to JSON: " + err;
      return false;
    }
    add(root, "sim", std::move(simObj));
  }

  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(ticks.size());
    for (const Stats& s : ticks) {
      JsonValue st = JsonValue::MakeObject();
      add(st, "day", JsonValue::MakeNumber(static_cast<double>(s.day)));
      add(st, "population", JsonValue::MakeNumber(static_cast<double>(s.population)));
      add(st, "money", JsonValue::MakeNumber(static_cast<double>(s.money)));
      add(st, "housingCapacity", JsonValue::MakeNumber(static_cast<double>(s.housingCapacity)));
      add(st, "jobsCapacity", JsonValue::MakeNumber(static_cast<double>(s.jobsCapacity)));
      add(st, "jobsCapacityAccessible", JsonValue::MakeNumber(static_cast<double>(s.jobsCapacityAccessible)));
      add(st, "employed", JsonValue::MakeNumber(static_cast<double>(s.employed)));
      add(st, "happiness", JsonValue::MakeNumber(static_cast<double>(s.happiness)));
      add(st, "roads", JsonValue::MakeNumber(static_cast<double>(s.roads)));
      add(st, "parks", JsonValue::MakeNumber(static_cast<double>(s.parks)));
      add(st, "avgCommuteTime", JsonValue::MakeNumber(static_cast<double>(s.avgCommuteTime)));
      add(st, "trafficCongestion", JsonValue::MakeNumber(static_cast<double>(s.trafficCongestion)));
      add(st, "goodsDemand", JsonValue::MakeNumber(static_cast<double>(s.goodsDemand)));
      add(st, "goodsDelivered", JsonValue::MakeNumber(static_cast<double>(s.goodsDelivered)));
      add(st, "goodsSatisfaction", JsonValue::MakeNumber(static_cast<double>(s.goodsSatisfaction)));
      add(st, "avgLandValue", JsonValue::MakeNumber(static_cast<double>(s.avgLandValue)));
      add(st, "demandResidential", JsonValue::MakeNumber(static_cast<double>(s.demandResidential)));
      add(st, "demandCommercial", JsonValue::MakeNumber(static_cast<double>(s.demandCommercial)));
      add(st, "demandIndustrial", JsonValue::MakeNumber(static_cast<double>(s.demandIndustrial)));
      arr.arrayValue.push_back(std::move(st));
    }
    add(root, "ticks", std::move(arr));
  }

  {
    JsonValue exp = JsonValue::MakeObject();
    add(exp, "imageExt", JsonValue::MakeString(imageExt));
    add(exp, "scale", JsonValue::MakeNumber(static_cast<double>(exportScale)));
    add(exp, "include3d", JsonValue::MakeBool(include3d));

    JsonValue a2d = JsonValue::MakeArray();
    for (ExportLayer l : layers2d) a2d.arrayValue.push_back(JsonValue::MakeString(ExportLayerName(l)));
    add(exp, "layers2d", std::move(a2d));

    JsonValue aiso = JsonValue::MakeArray();
    for (ExportLayer l : layersIso) aiso.arrayValue.push_back(JsonValue::MakeString(ExportLayerName(l)));
    add(exp, "layersIso", std::move(aiso));

    add(root, "exports", std::move(exp));
  }

  const JsonWriteOptions wopt{.pretty = true, .indent = 2, .sortKeys = false};
  return WriteJsonFile(outPath.string(), root, outErr, wopt);
}

bool WriteHtmlReport(const std::filesystem::path& outPath,
                     const World& world,
                     std::uint64_t hash,
                     const ProcGenConfig& procCfg,
                     const SimConfig& simCfg,
                     const std::vector<Stats>& ticks,
                     const std::vector<ExportLayer>& layers2d,
                     const std::vector<ExportLayer>& layersIso,
                     bool include3d,
                     const std::string& imageExt,
                     int exportScale,
                     std::string& outErr)
{
  outErr.clear();
  if (!EnsureParentDir(outPath)) {
    outErr = "Failed to create report parent directory";
    return false;
  }

  const Stats st = ticks.empty() ? world.stats() : ticks.back();

  std::ofstream f(outPath, std::ios::binary);
  if (!f) {
    outErr = "Failed to open HTML report for writing";
    return false;
  }

  auto layerFile2d = [&](ExportLayer l) {
    return std::string("map_") + ExportLayerName(l) + "." + imageExt;
  };
  auto layerFileIso = [&](ExportLayer l) {
    return std::string("iso_") + ExportLayerName(l) + "." + imageExt;
  };
  const std::string layerDefault = layers2d.empty() ? "" : layerFile2d(layers2d.front());

  f << "<!doctype html>\n";
  f << "<html lang=\"en\">\n<head>\n";
  f << "<meta charset=\"utf-8\">\n";
  f << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  f << "<title>ProcIsoCity Dossier - seed " << world.seed() << "</title>\n";
  f << "<style>\n";
  f << "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Noto Sans,sans-serif; margin:24px; line-height:1.35;}\n";
  f << "code,pre{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;}\n";
  f << ".row{display:flex; gap:24px; flex-wrap:wrap; align-items:flex-start;}\n";
  f << ".card{border:1px solid #ddd; border-radius:10px; padding:16px; background:#fff;}\n";
  f << ".card h2{margin:0 0 12px 0; font-size:18px;}\n";
  f << ".meta{color:#444; font-size:14px;}\n";
  f << ".kv{border-collapse:collapse; font-size:14px;}\n";
  f << ".kv td{padding:2px 10px 2px 0; vertical-align:top;}\n";
  f << ".viewer{max-width:100%;}\n";
  f << ".viewer img{max-width:100%; height:auto; image-rendering:pixelated; border:1px solid #ddd; border-radius:8px;}\n";
  f << ".imgWrap{position:relative; display:inline-block; max-width:100%;}\n";
  f << ".imgWrap canvas{position:absolute; left:0; top:0; width:100%; height:100%; pointer-events:none;}\n";
  f << ".thumbs{display:grid; grid-template-columns:repeat(auto-fill, minmax(180px,1fr)); gap:12px;}\n";
  f << ".thumbs a{text-decoration:none; color:inherit;}\n";
  f << ".thumbs img{width:100%; height:auto; image-rendering:pixelated; border:1px solid #eee; border-radius:8px;}\n";
  f << ".small{font-size:13px; color:#444;}\n";
  f << "button{padding:4px 10px; border:1px solid #ccc; border-radius:8px; background:#f8f8f8; cursor:pointer;}\n";
  f << "button:hover{background:#f0f0f0;}\n";
  f << "select{padding:4px;}\n";
  f << "input[type=text]{padding:4px; border:1px solid #ccc; border-radius:8px;}\n";
  f << ".pill{display:inline-block; padding:2px 8px; border:1px solid #ddd; border-radius:999px; font-size:12px; margin:0 6px 6px 0;}\n";
  f << ".chronEntry{border-top:1px solid #eee; padding-top:10px; margin-top:10px;}\n";
  f << ".chronBody{white-space:pre-line;}\n";
  f << ".tone{font-weight:600;}\n";
  f << ".tone-good{color:#1b7f2a;}\n";
  f << ".tone-neutral{color:#444;}\n";
  f << ".tone-bad{color:#b00020;}\n";
  f << ".tone-alert{color:#b00020;}\n";
  f << "</style>\n";
  f << "</head>\n<body>\n";

  f << "<h1>ProcIsoCity dossier</h1>\n";
  f << "<div class=\"meta\">Seed <code>" << world.seed() << "</code> • "
    << world.width() << "×" << world.height() << " • hash <code>" << HexU64(hash) << "</code>";
  f << " • preset <code>" << ToString(procCfg.terrainPreset) << "</code>";
  f << " • road <code>" << ToString(procCfg.roadLayout) << "</code>";
  f << " • districting <code>" << ToString(procCfg.districtingMode) << "</code>";
  f << " • waterLevel <code>" << std::fixed << std::setprecision(3) << static_cast<double>(procCfg.waterLevel) << "</code>";
  f << " • requireOutside <code>" << (simCfg.requireOutsideConnection ? 1 : 0) << "</code>";
  f << "</div>\n";

  f << "<div class=\"row\">\n";
  f << "  <div class=\"card\" style=\"min-width:320px; flex:1\">\n";
  f << "    <h2>Stats</h2>\n";
  f << "    <table class=\"kv\">\n";
  f << "      <tr><td>Day</td><td><code>" << st.day << "</code></td></tr>\n";
  f << "      <tr><td>Population</td><td><code>" << st.population << "</code></td></tr>\n";
  f << "      <tr><td>Employed</td><td><code>" << st.employed << "</code></td></tr>\n";
  f << "      <tr><td>Money</td><td><code>" << st.money << "</code></td></tr>\n";
  f << "      <tr><td>Avg commute time</td><td><code>" << st.avgCommuteTime << "</code></td></tr>\n";
  f << "      <tr><td>Traffic congestion</td><td><code>" << st.trafficCongestion << "</code></td></tr>\n";
  f << "      <tr><td>Goods satisfaction</td><td><code>" << st.goodsSatisfaction << "</code></td></tr>\n";
  f << "      <tr><td>Avg land value</td><td><code>" << st.avgLandValue << "</code></td></tr>\n";
  f << "    </table>\n";
  f << "    <div class=\"small\" style=\"margin-top:10px\">Exports are written next to this HTML file.\n";
  f << "    Open images by clicking thumbnails; CSV/JSON are linked below.</div>\n";
  f << "  </div>\n";

  f << "  <div class=\"card\" style=\"flex:2; min-width:420px\">\n";
  f << "    <h2>Map viewer</h2>\n";
  f << "    <div>Layer: <select id=\"layerSel\"></select></div>\n";
  f << "    <div class=\"viewer\" style=\"margin-top:10px\">\n";
  f << "      <div id=\"imgWrap\" class=\"imgWrap\">\n";
  f << "        <img id=\"mainImg\" alt=\"layer\" src=\"" << layerDefault << "\">\n";
  f << "        <canvas id=\"overlayCanvas\"></canvas>\n";
  f << "      </div>\n";
  f << "    </div>\n";
  f << "    <div class=\"small\" style=\"margin-top:8px\">\n";
  f << "      Hover: <code id=\"tileCoord\">-</code>";
  f << "      <span id=\"tileInfo\"></span><br>\n";
  f << "      Pinned: <code id=\"pinCoord\">-</code>";
  f << "      <span id=\"pinInfo\"></span>\n";
  f << "      <span style=\"margin-left:8px\"></span>\n";
  f << "      <button id=\"pinClear\" type=\"button\">Clear</button>\n";
  f << "      <button id=\"pinCopy\" type=\"button\">Copy JSON</button>\n";
  f << "      <span style=\"margin-left:10px\">Go:</span> <input type=\"text\" id=\"pinGoto\" placeholder=\"x,y\" style=\"width:90px\"> <button id=\"pinGoBtn\" type=\"button\">Pin</button>\n";
  f << "    </div>\n";
  f << "    <div class=\"small\" style=\"margin-top:10px\">\n";
  f << "      Optional: load <code>tile_metrics.csv</code> for per-tile inspection (works even when opened as file://).\n";
  f << "      <div style=\"margin-top:6px\"><input type=\"file\" id=\"metricsFile\" accept=\".csv\"></div>\n";
  f << "      <div id=\"metricsStatus\"></div>\n";
  f << "      <div style=\"margin-top:10px\">\n";
  f << "        <div><b>Hover metrics</b> (numeric columns from <code>tile_metrics.csv</code>)</div>\n";
  f << "        <div style=\"display:flex; gap:8px; flex-wrap:wrap; align-items:center; margin-top:6px\">\n";
  f << "          <input type=\"text\" id=\"metricFilter\" placeholder=\"filter columns\" style=\"flex:1; min-width:200px\">\n";
  f << "          <button id=\"metricDefault\" type=\"button\">Default</button>\n";
  f << "          <button id=\"metricAll\" type=\"button\">All</button>\n";
  f << "          <button id=\"metricNone\" type=\"button\">None</button>\n";
  f << "        </div>\n";
  f << "        <div style=\"margin-top:6px\">\n";
  f << "          <select id=\"metricSel\" multiple size=\"10\" style=\"width:100%\"></select>\n";
  f << "        </div>\n";
  f << "      </div>\n";
  f << "    </div>\n";
  f << "  </div>\n";
  f << "</div>\n";

  f << "<h2 style=\"margin-top:28px\">Time series</h2>\n";
  f << "<div class=\"row\">\n";
  f << "  <div class=\"card\" style=\"flex:1; min-width:420px\">\n";
  f << "    <div class=\"small\">Optional: load <code>ticks.csv</code> to plot a metric over time (works even when opened as file://).\n";
  f << "      <div style=\"margin-top:6px\"><input type=\"file\" id=\"ticksFile\" accept=\".csv\"></div>\n";
  f << "      <div id=\"ticksStatus\"></div>\n";
  f << "    </div>\n";
  f << "    <div style=\"margin-top:10px\">Metric: <select id=\"tickMetric\"></select>\n";
  f << "      <label style=\"margin-left:12px\"><input type=\"checkbox\" id=\"tickNormalize\"> normalize</label>\n";
  f << "    </div>\n";
  f << "    <canvas id=\"tickChart\" style=\"margin-top:10px; width:100%; height:260px; border:1px solid #eee; border-radius:8px;\"></canvas>\n";
  f << "    <div class=\"small\" id=\"tickHint\" style=\"margin-top:6px\"></div>\n";
  f << "  </div>\n";
  f << "</div>\n";

  f << "<h2 style=\"margin-top:28px\">Chronicle</h2>\n";
  f << "<div class=\"row\">\n";
  f << "  <div class=\"card\" style=\"flex:1; min-width:420px\">\n";
  f << "    <div class=\"small\">Optional: load <code>chronicle.json</code> for a procedural daily newspaper/advisor feed (works even when opened as file://).\n";
  f << "      <div style=\"margin-top:6px\"><input type=\"file\" id=\"chronFile\" accept=\".json\"></div>\n";
  f << "      <div id=\"chronStatus\"></div>\n";
  f << "    </div>\n";
  f << "    <div style=\"margin-top:10px; display:flex; gap:8px; flex-wrap:wrap; align-items:center\">\n";
  f << "      <input type=\"text\" id=\"chronFilter\" placeholder=\"filter headlines / tags\" style=\"flex:1; min-width:200px\">\n";
  f << "      <select id=\"chronTone\"></select>\n";
  f << "    </div>\n";
  f << "    <div id=\"chronList\" style=\"margin-top:10px\"></div>\n";
  f << "  </div>\n";
  f << "</div>\n";

  if (!layersIso.empty() || include3d) {
    f << "<h2 style=\"margin-top:28px\">Isometric + 3D</h2>\n";
    f << "<div class=\"thumbs\">\n";
    for (ExportLayer l : layersIso) {
      const std::string fn = layerFileIso(l);
      f << "<a href=\"" << fn << "\"><div class=\"card\"><div><b>iso_" << ExportLayerName(l) << "</b></div>\n";
      f << "<img src=\"" << fn << "\" alt=\"iso" << ExportLayerName(l) << "\"></div></a>\n";
    }
    if (include3d) {
      const std::string fn = std::string("view3d_overlay.") + imageExt;
      f << "<a href=\"" << fn << "\"><div class=\"card\"><div><b>3d_overlay</b></div>\n";
      f << "<img src=\"" << fn << "\" alt=\"3d overlay\"></div></a>\n";
    }
    f << "</div>\n";
  }

  f << "<h2 style=\"margin-top:28px\">Top-down exports</h2>\n";
  f << "<div class=\"thumbs\">\n";
  for (ExportLayer l : layers2d) {
    const std::string fn = layerFile2d(l);
    f << "<a href=\"" << fn << "\"><div class=\"card\"><div><b>" << ExportLayerName(l) << "</b></div>\n";
    f << "<img src=\"" << fn << "\" alt=\"" << ExportLayerName(l) << "\"></div></a>\n";
  }
  f << "</div>\n";

  f << "<h2 style=\"margin-top:28px\">Data</h2>\n";
  f << "<ul>\n";
  f << "  <li><a href=\"summary.json\">summary.json</a></li>\n";
  f << "  <li><a href=\"ticks.csv\">ticks.csv</a></li>\n";
  f << "  <li><a href=\"chronicle.json\">chronicle.json</a></li>\n";
  f << "  <li><a href=\"chronicle.md\">chronicle.md</a></li>\n";
  f << "  <li><a href=\"tile_metrics.csv\">tile_metrics.csv</a></li>\n";
  f << "  <li><a href=\"world.bin\">world.bin</a></li>\n";
  f << "</ul>\n";

  // --- JS ---
  f << "<script>\n";
  f << "const MAP_W = " << world.width() << ";\n";
  f << "const MAP_H = " << world.height() << ";\n";
  f << "const EXPORT_SCALE = " << exportScale << ";\n";
  f << "const LAYERS_2D = [\n";
  for (std::size_t i = 0; i < layers2d.size(); ++i) {
    const ExportLayer l = layers2d[i];
    const std::string fn = layerFile2d(l);
    f << "  { key: '" << ExportLayerName(l) << "', file: '" << fn << "' }";
    if (i + 1 < layers2d.size()) f << ',';
    f << "\n";
  }
  f << "];\n";

  f << R"JS(
const layerSel = document.getElementById('layerSel');
const mainImg = document.getElementById('mainImg');
const tileCoord = document.getElementById('tileCoord');
const tileInfo = document.getElementById('tileInfo');

// --- Pin + overlay UI ---
const overlayCanvas = document.getElementById('overlayCanvas');
const pinCoord = document.getElementById('pinCoord');
const pinInfo = document.getElementById('pinInfo');
const pinClear = document.getElementById('pinClear');
const pinCopy = document.getElementById('pinCopy');
const pinGoto = document.getElementById('pinGoto');
const pinGoBtn = document.getElementById('pinGoBtn');

// --- Tile metrics UI ---
const metricsFile = document.getElementById('metricsFile');
const metricsStatus = document.getElementById('metricsStatus');
const metricSel = document.getElementById('metricSel');
const metricFilter = document.getElementById('metricFilter');
const metricDefaultBtn = document.getElementById('metricDefault');
const metricAllBtn = document.getElementById('metricAll');
const metricNoneBtn = document.getElementById('metricNone');

// --- Ticks chart UI ---
const ticksFile = document.getElementById('ticksFile');
const ticksStatus = document.getElementById('ticksStatus');
const tickMetric = document.getElementById('tickMetric');
const tickNormalize = document.getElementById('tickNormalize');
const tickChart = document.getElementById('tickChart');
const tickHint = document.getElementById('tickHint');

// --- Chronicle UI ---
const chronFile = document.getElementById('chronFile');
const chronStatus = document.getElementById('chronStatus');
const chronFilter = document.getElementById('chronFilter');
const chronTone = document.getElementById('chronTone');
const chronList = document.getElementById('chronList');

// -----------------------------
// Layer selector
// -----------------------------
for (const l of LAYERS_2D) {
  const opt = document.createElement('option');
  opt.value = l.file;
  opt.textContent = l.key;
  layerSel.appendChild(opt);
}

layerSel.addEventListener('change', () => {
  mainImg.src = layerSel.value;
  updateHashFromState();
});

// -----------------------------
// CSV parsing helpers (simple, no quotes)
// -----------------------------
function splitNonEmptyLines(text) {
  return text.split(/\r?\n/).filter(l => l.length > 0);
}

function tryParseNumber(s) {
  if (s == null) return null;
  const t = String(s).trim();
  if (t === '' || t.toLowerCase() === 'nan' || t.toLowerCase() === 'null') return null;
  const v = parseFloat(t);
  return Number.isFinite(v) ? v : null;
}

function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

// -----------------------------
// Tile metrics loader (generic)
// -----------------------------
let metrics = null; // {header, cols, types, numericKeys, stringKeys}
let metricFilterValue = '';

const DEFAULT_HOVER_METRICS = [
  'land_value', 'livability', 'intervention_priority',
  'traffic_crash_risk', 'crime_risk', 'noise', 'air_pollution',
  'heat_island', 'runoff_pollution', 'flood_depth', 'ponding_depth',
  'goods_fill', 'commute_traffic', 'road_centrality', 'road_vulnerability',
  'job_opportunity', 'job_access', 'transit_access', 'walkability'
];

const ALWAYS_INFO_FIELDS = ['terrain', 'overlay', 'level', 'district', 'height', 'occupants'];

function classifyColumns(header, sampleRows) {
  const types = {};
  for (let ci = 0; ci < header.length; ++ci) {
    const name = header[ci];
    if (name === 'x' || name === 'y') continue;
    if (name === 'terrain' || name === 'overlay') {
      types[name] = 'string';
      continue;
    }

    let seen = 0;
    let numeric = 0;
    for (const row of sampleRows) {
      if (ci >= row.length) continue;
      const s = row[ci];
      if (s == null) continue;
      const t = String(s).trim();
      if (t === '') continue;
      seen += 1;
      if (tryParseNumber(t) != null) numeric += 1;
    }

    // If the column looks mostly numeric, treat as numeric.
    if (seen > 0 && (numeric / seen) >= 0.80) {
      types[name] = 'number';
    } else {
      types[name] = 'string';
    }
  }
  return types;
}

function parseTileMetricsCsv(text) {
  const lines = splitNonEmptyLines(text);
  if (lines.length < 2) throw new Error('CSV has no data');

  const header = lines[0].split(',');
  const ix = header.indexOf('x');
  const iy = header.indexOf('y');
  if (ix < 0 || iy < 0) throw new Error('CSV missing x/y columns');

  const n = MAP_W * MAP_H;

  // Sample a few rows to classify columns.
  const sampleRows = [];
  const sampleCount = Math.min(200, lines.length - 1);
  for (let li = 1; li <= sampleCount; ++li) {
    sampleRows.push(lines[li].split(','));
  }

  const types = classifyColumns(header, sampleRows);

  const cols = {};
  const numericKeys = [];
  const stringKeys = [];

  for (const name of header) {
    if (name === 'x' || name === 'y') continue;
    const ty = types[name] || 'string';
    if (ty === 'number') {
      const arr = new Float32Array(n);
      arr.fill(NaN);
      cols[name] = arr;
      numericKeys.push(name);
    } else {
      cols[name] = new Array(n).fill('');
      stringKeys.push(name);
    }
  }

  for (let li = 1; li < lines.length; ++li) {
    const parts = lines[li].split(',');
    if (parts.length < 2) continue;
    const x = parseInt(parts[ix], 10);
    const y = parseInt(parts[iy], 10);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    if (x < 0 || y < 0 || x >= MAP_W || y >= MAP_H) continue;
    const i = y * MAP_W + x;

    for (let ci = 0; ci < header.length; ++ci) {
      if (ci === ix || ci === iy) continue;
      const name = header[ci];
      const arr = cols[name];
      if (!arr) continue;
      const s = (ci < parts.length) ? parts[ci] : '';
      if (types[name] === 'number') {
        const v = tryParseNumber(s);
        arr[i] = (v == null) ? NaN : v;
      } else {
        arr[i] = s;
      }
    }
  }

  return { header, cols, types, numericKeys, stringKeys };
}

function setMetrics(m) {
  metrics = m;
  if (metricsStatus) {
    metricsStatus.textContent = 'Loaded tile_metrics.csv (' + (MAP_W * MAP_H) + ' tiles, ' + metrics.numericKeys.length + ' numeric cols)';
  }
  rebuildMetricSelector();
  // Keep pinned info in sync once metrics are available.
  if (pinnedTile) setPinned(pinnedTile.tx, pinnedTile.ty, { updateHash: false });
}

function getSelectedMetrics() {
  if (!metricSel) return [];
  const out = [];
  for (const opt of metricSel.selectedOptions) out.push(opt.value);
  return out;
}

function rebuildMetricSelector() {
  if (!metricSel) return;

  const prev = new Set(getSelectedMetrics());

  metricSel.innerHTML = '';
  if (!metrics) return;

  const filter = metricFilterValue.trim().toLowerCase();
  const keys = metrics.numericKeys.slice().sort();

  for (const k of keys) {
    if (filter && !k.toLowerCase().includes(filter)) continue;
    const opt = document.createElement('option');
    opt.value = k;
    opt.textContent = k;
    if (prev.has(k)) opt.selected = true;
    metricSel.appendChild(opt);
  }

  // If nothing was selected (fresh load), apply a sane default.
  if (metricSel.selectedOptions.length === 0) {
    applyDefaultHoverMetrics();
  }
}

function applyDefaultHoverMetrics() {
  if (!metricSel || !metrics) return;
  const want = new Set(DEFAULT_HOVER_METRICS);
  for (const opt of metricSel.options) {
    opt.selected = want.has(opt.value);
  }
  // If none matched, select the first few columns.
  if (metricSel.selectedOptions.length === 0) {
    for (let i = 0; i < metricSel.options.length && i < 8; ++i) {
      metricSel.options[i].selected = true;
    }
  }
}

function applyAllHoverMetrics() {
  if (!metricSel) return;
  for (const opt of metricSel.options) opt.selected = true;
}

function applyNoneHoverMetrics() {
  if (!metricSel) return;
  for (const opt of metricSel.options) opt.selected = false;
}

if (metricFilter) {
  metricFilter.addEventListener('input', () => {
    metricFilterValue = metricFilter.value || '';
    rebuildMetricSelector();
  });
}

if (metricDefaultBtn) {
  metricDefaultBtn.addEventListener('click', () => applyDefaultHoverMetrics());
}
if (metricAllBtn) {
  metricAllBtn.addEventListener('click', () => applyAllHoverMetrics());
}
if (metricNoneBtn) {
  metricNoneBtn.addEventListener('click', () => applyNoneHoverMetrics());
}

async function tryAutoLoadMetrics() {
  if (!metricsStatus) return;
  try {
    const resp = await fetch('tile_metrics.csv');
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const txt = await resp.text();
    setMetrics(parseTileMetricsCsv(txt));
  } catch (e) {
    metricsStatus.textContent = 'Not loaded. Use the file picker above to load tile_metrics.csv.';
  }
}

tryAutoLoadMetrics();

if (metricsFile) {
  metricsFile.addEventListener('change', () => {
    const file = metricsFile.files && metricsFile.files[0];
    if (!file) return;
    const r = new FileReader();
    r.onload = () => {
      try {
        setMetrics(parseTileMetricsCsv(String(r.result)));
      } catch (e) {
        if (metricsStatus) metricsStatus.textContent = 'Failed to parse: ' + e;
      }
    };
    r.readAsText(file);
  });
}

function formatTileMetric(name, v) {
  if (v == null || !Number.isFinite(v)) return null;

  // Heuristic: costs are often stored in "milli-steps".
  if (name && name.includes('cost') && Math.abs(v) >= 1000) {
    const steps = v / 1000.0;
    return steps.toFixed(1);
  }

  // Integer-ish values: print without decimals.
  if (Math.abs(v - Math.round(v)) < 1e-6 && Math.abs(v) < 1e9) {
    return String(Math.round(v));
  }

  const a = Math.abs(v);
  if (a >= 1000) return v.toFixed(1);
  if (a >= 10) return v.toFixed(2);
  return v.toFixed(3);
}

// -----------------------------
// Map hover inspector + pinned tile + overlay
// -----------------------------
let hoverTile = null;   // {tx, ty}
let pinnedTile = null;  // {tx, ty}
let pinnedJson = null;

const overlayCtx = overlayCanvas ? overlayCanvas.getContext('2d') : null;

function eventToTile(ev) {
  const rect = mainImg.getBoundingClientRect();
  const u = (ev.clientX - rect.left) / rect.width;
  const v = (ev.clientY - rect.top) / rect.height;
  if (!Number.isFinite(u) || !Number.isFinite(v)) return null;
  if (u < 0 || v < 0 || u > 1 || v > 1) return null;
  if (!mainImg.naturalWidth || !mainImg.naturalHeight) return null;

  const px = Math.floor(u * mainImg.naturalWidth);
  const py = Math.floor(v * mainImg.naturalHeight);
  const tx = Math.floor(px / EXPORT_SCALE);
  const ty = Math.floor(py / EXPORT_SCALE);
  if (tx < 0 || ty < 0 || tx >= MAP_W || ty >= MAP_H) return null;
  return { tx, ty };
}

function resizeOverlayCanvas() {
  if (!overlayCanvas) return;
  const rect = mainImg.getBoundingClientRect();
  if (!rect || rect.width <= 0 || rect.height <= 0) return;
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(rect.width * dpr));
  const h = Math.max(1, Math.round(rect.height * dpr));
  if (overlayCanvas.width !== w) overlayCanvas.width = w;
  if (overlayCanvas.height !== h) overlayCanvas.height = h;
  drawOverlay();
}

function tileRectCanvas(tx, ty) {
  if (!overlayCanvas || !mainImg.naturalWidth || !mainImg.naturalHeight) return null;
  const sx = overlayCanvas.width / mainImg.naturalWidth;
  const sy = overlayCanvas.height / mainImg.naturalHeight;
  const x = tx * EXPORT_SCALE * sx;
  const y = ty * EXPORT_SCALE * sy;
  const w = EXPORT_SCALE * sx;
  const h = EXPORT_SCALE * sy;
  return { x, y, w, h };
}

function drawTileBox(tx, ty, cssLineWidth, strokeStyle, fillStyle) {
  if (!overlayCtx) return;
  const r = tileRectCanvas(tx, ty);
  if (!r) return;

  const dpr = window.devicePixelRatio || 1;
  const lw = Math.max(1, cssLineWidth * dpr);

  overlayCtx.save();
  if (fillStyle) {
    overlayCtx.fillStyle = fillStyle;
    overlayCtx.fillRect(r.x, r.y, r.w, r.h);
  }
  overlayCtx.strokeStyle = strokeStyle;
  overlayCtx.lineWidth = lw;
  const inset = 0.5 * dpr;
  const rw = Math.max(0, r.w - 1.0 * dpr);
  const rh = Math.max(0, r.h - 1.0 * dpr);
  overlayCtx.strokeRect(r.x + inset, r.y + inset, rw, rh);
  overlayCtx.restore();
}

function drawOverlay() {
  if (!overlayCtx || !overlayCanvas) return;
  overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);

  // Pinned first (under hover).
  if (pinnedTile) {
    drawTileBox(pinnedTile.tx, pinnedTile.ty, 2, 'rgba(0,0,0,0.95)', 'rgba(255,255,255,0.10)');
  }
  if (hoverTile) {
    drawTileBox(hoverTile.tx, hoverTile.ty, 1, 'rgba(0,0,0,0.65)', 'rgba(255,255,255,0.06)');
  }
}

function updateHashFromState() {
  try {
    const params = new URLSearchParams();
    if (layerSel && layerSel.value) params.set('layer', layerSel.value);
    if (pinnedTile) params.set('tile', pinnedTile.tx + ',' + pinnedTile.ty);
    const s = params.toString();
    if (s) window.location.hash = s;
  } catch (e) {
    // ignore (file:// quirks)
  }
}

function buildTileInfo(tx, ty) {
  const out = {
    text: '',
    json: { x: tx, y: ty, layer: (layerSel ? layerSel.value : ''), fields: {} }
  };

  if (!metrics) return out;

  const i = ty * MAP_W + tx;
  const selected = getSelectedMetrics();
  const parts = [];
  const fields = out.json.fields;

  // Always show a stable set of fields if present.
  for (const k of ALWAYS_INFO_FIELDS) {
    const arr = metrics.cols[k];
    if (!arr) continue;
    const val = arr[i];
    if (typeof val === 'string') {
      if (val !== '') {
        parts.push(k + '=' + val);
        fields[k] = val;
      }
    } else {
      const s = formatTileMetric(k, val);
      if (s != null) {
        parts.push(k + '=' + s);
        fields[k] = val;
      }
    }
  }

  // Selected numeric hover metrics.
  for (const k of selected) {
    const arr = metrics.cols[k];
    if (!arr) continue;
    const val = arr[i];
    if (typeof val === 'string') continue;
    const s = formatTileMetric(k, val);
    if (s == null) continue;
    parts.push(k + '=' + s);
    fields[k] = val;
  }

  out.text = parts.join(' • ');
  return out;
}

function setPinned(tx, ty, opts) {
  opts = opts || {};
  pinnedTile = { tx, ty };
  if (pinCoord) pinCoord.textContent = tx + ',' + ty;

  const info = buildTileInfo(tx, ty);
  pinnedJson = info.json;
  if (pinInfo) {
    if (!metrics) {
      pinInfo.textContent = ' (tile_metrics not loaded)';
    } else {
      pinInfo.textContent = info.text ? (' ' + info.text) : '';
    }
  }
  drawOverlay();
  if (opts.updateHash !== false) updateHashFromState();
}

function clearPinned(opts) {
  opts = opts || {};
  pinnedTile = null;
  pinnedJson = null;
  if (pinCoord) pinCoord.textContent = '-';
  if (pinInfo) pinInfo.textContent = '';
  drawOverlay();
  if (opts.updateHash !== false) updateHashFromState();
}

function tryCopyText(text) {
  if (!text) return;
  if (navigator && navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).catch(() => {});
    return;
  }
  const ta = document.createElement('textarea');
  ta.value = text;
  ta.style.position = 'fixed';
  ta.style.left = '-10000px';
  ta.style.top = '-10000px';
  document.body.appendChild(ta);
  ta.focus();
  ta.select();
  try { document.execCommand('copy'); } catch(e) {}
  document.body.removeChild(ta);
}

function applyHash() {
  try {
    const h = window.location.hash ? window.location.hash.substring(1) : '';
    if (!h) return;
    const params = new URLSearchParams(h);
    const layer = params.get('layer');
    if (layer && layerSel) {
      for (const opt of layerSel.options) {
        if (opt.value === layer) {
          layerSel.value = layer;
          mainImg.src = layer;
          break;
        }
      }
    }
    const t = params.get('tile');
    if (t) {
      const parts = t.split(',');
      if (parts.length === 2) {
        const tx = parseInt(parts[0], 10);
        const ty = parseInt(parts[1], 10);
        if (Number.isFinite(tx) && Number.isFinite(ty) && tx >= 0 && ty >= 0 && tx < MAP_W && ty < MAP_H) {
          setPinned(tx, ty, { updateHash: false });
        }
      }
    }
  } catch (e) {
    // ignore
  }
}

function updateHover(ev) {
  const t = eventToTile(ev);
  if (!t) return;

  hoverTile = t;
  if (tileCoord) tileCoord.textContent = t.tx + ',' + t.ty;

  if (!metrics) {
    if (tileInfo) tileInfo.textContent = '';
    drawOverlay();
    return;
  }

  const info = buildTileInfo(t.tx, t.ty);
  if (tileInfo) tileInfo.textContent = info.text ? (' ' + info.text) : '';
  drawOverlay();
}

mainImg.addEventListener('mousemove', updateHover);
mainImg.addEventListener('mouseleave', () => {
  hoverTile = null;
  if (tileCoord) tileCoord.textContent = '-';
  if (tileInfo) tileInfo.textContent = '';
  drawOverlay();
});

mainImg.addEventListener('click', (ev) => {
  const t = eventToTile(ev);
  if (!t) return;
  setPinned(t.tx, t.ty);
});

if (pinClear) pinClear.addEventListener('click', () => clearPinned());
if (pinCopy) pinCopy.addEventListener('click', () => {
  if (!pinnedJson) return;
  tryCopyText(JSON.stringify(pinnedJson, null, 2));
});

if (pinGoBtn && pinGoto) {
  pinGoBtn.addEventListener('click', () => {
    const s = String(pinGoto.value || '').trim();
    const m = s.split(',');
    if (m.length !== 2) return;
    const tx = parseInt(m[0], 10);
    const ty = parseInt(m[1], 10);
    if (!Number.isFinite(tx) || !Number.isFinite(ty)) return;
    if (tx < 0 || ty < 0 || tx >= MAP_W || ty >= MAP_H) return;
    setPinned(tx, ty);
  });

  pinGoto.addEventListener('keydown', (ev) => {
    if (ev.key === 'Enter') pinGoBtn.click();
  });
}

mainImg.addEventListener('load', () => resizeOverlayCanvas());
window.addEventListener('resize', () => resizeOverlayCanvas());

if (metricSel) metricSel.addEventListener('change', () => {
  if (pinnedTile) setPinned(pinnedTile.tx, pinnedTile.ty, { updateHash: false });
});

window.addEventListener('hashchange', () => applyHash());
applyHash();

// -----------------------------
// Ticks.csv charting
// -----------------------------
let ticks = null; // {header, cols, numericKeys, day}

const DEFAULT_TICK_METRICS = [
  'population', 'money', 'happiness', 'trafficCongestion', 'goodsSatisfaction',
  'avgLandValue', 'servicesOverallSatisfaction', 'airPollutionResidentAvg01'
];

function classifyNumericColumns(header, sampleRows) {
  const numeric = new Set();
  for (let ci = 0; ci < header.length; ++ci) {
    const name = header[ci];
    if (name === '') continue;

    let seen = 0;
    let ok = 0;
    for (const row of sampleRows) {
      if (ci >= row.length) continue;
      const s = row[ci];
      if (s == null) continue;
      const t = String(s).trim();
      if (t === '') continue;
      seen += 1;
      if (tryParseNumber(t) != null) ok += 1;
    }
    if (seen > 0 && (ok / seen) >= 0.80) numeric.add(name);
  }
  return numeric;
}

function parseTicksCsv(text) {
  const lines = splitNonEmptyLines(text);
  if (lines.length < 2) throw new Error('CSV has no data');

  const header = lines[0].split(',');
  const sampleRows = [];
  const sampleCount = Math.min(200, lines.length - 1);
  for (let li = 1; li <= sampleCount; ++li) sampleRows.push(lines[li].split(','));

  const numericSet = classifyNumericColumns(header, sampleRows);

  const rows = lines.length - 1;
  const cols = {};
  const numericKeys = [];

  for (const name of header) {
    if (name === '') continue;
    if (numericSet.has(name)) {
      cols[name] = new Array(rows).fill(NaN);
      numericKeys.push(name);
    }
  }

  const dayKey = header.includes('day') ? 'day' : (header.length > 0 ? header[0] : 'day');

  for (let li = 1; li < lines.length; ++li) {
    const parts = lines[li].split(',');
    const ri = li - 1;
    for (let ci = 0; ci < header.length; ++ci) {
      const name = header[ci];
      const arr = cols[name];
      if (!arr) continue;
      const s = (ci < parts.length) ? parts[ci] : '';
      const v = tryParseNumber(s);
      arr[ri] = (v == null) ? NaN : v;
    }
  }

  numericKeys.sort();

  return { header, cols, numericKeys, dayKey };
}

function setTicks(t) {
  ticks = t;
  if (ticksStatus) {
    ticksStatus.textContent = 'Loaded ticks.csv (' + (ticks.cols[ticks.dayKey]?.length || 0) + ' rows, ' + ticks.numericKeys.length + ' numeric cols)';
  }
  rebuildTickSelector();
  drawTickChart();
}

function rebuildTickSelector() {
  if (!tickMetric) return;
  tickMetric.innerHTML = '';
  if (!ticks) return;

  for (const k of ticks.numericKeys) {
    // Skip day key in the metric selector.
    if (k === ticks.dayKey) continue;
    const opt = document.createElement('option');
    opt.value = k;
    opt.textContent = k;
    tickMetric.appendChild(opt);
  }

  // Choose a default.
  const keys = new Set(ticks.numericKeys);
  let chosen = null;
  for (const k of DEFAULT_TICK_METRICS) {
    if (keys.has(k)) { chosen = k; break; }
  }
  if (!chosen && tickMetric.options.length > 0) chosen = tickMetric.options[0].value;
  if (chosen) tickMetric.value = chosen;
}

async function tryAutoLoadTicks() {
  if (!ticksStatus) return;
  try {
    const resp = await fetch('ticks.csv');
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const txt = await resp.text();
    setTicks(parseTicksCsv(txt));
  } catch (e) {
    ticksStatus.textContent = 'Not loaded. Use the file picker above to load ticks.csv.';
  }
}

tryAutoLoadTicks();

if (ticksFile) {
  ticksFile.addEventListener('change', () => {
    const file = ticksFile.files && ticksFile.files[0];
    if (!file) return;
    const r = new FileReader();
    r.onload = () => {
      try {
        setTicks(parseTicksCsv(String(r.result)));
      } catch (e) {
        if (ticksStatus) ticksStatus.textContent = 'Failed to parse: ' + e;
      }
    };
    r.readAsText(file);
  });
}



// -----------------------------
// Chronicle loading + render
// -----------------------------
let chronicle = null;

function toneCss(t) {
  if (t === 'good') return 'tone-good';
  if (t === 'bad') return 'tone-bad';
  if (t === 'alert') return 'tone-alert';
  return 'tone-neutral';
}

function clearNode(n) {
  while (n && n.firstChild) n.removeChild(n.firstChild);
}

function setChronicle(c) {
  chronicle = c;
  if (chronStatus) {
    const n = (chronicle && chronicle.entries) ? chronicle.entries.length : 0;
    chronStatus.textContent = 'Loaded chronicle.json (' + n + ' entries)';
  }

  if (chronTone) {
    const cur = chronTone.value || 'all';
    chronTone.innerHTML = '';
    const opts = ['all', 'good', 'neutral', 'bad', 'alert'];
    for (const t of opts) {
      const opt = document.createElement('option');
      opt.value = t;
      opt.textContent = (t === 'all') ? 'tone: all' : ('tone: ' + t);
      chronTone.appendChild(opt);
    }
    chronTone.value = cur;
  }

  renderChronicle();
}

function renderChronicle() {
  if (!chronList) return;
  clearNode(chronList);

  if (!chronicle || !Array.isArray(chronicle.entries)) {
    return;
  }

  const q = (chronFilter && chronFilter.value) ? chronFilter.value.trim().toLowerCase() : '';
  const tone = (chronTone && chronTone.value) ? chronTone.value : 'all';

  let lastDay = null;
  let shown = 0;

  for (const e of chronicle.entries) {
    const etone = (e && e.tone) ? String(e.tone) : 'neutral';
    if (tone !== 'all' && etone !== tone) continue;

    const tags = Array.isArray(e.tags) ? e.tags : [];
    const hay = (String(e.headline || '') + ' ' + String(e.body || '') + ' ' + tags.join(' ')).toLowerCase();
    if (q && hay.indexOf(q) === -1) continue;

    if (lastDay !== e.day) {
      lastDay = e.day;
      const h = document.createElement('div');
      h.style.marginTop = '10px';
      h.style.fontWeight = '700';
      h.textContent = 'Day ' + e.day;
      chronList.appendChild(h);
    }

    const wrap = document.createElement('div');
    wrap.className = 'chronEntry';

    const title = document.createElement('div');
    const toneSpan = document.createElement('span');
    toneSpan.className = 'tone ' + toneCss(etone);
    toneSpan.textContent = '[' + etone + '] ';
    title.appendChild(toneSpan);

    const hline = document.createElement('span');
    hline.style.fontWeight = '700';
    hline.textContent = String(e.headline || '');
    title.appendChild(hline);
    wrap.appendChild(title);

    if (tags && tags.length > 0) {
      const tagRow = document.createElement('div');
      tagRow.style.marginTop = '6px';
      for (const t of tags) {
        const pill = document.createElement('span');
        pill.className = 'pill';
        pill.textContent = String(t);
        tagRow.appendChild(pill);
      }
      wrap.appendChild(tagRow);
    }

    if (e.body) {
      const body = document.createElement('div');
      body.className = 'chronBody small';
      body.style.marginTop = '6px';
      body.textContent = String(e.body);
      wrap.appendChild(body);
    }

    if (e.tip) {
      const tip = document.createElement('div');
      tip.className = 'small';
      tip.style.marginTop = '6px';
      const b = document.createElement('b');
      b.textContent = 'Tip: ';
      tip.appendChild(b);
      const t = document.createElement('span');
      t.textContent = String(e.tip);
      tip.appendChild(t);
      wrap.appendChild(tip);
    }

    chronList.appendChild(wrap);
    shown++;
  }

  if (shown === 0) {
    const empty = document.createElement('div');
    empty.className = 'small';
    empty.textContent = 'No entries match the current filters.';
    chronList.appendChild(empty);
  }
}

async function tryAutoLoadChronicle() {
  if (!chronStatus) return;
  try {
    const resp = await fetch('chronicle.json');
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const txt = await resp.text();
    setChronicle(JSON.parse(txt));
  } catch (e) {
    chronStatus.textContent = 'Not loaded. Use the file picker above to load chronicle.json.';
  }
}

tryAutoLoadChronicle();

if (chronFile) {
  chronFile.addEventListener('change', () => {
    const file = chronFile.files && chronFile.files[0];
    if (!file) return;
    const r = new FileReader();
    r.onload = () => {
      try {
        setChronicle(JSON.parse(String(r.result)));
      } catch (e) {
        if (chronStatus) chronStatus.textContent = 'Failed to parse: ' + e;
      }
    };
    r.readAsText(file);
  });
}

if (chronFilter) chronFilter.addEventListener('input', renderChronicle);
if (chronTone) chronTone.addEventListener('change', renderChronicle);
function resizeCanvasToDisplaySize(canvas) {
  if (!canvas) return;
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const w = Math.max(10, Math.floor(rect.width * dpr));
  const h = Math.max(10, Math.floor(rect.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
}

function clearCanvas(canvas) {
  if (!canvas) return;
  resizeCanvasToDisplaySize(canvas);
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
}

function drawTickChart() {
  if (!tickChart || !tickMetric) return;
  resizeCanvasToDisplaySize(tickChart);
  const ctx = tickChart.getContext('2d');
  if (!ctx) return;

  ctx.clearRect(0, 0, tickChart.width, tickChart.height);

  if (!ticks) {
    if (tickHint) tickHint.textContent = '';
    return;
  }

  const key = tickMetric.value;
  const yArr = ticks.cols[key];
  if (!yArr || yArr.length === 0) return;

  const xArr = ticks.cols[ticks.dayKey] || new Array(yArr.length).fill(0).map((_, i) => i);

  let ymin = Infinity;
  let ymax = -Infinity;
  for (let i = 0; i < yArr.length; ++i) {
    const y = yArr[i];
    if (!Number.isFinite(y)) continue;
    ymin = Math.min(ymin, y);
    ymax = Math.max(ymax, y);
  }
  if (!Number.isFinite(ymin) || !Number.isFinite(ymax)) return;
  if (ymin === ymax) {
    ymin -= 1.0;
    ymax += 1.0;
  }

  const normalize = !!(tickNormalize && tickNormalize.checked);

  const padL = 46;
  const padR = 16;
  const padT = 14;
  const padB = 26;
  const W = tickChart.width;
  const H = tickChart.height;

  const plotW = Math.max(1, W - padL - padR);
  const plotH = Math.max(1, H - padT - padB);

  function xToPx(i) {
    const t = (yArr.length <= 1) ? 0.0 : (i / (yArr.length - 1));
    return padL + t * plotW;
  }

  function yToPx(y) {
    let yy = y;
    if (normalize) {
      yy = (y - ymin) / (ymax - ymin);
      yy = clamp(yy, 0.0, 1.0);
      return padT + (1.0 - yy) * plotH;
    }
    const t = (y - ymin) / (ymax - ymin);
    return padT + (1.0 - clamp(t, 0.0, 1.0)) * plotH;
  }

  // Axes
  ctx.strokeStyle = '#666';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(padL, padT);
  ctx.lineTo(padL, padT + plotH);
  ctx.lineTo(padL + plotW, padT + plotH);
  ctx.stroke();

  // Labels
  ctx.fillStyle = '#333';
  ctx.font = (12 * (window.devicePixelRatio || 1)) + 'px system-ui, sans-serif';
  const yLabelMin = normalize ? '0' : String(ymin.toFixed(2));
  const yLabelMax = normalize ? '1' : String(ymax.toFixed(2));
  ctx.fillText(yLabelMax, 6, padT + 10);
  ctx.fillText(yLabelMin, 6, padT + plotH);

  // Line
  ctx.strokeStyle = '#1976d2';
  ctx.lineWidth = 2;
  ctx.beginPath();
  let started = false;
  for (let i = 0; i < yArr.length; ++i) {
    const y = yArr[i];
    if (!Number.isFinite(y)) continue;
    const xpx = xToPx(i);
    const ypx = yToPx(y);
    if (!started) {
      ctx.moveTo(xpx, ypx);
      started = true;
    } else {
      ctx.lineTo(xpx, ypx);
    }
  }
  ctx.stroke();

  // Update hint text
  if (tickHint) {
    tickHint.textContent = key + (normalize ? ' (normalized)' : '') + ' — min ' + ymin.toFixed(3) + ', max ' + ymax.toFixed(3);
  }
}

if (tickMetric) tickMetric.addEventListener('change', drawTickChart);
if (tickNormalize) tickNormalize.addEventListener('change', drawTickChart);
window.addEventListener('resize', () => drawTickChart());

if (tickChart) {
  tickChart.addEventListener('mousemove', (ev) => {
    if (!ticks || !tickHint) return;
    const key = tickMetric ? tickMetric.value : '';
    const yArr = ticks.cols[key];
    const xArr = ticks.cols[ticks.dayKey] || null;
    if (!yArr || yArr.length === 0) return;

    const rect = tickChart.getBoundingClientRect();
    const u = (ev.clientX - rect.left) / rect.width;
    const idx = Math.round(clamp(u, 0.0, 1.0) * (yArr.length - 1));
    const day = xArr ? xArr[idx] : idx;
    const val = yArr[idx];
    if (!Number.isFinite(val)) return;
    tickHint.textContent = 'Day ' + day + ' • ' + key + '=' + val.toFixed(4);
  });

  tickChart.addEventListener('mouseleave', () => {
    // Restore summary.
    drawTickChart();
  });
}
)JS";

  f << "</script>\n";
  f << "</body>\n</html>\n";

  if (!f) {
    outErr = "Failed while writing HTML report";
    return false;
  }
  return true;
}

} // namespace

bool WriteCityDossier(World& world,
                      const ProcGenConfig& procCfg,
                      const SimConfig& simCfg,
                      const std::vector<Stats>& ticks,
                      const CityDossierConfig& cfg,
                      CityDossierResult* outRes,
                      std::string& outErr,
                      const CityDossierProgressFn& progress)
{
  outErr.clear();

  if (cfg.outDir.empty()) {
    outErr = "CityDossierConfig.outDir is empty";
    return false;
  }

  if (!EnsureDir(cfg.outDir)) {
    outErr = "Failed to create output directory";
    return false;
  }

  const std::string imageExt = cfg.format.empty() ? std::string("png") : cfg.format;
  const int exportScale = std::max(1, cfg.exportScale);

  // Progress/cancellation support.
  const bool wantProgress = static_cast<bool>(progress);

  int stepCount = 0;
  stepCount += 1; // refreshDerivedStats
  if (simCfg.requireOutsideConnection) stepCount += 1; // road-to-edge mask
  stepCount += 3; // traffic, goods, land value
  stepCount += 2; // sea flood + ponding
  if (cfg.writeTicksCsv) stepCount += 1;
  if (cfg.writeChronicleJson) stepCount += 1;
  if (cfg.writeChronicleMarkdown) stepCount += 1;
  if (cfg.writeTileMetricsCsv) stepCount += 1;
  stepCount += static_cast<int>(cfg.layers2d.size());
  if (cfg.exportIso) stepCount += static_cast<int>(cfg.layersIso.size());
  if (cfg.export3d) stepCount += 1;
  if (cfg.writeSummaryJson) stepCount += 1;
  if (cfg.writeWorldBinary) stepCount += 1;
  if (cfg.writeHtml) stepCount += 1;

  int stepIndex = 0;
  auto cancel = [&]() -> bool {
    outErr = "Cancelled";
    return false;
  };

  auto beginStage = [&](const std::string& stage) -> bool {
    ++stepIndex;
    if (!wantProgress) return true;
    CityDossierProgress p;
    p.stepIndex = stepIndex;
    p.stepCount = stepCount;
    p.stage = stage;
    return progress(p);
  };

  // Ensure derived stats are current.
  if (!beginStage("refresh_derived_stats")) return cancel();
  Simulator sim(simCfg);
  sim.refreshDerivedStats(world);

  // Derived overlays for exports.
  std::vector<std::uint8_t> roadToEdgeMask;
  const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (sim.config().requireOutsideConnection) {
    if (!beginStage("compute_roads_to_edge")) return cancel();
    ComputeRoadsConnectedToEdge(world, roadToEdgeMask);
    roadToEdgePtr = &roadToEdgeMask;
  }

  // Traffic: approximate transit mode shift by reducing the car commuter share.
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
  if (!beginStage("compute_traffic")) return cancel();
  const TrafficResult trafficRes = ComputeCommuteTraffic(world, tc, carShare, roadToEdgePtr);

  GoodsConfig gc{};
  gc.requireOutsideConnection = sim.config().requireOutsideConnection;
  gc.allowImports = true;
  gc.allowExports = true;
  gc.importCapacityPct = std::clamp(world.stats().tradeImportCapacityPct, 0, 100);
  gc.exportCapacityPct = std::clamp(world.stats().tradeExportCapacityPct, 0, 100);
  if (!beginStage("compute_goods")) return cancel();
  const GoodsResult goodsRes = ComputeGoodsFlow(world, gc, roadToEdgePtr);

  LandValueConfig lc{};
  lc.requireOutsideConnection = sim.config().requireOutsideConnection;
  if (!beginStage("compute_land_value")) return cancel();
  const LandValueResult landValueRes = ComputeLandValue(world, lc, &trafficRes, roadToEdgePtr);

  std::vector<float> heights;
  std::vector<std::uint8_t> drainMask;
  BuildHeightFieldAndDrainMask(world, heights, drainMask);
  const float seaLevel = InferCoastalSeaLevel(world);

  SeaFloodConfig sfc{};
  sfc.requireEdgeConnection = true;
  sfc.eightConnected = false;
  if (!beginStage("compute_sea_flood")) return cancel();
  const SeaFloodResult seaFlood = ComputeSeaLevelFlood(heights, world.width(), world.height(), seaLevel, sfc);

  DepressionFillConfig dfc{};
  dfc.includeEdges = true;
  dfc.epsilon = 0.0f;
  if (!beginStage("compute_ponding")) return cancel();
  const DepressionFillResult ponding =
      FillDepressionsPriorityFlood(heights, world.width(), world.height(), &drainMask, dfc);

  // ticks.csv
  if (cfg.writeTicksCsv) {
    if (!beginStage("write_ticks_csv")) return cancel();
    const std::filesystem::path csvPath = cfg.outDir / "ticks.csv";
    std::ofstream csv(csvPath, std::ios::binary);
    if (!csv) {
      outErr = "Failed to write: " + csvPath.string();
      return false;
    }
    if (!WriteCsvHeader(csv)) {
      outErr = "Failed while writing ticks.csv header";
      return false;
    }
    for (const Stats& s : ticks) {
      if (!WriteCsvRow(csv, s)) {
        outErr = "Failed while writing ticks.csv";
        return false;
      }
    }
  }

  // chronicle.json / chronicle.md
  Chronicle chronicle;
  bool chronicleReady = false;
  auto getChronicle = [&]() -> const Chronicle& {
    if (!chronicleReady) {
      chronicle = GenerateCityChronicle(world, ticks);
      chronicleReady = true;
    }
    return chronicle;
  };

  if (cfg.writeChronicleJson) {
    if (!beginStage("write_chronicle_json")) return cancel();
    const std::filesystem::path jsonPath = cfg.outDir / "chronicle.json";
    std::string err;
    if (!WriteCityChronicleJson(jsonPath.string(), getChronicle(), err)) {
      outErr = "Failed to write chronicle.json: " + err;
      return false;
    }
  }

  if (cfg.writeChronicleMarkdown) {
    if (!beginStage("write_chronicle_markdown")) return cancel();
    const std::filesystem::path mdPath = cfg.outDir / "chronicle.md";
    std::string err;
    if (!WriteCityChronicleMarkdown(mdPath.string(), getChronicle(), err)) {
      outErr = "Failed to write chronicle.md: " + err;
      return false;
    }
  }


  // tile_metrics.csv
  if (cfg.writeTileMetricsCsv) {
    if (!beginStage("write_tile_metrics_csv")) return cancel();

    // Derived soundscape/noise field.
    NoiseConfig nc{};
    const NoiseResult noiseRes = ComputeNoisePollution(world, nc, &trafficRes, &goodsRes);

    // Local land-use mix / diversity.
    LandUseMixConfig lmc{};
    const LandUseMixResult landUseMixRes = ComputeLandUseMix(world, lmc);

    // Heuristic urban heat island.
    HeatIslandConfig hic{};
    const HeatIslandResult heatIslandRes = ComputeHeatIsland(world, hic, &trafficRes, &goodsRes);

    // Heuristic transported air pollution (traffic + land use + wind advection/diffusion).
    AirPollutionConfig apc{};
    apc.windFromSeed = true;
    const AirPollutionResult airPollutionRes = ComputeAirPollution(world, apc, &trafficRes, &goodsRes);

    // Heuristic runoff / stormwater pollution (sources + downhill routing).
    RunoffPollutionConfig rpc{};
    const RunoffPollutionResult runoffRes = ComputeRunoffPollution(world, rpc, &trafficRes);

    // Hydrology-aware green infrastructure (park) placement suggestions.
    RunoffMitigationConfig rmc{};
    rmc.demandMode = RunoffMitigationDemandMode::ResidentialOccupants;
    rmc.parksToAdd = 12;
    rmc.minSeparation = 3;
    rmc.excludeWater = true;
    rmc.allowReplaceRoad = false;
    rmc.allowReplaceZones = false;
    // Use the same runoff settings to keep the plan consistent with exported runoff layers.
    rmc.runoffCfg = rpc;
    const RunoffMitigationResult runoffMitRes = SuggestRunoffMitigationParks(world, rmc, &trafficRes);

    // Solar exposure + rooftop PV potential (coarse horizon scan).
    SolarPotentialConfig spc{};
    spc.azimuthSamples = 16;
    spc.maxHorizonRadius = 64;
    const SolarPotentialResult solarRes = ComputeSolarPotential(world, spc);

    // Urban openness / canyon confinement (sky view factor; uses the same urban height field).
    SkyViewConfig svc{};
    svc.azimuthSamples = 16;
    svc.maxHorizonRadius = 64;
    svc.includeBuildings = true;
    const SkyViewResult skyViewRes = ComputeSkyViewFactor(world, svc);

    // Building energy demand vs rooftop solar (normalized proxy).
    EnergyModelConfig emc{};
    const EnergyModelResult energyRes = ComputeEnergyModel(world, emc, &solarRes, &heatIslandRes);
    const CarbonModelResult carbonRes = ComputeCarbonModel(world, CarbonModelConfig{}, &energyRes, &trafficRes, &goodsRes);


    // Transit accessibility (stop proximity) + localized mode-share potential.
    TransitAccessibilityConfig tac{};
    tac.requireOutsideConnection = sim.config().requireOutsideConnection;
    const TransitModelSettings& tm = sim.transitModel();
    tac.demandMode = tm.demandMode;
    tac.stopSpacingTiles = tm.stopSpacingTiles;
    // TransitModelSettings does not currently expose a walk radius.
    // Use a simple derived default: about half a stop spacing, clamped.
    tac.walkRadiusSteps = std::clamp(tm.stopSpacingTiles / 2, 6, 20);
    tac.serviceLevel = tm.serviceLevel;
    tac.maxModeShare = tm.maxModeShare;
    tac.travelTimeMultiplier = tm.travelTimeMultiplier;
    tac.plannerCfg = tm.plannerCfg;
    TransitAccessibilityInputs tai{};
    tai.traffic = &trafficRes;
    tai.goods = &goodsRes;
    tai.roadToEdgeMask = roadToEdgePtr;
    const TransitAccessibilityResult transitRes = ComputeTransitAccessibility(world, tac, tai);

    // SimCity-style fire risk (density + fire station response coverage).
    FireRiskConfig frc{};
    frc.requireOutsideConnection = true;
    frc.weightMode = IsochroneWeightMode::TravelTime;
    frc.responseRadiusSteps = 18;
    const FireRiskResult fireRiskRes = ComputeFireRisk(world, frc);

    // Walkability / 15-minute city amenity accessibility.
    WalkabilityConfig wc{};
    wc.enabled = true;
    wc.requireOutsideConnection = true;
    wc.weightMode = IsochroneWeightMode::TravelTime;
    wc.coverageThresholdSteps = 15;
    const WalkabilityResult walkabilityRes = ComputeWalkability(world, wc);

    // Job accessibility + opportunity (reachable jobs via the road graph).
    JobOpportunityConfig joc{};
    joc.requireOutsideConnection = sim.config().requireOutsideConnection;
    joc.useTravelTime = true;
    joc.congestionCosts = true;
    const JobOpportunityResult jobsRes =
        ComputeJobOpportunity(world, joc, &trafficRes, roadToEdgePtr, /*precomputedZoneAccess=*/nullptr);

    // Crime risk + police access proxy (uses jobs + noise + traffic/goods).
    CrimeModelConfig crc{};
    crc.requireOutsideConnection = sim.config().requireOutsideConnection;
    crc.weightMode = IsochroneWeightMode::TravelTime;
    const CrimeModelResult crimeRes =
        ComputeCrimeModel(world, crc, &trafficRes, &goodsRes, &jobsRes, &noiseRes, roadToEdgePtr, /*precomputedZoneAccess=*/nullptr);

    // Traffic collision risk proxy (traffic volume + intersection geometry + canyon confinement).
    TrafficSafetyConfig tsc{};
    tsc.requireOutsideConnection = sim.config().requireOutsideConnection;
    tsc.exposureRadius = 6;
    const TrafficSafetyResult trafficSafetyRes =
        ComputeTrafficSafety(world, tsc, &trafficRes, &skyViewRes, roadToEdgePtr);

    // Road network structural analytics (centrality + vulnerability + suggested bypasses).
    RoadHealthConfig rhc{};
    rhc.weightMode = RoadGraphEdgeWeightMode::TravelTimeMilli;
    rhc.maxSources = 0; // auto
    rhc.autoExactMaxNodes = 650;
    rhc.autoSampleSources = 256;
    rhc.includeNodeCentrality = true;
    rhc.articulationVulnerabilityBase = 0.70f;
    rhc.includeBypass = true;
    rhc.bypassCfg.top = 3;
    rhc.bypassCfg.moneyObjective = true;
    rhc.bypassCfg.targetLevel = 1;
    rhc.bypassCfg.allowBridges = false;
    rhc.bypassCfg.rankByTraffic = true;
    const RoadHealthResult roadHealthRes = ComputeRoadHealth(world, rhc, &trafficRes);

    // Composite livability index + intervention priority (services + walkability + environment).
    LivabilityConfig lvc{};
    lvc.requireOutsideConnection = true;
    lvc.weightMode = IsochroneWeightMode::TravelTime;
    lvc.servicesCatchmentRadiusSteps = 18;
    lvc.walkCoverageThresholdSteps = 15;
    const LivabilityResult livabilityRes = ComputeLivability(world, lvc, &trafficRes, &goodsRes);

    // Spatial hotspots (Getis-Ord Gi*) for clustering analysis.
    HotspotConfig hsc{};
    hsc.radius = 8;
    hsc.excludeWater = true;
    hsc.zThreshold = 1.96f;
    hsc.zScale = 3.0f;
    const HotspotResult livHotRes = ComputeHotspotsGiStar(world, livabilityRes.livability01, hsc);
    const HotspotResult priHotRes = ComputeHotspotsGiStar(world, livabilityRes.priority01, hsc);

    TileMetricsCsvInputs inputs;
    inputs.landValue = &landValueRes;
    inputs.traffic = &trafficRes;
    inputs.goods = &goodsRes;
    inputs.noise = &noiseRes;
    inputs.landUseMix = &landUseMixRes;
    inputs.heatIsland = &heatIslandRes;
    inputs.airPollution = &airPollutionRes;
    inputs.runoff = &runoffRes;
    inputs.runoffMitigation = &runoffMitRes;
    inputs.solar = &solarRes;
    inputs.skyView = &skyViewRes;
    inputs.energy = &energyRes;
    inputs.carbon = &carbonRes;
    inputs.crime = &crimeRes;
    inputs.trafficSafety = &trafficSafetyRes;
    inputs.transit = &transitRes;
    inputs.fireRisk = &fireRiskRes;
    inputs.walkability = &walkabilityRes;
    inputs.jobs = &jobsRes;
    inputs.roadHealth = &roadHealthRes;
    inputs.livability = &livabilityRes;
    inputs.livabilityHotspot = &livHotRes;
    inputs.interventionHotspot = &priHotRes;
    inputs.seaFlood = &seaFlood;
    inputs.ponding = &ponding;

    TileMetricsCsvOptions opt;
    opt.includeLandValue = true;
    opt.includeLandValueComponents = true;
    opt.includeTraffic = true;
    opt.includeGoods = true;
    opt.includeNoise = true;
    opt.includeLandUseMix = true;
    opt.includeHeatIsland = true;
    opt.includeAirPollution = true;
    opt.includeRunoffPollution = true;
    opt.includeRunoffMitigation = true;
    opt.includeSolar = true;
    opt.includeSkyView = true;
    opt.includeEnergy = true;
    opt.includeCarbon = true;
    opt.includeCrime = true;
    opt.includeTrafficSafety = true;
    opt.includeTransit = true;
    opt.includeFireRisk = true;
    opt.includeWalkability = true;
    opt.includeJobs = true;
    opt.includeWalkabilityComponents = true;
    opt.includeWalkabilityDistances = false;
    opt.includeRoadHealth = true;
    opt.includeLivability = true;
    opt.includeHotspots = true;
    opt.includeFlood = true;
    opt.includePonding = true;
    opt.floatPrecision = 6;

    std::string err;
    const std::filesystem::path csvPath = cfg.outDir / "tile_metrics.csv";
    if (!WriteTileMetricsCsv(world, csvPath.string(), err, inputs, opt)) {
      outErr = "Failed to write tile_metrics.csv: " + err;
      return false;
    }
  }

  // Export top-down images.
  for (ExportLayer layer : cfg.layers2d) {
    if (!beginStage(std::string("render_map_") + ExportLayerName(layer))) return cancel();
    const std::filesystem::path outP = cfg.outDir / (std::string("map_") + ExportLayerName(layer) + "." + imageExt);

    PpmImage img = RenderPpmLayer(world, layer, &landValueRes, &trafficRes, &goodsRes);
    if (exportScale > 1) img = ScaleNearest(img, exportScale);

    std::string err;
    if (!WriteImageAuto(outP.string(), img, err)) {
      outErr = "Failed to write image (" + std::string(ExportLayerName(layer)) + "): " + outP.string() + " (" + err + ")";
      return false;
    }
  }

  // Iso images.
  if (cfg.exportIso) {
    IsoOverviewConfig isoCfg{};
    isoCfg.tileW = 16;
    isoCfg.tileH = 8;
    isoCfg.heightScalePx = 14;
    isoCfg.marginPx = 2;
    isoCfg.drawGrid = false;
    isoCfg.drawCliffs = true;
    isoCfg.fancy = true;
    isoCfg.textureStrength = 0.15f;
    isoCfg.drawShore = true;
    isoCfg.drawRoadMarkings = true;
    isoCfg.drawZonePatterns = true;
    isoCfg.dayNight.enabled = false;

    for (ExportLayer layer : cfg.layersIso) {
      if (!beginStage(std::string("render_iso_") + ExportLayerName(layer))) return cancel();
      const std::filesystem::path outP = cfg.outDir / (std::string("iso_") + ExportLayerName(layer) + "." + imageExt);

      const IsoOverviewResult iso = RenderIsoOverview(world, layer, isoCfg, &landValueRes, &trafficRes, &goodsRes, nullptr);
      if (iso.image.width <= 0 || iso.image.height <= 0) {
        outErr = "Failed to render iso layer: " + std::string(ExportLayerName(layer));
        return false;
      }

      std::string err;
      if (!WriteImageAuto(outP.string(), iso.image, err)) {
        outErr = "Failed to write iso image (" + std::string(ExportLayerName(layer)) + "): " + outP.string() + " (" + err + ")";
        return false;
      }
    }
  }

  // Optional 3D render.
  if (cfg.export3d) {
    if (!beginStage("render_3d")) return cancel();
    PpmImage img3d = RenderWorld3D(world, ExportLayer::Overlay, cfg.render3dCfg, &landValueRes, &trafficRes, &goodsRes);
    if (img3d.width <= 0 || img3d.height <= 0) {
      outErr = "Failed to render 3D view";
      return false;
    }

    const std::filesystem::path outP = cfg.outDir / (std::string("view3d_overlay.") + imageExt);
    std::string err;
    if (!WriteImageAuto(outP.string(), img3d, err)) {
      outErr = "Failed to write 3D image: " + outP.string() + " (" + err + ")";
      return false;
    }
  }

  // summary.json + world.bin.
  const std::uint64_t hash = HashWorld(world, true);

  if (cfg.writeSummaryJson) {
    if (!beginStage("write_summary_json")) return cancel();
    std::string err;
    const std::filesystem::path outJson = cfg.outDir / "summary.json";
    const std::vector<ExportLayer> isoLayers = cfg.exportIso ? cfg.layersIso : std::vector<ExportLayer>{};
    if (!WriteSummaryJson(outJson, world, hash, procCfg, sim.config(), ticks, cfg.layers2d, isoLayers,
                          cfg.export3d, imageExt, exportScale, err)) {
      outErr = "Failed to write summary.json: " + err;
      return false;
    }
  }

  if (cfg.writeWorldBinary) {
    if (!beginStage("write_world_bin")) return cancel();
    std::string err;
    const std::filesystem::path outSave = cfg.outDir / "world.bin";
    if (!SaveWorldBinary(world, procCfg, sim.config(), outSave.string(), err)) {
      outErr = "Failed to write world.bin: " + err;
      return false;
    }
  }

  if (cfg.writeHtml) {
    if (!beginStage("write_index_html")) return cancel();
    std::string err;
    const std::filesystem::path outHtml = cfg.outDir / "index.html";
    const std::vector<ExportLayer> isoLayers = cfg.exportIso ? cfg.layersIso : std::vector<ExportLayer>{};
    if (!WriteHtmlReport(outHtml, world, hash, procCfg, sim.config(), ticks, cfg.layers2d, isoLayers,
                         cfg.export3d, imageExt, exportScale, err)) {
      outErr = "Failed to write index.html: " + err;
      return false;
    }
  }

  if (outRes) {
    outRes->outDir = cfg.outDir;
    outRes->hash = hash;
  }

  return true;
}

} // namespace isocity
