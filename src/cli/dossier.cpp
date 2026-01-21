#include "isocity/AutoBuild.hpp"
#include "isocity/ConfigIO.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/Export.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  int base = 10;
  std::size_t offset = 0;
  if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
    base = 16;
    offset = 2;
  }

  char* end = nullptr;
  errno = 0;
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0) return false;
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  if (v < -1.0e9 || v > 1.0e9) return false;
  *out = static_cast<float>(v);
  return true;
}

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string::npos) return false;
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, pos), &w)) return false;
  if (!ParseI32(s.substr(pos + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

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

std::vector<std::string> SplitCommaList(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) out.push_back(cur);
  // trim whitespace
  auto trim = [](std::string& t) {
    auto isWs = [](unsigned char ch) { return std::isspace(ch) != 0; };
    while (!t.empty() && isWs(static_cast<unsigned char>(t.front()))) t.erase(t.begin());
    while (!t.empty() && isWs(static_cast<unsigned char>(t.back()))) t.pop_back();
  };
  for (auto& e : out) trim(e);
  out.erase(std::remove_if(out.begin(), out.end(), [](const std::string& v) { return v.empty(); }), out.end());
  return out;
}

bool ParseLayerList(const std::string& s, std::vector<ExportLayer>* outLayers, std::string& outErr)
{
  if (!outLayers) return false;
  outLayers->clear();

  for (const std::string& item : SplitCommaList(s)) {
    ExportLayer layer{};
    if (!ParseExportLayer(item, layer)) {
      outErr = "Unknown layer: " + item;
      return false;
    }
    outLayers->push_back(layer);
  }
  if (outLayers->empty()) {
    outErr = "No layers specified";
    return false;
  }
  return true;
}

// Replicates the engine's internal inference used by the FloodDepth export layer.
// We infer sea level from edge-connected water tiles (avoids inland lakes).
float InferCoastalSeaLevel(const World& world)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) return 0.35f;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<std::uint8_t> visited(n, std::uint8_t{0});
  std::vector<std::size_t> stack;
  stack.reserve(static_cast<std::size_t>(w + h) * 2u);

  auto idx = [&](int x, int y) -> std::size_t {
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

  while (!stack.empty()) {
    const std::size_t i = stack.back();
    stack.pop_back();

    const int x = static_cast<int>(i % static_cast<std::size_t>(w));
    const int y = static_cast<int>(i / static_cast<std::size_t>(w));
    seaLevel = std::max(seaLevel, world.at(x, y).height);

    pushIfOcean(x - 1, y);
    pushIfOcean(x + 1, y);
    pushIfOcean(x, y - 1);
    pushIfOcean(x, y + 1);
  }

  if (anyEdgeWater) return seaLevel;

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
  os << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
  return static_cast<bool>(os);
}

bool WriteCsvRow(std::ostream& os, const Stats& s)
{
  os << s.day << ','
     << s.population << ','
     << s.money << ','
     << s.housingCapacity << ','
     << s.jobsCapacity << ','
     << s.jobsCapacityAccessible << ','
     << s.employed << ','
     << s.happiness << ','
     << s.roads << ','
     << s.parks << ','
     << s.avgCommuteTime << ','
     << s.trafficCongestion << ','
     << s.goodsDemand << ','
     << s.goodsDelivered << ','
     << s.goodsSatisfaction << ','
     << s.avgLandValue << ','
     << s.demandResidential
     << '\n';
  return static_cast<bool>(os);
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
  f << ".thumbs{display:grid; grid-template-columns:repeat(auto-fill, minmax(180px,1fr)); gap:12px;}\n";
  f << ".thumbs a{text-decoration:none; color:inherit;}\n";
  f << ".thumbs img{width:100%; height:auto; image-rendering:pixelated; border:1px solid #eee; border-radius:8px;}\n";
  f << ".small{font-size:13px; color:#444;}\n";
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
  f << "      <img id=\"mainImg\" alt=\"layer\" src=\"" << layerDefault << "\">\n";
  f << "    </div>\n";
  f << "    <div class=\"small\" style=\"margin-top:8px\">\n";
  f << "      Tile: <code id=\"tileCoord\">-</code>";
  f << "      <span id=\"tileInfo\"></span>\n";
  f << "    </div>\n";
  f << "    <div class=\"small\" style=\"margin-top:10px\">\n";
  f << "      Optional: load <code>tile_metrics.csv</code> for per-tile inspection (works even when opened as file://).\n";
  f << "      <div style=\"margin-top:6px\"><input type=\"file\" id=\"metricsFile\" accept=\".csv\"></div>\n";
  f << "      <div id=\"metricsStatus\"></div>\n";
  f << "    </div>\n";
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
const metricsFile = document.getElementById('metricsFile');
const metricsStatus = document.getElementById('metricsStatus');

for (const l of LAYERS_2D) {
  const opt = document.createElement('option');
  opt.value = l.file;
  opt.textContent = l.key;
  layerSel.appendChild(opt);
}

layerSel.addEventListener('change', () => {
  mainImg.src = layerSel.value;
});

// --- Tile metrics ---
let metrics = null; // {terrain,overlay,level,district,height,occupants,land_value,commute_traffic,goods_fill,flood_depth,ponding_depth}

function parseCsv(text) {
  const lines = text.split(/\r?\n/).filter(l => l.length > 0);
  if (lines.length < 2) throw new Error('CSV has no data');
  const header = lines[0].split(',');
  const idx = (name) => header.indexOf(name);
  const ix = idx('x');
  const iy = idx('y');
  if (ix < 0 || iy < 0) throw new Error('CSV missing x/y');

  const take = {
    terrain: idx('terrain'),
    overlay: idx('overlay'),
    level: idx('level'),
    district: idx('district'),
    height: idx('height'),
    occupants: idx('occupants'),
    land_value: idx('land_value'),
    commute_traffic: idx('commute_traffic'),
    goods_fill: idx('goods_fill'),
    flood_depth: idx('flood_depth'),
    ponding_depth: idx('ponding_depth'),
  };

  const n = MAP_W * MAP_H;
  const out = {
    terrain: new Array(n).fill(''),
    overlay: new Array(n).fill(''),
    level: new Int16Array(n),
    district: new Int16Array(n),
    height: new Float32Array(n),
    occupants: new Int32Array(n),
    land_value: new Float32Array(n),
    commute_traffic: new Int32Array(n),
    goods_fill: new Int32Array(n),
    flood_depth: new Float32Array(n),
    ponding_depth: new Float32Array(n),
  };

  // Initialize optional numeric channels to NaN so we can detect missing.
  out.land_value.fill(NaN);
  out.flood_depth.fill(NaN);
  out.ponding_depth.fill(NaN);

  for (let li = 1; li < lines.length; ++li) {
    const parts = lines[li].split(',');
    if (parts.length < 2) continue;
    const x = parseInt(parts[ix], 10);
    const y = parseInt(parts[iy], 10);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    if (x < 0 || y < 0 || x >= MAP_W || y >= MAP_H) continue;
    const i = y * MAP_W + x;

    if (take.terrain >= 0) out.terrain[i] = parts[take.terrain];
    if (take.overlay >= 0) out.overlay[i] = parts[take.overlay];
    if (take.level >= 0) out.level[i] = parseInt(parts[take.level], 10) || 0;
    if (take.district >= 0) out.district[i] = parseInt(parts[take.district], 10) || 0;
    if (take.height >= 0) out.height[i] = parseFloat(parts[take.height]) || 0;
    if (take.occupants >= 0) out.occupants[i] = parseInt(parts[take.occupants], 10) || 0;

    if (take.land_value >= 0) out.land_value[i] = parseFloat(parts[take.land_value]);
    if (take.commute_traffic >= 0) out.commute_traffic[i] = parseInt(parts[take.commute_traffic], 10) || 0;
    if (take.goods_fill >= 0) out.goods_fill[i] = parseInt(parts[take.goods_fill], 10) || 0;
    if (take.flood_depth >= 0) out.flood_depth[i] = parseFloat(parts[take.flood_depth]);
    if (take.ponding_depth >= 0) out.ponding_depth[i] = parseFloat(parts[take.ponding_depth]);
  }

  return out;
}

function setMetrics(m) {
  metrics = m;
  metricsStatus.textContent = 'Loaded tile_metrics.csv (' + (MAP_W*MAP_H) + ' tiles)';
}

async function tryAutoLoadMetrics() {
  try {
    const resp = await fetch('tile_metrics.csv');
    if (!resp.ok) throw new Error('HTTP ' + resp.status);
    const txt = await resp.text();
    setMetrics(parseCsv(txt));
  } catch (e) {
    metricsStatus.textContent = 'Not loaded. Use the file picker above to load tile_metrics.csv.';
  }
}

tryAutoLoadMetrics();

metricsFile.addEventListener('change', () => {
  const file = metricsFile.files && metricsFile.files[0];
  if (!file) return;
  const r = new FileReader();
  r.onload = () => {
    try {
      setMetrics(parseCsv(String(r.result)));
    } catch (e) {
      metricsStatus.textContent = 'Failed to parse: ' + e;
    }
  };
  r.readAsText(file);
});

function updateHover(ev) {
  const rect = mainImg.getBoundingClientRect();
  const u = (ev.clientX - rect.left) / rect.width;
  const v = (ev.clientY - rect.top) / rect.height;
  if (!Number.isFinite(u) || !Number.isFinite(v)) return;
  if (u < 0 || v < 0 || u > 1 || v > 1) return;

  const px = Math.floor(u * mainImg.naturalWidth);
  const py = Math.floor(v * mainImg.naturalHeight);
  const tx = Math.floor(px / EXPORT_SCALE);
  const ty = Math.floor(py / EXPORT_SCALE);
  if (tx < 0 || ty < 0 || tx >= MAP_W || ty >= MAP_H) return;

  tileCoord.textContent = tx + ',' + ty;

  if (metrics) {
    const i = ty * MAP_W + tx;
    const lv = metrics.land_value[i];
    const fd = metrics.flood_depth[i];
    const pd = metrics.ponding_depth[i];

    const parts = [];
    parts.push(' terrain=' + metrics.terrain[i]);
    parts.push(' overlay=' + metrics.overlay[i]);
    parts.push(' lvl=' + metrics.level[i]);
    parts.push(' dist=' + metrics.district[i]);
    parts.push(' h=' + metrics.height[i].toFixed(3));
    parts.push(' occ=' + metrics.occupants[i]);

    if (!Number.isNaN(lv)) parts.push(' lv=' + lv.toFixed(3));
    parts.push(' commute=' + metrics.commute_traffic[i]);
    parts.push(' goodsFill=' + metrics.goods_fill[i]);
    if (!Number.isNaN(fd)) parts.push(' flood=' + fd.toFixed(3));
    if (!Number.isNaN(pd)) parts.push(' pond=' + pd.toFixed(3));

    tileInfo.textContent = parts.join('');
  } else {
    tileInfo.textContent = '';
  }
}

mainImg.addEventListener('mousemove', updateHover);
mainImg.addEventListener('mouseleave', () => { tileCoord.textContent='-'; tileInfo.textContent=''; });
)JS";

  f << "</script>\n";
  f << "</body>\n</html>\n";

  if (!f) {
    outErr = "Failed while writing HTML report";
    return false;
  }
  return true;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_dossier (one-command city dossier exporter)\n\n"
      << "Generates (or loads) a world, optionally runs the simulator and/or AutoBuild,\n"
      << "then exports a full dossier: top-down layers, optional isometric and 3D renders,\n"
      << "tile_metrics.csv, ticks.csv, summary.json, and an index.html viewer.\n\n"
      << "Usage:\n"
      << "  proc_isocity_dossier --out-dir <dir> [options]\n\n"
      << "Inputs:\n"
      << "  --load <save.bin>              Load an existing save (overrides --seed/--size).\n"
      << "  --seed <u64>                   Seed for generation (default 1).\n"
      << "  --size <WxH>                   World size (default 256x256).\n\n"
      << "ProcGen (when generating):\n"
      << "  --gen-preset <name>            Terrain preset (classic/island/archipelago/...).\n"
      << "  --gen-preset-strength <N>      Preset strength (default 1).\n"
      << "  --gen-road-layout <name>       Road layout (organic/grid/radial/space_colonization).\n"
      << "  --gen-districting <name>       Districting (voronoi/road_flow/block_graph).\n"
      << "  --gen-hubs <N>                 Hub count (default 4).\n"
      << "  --gen-water-level <0..1>       Water threshold (default 0.35).\n\n"
      << "Simulation:\n"
      << "  --require-outside <0|1>        Require road-to-edge for zones (default 1).\n"
      << "  --autobuild-days <N>           Run AutoBuild for N days (default 0).\n"
      << "  --days <N>                     Simulate N additional days (default 0).\n\n"
      << "Export:\n"
      << "  --format <png|ppm>             Image format (default png).\n"
      << "  --scale <N>                    Nearest-neighbor scale for top-down layers (default 2).\n"
      << "  --layers <a,b,c>               Top-down layers to export (default: terrain,overlay,height,landvalue,traffic,goods_traffic,goods_fill,district,flood_depth,ponding_depth).\n"
      << "  --iso <0|1>                    Enable isometric exports (default 1).\n"
      << "  --iso-layers <a,b,c>           Iso layers (default: overlay,landvalue).\n"
      << "  --3d <0|1>                     Enable a 3D overlay render (default 0).\n\n";
}

} // namespace

int main(int argc, char** argv)
{
  std::string loadPath;
  std::filesystem::path outDir;

  std::uint64_t seed = 1;
  int w = 256;
  int h = 256;

  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::optional<bool> requireOutsideOverride;

  int autobuildDays = 0;
  int simDays = 0;

  std::string format = "png";
  int exportScale = 2;

  std::vector<ExportLayer> layers2d = {
      ExportLayer::Terrain,
      ExportLayer::Overlay,
      ExportLayer::Height,
      ExportLayer::LandValue,
      ExportLayer::Traffic,
      ExportLayer::GoodsTraffic,
      ExportLayer::GoodsFill,
      ExportLayer::District,
      ExportLayer::FloodDepth,
      ExportLayer::PondingDepth,
  };

  bool exportIso = true;
  std::vector<ExportLayer> layersIso = {
      ExportLayer::Overlay,
      ExportLayer::LandValue,
  };

  bool export3d = false;

  // Extra ProcGen tuning (optional).
  bool procExplicitPreset = false;
  bool procExplicitPresetStrength = false;
  bool procExplicitRoadLayout = false;
  bool procExplicitDistricting = false;
  bool procExplicitHubs = false;
  bool procExplicitWater = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        std::exit(1);
      }
      return std::string(argv[++i]);
    };

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    }
    if (arg == "--load") {
      loadPath = need("--load");
      continue;
    }
    if (arg == "--out-dir") {
      outDir = need("--out-dir");
      continue;
    }
    if (arg == "--seed") {
      const std::string v = need("--seed");
      if (!ParseU64(v, &seed)) {
        std::cerr << "Invalid seed: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--size") {
      const std::string v = need("--size");
      if (!ParseWxH(v, &w, &h)) {
        std::cerr << "Invalid size: " << v << " (expected WxH)\n";
        return 1;
      }
      continue;
    }
    if (arg == "--require-outside") {
      const std::string v = need("--require-outside");
      bool b = true;
      if (!ParseBool01(v, &b)) {
        std::cerr << "Invalid bool: " << v << "\n";
        return 1;
      }
      requireOutsideOverride = b;
      continue;
    }
    if (arg == "--autobuild-days") {
      const std::string v = need("--autobuild-days");
      if (!ParseI32(v, &autobuildDays) || autobuildDays < 0) {
        std::cerr << "Invalid autobuild days: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--days") {
      const std::string v = need("--days");
      if (!ParseI32(v, &simDays) || simDays < 0) {
        std::cerr << "Invalid days: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--format") {
      format = need("--format");
      if (format != "png" && format != "ppm") {
        std::cerr << "Unsupported format: " << format << " (expected png|ppm)\n";
        return 1;
      }
      continue;
    }
    if (arg == "--scale") {
      const std::string v = need("--scale");
      if (!ParseI32(v, &exportScale) || exportScale < 1 || exportScale > 32) {
        std::cerr << "Invalid scale: " << v << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--layers") {
      const std::string v = need("--layers");
      std::string err;
      if (!ParseLayerList(v, &layers2d, err)) {
        std::cerr << "Invalid layers: " << err << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--iso") {
      const std::string v = need("--iso");
      bool b = true;
      if (!ParseBool01(v, &b)) {
        std::cerr << "Invalid bool: " << v << "\n";
        return 1;
      }
      exportIso = b;
      continue;
    }
    if (arg == "--iso-layers") {
      const std::string v = need("--iso-layers");
      std::string err;
      if (!ParseLayerList(v, &layersIso, err)) {
        std::cerr << "Invalid iso layers: " << err << "\n";
        return 1;
      }
      continue;
    }
    if (arg == "--3d") {
      const std::string v = need("--3d");
      bool b = false;
      if (!ParseBool01(v, &b)) {
        std::cerr << "Invalid bool: " << v << "\n";
        return 1;
      }
      export3d = b;
      continue;
    }

    // ProcGen tuning.
    if (arg == "--gen-preset") {
      const std::string v = need("--gen-preset");
      ProcGenTerrainPreset p{};
      if (!ParseProcGenTerrainPreset(v, p)) {
        std::cerr << "Invalid terrain preset: " << v << "\n";
        return 1;
      }
      procCfg.terrainPreset = p;
      procExplicitPreset = true;
      continue;
    }
    if (arg == "--gen-preset-strength") {
      const std::string v = need("--gen-preset-strength");
      float f = 1.0f;
      if (!ParseF32(v, &f) || f < 0.0f || f > 10.0f) {
        std::cerr << "Invalid preset strength: " << v << "\n";
        return 1;
      }
      procCfg.terrainPresetStrength = f;
      procExplicitPresetStrength = true;
      continue;
    }
    if (arg == "--gen-road-layout") {
      const std::string v = need("--gen-road-layout");
      ProcGenRoadLayout r{};
      if (!ParseProcGenRoadLayout(v, r)) {
        std::cerr << "Invalid road layout: " << v << "\n";
        return 1;
      }
      procCfg.roadLayout = r;
      procExplicitRoadLayout = true;
      continue;
    }
    if (arg == "--gen-districting") {
      const std::string v = need("--gen-districting");
      ProcGenDistrictingMode m{};
      if (!ParseProcGenDistrictingMode(v, m)) {
        std::cerr << "Invalid districting mode: " << v << "\n";
        return 1;
      }
      procCfg.districtingMode = m;
      procExplicitDistricting = true;
      continue;
    }
    if (arg == "--gen-hubs") {
      const std::string v = need("--gen-hubs");
      int iv = 0;
      if (!ParseI32(v, &iv) || iv < 1 || iv > 64) {
        std::cerr << "Invalid hubs: " << v << "\n";
        return 1;
      }
      procCfg.hubs = iv;
      procExplicitHubs = true;
      continue;
    }
    if (arg == "--gen-water-level") {
      const std::string v = need("--gen-water-level");
      float f = 0.0f;
      if (!ParseF32(v, &f) || f < 0.0f || f > 1.0f) {
        std::cerr << "Invalid water level: " << v << "\n";
        return 1;
      }
      procCfg.waterLevel = f;
      procExplicitWater = true;
      continue;
    }

    std::cerr << "Unknown argument: " << arg << " (try --help)\n";
    return 1;
  }

  if (outDir.empty()) {
    std::cerr << "Missing required --out-dir\n";
    return 1;
  }
  if (!EnsureDir(outDir)) {
    std::cerr << "Failed to create output dir: " << outDir.string() << "\n";
    return 1;
  }

  // Load or generate.
  World world;
  {
    std::string err;
    if (!loadPath.empty()) {
      if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
        std::cerr << "Failed to load save: " << loadPath << " (" << err << ")\n";
        return 1;
      }
      // If the user explicitly specified requireOutsideConnection, override the save.
      if (requireOutsideOverride.has_value()) {
        simCfg.requireOutsideConnection = *requireOutsideOverride;
      }
    } else {
      // Use defaults unless explicitly changed.
      // (This is mainly to avoid surprising changes if defaults evolve.)
      ProcGenConfig cfg = procCfg;
      if (!procExplicitPreset) cfg.terrainPreset = ProcGenConfig{}.terrainPreset;
      if (!procExplicitPresetStrength) cfg.terrainPresetStrength = ProcGenConfig{}.terrainPresetStrength;
      if (!procExplicitRoadLayout) cfg.roadLayout = ProcGenConfig{}.roadLayout;
      if (!procExplicitDistricting) cfg.districtingMode = ProcGenConfig{}.districtingMode;
      if (!procExplicitHubs) cfg.hubs = ProcGenConfig{}.hubs;
      if (!procExplicitWater) cfg.waterLevel = ProcGenConfig{}.waterLevel;
      procCfg = cfg;

      world = GenerateWorld(w, h, seed, procCfg);
      // Keep the world's seed stable (GenerateWorld already uses it but also stores it).
    }
  }

  // Apply runtime SimConfig overrides (regardless of load vs generate).
  if (requireOutsideOverride.has_value()) {
    simCfg.requireOutsideConnection = *requireOutsideOverride;
  }

  // Simulation / AutoBuild.
  Simulator sim(simCfg);
  sim.refreshDerivedStats(world);

  std::vector<Stats> ticks;
  ticks.push_back(world.stats());

  if (autobuildDays > 0) {
    AutoBuildConfig acfg{};
    acfg.respectOutsideConnection = true;
    acfg.ensureOutsideConnection = true;
    RunAutoBuild(world, sim, acfg, autobuildDays, &ticks);
  }
  for (int d = 0; d < simDays; ++d) {
    sim.stepOnce(world);
    ticks.push_back(world.stats());
  }

  // Derived overlays for exports.
  std::vector<std::uint8_t> roadToEdgeMask;
  const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
  if (sim.config().requireOutsideConnection) {
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
  const TrafficResult trafficRes = ComputeCommuteTraffic(world, tc, carShare, roadToEdgePtr);

  GoodsConfig gc{};
  gc.requireOutsideConnection = sim.config().requireOutsideConnection;
  gc.allowImports = true;
  gc.allowExports = true;
  gc.importCapacityPct = std::clamp(world.stats().tradeImportCapacityPct, 0, 100);
  gc.exportCapacityPct = std::clamp(world.stats().tradeExportCapacityPct, 0, 100);
  const GoodsResult goodsRes = ComputeGoodsFlow(world, gc, roadToEdgePtr);

  LandValueConfig lc{};
  lc.requireOutsideConnection = sim.config().requireOutsideConnection;
  const LandValueResult landValueRes = ComputeLandValue(world, lc, &trafficRes, roadToEdgePtr);

  std::vector<float> heights;
  std::vector<std::uint8_t> drainMask;
  BuildHeightFieldAndDrainMask(world, heights, drainMask);
  const float seaLevel = InferCoastalSeaLevel(world);
  SeaFloodConfig sfc{};
  sfc.requireEdgeConnection = true;
  sfc.eightConnected = false;
  const SeaFloodResult seaFlood = ComputeSeaLevelFlood(heights, world.width(), world.height(), seaLevel, sfc);

  DepressionFillConfig dfc{};
  dfc.includeEdges = true;
  dfc.epsilon = 0.0f;
  const DepressionFillResult ponding = FillDepressionsPriorityFlood(heights, world.width(), world.height(), &drainMask, dfc);

  const std::string imageExt = format;

  // Write ticks.csv
  {
    const std::filesystem::path csvPath = outDir / "ticks.csv";
    std::ofstream csv(csvPath, std::ios::binary);
    if (!csv) {
      std::cerr << "Failed to write: " << csvPath.string() << "\n";
      return 1;
    }
    WriteCsvHeader(csv);
    for (const Stats& s : ticks) WriteCsvRow(csv, s);
  }

  // Write tile_metrics.csv
  {
    TileMetricsCsvInputs inputs;
    inputs.landValue = &landValueRes;
    inputs.traffic = &trafficRes;
    inputs.goods = &goodsRes;
    inputs.seaFlood = &seaFlood;
    inputs.ponding = &ponding;
    TileMetricsCsvOptions opt;
    opt.includeLandValue = true;
    opt.includeLandValueComponents = true;
    opt.includeTraffic = true;
    opt.includeGoods = true;
    opt.includeFlood = true;
    opt.includePonding = true;
    opt.floatPrecision = 6;

    std::string err;
    const std::filesystem::path csvPath = outDir / "tile_metrics.csv";
    if (!WriteTileMetricsCsv(world, csvPath.string(), err, inputs, opt)) {
      std::cerr << "Failed to write tile_metrics.csv: " << err << "\n";
      return 1;
    }
  }

  // Export images.
  for (ExportLayer layer : layers2d) {
    const std::filesystem::path outP = outDir / (std::string("map_") + ExportLayerName(layer) + "." + imageExt);

    PpmImage img = RenderPpmLayer(world, layer, &landValueRes, &trafficRes, &goodsRes);
    if (exportScale > 1) img = ScaleNearest(img, exportScale);

    std::string err;
    if (!WriteImageAuto(outP.string(), img, err)) {
      std::cerr << "Failed to write image (" << ExportLayerName(layer) << "): " << outP.string() << " (" << err << ")\n";
      return 1;
    }
  }

  if (exportIso) {
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
    // Keep the day/night pass disabled by default to preserve a "clean" baseline render.
    isoCfg.dayNight.enabled = false;

    for (ExportLayer layer : layersIso) {
      const std::filesystem::path outP = outDir / (std::string("iso_") + ExportLayerName(layer) + "." + imageExt);

      const IsoOverviewResult iso = RenderIsoOverview(world, layer, isoCfg, &landValueRes, &trafficRes, &goodsRes, nullptr);
      if (iso.image.width <= 0 || iso.image.height <= 0) {
        std::cerr << "Failed to render iso layer: " << ExportLayerName(layer) << "\n";
        return 1;
      }

      std::string err;
      if (!WriteImageAuto(outP.string(), iso.image, err)) {
        std::cerr << "Failed to write iso image (" << ExportLayerName(layer) << "): " << outP.string() << " (" << err << ")\n";
        return 1;
      }
    }
  }

  if (export3d) {
    Render3DConfig cfg{};
    cfg.width = 960;
    cfg.height = 540;
    cfg.projection = Render3DConfig::Projection::IsometricOrtho;
    cfg.autoFit = true;
    cfg.supersample = 2;
    cfg.drawOutlines = true;
    cfg.heightfieldTopSurfaces = true;
    cfg.addSkirt = true;
    cfg.skirtDrop = 6.0f;
    cfg.meshCfg.heightScale = 12.0f;
    cfg.meshCfg.includeBuildings = true;
    cfg.meshCfg.includeCliffs = true;

    // Subtle post passes for nicer screenshots.
    cfg.postAO = true;
    cfg.postEdge = true;
    cfg.postTonemap = true;
    cfg.postDither = true;
    cfg.postSeed = 1337u;

    PpmImage img3d = RenderWorld3D(world, ExportLayer::Overlay, cfg, &landValueRes, &trafficRes, &goodsRes);
    if (img3d.width <= 0 || img3d.height <= 0) {
      std::cerr << "Failed to render 3D view\n";
      return 1;
    }

    const std::filesystem::path outP = outDir / (std::string("view3d_overlay.") + imageExt);
    std::string err;
    if (!WriteImageAuto(outP.string(), img3d, err)) {
      std::cerr << "Failed to write 3D image: " << outP.string() << " (" << err << ")\n";
      return 1;
    }
  }

  // Write summary.json + world.bin.
  const std::uint64_t hash = HashWorld(world, true);
  {
    std::string err;
    const std::filesystem::path outJson = outDir / "summary.json";
    if (!WriteSummaryJson(outJson, world, hash, procCfg, sim.config(), ticks, layers2d, (exportIso ? layersIso : std::vector<ExportLayer>{}),
                          export3d, imageExt, exportScale, err)) {
      std::cerr << "Failed to write summary.json: " << err << "\n";
      return 1;
    }
  }

  {
    std::string err;
    const std::filesystem::path outSave = outDir / "world.bin";
    if (!SaveWorldBinary(world, procCfg, sim.config(), outSave.string(), err)) {
      std::cerr << "Failed to write world.bin: " << err << "\n";
      return 1;
    }
  }

  // Write HTML report.
  {
    std::string err;
    const std::filesystem::path outHtml = outDir / "index.html";
    if (!WriteHtmlReport(outHtml, world, hash, procCfg, sim.config(), ticks, layers2d,
                         (exportIso ? layersIso : std::vector<ExportLayer>{}),
                         export3d, imageExt, exportScale, err)) {
      std::cerr << "Failed to write index.html: " << err << "\n";
      return 1;
    }
  }

  std::cout << "Wrote dossier to: " << outDir.string() << "\n";
  std::cout << "  index.html\n";
  std::cout << "  summary.json\n";
  std::cout << "  tile_metrics.csv\n";
  std::cout << "  ticks.csv\n";
  return 0;
}
