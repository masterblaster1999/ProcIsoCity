#include "isocity/Dossier.hpp"

#include "isocity/ConfigIO.hpp"
#include "isocity/DepressionFill.hpp"
#include "isocity/FloodRisk.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/SaveLoad.hpp"
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

  // tile_metrics.csv
  if (cfg.writeTileMetricsCsv) {
    if (!beginStage("write_tile_metrics_csv")) return cancel();
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
