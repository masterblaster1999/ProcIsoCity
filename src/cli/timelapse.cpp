#include "isocity/ConfigIO.hpp"
#include "isocity/Export.hpp"
#include "isocity/Goods.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"
#include "isocity/StatsCsv.hpp"
#include "isocity/Traffic.hpp"
#include "isocity/ZoneAccess.hpp"

#include "cli/CliParse.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

bool ParseI32(const std::string& s, int* out)
{
  return isocity::cli::ParseI32(s, out);
}

bool ParseU64(const std::string& s, std::uint64_t* out)
{
  return isocity::cli::ParseU64(s, out);
}

bool ParseF32(const std::string& s, float* out)
{
  return isocity::cli::ParseF32(s, out);
}

bool ParseBool01(const std::string& s, bool* out)
{
  return isocity::cli::ParseBool01(s, out);
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  return isocity::cli::ParseWxH(s, outW, outH);
}

bool EnsureDir(const std::string& dir)
{
  return isocity::cli::EnsureDir(std::filesystem::path(dir));
}

static std::string ToLowerAscii(std::string s)
{
  for (char& c : s) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return s;
}

static std::string TrimAscii(const std::string& s)
{
  std::size_t a = 0;
  std::size_t b = s.size();
  while (a < b) {
    const char c = s[a];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    ++a;
  }
  while (b > a) {
    const char c = s[b - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    --b;
  }
  return s.substr(a, b - a);
}

bool ParseWeatherMode(const std::string& s, IsoOverviewConfig::WeatherConfig::Mode& outMode)
{
  const std::string v = ToLowerAscii(s);
  if (v == "clear") {
    outMode = IsoOverviewConfig::WeatherConfig::Mode::Clear;
    return true;
  }
  if (v == "rain") {
    outMode = IsoOverviewConfig::WeatherConfig::Mode::Rain;
    return true;
  }
  if (v == "snow") {
    outMode = IsoOverviewConfig::WeatherConfig::Mode::Snow;
    return true;
  }
  return false;
}

std::vector<std::string> SplitComma(const std::string& s)
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
  return out;
}

bool ParseLayers(const std::string& s, std::vector<ExportLayer>& outLayers, std::string& outErr)
{
  outErr.clear();
  outLayers.clear();

  const std::vector<std::string> parts = SplitComma(s);
  if (parts.empty()) {
    outErr = "empty layer list";
    return false;
  }

  for (const std::string& pRaw : parts) {
    const std::string pTrim = TrimAscii(pRaw);
    if (pTrim.empty()) continue;
    const std::string p = ToLowerAscii(pTrim);
    ExportLayer layer;
    if (!ParseExportLayer(p, layer)) {
      outErr = std::string("unknown layer: ") + pRaw;
      return false;
    }
    outLayers.push_back(layer);
  }

  if (outLayers.empty()) {
    outErr = "empty layer list";
    return false;
  }

  return true;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_timelapse (headless timelapse frame exporter)\n\n"
      << "Generates an image sequence by running the simulator forward and exporting isometric\n"
      << "renders at a fixed interval. This is useful for CI artifacts, regression visuals,\n"
      << "and quick scenario previews.\n\n"
      << "Usage:\n"
      << "  proc_isocity_timelapse --load <save.bin> --out <dir> [options]\n"
      << "  proc_isocity_timelapse --size <WxH> --seed <u64> --out <dir> [--config <cfg.json>] [options]\n\n"
      << "Core options:\n"
      << "  --out <dir>                 Output directory (created if needed).\n"
      << "  --prefix <name>             Filename prefix (default: frame).\n"
      << "  --format <png|ppm>          Output image format (default: png).\n"
      << "  --layers <a,b,c>            Comma-separated layers (default: overlay).\n"
      << "                             Valid: terrain overlay height landvalue traffic goods_traffic goods_fill district\n"
      << "  --days <N>                  Sim days to advance (default: 120).\n"
      << "  --every <N>                 Export every N days (default: 1).\n"
      << "  --no-initial                Do not export the initial frame.\n"
      << "  --scale <N>                 Nearest-neighbor upscaling factor (default: 1).\n"
      << "  --csv <path>                Write per-frame stats CSV (default: <out>/stats.csv).\n\n"
      << "World source:\n"
      << "  --load <save.bin>           Load an existing save (includes ProcGen+Sim config when available).\n"
      << "  --size <WxH>                Generate a new world.\n"
      << "  --seed <u64>                Seed for generation (decimal or 0x...).\n"
      << "  --config <cfg.json>         Optional combined config JSON: {\"proc\":{...},\"sim\":{...}}\n\n"
      << "Iso render tuning (applies to all exported layers):\n"
      << "  --tileW <px>                Iso tile width (default: 16).\n"
      << "  --tileH <px>                Iso tile height (default: 8).\n"
      << "  --heightScale <px>          Elevation scale in pixels (default: 14).\n"
      << "  --margin <px>               Border around bounds (default: 8).\n"
      << "  --fancy <0|1>               Fancy terrain/overlay rendering (default: 1).\n"
      << "  --grid <0|1>                Draw tile outlines (default: 0).\n"
      << "  --cliffs <0|1>              Draw simple cliff faces (default: 1).\n"
      << "  --texture <k>               Micro texture strength (default: 1).\n\n"
      << "Atmosphere (Terrain/Overlay only):\n"
      << "  --daynight <0|1>            Enable day/night grade (default: 0).\n"
      << "  --phase <0..1>              Day/night phase (0=sunrise, 0.25=noon, 0.5=sunset, 0.75=midnight).\n"
      << "  --lights <0|1>              Emissive lights at night (default: 1).\n"
      << "  --nightDarken <0..1>        Night darkening strength (default: 0.8).\n"
      << "  --duskTint <0..1>           Warm dawn/dusk tint strength (default: 0.55).\n"
      << "  --weather <clear|rain|snow> Weather mode (default: clear).\n"
      << "  --wxIntensity <0..1>        Rain wetness / snow cover (default: 0).\n"
      << "  --wxOvercast <0..1>         Overcast grade strength (default: 0.60).\n"
      << "  --wxFog <0..1>              Fog gradient strength (default: 0).\n"
      << "  --precip <0|1>              Draw precipitation overlay (default: 1).\n"
      << "  --reflect <0|1>             Wet reflections (default: 1).\n"
      << "  --clouds <0|1>              Enable cloud shadows (default: 0).\n"
      << "  --cloudCoverage <0..1>      Fraction of sky covered by clouds (default: 0.5).\n"
      << "  --cloudStrength <0..1>      Shadow strength (default: 0.55).\n"
      << "  --cloudScale <tiles>        Cloud feature size in tiles (default: 16).\n\n";
}

bool WriteStatsHeader(std::ostream& os)
{
  os << "frame," << kStatsCsvHeader << "\n";
  return static_cast<bool>(os);
}

bool WriteStatsRow(std::ostream& os, int frameIdx, const Stats& s)
{
  os << frameIdx << ',';
  return WriteStatsCsvRow(os, s);
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  int genW = 0;
  int genH = 0;
  std::uint64_t genSeed = 0;
  bool haveGen = false;

  std::string combinedCfgPath;

  std::string outDir;
  std::string prefix = "frame";
  std::string format = "png";
  int scale = 1;

  int days = 120;
  int every = 1;
  bool exportInitial = true;

  std::string csvPath;

  std::vector<ExportLayer> layers{ExportLayer::Overlay};
  bool layersExplicit = false;

  IsoOverviewConfig isoCfg{};

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i] ? std::string(argv[i]) : std::string();
    if (a == "-h" || a == "--help") {
      PrintHelp();
      return 0;
    }

    auto requireValue = [&](const char* opt, std::string& out) -> bool {
      if (i + 1 >= argc) {
        std::cerr << opt << " requires a value\n";
        return false;
      }
      out = argv[++i];
      return true;
    };

    if (a == "--load") {
      if (!requireValue("--load", loadPath)) return 2;
      continue;
    }
    if (a == "--size") {
      std::string v;
      if (!requireValue("--size", v)) return 2;
      if (!ParseWxH(v, &genW, &genH)) {
        std::cerr << "invalid --size (expected WxH)\n";
        return 2;
      }
      haveGen = true;
      continue;
    }
    if (a == "--seed") {
      std::string v;
      if (!requireValue("--seed", v)) return 2;
      if (!ParseU64(v, &genSeed)) {
        std::cerr << "invalid --seed\n";
        return 2;
      }
      haveGen = true;
      continue;
    }
    if (a == "--config") {
      if (!requireValue("--config", combinedCfgPath)) return 2;
      continue;
    }

    if (a == "--out" || a == "--out-dir") {
      if (!requireValue("--out", outDir)) return 2;
      continue;
    }
    if (a == "--prefix") {
      if (!requireValue("--prefix", prefix)) return 2;
      continue;
    }
    if (a == "--format") {
      if (!requireValue("--format", format)) return 2;
      format = ToLowerAscii(format);
      if (format != "png" && format != "ppm") {
        std::cerr << "--format must be png or ppm\n";
        return 2;
      }
      continue;
    }
    if (a == "--scale") {
      std::string v;
      if (!requireValue("--scale", v)) return 2;
      if (!ParseI32(v, &scale) || scale < 1) {
        std::cerr << "--scale expects integer >= 1\n";
        return 2;
      }
      continue;
    }

    if (a == "--days") {
      std::string v;
      if (!requireValue("--days", v)) return 2;
      if (!ParseI32(v, &days) || days < 0) {
        std::cerr << "--days expects integer >= 0\n";
        return 2;
      }
      continue;
    }
    if (a == "--every") {
      std::string v;
      if (!requireValue("--every", v)) return 2;
      if (!ParseI32(v, &every) || every < 1) {
        std::cerr << "--every expects integer >= 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--no-initial") {
      exportInitial = false;
      continue;
    }

    if (a == "--csv") {
      if (!requireValue("--csv", csvPath)) return 2;
      continue;
    }

    if (a == "--layers" || a == "--layer") {
      std::string v;
      if (!requireValue(a.c_str(), v)) return 2;
      std::vector<ExportLayer> parsed;
      std::string err;
      if (!ParseLayers(v, parsed, err)) {
        std::cerr << "invalid --layers: " << err << "\n";
        return 2;
      }
      if (!layersExplicit) {
        layers.clear();
        layersExplicit = true;
      }
      layers.insert(layers.end(), parsed.begin(), parsed.end());
      continue;
    }

    // --- Iso tuning ---
    if (a == "--tileW") {
      std::string v;
      if (!requireValue("--tileW", v)) return 2;
      if (!ParseI32(v, &isoCfg.tileW) || isoCfg.tileW < 2) {
        std::cerr << "--tileW expects integer >= 2\n";
        return 2;
      }
      continue;
    }
    if (a == "--tileH") {
      std::string v;
      if (!requireValue("--tileH", v)) return 2;
      if (!ParseI32(v, &isoCfg.tileH) || isoCfg.tileH < 2) {
        std::cerr << "--tileH expects integer >= 2\n";
        return 2;
      }
      continue;
    }
    if (a == "--heightScale") {
      std::string v;
      if (!requireValue("--heightScale", v)) return 2;
      if (!ParseI32(v, &isoCfg.heightScalePx) || isoCfg.heightScalePx < 0) {
        std::cerr << "--heightScale expects integer >= 0\n";
        return 2;
      }
      continue;
    }
    if (a == "--margin") {
      std::string v;
      if (!requireValue("--margin", v)) return 2;
      if (!ParseI32(v, &isoCfg.marginPx) || isoCfg.marginPx < 0) {
        std::cerr << "--margin expects integer >= 0\n";
        return 2;
      }
      continue;
    }
    if (a == "--fancy") {
      std::string v;
      if (!requireValue("--fancy", v)) return 2;
      if (!ParseBool01(v, &isoCfg.fancy)) {
        std::cerr << "--fancy expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--grid") {
      std::string v;
      if (!requireValue("--grid", v)) return 2;
      if (!ParseBool01(v, &isoCfg.drawGrid)) {
        std::cerr << "--grid expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--cliffs") {
      std::string v;
      if (!requireValue("--cliffs", v)) return 2;
      if (!ParseBool01(v, &isoCfg.drawCliffs)) {
        std::cerr << "--cliffs expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--texture") {
      std::string v;
      if (!requireValue("--texture", v)) return 2;
      if (!ParseF32(v, &isoCfg.textureStrength)) {
        std::cerr << "--texture expects a float\n";
        return 2;
      }
      continue;
    }

    // --- Atmosphere ---
    if (a == "--daynight") {
      std::string v;
      if (!requireValue("--daynight", v)) return 2;
      if (!ParseBool01(v, &isoCfg.dayNight.enabled)) {
        std::cerr << "--daynight expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--phase") {
      std::string v;
      if (!requireValue("--phase", v)) return 2;
      if (!ParseF32(v, &isoCfg.dayNight.phase01)) {
        std::cerr << "--phase expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--lights") {
      std::string v;
      if (!requireValue("--lights", v)) return 2;
      if (!ParseBool01(v, &isoCfg.dayNight.drawLights)) {
        std::cerr << "--lights expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--nightDarken") {
      std::string v;
      if (!requireValue("--nightDarken", v)) return 2;
      if (!ParseF32(v, &isoCfg.dayNight.nightDarken)) {
        std::cerr << "--nightDarken expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--duskTint") {
      std::string v;
      if (!requireValue("--duskTint", v)) return 2;
      if (!ParseF32(v, &isoCfg.dayNight.duskTint)) {
        std::cerr << "--duskTint expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--weather") {
      std::string v;
      if (!requireValue("--weather", v)) return 2;
      IsoOverviewConfig::WeatherConfig::Mode m;
      if (!ParseWeatherMode(v, m)) {
        std::cerr << "--weather must be clear, rain, or snow\n";
        return 2;
      }
      isoCfg.weather.mode = m;
      continue;
    }
    if (a == "--wxIntensity") {
      std::string v;
      if (!requireValue("--wxIntensity", v)) return 2;
      if (!ParseF32(v, &isoCfg.weather.intensity)) {
        std::cerr << "--wxIntensity expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--wxOvercast") {
      std::string v;
      if (!requireValue("--wxOvercast", v)) return 2;
      if (!ParseF32(v, &isoCfg.weather.overcast)) {
        std::cerr << "--wxOvercast expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--wxFog") {
      std::string v;
      if (!requireValue("--wxFog", v)) return 2;
      if (!ParseF32(v, &isoCfg.weather.fog)) {
        std::cerr << "--wxFog expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--precip") {
      std::string v;
      if (!requireValue("--precip", v)) return 2;
      if (!ParseBool01(v, &isoCfg.weather.drawPrecipitation)) {
        std::cerr << "--precip expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--reflect") {
      std::string v;
      if (!requireValue("--reflect", v)) return 2;
      if (!ParseBool01(v, &isoCfg.weather.reflectLights)) {
        std::cerr << "--reflect expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--clouds") {
      std::string v;
      if (!requireValue("--clouds", v)) return 2;
      if (!ParseBool01(v, &isoCfg.clouds.enabled)) {
        std::cerr << "--clouds expects 0 or 1\n";
        return 2;
      }
      continue;
    }
    if (a == "--cloudCoverage") {
      std::string v;
      if (!requireValue("--cloudCoverage", v)) return 2;
      if (!ParseF32(v, &isoCfg.clouds.coverage)) {
        std::cerr << "--cloudCoverage expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--cloudStrength") {
      std::string v;
      if (!requireValue("--cloudStrength", v)) return 2;
      if (!ParseF32(v, &isoCfg.clouds.strength)) {
        std::cerr << "--cloudStrength expects a float\n";
        return 2;
      }
      continue;
    }
    if (a == "--cloudScale") {
      std::string v;
      if (!requireValue("--cloudScale", v)) return 2;
      if (!ParseF32(v, &isoCfg.clouds.scaleTiles)) {
        std::cerr << "--cloudScale expects a float\n";
        return 2;
      }
      continue;
    }

    std::cerr << "unknown option: " << a << "\n";
    return 2;
  }

  if (outDir.empty()) {
    PrintHelp();
    std::cerr << "missing --out\n";
    return 2;
  }

  if (!loadPath.empty() && haveGen) {
    std::cerr << "use either --load OR (--size + --seed), not both\n";
    return 2;
  }

  if (loadPath.empty()) {
    if (!haveGen || genW <= 0 || genH <= 0) {
      PrintHelp();
      std::cerr << "missing world source: use --load or (--size + --seed)\n";
      return 2;
    }
  }

  ProcGenConfig procCfg;
  SimConfig simCfg;
  World world;

  CombinedConfig combinedCfg;
  bool haveCombinedCfg = false;

  std::string err;
  if (!combinedCfgPath.empty()) {
    if (!LoadCombinedConfigJsonFile(combinedCfgPath, combinedCfg, err)) {
      std::cerr << "failed to load config: " << err << "\n";
      return 1;
    }
    haveCombinedCfg = true;
  }

  if (!loadPath.empty()) {
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "failed to load save: " << err << "\n";
      return 1;
    }

    // Optional override: allow the caller to run a timelapse using a different policy/sim config
    // than the one stored in the save.
    if (haveCombinedCfg) {
      if (combinedCfg.hasProc) procCfg = combinedCfg.proc;
      if (combinedCfg.hasSim) simCfg = combinedCfg.sim;
    }
  } else {
    if (haveCombinedCfg) {
      if (combinedCfg.hasProc) procCfg = combinedCfg.proc;
      if (combinedCfg.hasSim) simCfg = combinedCfg.sim;
    }
    world = GenerateWorld(genW, genH, genSeed, procCfg);
  }

  Simulator sim(simCfg);
  sim.refreshDerivedStats(world);

  if (!EnsureDir(outDir)) {
    std::cerr << "failed to create output directory: " << outDir << "\n";
    return 1;
  }

  if (csvPath.empty()) {
    csvPath = (std::filesystem::path(outDir) / "stats.csv").string();
  }

  {
    // Ensure the CSV parent dir exists.
    try {
      const std::filesystem::path p(csvPath);
      const std::filesystem::path parent = p.parent_path();
      if (!parent.empty()) std::filesystem::create_directories(parent);
    } catch (...) {
      // ignore; open below will fail and print an error.
    }
  }

  std::ofstream csv(csvPath, std::ios::binary);
  if (!csv) {
    std::cerr << "failed to open CSV for writing: " << csvPath << "\n";
    return 1;
  }
  csv << std::fixed << std::setprecision(6);
  if (!WriteStatsHeader(csv)) {
    std::cerr << "failed to write CSV header\n";
    return 1;
  }

  auto exportFrame = [&](int frameIdx) -> bool {
    const int w = world.width();
    const int h = world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w)) * static_cast<std::size_t>(std::max(0, h));

    std::vector<std::uint8_t> roadToEdge;
    const std::vector<std::uint8_t>* roadMask = nullptr;
    if (simCfg.requireOutsideConnection) {
      roadToEdge.resize(n);
      ComputeRoadsConnectedToEdge(world, roadToEdge);
      roadMask = &roadToEdge;
    }

    ZoneAccessMap zoneAccess = BuildZoneAccessMap(world, roadMask);

    bool needTraffic = false;
    bool needGoods = false;
    bool needLand = false;

    for (ExportLayer l : layers) {
      if (l == ExportLayer::Traffic) needTraffic = true;
      if (l == ExportLayer::GoodsTraffic || l == ExportLayer::GoodsFill) needGoods = true;
      if (l == ExportLayer::LandValue) needLand = true;
    }

    TrafficResult traffic;
    GoodsResult goods;
    LandValueResult land;

    if (needTraffic) {
      TrafficConfig tcfg;
      tcfg.requireOutsideConnection = simCfg.requireOutsideConnection;

      const Stats& s = world.stats();
      const float employedShare = (s.population > 0) ? std::clamp(static_cast<float>(s.employed) / static_cast<float>(s.population), 0.0f, 1.0f) : 1.0f;

      traffic = ComputeCommuteTraffic(world, tcfg, employedShare, roadMask, &zoneAccess);
    }

    if (needGoods) {
      GoodsConfig gcfg;
      gcfg.requireOutsideConnection = simCfg.requireOutsideConnection;
      goods = ComputeGoodsFlow(world, gcfg, roadMask, &zoneAccess, nullptr);
    }

    if (needLand) {
      LandValueConfig lcfg;
      lcfg.requireOutsideConnection = simCfg.requireOutsideConnection;
      const TrafficResult* lTraffic = needTraffic ? &traffic : nullptr;
      land = ComputeLandValue(world, lcfg, lTraffic, roadMask);
    }

    // Write the stats row once per frame.
    if (!WriteStatsRow(csv, frameIdx, world.stats())) {
      std::cerr << "failed to write stats row\n";
      return false;
    }

    // Export all requested layers.
    for (ExportLayer l : layers) {
      const LandValueResult* lvPtr = (l == ExportLayer::LandValue) ? &land : nullptr;
      const TrafficResult* trPtr = (l == ExportLayer::Traffic) ? &traffic : nullptr;
      const GoodsResult* gdPtr = (l == ExportLayer::GoodsTraffic || l == ExportLayer::GoodsFill) ? &goods : nullptr;

      const IsoOverviewResult iso = RenderIsoOverview(world, l, isoCfg, lvPtr, trPtr, gdPtr);
      if (iso.image.width <= 0 || iso.image.height <= 0) {
        std::cerr << "iso render produced empty image for layer " << ExportLayerName(l) << "\n";
        return false;
      }

      const PpmImage img = (scale > 1) ? ScaleNearest(iso.image, scale) : iso.image;

      std::ostringstream name;
      name << prefix << '_' << ExportLayerName(l) << '_' << std::setw(4) << std::setfill('0') << frameIdx;
      if (format == "png") name << ".png";
      else name << ".ppm";

      const std::filesystem::path outPath = std::filesystem::path(outDir) / name.str();

      std::string werr;
      if (!WriteImageAuto(outPath.string(), img, werr)) {
        std::cerr << "failed to write image: " << outPath.string() << " : " << werr << "\n";
        return false;
      }
    }

    return true;
  };

  int frameIdx = 0;
  if (exportInitial) {
    if (!exportFrame(frameIdx++)) return 1;
  }

  for (int d = 0; d < days; ++d) {
    sim.stepOnce(world);
    if (((d + 1) % every) == 0) {
      if (!exportFrame(frameIdx++)) return 1;
    }
  }

  return 0;
}
