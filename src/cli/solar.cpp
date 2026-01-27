#include "isocity/SaveLoad.hpp"
#include "isocity/SolarPotential.hpp"
#include "isocity/Json.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace isocity;

void PrintHelp()
{
  std::cout
      << "proc_isocity_solar (headless solar exposure / rooftop PV potential report)\n\n"
      << "Usage:\n"
      << "  proc_isocity_solar <save.bin> [options]\n\n"
      << "Options:\n"
      << "  --json <out.json>          Write a JSON report.\n"
      << "  --azimuth-samples <N>      Use 8 or 16 azimuth directions (default: 16).\n"
      << "  --max-radius <N>           Horizon scan radius in tiles (default: 64).\n"
      << "  --altitudes <a,b,c>        Altitude samples in degrees (default: 15,30,45,60).\n"
      << "  --no-buildings             Ignore building heights for shading.\n"
      << "  --sun-azimuth <deg>        Use single-sample mode with this azimuth (0=E,90=N,180=W,270=S).\n"
      << "  --sun-altitude <deg>       Use single-sample mode with this altitude above horizon.\n"
      << "  --verify-crc               Verify CRC for v3+ saves (slower, but detects corruption).\n"
      << "  --quiet                    Suppress stdout summary (errors still print).\n"
      << "  -h, --help                 Show this help.\n";
}

bool ParseInt(const std::string& s, int& out)
{
  try {
    std::size_t pos = 0;
    const int v = std::stoi(s, &pos, 10);
    if (pos != s.size()) return false;
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseFloat(const std::string& s, float& out)
{
  try {
    std::size_t pos = 0;
    const float v = std::stof(s, &pos);
    if (pos != s.size()) return false;
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseFloatListCsv(const std::string& s, std::vector<float>& outList)
{
  outList.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    float v = 0.0f;
    if (!ParseFloat(item, v)) return false;
    outList.push_back(v);
  }
  return !outList.empty();
}

static void PrintSummary(const SolarPotentialResult& r)
{
  auto pct = [](float v01) -> double { return static_cast<double>(v01) * 100.0; };

  std::cout << "Solar potential summary\n";
  std::cout << "- grid: " << r.w << "x" << r.h << "\n";
  std::cout << "- roof tiles: " << r.roofTileCount << "\n";
  std::cout << "- resident population (residential occupants): " << r.residentPopulation << "\n";
  std::cout << "- max exposure: " << pct(r.maxExposure01) << "%\n";
  std::cout << "- max potential: " << pct(r.maxPotential01) << "%\n";
  std::cout << "- per-capita potential: " << static_cast<double>(r.perCapitaPotential) << " (proxy units)\n";
  std::cout << "- roof tiles >= " << pct(r.highPotentialThreshold) << "% potential: "
            << pct(r.roofHighPotentialFrac) << "%\n";
}

static bool WriteReportJson(const std::string& outPath, const std::string& inPath,
                            const SaveSummary* sum, const SolarPotentialResult& r)
{
  JsonValue root = JsonValue::MakeObject();

  auto add = [](JsonValue& obj, const char* key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

  add(root, "file", JsonValue::MakeString(inPath));
  add(root, "width", JsonValue::MakeNumber(static_cast<double>(r.w)));
  add(root, "height", JsonValue::MakeNumber(static_cast<double>(r.h)));

  if (sum) {
    add(root, "version", JsonValue::MakeNumber(static_cast<double>(sum->version)));
    add(root, "seed", JsonValue::MakeNumber(static_cast<double>(sum->seed)));
  }

  // Config.
  {
    JsonValue cfg = JsonValue::MakeObject();
    add(cfg, "maxHorizonRadius", JsonValue::MakeNumber(static_cast<double>(r.cfg.maxHorizonRadius)));
    add(cfg, "azimuthSamples", JsonValue::MakeNumber(static_cast<double>(r.cfg.azimuthSamples)));
    add(cfg, "includeBuildings", JsonValue::MakeBool(r.cfg.includeBuildings));
    add(cfg, "singleSample", JsonValue::MakeBool(r.cfg.singleSample));
    add(cfg, "singleAzimuthDeg", JsonValue::MakeNumber(static_cast<double>(r.cfg.singleAzimuthDeg)));
    add(cfg, "singleAltitudeDeg", JsonValue::MakeNumber(static_cast<double>(r.cfg.singleAltitudeDeg)));

    JsonValue alts = JsonValue::MakeArray();
    for (float a : r.cfg.altitudeDeg) {
      alts.arrayValue.push_back(JsonValue::MakeNumber(static_cast<double>(a)));
    }
    add(cfg, "altitudeDeg", std::move(alts));

    add(root, "config", std::move(cfg));
  }

  // Summary.
  {
    JsonValue s = JsonValue::MakeObject();
    add(s, "roofTileCount", JsonValue::MakeNumber(static_cast<double>(r.roofTileCount)));
    add(s, "residentPopulation", JsonValue::MakeNumber(static_cast<double>(r.residentPopulation)));
    add(s, "maxExposure01", JsonValue::MakeNumber(static_cast<double>(r.maxExposure01)));
    add(s, "maxPotential01", JsonValue::MakeNumber(static_cast<double>(r.maxPotential01)));
    add(s, "perCapitaPotential", JsonValue::MakeNumber(static_cast<double>(r.perCapitaPotential)));
    add(s, "highPotentialThreshold", JsonValue::MakeNumber(static_cast<double>(r.highPotentialThreshold)));
    add(s, "roofHighPotentialFrac", JsonValue::MakeNumber(static_cast<double>(r.roofHighPotentialFrac)));
    add(root, "summary", std::move(s));
  }

  std::string err;
  return WriteJsonFile(outPath, root, err,
                       JsonWriteOptions{.pretty = true, .indent = 2, .sortKeys = false});
}

} // namespace

int main(int argc, char** argv)
{
  std::string inPath;
  std::string outJson;
  bool quiet = false;
  bool verifyCrc = false;

  SolarPotentialConfig cfg{};
  cfg.azimuthSamples = 16;
  cfg.maxHorizonRadius = 64;

  bool sunAzSet = false;
  bool sunAltSet = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? std::string(argv[i]) : std::string();
    if (arg == "-h" || arg == "--help") {
      PrintHelp();
      return 0;
    }
    if (arg == "--quiet") {
      quiet = true;
      continue;
    }
    if (arg == "--verify-crc") {
      verifyCrc = true;
      continue;
    }
    if (arg == "--json" && i + 1 < argc) {
      outJson = argv[++i];
      continue;
    }
    if (arg == "--no-buildings") {
      cfg.includeBuildings = false;
      continue;
    }
    if (arg == "--azimuth-samples" && i + 1 < argc) {
      int v = 0;
      if (!ParseInt(argv[++i], v) || v <= 0) {
        std::cerr << "Invalid --azimuth-samples value\n";
        return 2;
      }
      cfg.azimuthSamples = v;
      continue;
    }
    if (arg == "--max-radius" && i + 1 < argc) {
      int v = 0;
      if (!ParseInt(argv[++i], v) || v < 0) {
        std::cerr << "Invalid --max-radius value\n";
        return 2;
      }
      cfg.maxHorizonRadius = v;
      continue;
    }
    if (arg == "--altitudes" && i + 1 < argc) {
      std::vector<float> alts;
      if (!ParseFloatListCsv(argv[++i], alts)) {
        std::cerr << "Invalid --altitudes value (expected comma-separated floats)\n";
        return 2;
      }
      cfg.altitudeDeg = std::move(alts);
      continue;
    }
    if (arg == "--sun-azimuth" && i + 1 < argc) {
      float v = 0.0f;
      if (!ParseFloat(argv[++i], v)) {
        std::cerr << "Invalid --sun-azimuth value\n";
        return 2;
      }
      cfg.singleAzimuthDeg = v;
      sunAzSet = true;
      continue;
    }
    if (arg == "--sun-altitude" && i + 1 < argc) {
      float v = 0.0f;
      if (!ParseFloat(argv[++i], v)) {
        std::cerr << "Invalid --sun-altitude value\n";
        return 2;
      }
      cfg.singleAltitudeDeg = v;
      sunAltSet = true;
      continue;
    }

    if (!arg.empty() && arg[0] == '-') {
      std::cerr << "Unknown option: " << arg << "\n";
      return 2;
    }

    if (inPath.empty()) {
      inPath = arg;
    } else {
      std::cerr << "Unexpected extra argument: " << arg << "\n";
      return 2;
    }
  }

  if (inPath.empty()) {
    PrintHelp();
    return 2;
  }

  if (sunAzSet || sunAltSet) {
    cfg.singleSample = true;
  }

  SaveSummary summary{};
  SaveSummary* summaryPtr = nullptr;
  std::string err;

  if (verifyCrc) {
    if (!ReadSaveSummary(inPath, summary, err, /*verifyCrc=*/true)) {
      std::cerr << "Failed to read save summary: " << err << "\n";
      return 1;
    }
    if (summary.crcChecked && !summary.crcOk) {
      std::cerr << "CRC check failed: save appears corrupted\n";
      return 1;
    }
    summaryPtr = &summary;
  }

  World world;
  ProcGenConfig procCfg;
  SimConfig simCfg;
  if (!LoadWorldBinary(world, procCfg, simCfg, inPath, err)) {
    std::cerr << "Failed to load save: " << err << "\n";
    return 1;
  }

  const SolarPotentialResult res = ComputeSolarPotential(world, cfg);

  if (!quiet) {
    PrintSummary(res);
  }

  if (!outJson.empty()) {
    if (!WriteReportJson(outJson, inPath, summaryPtr, res)) {
      std::cerr << "Failed to write JSON report\n";
      return 1;
    }
  }

  return 0;
}
