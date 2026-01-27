#include "isocity/SaveLoad.hpp"
#include "isocity/Walkability.hpp"
#include "isocity/Json.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

namespace {

using namespace isocity;

void PrintHelp()
{
  std::cout
      << "proc_isocity_walkability (headless walkability / 15-minute city report)\n\n"
      << "Usage:\n"
      << "  proc_isocity_walkability <save.bin> [options]\n\n"
      << "Options:\n"
      << "  --json <out.json>          Write a JSON report.\n"
      << "  --coverage-steps <N>       Coverage threshold (steps) for per-category coverage stats (default: 15).\n"
      << "  --weight-mode <time|steps> Isochrone weighting for distance (default: time).\n"
      << "  --no-outside               Do not require road connectivity to the map edge.\n"
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

bool ParseWeightMode(const std::string& s, IsochroneWeightMode& out)
{
  if (s == "time" || s == "travel" || s == "traveltime") {
    out = IsochroneWeightMode::TravelTime;
    return true;
  }
  if (s == "steps" || s == "distance" || s == "dist") {
    out = IsochroneWeightMode::Steps;
    return true;
  }
  return false;
}

static void PrintSummary(const WalkabilityResult& r)
{
  auto pct = [](float v01) -> double { return static_cast<double>(v01) * 100.0; };

  std::cout << "Walkability summary\n";
  std::cout << "- grid: " << r.w << "x" << r.h << "\n";
  std::cout << "- residents: " << r.residentPopulation << " (" << r.residentialTileCount << " tiles)\n";
  std::cout << "- resident avg score: " << pct(r.residentAvgOverall01) << "%\n";

  std::cout << "- coverage within " << r.cfg.coverageThresholdSteps << " steps (share of resident population)\n";
  std::cout << "  - parks: " << pct(r.residentCoverageFrac[0]) << "%\n";
  std::cout << "  - retail: " << pct(r.residentCoverageFrac[1]) << "%\n";
  std::cout << "  - education: " << pct(r.residentCoverageFrac[2]) << "%\n";
  std::cout << "  - health: " << pct(r.residentCoverageFrac[3]) << "%\n";
  std::cout << "  - safety: " << pct(r.residentCoverageFrac[4]) << "%\n";
  std::cout << "  - all categories: " << pct(r.residentAllCategoriesFrac) << "%\n";
}

static bool WriteReportJson(const std::string& outPath, const std::string& inPath,
                            const SaveSummary* sum, const WalkabilityResult& r)
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
    add(cfg, "requireOutsideConnection", JsonValue::MakeBool(r.cfg.requireOutsideConnection));
    add(cfg, "weightMode", JsonValue::MakeString(r.cfg.weightMode == IsochroneWeightMode::Steps ? "steps" : "time"));
    add(cfg, "coverageThresholdSteps", JsonValue::MakeNumber(static_cast<double>(r.cfg.coverageThresholdSteps)));
    add(cfg, "accessStepCostMilli", JsonValue::MakeNumber(static_cast<double>(r.cfg.accessStepCostMilli)));
    add(root, "config", std::move(cfg));
  }

  // Summary.
  {
    JsonValue s = JsonValue::MakeObject();
    add(s, "residentPopulation", JsonValue::MakeNumber(static_cast<double>(r.residentPopulation)));
    add(s, "residentialTiles", JsonValue::MakeNumber(static_cast<double>(r.residentialTileCount)));
    add(s, "residentAvgOverall01", JsonValue::MakeNumber(static_cast<double>(r.residentAvgOverall01)));
    add(s, "residentAllCategoriesFrac", JsonValue::MakeNumber(static_cast<double>(r.residentAllCategoriesFrac)));

    JsonValue cov = JsonValue::MakeObject();
    add(cov, "park", JsonValue::MakeNumber(static_cast<double>(r.residentCoverageFrac[0])));
    add(cov, "retail", JsonValue::MakeNumber(static_cast<double>(r.residentCoverageFrac[1])));
    add(cov, "education", JsonValue::MakeNumber(static_cast<double>(r.residentCoverageFrac[2])));
    add(cov, "health", JsonValue::MakeNumber(static_cast<double>(r.residentCoverageFrac[3])));
    add(cov, "safety", JsonValue::MakeNumber(static_cast<double>(r.residentCoverageFrac[4])));
    add(s, "coverageFrac", std::move(cov));
    add(root, "summary", std::move(s));
  }

  // Amenity source counts (useful for diagnosing "why is everything zero?").
  {
    JsonValue sc = JsonValue::MakeObject();
    add(sc, "park", JsonValue::MakeNumber(static_cast<double>(r.sourceCount[0])));
    add(sc, "retail", JsonValue::MakeNumber(static_cast<double>(r.sourceCount[1])));
    add(sc, "education", JsonValue::MakeNumber(static_cast<double>(r.sourceCount[2])));
    add(sc, "health", JsonValue::MakeNumber(static_cast<double>(r.sourceCount[3])));
    add(sc, "safety", JsonValue::MakeNumber(static_cast<double>(r.sourceCount[4])));
    add(root, "sourceCount", std::move(sc));
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

  WalkabilityConfig wc{};
  wc.enabled = true;
  wc.requireOutsideConnection = true;
  wc.weightMode = IsochroneWeightMode::TravelTime;
  wc.coverageThresholdSteps = 15;

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
    if (arg == "--no-outside") {
      wc.requireOutsideConnection = false;
      continue;
    }
    if (arg == "--json" && i + 1 < argc) {
      outJson = argv[++i];
      continue;
    }
    if (arg == "--coverage-steps" && i + 1 < argc) {
      int v = 0;
      if (!ParseInt(argv[++i], v)) {
        std::cerr << "Invalid --coverage-steps value\n";
        return 2;
      }
      wc.coverageThresholdSteps = std::max(0, v);
      continue;
    }
    if (arg == "--weight-mode" && i + 1 < argc) {
      IsochroneWeightMode wm = IsochroneWeightMode::TravelTime;
      if (!ParseWeightMode(argv[++i], wm)) {
        std::cerr << "Invalid --weight-mode (use 'time' or 'steps')\n";
        return 2;
      }
      wc.weightMode = wm;
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

  const WalkabilityResult res = ComputeWalkability(world, wc);

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
