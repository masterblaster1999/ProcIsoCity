// proc_isocity_worlddiff
//
// Headless world comparison tool.
//   - Compare two saves (LoadWorldBinary) OR two generated seeds (GenerateWorld + optional sim days).
//   - Print diff stats + hashes.
//   - Optionally write diff images (PNG) for quick visual inspection.

#include "isocity/WorldDiff.hpp"
#include "isocity/WorldDiffViz.hpp"

#include "isocity/Export.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Json.hpp"
#include "isocity/PerceptualHash.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Sim.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

namespace {

bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
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
  const unsigned long long v = std::strtoull(s.c_str() + offset, &end, base);
  if (!end || *end != '\0') return false;
  *out = static_cast<std::uint64_t>(v);
  return true;
}

bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const float v = std::strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string::npos) return false;

  const std::string a = s.substr(0, pos);
  const std::string b = s.substr(pos + 1);
  int w = 0, h = 0;
  if (!ParseI32(a, &w) || !ParseI32(b, &h)) return false;
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

void PrintHelp()
{
  std::cout
      << "proc_isocity_worlddiff (headless: compare two worlds)\n\n"
      << "Usage:\n"
      << "  proc_isocity_worlddiff (--load-a <saveA.bin> | --seed-a <u64> --size-a <WxH>)\n"
      << "                        (--load-b <saveB.bin> | --seed-b <u64> --size-b <WxH>)\n"
      << "                        [--days-a <N>] [--days-b <N>] [--height-eps <F>]\n"
      << "                        [--out <diff.png>] [--out-count <diff_count.png>] [--json <summary.json>]\n\n"
      << "Notes:\n"
      << "  - If --load-* is used, ProcGenConfig + SimConfig are loaded from the save and --days-* will\n"
      << "    advance the simulation deterministically before comparison.\n"
      << "  - If --seed-* is used, a new world is generated via ProcGen and optionally simulated for --days-*\n"
      << "    ticks (default SimConfig{}).\n"
      << "  - Diff images are rendered over the overlapping region only when sizes differ.\n";
}

struct WorldSource {
  enum class Kind : std::uint8_t { None = 0, Load = 1, Seed = 2 };
  Kind kind = Kind::None;

  std::string loadPath;
  std::uint64_t seed = 0;
  int w = 0;
  int h = 0;
  int days = 0;
};

bool BuildWorld(const WorldSource& src, isocity::World& outWorld, isocity::ProcGenConfig& outProc,
                isocity::SimConfig& outSim, std::string& outErr)
{
  using namespace isocity;
  outErr.clear();

  if (src.kind == WorldSource::Kind::Load) {
    if (src.loadPath.empty()) {
      outErr = "missing --load-* path";
      return false;
    }
    if (!LoadWorldBinary(outWorld, outProc, outSim, src.loadPath, outErr)) {
      return false;
    }
    if (src.days > 0) {
      Simulator sim(outSim);
      for (int i = 0; i < src.days; ++i) sim.stepOnce(outWorld);
    }
    return true;
  }

  if (src.kind == WorldSource::Kind::Seed) {
    if (src.w <= 0 || src.h <= 0) {
      outErr = "missing/invalid --size-* for seeded world";
      return false;
    }
    outProc = ProcGenConfig{};
    outSim = SimConfig{};
    outWorld = GenerateWorld(src.w, src.h, src.seed, outProc);
    if (src.days > 0) {
      Simulator sim(outSim);
      for (int i = 0; i < src.days; ++i) sim.stepOnce(outWorld);
    }
    return true;
  }

  outErr = "missing world source (use --load-* or --seed-* + --size-*)";
  return false;
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  WorldSource a;
  WorldSource b;
  float heightEps = 1e-6f;

  std::optional<std::filesystem::path> outColor;
  std::optional<std::filesystem::path> outCount;
  std::optional<std::filesystem::path> outJson;

  if (argc <= 1) {
    PrintHelp();
    return 2;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    }

    auto needValue = [&](const char* flag) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << flag << "\n";
        return std::nullopt;
      }
      return std::string(argv[++i]);
    };

    if (arg == "--load-a") {
      auto v = needValue("--load-a");
      if (!v) return 2;
      a.kind = WorldSource::Kind::Load;
      a.loadPath = *v;
      continue;
    }
    if (arg == "--load-b") {
      auto v = needValue("--load-b");
      if (!v) return 2;
      b.kind = WorldSource::Kind::Load;
      b.loadPath = *v;
      continue;
    }
    if (arg == "--seed-a") {
      auto v = needValue("--seed-a");
      if (!v) return 2;
      std::uint64_t s = 0;
      if (!ParseU64(*v, &s)) {
        std::cerr << "Bad --seed-a\n";
        return 2;
      }
      a.kind = WorldSource::Kind::Seed;
      a.seed = s;
      continue;
    }
    if (arg == "--seed-b") {
      auto v = needValue("--seed-b");
      if (!v) return 2;
      std::uint64_t s = 0;
      if (!ParseU64(*v, &s)) {
        std::cerr << "Bad --seed-b\n";
        return 2;
      }
      b.kind = WorldSource::Kind::Seed;
      b.seed = s;
      continue;
    }
    if (arg == "--size-a") {
      auto v = needValue("--size-a");
      if (!v) return 2;
      if (!ParseWxH(*v, &a.w, &a.h)) {
        std::cerr << "Bad --size-a (expected WxH)\n";
        return 2;
      }
      continue;
    }
    if (arg == "--size-b") {
      auto v = needValue("--size-b");
      if (!v) return 2;
      if (!ParseWxH(*v, &b.w, &b.h)) {
        std::cerr << "Bad --size-b (expected WxH)\n";
        return 2;
      }
      continue;
    }
    if (arg == "--days-a") {
      auto v = needValue("--days-a");
      if (!v) return 2;
      if (!ParseI32(*v, &a.days) || a.days < 0) {
        std::cerr << "Bad --days-a\n";
        return 2;
      }
      continue;
    }
    if (arg == "--days-b") {
      auto v = needValue("--days-b");
      if (!v) return 2;
      if (!ParseI32(*v, &b.days) || b.days < 0) {
        std::cerr << "Bad --days-b\n";
        return 2;
      }
      continue;
    }
    if (arg == "--height-eps") {
      auto v = needValue("--height-eps");
      if (!v) return 2;
      float f = 0.0f;
      if (!ParseF32(*v, &f) || f < 0.0f) {
        std::cerr << "Bad --height-eps\n";
        return 2;
      }
      heightEps = f;
      continue;
    }
    if (arg == "--out") {
      auto v = needValue("--out");
      if (!v) return 2;
      outColor = std::filesystem::path(*v);
      continue;
    }
    if (arg == "--out-count") {
      auto v = needValue("--out-count");
      if (!v) return 2;
      outCount = std::filesystem::path(*v);
      continue;
    }
    if (arg == "--json") {
      auto v = needValue("--json");
      if (!v) return 2;
      outJson = std::filesystem::path(*v);
      continue;
    }

    std::cerr << "Unknown arg: " << arg << "\n";
    return 2;
  }

  World wa;
  World wb;
  ProcGenConfig procA{};
  ProcGenConfig procB{};
  SimConfig simA{};
  SimConfig simB{};
  std::string err;

  if (!BuildWorld(a, wa, procA, simA, err)) {
    std::cerr << "Failed to build world A: " << err << "\n";
    return 1;
  }
  if (!BuildWorld(b, wb, procB, simB, err)) {
    std::cerr << "Failed to build world B: " << err << "\n";
    return 1;
  }

  const WorldDiffStats d = DiffWorldTiles(wa, wb, heightEps);
  const WorldDiffBounds bounds = ComputeWorldDiffBounds(wa, wb, heightEps);

  const std::uint64_t hashA = HashWorld(wa, true);
  const std::uint64_t hashB = HashWorld(wb, true);

  const std::uint64_t phA = ComputeWorldOverlayPHash(wa);
  const std::uint64_t phB = ComputeWorldOverlayPHash(wb);
  const int phHam = HammingDistance64(phA, phB);

  std::cout << "A: " << d.widthA << "x" << d.heightA << " seed=" << HexU64(wa.seed()) << " hash=" << HexU64(hashA)
            << " phash=" << HexU64(phA) << "\n";
  std::cout << "B: " << d.widthB << "x" << d.heightB << " seed=" << HexU64(wb.seed()) << " hash=" << HexU64(hashB)
            << " phash=" << HexU64(phB) << "\n";
  std::cout << "Diff: tilesCompared=" << d.tilesCompared
            << " tilesDifferent=" << d.tilesDifferent
            << " sizeMismatch=" << (d.sizeMismatch ? "true" : "false")
            << " overlayPHashHamming=" << phHam << "\n";

  if (d.tilesDifferent > 0) {
    std::cout << "  terrain=" << d.terrainDifferent
              << " overlay=" << d.overlayDifferent
              << " height=" << d.heightDifferent
              << " variation=" << d.variationDifferent
              << " level=" << d.levelDifferent
              << " occupants=" << d.occupantsDifferent
              << " district=" << d.districtDifferent << "\n";
    if (bounds.anyDifferent) {
      std::cout << "  bounds: x=[" << bounds.minX << "," << bounds.maxX << ") y=[" << bounds.minY << "," << bounds.maxY
                << ")\n";
    }
  }

  if (outColor) {
    const PpmImage img = RenderWorldDiffColor(wa, wb, heightEps);
    std::string e;
    if (!WritePng(outColor->string(), img, e)) {
      std::cerr << "Failed to write " << outColor->string() << ": " << e << "\n";
      return 1;
    }
  }

  if (outCount) {
    const PpmImage img = RenderWorldDiffCount(wa, wb, heightEps);
    std::string e;
    if (!WritePng(outCount->string(), img, e)) {
      std::cerr << "Failed to write " << outCount->string() << ": " << e << "\n";
      return 1;
    }
  }

  if (outJson) {
    JsonValue root = JsonValue::MakeObject();
    root.objectValue.emplace_back("type", JsonValue::MakeString("procisocity_worlddiff"));
    root.objectValue.emplace_back("version", JsonValue::MakeNumber(1.0));

    auto worldObj = [&](const char* name, const World& w, std::uint64_t h, std::uint64_t ph, const WorldSource& src) {
      JsonValue o = JsonValue::MakeObject();
      o.objectValue.emplace_back("width", JsonValue::MakeNumber(static_cast<double>(w.width())));
      o.objectValue.emplace_back("height", JsonValue::MakeNumber(static_cast<double>(w.height())));
      o.objectValue.emplace_back("seed_hex", JsonValue::MakeString(HexU64(w.seed())));
      o.objectValue.emplace_back("hash_hex", JsonValue::MakeString(HexU64(h)));
      o.objectValue.emplace_back("overlay_phash_hex", JsonValue::MakeString(HexU64(ph)));

      if (src.kind == WorldSource::Kind::Load) {
        o.objectValue.emplace_back("source", JsonValue::MakeString("load"));
        o.objectValue.emplace_back("path", JsonValue::MakeString(src.loadPath));
        o.objectValue.emplace_back("days", JsonValue::MakeNumber(static_cast<double>(src.days)));
      } else if (src.kind == WorldSource::Kind::Seed) {
        o.objectValue.emplace_back("source", JsonValue::MakeString("seed"));
        o.objectValue.emplace_back("size", JsonValue::MakeString(std::to_string(src.w) + "x" + std::to_string(src.h)));
        o.objectValue.emplace_back("days", JsonValue::MakeNumber(static_cast<double>(src.days)));
      } else {
        o.objectValue.emplace_back("source", JsonValue::MakeString("unknown"));
      }
      root.objectValue.emplace_back(name, std::move(o));
    };

    worldObj("a", wa, hashA, phA, a);
    worldObj("b", wb, hashB, phB, b);

    JsonValue diff = JsonValue::MakeObject();
    diff.objectValue.emplace_back("tilesCompared", JsonValue::MakeNumber(static_cast<double>(d.tilesCompared)));
    diff.objectValue.emplace_back("tilesDifferent", JsonValue::MakeNumber(static_cast<double>(d.tilesDifferent)));
    diff.objectValue.emplace_back("sizeMismatch", JsonValue::MakeBool(d.sizeMismatch));
    diff.objectValue.emplace_back("overlayPHashHamming", JsonValue::MakeNumber(static_cast<double>(phHam)));

    JsonValue fields = JsonValue::MakeObject();
    fields.objectValue.emplace_back("terrain", JsonValue::MakeNumber(static_cast<double>(d.terrainDifferent)));
    fields.objectValue.emplace_back("overlay", JsonValue::MakeNumber(static_cast<double>(d.overlayDifferent)));
    fields.objectValue.emplace_back("height", JsonValue::MakeNumber(static_cast<double>(d.heightDifferent)));
    fields.objectValue.emplace_back("variation", JsonValue::MakeNumber(static_cast<double>(d.variationDifferent)));
    fields.objectValue.emplace_back("level", JsonValue::MakeNumber(static_cast<double>(d.levelDifferent)));
    fields.objectValue.emplace_back("occupants", JsonValue::MakeNumber(static_cast<double>(d.occupantsDifferent)));
    fields.objectValue.emplace_back("district", JsonValue::MakeNumber(static_cast<double>(d.districtDifferent)));
    diff.objectValue.emplace_back("fields", std::move(fields));

    JsonValue bnd = JsonValue::MakeObject();
    bnd.objectValue.emplace_back("anyDifferent", JsonValue::MakeBool(bounds.anyDifferent));
    bnd.objectValue.emplace_back("minX", JsonValue::MakeNumber(static_cast<double>(bounds.minX)));
    bnd.objectValue.emplace_back("minY", JsonValue::MakeNumber(static_cast<double>(bounds.minY)));
    bnd.objectValue.emplace_back("maxX", JsonValue::MakeNumber(static_cast<double>(bounds.maxX)));
    bnd.objectValue.emplace_back("maxY", JsonValue::MakeNumber(static_cast<double>(bounds.maxY)));
    bnd.objectValue.emplace_back("tilesDifferent", JsonValue::MakeNumber(static_cast<double>(bounds.tilesDifferent)));
    bnd.objectValue.emplace_back("overlapW", JsonValue::MakeNumber(static_cast<double>(bounds.overlapW)));
    bnd.objectValue.emplace_back("overlapH", JsonValue::MakeNumber(static_cast<double>(bounds.overlapH)));
    diff.objectValue.emplace_back("bounds", std::move(bnd));

    root.objectValue.emplace_back("diff", std::move(diff));

    std::string e;
    JsonWriteOptions opt{};
    opt.pretty = true;
    opt.indent = 2;
    opt.sortKeys = false;
    if (!WriteJsonFile(outJson->string(), root, e, opt)) {
      std::cerr << "Failed to write " << outJson->string() << ": " << e << "\n";
      return 1;
    }
  }

  return 0;
}
