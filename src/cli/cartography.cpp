#include "isocity/Cartography.hpp"
#include "isocity/Export.hpp"
#include "isocity/Json.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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

bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  int v = 0;
  if (!ParseI32(s, &v)) return false;
  if (v != 0 && v != 1) return false;
  *out = (v != 0);
  return true;
}

bool EnsureParentDir(const std::string& path)
{
  try {
    const std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (parent.empty()) return true;
    std::filesystem::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

void PrintHelp()
{
  using namespace isocity;
  std::cout
      << "proc_isocity_cartography (headless labeled isometric poster exporter)\n\n"
      << "Renders an isometric overview and overlays deterministic labels (district names + street names)\n"
      << "using the built-in 5x7 font. Outputs a PNG that is easy to share / print and stable enough\n"
      << "for regression artifacts.\n\n"
      << "Usage:\n"
      << "  proc_isocity_cartography --out <poster.png> --load <save.bin> [options]\n"
      << "  proc_isocity_cartography --out <poster.png> --seed <u64> --size <WxH> [options]\n\n"
      << "World source:\n"
      << "  --load <save.bin>        Load an existing save.\n"
      << "  --seed <u64>             Seed for generation (decimal or 0x...).\n"
      << "  --size <WxH>             World size if generating (default: 128x128).\n\n"
      << "Output:\n"
      << "  --out <poster.png>       Output PNG path (required).\n"
      << "  --labels-json <file>     Optional JSON dump of placed labels + district names.\n\n"
      << "Base render:\n"
      << "  --layer <name>           Base layer (default: overlay). Valid: terrain overlay height landvalue traffic\n"
      << "                          goods_traffic goods_fill district\n"
      << "  --tileW <px>             Iso tile width (default: 16).\n"
      << "  --tileH <px>             Iso tile height (default: 8).\n"
      << "  --heightScale <px>       Elevation scale in pixels (default: 14).\n"
      << "  --margin <px>            Margin around iso bounds (default: 8).\n"
      << "  --fancy <0|1>            Fancy terrain/overlay rendering (default: 1).\n"
      << "  --grid <0|1>             Draw tile grid lines (default: 0).\n"
      << "  --cliffs <0|1>           Draw cliff walls (default: 1).\n\n"
      << "Labels / poster styling:\n"
      << "  --poster <0|1>           Add title + legend margins (default: 1).\n"
      << "  --title <text>           Override poster title (default: generated from seed).\n"
      << "  --no-streets             Disable street labels.\n"
      << "  --no-districts           Disable district labels.\n"
      << "  --no-boundaries          Disable district boundary outlines.\n"
      << "  --max-street-labels <N>  Maximum street labels (default: 36).\n"
      << "  --max-district-labels <N>Maximum district labels (default: 8).\n"
      << "  --street-scale <N>       Street label text scale (default: 2).\n"
      << "  --district-scale <N>     District label text scale (default: 3).\n"
      << "  --title-scale <N>        Title text scale (default: 4).\n"
      << "  --label-bg <0|1>         Draw translucent label boxes (default: 1).\n"
      << "  --label-pad <N>          Label padding pixels (default: 2).\n\n"
      << "Street naming knobs (same as proc_isocity_streetnames):\n"
      << "  --merge-intersections <0|1>  Merge through 4-way intersections (default: 1).\n"
      << "  --merge-corners <0|1>        Merge through gentle corners (default: 1).\n"
      << "  --ordinals <0|1>             Allow ordinal street names (default: 1).\n";
}

} // namespace

int main(int argc, char** argv)
{
  using namespace isocity;

  std::string loadPath;
  std::string outPng;
  std::string labelsJson;

  std::uint64_t seed = 1;
  int w = 128;
  int h = 128;

  ExportLayer layer = ExportLayer::Overlay;
  IsoOverviewConfig isoCfg;
  StreetNamingConfig streetCfg;
  CartographyConfig cartCfg;

  auto requireValue = [&](int& i, std::string& out) -> bool {
    if (i + 1 >= argc) return false;
    out = argv[++i];
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string val;

    if (arg == "--help" || arg == "-h") {
      PrintHelp();
      return 0;
    } else if (arg == "--load") {
      if (!requireValue(i, val)) {
        std::cerr << "--load requires a path\n";
        return 2;
      }
      loadPath = val;
    } else if (arg == "--seed") {
      if (!requireValue(i, val) || !ParseU64(val, &seed)) {
        std::cerr << "--seed requires a valid integer (decimal or 0x...)\n";
        return 2;
      }
    } else if (arg == "--size") {
      if (!requireValue(i, val) || !ParseWxH(val, &w, &h)) {
        std::cerr << "--size requires format WxH (e.g. 128x128)\n";
        return 2;
      }
    } else if (arg == "--out") {
      if (!requireValue(i, val)) {
        std::cerr << "--out requires a path\n";
        return 2;
      }
      outPng = val;
    } else if (arg == "--labels-json") {
      if (!requireValue(i, val)) {
        std::cerr << "--labels-json requires a path\n";
        return 2;
      }
      labelsJson = val;
    } else if (arg == "--layer") {
      if (!requireValue(i, val) || !ParseExportLayer(val, layer)) {
        std::cerr << "--layer requires a valid layer name (e.g. overlay)\n";
        return 2;
      }
    } else if (arg == "--tileW") {
      if (!requireValue(i, val) || !ParseI32(val, &isoCfg.tileW) || isoCfg.tileW <= 0) {
        std::cerr << "--tileW requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--tileH") {
      if (!requireValue(i, val) || !ParseI32(val, &isoCfg.tileH) || isoCfg.tileH <= 0) {
        std::cerr << "--tileH requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--heightScale") {
      if (!requireValue(i, val) || !ParseI32(val, &isoCfg.heightScalePx) || isoCfg.heightScalePx < 0) {
        std::cerr << "--heightScale requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--margin") {
      if (!requireValue(i, val) || !ParseI32(val, &isoCfg.marginPx) || isoCfg.marginPx < 0) {
        std::cerr << "--margin requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--fancy") {
      if (!requireValue(i, val) || !ParseBool01(val, &isoCfg.fancy)) {
        std::cerr << "--fancy requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--grid") {
      if (!requireValue(i, val) || !ParseBool01(val, &isoCfg.drawGrid)) {
        std::cerr << "--grid requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--cliffs") {
      if (!requireValue(i, val) || !ParseBool01(val, &isoCfg.drawCliffs)) {
        std::cerr << "--cliffs requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--poster") {
      if (!requireValue(i, val) || !ParseBool01(val, &cartCfg.poster)) {
        std::cerr << "--poster requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--title") {
      if (!requireValue(i, val)) {
        std::cerr << "--title requires a string\n";
        return 2;
      }
      cartCfg.titleOverride = val;
    } else if (arg == "--no-streets") {
      cartCfg.labelStreets = false;
    } else if (arg == "--no-districts") {
      cartCfg.labelDistricts = false;
    } else if (arg == "--no-boundaries") {
      cartCfg.drawDistrictBoundaries = false;
    } else if (arg == "--max-street-labels") {
      if (!requireValue(i, val) || !ParseI32(val, &cartCfg.maxStreetLabels) || cartCfg.maxStreetLabels < 0) {
        std::cerr << "--max-street-labels requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--max-district-labels") {
      if (!requireValue(i, val) || !ParseI32(val, &cartCfg.maxDistrictLabels) || cartCfg.maxDistrictLabels < 0) {
        std::cerr << "--max-district-labels requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--street-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &cartCfg.streetTextScale) || cartCfg.streetTextScale <= 0) {
        std::cerr << "--street-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--district-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &cartCfg.districtTextScale) || cartCfg.districtTextScale <= 0) {
        std::cerr << "--district-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--title-scale") {
      if (!requireValue(i, val) || !ParseI32(val, &cartCfg.titleTextScale) || cartCfg.titleTextScale <= 0) {
        std::cerr << "--title-scale requires a positive integer\n";
        return 2;
      }
    } else if (arg == "--label-bg") {
      if (!requireValue(i, val) || !ParseBool01(val, &cartCfg.labelBackground)) {
        std::cerr << "--label-bg requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--label-pad") {
      if (!requireValue(i, val) || !ParseI32(val, &cartCfg.labelPaddingPx) || cartCfg.labelPaddingPx < 0) {
        std::cerr << "--label-pad requires a non-negative integer\n";
        return 2;
      }
    } else if (arg == "--merge-intersections") {
      if (!requireValue(i, val) || !ParseBool01(val, &streetCfg.mergeThroughIntersections)) {
        std::cerr << "--merge-intersections requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--merge-corners") {
      if (!requireValue(i, val) || !ParseBool01(val, &streetCfg.mergeThroughCorners)) {
        std::cerr << "--merge-corners requires 0 or 1\n";
        return 2;
      }
    } else if (arg == "--ordinals") {
      if (!requireValue(i, val) || !ParseBool01(val, &streetCfg.allowOrdinalNames)) {
        std::cerr << "--ordinals requires 0 or 1\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown arg: " << arg << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (outPng.empty()) {
    std::cerr << "--out <poster.png> is required\n";
    PrintHelp();
    return 2;
  }

  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};

  if (!loadPath.empty()) {
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, loadPath, err)) {
      std::cerr << "Failed to load save: " << loadPath << "\n";
      std::cerr << err << "\n";
      return 2;
    }
  } else {
    world = GenerateWorld(w, h, seed, procCfg);
  }

  const CartographyResult poster = RenderLabeledIsoPoster(world, layer, isoCfg, streetCfg, cartCfg);

  if (!EnsureParentDir(outPng)) {
    std::cerr << "Failed to create output directory for: " << outPng << "\n";
    return 2;
  }

  std::string err;
  if (!WritePngRGBA(outPng, poster.image, err)) {
    std::cerr << "Failed to write PNG: " << outPng << "\n";
    std::cerr << err << "\n";
    return 2;
  }

  if (!labelsJson.empty()) {
    if (!EnsureParentDir(labelsJson)) {
      std::cerr << "Failed to create output directory for: " << labelsJson << "\n";
      return 2;
    }

    std::ofstream os(labelsJson);
    if (!os) {
      std::cerr << "Failed to open labels json: " << labelsJson << "\n";
      return 2;
    }

    JsonWriteOptions opt;
    opt.pretty = true;
    opt.sortKeys = true;

    JsonWriter jw(os, opt);
    jw.beginObject();
    jw.key("seed");
    jw.uintValue(world.seed());
    jw.key("width");
    jw.intValue(world.width());
    jw.key("height");
    jw.intValue(world.height());
    jw.key("title");
    jw.stringValue(poster.title);

    jw.key("districtNames");
    jw.beginArray();
    for (const std::string& n : poster.districtNames) jw.stringValue(n);
    jw.endArray();

    jw.key("labels");
    jw.beginArray();
    for (const MapLabel& l : poster.labels) {
      jw.beginObject();
      jw.key("kind");
      switch (l.kind) {
        case MapLabelKind::Title: jw.stringValue("title"); break;
        case MapLabelKind::District: jw.stringValue("district"); break;
        case MapLabelKind::Street: jw.stringValue("street"); break;
      }
      jw.key("id");
      jw.intValue(l.id);
      jw.key("text");
      jw.stringValue(l.text);
      jw.key("x");
      jw.intValue(l.x);
      jw.key("y");
      jw.intValue(l.y);
      jw.key("w");
      jw.intValue(l.w);
      jw.key("h");
      jw.intValue(l.h);
      jw.key("anchorX");
      jw.intValue(l.anchorX);
      jw.key("anchorY");
      jw.intValue(l.anchorY);
      jw.key("scale");
      jw.intValue(l.scale);
      jw.endObject();
    }
    jw.endArray();

    jw.endObject();
  }

  std::cout << "Cartography\n";
  std::cout << "  title:   " << poster.title << "\n";
  std::cout << "  labels:  " << poster.labels.size() << "\n";
  std::cout << "  output:  " << outPng << "\n";
  if (!labelsJson.empty()) std::cout << "  labels-json: " << labelsJson << "\n";

  return 0;
}
