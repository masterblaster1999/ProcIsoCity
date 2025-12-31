#include "isocity/Blueprint.hpp"
#include "isocity/SaveLoad.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace isocity;

namespace {

void PrintUsage()
{
  std::cout
      << "proc_isocity_blueprint - capture/apply/transform tile stamps\n\n"
      << "USAGE:\n"
      << "  proc_isocity_blueprint info <bp.isobp>\n"
      << "  proc_isocity_blueprint capture <save.bin> <x0> <y0> <w> <h> <out.isobp> [options]\n"
      << "  proc_isocity_blueprint apply <save.bin> <bp.isobp> <dstX> <dstY> <out.bin> [options]\n"
      << "  proc_isocity_blueprint diff <base.bin> <target.bin> <out.isobp> [options]\n"
      << "  proc_isocity_blueprint crop <bp.isobp> <out.isobp> [options]\n"
      << "  proc_isocity_blueprint transform <bp.isobp> <out.isobp> [options]\n\n"
      << "CAPTURE options:\n"
      << "  --fields <list>       Comma list: terrain,overlay,height,variation,level,occupants,district\n"
      << "  --sparse 0|1          If 1, only include tiles with overlay!=None (requires overlay in --fields)\n"
      << "  --zero-occ 0|1        If 1, zero occupants even if occupants in --fields\n"
      << "  --compress none|sllz  Blueprint payload compression (default: sllz)\n\n"
      << "APPLY options:\n"
      << "  --mode replace|stamp  Replace applies all deltas; stamp skips deltas where overlay==None\n"
      << "  --fields <list>       AND-mask applied at apply-time (lets you apply a subset of stored fields)\n"
      << "  --rotate 0|90|180|270 Rotation (clockwise)\n"
      << "  --mirrorx 0|1         Mirror horizontally after rotation\n"
      << "  --mirrory 0|1         Mirror vertically after rotation\n"
      << "  --allow-oob 0|1       If 1, silently skip tiles that land out of bounds\n"
      << "  --force 0|1           If 0, error on non-road overlays placed on water\n"
      << "  --recompute-roads 0|1 Recompute road auto-tiling masks after apply (default: 1)\n\n"
      << "DIFF options:\n"
      << "  --fields <list>       Fields to compare & emit (default: overlay,level,district,variation)\n"
      << "  --rect <x0> <y0> <w> <h>  Limit diff to a region (default: whole map)\n"
      << "  --crop 0|1            If 1, crop to minimal delta bounds (default: 1)\n"
      << "  --pad <N>             Extra padding tiles when cropping (default: 0)\n"
      << "  --zero-occ 0|1        If 1, emit occupant=0 even if occupants differs (layout-only diffs)\n"
      << "  --height-eps <E>      Height epsilon (default: 0 exact compare)\n"
      << "  --compress none|sllz  Output compression (default: sllz)\n\n"
      << "CROP options:\n"
      << "  --pad <N>             Extra padding tiles around delta bounds (default: 0)\n"
      << "  --compress none|sllz  Output compression (default: sllz)\n\n"
      << "TRANSFORM options:\n"
      << "  --rotate 0|90|180|270 Rotation (clockwise)\n"
      << "  --mirrorx 0|1         Mirror horizontally after rotation\n"
      << "  --mirrory 0|1         Mirror vertically after rotation\n"
      << "  --compress none|sllz  Output compression (default: sllz)\n";
}

bool ParseInt(const std::string& s, int& out)
{
  try {
    std::size_t idx = 0;
    const int v = std::stoi(s, &idx, 10);
    if (idx != s.size()) return false;
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseFloat(const std::string& s, float& out)
{
  try {
    std::size_t idx = 0;
    const float v = std::stof(s, &idx);
    if (idx != s.size()) return false;
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseBool01(const std::string& s, bool& out)
{
  if (s == "0") {
    out = false;
    return true;
  }
  if (s == "1") {
    out = true;
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
  for (auto& t : out) {
    std::transform(t.begin(), t.end(), t.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  }
  return out;
}

bool ParseFieldMaskList(const std::string& s, std::uint8_t& outMask, std::string& outError)
{
  outError.clear();
  outMask = 0;
  if (s.empty()) {
    outError = "empty fields list";
    return false;
  }

  const auto toks = SplitComma(s);
  for (const auto& t : toks) {
    if (t == "all") {
      outMask = 0xFFu;
      continue;
    }
    if (t == "none") {
      outMask = 0;
      continue;
    }

    if (t == "terrain") outMask |= static_cast<std::uint8_t>(TileFieldMask::Terrain);
    else if (t == "overlay") outMask |= static_cast<std::uint8_t>(TileFieldMask::Overlay);
    else if (t == "height") outMask |= static_cast<std::uint8_t>(TileFieldMask::Height);
    else if (t == "variation") outMask |= static_cast<std::uint8_t>(TileFieldMask::Variation);
    else if (t == "level") outMask |= static_cast<std::uint8_t>(TileFieldMask::Level);
    else if (t == "occupants") outMask |= static_cast<std::uint8_t>(TileFieldMask::Occupants);
    else if (t == "district") outMask |= static_cast<std::uint8_t>(TileFieldMask::District);
    else {
      outError = "unknown field: " + t;
      return false;
    }
  }
  return true;
}

bool ParseCompression(const std::string& s, BlueprintCompression& out)
{
  std::string t = s;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (t == "none") {
    out = BlueprintCompression::None;
    return true;
  }
  if (t == "sllz") {
    out = BlueprintCompression::SLLZ;
    return true;
  }
  return false;
}

bool ParseMode(const std::string& s, BlueprintApplyMode& out)
{
  std::string t = s;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (t == "replace") {
    out = BlueprintApplyMode::Replace;
    return true;
  }
  if (t == "stamp") {
    out = BlueprintApplyMode::Stamp;
    return true;
  }
  return false;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage();
    return 1;
  }

  const std::string cmd = argv[1];

  if (cmd == "help" || cmd == "-h" || cmd == "--help") {
    PrintUsage();
    return 0;
  }

  if (cmd == "info") {
    if (argc < 3) {
      PrintUsage();
      return 1;
    }
    Blueprint bp;
    std::string err;
    if (!LoadBlueprintBinary(bp, argv[2], err)) {
      std::cerr << "ERROR: " << err << "\n";
      return 2;
    }
    std::cout << "Blueprint: " << argv[2] << "\n";
    std::cout << "  version: " << bp.version << "\n";
    std::cout << "  size:    " << bp.width << "x" << bp.height << "\n";
    std::cout << "  tiles:   " << bp.tiles.size() << " deltas\n";

    // Also print delta bounds (useful for cropping decisions).
    Blueprint cropped;
    int offX = 0, offY = 0;
    if (CropBlueprintToDeltasBounds(bp, cropped, offX, offY, err, 0)) {
      if (!bp.tiles.empty()) {
        std::cout << "  deltasBounds: x=" << offX << " y=" << offY
                  << " w=" << cropped.width << " h=" << cropped.height << "\n";
      }
    }

    return 0;
  }

  if (cmd == "transform") {
    if (argc < 4) {
      PrintUsage();
      return 1;
    }

    const std::string inPath = argv[2];
    const std::string outPath = argv[3];

    BlueprintTransform tr;
    tr.rotateDeg = 0;
    tr.mirrorX = false;
    tr.mirrorY = false;

    BlueprintCompression comp = BlueprintCompression::SLLZ;

    for (int i = 4; i < argc; ++i) {
      const std::string k = argv[i];
      if (k == "--rotate" && i + 1 < argc) {
        int r = 0;
        if (!ParseInt(argv[++i], r)) {
          std::cerr << "ERROR: --rotate expects 0|90|180|270\n";
          return 2;
        }
        tr.rotateDeg = r;
      } else if (k == "--mirrorx" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --mirrorx expects 0|1\n";
          return 2;
        }
        tr.mirrorX = b;
      } else if (k == "--mirrory" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --mirrory expects 0|1\n";
          return 2;
        }
        tr.mirrorY = b;
      } else if (k == "--compress" && i + 1 < argc) {
        if (!ParseCompression(argv[++i], comp)) {
          std::cerr << "ERROR: --compress expects none|sllz\n";
          return 2;
        }
      } else {
        std::cerr << "ERROR: unknown option: " << k << "\n";
        return 2;
      }
    }

    Blueprint bp;
    std::string err;
    if (!LoadBlueprintBinary(bp, inPath, err)) {
      std::cerr << "ERROR: failed to load blueprint: " << err << "\n";
      return 2;
    }

    Blueprint out;
    if (!TransformBlueprint(bp, tr, out, err)) {
      std::cerr << "ERROR: transform failed: " << err << "\n";
      return 2;
    }

    if (!SaveBlueprintBinary(out, outPath, err, comp)) {
      std::cerr << "ERROR: failed to save blueprint: " << err << "\n";
      return 2;
    }

    std::cout << "Wrote blueprint: " << outPath << " (" << out.width << "x" << out.height
              << ", " << out.tiles.size() << " deltas, compression=" << BlueprintCompressionName(comp) << ")\n";
    return 0;
  }

  if (cmd == "crop") {
    if (argc < 4) {
      PrintUsage();
      return 1;
    }

    const std::string inPath = argv[2];
    const std::string outPath = argv[3];

    int pad = 0;
    BlueprintCompression comp = BlueprintCompression::SLLZ;

    for (int i = 4; i < argc; ++i) {
      const std::string k = argv[i];
      if (k == "--pad" && i + 1 < argc) {
        if (!ParseInt(argv[++i], pad) || pad < 0) {
          std::cerr << "ERROR: --pad expects a non-negative integer\n";
          return 2;
        }
      } else if (k == "--compress" && i + 1 < argc) {
        if (!ParseCompression(argv[++i], comp)) {
          std::cerr << "ERROR: --compress expects none|sllz\n";
          return 2;
        }
      } else {
        std::cerr << "ERROR: unknown option: " << k << "\n";
        return 2;
      }
    }

    Blueprint bp;
    std::string err;
    if (!LoadBlueprintBinary(bp, inPath, err)) {
      std::cerr << "ERROR: failed to load blueprint: " << err << "\n";
      return 2;
    }

    Blueprint cropped;
    int offX = 0, offY = 0;
    if (!CropBlueprintToDeltasBounds(bp, cropped, offX, offY, err, pad)) {
      std::cerr << "ERROR: crop failed: " << err << "\n";
      return 2;
    }

    if (!SaveBlueprintBinary(cropped, outPath, err, comp)) {
      std::cerr << "ERROR: failed to save blueprint: " << err << "\n";
      return 2;
    }

    std::cout << "Wrote blueprint: " << outPath << " (" << cropped.width << "x" << cropped.height
              << ", " << cropped.tiles.size() << " deltas, compression=" << BlueprintCompressionName(comp) << ")\n";
    if (!bp.tiles.empty()) {
      std::cout << "Crop offset: (" << offX << ", " << offY << ")\n";
    }
    return 0;
  }

  if (cmd == "diff") {
    if (argc < 5) {
      PrintUsage();
      return 1;
    }

    const std::string basePath = argv[2];
    const std::string targetPath = argv[3];
    const std::string outPath = argv[4];

    // Defaults.
    BlueprintDiffOptions diffOpt;
    diffOpt.fieldMask = static_cast<std::uint8_t>(TileFieldMask::Overlay) |
                        static_cast<std::uint8_t>(TileFieldMask::Level) |
                        static_cast<std::uint8_t>(TileFieldMask::District) |
                        static_cast<std::uint8_t>(TileFieldMask::Variation);
    diffOpt.heightEpsilon = 0.0f;
    diffOpt.zeroOccupants = false;

    bool doCrop = true;
    int pad = 0;
    bool rectProvided = false;
    int x0 = 0, y0 = 0, w = 0, h = 0;

    BlueprintCompression comp = BlueprintCompression::SLLZ;

    for (int i = 5; i < argc; ++i) {
      const std::string k = argv[i];
      if (k == "--fields" && i + 1 < argc) {
        std::uint8_t m = 0;
        std::string perr;
        if (!ParseFieldMaskList(argv[++i], m, perr)) {
          std::cerr << "ERROR: " << perr << "\n";
          return 2;
        }
        diffOpt.fieldMask = m;
      } else if (k == "--rect" && i + 4 < argc) {
        if (!ParseInt(argv[++i], x0) || !ParseInt(argv[++i], y0) || !ParseInt(argv[++i], w) || !ParseInt(argv[++i], h)) {
          std::cerr << "ERROR: --rect expects 4 integers\n";
          return 2;
        }
        rectProvided = true;
      } else if (k == "--crop" && i + 1 < argc) {
        bool b = true;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --crop expects 0|1\n";
          return 2;
        }
        doCrop = b;
      } else if (k == "--pad" && i + 1 < argc) {
        if (!ParseInt(argv[++i], pad) || pad < 0) {
          std::cerr << "ERROR: --pad expects a non-negative integer\n";
          return 2;
        }
      } else if (k == "--zero-occ" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --zero-occ expects 0|1\n";
          return 2;
        }
        diffOpt.zeroOccupants = b;
      } else if (k == "--height-eps" && i + 1 < argc) {
        float e = 0.0f;
        if (!ParseFloat(argv[++i], e) || e < 0.0f) {
          std::cerr << "ERROR: --height-eps expects a non-negative float\n";
          return 2;
        }
        diffOpt.heightEpsilon = e;
      } else if (k == "--compress" && i + 1 < argc) {
        if (!ParseCompression(argv[++i], comp)) {
          std::cerr << "ERROR: --compress expects none|sllz\n";
          return 2;
        }
      } else {
        std::cerr << "ERROR: unknown option: " << k << "\n";
        return 2;
      }
    }

    World baseWorld;
    World targetWorld;
    ProcGenConfig baseProc, targetProc;
    SimConfig baseSim, targetSim;
    std::string err;

    if (!LoadWorldBinary(baseWorld, baseProc, baseSim, basePath, err)) {
      std::cerr << "ERROR: failed to load base save: " << err << "\n";
      return 2;
    }
    if (!LoadWorldBinary(targetWorld, targetProc, targetSim, targetPath, err)) {
      std::cerr << "ERROR: failed to load target save: " << err << "\n";
      return 2;
    }

    if (!rectProvided) {
      x0 = 0;
      y0 = 0;
      w = baseWorld.width();
      h = baseWorld.height();
    }

    Blueprint bp;
    if (!CaptureBlueprintDiffRect(baseWorld, targetWorld, x0, y0, w, h, bp, err, diffOpt)) {
      std::cerr << "ERROR: diff capture failed: " << err << "\n";
      return 2;
    }

    int cropX = 0, cropY = 0;
    Blueprint outBp = bp;
    if (doCrop) {
      Blueprint cropped;
      if (!CropBlueprintToDeltasBounds(bp, cropped, cropX, cropY, err, pad)) {
        std::cerr << "ERROR: crop failed: " << err << "\n";
        return 2;
      }
      outBp = std::move(cropped);
    }

    if (!SaveBlueprintBinary(outBp, outPath, err, comp)) {
      std::cerr << "ERROR: failed to save blueprint: " << err << "\n";
      return 2;
    }

    std::cout << "Wrote blueprint: " << outPath << " (" << outBp.width << "x" << outBp.height
              << ", " << outBp.tiles.size() << " deltas, compression=" << BlueprintCompressionName(comp) << ")\n";
    if (doCrop && !bp.tiles.empty()) {
      const int dstX = x0 + cropX;
      const int dstY = y0 + cropY;
      std::cout << "Suggested apply dst: (" << dstX << ", " << dstY << ")\n";
      std::cout << "(Original region: x=" << x0 << " y=" << y0 << " w=" << w << " h=" << h << ")\n";
    }

    return 0;
  }

  if (cmd == "capture") {
    if (argc < 8) {
      PrintUsage();
      return 1;
    }

    const std::string savePath = argv[2];
    int x0 = 0, y0 = 0, w = 0, h = 0;
    if (!ParseInt(argv[3], x0) || !ParseInt(argv[4], y0) || !ParseInt(argv[5], w) || !ParseInt(argv[6], h)) {
      std::cerr << "ERROR: invalid rect args\n";
      return 2;
    }
    const std::string outPath = argv[7];

    BlueprintCaptureOptions opt;
    BlueprintCompression comp = BlueprintCompression::SLLZ;

    for (int i = 8; i < argc; ++i) {
      const std::string k = argv[i];
      if (k == "--fields" && i + 1 < argc) {
        std::string err;
        if (!ParseFieldMaskList(argv[++i], opt.fieldMask, err)) {
          std::cerr << "ERROR: " << err << "\n";
          return 2;
        }
      } else if (k == "--sparse" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --sparse expects 0|1\n";
          return 2;
        }
        opt.sparseByOverlay = b;
      } else if (k == "--zero-occ" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --zero-occ expects 0|1\n";
          return 2;
        }
        opt.zeroOccupants = b;
      } else if (k == "--compress" && i + 1 < argc) {
        if (!ParseCompression(argv[++i], comp)) {
          std::cerr << "ERROR: --compress expects none|sllz\n";
          return 2;
        }
      } else {
        std::cerr << "ERROR: unknown option: " << k << "\n";
        return 2;
      }
    }

    World world;
    ProcGenConfig procCfg;
    SimConfig simCfg;
    std::string err;
    if (!LoadWorldBinary(world, procCfg, simCfg, savePath, err)) {
      std::cerr << "ERROR: failed to load save: " << err << "\n";
      return 2;
    }

    Blueprint bp;
    if (!CaptureBlueprintRect(world, x0, y0, w, h, bp, err, opt)) {
      std::cerr << "ERROR: capture failed: " << err << "\n";
      return 2;
    }

    if (!SaveBlueprintBinary(bp, outPath, err, comp)) {
      std::cerr << "ERROR: failed to save blueprint: " << err << "\n";
      return 2;
    }

    std::cout << "Wrote blueprint: " << outPath << " (" << bp.width << "x" << bp.height
              << ", " << bp.tiles.size() << " deltas, compression=" << BlueprintCompressionName(comp) << ")\n";
    return 0;
  }

  if (cmd == "apply") {
    if (argc < 7) {
      PrintUsage();
      return 1;
    }

    const std::string savePath = argv[2];
    const std::string bpPath = argv[3];
    int dstX = 0, dstY = 0;
    if (!ParseInt(argv[4], dstX) || !ParseInt(argv[5], dstY)) {
      std::cerr << "ERROR: invalid dst args\n";
      return 2;
    }
    const std::string outPath = argv[6];

    Blueprint bp;
    std::string err;
    if (!LoadBlueprintBinary(bp, bpPath, err)) {
      std::cerr << "ERROR: failed to load blueprint: " << err << "\n";
      return 2;
    }

    BlueprintApplyOptions opt;
    opt.mode = BlueprintApplyMode::Stamp;
    opt.fieldMask = 0xFFu;
    opt.allowOutOfBounds = false;
    opt.force = true;
    opt.recomputeRoadMasks = true;
    opt.transform.rotateDeg = 0;
    opt.transform.mirrorX = false;
    opt.transform.mirrorY = false;

    for (int i = 7; i < argc; ++i) {
      const std::string k = argv[i];
      if (k == "--mode" && i + 1 < argc) {
        if (!ParseMode(argv[++i], opt.mode)) {
          std::cerr << "ERROR: --mode expects replace|stamp\n";
          return 2;
        }
      } else if (k == "--fields" && i + 1 < argc) {
        std::uint8_t m = 0;
        std::string perr;
        if (!ParseFieldMaskList(argv[++i], m, perr)) {
          std::cerr << "ERROR: " << perr << "\n";
          return 2;
        }
        opt.fieldMask = m;
      } else if (k == "--rotate" && i + 1 < argc) {
        int r = 0;
        if (!ParseInt(argv[++i], r)) {
          std::cerr << "ERROR: --rotate expects 0|90|180|270\n";
          return 2;
        }
        opt.transform.rotateDeg = r;
      } else if (k == "--mirrorx" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --mirrorx expects 0|1\n";
          return 2;
        }
        opt.transform.mirrorX = b;
      } else if (k == "--mirrory" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --mirrory expects 0|1\n";
          return 2;
        }
        opt.transform.mirrorY = b;
      } else if (k == "--allow-oob" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --allow-oob expects 0|1\n";
          return 2;
        }
        opt.allowOutOfBounds = b;
      } else if (k == "--force" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --force expects 0|1\n";
          return 2;
        }
        opt.force = b;
      } else if (k == "--recompute-roads" && i + 1 < argc) {
        bool b = false;
        if (!ParseBool01(argv[++i], b)) {
          std::cerr << "ERROR: --recompute-roads expects 0|1\n";
          return 2;
        }
        opt.recomputeRoadMasks = b;
      } else {
        std::cerr << "ERROR: unknown option: " << k << "\n";
        return 2;
      }
    }

    World world;
    ProcGenConfig procCfg;
    SimConfig simCfg;
    if (!LoadWorldBinary(world, procCfg, simCfg, savePath, err)) {
      std::cerr << "ERROR: failed to load save: " << err << "\n";
      return 2;
    }

    if (!ApplyBlueprint(world, bp, dstX, dstY, opt, err)) {
      std::cerr << "ERROR: apply failed: " << err << "\n";
      return 2;
    }

    if (!SaveWorldBinary(world, procCfg, simCfg, outPath, err)) {
      std::cerr << "ERROR: failed to save out world: " << err << "\n";
      return 2;
    }

    std::cout << "Wrote save: " << outPath << "\n";
    return 0;
  }

  PrintUsage();
  return 1;
}
