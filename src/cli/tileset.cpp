// proc_isocity_tileset
//
// Headless procedural graphics generator.
//
// Generates a PNG sprite atlas containing the same procedural tile textures
// that the in-app renderer synthesizes at runtime (terrain diamonds, road and
// bridge auto-tiles, overlay diamonds).
//
// This tool is intentionally dependency-free and deterministic so tilesets can
// be produced in CI or shipped alongside mods.

#include "isocity/Export.hpp"
#include "isocity/GfxOutlines.hpp"
#include "isocity/GfxPalette.hpp"
#include "isocity/GfxMipmaps.hpp"
#include "isocity/GfxQuantize.hpp"
#include "isocity/GfxTileset.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
  *out = static_cast<float>(v);
  return true;
}

bool ParseF32Pair(const std::string& s, float* outA, float* outB)
{
  if (!outA || !outB) return false;
  const std::size_t pos = s.find_first_of(",xX");
  if (pos == std::string::npos) return false;
  float a = 0.0f;
  float b = 0.0f;
  if (!ParseF32(s.substr(0, pos), &a)) return false;
  if (!ParseF32(s.substr(pos + 1), &b)) return false;
  *outA = a;
  *outB = b;
  return true;
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_tileset (headless procedural tileset generator)\n\n"
      << "Generates a PNG sprite atlas of procedural textures (no external art):\n"
      << "  - terrain diamonds: water/sand/grass (8 variants each)\n"
      << "  - road auto-tiles: levels 1..3, masks 0..15, variants 0..3\n"
      << "  - bridge auto-tiles: levels 1..3, masks 0..15, variants 0..3\n"
      << "  - overlay diamonds: residential/commercial/industrial/park\n"
      << "  - optional building sprites: (kind x level x variants), with pivots\n"
      << "  - optional facility sprites: education/health/police/fire, with pivots\n"
      << "  - optional prop sprites: trees + streetlights, with pivots\n"
      << "  - optional vehicle sprites: cars + trucks, tile-sized with pivots\n\n"
      << "Usage:\n"
      << "  proc_isocity_tileset --out <tileset.png> [--meta <tileset.json>]\n"
      << "                       [--emit <emissive.png>] [--height <height.png>] [--normal <normal.png>] [--shadow <shadow.png>]\n"
      << "                       [--html <preview.html>]\n"
      << "                       [--outlines <outlines.json>] [--outline-svg <outlines.svg>]\n"
      << "                       [--seed <u64>] [--theme <name>]\n"
      << "                       [--tile <WxH>] [--pack <mode>] [--cols <n>] [--pack-width <px>] [--pow2 <0|1>] [--trim <0|1>] [--trim-border <px>] [--pad <n>] [--extrude <px>]\n"
      << "                       [--transitions <0|1>] [--transition-variants <n>]\n"
      << "                       [--buildings <0|1>] [--building-variants <n>] [--building-sprite-h <px>]\n"
      << "                       [--facilities <0|1>] [--facility-variants <n>] [--facility-sprite-h <px>]\n"
      << "                       [--props <0|1>] [--prop-variants <n>] [--prop-sprite-h <px>]\n"
      << "                       [--vehicles <0|1>] [--vehicle-variants <n>]\n"
      << "                       [--height-from <mode>] [--normal-strength <f>]\n"
      << "                       [--shadow-dir <dx,dy>] [--shadow-length <px>] [--shadow-blur <px>] [--shadow-opacity <f>]\n"
      << "                       [--shadow-tall-only <0|1>]\n"
      << "                       [--mip-dir <dir>] [--mip-levels <n>] [--mip-min-size <px>] [--mip-premultiply <0|1>]\n"
      << "                       [--mip-alpha-coverage <0|1>] [--mip-alpha-threshold <f>] [--mip-alpha-iters <n>]\n"
      << "                       [--indexed <0|1>] [--indexed-colors <n>] [--indexed-dither <0|1>] [--indexed-dither-strength <f>]\n"
      << "                       [--indexed-alpha <0|1>]\n"
      << "                       [--opaque <0|1>]\n\n"
      << "Options:\n"
      << "  --out <png>          Output sprite atlas PNG (RGBA by default).\n"
      << "  --meta <json>        Optional JSON metadata (name -> rect).\n"
      << "  --emit <png>         Optional emissive atlas (RGBA) for night lights (same layout).\n"
      << "  --height <png>       Optional grayscale height atlas (RGBA, same layout).\n"
      << "  --normal <png>       Optional normal map atlas (RGBA, same layout).\n"
      << "  --shadow <png>       Optional shadow mask atlas (RGBA, same layout).\n"
      << "  --sdf <png>          Optional signed distance field (SDF) atlas (RGBA, same layout).\n"
      << "  --html <html>        Optional HTML preview page for the generated atlas.\n"
      << "  --outlines <json>    Optional per-sprite vector outline JSON (polygons + hull).\n"
      << "  --outline-svg <svg>  Optional SVG overlay preview of outlines (atlas image as background).\n"
      << "  --outline-threshold <f> Alpha threshold in [0,1] for outline vectorization (default: 0.5).\n"
      << "  --outline-hull <0|1> If 1, include a convex hull around outline vertices (default: 1).\n"
      << "  --outline-holes <0|1> If 1, include holes; if 0, strip holes (default: 1).\n"
      << "  --outline-svg-scale <n> Scale factor for the SVG output dimensions (default: 1).\n"
      << "  --seed <u64>         Seed for graphics (default: 1). Accepts 0x... hex.\n"
      << "  --theme <name>       classic|autumn|desert|noir|neon|pastel|space_colony|procedural|procedural_muted|procedural_vibrant (default: classic).\n"
      << "  --tile <WxH>         Tile size (default: 64x32).\n"
      << "  --pack <mode>        Atlas packing: grid|maxrects (default: grid).\n"
      << "  --cols <n>           Atlas columns for grid packing (default: 32).\n"
      << "  --pack-width <px>    Target width for maxrects packing (0 = auto).\n"
      << "  --pow2 <0|1>         Round atlas dimensions up to the next power-of-two (default: 0).\n"
      << "  --trim <0|1>         Trim transparent borders per sprite before packing (default: 0).\n"
      << "  --trim-border <px>   Extra border kept when trimming (default: 1).\n"
      << "  --pad <n>            Padding/margin in pixels (default: 2).\n"
      << "  --extrude <px>       Extrude sprite borders into transparent padding (default: 0).\n"
      << "  --transitions <0|1>  Include terrain transition (shore/biome) tiles (default: 1).\n"
      << "  --transition-variants <n> Variants per transition mask (default: 4).\n"
      << "  --buildings <0|1>    Include taller building sprites (default: 0).\n"
      << "  --building-variants <n>  Variants per (kind, level) pair (default: 12).\n"
      << "  --building-sprite-h <px> Fixed sprite canvas height for buildings (default: auto).\n"
      << "  --facilities <0|1>   Include civic/service facility sprites (default: 0).\n"
      << "  --facility-variants <n> Variants per (kind, level) pair (default: 8).\n"
      << "  --facility-sprite-h <px> Fixed sprite canvas height for facilities (default: auto).\n"
      << "  --props <0|1>        Include prop sprites (trees + streetlights) (default: 0).\n"
      << "  --prop-variants <n>  Variants per prop kind (default: 16).\n"
      << "  --prop-sprite-h <px> Fixed canvas height for tall props (default: auto).\n"
      << "  --vehicles <0|1>     Include vehicle sprites (cars + trucks) (default: 0).\n"
      << "  --vehicle-variants <n> Variants per vehicle kind (default: 12).\n"
      << "  --height-from <mode> Height derivation: alpha|luma|alpha_luma (default: alpha_luma).\n"
      << "  --normal-strength <f> Normal map strength (default: 2.0).\n"
      << "  --shadow-dir <dx,dy> Shadow direction in pixel space (default: 1,1).\n"
      << "  --shadow-length <px> Shadow max offset in pixels (default: 18).\n"
      << "  --shadow-blur <px>   Shadow blur radius in pixels (default: 2).\n"
      << "  --shadow-opacity <f> Shadow opacity multiplier in [0,1] (default: 0.70).\n"
      << "  --shadow-tall-only <0|1> Only generate shadows for sprites taller than tileH (default: 1).\n"
      << "  --sdf-spread <px>    SDF spread in pixels (default: 8).\n"
      << "  --sdf-threshold <f>  Alpha threshold in [0,1] for SDF inside/outside (default: 0.5).\n"
      << "  --sdf-opaque-alpha <0|1> If 1, write SDF with A=255 everywhere (default: 1).\n"
      << "  --mip-dir <dir>      If set, write a mip chain to this directory (files named <base>_mipN.png).\n"
      << "  --mip-levels <n>     Max mip levels to write after mip0 (0 = until min-size/1x1).\n"
      << "  --mip-min-size <px>  Stop once both dimensions are <= this size (default: 1 -> down to 1x1).\n"
      << "  --mip-premultiply <0|1> Premultiply alpha during color downsampling (default: 1).\n"
      << "  --mip-alpha-coverage <0|1> Preserve alpha-cutout silhouette by scaling sprite alpha per mip (default: 0).\n"
      << "  --mip-alpha-threshold <f> Alpha threshold in [0,1] used for coverage matching (default: 0.5).\n"
      << "  --mip-alpha-iters <n> Binary search iterations for coverage matching (default: 12).\n"
      << "  --indexed <0|1>      If 1, write the main (and emissive) atlas as an indexed-color PNG (palette).\n"
      << "  --indexed-colors <n> Max palette size in [2,256] (default: 256). Index 0 is reserved for transparent.\n"
      << "  --indexed-dither <0|1> Enable Floyd–Steinberg error diffusion (default: 0).\n"
      << "  --indexed-dither-strength <f> Dither strength multiplier (default: 1.0).\n"
      << "  --indexed-alpha <0|1> If 1, alpha participates in palette matching (default: 1).\n"
      << "  --opaque <0|1>       If 1, composite alpha over a dark background and write RGB PNG.\n";
}

bool EnsureParentDir(const std::string& path)
{
  if (path.empty()) return true;
  try {
    const std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
  } catch (...) {
    return false;
  }
  return true;
}

bool EnsureDir(const std::string& dir)
{
  if (dir.empty()) return true;
  try {
    std::filesystem::create_directories(std::filesystem::path(dir));
  } catch (...) {
    return false;
  }
  return true;
}

std::string MakeMipPath(const std::string& mipDir, const std::string& basePath, int level)
{
  if (mipDir.empty() || basePath.empty()) return std::string();
  try {
    const std::filesystem::path bp(basePath);
    const std::string stem = bp.stem().string();
    const std::string ext = bp.extension().string();
    const std::string name = stem + "_mip" + std::to_string(level) + ext;
    const std::filesystem::path out = std::filesystem::path(mipDir) / std::filesystem::path(name);
    return out.string();
  } catch (...) {
  }
  return std::string();
}

std::string RelPathForHtml(const std::string& htmlPath, const std::string& assetPath)
{
  try {
    const std::filesystem::path hp(htmlPath);
    const std::filesystem::path ap(assetPath);
    const std::filesystem::path base = hp.parent_path();
    if (!base.empty()) {
      const std::filesystem::path rel = std::filesystem::relative(ap, base);
      if (!rel.empty()) return rel.generic_string();
    }
  } catch (...) {
  }
  return assetPath;
}

bool WriteHtmlPreview(const std::string& htmlPath, const std::string& atlasPng,
                      const std::string& emissivePng, const std::string& heightPng,
                      const std::string& normalPng, const std::string& shadowPng,
                      const std::string& sdfPng,
                      const std::vector<std::string>& atlasMipPngs,
                      const isocity::GfxTilesetResult& ts)
{
  if (!EnsureParentDir(htmlPath)) return false;

  std::ofstream f(htmlPath);
  if (!f) return false;

  const std::string atlasRel = RelPathForHtml(htmlPath, atlasPng);
  const std::string emiRel = emissivePng.empty() ? std::string() : RelPathForHtml(htmlPath, emissivePng);
  const std::string heightRel = heightPng.empty() ? std::string() : RelPathForHtml(htmlPath, heightPng);
  const std::string normalRel = normalPng.empty() ? std::string() : RelPathForHtml(htmlPath, normalPng);
  const std::string shadowRel = shadowPng.empty() ? std::string() : RelPathForHtml(htmlPath, shadowPng);
  const std::string sdfRel = sdfPng.empty() ? std::string() : RelPathForHtml(htmlPath, sdfPng);

  std::vector<std::string> mipRels;
  mipRels.reserve(atlasMipPngs.size());
  for (const auto& mp : atlasMipPngs) {
    mipRels.push_back(RelPathForHtml(htmlPath, mp));
  }

  f << "<!doctype html>\n";
  f << "<html><head><meta charset='utf-8'>\n";
  f << "<title>ProcIsoCity Tileset Preview</title>\n";
  f << "<style>\n";
  f << "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Noto Sans,sans-serif; margin:16px; background:#101214; color:#e6e6e6;}\n";
  f << "a{color:#9ad;}\n";
  f << ".grid{display:grid; grid-template-columns: 380px 1fr";
  if (!emiRel.empty()) f << " 1fr";
  if (!normalRel.empty()) f << " 1fr";
  if (!heightRel.empty()) f << " 1fr";
  if (!sdfRel.empty()) f << " 1fr";
  if (!shadowRel.empty()) f << " 1fr";
  f << "; gap:10px 16px; align-items:center;}\n";
  f << ".head{font-weight:700; color:#fff; padding:6px 0;}\n";
  f << ".name{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace; font-size:12px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}\n";
  f << ".spr{image-rendering:pixelated; background-repeat:no-repeat; border:1px solid #2a2f35; box-shadow:0 1px 0 rgba(0,0,0,.35); background-color:#161a1f;}\n";
  f << ".spr.shadow{background-color:#d7d7d7;}\n";
  f << ".meta{font-size:12px; opacity:0.75; margin-bottom:10px;}\n";
  f << ".mips{display:flex; flex-wrap:wrap; gap:12px; margin:12px 0 18px;}\n";
  f << ".mip{border:1px solid #2a2f35; padding:10px; background:#161a1f; border-radius:8px;}\n";
  f << ".mip .lbl{font-size:12px; opacity:0.85; margin-bottom:6px;}\n";
  f << ".mip img{image-rendering:pixelated; border:1px solid #2a2f35; background:#0f1114;}\n";
  f << "</style></head><body>\n";
  f << "<h2>ProcIsoCity Tileset Preview</h2>\n";
  f << "<div class='meta'>Atlas: <code>" << atlasRel << "</code> (" << ts.atlas.width << "x" << ts.atlas.height << ")";
  if (!emiRel.empty()) f << " &nbsp; Emissive: <code>" << emiRel << "</code>";
  if (!normalRel.empty()) f << " &nbsp; Normal: <code>" << normalRel << "</code>";
  if (!heightRel.empty()) f << " &nbsp; Height: <code>" << heightRel << "</code>";
  if (!shadowRel.empty()) f << " &nbsp; Shadow: <code>" << shadowRel << "</code>";
  if (!sdfRel.empty()) f << " &nbsp; SDF: <code>" << sdfRel << "</code>";
  f << "</div>\n";

  if (!mipRels.empty()) {
    f << "<details open><summary style='cursor:pointer'>Mipmaps (" << mipRels.size() << " levels)</summary>\n";
    f << "<div class='mips'>\n";
    for (std::size_t i = 0; i < mipRels.size(); ++i) {
      f << "<div class='mip'>";
      f << "<div class='lbl'>mip" << i << ": <code>" << mipRels[i] << "</code></div>\n";
      f << "<img src='" << mipRels[i] << "' alt='mip" << i << "'>\n";
      f << "</div>\n";
    }
    f << "</div>\n";
    f << "</details>\n";
  }

  f << "<div class='grid'>\n";
  f << "<div class='head'>Sprite</div>\n";
  f << "<div class='head'>Color</div>\n";
  if (!emiRel.empty()) f << "<div class='head'>Emissive</div>\n";
  if (!normalRel.empty()) f << "<div class='head'>Normal</div>\n";
  if (!heightRel.empty()) f << "<div class='head'>Height</div>\n";
  if (!sdfRel.empty()) f << "<div class='head'>SDF</div>\n";
  if (!shadowRel.empty()) f << "<div class='head'>Shadow</div>\n";

  for (const auto& e : ts.entries) {
    f << "<div class='name'>" << e.name << "<br><span style='opacity:.7'>x=" << e.x << " y=" << e.y
      << " w=" << e.w << " h=" << e.h << " pivot(" << e.pivotX << "," << e.pivotY << ")</span></div>\n";

    f << "<div class='spr' style='width:" << e.w << "px;height:" << e.h << "px;"
      << "background-image:url(\"" << atlasRel << "\");"
      << "background-size:" << ts.atlas.width << "px " << ts.atlas.height << "px;"
      << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";

    if (!emiRel.empty()) {
      f << "<div class='spr' style='width:" << e.w << "px;height:" << e.h << "px;"
        << "background-image:url(\"" << emiRel << "\");"
        << "background-size:" << ts.atlas.width << "px " << ts.atlas.height << "px;"
        << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";
    }

    if (!normalRel.empty()) {
      f << "<div class='spr' style='width:" << e.w << "px;height:" << e.h << "px;"
        << "background-image:url(\"" << normalRel << "\");"
        << "background-size:" << ts.atlas.width << "px " << ts.atlas.height << "px;"
        << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";
    }

    if (!heightRel.empty()) {
      f << "<div class='spr' style='width:" << e.w << "px;height:" << e.h << "px;"
        << "background-image:url(\"" << heightRel << "\");"
        << "background-size:" << ts.atlas.width << "px " << ts.atlas.height << "px;"
        << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";
    }

    if (!sdfRel.empty()) {
      f << "<div class='spr' style='width:" << e.w << "px;height:" << e.h << "px;"
        << "background-image:url(\"" << sdfRel << "\");"
        << "background-size:" << ts.atlas.width << "px " << ts.atlas.height << "px;"
        << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";
    }

    if (!shadowRel.empty()) {
      f << "<div class='spr shadow' style='width:" << e.w << "px;height:" << e.h << "px;"
        << "background-image:url(\"" << shadowRel << "\");"
        << "background-size:" << ts.atlas.width << "px " << ts.atlas.height << "px;"
        << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";
    }
  }

  f << "</div>\n";
  f << "</body></html>\n";
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  std::string outPng;
  std::string outMeta;
  std::string outEmit;
  std::string outHeight;
  std::string outNormal;
  std::string outShadow;
  std::string outSdf;
  std::string outHtml;
  std::string outOutlines;
  std::string outOutlineSvg;
  float outlineThreshold = 0.5f;
  bool outlineHull = true;
  bool outlineHoles = true;
  int outlineSvgScale = 1;
  std::uint64_t seed64 = 1;
  std::string themeStr = "classic";
  std::string heightFromStr = "alpha_luma";
  int tileW = 64;
  int tileH = 32;
  std::string packModeStr = "grid";
  int packWidth = 0;
  bool packPow2 = false;
  bool trimTransparent = false;
  int trimBorder = 1;
  int cols = 32;
  int pad = 2;
  int extrudePx = 0;
  std::string mipDir;
  int mipLevels = 0;
  int mipMinSize = 1;
  bool mipPremultiply = true;
  bool mipAlphaCoverage = false;
  float mipAlphaThreshold = 0.5f;
  int mipAlphaIters = 12;
  bool transitions = true;
  int transitionVariants = 4;
  bool buildings = false;
  int buildingVariants = 12;
  int buildingSpriteH = 0;

  bool facilities = false;
  int facilityVariants = 8;
  int facilitySpriteH = 0;

  bool props = false;
  int propVariants = 16;
  int propSpriteH = 0;
  bool vehicles = false;
  int vehicleVariants = 12;
  float normalStrength = 2.0f;
  float shadowDirX = 1.0f;
  float shadowDirY = 1.0f;
  float shadowLengthPx = 18.0f;
  int shadowBlurPx = 2;
  float shadowOpacity = 0.70f;
  bool shadowTallOnly = true;
  float sdfSpreadPx = 8.0f;
  float sdfThreshold = 0.5f;
  bool sdfOpaqueAlpha = true;
  bool indexed = false;
  int indexedColors = 256;
  bool indexedDither = false;
  float indexedDitherStrength = 1.0f;
  bool indexedAlpha = true;
  bool opaque = false;

  std::vector<std::string> args;
  args.reserve(static_cast<std::size_t>(std::max(0, argc - 1)));
  for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];

    auto need = [&](const char* name) -> const std::string& {
      if (i + 1 >= args.size()) {
        std::cerr << "Missing value for " << name << "\n";
        std::exit(2);
      }
      return args[++i];
    };

    if (a == "--help" || a == "-h") {
      PrintHelp();
      return 0;
    } else if (a == "--out") {
      outPng = need("--out");
    } else if (a == "--meta") {
      outMeta = need("--meta");
    } else if (a == "--seed") {
      if (!ParseU64(need("--seed"), &seed64)) {
        std::cerr << "Bad --seed value\n";
        return 2;
      }
    } else if (a == "--theme") {
      themeStr = need("--theme");
    } else if (a == "--tile") {
      if (!ParseWxH(need("--tile"), &tileW, &tileH)) {
        std::cerr << "Bad --tile value (expected WxH)\n";
        return 2;
      }
    } else if (a == "--pack") {
      packModeStr = need("--pack");
      // Normalize a tiny bit.
      std::transform(packModeStr.begin(), packModeStr.end(), packModeStr.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      if (packModeStr != "grid" && packModeStr != "maxrects") {
        std::cerr << "Bad --pack value (expected grid|maxrects)\n";
        return 2;
      }
    } else if (a == "--pack-width") {
      if (!ParseI32(need("--pack-width"), &packWidth) || packWidth < 0) {
        std::cerr << "Bad --pack-width value (expected >= 0)\n";
        return 2;
      }
    } else if (a == "--pow2") {
      if (!ParseBool01(need("--pow2"), &packPow2)) {
        std::cerr << "Bad --pow2 value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--trim") {
      if (!ParseBool01(need("--trim"), &trimTransparent)) {
        std::cerr << "Bad --trim value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--trim-border") {
      if (!ParseI32(need("--trim-border"), &trimBorder) || trimBorder < 0) {
        std::cerr << "Bad --trim-border value (expected >= 0)\n";
        return 2;
      }
    } else if (a == "--cols") {
      if (!ParseI32(need("--cols"), &cols) || cols <= 0) {
        std::cerr << "Bad --cols value\n";
        return 2;
      }
    } else if (a == "--pad") {
      if (!ParseI32(need("--pad"), &pad) || pad < 0) {
        std::cerr << "Bad --pad value\n";
        return 2;
      }
    } else if (a == "--extrude") {
      if (!ParseI32(need("--extrude"), &extrudePx) || extrudePx < 0) {
        std::cerr << "Bad --extrude value\n";
        return 2;
      }
    } else if (a == "--transitions") {
      if (!ParseBool01(need("--transitions"), &transitions)) {
        std::cerr << "Bad --transitions value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--transition-variants") {
      if (!ParseI32(need("--transition-variants"), &transitionVariants) || transitionVariants <= 0) {
        std::cerr << "Bad --transition-variants value\n";
        return 2;
      }
    } else if (a == "--opaque") {
      if (!ParseBool01(need("--opaque"), &opaque)) {
        std::cerr << "Bad --opaque value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--height") {
      outHeight = need("--height");
    } else if (a == "--normal") {
      outNormal = need("--normal");
    } else if (a == "--shadow") {
      outShadow = need("--shadow");
    } else if (a == "--sdf") {
      outSdf = need("--sdf");
    } else if (a == "--sdf-spread") {
      if (!ParseF32(need("--sdf-spread"), &sdfSpreadPx) || sdfSpreadPx <= 0.0f) {
        std::cerr << "Bad --sdf-spread value\n";
        return 2;
      }
    } else if (a == "--sdf-threshold") {
      if (!ParseF32(need("--sdf-threshold"), &sdfThreshold) || sdfThreshold < 0.0f || sdfThreshold > 1.0f) {
        std::cerr << "Bad --sdf-threshold value (expected 0..1)\n";
        return 2;
      }
    } else if (a == "--sdf-opaque-alpha") {
      if (!ParseBool01(need("--sdf-opaque-alpha"), &sdfOpaqueAlpha)) {
        std::cerr << "Bad --sdf-opaque-alpha value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--indexed") {
      if (!ParseBool01(need("--indexed"), &indexed)) {
        std::cerr << "Bad --indexed value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--indexed-colors") {
      if (!ParseI32(need("--indexed-colors"), &indexedColors) || indexedColors < 2 || indexedColors > 256) {
        std::cerr << "Bad --indexed-colors value (expected 2..256)\n";
        return 2;
      }
    } else if (a == "--indexed-dither") {
      if (!ParseBool01(need("--indexed-dither"), &indexedDither)) {
        std::cerr << "Bad --indexed-dither value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--indexed-dither-strength") {
      if (!ParseF32(need("--indexed-dither-strength"), &indexedDitherStrength) || indexedDitherStrength < 0.0f) {
        std::cerr << "Bad --indexed-dither-strength value (expected >= 0)\n";
        return 2;
      }
    } else if (a == "--indexed-alpha") {
      if (!ParseBool01(need("--indexed-alpha"), &indexedAlpha)) {
        std::cerr << "Bad --indexed-alpha value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--height-from") {
      heightFromStr = need("--height-from");
    } else if (a == "--normal-strength") {
      if (!ParseF32(need("--normal-strength"), &normalStrength)) {
        std::cerr << "Bad --normal-strength value\n";
        return 2;
      }
    } else if (a == "--shadow-dir") {
      if (!ParseF32Pair(need("--shadow-dir"), &shadowDirX, &shadowDirY)) {
        std::cerr << "Bad --shadow-dir value (expected dx,dy)\n";
        return 2;
      }
    } else if (a == "--shadow-length") {
      if (!ParseF32(need("--shadow-length"), &shadowLengthPx) || shadowLengthPx < 0.0f) {
        std::cerr << "Bad --shadow-length value\n";
        return 2;
      }
    } else if (a == "--shadow-blur") {
      if (!ParseI32(need("--shadow-blur"), &shadowBlurPx) || shadowBlurPx < 0) {
        std::cerr << "Bad --shadow-blur value\n";
        return 2;
      }
    } else if (a == "--shadow-opacity") {
      if (!ParseF32(need("--shadow-opacity"), &shadowOpacity)) {
        std::cerr << "Bad --shadow-opacity value\n";
        return 2;
      }
    } else if (a == "--shadow-tall-only") {
      if (!ParseBool01(need("--shadow-tall-only"), &shadowTallOnly)) {
        std::cerr << "Bad --shadow-tall-only value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--mip-dir") {
      mipDir = need("--mip-dir");
    } else if (a == "--mip-levels") {
      if (!ParseI32(need("--mip-levels"), &mipLevels) || mipLevels < 0) {
        std::cerr << "Bad --mip-levels value (expected >= 0)\n";
        return 2;
      }
    } else if (a == "--mip-min-size") {
      if (!ParseI32(need("--mip-min-size"), &mipMinSize) || mipMinSize < 1) {
        std::cerr << "Bad --mip-min-size value (expected >= 1)\n";
        return 2;
      }
    } else if (a == "--mip-premultiply") {
      if (!ParseBool01(need("--mip-premultiply"), &mipPremultiply)) {
        std::cerr << "Bad --mip-premultiply value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--mip-alpha-coverage") {
      if (!ParseBool01(need("--mip-alpha-coverage"), &mipAlphaCoverage)) {
        std::cerr << "Bad --mip-alpha-coverage value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--mip-alpha-threshold") {
      if (!ParseF32(need("--mip-alpha-threshold"), &mipAlphaThreshold) || mipAlphaThreshold < 0.0f ||
          mipAlphaThreshold > 1.0f) {
        std::cerr << "Bad --mip-alpha-threshold value (expected 0..1)\n";
        return 2;
      }
    } else if (a == "--mip-alpha-iters") {
      if (!ParseI32(need("--mip-alpha-iters"), &mipAlphaIters) || mipAlphaIters < 1) {
        std::cerr << "Bad --mip-alpha-iters value (expected >= 1)\n";
        return 2;
      }
    } else if (a == "--buildings") {
      if (!ParseBool01(need("--buildings"), &buildings)) {
        std::cerr << "Bad --buildings value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--building-variants") {
      if (!ParseI32(need("--building-variants"), &buildingVariants) || buildingVariants < 0) {
        std::cerr << "Bad --building-variants value\n";
        return 2;
      }
    } else if (a == "--building-sprite-h") {
      if (!ParseI32(need("--building-sprite-h"), &buildingSpriteH) || buildingSpriteH < 0) {
        std::cerr << "Bad --building-sprite-h value\n";
        return 2;
      }
    } else if (a == "--facilities") {
      if (!ParseBool01(need("--facilities"), &facilities)) {
        std::cerr << "Bad --facilities value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--facility-variants") {
      if (!ParseI32(need("--facility-variants"), &facilityVariants) || facilityVariants < 0) {
        std::cerr << "Bad --facility-variants value\n";
        return 2;
      }
    } else if (a == "--facility-sprite-h") {
      if (!ParseI32(need("--facility-sprite-h"), &facilitySpriteH) || facilitySpriteH < 0) {
        std::cerr << "Bad --facility-sprite-h value\n";
        return 2;
      }
    } else if (a == "--props") {
      if (!ParseBool01(need("--props"), &props)) {
        std::cerr << "Bad --props value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--prop-variants") {
      if (!ParseI32(need("--prop-variants"), &propVariants) || propVariants < 0) {
        std::cerr << "Bad --prop-variants value\n";
        return 2;
      }
    } else if (a == "--prop-sprite-h") {
      if (!ParseI32(need("--prop-sprite-h"), &propSpriteH) || propSpriteH < 0) {
        std::cerr << "Bad --prop-sprite-h value\n";
        return 2;
      }
    } else if (a == "--vehicles") {
      if (!ParseBool01(need("--vehicles"), &vehicles)) {
        std::cerr << "Bad --vehicles value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--vehicle-variants") {
      if (!ParseI32(need("--vehicle-variants"), &vehicleVariants) || vehicleVariants < 0) {
        std::cerr << "Bad --vehicle-variants value\n";
        return 2;
      }
    } else if (a == "--emit") {
      outEmit = need("--emit");
    } else if (a == "--html") {
      outHtml = need("--html");
    } else if (a == "--outlines") {
      outOutlines = need("--outlines");
    } else if (a == "--outline-svg") {
      outOutlineSvg = need("--outline-svg");
    } else if (a == "--outline-threshold") {
      if (!ParseF32(need("--outline-threshold"), &outlineThreshold) || outlineThreshold < 0.0f || outlineThreshold > 1.0f) {
        std::cerr << "Bad --outline-threshold value (expected 0..1)\n";
        return 2;
      }
    } else if (a == "--outline-hull") {
      if (!ParseBool01(need("--outline-hull"), &outlineHull)) {
        std::cerr << "Bad --outline-hull value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--outline-holes") {
      if (!ParseBool01(need("--outline-holes"), &outlineHoles)) {
        std::cerr << "Bad --outline-holes value (expected 0 or 1)\n";
        return 2;
      }
    } else if (a == "--outline-svg-scale") {
      if (!ParseI32(need("--outline-svg-scale"), &outlineSvgScale) || outlineSvgScale < 1) {
        std::cerr << "Bad --outline-svg-scale value (expected >= 1)\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown option: " << a << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (outPng.empty()) {
    std::cerr << "--out is required\n";
    PrintHelp();
    return 2;
  }

  if (opaque && indexed) {
    std::cerr << "--indexed is incompatible with --opaque 1 (indexed output preserves alpha).\n";
    return 2;
  }

  if (extrudePx > pad) {
    std::cerr << "--extrude must be <= --pad to avoid overlapping sprite writes; increase --pad.\n";
    return 2;
  }

  GfxTheme theme = GfxTheme::Classic;
  if (!ParseGfxTheme(themeStr, theme)) {
    std::cerr << "Bad --theme value: " << themeStr << "\n";
    return 2;
  }

  GfxHeightMode heightMode = GfxHeightMode::AlphaLuma;
  if (!ParseGfxHeightMode(heightFromStr, heightMode)) {
    std::cerr << "Bad --height-from value: " << heightFromStr << " (expected alpha|luma|alpha_luma)\n";
    return 2;
  }

  GfxTilesetConfig cfg;
  cfg.tileW = tileW;
  cfg.tileH = tileH;
  cfg.columns = cols;
  cfg.padding = pad;

  cfg.packMode = (packModeStr == "maxrects") ? GfxAtlasPackMode::MaxRects : GfxAtlasPackMode::Grid;
  cfg.packWidth = packWidth;
  cfg.packPow2 = packPow2;
  cfg.trimTransparent = trimTransparent;
  cfg.trimBorder = trimBorder;
  cfg.includeTransitions = transitions;
  cfg.transitionVariants = transitionVariants;
  cfg.seed = static_cast<std::uint32_t>(seed64 & 0xFFFFFFFFu);
  cfg.theme = theme;
  cfg.includeBuildings = buildings;
  cfg.buildingVariants = buildingVariants;
  cfg.buildingSpriteH = buildingSpriteH;

  cfg.includeFacilities = facilities;
  cfg.facilityVariants = facilityVariants;
  cfg.facilitySpriteH = facilitySpriteH;

  cfg.includeProps = props;
  cfg.propVariants = propVariants;
  cfg.propSpriteH = propSpriteH;
  cfg.includeVehicles = vehicles;
  cfg.vehicleVariants = vehicleVariants;
  cfg.includeEmissive = !outEmit.empty();

  cfg.includeHeight = !outHeight.empty();
  cfg.includeNormals = !outNormal.empty();
  cfg.includeShadows = !outShadow.empty();
  cfg.includeSdf = !outSdf.empty();
  cfg.heightMode = heightMode;
  cfg.normalStrength = normalStrength;
  cfg.shadow.dirX = shadowDirX;
  cfg.shadow.dirY = shadowDirY;
  cfg.shadow.lengthPx = shadowLengthPx;
  cfg.shadow.blurRadiusPx = shadowBlurPx;
  cfg.shadow.opacity = shadowOpacity;
  cfg.shadowTallSpritesOnly = shadowTallOnly;
  cfg.sdf.spreadPx = sdfSpreadPx;
  cfg.sdf.alphaThreshold = sdfThreshold;
  cfg.sdf.opaqueAlpha = sdfOpaqueAlpha;

  GfxTilesetResult ts;
  std::string err;
  if (!GenerateGfxTileset(cfg, ts, err)) {
    std::cerr << "Tileset generation failed: " << err << "\n";
    return 1;
  }

  // Optional: extrude sprite borders into transparent padding to reduce bleeding when
  // using linear sampling + mipmapping in external engines.
  if (extrudePx > 0) {
    for (const auto& e : ts.entries) {
      if (!ExtrudeSpritePadding(ts.atlas, e.x, e.y, e.w, e.h, extrudePx, err)) {
        std::cerr << "Extrude failed for atlas sprite '" << e.name << "': " << err << "\n";
        return 1;
      }

      if (!ts.emissiveAtlas.rgba.empty()) {
        if (!ExtrudeSpritePadding(ts.emissiveAtlas, e.x, e.y, e.w, e.h, extrudePx, err)) {
          std::cerr << "Extrude failed for emissive sprite '" << e.name << "': " << err << "\n";
          return 1;
        }
      }
      if (!ts.heightAtlas.rgba.empty()) {
        if (!ExtrudeSpritePadding(ts.heightAtlas, e.x, e.y, e.w, e.h, extrudePx, err)) {
          std::cerr << "Extrude failed for height sprite '" << e.name << "': " << err << "\n";
          return 1;
        }
      }
      if (!ts.normalAtlas.rgba.empty()) {
        if (!ExtrudeSpritePadding(ts.normalAtlas, e.x, e.y, e.w, e.h, extrudePx, err)) {
          std::cerr << "Extrude failed for normal sprite '" << e.name << "': " << err << "\n";
          return 1;
        }
      }
      if (!ts.shadowAtlas.rgba.empty()) {
        if (!ExtrudeSpritePadding(ts.shadowAtlas, e.x, e.y, e.w, e.h, extrudePx, err)) {
          std::cerr << "Extrude failed for shadow sprite '" << e.name << "': " << err << "\n";
          return 1;
        }
      }
      // Note: we intentionally do NOT extrude SDF, because the field is meaningful in
      // the transparent region and callers may rely on the alpha mask.
    }
  }

  std::vector<std::string> atlasMipFiles;

  if (opaque) {
    const PpmImage rgb = CompositeOverSolid(ts.atlas, 22, 22, 22);
    if (!WritePng(outPng, rgb, err)) {
      std::cerr << "Failed to write PNG: " << err << "\n";
      return 1;
    }
  } else if (indexed) {
    GfxQuantizeConfig qc;
    qc.maxColors = indexedColors;
    qc.dither = indexedDither;
    qc.ditherStrength = indexedDitherStrength;
    qc.includeAlphaInDistance = indexedAlpha;

    IndexedImage ii;
    if (!QuantizeRgbaToIndexed(ts.atlas, qc, ii, err)) {
      std::cerr << "Failed to quantize atlas: " << err << "\n";
      return 1;
    }
    if (!WritePngIndexed(outPng, ii.width, ii.height, ii.indices, ii.paletteRgba, err)) {
      std::cerr << "Failed to write indexed PNG: " << err << "\n";
      return 1;
    }
  } else {
    if (!WritePngRGBA(outPng, ts.atlas, err)) {
      std::cerr << "Failed to write PNG: " << err << "\n";
      return 1;
    }
  }

  if (!outEmit.empty()) {
    if (!EnsureParentDir(outEmit)) {
      std::cerr << "Failed to create output directory for emissive atlas\n";
      return 1;
    }
    if (indexed) {
      GfxQuantizeConfig qc;
      qc.maxColors = indexedColors;
      qc.dither = indexedDither;
      qc.ditherStrength = indexedDitherStrength;
      qc.includeAlphaInDistance = indexedAlpha;

      IndexedImage ii;
      if (!QuantizeRgbaToIndexed(ts.emissiveAtlas, qc, ii, err)) {
        std::cerr << "Failed to quantize emissive atlas: " << err << "\n";
        return 1;
      }
      if (!WritePngIndexed(outEmit, ii.width, ii.height, ii.indices, ii.paletteRgba, err)) {
        std::cerr << "Failed to write indexed emissive PNG: " << err << "\n";
        return 1;
      }
    } else {
      if (!WritePngRGBA(outEmit, ts.emissiveAtlas, err)) {
        std::cerr << "Failed to write emissive PNG: " << err << "\n";
        return 1;
      }
    }
  }

  if (!outHeight.empty()) {
    if (!EnsureParentDir(outHeight)) {
      std::cerr << "Failed to create output directory for height atlas\n";
      return 1;
    }
    if (!WritePngRGBA(outHeight, ts.heightAtlas, err)) {
      std::cerr << "Failed to write height PNG: " << err << "\n";
      return 1;
    }
  }

  if (!outNormal.empty()) {
    if (!EnsureParentDir(outNormal)) {
      std::cerr << "Failed to create output directory for normal atlas\n";
      return 1;
    }
    if (!WritePngRGBA(outNormal, ts.normalAtlas, err)) {
      std::cerr << "Failed to write normal PNG: " << err << "\n";
      return 1;
    }
  }

  if (!outShadow.empty()) {
    if (!EnsureParentDir(outShadow)) {
      std::cerr << "Failed to create output directory for shadow atlas\n";
      return 1;
    }
    if (!WritePngRGBA(outShadow, ts.shadowAtlas, err)) {
      std::cerr << "Failed to write shadow PNG: " << err << "\n";
      return 1;
    }
  }

  if (!outSdf.empty()) {
    if (!EnsureParentDir(outSdf)) {
      std::cerr << "Failed to create output directory for SDF atlas\n";
      return 1;
    }
    if (!WritePngRGBA(outSdf, ts.sdfAtlas, err)) {
      std::cerr << "Failed to write SDF PNG: " << err << "\n";
      return 1;
    }
  }

  if (!mipDir.empty()) {
    if (!EnsureDir(mipDir)) {
      std::cerr << "Failed to create --mip-dir directory: " << mipDir << "\n";
      return 1;
    }

    GfxMipmapChainConfig mc;
    mc.levels = mipLevels;
    mc.minSize = mipMinSize;
    mc.premultiplyAlpha = mipPremultiply;

    // Precompute sprite rectangles (mip0 space) for per-sprite mip alpha coverage.
    std::vector<GfxSpriteRect> spriteRects;
    spriteRects.reserve(ts.entries.size());
    for (const auto& e : ts.entries) {
      GfxSpriteRect r;
      r.x = e.x;
      r.y = e.y;
      r.w = e.w;
      r.h = e.h;
      spriteRects.push_back(r);
    }

    std::vector<float> mipAlphaTargets;
    std::vector<std::vector<float>> mipAlphaScales; // [mipLevel][spriteIndex]

    // Main atlas.
    {
      std::vector<RgbaImage> mips;
      if (!GenerateMipChainRgba(ts.atlas, mc, mips, err)) {
        std::cerr << "Failed to generate atlas mipmaps: " << err << "\n";
        return 1;
      }

      // Optional: preserve alpha-coverage per sprite across mip levels (useful for cutout sprites).
      if (mipAlphaCoverage) {
        if (!ComputeAlphaCoverageTargets(mips[0], spriteRects, mipAlphaThreshold, mipAlphaTargets, err)) {
          std::cerr << "Failed to compute mip alpha coverage targets: " << err << "\n";
          return 1;
        }

        mipAlphaScales.clear();
        mipAlphaScales.resize(mips.size());
        if (!spriteRects.empty()) mipAlphaScales[0].assign(spriteRects.size(), 1.0f);

        for (std::size_t level = 1; level < mips.size(); ++level) {
          std::vector<float> scales;
          if (!PreserveAlphaCoverageForMip(mips[level], spriteRects, mipAlphaTargets, static_cast<int>(level),
                                           mipAlphaThreshold, mipAlphaIters, &scales, err)) {
            std::cerr << "Failed to preserve mip alpha coverage (level " << level << "): " << err << "\n";
            return 1;
          }
          mipAlphaScales[level] = std::move(scales);
        }
      }

      for (std::size_t level = 0; level < mips.size(); ++level) {
        const std::string mp = MakeMipPath(mipDir, outPng, static_cast<int>(level));
        atlasMipFiles.push_back(mp);
        if (!EnsureParentDir(mp)) {
          std::cerr << "Failed to create mip output dir for " << mp << "\n";
          return 1;
        }
        if (opaque) {
          const PpmImage rgb = CompositeOverSolid(mips[level], 22, 22, 22);
          if (!WritePng(mp, rgb, err)) {
            std::cerr << "Failed to write atlas mip PNG: " << err << "\n";
            return 1;
          }
        } else if (indexed) {
          GfxQuantizeConfig qc;
          qc.maxColors = indexedColors;
          qc.dither = indexedDither;
          qc.ditherStrength = indexedDitherStrength;
          qc.includeAlphaInDistance = indexedAlpha;
          IndexedImage ii;
          if (!QuantizeRgbaToIndexed(mips[level], qc, ii, err)) {
            std::cerr << "Failed to quantize atlas mip: " << err << "\n";
            return 1;
          }
          if (!WritePngIndexed(mp, ii.width, ii.height, ii.indices, ii.paletteRgba, err)) {
            std::cerr << "Failed to write indexed atlas mip PNG: " << err << "\n";
            return 1;
          }
        } else {
          if (!WritePngRGBA(mp, mips[level], err)) {
            std::cerr << "Failed to write atlas mip PNG: " << err << "\n";
            return 1;
          }
        }
      }
    }

    // Emissive atlas.
    if (!outEmit.empty()) {
      std::vector<RgbaImage> mips;
      if (!GenerateMipChainRgba(ts.emissiveAtlas, mc, mips, err)) {
        std::cerr << "Failed to generate emissive mipmaps: " << err << "\n";
        return 1;
      }

      if (mipAlphaCoverage && !mipAlphaScales.empty()) {
        const std::size_t n = std::min(mips.size(), mipAlphaScales.size());
        for (std::size_t level = 0; level < n; ++level) {
          if (!mipAlphaScales[level].empty()) {
            if (!ApplyAlphaScalesForMip(mips[level], spriteRects, mipAlphaScales[level], static_cast<int>(level), err)) {
              std::cerr << "Failed to apply mip alpha scales to emissive atlas (level " << level << "): " << err
                        << "\n";
              return 1;
            }
          }
        }
      }

      for (std::size_t level = 0; level < mips.size(); ++level) {
        const std::string mp = MakeMipPath(mipDir, outEmit, static_cast<int>(level));
        if (!EnsureParentDir(mp)) {
          std::cerr << "Failed to create mip output dir for " << mp << "\n";
          return 1;
        }
        if (indexed) {
          GfxQuantizeConfig qc;
          qc.maxColors = indexedColors;
          qc.dither = indexedDither;
          qc.ditherStrength = indexedDitherStrength;
          qc.includeAlphaInDistance = indexedAlpha;
          IndexedImage ii;
          if (!QuantizeRgbaToIndexed(mips[level], qc, ii, err)) {
            std::cerr << "Failed to quantize emissive mip: " << err << "\n";
            return 1;
          }
          if (!WritePngIndexed(mp, ii.width, ii.height, ii.indices, ii.paletteRgba, err)) {
            std::cerr << "Failed to write indexed emissive mip PNG: " << err << "\n";
            return 1;
          }
        } else {
          if (!WritePngRGBA(mp, mips[level], err)) {
            std::cerr << "Failed to write emissive mip PNG: " << err << "\n";
            return 1;
          }
        }
      }
    }

    // Derived atlases.
    if (!outHeight.empty()) {
      std::vector<RgbaImage> mips;
      if (!GenerateMipChainRgba(ts.heightAtlas, mc, mips, err)) {
        std::cerr << "Failed to generate height mipmaps: " << err << "\n";
        return 1;
      }

      if (mipAlphaCoverage && !mipAlphaScales.empty()) {
        const std::size_t n = std::min(mips.size(), mipAlphaScales.size());
        for (std::size_t level = 0; level < n; ++level) {
          if (!mipAlphaScales[level].empty()) {
            if (!ApplyAlphaScalesForMip(mips[level], spriteRects, mipAlphaScales[level], static_cast<int>(level), err)) {
              std::cerr << "Failed to apply mip alpha scales to height atlas (level " << level << "): " << err
                        << "\n";
              return 1;
            }
          }
        }
      }

      for (std::size_t level = 0; level < mips.size(); ++level) {
        const std::string mp = MakeMipPath(mipDir, outHeight, static_cast<int>(level));
        if (!EnsureParentDir(mp)) {
          std::cerr << "Failed to create mip output dir for " << mp << "\n";
          return 1;
        }
        if (!WritePngRGBA(mp, mips[level], err)) {
          std::cerr << "Failed to write height mip PNG: " << err << "\n";
          return 1;
        }
      }
    }

    if (!outNormal.empty()) {
      std::vector<RgbaImage> mips;
      if (!GenerateMipChainNormalMap(ts.normalAtlas, mc, mips, err)) {
        std::cerr << "Failed to generate normal mipmaps: " << err << "\n";
        return 1;
      }

      if (mipAlphaCoverage && !mipAlphaScales.empty()) {
        const std::size_t n = std::min(mips.size(), mipAlphaScales.size());
        for (std::size_t level = 0; level < n; ++level) {
          if (!mipAlphaScales[level].empty()) {
            if (!ApplyAlphaScalesForMip(mips[level], spriteRects, mipAlphaScales[level], static_cast<int>(level), err)) {
              std::cerr << "Failed to apply mip alpha scales to normal atlas (level " << level << "): " << err
                        << "\n";
              return 1;
            }
          }
        }
      }

      for (std::size_t level = 0; level < mips.size(); ++level) {
        const std::string mp = MakeMipPath(mipDir, outNormal, static_cast<int>(level));
        if (!EnsureParentDir(mp)) {
          std::cerr << "Failed to create mip output dir for " << mp << "\n";
          return 1;
        }
        if (!WritePngRGBA(mp, mips[level], err)) {
          std::cerr << "Failed to write normal mip PNG: " << err << "\n";
          return 1;
        }
      }
    }

    if (!outShadow.empty()) {
      std::vector<RgbaImage> mips;
      if (!GenerateMipChainRgba(ts.shadowAtlas, mc, mips, err)) {
        std::cerr << "Failed to generate shadow mipmaps: " << err << "\n";
        return 1;
      }

      if (mipAlphaCoverage && !mipAlphaScales.empty()) {
        const std::size_t n = std::min(mips.size(), mipAlphaScales.size());
        for (std::size_t level = 0; level < n; ++level) {
          if (!mipAlphaScales[level].empty()) {
            if (!ApplyAlphaScalesForMip(mips[level], spriteRects, mipAlphaScales[level], static_cast<int>(level), err)) {
              std::cerr << "Failed to apply mip alpha scales to shadow atlas (level " << level << "): " << err
                        << "\n";
              return 1;
            }
          }
        }
      }

      for (std::size_t level = 0; level < mips.size(); ++level) {
        const std::string mp = MakeMipPath(mipDir, outShadow, static_cast<int>(level));
        if (!EnsureParentDir(mp)) {
          std::cerr << "Failed to create mip output dir for " << mp << "\n";
          return 1;
        }
        if (!WritePngRGBA(mp, mips[level], err)) {
          std::cerr << "Failed to write shadow mip PNG: " << err << "\n";
          return 1;
        }
      }
    }

    if (!outSdf.empty()) {
      GfxMipmapChainConfig sdfMc = mc;
      sdfMc.premultiplyAlpha = false; // keep the field stable even if alpha is masked

      std::vector<RgbaImage> mips;
      if (!GenerateMipChainRgba(ts.sdfAtlas, sdfMc, mips, err)) {
        std::cerr << "Failed to generate SDF mipmaps: " << err << "\n";
        return 1;
      }
      for (std::size_t level = 0; level < mips.size(); ++level) {
        const std::string mp = MakeMipPath(mipDir, outSdf, static_cast<int>(level));
        if (!EnsureParentDir(mp)) {
          std::cerr << "Failed to create mip output dir for " << mp << "\n";
          return 1;
        }
        if (!WritePngRGBA(mp, mips[level], err)) {
          std::cerr << "Failed to write SDF mip PNG: " << err << "\n";
          return 1;
        }
      }
    }
  }

  if (!outMeta.empty()) {
    if (!EnsureParentDir(outMeta)) {
      std::cerr << "Failed to create output directory for meta\n";
      return 1;
    }
    if (!WriteGfxTilesetMetaJson(outMeta, ts, err)) {
      std::cerr << "Failed to write meta JSON: " << err << "\n";
      return 1;
    }
  }

  // Optional: compute vector outlines and export JSON/SVG for external tooling.
  std::vector<GfxSpriteOutline> outlines;
  const bool wantOutlines = !outOutlines.empty() || !outOutlineSvg.empty();
  if (wantOutlines) {
    GfxOutlineConfig oc;
    oc.alphaThreshold = outlineThreshold;
    oc.computeConvexHull = outlineHull;
    oc.includeHoles = outlineHoles;

    if (!ComputeGfxTilesetOutlines(ts, oc, outlines, err)) {
      std::cerr << "Failed to compute outlines: " << err << "\n";
      return 1;
    }

    if (!outOutlines.empty()) {
      if (!EnsureParentDir(outOutlines)) {
        std::cerr << "Failed to create output directory for outlines JSON\n";
        return 1;
      }
      if (!WriteGfxTilesetOutlinesJson(outOutlines, ts, oc, outlines, err)) {
        std::cerr << "Failed to write outlines JSON: " << err << "\n";
        return 1;
      }
    }

    if (!outOutlineSvg.empty()) {
      if (!EnsureParentDir(outOutlineSvg)) {
        std::cerr << "Failed to create output directory for outline SVG\n";
        return 1;
      }
      const std::string atlasRel = RelPathForHtml(outOutlineSvg, outPng);
      if (!WriteGfxTilesetOutlinesSvg(outOutlineSvg, atlasRel, ts, outlines, outlineSvgScale, err)) {
        std::cerr << "Failed to write outline SVG: " << err << "\n";
        return 1;
      }
    }
  }

  if (!outHtml.empty()) {
    if (!WriteHtmlPreview(outHtml, outPng, outEmit, outHeight, outNormal, outShadow, outSdf, atlasMipFiles, ts)) {
      std::cerr << "Failed to write HTML preview\n";
      return 1;
    }
  }

  std::cout << "ok\n";
  return 0;
}
