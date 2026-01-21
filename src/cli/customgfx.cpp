#include "isocity/Export.hpp"
#include "isocity/GfxAtlasFx.hpp"
#include "isocity/GfxFrames.hpp"
#include "isocity/GfxOutlines.hpp"
#include "isocity/GfxPalette.hpp"
#include "isocity/GfxPatterns.hpp"
#include "isocity/GfxQuantize.hpp"
#include "isocity/GfxSigils.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

enum class Mode : std::uint8_t {
  Sigils = 0,
  Patterns = 1,
  Frames = 2,
};

const char* ModeName(Mode m)
{
  switch (m) {
  case Mode::Sigils: return "sigils";
  case Mode::Patterns: return "patterns";
  case Mode::Frames: return "frames";
  default: return "sigils";
  }
}

bool ParseMode(const std::string& s, Mode* out)
{
  if (!out) return false;
  if (s == "sigils" || s == "sigil") {
    *out = Mode::Sigils;
    return true;
  }
  if (s == "patterns" || s == "pattern") {
    *out = Mode::Patterns;
    return true;
  }
  if (s == "frames" || s == "frame" || s == "panel" || s == "panels") {
    *out = Mode::Frames;
    return true;
  }
  return false;
}

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
  const double v = std::strtod(s.c_str(), &end);
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

bool EnsureParentDir(const std::string& path)
{
  try {
    const fs::path p(path);
    if (p.has_parent_path()) {
      fs::create_directories(p.parent_path());
    }
  } catch (...) {
    return false;
  }
  return true;
}

std::string RelPathForHtml(const std::string& htmlPath, const std::string& assetPath)
{
  try {
    const fs::path hp(htmlPath);
    const fs::path ap(assetPath);
    const fs::path base = hp.parent_path();
    if (!base.empty()) {
      const fs::path rel = fs::relative(ap, base);
      if (!rel.empty()) return rel.generic_string();
    }
  } catch (...) {
  }
  return assetPath;
}

std::string SanitizeFileName(const std::string& name)
{
  std::string out;
  out.reserve(name.size());
  for (char c : name) {
    const bool ok = (c >= 'a' && c <= 'z') ||
                    (c >= 'A' && c <= 'Z') ||
                    (c >= '0' && c <= '9') ||
                    (c == '_') || (c == '-');
    out.push_back(ok ? c : '_');
  }
  if (out.empty()) out = "tile";
  return out;
}

bool ExtractSubImage(const isocity::RgbaImage& src, int x, int y, int w, int h, isocity::RgbaImage& out)
{
  if (w <= 0 || h <= 0) return false;
  if (src.width <= 0 || src.height <= 0) return false;
  if (x < 0 || y < 0) return false;
  if (x + w > src.width || y + h > src.height) return false;
  if (src.rgba.size() < static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 4u) return false;

  out.width = w;
  out.height = h;
  out.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u, std::uint8_t{0});

  for (int row = 0; row < h; ++row) {
    const std::size_t srcOff = (static_cast<std::size_t>(y + row) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 4u;
    const std::size_t dstOff = (static_cast<std::size_t>(row) * static_cast<std::size_t>(w)) * 4u;
    std::copy_n(src.rgba.data() + srcOff, static_cast<std::size_t>(w) * 4u, out.rgba.data() + dstOff);
  }

  return true;
}

bool BlitSubImage(const isocity::RgbaImage& src, int dstX, int dstY, isocity::RgbaImage& dst)
{
  if (src.width <= 0 || src.height <= 0) return false;
  if (dst.width <= 0 || dst.height <= 0) return false;
  if (dstX < 0 || dstY < 0) return false;
  if (dstX + src.width > dst.width) return false;
  if (dstY + src.height > dst.height) return false;

  const std::size_t srcNeed = static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 4u;
  const std::size_t dstNeed = static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 4u;
  if (src.rgba.size() < srcNeed) return false;
  if (dst.rgba.size() < dstNeed) return false;

  for (int row = 0; row < src.height; ++row) {
    const std::size_t srcOff = (static_cast<std::size_t>(row) * static_cast<std::size_t>(src.width)) * 4u;
    const std::size_t dstOff = (static_cast<std::size_t>(dstY + row) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(dstX)) * 4u;
    std::copy_n(src.rgba.data() + srcOff, static_cast<std::size_t>(src.width) * 4u, dst.rgba.data() + dstOff);
  }

  return true;
}

void ExtrudeTileEdges(isocity::RgbaImage& img, int x, int y, int w, int h, int px)
{
  if (px <= 0) return;
  if (w <= 0 || h <= 0) return;
  if (img.width <= 0 || img.height <= 0) return;
  if (x < 0 || y < 0) return;
  if (x + w > img.width || y + h > img.height) return;

  const std::size_t need = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u;
  if (img.rgba.size() < need) return;

  auto pxPtr = [&](int xx, int yy) -> std::uint8_t* {
    return img.rgba.data() + (static_cast<std::size_t>(yy) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(xx)) * 4u;
  };

  // 1) Horizontal extrusion for each row inside the tile.
  for (int row = 0; row < h; ++row) {
    const int yy = y + row;
    const std::uint8_t* left = pxPtr(x, yy);
    const std::uint8_t* right = pxPtr(x + w - 1, yy);

    for (int i = 1; i <= px; ++i) {
      const int lx = x - i;
      if (lx >= 0) std::copy_n(left, 4, pxPtr(lx, yy));
      const int rx = x + w - 1 + i;
      if (rx < img.width) std::copy_n(right, 4, pxPtr(rx, yy));
    }
  }

  // 2) Vertical extrusion: copy the (already horizontally extruded) top/bottom rows.
  const int leftX = std::max(0, x - px);
  const int rightX = std::min(img.width - 1, x + w - 1 + px);
  const std::size_t rowBytes = static_cast<std::size_t>(rightX - leftX + 1) * 4u;

  for (int i = 1; i <= px; ++i) {
    const int ty = y - i;
    if (ty < 0) break;
    std::copy_n(pxPtr(leftX, y), rowBytes, pxPtr(leftX, ty));
  }

  for (int i = 1; i <= px; ++i) {
    const int by = y + h - 1 + i;
    if (by >= img.height) break;
    std::copy_n(pxPtr(leftX, y + h - 1), rowBytes, pxPtr(leftX, by));
  }
}

void PrintHelp()
{
  std::cout
      << "proc_isocity_customgfx (headless procedural graphics generator)\n\n"
      << "Generates mod-friendly, deterministic custom graphics with no external art assets.\n"
      << "Currently supported modes:\n"
      << "  - sigils: circular badge icons (district/marker icons, UI symbols)\n"
      << "  - patterns: seamless square pattern tiles (UI backgrounds, overlays)\n"
      << "  - frames: UI panel/frame sprites suitable for 9-slice scaling\n\n"
      << "Usage:\n"
      << "  proc_isocity_customgfx --out <out.png> [options]\n\n"
      << "Core options:\n"
      << "  --out <path>          Output PNG path (required)\n"
      << "  --mode <name>         sigils|patterns|frames (default: sigils)\n"
      << "  --meta <path>         Optional JSON metadata output\n"
      << "  --html <path>         Optional HTML preview page\n"
      << "  --split-dir <dir>     Optional: export each entry as its own RGBA PNG into this directory\n"
      << "  --seed <u64>          Seed (default: time-based). Accepts 0x... hex.\n"
      << "  --theme <name>        Palette theme (classic/autumn/desert/noir/neon/pastel/space_colony\n"
      << "                        /procedural/procedural_muted/procedural_vibrant)\n"
      << "  --gutter <px>         Pixels between tiles in the output sheet (default: 0)\n"
      << "  --extrude <px>        Edge extrusion into the gutter (reduces atlas bleed).\n"
      << "                        Effective extrusion is clamped to <= gutter/2. (default: 0)\n"
      << "  --count <N>           Number of tiles/icons to generate (default: 32)\n"
      << "  --cols <N>            Columns in the output sheet (default: 8)\n"
      << "  --size <px>           Cell size in pixels (square) (default: 64)\n\n"
      << "Sigil options (mode=sigils):\n"
      << "  --border <px>         Border thickness (0 = auto) (default: 0)\n"
      << "  --opaque-square <0|1> If 1, fill the full square background (default: 0)\n"
      << "  --sigil-style <name>  random|blocks|starburst|chevron (default: random)\n"
      << "  --sigil-glyph <name>  random|triangle|dots|tower (default: random)\n"
      << "  --sigil-glyph-chance <f> Center glyph probability 0..1 (default: 0.85)\n\n"

      << "Frame options (mode=frames):\n"
      << "  --frame-border <px>   Border thickness (0 = auto) (default: 0)\n"
      << "  --frame-radius <px>   Rounded corner radius (0 = square) (default: 0)\n"
      << "  --frame-pattern-strength <f> Pattern blend strength in [0,1] (default: 0.35)\n"
      << "  --frame-deco <name>   random|corner_triangles|corner_dots|title_bar (default: random)\n\n"
      << "Pattern options (mode=patterns):\n"
      << "  --tileable <0|1>      If 1, edges match for seamless tiling (default: 1)\n"
      << "  --pattern-period <n>  Internal noise period (default: 32)\n"
      << "  --pattern-contrast <f> Contrast multiplier (default: 1.0)\n"
      << "  --pattern-style <name> random|grain|hatch|bricks|waves (default: random)\n\n"
      << "Indexed PNG output (smaller files):\n"
      << "  --indexed <0|1>       If 1, write color-indexed PNG (default: 0)\n"
      << "  --indexed-colors <n>  Palette size in [2,256] (default: 256)\n"
      << "  --indexed-dither <0|1> If 1, enable Floyd-Steinberg dithering (default: 0)\n"
      << "  --indexed-dither-strength <f> Dither strength (default: 1.0)\n"
      << "  --indexed-alpha <0|1> If 1, alpha participates in quantization distance (default: 1)\n\n"
      << "Optional derived outputs (RGBA PNG, same layout as the main sheet):\n"
      << "  --height <path>       Optional grayscale height map\n"
      << "  --normal <path>       Optional normal map derived from the height field\n"
      << "  --shadow <path>       Optional soft shadow mask (alpha-only)\n"
      << "  --sdf <path>          Optional signed distance field PNG\n\n"
      << "Derived map options:\n"
      << "  --height-from <mode>  alpha|luma|alpha_luma (default: alpha_luma)\n"
      << "  --normal-strength <f> Normal map strength (default: 2.0)\n"
      << "  --shadow-dir <dx,dy>  Shadow direction in pixel space (default: 1,1)\n"
      << "  --shadow-length <px>  Shadow max offset in pixels (default: 18)\n"
      << "  --shadow-blur <px>    Shadow blur radius (default: 2)\n"
      << "  --shadow-opacity <f>  Shadow opacity in [0,1] (default: 0.70)\n\n"
      << "Vector outlines (alpha silhouettes):\n"
      << "  --outlines <path>     Optional outline geometry JSON\n"
      << "  --outline-svg <path>  Optional SVG overlay preview of outlines\n"
      << "  --outline-threshold <f> Alpha threshold in [0,1] (default: 0.5)\n"
      << "  --outline-hull <0|1>  If 1, include convex hull (default: 1)\n"
      << "  --outline-holes <0|1> If 1, keep holes (default: 1)\n"
      << "  --outline-svg-scale <n> SVG scale factor (default: 1)\n\n"
      << "SDF options:\n"
      << "  --sdf-spread <px>     SDF spread in pixels (default: 8)\n"
      << "  --sdf-threshold <f>   SDF alpha threshold in [0,1] (default: 0.5)\n"
      << "  --sdf-opaque-alpha <0|1> If 1, force SDF alpha to 255 (default: 1)\n\n"
      << "  -h, --help            Show this help\n";
}

std::string JsonEscape(const std::string& s)
{
  std::ostringstream oss;
  for (char ch : s) {
    switch (ch) {
    case '\\': oss << "\\\\"; break;
    case '"': oss << "\\\""; break;
    case '\n': oss << "\\n"; break;
    case '\r': oss << "\\r"; break;
    case '\t': oss << "\\t"; break;
    default:
      if (static_cast<unsigned char>(ch) < 0x20) {
        // Encode JSON control characters as \u00XX.
        static constexpr char kHex[] = "0123456789ABCDEF";
        const unsigned char uc = static_cast<unsigned char>(ch);
        oss << "\\u00" << kHex[(uc >> 4) & 0xF] << kHex[uc & 0xF];
      } else {
        oss << ch;
      }
      break;
    }
  }
  return oss.str();
}

bool WriteMetaJson(const std::string& path, const std::vector<std::string>& names,
                   const char* modeName, int cellSize, int columns,
                   int atlasW, int atlasH, int gutterPx, int extrudePx, std::uint32_t seed,
                   const char* themeName, bool indexed, int indexedColors,
                   int frameBorderPx, int frameRadiusPx, float framePatternStrength,
                   const char* sigilStyleName, const char* sigilGlyphName, float sigilGlyphChance,
                   const char* patternStyleName, const char* frameDecoName,
                   const char* splitDir,
                   std::string& outErr)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outErr = "failed to open meta file for writing: " + path;
    return false;
  }

  const int count = static_cast<int>(names.size());
  const int rows = (count + columns - 1) / columns;

  f << "{\n";
  f << "  \"mode\": \"" << JsonEscape(modeName ? modeName : "") << "\",\n";
  f << "  \"seed\": " << seed << ",\n";
  f << "  \"theme\": \"" << JsonEscape(themeName ? themeName : "") << "\",\n";
  f << "  \"cell_size_px\": " << cellSize << ",\n";
  // Compatibility alias for early sigil JSON consumers.
  f << "  \"icon_size_px\": " << cellSize << ",\n";
  f << "  \"count\": " << count << ",\n";
  f << "  \"columns\": " << columns << ",\n";
  f << "  \"rows\": " << rows << ",\n";
  f << "  \"atlas_width_px\": " << atlasW << ",\n";
  f << "  \"atlas_height_px\": " << atlasH << ",\n";
  f << "  \"gutter_px\": " << gutterPx << ",\n";
  f << "  \"extrude_px\": " << extrudePx << ",\n";
  f << "  \"stride_px\": " << (cellSize + gutterPx) << ",\n";
  f << "  \"indexed\": " << (indexed ? "true" : "false") << ",\n";
  if (indexed) {
    f << "  \"indexed_colors\": " << indexedColors << ",\n";
  }

  if (frameBorderPx >= 0) {
    f << "  \"frame_border_px\": " << frameBorderPx << ",\n";
    f << "  \"frame_radius_px\": " << frameRadiusPx << ",\n";
    f << "  \"frame_pattern_strength\": " << framePatternStrength << ",\n";
    f << "  \"nine_slice\": {\"left\": " << frameBorderPx
      << ", \"right\": " << frameBorderPx
      << ", \"top\": " << frameBorderPx
      << ", \"bottom\": " << frameBorderPx << "},\n";
  }

  const std::string modeStr = modeName ? modeName : "";
  if (splitDir && *splitDir) {
    f << "  \"split_dir\": \"" << JsonEscape(splitDir) << "\",\n";
  }

  if (modeStr == "sigils") {
    f << "  \"sigil_style\": \"" << JsonEscape(sigilStyleName ? sigilStyleName : "") << "\",\n";
    f << "  \"sigil_glyph\": \"" << JsonEscape(sigilGlyphName ? sigilGlyphName : "") << "\",\n";
    f << "  \"sigil_glyph_chance\": " << sigilGlyphChance << ",\n";
  } else if (modeStr == "patterns") {
    f << "  \"pattern_style\": \"" << JsonEscape(patternStyleName ? patternStyleName : "") << "\",\n";
  } else if (modeStr == "frames") {
    f << "  \"frame_deco\": \"" << JsonEscape(frameDecoName ? frameDecoName : "") << "\",\n";
  }

  f << "  \"entries\": [\n";

  const int stride = cellSize + gutterPx;

  for (int i = 0; i < count; ++i) {
    const int x = (i % columns) * stride;
    const int y = (i / columns) * stride;
    f << "    {\"name\": \"" << JsonEscape(names[static_cast<std::size_t>(i)]) << "\", \"x\": " << x
      << ", \"y\": " << y << ", \"w\": " << cellSize << ", \"h\": " << cellSize << "}";
    if (i + 1 < count) f << ",";
    f << "\n";
  }

  f << "  ]\n";
  f << "}\n";
  return true;
}

bool WriteHtmlPreview(const std::string& htmlPath,
                      const std::string& sheetPng,
                      const std::string& heightPng,
                      const std::string& normalPng,
                      const std::string& shadowPng,
                      const std::string& sdfPng,
                      const isocity::RgbaImage& sheet,
                      const std::vector<isocity::GfxAtlasEntry>& entries,
                      const char* modeName,
                      std::uint32_t seed,
                      const char* themeName,
                      bool indexed,
                      int indexedColors)
{
  if (!EnsureParentDir(htmlPath)) return false;

  std::ofstream f(htmlPath);
  if (!f) return false;

  const std::string sheetRel = RelPathForHtml(htmlPath, sheetPng);
  const std::string heightRel = heightPng.empty() ? std::string() : RelPathForHtml(htmlPath, heightPng);
  const std::string normalRel = normalPng.empty() ? std::string() : RelPathForHtml(htmlPath, normalPng);
  const std::string shadowRel = shadowPng.empty() ? std::string() : RelPathForHtml(htmlPath, shadowPng);
  const std::string sdfRel = sdfPng.empty() ? std::string() : RelPathForHtml(htmlPath, sdfPng);

  f << "<!doctype html>\n";
  f << "<html><head><meta charset='utf-8'>\n";
  f << "<title>ProcIsoCity CustomGfx Preview</title>\n";
  f << "<style>\n";
  f << "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Noto Sans,sans-serif; margin:16px; background:#101214; color:#e6e6e6;}\n";
  f << "a{color:#9ad;}\n";
  f << ".grid{display:grid; grid-template-columns: 360px 1fr";
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
  f << "</style></head><body>\n";
  f << "<h2>ProcIsoCity CustomGfx Preview</h2>\n";
  f << "<div class='meta'>Sheet: <code>" << sheetRel << "</code> (" << sheet.width << "x" << sheet.height << ")";
  f << " &nbsp; Mode: <b>" << (modeName ? modeName : "") << "</b>";
  f << " &nbsp; Seed: <code>" << seed << "</code>";
  f << " &nbsp; Theme: <code>" << (themeName ? themeName : "") << "</code>";
  if (indexed) f << " &nbsp; Indexed: <b>true</b> (" << indexedColors << " colors)";
  f << "</div>\n";

  f << "<div class='grid'>\n";
  f << "<div class='head'>Entry</div>\n";
  f << "<div class='head'>Color</div>\n";
  if (!normalRel.empty()) f << "<div class='head'>Normal</div>\n";
  if (!heightRel.empty()) f << "<div class='head'>Height</div>\n";
  if (!sdfRel.empty()) f << "<div class='head'>SDF</div>\n";
  if (!shadowRel.empty()) f << "<div class='head'>Shadow</div>\n";

  for (const auto& e : entries) {
    f << "<div class='name'>" << e.name << "<br><span style='opacity:.7'>x=" << e.x << " y=" << e.y
      << " w=" << e.w << " h=" << e.h << "</span></div>\n";

    auto writeCell = [&](const std::string& rel, bool shadow) {
      if (rel.empty()) return;
      f << "<div class='spr";
      if (shadow) f << " shadow";
      f << "' style='width:" << e.w << "px;height:" << e.h << "px;"
        << "background-image:url(\"" << rel << "\");"
        << "background-size:" << sheet.width << "px " << sheet.height << "px;"
        << "background-position:-" << e.x << "px -" << e.y << "px;'></div>\n";
    };

    writeCell(sheetRel, false);
    if (!normalRel.empty()) writeCell(normalRel, false);
    if (!heightRel.empty()) writeCell(heightRel, false);
    if (!sdfRel.empty()) writeCell(sdfRel, false);
    if (!shadowRel.empty()) writeCell(shadowRel, true);
  }

  f << "</div>\n";
  f << "</body></html>\n";
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  std::string outPath;
  std::string metaPath;
  std::string htmlPath;

  std::string heightPath;
  std::string normalPath;
  std::string shadowPath;
  std::string sdfPath;

  std::string outlinesPath;
  std::string outlineSvgPath;

  std::string splitDir;

  Mode mode = Mode::Sigils;

  std::uint64_t seed64 = isocity::TimeSeed();
  int count = 32;
  int cols = 8;
  int size = 64;

  // Sprite-sheet layout.
  int gutterPx = 0;
  int extrudePx = 0;

  // Sigils.
  int border = 0;
  bool opaqueSquare = false;

  isocity::GfxSigilStyle sigilStyle = isocity::GfxSigilStyle::Random;
  isocity::GfxSigilGlyph sigilGlyph = isocity::GfxSigilGlyph::Random;
  float sigilGlyphChance = 0.85f;

  // Frames.
  int frameBorder = 0;
  int frameRadius = 0;
  float framePatternStrength = 0.35f;

  isocity::GfxFrameDeco frameDeco = isocity::GfxFrameDeco::Random;

  // Patterns.
  bool tileable = true;
  int patternPeriod = 32;
  float patternContrast = 1.0f;

  isocity::GfxPatternStyle patternStyle = isocity::GfxPatternStyle::Random;

  // Indexed output.
  bool indexed = false;
  int indexedColors = 256;
  bool indexedDither = false;
  float indexedDitherStrength = 1.0f;
  bool indexedAlpha = true;

  // Derived maps.
  isocity::GfxHeightMode heightMode = isocity::GfxHeightMode::AlphaLuma;
  float normalStrength = 2.0f;

  float shadowDirX = 1.0f;
  float shadowDirY = 1.0f;
  float shadowLengthPx = 18.0f;
  int shadowBlurPx = 2;
  float shadowOpacity = 0.70f;

  // Vector outlines.
  float outlineThreshold = 0.5f;
  bool outlineHull = true;
  bool outlineHoles = true;
  int outlineSvgScale = 1;

  // SDF.
  float sdfSpreadPx = 8.0f;
  float sdfThreshold = 0.5f;
  bool sdfOpaqueAlpha = true;

  isocity::GfxTheme theme = isocity::GfxTheme::Procedural;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];

    auto needValue = [&](const char* flag) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value after " << flag << "\n";
        std::exit(2);
      }
      return argv[++i];
    };

    if (a == "-h" || a == "--help") {
      PrintHelp();
      return 0;
    } else if (a == "--out") {
      outPath = needValue("--out");
    } else if (a == "--mode") {
      const char* v = needValue("--mode");
      if (!ParseMode(v, &mode)) {
        std::cerr << "Invalid --mode: " << v << "\n";
        return 2;
      }
    } else if (a == "--meta") {
      metaPath = needValue("--meta");
    } else if (a == "--html") {
      htmlPath = needValue("--html");
    } else if (a == "--split-dir" || a == "--split") {
      splitDir = needValue("--split-dir");
    } else if (a == "--height") {
      heightPath = needValue("--height");
    } else if (a == "--normal") {
      normalPath = needValue("--normal");
    } else if (a == "--shadow") {
      shadowPath = needValue("--shadow");
    } else if (a == "--sdf") {
      sdfPath = needValue("--sdf");
    } else if (a == "--outlines") {
      outlinesPath = needValue("--outlines");
    } else if (a == "--outline-svg") {
      outlineSvgPath = needValue("--outline-svg");
    } else if (a == "--seed") {
      const char* v = needValue("--seed");
      if (!ParseU64(v, &seed64)) {
        std::cerr << "Invalid --seed: " << v << "\n";
        return 2;
      }
    } else if (a == "--theme") {
      const char* v = needValue("--theme");
      isocity::GfxTheme t;
      if (!isocity::ParseGfxTheme(v, t)) {
        std::cerr << "Invalid --theme: " << v << "\n";
        return 2;
      }
      theme = t;
    } else if (a == "--gutter") {
      const char* v = needValue("--gutter");
      if (!ParseI32(v, &gutterPx) || gutterPx < 0) {
        std::cerr << "Invalid --gutter (expected >= 0): " << v << "\n";
        return 2;
      }
    } else if (a == "--extrude") {
      const char* v = needValue("--extrude");
      if (!ParseI32(v, &extrudePx) || extrudePx < 0) {
        std::cerr << "Invalid --extrude (expected >= 0): " << v << "\n";
        return 2;
      }
    } else if (a == "--count") {
      const char* v = needValue("--count");
      if (!ParseI32(v, &count) || count <= 0) {
        std::cerr << "Invalid --count: " << v << "\n";
        return 2;
      }
    } else if (a == "--cols") {
      const char* v = needValue("--cols");
      if (!ParseI32(v, &cols) || cols <= 0) {
        std::cerr << "Invalid --cols: " << v << "\n";
        return 2;
      }
    } else if (a == "--size") {
      const char* v = needValue("--size");
      if (!ParseI32(v, &size) || size <= 0) {
        std::cerr << "Invalid --size: " << v << "\n";
        return 2;
      }

      // Sigils.
    } else if (a == "--border") {
      const char* v = needValue("--border");
      if (!ParseI32(v, &border) || border < 0) {
        std::cerr << "Invalid --border: " << v << "\n";
        return 2;
      }
    } else if (a == "--opaque-square") {
      const char* v = needValue("--opaque-square");
      if (!ParseBool01(v, &opaqueSquare)) {
        std::cerr << "Invalid --opaque-square: " << v << "\n";
        return 2;
      }
    } else if (a == "--sigil-style") {
      const char* v = needValue("--sigil-style");
      if (!isocity::ParseGfxSigilStyle(v, sigilStyle)) {
        std::cerr << "Invalid --sigil-style: " << v << "\n";
        return 2;
      }
    } else if (a == "--sigil-glyph") {
      const char* v = needValue("--sigil-glyph");
      if (!isocity::ParseGfxSigilGlyph(v, sigilGlyph)) {
        std::cerr << "Invalid --sigil-glyph: " << v << "\n";
        return 2;
      }
    } else if (a == "--sigil-glyph-chance") {
      const char* v = needValue("--sigil-glyph-chance");
      float f = 0.0f;
      if (!ParseF32(v, &f) || !std::isfinite(f) || f < 0.0f || f > 1.0f) {
        std::cerr << "Invalid --sigil-glyph-chance (expected 0..1): " << v << "\n";
        return 2;
      }
      sigilGlyphChance = f;

      // Frames.
    } else if (a == "--frame-border") {
      const char* v = needValue("--frame-border");
      if (!ParseI32(v, &frameBorder) || frameBorder < 0) {
        std::cerr << "Invalid --frame-border: " << v << "\n";
        return 2;
      }
    } else if (a == "--frame-radius") {
      const char* v = needValue("--frame-radius");
      if (!ParseI32(v, &frameRadius) || frameRadius < 0) {
        std::cerr << "Invalid --frame-radius: " << v << "\n";
        return 2;
      }
    } else if (a == "--frame-pattern-strength") {
      const char* v = needValue("--frame-pattern-strength");
      if (!ParseF32(v, &framePatternStrength) || !std::isfinite(framePatternStrength) || framePatternStrength < 0.0f || framePatternStrength > 1.0f) {
        std::cerr << "Invalid --frame-pattern-strength (expected 0..1): " << v << "\n";
        return 2;
      }
    } else if (a == "--frame-deco") {
      const char* v = needValue("--frame-deco");
      if (!isocity::ParseGfxFrameDeco(v, frameDeco)) {
        std::cerr << "Invalid --frame-deco: " << v << "\n";
        return 2;
      }

      // Patterns.
    } else if (a == "--tileable") {
      const char* v = needValue("--tileable");
      if (!ParseBool01(v, &tileable)) {
        std::cerr << "Invalid --tileable: " << v << "\n";
        return 2;
      }
    } else if (a == "--pattern-period") {
      const char* v = needValue("--pattern-period");
      if (!ParseI32(v, &patternPeriod) || patternPeriod <= 0) {
        std::cerr << "Invalid --pattern-period: " << v << "\n";
        return 2;
      }
    } else if (a == "--pattern-contrast") {
      const char* v = needValue("--pattern-contrast");
      if (!ParseF32(v, &patternContrast) || !std::isfinite(patternContrast) || patternContrast <= 0.0f) {
        std::cerr << "Invalid --pattern-contrast: " << v << "\n";
        return 2;
      }
    } else if (a == "--pattern-style") {
      const char* v = needValue("--pattern-style");
      if (!isocity::ParseGfxPatternStyle(v, patternStyle)) {
        std::cerr << "Invalid --pattern-style: " << v << "\n";
        return 2;
      }

      // Indexed.
    } else if (a == "--indexed") {
      const char* v = needValue("--indexed");
      if (!ParseBool01(v, &indexed)) {
        std::cerr << "Invalid --indexed: " << v << "\n";
        return 2;
      }
    } else if (a == "--indexed-colors") {
      const char* v = needValue("--indexed-colors");
      if (!ParseI32(v, &indexedColors) || indexedColors < 2 || indexedColors > 256) {
        std::cerr << "Invalid --indexed-colors (expected 2..256): " << v << "\n";
        return 2;
      }
    } else if (a == "--indexed-dither") {
      const char* v = needValue("--indexed-dither");
      if (!ParseBool01(v, &indexedDither)) {
        std::cerr << "Invalid --indexed-dither: " << v << "\n";
        return 2;
      }
    } else if (a == "--indexed-dither-strength") {
      const char* v = needValue("--indexed-dither-strength");
      if (!ParseF32(v, &indexedDitherStrength) || !std::isfinite(indexedDitherStrength) || indexedDitherStrength < 0.0f) {
        std::cerr << "Invalid --indexed-dither-strength: " << v << "\n";
        return 2;
      }
    } else if (a == "--indexed-alpha") {
      const char* v = needValue("--indexed-alpha");
      if (!ParseBool01(v, &indexedAlpha)) {
        std::cerr << "Invalid --indexed-alpha: " << v << "\n";
        return 2;
      }

      // Derived maps.
    } else if (a == "--height-from") {
      const char* v = needValue("--height-from");
      isocity::GfxHeightMode hm;
      if (!isocity::ParseGfxHeightMode(v, hm)) {
        std::cerr << "Invalid --height-from: " << v << "\n";
        return 2;
      }
      heightMode = hm;
    } else if (a == "--normal-strength") {
      const char* v = needValue("--normal-strength");
      if (!ParseF32(v, &normalStrength) || !std::isfinite(normalStrength) || normalStrength <= 0.0f) {
        std::cerr << "Invalid --normal-strength: " << v << "\n";
        return 2;
      }
    } else if (a == "--shadow-dir") {
      const char* v = needValue("--shadow-dir");
      float dx = 0.0f;
      float dy = 0.0f;
      if (!ParseF32Pair(v, &dx, &dy) || (!std::isfinite(dx) || !std::isfinite(dy))) {
        std::cerr << "Invalid --shadow-dir: " << v << "\n";
        return 2;
      }
      shadowDirX = dx;
      shadowDirY = dy;
    } else if (a == "--shadow-length") {
      const char* v = needValue("--shadow-length");
      float len = 0.0f;
      if (!ParseF32(v, &len) || !std::isfinite(len) || len < 0.0f) {
        std::cerr << "Invalid --shadow-length: " << v << "\n";
        return 2;
      }
      shadowLengthPx = len;
    } else if (a == "--shadow-blur") {
      const char* v = needValue("--shadow-blur");
      if (!ParseI32(v, &shadowBlurPx) || shadowBlurPx < 0) {
        std::cerr << "Invalid --shadow-blur: " << v << "\n";
        return 2;
      }
    } else if (a == "--shadow-opacity") {
      const char* v = needValue("--shadow-opacity");
      float op = 0.0f;
      if (!ParseF32(v, &op) || !std::isfinite(op) || op < 0.0f || op > 1.0f) {
        std::cerr << "Invalid --shadow-opacity: " << v << "\n";
        return 2;
      }
      shadowOpacity = op;

      // Outlines.
    } else if (a == "--outline-threshold") {
      const char* v = needValue("--outline-threshold");
      float t = 0.0f;
      if (!ParseF32(v, &t) || !std::isfinite(t) || t < 0.0f || t > 1.0f) {
        std::cerr << "Invalid --outline-threshold: " << v << "\n";
        return 2;
      }
      outlineThreshold = t;
    } else if (a == "--outline-hull") {
      const char* v = needValue("--outline-hull");
      if (!ParseBool01(v, &outlineHull)) {
        std::cerr << "Invalid --outline-hull: " << v << "\n";
        return 2;
      }
    } else if (a == "--outline-holes") {
      const char* v = needValue("--outline-holes");
      if (!ParseBool01(v, &outlineHoles)) {
        std::cerr << "Invalid --outline-holes: " << v << "\n";
        return 2;
      }
    } else if (a == "--outline-svg-scale") {
      const char* v = needValue("--outline-svg-scale");
      if (!ParseI32(v, &outlineSvgScale) || outlineSvgScale <= 0) {
        std::cerr << "Invalid --outline-svg-scale: " << v << "\n";
        return 2;
      }

      // SDF.
    } else if (a == "--sdf-spread") {
      const char* v = needValue("--sdf-spread");
      if (!ParseF32(v, &sdfSpreadPx) || !std::isfinite(sdfSpreadPx) || sdfSpreadPx <= 0.0f) {
        std::cerr << "Invalid --sdf-spread: " << v << "\n";
        return 2;
      }
    } else if (a == "--sdf-threshold") {
      const char* v = needValue("--sdf-threshold");
      if (!ParseF32(v, &sdfThreshold) || !std::isfinite(sdfThreshold) || sdfThreshold < 0.0f || sdfThreshold > 1.0f) {
        std::cerr << "Invalid --sdf-threshold: " << v << "\n";
        return 2;
      }
    } else if (a == "--sdf-opaque-alpha") {
      const char* v = needValue("--sdf-opaque-alpha");
      if (!ParseBool01(v, &sdfOpaqueAlpha)) {
        std::cerr << "Invalid --sdf-opaque-alpha: " << v << "\n";
        return 2;
      }
    } else {
      std::cerr << "Unknown argument: " << a << "\n";
      PrintHelp();
      return 2;
    }
  }

  if (outPath.empty()) {
    std::cerr << "Missing required --out <path>\n";
    PrintHelp();
    return 2;
  }

  if (!EnsureParentDir(outPath)) {
    std::cerr << "Failed to create output directory for: " << outPath << "\n";
    return 1;
  }

  auto ensureOptDir = [&](const std::string& p) -> bool {
    return p.empty() ? true : EnsureParentDir(p);
  };

  if (!ensureOptDir(metaPath)) {
    std::cerr << "Failed to create output directory for: " << metaPath << "\n";
    return 1;
  }
  if (!ensureOptDir(htmlPath)) {
    std::cerr << "Failed to create output directory for: " << htmlPath << "\n";
    return 1;
  }
  if (!ensureOptDir(heightPath)) {
    std::cerr << "Failed to create output directory for: " << heightPath << "\n";
    return 1;
  }
  if (!ensureOptDir(normalPath)) {
    std::cerr << "Failed to create output directory for: " << normalPath << "\n";
    return 1;
  }
  if (!ensureOptDir(shadowPath)) {
    std::cerr << "Failed to create output directory for: " << shadowPath << "\n";
    return 1;
  }
  if (!ensureOptDir(sdfPath)) {
    std::cerr << "Failed to create output directory for: " << sdfPath << "\n";
    return 1;
  }
  if (!ensureOptDir(outlinesPath)) {
    std::cerr << "Failed to create output directory for: " << outlinesPath << "\n";
    return 1;
  }
  if (!ensureOptDir(outlineSvgPath)) {
    std::cerr << "Failed to create output directory for: " << outlineSvgPath << "\n";
    return 1;
  }

  if (!splitDir.empty()) {
    try {
      fs::create_directories(splitDir);
    } catch (...) {
      std::cerr << "Failed to create split output directory: " << splitDir << "\n";
      return 1;
    }
  }

  const std::uint32_t seed = static_cast<std::uint32_t>(seed64 & 0xFFFFFFFFu);
  const isocity::GfxPalette pal = isocity::GenerateGfxPalette(seed, theme);

  isocity::RgbaImage sheet;
  std::vector<std::string> names;
  std::string err;

  if (mode == Mode::Sigils) {
    isocity::GfxSigilConfig cfg;
    cfg.sizePx = size;
    cfg.borderPx = border;
    cfg.transparentOutside = !opaqueSquare;

    cfg.style = sigilStyle;
    cfg.glyph = sigilGlyph;
    cfg.glyphChance = sigilGlyphChance;

    if (!isocity::GenerateGfxSigilSheet(count, cols, seed, cfg, pal, sheet, &names, err)) {
      std::cerr << "customgfx generation failed: " << err << "\n";
      return 1;
    }
  } else if (mode == Mode::Patterns) {
    isocity::GfxPatternConfig cfg;
    cfg.sizePx = size;
    cfg.tileable = tileable;
    cfg.period = patternPeriod;
    cfg.contrast = patternContrast;

    cfg.style = patternStyle;

    if (!isocity::GenerateGfxPatternSheet(count, cols, seed, cfg, pal, sheet, &names, err)) {
      std::cerr << "customgfx generation failed: " << err << "\n";
      return 1;
    }
  } else {
    isocity::GfxFrameConfig cfg;
    cfg.sizePx = size;
    cfg.borderPx = frameBorder;
    cfg.cornerRadiusPx = frameRadius;
    cfg.patternStrength = framePatternStrength;

    cfg.deco = frameDeco;

    if (!isocity::GenerateGfxFrameSheet(count, cols, seed, cfg, pal, sheet, &names, err)) {
      std::cerr << "customgfx generation failed: " << err << "\n";
      return 1;
    }
  }

  // Optional layout adjustments for atlas friendliness.
  const int rows = (count + cols - 1) / cols;
  const int stride = size + gutterPx;
  int extrudeApplied = 0;
  if (gutterPx <= 0) {
    if (extrudePx > 0) {
      std::cerr << "Note: --extrude ignored because --gutter is 0\n";
    }
  } else {
    extrudeApplied = std::min(extrudePx, gutterPx / 2);
    if (extrudePx > 0 && extrudeApplied != extrudePx) {
      std::cerr << "Note: --extrude clamped from " << extrudePx << " to " << extrudeApplied
                << " (must be <= gutter/2)\n";
    }
  }

  if (gutterPx > 0) {
    isocity::RgbaImage spaced;
    spaced.width = cols * size + (cols - 1) * gutterPx;
    spaced.height = rows * size + (rows - 1) * gutterPx;
    spaced.rgba.assign(static_cast<std::size_t>(spaced.width) * static_cast<std::size_t>(spaced.height) * 4u, std::uint8_t{0});

    for (int i = 0; i < count; ++i) {
      const int srcX = (i % cols) * size;
      const int srcY = (i / cols) * size;
      const int dstX = (i % cols) * stride;
      const int dstY = (i / cols) * stride;

      isocity::RgbaImage sub;
      if (!ExtractSubImage(sheet, srcX, srcY, size, size, sub)) {
        std::cerr << "failed to extract source cell during gutter re-pack (i=" << i << ")\n";
        return 1;
      }
      if (!BlitSubImage(sub, dstX, dstY, spaced)) {
        std::cerr << "failed to blit destination cell during gutter re-pack (i=" << i << ")\n";
        return 1;
      }
      if (extrudeApplied > 0) {
        ExtrudeTileEdges(spaced, dstX, dstY, size, size, extrudeApplied);
      }
    }

    sheet = std::move(spaced);
  }

  // Build atlas entries (cell-aligned) for downstream exports (HTML, outlines).
  std::vector<isocity::GfxAtlasEntry> entries;
  entries.reserve(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    const int ii = static_cast<int>(i);
    isocity::GfxAtlasEntry e;
    e.name = names[i];
    e.x = (ii % cols) * stride;
    e.y = (ii / cols) * stride;
    e.w = size;
    e.h = size;
    e.pivotX = size / 2;
    e.pivotY = size / 2;
    e.srcW = size;
    e.srcH = size;
    e.trimX = 0;
    e.trimY = 0;
    entries.push_back(std::move(e));
  }

  // Write main output.
  if (indexed) {
    isocity::GfxQuantizeConfig qc;
    qc.maxColors = indexedColors;
    qc.dither = indexedDither;
    qc.ditherStrength = indexedDitherStrength;
    qc.includeAlphaInDistance = indexedAlpha;

    isocity::IndexedImage ii;
    if (!isocity::QuantizeRgbaToIndexed(sheet, qc, ii, err)) {
      std::cerr << "failed to quantize output: " << err << "\n";
      return 1;
    }
    if (!isocity::WritePngIndexed(outPath, ii.width, ii.height, ii.indices, ii.paletteRgba, err)) {
      std::cerr << "failed to write indexed PNG: " << err << "\n";
      return 1;
    }
  } else {
    if (!isocity::WritePngRGBA(outPath, sheet, err)) {
      std::cerr << "failed to write PNG: " << err << "\n";
      return 1;
    }
  }

  // Split exports.
  if (!splitDir.empty()) {
    int wrote = 0;
    for (const auto& e : entries) {
      isocity::RgbaImage sub;
      if (!ExtractSubImage(sheet, e.x, e.y, e.w, e.h, sub)) {
        std::cerr << "failed to extract sub-image for: " << e.name << "\n";
        return 1;
      }

      const std::string fileBase = SanitizeFileName(e.name);
      const fs::path p = fs::path(splitDir) / (fileBase + ".png");
      if (!isocity::WritePngRGBA(p.string(), sub, err)) {
        std::cerr << "failed to write split PNG: " << p.string() << "\n";
        return 1;
      }
      ++wrote;
    }
    std::cout << "Wrote " << wrote << " split sprites to " << splitDir << "\n";
  }

// Derived outputs.
//
// We generate derived maps (height/normal/shadow/sdf) *per sprite entry* and then reassemble
// them into a full atlas. This avoids cross-sprite "bleeding" artifacts (most noticeable for
// signed distance fields) when sprites are adjacent in the sheet.
auto initDerivedAtlas = [&](isocity::RgbaImage& dst) {
  dst.width = sheet.width;
  dst.height = sheet.height;
  dst.rgba.assign(static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 4u, std::uint8_t{0});
};

auto forEachEntry = [&](auto&& fn) -> bool {
  for (const auto& e : entries) {
    isocity::RgbaImage sub;
    if (!ExtractSubImage(sheet, e.x, e.y, e.w, e.h, sub)) {
      err = "failed to extract sub-image for: " + e.name;
      return false;
    }
    if (!fn(e, sub)) return false;
  }
  return true;
};

if (!heightPath.empty()) {
  isocity::RgbaImage height;
  initDerivedAtlas(height);

  if (!forEachEntry([&](const isocity::GfxAtlasEntry& e, const isocity::RgbaImage& sub) -> bool {
        isocity::RgbaImage h;
        std::string derr;
        if (!isocity::GenerateHeightMap(sub, heightMode, h, derr)) {
          err = "failed to generate height map for '" + e.name + "': " + derr;
          return false;
        }
        if (!BlitSubImage(h, e.x, e.y, height)) {
          err = "failed to blit height map for '" + e.name + "'";
          return false;
        }
        if (extrudeApplied > 0) {
          ExtrudeTileEdges(height, e.x, e.y, e.w, e.h, extrudeApplied);
        }
        return true;
      })) {
    std::cerr << err << "\n";
    return 1;
  }

  if (!isocity::WritePngRGBA(heightPath, height, err)) {
    std::cerr << "failed to write height PNG: " << err << "\n";
    return 1;
  }
}

if (!normalPath.empty()) {
  isocity::RgbaImage normal;
  initDerivedAtlas(normal);

  isocity::GfxNormalMapConfig nc;
  nc.heightMode = heightMode;
  nc.strength = normalStrength;

  if (!forEachEntry([&](const isocity::GfxAtlasEntry& e, const isocity::RgbaImage& sub) -> bool {
        isocity::RgbaImage n;
        std::string derr;
        if (!isocity::GenerateNormalMap(sub, nc, n, derr)) {
          err = "failed to generate normal map for '" + e.name + "': " + derr;
          return false;
        }
        if (!BlitSubImage(n, e.x, e.y, normal)) {
          err = "failed to blit normal map for '" + e.name + "'";
          return false;
        }
        if (extrudeApplied > 0) {
          ExtrudeTileEdges(normal, e.x, e.y, e.w, e.h, extrudeApplied);
        }
        return true;
      })) {
    std::cerr << err << "\n";
    return 1;
  }

  if (!isocity::WritePngRGBA(normalPath, normal, err)) {
    std::cerr << "failed to write normal PNG: " << err << "\n";
    return 1;
  }
}

if (!shadowPath.empty()) {
  isocity::RgbaImage shadow;
  initDerivedAtlas(shadow);

  isocity::GfxShadowConfig sc;
  sc.dirX = shadowDirX;
  sc.dirY = shadowDirY;
  sc.lengthPx = shadowLengthPx;
  sc.blurRadiusPx = shadowBlurPx;
  sc.opacity = shadowOpacity;

  if (!forEachEntry([&](const isocity::GfxAtlasEntry& e, const isocity::RgbaImage& sub) -> bool {
        isocity::RgbaImage s;
        std::string derr;
        if (!isocity::GenerateShadowMap(sub, sc, s, derr)) {
          err = "failed to generate shadow map for '" + e.name + "': " + derr;
          return false;
        }
        if (!BlitSubImage(s, e.x, e.y, shadow)) {
          err = "failed to blit shadow map for '" + e.name + "'";
          return false;
        }
        if (extrudeApplied > 0) {
          ExtrudeTileEdges(shadow, e.x, e.y, e.w, e.h, extrudeApplied);
        }
        return true;
      })) {
    std::cerr << err << "\n";
    return 1;
  }

  if (!isocity::WritePngRGBA(shadowPath, shadow, err)) {
    std::cerr << "failed to write shadow PNG: " << err << "\n";
    return 1;
  }
}

if (!sdfPath.empty()) {
  isocity::RgbaImage sdf;
  initDerivedAtlas(sdf);

  isocity::GfxSdfConfig sc;
  sc.spreadPx = sdfSpreadPx;
  sc.alphaThreshold = sdfThreshold;
  sc.opaqueAlpha = sdfOpaqueAlpha;

  if (!forEachEntry([&](const isocity::GfxAtlasEntry& e, const isocity::RgbaImage& sub) -> bool {
        isocity::RgbaImage s;
        std::string derr;
        if (!isocity::GenerateSignedDistanceField(sub, sc, s, derr)) {
          err = "failed to generate SDF for '" + e.name + "': " + derr;
          return false;
        }
        if (!BlitSubImage(s, e.x, e.y, sdf)) {
          err = "failed to blit SDF for '" + e.name + "'";
          return false;
        }
        if (extrudeApplied > 0) {
          ExtrudeTileEdges(sdf, e.x, e.y, e.w, e.h, extrudeApplied);
        }
        return true;
      })) {
    std::cerr << err << "\n";
    return 1;
  }

  if (!isocity::WritePngRGBA(sdfPath, sdf, err)) {
    std::cerr << "failed to write SDF PNG: " << err << "\n";
    return 1;
  }
}
  // Outlines.
  if (!outlinesPath.empty() || !outlineSvgPath.empty()) {
    isocity::GfxTilesetResult ts;
    ts.tileW = size;
    ts.tileH = size;
    ts.atlas = sheet;
    ts.entries = entries;

    isocity::GfxOutlineConfig oc;
    oc.alphaThreshold = outlineThreshold;
    oc.computeConvexHull = outlineHull;
    oc.includeHoles = outlineHoles;

    std::vector<isocity::GfxSpriteOutline> outlines;
    if (!isocity::ComputeGfxTilesetOutlines(ts, oc, outlines, err)) {
      std::cerr << "failed to compute outlines: " << err << "\n";
      return 1;
    }

    if (!outlinesPath.empty()) {
      if (!isocity::WriteGfxTilesetOutlinesJson(outlinesPath, ts, oc, outlines, err)) {
        std::cerr << "failed to write outlines JSON: " << err << "\n";
        return 1;
      }
    }

    if (!outlineSvgPath.empty()) {
      const std::string atlasHref = RelPathForHtml(outlineSvgPath, outPath);
      if (!isocity::WriteGfxTilesetOutlinesSvg(outlineSvgPath, atlasHref, ts, outlines, outlineSvgScale, err)) {
        std::cerr << "failed to write outlines SVG: " << err << "\n";
        return 1;
      }
    }
  }

  // Meta JSON.
  if (!metaPath.empty()) {
    std::string metaErr;
    const int usedFrameBorder = (mode == Mode::Frames)
                                  ? ((frameBorder > 0) ? frameBorder : (size >= 64 ? 6 : 4))
                                  : -1;
    const int usedFrameRadius = (mode == Mode::Frames) ? frameRadius : -1;
    const float usedFramePat = (mode == Mode::Frames) ? std::clamp(framePatternStrength, 0.0f, 1.0f) : -1.0f;
    if (!WriteMetaJson(metaPath, names, ModeName(mode), size, cols,
                       sheet.width, sheet.height, gutterPx, extrudeApplied, seed, isocity::GfxThemeName(theme),
                       indexed, indexedColors, usedFrameBorder, usedFrameRadius, usedFramePat,
                       isocity::GfxSigilStyleName(sigilStyle), isocity::GfxSigilGlyphName(sigilGlyph), std::clamp(sigilGlyphChance, 0.0f, 1.0f),
                       isocity::GfxPatternStyleName(patternStyle), isocity::GfxFrameDecoName(frameDeco),
                       splitDir.c_str(),
                       metaErr)) {
      std::cerr << "failed to write meta JSON: " << metaErr << "\n";
      return 1;
    }
  }

  // HTML preview.
  if (!htmlPath.empty()) {
    if (!WriteHtmlPreview(htmlPath, outPath, heightPath, normalPath, shadowPath, sdfPath, sheet, entries,
                          ModeName(mode), seed, isocity::GfxThemeName(theme), indexed, indexedColors)) {
      std::cerr << "failed to write HTML preview: " << htmlPath << "\n";
      return 1;
    }
  }

  std::cout << "Wrote " << outPath << " (" << sheet.width << "x" << sheet.height << ")\n";
  if (!heightPath.empty()) std::cout << "Wrote " << heightPath << "\n";
  if (!normalPath.empty()) std::cout << "Wrote " << normalPath << "\n";
  if (!shadowPath.empty()) std::cout << "Wrote " << shadowPath << "\n";
  if (!sdfPath.empty()) std::cout << "Wrote " << sdfPath << "\n";
  if (!metaPath.empty()) std::cout << "Wrote " << metaPath << "\n";
  if (!htmlPath.empty()) std::cout << "Wrote " << htmlPath << "\n";
  if (!outlinesPath.empty()) std::cout << "Wrote " << outlinesPath << "\n";
  if (!outlineSvgPath.empty()) std::cout << "Wrote " << outlineSvgPath << "\n";
  return 0;
}
