#include "isocity/Export.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

namespace isocity {

namespace {

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline std::uint8_t ToByte(float v)
{
  const float c = std::clamp(v, 0.0f, 255.0f);
  return static_cast<std::uint8_t>(std::lround(c));
}

inline void SetPixel(std::vector<std::uint8_t>& rgb, int w, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
  const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
  rgb[idx + 0] = r;
  rgb[idx + 1] = g;
  rgb[idx + 2] = b;
}

inline void MulPixel(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b, float m)
{
  r = ToByte(static_cast<float>(r) * m);
  g = ToByte(static_cast<float>(g) * m);
  b = ToByte(static_cast<float>(b) * m);
}

inline void TerrainBaseColor(const Tile& t, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  switch (t.terrain) {
  case Terrain::Water:
    r = 18; g = 70; b = 180;
    break;
  case Terrain::Sand:
    r = 198; g = 182; b = 120;
    break;
  case Terrain::Grass:
  default:
    r = 60; g = 170; b = 70;
    break;
  }
}

inline void OverlayColor(const Tile& t, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  switch (t.overlay) {
  case Overlay::Road: {
    // Slightly different tint for bridges.
    if (t.terrain == Terrain::Water) { r = 210; g = 210; b = 220; }
    else { r = 120; g = 120; b = 120; }

    // Road class shading (level 1..3).
    const int lvl = std::clamp(static_cast<int>(t.level), 1, 3);
    const float m = 0.85f + 0.10f * static_cast<float>(lvl - 1);
    MulPixel(r, g, b, m);
  } break;
  case Overlay::Residential:
    r = 70; g = 210; b = 90;
    break;
  case Overlay::Commercial:
    r = 70; g = 140; b = 230;
    break;
  case Overlay::Industrial:
    r = 220; g = 170; b = 70;
    break;
  case Overlay::Park:
    r = 40; g = 140; b = 60;
    break;
  case Overlay::None:
  default:
    // Keep base terrain.
    break;
  }
}

inline void HeatRampRedYellowGreen(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  // 0 -> red, 0.5 -> yellow, 1 -> green
  if (t <= 0.5f) {
    r = 255;
    g = ToByte(255.0f * (t * 2.0f));
    b = 0;
  } else {
    r = ToByte(255.0f * (1.0f - (t - 0.5f) * 2.0f));
    g = 255;
    b = 0;
  }
}

inline void HeatRampPurple(float v01, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const float t = Clamp01(v01);
  r = ToByte(80.0f + 175.0f * t);
  g = ToByte(30.0f + 70.0f * t);
  b = ToByte(90.0f + 165.0f * t);
}

inline void DistrictPalette(std::uint8_t id, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  // 8 distinct-ish colors (matches kDistrictCount).
  // Chosen to be readable on dark backgrounds and in PPM viewers.
  static constexpr std::uint8_t k[8][3] = {
      {220, 220, 220}, // 0 (default) - light gray
      {255, 120, 120}, // 1 - red
      {120, 210, 120}, // 2 - green
      {120, 160, 255}, // 3 - blue
      {255, 210, 120}, // 4 - orange
      {200, 120, 255}, // 5 - purple
      {120, 230, 230}, // 6 - cyan
      {255, 120, 220}, // 7 - pink
  };
  const std::uint8_t idx = static_cast<std::uint8_t>(id % 8u);
  r = k[idx][0]; g = k[idx][1]; b = k[idx][2];
}

inline std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

} // namespace

bool ParseExportLayer(const std::string& s, ExportLayer& outLayer)
{
  const std::string k = ToLower(s);
  if (k == "terrain") { outLayer = ExportLayer::Terrain; return true; }
  if (k == "overlay") { outLayer = ExportLayer::Overlay; return true; }
  if (k == "height" || k == "elevation") { outLayer = ExportLayer::Height; return true; }
  if (k == "landvalue" || k == "land_value" || k == "lv") { outLayer = ExportLayer::LandValue; return true; }
  if (k == "traffic" || k == "commute") { outLayer = ExportLayer::Traffic; return true; }
  if (k == "goods" || k == "goods_traffic" || k == "goodstraffic") { outLayer = ExportLayer::GoodsTraffic; return true; }
  if (k == "goods_fill" || k == "goodsfill" || k == "fill") { outLayer = ExportLayer::GoodsFill; return true; }
  if (k == "district" || k == "districts") { outLayer = ExportLayer::District; return true; }
  return false;
}

const char* ExportLayerName(ExportLayer layer)
{
  switch (layer) {
  case ExportLayer::Terrain: return "terrain";
  case ExportLayer::Overlay: return "overlay";
  case ExportLayer::Height: return "height";
  case ExportLayer::LandValue: return "landvalue";
  case ExportLayer::Traffic: return "traffic";
  case ExportLayer::GoodsTraffic: return "goods_traffic";
  case ExportLayer::GoodsFill: return "goods_fill";
  case ExportLayer::District: return "district";
  default: return "unknown";
  }
}

PpmImage RenderPpmLayer(const World& world, ExportLayer layer, const LandValueResult* landValue, const TrafficResult* traffic,
                        const GoodsResult* goods)
{
  PpmImage img{};
  img.width = world.width();
  img.height = world.height();
  if (img.width <= 0 || img.height <= 0) return img;

  img.rgb.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u, 0);

  const int w = img.width;
  const int h = img.height;

  // Precompute maxima for heatmaps when available.
  std::uint16_t maxTraffic = 0;
  if (traffic && !traffic->roadTraffic.empty()) {
    maxTraffic = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (maxTraffic == 0) {
      for (std::uint16_t v : traffic->roadTraffic) maxTraffic = std::max(maxTraffic, v);
    }
  }

  std::uint16_t maxGoodsTraffic = 0;
  if (goods && !goods->roadGoodsTraffic.empty()) {
    maxGoodsTraffic = static_cast<std::uint16_t>(std::clamp(goods->maxRoadGoodsTraffic, 0, 65535));
    if (maxGoodsTraffic == 0) {
      for (std::uint16_t v : goods->roadGoodsTraffic) maxGoodsTraffic = std::max(maxGoodsTraffic, v);
    }
  }

  auto idxOf = [&](int x, int y) -> std::size_t { return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x); };

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);

      std::uint8_t r = 0, g = 0, b = 0;
      TerrainBaseColor(t, r, g, b);

      // Height shading for terrain-ish layers.
      const float shade = 0.72f + 0.28f * Clamp01(t.height);

      switch (layer) {
      case ExportLayer::Terrain: {
        MulPixel(r, g, b, shade);
      } break;

      case ExportLayer::Overlay: {
        MulPixel(r, g, b, shade);
        std::uint8_t orr = r, org = g, orb = b;
        OverlayColor(t, orr, org, orb);
        // If overlay == None, OverlayColor leaves it as base. If it's a real overlay, overwrite.
        if (t.overlay != Overlay::None) {
          r = orr; g = org; b = orb;
        }
      } break;

      case ExportLayer::Height: {
        const std::uint8_t c = ToByte(255.0f * Clamp01(t.height));
        r = c; g = c; b = c;
      } break;

      case ExportLayer::LandValue: {
        if (landValue && landValue->w == w && landValue->h == h && !landValue->value.empty()) {
          const float v = landValue->value[idxOf(x, y)];
          HeatRampRedYellowGreen(v, r, g, b);
        } else {
          // Fallback: terrain with height shading.
          MulPixel(r, g, b, shade);
        }
      } break;

      case ExportLayer::Traffic: {
        // Background: terrain.
        MulPixel(r, g, b, shade);
        if (traffic && traffic->roadTraffic.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h) && t.overlay == Overlay::Road) {
          const std::uint16_t v = traffic->roadTraffic[idxOf(x, y)];
          const float t01 = (maxTraffic > 0) ? Clamp01(static_cast<float>(v) / static_cast<float>(maxTraffic)) : 0.0f;
          std::uint8_t hr, hg, hb;
          HeatRampRedYellowGreen(t01, hr, hg, hb);
          // Blend a bit so roads still show their context.
          r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
          g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
          b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
        }
      } break;

      case ExportLayer::GoodsTraffic: {
        MulPixel(r, g, b, shade);
        if (goods && goods->roadGoodsTraffic.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h) && t.overlay == Overlay::Road) {
          const std::uint16_t v = goods->roadGoodsTraffic[idxOf(x, y)];
          const float t01 = (maxGoodsTraffic > 0) ? Clamp01(static_cast<float>(v) / static_cast<float>(maxGoodsTraffic)) : 0.0f;
          std::uint8_t hr, hg, hb;
          HeatRampPurple(t01, hr, hg, hb);
          r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
          g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
          b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
        }
      } break;

      case ExportLayer::GoodsFill: {
        MulPixel(r, g, b, shade);
        if (goods && goods->commercialFill.size() == static_cast<std::size_t>(w) * static_cast<std::size_t>(h) && t.overlay == Overlay::Commercial) {
          const std::uint8_t fill = goods->commercialFill[idxOf(x, y)];
          const float t01 = Clamp01(static_cast<float>(fill) / 255.0f);
          HeatRampRedYellowGreen(t01, r, g, b);
        }
      } break;

      case ExportLayer::District: {
        DistrictPalette(t.district, r, g, b);
        // Darken water a bit so coastlines pop.
        if (t.terrain == Terrain::Water) {
          MulPixel(r, g, b, 0.7f);
        }
      } break;

      default:
        break;
      }

      SetPixel(img.rgb, w, x, y, r, g, b);
    }
  }

  return img;
}

PpmImage ScaleNearest(const PpmImage& src, int factor)
{
  if (factor <= 1) return src;
  if (src.width <= 0 || src.height <= 0) return src;
  if (src.rgb.size() != static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 3u) return src;

  PpmImage out{};
  out.width = src.width * factor;
  out.height = src.height * factor;
  out.rgb.resize(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 3u);

  for (int y = 0; y < out.height; ++y) {
    const int sy = y / factor;
    for (int x = 0; x < out.width; ++x) {
      const int sx = x / factor;
      const std::size_t sidx = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(sx)) * 3u;
      const std::size_t didx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x)) * 3u;
      out.rgb[didx + 0] = src.rgb[sidx + 0];
      out.rgb[didx + 1] = src.rgb[sidx + 1];
      out.rgb[didx + 2] = src.rgb[sidx + 2];
    }
  }

  return out;
}

bool WritePpm(const std::string& path, const PpmImage& img, std::string& outError)
{
  outError.clear();

  if (img.width <= 0 || img.height <= 0) {
    outError = "Invalid image dimensions";
    return false;
  }
  const std::size_t expected = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u;
  if (img.rgb.size() != expected) {
    std::ostringstream oss;
    oss << "Invalid image buffer size (expected " << expected << ", got " << img.rgb.size() << ")";
    outError = oss.str();
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for writing";
    return false;
  }

  f << "P6\n" << img.width << " " << img.height << "\n255\n";
  f.write(reinterpret_cast<const char*>(img.rgb.data()), static_cast<std::streamsize>(img.rgb.size()));
  if (!f) {
    outError = "Failed while writing file";
    return false;
  }
  return true;
}

} // namespace isocity
