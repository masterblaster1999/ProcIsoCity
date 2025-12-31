#include "isocity/Export.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
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

inline std::size_t FlatIdx(int x, int y, int w)
{
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
}

struct TileColorContext {
  int w = 0;
  int h = 0;

  const LandValueResult* landValue = nullptr;
  const TrafficResult* traffic = nullptr;
  const GoodsResult* goods = nullptr;

  std::uint16_t maxTraffic = 0;
  std::uint16_t maxGoodsTraffic = 0;
};

inline TileColorContext MakeTileColorContext(const World& world, const LandValueResult* landValue, const TrafficResult* traffic,
                                            const GoodsResult* goods)
{
  TileColorContext ctx{};
  ctx.w = world.width();
  ctx.h = world.height();
  ctx.landValue = landValue;
  ctx.traffic = traffic;
  ctx.goods = goods;

  // Precompute maxima for heatmaps when available.
  if (traffic && !traffic->roadTraffic.empty()) {
    ctx.maxTraffic = static_cast<std::uint16_t>(std::clamp(traffic->maxTraffic, 0, 65535));
    if (ctx.maxTraffic == 0) {
      for (std::uint16_t v : traffic->roadTraffic) ctx.maxTraffic = std::max(ctx.maxTraffic, v);
    }
  }

  if (goods && !goods->roadGoodsTraffic.empty()) {
    ctx.maxGoodsTraffic = static_cast<std::uint16_t>(std::clamp(goods->maxRoadGoodsTraffic, 0, 65535));
    if (ctx.maxGoodsTraffic == 0) {
      for (std::uint16_t v : goods->roadGoodsTraffic) ctx.maxGoodsTraffic = std::max(ctx.maxGoodsTraffic, v);
    }
  }

  return ctx;
}

inline void ComputeTileColor(const World& world, int x, int y, ExportLayer layer, const TileColorContext& ctx,
                             std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
  const Tile& t = world.at(x, y);

  r = 0;
  g = 0;
  b = 0;
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
    if (ctx.landValue && ctx.landValue->w == ctx.w && ctx.landValue->h == ctx.h && !ctx.landValue->value.empty()) {
      const float v = ctx.landValue->value[FlatIdx(x, y, ctx.w)];
      HeatRampRedYellowGreen(v, r, g, b);
    } else {
      // Fallback: terrain with height shading.
      MulPixel(r, g, b, shade);
    }
  } break;

  case ExportLayer::Traffic: {
    // Background: terrain.
    MulPixel(r, g, b, shade);
    if (ctx.traffic && ctx.traffic->roadTraffic.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Road) {
      const std::uint16_t v = ctx.traffic->roadTraffic[FlatIdx(x, y, ctx.w)];
      const float t01 = (ctx.maxTraffic > 0) ? Clamp01(static_cast<float>(v) / static_cast<float>(ctx.maxTraffic)) : 0.0f;
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
    if (ctx.goods && ctx.goods->roadGoodsTraffic.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Road) {
      const std::uint16_t v = ctx.goods->roadGoodsTraffic[FlatIdx(x, y, ctx.w)];
      const float t01 = (ctx.maxGoodsTraffic > 0) ? Clamp01(static_cast<float>(v) / static_cast<float>(ctx.maxGoodsTraffic)) : 0.0f;
      std::uint8_t hr, hg, hb;
      HeatRampPurple(t01, hr, hg, hb);
      r = static_cast<std::uint8_t>((static_cast<int>(r) + static_cast<int>(hr) * 2) / 3);
      g = static_cast<std::uint8_t>((static_cast<int>(g) + static_cast<int>(hg) * 2) / 3);
      b = static_cast<std::uint8_t>((static_cast<int>(b) + static_cast<int>(hb) * 2) / 3);
    }
  } break;

  case ExportLayer::GoodsFill: {
    MulPixel(r, g, b, shade);
    if (ctx.goods && ctx.goods->commercialFill.size() == static_cast<std::size_t>(ctx.w) * static_cast<std::size_t>(ctx.h) &&
        t.overlay == Overlay::Commercial) {
      const std::uint8_t fill = ctx.goods->commercialFill[FlatIdx(x, y, ctx.w)];
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
}

// Convert normalized tile height -> pixel elevation (clamped).
inline int HeightToPx(float h01, int heightScalePx)
{
  if (heightScalePx <= 0) return 0;
  return static_cast<int>(std::lround(Clamp01(h01) * static_cast<float>(heightScalePx)));
}

struct Ipt {
  int x = 0;
  int y = 0;
};

inline Ipt IsoCenter(int tx, int ty, int halfW, int halfH, int heightPx)
{
  return Ipt{(tx - ty) * halfW, (tx + ty) * halfH - heightPx};
}

inline int EdgeFn(const Ipt& a, const Ipt& b, int px, int py)
{
  // 2D cross product (b-a) x (p-a)
  return (px - a.x) * (b.y - a.y) - (py - a.y) * (b.x - a.x);
}

inline void FillTriangle(PpmImage& img, const Ipt& a, const Ipt& b, const Ipt& c, std::uint8_t r, std::uint8_t g, std::uint8_t bl)
{
  if (img.width <= 0 || img.height <= 0) return;

  const int minX = std::max(0, std::min({a.x, b.x, c.x}));
  const int maxX = std::min(img.width - 1, std::max({a.x, b.x, c.x}));
  const int minY = std::max(0, std::min({a.y, b.y, c.y}));
  const int maxY = std::min(img.height - 1, std::max({a.y, b.y, c.y}));

  // Degenerate triangle.
  if (minX > maxX || minY > maxY) return;

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      const int w0 = EdgeFn(b, c, x, y);
      const int w1 = EdgeFn(c, a, x, y);
      const int w2 = EdgeFn(a, b, x, y);

      const bool hasNeg = (w0 < 0) || (w1 < 0) || (w2 < 0);
      const bool hasPos = (w0 > 0) || (w1 > 0) || (w2 > 0);
      if (hasNeg && hasPos) continue;

      SetPixel(img.rgb, img.width, x, y, r, g, bl);
    }
  }
}

inline void FillQuad(PpmImage& img, const Ipt& a, const Ipt& b, const Ipt& c, const Ipt& d, std::uint8_t r, std::uint8_t g,
                     std::uint8_t bl)
{
  // Split into two triangles (a,b,c) and (a,c,d)
  FillTriangle(img, a, b, c, r, g, bl);
  FillTriangle(img, a, c, d, r, g, bl);
}

inline void DrawLine(PpmImage& img, int x0, int y0, int x1, int y1, std::uint8_t r, std::uint8_t g, std::uint8_t bl)
{
  if (img.width <= 0 || img.height <= 0) return;

  // Bresenham
  int dx = std::abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  for (;;) {
    if (x0 >= 0 && x0 < img.width && y0 >= 0 && y0 < img.height) {
      SetPixel(img.rgb, img.width, x0, y0, r, g, bl);
    }
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
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

  const TileColorContext ctx = MakeTileColorContext(world, landValue, traffic, goods);

  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      std::uint8_t r, g, b;
      ComputeTileColor(world, x, y, layer, ctx, r, g, b);
      SetPixel(img.rgb, img.width, x, y, r, g, b);
    }
  }

  return img;
}

IsoOverviewResult RenderIsoOverview(const World& world, ExportLayer layer, const IsoOverviewConfig& cfg,
                                   const LandValueResult* landValue, const TrafficResult* traffic, const GoodsResult* goods)
{
  IsoOverviewResult out{};
  out.tileW = cfg.tileW;
  out.tileH = cfg.tileH;
  out.heightScalePx = std::max(0, cfg.heightScalePx);

  const int mapW = world.width();
  const int mapH = world.height();
  if (mapW <= 0 || mapH <= 0) return out;

  if (cfg.tileW <= 0 || cfg.tileH <= 0) return out;
  if ((cfg.tileW % 2) != 0 || (cfg.tileH % 2) != 0) return out;

  out.halfW = cfg.tileW / 2;
  out.halfH = cfg.tileH / 2;

  // Compute bounds in iso-space.
  int minX = std::numeric_limits<int>::max();
  int minY = std::numeric_limits<int>::max();
  int maxX = std::numeric_limits<int>::min();
  int maxY = std::numeric_limits<int>::min();

  for (int y = 0; y < mapH; ++y) {
    for (int x = 0; x < mapW; ++x) {
      const int hp = HeightToPx(world.at(x, y).height, out.heightScalePx);
      const Ipt c = IsoCenter(x, y, out.halfW, out.halfH, hp);

      const int leftX = c.x - out.halfW;
      const int rightX = c.x + out.halfW;
      const int topY = c.y - out.halfH;
      const int bottomY = c.y + out.halfH;

      minX = std::min(minX, leftX);
      maxX = std::max(maxX, rightX);
      minY = std::min(minY, topY);

      // Faces can extend below the tile by up to hp pixels.
      maxY = std::max(maxY, bottomY + hp);
    }
  }

  const int margin = std::max(0, cfg.marginPx);
  out.offsetX = -minX + margin;
  out.offsetY = -minY + margin;

  out.image.width = (maxX - minX + 1) + margin * 2 + 1;
  out.image.height = (maxY - minY + 1) + margin * 2 + 1;

  if (out.image.width <= 0 || out.image.height <= 0) return out;

  out.image.rgb.resize(static_cast<std::size_t>(out.image.width) * static_cast<std::size_t>(out.image.height) * 3u, 0);

  // Fill background.
  for (int y = 0; y < out.image.height; ++y) {
    for (int x = 0; x < out.image.width; ++x) {
      SetPixel(out.image.rgb, out.image.width, x, y, cfg.bgR, cfg.bgG, cfg.bgB);
    }
  }

  const TileColorContext ctx = MakeTileColorContext(world, landValue, traffic, goods);

  // Draw order: back-to-front along diagonals (increasing x+y).
  for (int sum = 0; sum <= (mapW - 1) + (mapH - 1); ++sum) {
    for (int x = 0; x < mapW; ++x) {
      const int y = sum - x;
      if (y < 0 || y >= mapH) continue;

      const Tile& t = world.at(x, y);
      const int hp = HeightToPx(t.height, out.heightScalePx);

      const Ipt cIso = IsoCenter(x, y, out.halfW, out.halfH, hp);
      const int cx = cIso.x + out.offsetX;
      const int cy = cIso.y + out.offsetY;

      const Ipt top{cx, cy - out.halfH};
      const Ipt right{cx + out.halfW, cy};
      const Ipt bottom{cx, cy + out.halfH};
      const Ipt left{cx - out.halfW, cy};

      std::uint8_t r, g, b;
      ComputeTileColor(world, x, y, layer, ctx, r, g, b);

      // Simple lighting: slightly brighten high tiles.
      const float heightLight = 0.90f + 0.10f * Clamp01(t.height);
      MulPixel(r, g, b, heightLight);

      // Vertical faces for height differences (optional).
      if (cfg.drawCliffs && out.heightScalePx > 0) {
        // East neighbor (x+1,y) corresponds to the right edge.
        const int hpE = (x + 1 < mapW) ? HeightToPx(world.at(x + 1, y).height, out.heightScalePx) : 0;
        const int dhR = hp - hpE;
        if (dhR > 0) {
          std::uint8_t fr = r, fg = g, fb = b;
          MulPixel(fr, fg, fb, 0.65f);
          FillQuad(out.image, right, bottom, Ipt{bottom.x, bottom.y + dhR}, Ipt{right.x, right.y + dhR}, fr, fg, fb);
        }

        // South neighbor (x,y+1) corresponds to the left edge.
        const int hpS = (y + 1 < mapH) ? HeightToPx(world.at(x, y + 1).height, out.heightScalePx) : 0;
        const int dhL = hp - hpS;
        if (dhL > 0) {
          std::uint8_t fr = r, fg = g, fb = b;
          MulPixel(fr, fg, fb, 0.55f);
          FillQuad(out.image, bottom, left, Ipt{left.x, left.y + dhL}, Ipt{bottom.x, bottom.y + dhL}, fr, fg, fb);
        }
      }

      // Top diamond (two triangles).
      FillTriangle(out.image, top, right, bottom, r, g, b);
      FillTriangle(out.image, top, bottom, left, r, g, b);

      if (cfg.drawGrid) {
        // Dark outline to help depth perception.
        const std::uint8_t lr = 25, lg = 25, lb = 25;
        DrawLine(out.image, top.x, top.y, right.x, right.y, lr, lg, lb);
        DrawLine(out.image, right.x, right.y, bottom.x, bottom.y, lr, lg, lb);
        DrawLine(out.image, bottom.x, bottom.y, left.x, left.y, lr, lg, lb);
        DrawLine(out.image, left.x, left.y, top.x, top.y, lr, lg, lb);
      }
    }
  }

  return out;
}

bool IsoTileCenterToPixel(const World& world, const IsoOverviewResult& iso, int tx, int ty, int& outPx, int& outPy)
{
  if (!world.inBounds(tx, ty)) return false;
  if (iso.halfW <= 0 || iso.halfH <= 0) return false;

  const int hp = HeightToPx(world.at(tx, ty).height, iso.heightScalePx);
  outPx = (tx - ty) * iso.halfW + iso.offsetX;
  outPy = (tx + ty) * iso.halfH - hp + iso.offsetY;
  return true;
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

namespace {

static bool ReadPpmToken(std::istream& in, std::string& out)
{
  out.clear();

  char c = 0;
  // Skip whitespace and comments.
  while (in.get(c)) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isspace(uc)) continue;
    if (c == '#') {
      std::string dummy;
      std::getline(in, dummy);
      continue;
    }
    out.push_back(c);
    break;
  }

  if (out.empty()) return false;

  // Read until next whitespace (or comment start).
  while (in.get(c)) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (std::isspace(uc)) break;
    if (c == '#') {
      std::string dummy;
      std::getline(in, dummy);
      break;
    }
    out.push_back(c);
  }

  return true;
}

static bool ParseI32Token(const std::string& tok, int& out)
{
  if (tok.empty()) return false;
  std::istringstream iss(tok);
  int v = 0;
  iss >> v;
  if (!iss || !iss.eof()) return false;
  out = v;
  return true;
}

} // namespace

bool ReadPpm(const std::string& path, PpmImage& outImg, std::string& outError)
{
  outError.clear();
  outImg = PpmImage{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for reading";
    return false;
  }

  std::string tok;
  if (!ReadPpmToken(f, tok) || tok != "P6") {
    outError = "Invalid PPM magic (expected P6)";
    return false;
  }

  int w = 0, h = 0, maxv = 0;
  if (!ReadPpmToken(f, tok) || !ParseI32Token(tok, w) || w <= 0) {
    outError = "Invalid PPM width";
    return false;
  }
  if (!ReadPpmToken(f, tok) || !ParseI32Token(tok, h) || h <= 0) {
    outError = "Invalid PPM height";
    return false;
  }
  if (!ReadPpmToken(f, tok) || !ParseI32Token(tok, maxv) || maxv <= 0) {
    outError = "Invalid PPM maxval";
    return false;
  }
  if (maxv > 255) {
    outError = "Unsupported PPM maxval (>255)";
    return false;
  }

  const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u;
  std::vector<std::uint8_t> buf(expected);

  f.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
  if (!f || static_cast<std::size_t>(f.gcount()) != expected) {
    outError = "Failed while reading pixel data";
    return false;
  }

  // Scale to 0..255 if maxval != 255.
  if (maxv != 255) {
    for (std::uint8_t& c : buf) {
      const int v = static_cast<int>(c);
      const int scaled = (v * 255 + maxv / 2) / maxv;
      c = static_cast<std::uint8_t>(std::clamp(scaled, 0, 255));
    }
  }

  outImg.width = w;
  outImg.height = h;
  outImg.rgb = std::move(buf);
  return true;
}

bool ComparePpm(const PpmImage& a, const PpmImage& b, PpmDiffStats& outStats, int threshold, PpmImage* outDiff)
{
  outStats = PpmDiffStats{};

  if (a.width <= 0 || a.height <= 0 || b.width <= 0 || b.height <= 0) return false;
  if (a.width != b.width || a.height != b.height) return false;

  const std::size_t expected = static_cast<std::size_t>(a.width) * static_cast<std::size_t>(a.height) * 3u;
  if (a.rgb.size() != expected || b.rgb.size() != expected) return false;

  const int thr = std::clamp(threshold, 0, 255);

  outStats.width = a.width;
  outStats.height = a.height;
  outStats.pixelsCompared = static_cast<std::uint64_t>(a.width) * static_cast<std::uint64_t>(a.height);

  if (outDiff) {
    outDiff->width = a.width;
    outDiff->height = a.height;
    outDiff->rgb.assign(expected, 0u);
  }

  double sumAbs = 0.0;
  double sumSq = 0.0;
  std::uint8_t maxAbs = 0;

  // Per-pixel compare (track pixelsDifferent with threshold).
  const int w = a.width;
  const int h = a.height;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;

      const int dr = std::abs(static_cast<int>(a.rgb[idx + 0]) - static_cast<int>(b.rgb[idx + 0]));
      const int dg = std::abs(static_cast<int>(a.rgb[idx + 1]) - static_cast<int>(b.rgb[idx + 1]));
      const int db = std::abs(static_cast<int>(a.rgb[idx + 2]) - static_cast<int>(b.rgb[idx + 2]));

      maxAbs = static_cast<std::uint8_t>(std::max<int>(static_cast<int>(maxAbs), std::max({dr, dg, db})));

      sumAbs += static_cast<double>(dr + dg + db);
      sumSq += static_cast<double>(dr * dr + dg * dg + db * db);

      const bool diff = (dr > thr) || (dg > thr) || (db > thr);
      if (diff) outStats.pixelsDifferent++;

      if (outDiff) {
        (*outDiff).rgb[idx + 0] = static_cast<std::uint8_t>((dr > thr) ? dr : 0);
        (*outDiff).rgb[idx + 1] = static_cast<std::uint8_t>((dg > thr) ? dg : 0);
        (*outDiff).rgb[idx + 2] = static_cast<std::uint8_t>((db > thr) ? db : 0);
      }
    }
  }

  outStats.maxAbsDiff = maxAbs;

  const double denom = static_cast<double>(outStats.pixelsCompared) * 3.0;
  if (denom > 0.0) {
    outStats.meanAbsDiff = sumAbs / denom;
    outStats.mse = sumSq / denom;
  }

  if (outStats.mse <= 0.0) {
    outStats.psnr = std::numeric_limits<double>::infinity();
  } else {
    const double peak = 255.0;
    outStats.psnr = 10.0 * std::log10((peak * peak) / outStats.mse);
  }

  return true;
}



bool WriteTilesCsv(const World& world, const std::string& path, std::string& outError)
{
  outError.clear();

  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    outError = "Invalid world dimensions";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open file for writing";
    return false;
  }

  f << "x,y,terrain,overlay,level,district,height,variation,occupants\n";
  f << std::fixed << std::setprecision(6);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      f << x << ',' << y << ',' << ToString(t.terrain) << ',' << ToString(t.overlay) << ','
        << static_cast<int>(t.level) << ',' << static_cast<int>(t.district) << ','
        << static_cast<double>(t.height) << ','
        << static_cast<int>(t.variation) << ','
        << static_cast<int>(t.occupants) << '\n';
    }
  }

  if (!f) {
    outError = "Failed while writing file";
    return false;
  }
  return true;
}

} // namespace isocity
