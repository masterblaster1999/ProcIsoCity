#include "isocity/Cartography.hpp"

#include "isocity/GfxCanvas.hpp"
#include "isocity/GfxText.hpp"
#include "isocity/Random.hpp"
#include "isocity/StreetNames.hpp"
#include "isocity/World.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>

namespace isocity {
namespace {

using gfx::BlendMode;

struct RectI {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0; // inclusive
  int y1 = 0; // inclusive
};

inline bool Overlaps(const RectI& a, const RectI& b)
{
  if (a.x1 < b.x0 || b.x1 < a.x0) return false;
  if (a.y1 < b.y0 || b.y1 < a.y0) return false;
  return true;
}

RgbaImage PpmToRgba(const PpmImage& src)
{
  RgbaImage out{};
  out.width = src.width;
  out.height = src.height;
  if (src.width <= 0 || src.height <= 0) return out;
  if (src.rgb.size() != static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 3u) return out;
  out.rgba.resize(static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 4u);
  for (int y = 0; y < src.height; ++y) {
    for (int x = 0; x < src.width; ++x) {
      const std::size_t si = (static_cast<std::size_t>(y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 3u;
      const std::size_t di = (static_cast<std::size_t>(y) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(x)) * 4u;
      out.rgba[di + 0] = src.rgb[si + 0];
      out.rgba[di + 1] = src.rgb[si + 1];
      out.rgba[di + 2] = src.rgb[si + 2];
      out.rgba[di + 3] = 255;
    }
  }
  return out;
}

void FillSolid(RgbaImage& img, std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255)
{
  if (img.width <= 0 || img.height <= 0) return;
  img.rgba.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u);
  for (int y = 0; y < img.height; ++y) {
    for (int x = 0; x < img.width; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
      img.rgba[i + 0] = r;
      img.rgba[i + 1] = g;
      img.rgba[i + 2] = b;
      img.rgba[i + 3] = a;
    }
  }
}

void BlitOpaque(RgbaImage& dst, const RgbaImage& src, int dstX, int dstY)
{
  if (dst.width <= 0 || dst.height <= 0) return;
  if (src.width <= 0 || src.height <= 0) return;
  if (dst.rgba.size() != static_cast<std::size_t>(dst.width) * static_cast<std::size_t>(dst.height) * 4u) return;
  if (src.rgba.size() != static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * 4u) return;

  for (int sy = 0; sy < src.height; ++sy) {
    const int dy = dstY + sy;
    if (dy < 0 || dy >= dst.height) continue;
    for (int sx = 0; sx < src.width; ++sx) {
      const int dx = dstX + sx;
      if (dx < 0 || dx >= dst.width) continue;
      const std::size_t si = (static_cast<std::size_t>(sy) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(sx)) * 4u;
      const std::size_t di = (static_cast<std::size_t>(dy) * static_cast<std::size_t>(dst.width) + static_cast<std::size_t>(dx)) * 4u;
      dst.rgba[di + 0] = src.rgba[si + 0];
      dst.rgba[di + 1] = src.rgba[si + 1];
      dst.rgba[di + 2] = src.rgba[si + 2];
      dst.rgba[di + 3] = src.rgba[si + 3];
    }
  }
}

static std::string ToUpperAscii(std::string s)
{
  for (char& c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    if (uc >= 'a' && uc <= 'z') c = static_cast<char>(uc - 'a' + 'A');
  }
  return s;
}

static std::string TitleCaseAscii(std::string s)
{
  if (s.empty()) return s;
  s = ToUpperAscii(s.substr(0, 1)) + s.substr(1);
  for (std::size_t i = 1; i < s.size(); ++i) {
    const unsigned char uc = static_cast<unsigned char>(s[i]);
    if (uc >= 'A' && uc <= 'Z') s[i] = static_cast<char>(uc - 'A' + 'a');
  }
  return s;
}

static std::string SyllableWord(RNG& rng, int syllables)
{
  static constexpr const char* kOnset[] = {
      "b",  "br", "c",  "ch", "d",  "dr", "f",  "g",  "gr", "h",  "j",  "k",
      "l",  "m",  "n",  "p",  "ph", "pl", "pr", "qu", "r",  "s",  "sh", "st",
      "t",  "tr", "v",  "w",  "wh", "y",  "z",
  };
  static constexpr const char* kNucleus[] = {
      "a", "ai", "ao", "au", "e", "ea", "ee", "ei", "i", "ia", "io", "o", "oa", "oo", "ou", "u", "ui", "y",
  };
  static constexpr const char* kCoda[] = {
      "",  "",  "",  "",  "n", "nd", "ng", "nt", "r", "rd", "rk", "rn", "rs", "rt", "s", "sh", "t", "th", "x",
  };

  syllables = std::clamp(syllables, 1, 4);
  std::string out;
  for (int i = 0; i < syllables; ++i) {
    const char* o = kOnset[rng.rangeInt(0, static_cast<int>(std::size(kOnset)) - 1)];
    const char* n = kNucleus[rng.rangeInt(0, static_cast<int>(std::size(kNucleus)) - 1)];
    const char* c = kCoda[rng.rangeInt(0, static_cast<int>(std::size(kCoda)) - 1)];
    out += o;
    out += n;
    out += c;
  }
  // Keep names readable.
  if (out.size() > 18) out.resize(18);
  return TitleCaseAscii(out);
}

static std::string GenerateCityNameInternal(std::uint64_t seed)
{
  // "C17" = cartography v1. Keep it stable so posters are deterministic per seed.
  RNG rng(seed ^ 0xC17C0FFEE1234ULL);
  const int syl = 2 + rng.rangeInt(0, 1);
  const std::string base = SyllableWord(rng, syl);

  static constexpr const char* kSuffix[] = {"", " City", " Town", " Haven", " Harbor", " Heights", " Springs"};
  const char* suf = kSuffix[rng.rangeInt(0, static_cast<int>(std::size(kSuffix)) - 1)];
  return base + suf;
}

struct DistrictAgg {
  int tiles = 0;
  int water = 0;
  int park = 0;
  int road = 0;
  int res = 0;
  int com = 0;
  int ind = 0;

  double sumX = 0.0;
  double sumY = 0.0;
  double sumH = 0.0;

  int minX = 1 << 30;
  int minY = 1 << 30;
  int maxX = -(1 << 30);
  int maxY = -(1 << 30);
};

static std::vector<std::string> GenerateDistrictNamesInternal(const World& world)
{
  std::vector<DistrictAgg> agg(static_cast<std::size_t>(kDistrictCount));

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);
      const int d = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
      DistrictAgg& a = agg[static_cast<std::size_t>(d)];
      a.tiles++;
      a.sumX += static_cast<double>(x);
      a.sumY += static_cast<double>(y);
      a.sumH += static_cast<double>(t.height);
      a.minX = std::min(a.minX, x);
      a.minY = std::min(a.minY, y);
      a.maxX = std::max(a.maxX, x);
      a.maxY = std::max(a.maxY, y);

      if (t.terrain == Terrain::Water) a.water++;
      if (t.overlay == Overlay::Park) a.park++;
      if (t.overlay == Overlay::Road) a.road++;
      if (t.overlay == Overlay::Residential) a.res++;
      if (t.overlay == Overlay::Commercial) a.com++;
      if (t.overlay == Overlay::Industrial) a.ind++;
    }
  }

  std::vector<std::string> names(static_cast<std::size_t>(kDistrictCount));
  std::unordered_set<std::string> used;
  used.reserve(static_cast<std::size_t>(kDistrictCount) * 2u);

  const float cx = 0.5f * static_cast<float>(std::max(1, world.width() - 1));
  const float cy = 0.5f * static_cast<float>(std::max(1, world.height() - 1));

  for (int d = 0; d < kDistrictCount; ++d) {
    const DistrictAgg& a = agg[static_cast<std::size_t>(d)];

    RNG rng(world.seed() ^ (static_cast<std::uint64_t>(d + 1) * 0x9E3779B97F4A7C15ULL));
    const std::string base = SyllableWord(rng, 2 + rng.rangeInt(0, 1));

    const float tiles = static_cast<float>(std::max(1, a.tiles));
    const float waterR = static_cast<float>(a.water) / tiles;
    const float parkR = static_cast<float>(a.park) / tiles;
    const float indR = static_cast<float>(a.ind) / tiles;
    const float comR = static_cast<float>(a.com) / tiles;
    const float resR = static_cast<float>(a.res) / tiles;
    const float hAvg = static_cast<float>(a.sumH / tiles);

    // Directional tag from centroid (kept short so labels fit).
    const float dx = static_cast<float>(a.sumX / tiles) - cx;
    const float dy = static_cast<float>(a.sumY / tiles) - cy;
    std::string dir;
    if (std::fabs(dx) > 0.15f * static_cast<float>(world.width()) || std::fabs(dy) > 0.15f * static_cast<float>(world.height())) {
      if (dy < -0.10f * static_cast<float>(world.height())) dir += "North";
      else if (dy > 0.10f * static_cast<float>(world.height())) dir += "South";
      if (dx < -0.10f * static_cast<float>(world.width())) dir += (dir.empty() ? "West" : "west");
      else if (dx > 0.10f * static_cast<float>(world.width())) dir += (dir.empty() ? "East" : "east");
      if (!dir.empty()) dir += " ";
    }

    std::string suffix;
    if (waterR > 0.18f) {
      static constexpr const char* kWater[] = {"Harbor", "Riverside", "Quays", "Marsh", "Bay"};
      suffix = kWater[rng.rangeInt(0, static_cast<int>(std::size(kWater)) - 1)];
    } else if (parkR > 0.22f) {
      static constexpr const char* kGreen[] = {"Gardens", "Grove", "Parklands", "Green"};
      suffix = kGreen[rng.rangeInt(0, static_cast<int>(std::size(kGreen)) - 1)];
    } else if (indR > 0.22f) {
      static constexpr const char* kInd[] = {"Works", "Foundry", "Yards", "Plant"};
      suffix = kInd[rng.rangeInt(0, static_cast<int>(std::size(kInd)) - 1)];
    } else if (comR > 0.18f) {
      static constexpr const char* kCom[] = {"Market", "Center", "Downtown"};
      suffix = kCom[rng.rangeInt(0, static_cast<int>(std::size(kCom)) - 1)];
    } else if (hAvg > 0.62f) {
      static constexpr const char* kHigh[] = {"Heights", "Ridge", "Highlands"};
      suffix = kHigh[rng.rangeInt(0, static_cast<int>(std::size(kHigh)) - 1)];
    } else if (resR > 0.20f) {
      static constexpr const char* kRes[] = {"Estates", "Village", "Hills", "Terrace"};
      suffix = kRes[rng.rangeInt(0, static_cast<int>(std::size(kRes)) - 1)];
    } else {
      static constexpr const char* kGen[] = {"Ward", "Quarter", "District"};
      suffix = kGen[rng.rangeInt(0, static_cast<int>(std::size(kGen)) - 1)];
    }

    std::string name = dir + base;
    if (!suffix.empty()) name += " " + suffix;

    // Ensure uniqueness (append roman-ish numerals deterministically).
    if (used.find(name) != used.end()) {
      static constexpr const char* kDup[] = {" II", " III", " IV", " V"};
      for (const char* s : kDup) {
        const std::string alt = name + s;
        if (used.find(alt) == used.end()) {
          name = alt;
          break;
        }
      }
    }

    used.insert(name);
    names[static_cast<std::size_t>(d)] = name;
  }

  return names;
}

struct Candidate {
  MapLabelKind kind = MapLabelKind::Street;
  int id = -1;
  std::string text;
  int anchorX = 0;
  int anchorY = 0;
  int scale = 1;
  int priority = 0;
};

static void DrawLabel(RgbaImage& img, const CartographyConfig& cfg, const MapLabel& l, Rgba8 fill, Rgba8 outline)
{
  const int pad = std::max(0, cfg.labelPaddingPx);
  if (cfg.labelBackground) {
    const Rgba8 bg{0, 0, 0, cfg.labelBgAlpha};
    gfx::FillRect(img, l.x, l.y, l.x + l.w - 1, l.y + l.h - 1, bg, BlendMode::Alpha);
  }

  gfx::DrawText5x7Outlined(img, l.x + pad, l.y + pad, l.text, fill, outline, l.scale, 1, BlendMode::Alpha);
}

static std::vector<MapLabel> PlaceAndDrawLabels(RgbaImage& img,
                                                const RectI& bounds,
                                                const std::vector<Candidate>& candidates,
                                                const CartographyConfig& cfg)
{
  std::vector<Candidate> sorted = candidates;
  std::sort(sorted.begin(), sorted.end(), [](const Candidate& a, const Candidate& b) {
    if (a.priority != b.priority) return a.priority > b.priority;
    if (a.kind != b.kind) return static_cast<int>(a.kind) < static_cast<int>(b.kind);
    return a.text < b.text;
  });

  const int pad = std::max(0, cfg.labelPaddingPx);
  std::vector<RectI> used;
  used.reserve(sorted.size());

  std::vector<MapLabel> out;
  out.reserve(sorted.size());

  const int offsets[][2] = {
      {0, 0},
      {0, -8},
      {0, 8},
      {-8, 0},
      {8, 0},
      {-10, -10},
      {10, -10},
      {-10, 10},
      {10, 10},
      {0, -16},
      {0, 16},
      {-16, 0},
      {16, 0},
  };

  for (const Candidate& c : sorted) {
    const int tw = gfx::MeasureTextWidth5x7(c.text, c.scale, 1);
    const int th = gfx::MeasureTextHeight5x7(c.scale);
    const int w = tw + pad * 2;
    const int h = th + pad * 2;
    if (w <= 0 || h <= 0) continue;

    bool placed = false;
    RectI r{};
    int bestX = 0, bestY = 0;
    for (const auto& o : offsets) {
      const int ox = o[0] * std::max(1, c.scale);
      const int oy = o[1] * std::max(1, c.scale);

      const int x0 = c.anchorX + ox - w / 2;
      const int y0 = c.anchorY + oy - h / 2;

      r.x0 = x0;
      r.y0 = y0;
      r.x1 = x0 + w - 1;
      r.y1 = y0 + h - 1;

      // Keep labels inside the map bounds (poster margins excluded).
      if (r.x0 < bounds.x0 || r.y0 < bounds.y0 || r.x1 > bounds.x1 || r.y1 > bounds.y1) continue;

      bool overlap = false;
      for (const RectI& u : used) {
        if (Overlaps(r, u)) {
          overlap = true;
          break;
        }
      }
      if (overlap) continue;

      bestX = x0;
      bestY = y0;
      placed = true;
      break;
    }

    if (!placed) continue;

    MapLabel ml;
    ml.kind = c.kind;
    ml.id = c.id;
    ml.text = c.text;
    ml.x = bestX;
    ml.y = bestY;
    ml.w = w;
    ml.h = h;
    ml.anchorX = c.anchorX;
    ml.anchorY = c.anchorY;
    ml.scale = c.scale;

    used.push_back(RectI{ml.x, ml.y, ml.x + ml.w - 1, ml.y + ml.h - 1});
    out.push_back(ml);
  }

  // Draw pass (after all placement so earlier labels don't affect later placement)
  for (const MapLabel& l : out) {
    if (l.kind == MapLabelKind::District) {
      DrawLabel(img, cfg, l, Rgba8{250, 245, 220, 245}, Rgba8{15, 15, 15, 220});
    } else {
      DrawLabel(img, cfg, l, Rgba8{250, 250, 250, 245}, Rgba8{10, 10, 10, 220});
    }
  }

  return out;
}

static void DrawDistrictBoundaries(RgbaImage& img, const World& world, const IsoOverviewResult& iso)
{
  if (world.width() <= 0 || world.height() <= 0) return;
  if (img.width <= 0 || img.height <= 0) return;

  // Subtle outline that stays readable over bright terrain.
  const Rgba8 line{0, 0, 0, 85};

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const int d = static_cast<int>(world.at(x, y).district);

      int cx = 0, cy = 0;
      if (!IsoTileCenterToPixel(world, iso, x, y, cx, cy)) continue;

      const int topX = cx;
      const int topY = cy - iso.halfH;
      const int rightX = cx + iso.halfW;
      const int rightY = cy;
      const int botX = cx;
      const int botY = cy + iso.halfH;
      const int leftX = cx - iso.halfW;
      const int leftY = cy;

      // Compare with east neighbor -> draw SE edge (right->bottom).
      if (x + 1 < world.width()) {
        const int de = static_cast<int>(world.at(x + 1, y).district);
        if (de != d) {
          gfx::StrokeLine(img, rightX, rightY, botX, botY, line, BlendMode::Alpha);
        }
      }

      // Compare with south neighbor -> draw SW edge (bottom->left).
      if (y + 1 < world.height()) {
        const int ds = static_cast<int>(world.at(x, y + 1).district);
        if (ds != d) {
          gfx::StrokeLine(img, botX, botY, leftX, leftY, line, BlendMode::Alpha);
        }
      }

      // Optionally also outline coastlines? (future)
      (void)topX;
      (void)topY;
    }
  }
}

static void DrawLegend(RgbaImage& img, int x, int y)
{
  // Minimal legend for the most common overlays (colors are only approximate).
  struct Entry {
    const char* name;
    Rgba8 c;
  };
  const Entry entries[] = {
      {"Road", Rgba8{60, 60, 60, 255}},
      {"Residential", Rgba8{80, 180, 90, 255}},
      {"Commercial", Rgba8{90, 140, 220, 255}},
      {"Industrial", Rgba8{220, 170, 70, 255}},
      {"Park", Rgba8{50, 140, 60, 255}},
      {"Water", Rgba8{70, 140, 220, 255}},
  };

  int cy = y;
  const int box = 10;
  for (const auto& e : entries) {
    gfx::FillRect(img, x, cy, x + box, cy + box, e.c, BlendMode::Alpha);
    gfx::DrawText5x7Outlined(img, x + box + 6, cy - 1, e.name, Rgba8{250, 250, 250, 235}, Rgba8{10, 10, 10, 200}, 2);
    cy += 16;
  }
}

} // namespace

std::string GenerateCityName(std::uint64_t seed)
{
  return GenerateCityNameInternal(seed);
}

std::vector<std::string> GenerateDistrictNames(const World& world)
{
  return GenerateDistrictNamesInternal(world);
}

CartographyResult RenderLabeledIsoPoster(const World& world,
                                        ExportLayer layer,
                                        const IsoOverviewConfig& isoCfg,
                                        const StreetNamingConfig& streetCfg,
                                        const CartographyConfig& cfg)
{
  CartographyResult out{};

  // 1) Base isometric render (RGB).
  IsoOverviewResult iso = RenderIsoOverview(world, layer, isoCfg);
  RgbaImage base = PpmToRgba(iso.image);

  // 2) Optional poster margins.
  const int marginTop = cfg.poster ? std::max(0, cfg.marginTopPx) : 0;
  const int marginSide = cfg.poster ? std::max(0, cfg.marginSidePx) : 0;
  const int marginBottom = cfg.poster ? std::max(0, cfg.marginBottomPx) : 0;

  RgbaImage canvas;
  if (cfg.poster) {
    canvas.width = base.width + marginSide * 2;
    canvas.height = base.height + marginTop + marginBottom;

    // Use the iso background as poster backdrop.
    FillSolid(canvas, isoCfg.bgR, isoCfg.bgG, isoCfg.bgB, 255);
    BlitOpaque(canvas, base, marginSide, marginTop);

    // Shift iso transform so tile->pixel helpers remain valid in poster space.
    iso.offsetX += marginSide;
    iso.offsetY += marginTop;
  } else {
    canvas = std::move(base);
  }

  // Map bounds in the poster canvas (labels should stay inside this).
  const RectI mapBounds{cfg.poster ? marginSide : 0,
                        cfg.poster ? marginTop : 0,
                        (cfg.poster ? marginSide : 0) + iso.image.width - 1,
                        (cfg.poster ? marginTop : 0) + iso.image.height - 1};

  // 3) Optional district boundaries (under text).
  if (cfg.drawDistrictBoundaries) {
    DrawDistrictBoundaries(canvas, world, iso);
  }

  // 4) Deterministic names.
  out.districtNames = GenerateDistrictNames(world);
  out.title = cfg.titleOverride.empty() ? GenerateCityName(world.seed()) : cfg.titleOverride;

  // 5) Poster header (title + legend).
  if (cfg.poster) {
    if (cfg.labelTitle) {
      const int sc = std::max(1, cfg.titleTextScale);
      const int tw = gfx::MeasureTextWidth5x7(out.title, sc, 1);
      const int th = gfx::MeasureTextHeight5x7(sc);
      const int tx = std::max(0, (canvas.width - tw) / 2);
      const int ty = std::max(0, (marginTop - th) / 2 - 6);

      gfx::DrawText5x7Outlined(canvas, tx, ty, out.title, Rgba8{255, 255, 255, 245}, Rgba8{10, 10, 10, 220}, sc);

      MapLabel title;
      title.kind = MapLabelKind::Title;
      title.id = -1;
      title.text = out.title;
      title.x = tx;
      title.y = ty;
      title.w = tw;
      title.h = th;
      title.anchorX = tx + tw / 2;
      title.anchorY = ty + th / 2;
      title.scale = sc;
      out.labels.push_back(title);
    }

    // Legend in the top-left.
    DrawLegend(canvas, marginSide, 10);

    // Seed / size subtitle (top-right).
    {
      std::string meta = "seed " + std::to_string(world.seed()) + "  " + std::to_string(world.width()) + "x" + std::to_string(world.height());
      const int sc = 2;
      const int tw = gfx::MeasureTextWidth5x7(meta, sc, 1);
      const int th = gfx::MeasureTextHeight5x7(sc);
      const int tx = std::max(0, canvas.width - marginSide - tw);
      const int ty = std::max(0, marginTop - th - 10);
      gfx::DrawText5x7Outlined(canvas, tx, ty, meta, Rgba8{245, 245, 245, 220}, Rgba8{10, 10, 10, 190}, sc);
    }
  }

  // 6) Build label candidates (streets + districts) anchored in iso pixel space.
  std::vector<Candidate> candidates;
  candidates.reserve(64);

  // District labels: centroid of tiles.
  if (cfg.labelDistricts) {
    std::vector<double> sumX(static_cast<std::size_t>(kDistrictCount), 0.0);
    std::vector<double> sumY(static_cast<std::size_t>(kDistrictCount), 0.0);
    std::vector<int> count(static_cast<std::size_t>(kDistrictCount), 0);

    for (int y = 0; y < world.height(); ++y) {
      for (int x = 0; x < world.width(); ++x) {
        const Tile& t = world.at(x, y);
        if (t.terrain == Terrain::Water) continue; // keep labels off pure water
        const int d = std::clamp(static_cast<int>(t.district), 0, kDistrictCount - 1);
        sumX[static_cast<std::size_t>(d)] += static_cast<double>(x);
        sumY[static_cast<std::size_t>(d)] += static_cast<double>(y);
        count[static_cast<std::size_t>(d)]++;
      }
    }

    for (int d = 0; d < kDistrictCount; ++d) {
      if (count[static_cast<std::size_t>(d)] <= 0) continue;
      const int ax = static_cast<int>(std::lround(sumX[static_cast<std::size_t>(d)] / static_cast<double>(count[static_cast<std::size_t>(d)])));
      const int ay = static_cast<int>(std::lround(sumY[static_cast<std::size_t>(d)] / static_cast<double>(count[static_cast<std::size_t>(d)])));

      int px = 0, py = 0;
      if (!IsoTileCenterToPixel(world, iso, std::clamp(ax, 0, world.width() - 1), std::clamp(ay, 0, world.height() - 1), px, py)) continue;

      Candidate c;
      c.kind = MapLabelKind::District;
      c.id = d;
      c.text = out.districtNames[static_cast<std::size_t>(d)];
      c.anchorX = px;
      c.anchorY = py;
      c.scale = std::max(1, cfg.districtTextScale);
      c.priority = 2000000 + count[static_cast<std::size_t>(d)];
      candidates.push_back(std::move(c));
    }
  }

  // Street labels.
  if (cfg.labelStreets) {
    const StreetNamingResult streets = BuildStreetNames(world, streetCfg);
    const int n = static_cast<int>(streets.streets.size());
    if (n > 0) {
      struct Agg {
        int count = 0;
        int sumX = 0;
        int sumY = 0;
      };
      std::vector<Agg> agg(static_cast<std::size_t>(n));

      // Pass 1: centroid.
      for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
          const int sid = streets.roadTileToStreetId[static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) + static_cast<std::size_t>(x)];
          if (sid < 0 || sid >= n) continue;
          Agg& a = agg[static_cast<std::size_t>(sid)];
          a.count++;
          a.sumX += x;
          a.sumY += y;
        }
      }

      std::vector<int> anchorX(static_cast<std::size_t>(n), 0);
      std::vector<int> anchorY(static_cast<std::size_t>(n), 0);
      for (int sid = 0; sid < n; ++sid) {
        const Agg& a = agg[static_cast<std::size_t>(sid)];
        if (a.count <= 0) continue;
        anchorX[static_cast<std::size_t>(sid)] = static_cast<int>(std::lround(static_cast<double>(a.sumX) / static_cast<double>(a.count)));
        anchorY[static_cast<std::size_t>(sid)] = static_cast<int>(std::lround(static_cast<double>(a.sumY) / static_cast<double>(a.count)));
      }

      // Pass 2: snap to nearest road tile belonging to the street.
      std::vector<int> bestX(static_cast<std::size_t>(n), -1);
      std::vector<int> bestY(static_cast<std::size_t>(n), -1);
      std::vector<int> bestD2(static_cast<std::size_t>(n), 1 << 30);
      for (int y = 0; y < world.height(); ++y) {
        for (int x = 0; x < world.width(); ++x) {
          const int sid = streets.roadTileToStreetId[static_cast<std::size_t>(y) * static_cast<std::size_t>(world.width()) + static_cast<std::size_t>(x)];
          if (sid < 0 || sid >= n) continue;
          const int dx = x - anchorX[static_cast<std::size_t>(sid)];
          const int dy = y - anchorY[static_cast<std::size_t>(sid)];
          const int d2 = dx * dx + dy * dy;
          if (d2 < bestD2[static_cast<std::size_t>(sid)]) {
            bestD2[static_cast<std::size_t>(sid)] = d2;
            bestX[static_cast<std::size_t>(sid)] = x;
            bestY[static_cast<std::size_t>(sid)] = y;
          }
        }
      }

      // Rank streets by importance and add candidates.
      std::vector<int> order;
      order.reserve(static_cast<std::size_t>(n));
      for (int i = 0; i < n; ++i) order.push_back(i);
      std::sort(order.begin(), order.end(), [&](int a, int b) {
        const StreetInfo& sa = streets.streets[static_cast<std::size_t>(a)];
        const StreetInfo& sb = streets.streets[static_cast<std::size_t>(b)];
        if (sa.roadLevel != sb.roadLevel) return sa.roadLevel > sb.roadLevel;
        if (sa.tileCount != sb.tileCount) return sa.tileCount > sb.tileCount;
        return sa.name < sb.name;
      });

      const int maxLabels = std::max(0, cfg.maxStreetLabels);
      int emitted = 0;
      for (int sid : order) {
        if (emitted >= maxLabels) break;
        const StreetInfo& s = streets.streets[static_cast<std::size_t>(sid)];
        if (s.tileCount < 6) continue; // skip very short streets to reduce clutter
        if (bestX[static_cast<std::size_t>(sid)] < 0) continue;

        int px = 0, py = 0;
        if (!IsoTileCenterToPixel(world, iso, bestX[static_cast<std::size_t>(sid)], bestY[static_cast<std::size_t>(sid)], px, py)) continue;

        Candidate c;
        c.kind = MapLabelKind::Street;
        c.id = sid;
        c.text = s.name;
        c.anchorX = px;
        c.anchorY = py;
        c.scale = std::max(1, cfg.streetTextScale);
        c.priority = 1000000 + s.roadLevel * 100000 + s.tileCount;
        candidates.push_back(std::move(c));
        emitted++;
      }
    }
  }

  // Enforce district label limit (after candidates built).
  if (cfg.labelDistricts && cfg.maxDistrictLabels < kDistrictCount) {
    // Keep highest priority districts only.
    std::vector<Candidate> filtered;
    filtered.reserve(candidates.size());
    int keptDistricts = 0;
    for (const Candidate& c : candidates) {
      if (c.kind != MapLabelKind::District) {
        filtered.push_back(c);
        continue;
      }
      // We'll sort later, so keep for now.
      filtered.push_back(c);
    }
    // Apply limit after sort (handled by placement indirectly) by lowering priority
    // for excess districts deterministically.
    std::sort(filtered.begin(), filtered.end(), [](const Candidate& a, const Candidate& b) {
      if (a.kind != b.kind) return static_cast<int>(a.kind) < static_cast<int>(b.kind);
      return a.priority > b.priority;
    });
    for (Candidate& c : filtered) {
      if (c.kind != MapLabelKind::District) continue;
      keptDistricts++;
      if (keptDistricts > std::max(0, cfg.maxDistrictLabels)) {
        c.priority = 0; // effectively discard
      }
    }
    candidates = std::move(filtered);
  }

  // 7) Place and render labels.
  const std::vector<MapLabel> placed = PlaceAndDrawLabels(canvas, mapBounds, candidates, cfg);
  out.labels.insert(out.labels.end(), placed.begin(), placed.end());

  out.image = std::move(canvas);
  return out;
}

} // namespace isocity
