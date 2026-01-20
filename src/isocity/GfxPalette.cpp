#include "isocity/GfxPalette.hpp"

#include "isocity/Random.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>

namespace isocity {

namespace {

struct Hsv {
  float h = 0.0f; // degrees [0,360)
  float s = 0.0f; // [0,1]
  float v = 0.0f; // [0,1]
};

static float WrapHue(float h)
{
  while (h < 0.0f) h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;
  return h;
}

static std::uint8_t ClampU8(int v)
{
  return static_cast<std::uint8_t>(std::clamp(v, 0, 255));
}

static Rgba8 HsvToRgba(const Hsv& in, std::uint8_t a = 255)
{
  // Standard HSV->RGB conversion.
  const float h = WrapHue(in.h);
  const float s = std::clamp(in.s, 0.0f, 1.0f);
  const float v = std::clamp(in.v, 0.0f, 1.0f);

  const float c = v * s;
  const float hh = h / 60.0f;
  const float x = c * (1.0f - std::fabs(std::fmod(hh, 2.0f) - 1.0f));

  float r1 = 0.0f;
  float g1 = 0.0f;
  float b1 = 0.0f;

  if (hh < 1.0f) {
    r1 = c;
    g1 = x;
    b1 = 0.0f;
  } else if (hh < 2.0f) {
    r1 = x;
    g1 = c;
    b1 = 0.0f;
  } else if (hh < 3.0f) {
    r1 = 0.0f;
    g1 = c;
    b1 = x;
  } else if (hh < 4.0f) {
    r1 = 0.0f;
    g1 = x;
    b1 = c;
  } else if (hh < 5.0f) {
    r1 = x;
    g1 = 0.0f;
    b1 = c;
  } else {
    r1 = c;
    g1 = 0.0f;
    b1 = x;
  }

  const float m = v - c;

  const int r = static_cast<int>(std::round((r1 + m) * 255.0f));
  const int g = static_cast<int>(std::round((g1 + m) * 255.0f));
  const int b = static_cast<int>(std::round((b1 + m) * 255.0f));

  return Rgba8{ClampU8(r), ClampU8(g), ClampU8(b), a};
}

static Rgba8 MulRgb(Rgba8 c, float m)
{
  const int r = static_cast<int>(std::round(static_cast<float>(c.r) * m));
  const int g = static_cast<int>(std::round(static_cast<float>(c.g) * m));
  const int b = static_cast<int>(std::round(static_cast<float>(c.b) * m));
  c.r = ClampU8(r);
  c.g = ClampU8(g);
  c.b = ClampU8(b);
  return c;
}

static Rgba8 Lerp(Rgba8 a, Rgba8 b, float t)
{
  t = std::clamp(t, 0.0f, 1.0f);
  auto lerpU8 = [&](std::uint8_t x, std::uint8_t y) -> std::uint8_t {
    return ClampU8(static_cast<int>(std::round(static_cast<float>(x) + (static_cast<float>(y) - static_cast<float>(x)) * t)));
  };
  return Rgba8{lerpU8(a.r, b.r), lerpU8(a.g, b.g), lerpU8(a.b, b.b), lerpU8(a.a, b.a)};
}

static float ThemeSat(GfxTheme t)
{
  switch (t) {
  case GfxTheme::Noir: return 0.0f;
  case GfxTheme::Pastel: return 0.35f;
  case GfxTheme::Neon: return 0.95f;
  case GfxTheme::SpaceColony: return 0.60f;

  case GfxTheme::ProceduralMuted: return 0.45f;
  case GfxTheme::ProceduralVibrant: return 0.95f;
  case GfxTheme::Procedural: return 0.75f;

  default: return 0.75f;
  }
}

} // namespace

GfxPalette GenerateGfxPalette(std::uint32_t seed, GfxTheme theme)
{
  // We mix in the theme so each theme has its own coherent subspace.
  std::uint64_t mix = static_cast<std::uint64_t>(seed) ^
                      (static_cast<std::uint64_t>(static_cast<std::uint8_t>(theme)) * 0x9E3779B97F4A7C15ULL);
  RNG rng(mix ? mix : 1ULL);

  // Small global hue jitter to prevent two cities with the same theme from looking identical.
  const float hueJitter = rng.rangeFloat(-10.0f, 10.0f);

  // Theme baselines.
  Hsv water{210.0f, ThemeSat(theme), 0.82f};
  Hsv sand{44.0f, ThemeSat(theme) * 0.35f, 0.90f};
  Hsv grass{125.0f, ThemeSat(theme) * 0.65f, 0.74f};

  // Overlays are intentionally more saturated than terrain so zones read well.
  Hsv res{6.0f, 0.70f, 0.78f};
  Hsv com{215.0f, 0.65f, 0.75f};
  Hsv ind{45.0f, 0.65f, 0.82f};
  Hsv park{132.0f, 0.65f, 0.74f};

  // Roads.
  Hsv asphalt1{220.0f, 0.10f, 0.40f};
  Hsv asphalt2{220.0f, 0.08f, 0.36f};
  Hsv asphalt3{220.0f, 0.06f, 0.30f};

  Hsv deck1{33.0f, 0.40f, 0.64f};
  Hsv deck2{220.0f, 0.02f, 0.66f};
  Hsv deck3{220.0f, 0.03f, 0.58f};

  switch (theme) {
  case GfxTheme::Classic:
    // Keep close to existing hardcoded colors.
    break;
  case GfxTheme::Autumn:
    water.h = 200.0f;
    water.v = 0.76f;
    sand.h = 36.0f;
    grass.h = 76.0f;
    grass.s *= 0.55f;
    grass.v *= 0.86f;
    park.h = 98.0f;
    deck1.h = 28.0f;
    break;
  case GfxTheme::Desert:
    water.h = 192.0f;
    sand.h = 50.0f;
    sand.v = 0.95f;
    grass.h = 95.0f;
    grass.s *= 0.55f;
    grass.v *= 0.78f;
    park.h = 102.0f;
    asphalt1.h = 30.0f;
    asphalt1.s = 0.12f;
    asphalt2.h = 30.0f;
    asphalt3.h = 30.0f;
    deck1.h = 38.0f;
    break;
  case GfxTheme::Noir:
    // Monochrome: drive saturation down and tune values.
    water.s = 0.0f;
    sand.s = 0.0f;
    grass.s = 0.0f;
    res.s = 0.0f;
    com.s = 0.0f;
    ind.s = 0.0f;
    park.s = 0.0f;
    asphalt1.s = 0.0f;
    asphalt2.s = 0.0f;
    asphalt3.s = 0.0f;
    deck1.s = 0.0f;
    deck2.s = 0.0f;
    deck3.s = 0.0f;

    water.v = 0.55f;
    sand.v = 0.78f;
    grass.v = 0.70f;
    res.v = 0.72f;
    com.v = 0.68f;
    ind.v = 0.76f;
    park.v = 0.70f;
    break;
  case GfxTheme::Neon:
    // Punchy, high saturation.
    water.h = 282.0f;
    grass.h = 170.0f;
    sand.h = 60.0f;
    sand.s = 0.55f;
    res.h = 330.0f;
    com.h = 205.0f;
    ind.h = 50.0f;
    park.h = 150.0f;
    asphalt1.h = 260.0f;
    asphalt1.s = 0.18f;
    asphalt2.h = 260.0f;
    asphalt3.h = 260.0f;
    deck2.h = 210.0f;
    deck3.h = 210.0f;
    break;
  case GfxTheme::Pastel:
    // Low saturation, bright values.
    water.h = 208.0f;
    water.s *= 0.55f;
    water.v = 0.90f;
    sand.h = 45.0f;
    sand.s *= 0.60f;
    sand.v = 0.96f;
    grass.h = 128.0f;
    grass.s *= 0.55f;
    grass.v = 0.88f;
    res.s *= 0.55f;
    com.s *= 0.55f;
    ind.s *= 0.55f;
    park.s *= 0.55f;
    asphalt1.v = 0.46f;
    asphalt2.v = 0.42f;
    asphalt3.v = 0.36f;
    break;
  case GfxTheme::SpaceColony:
    // Cold metals + regolith + saturated zone lighting.
    water.h = 200.0f;
    water.s *= 0.85f;
    water.v = 0.62f;
    sand.h = 32.0f;
    sand.s *= 0.40f;
    sand.v = 0.74f;
    grass.h = 155.0f;
    grass.s *= 0.85f;
    grass.v = 0.70f;

    // Zone overlays lean sci-fi: cyan / purple / amber / teal-green.
    res.h = 190.0f;
    res.s = 0.75f;
    res.v = 0.82f;
    com.h = 285.0f;
    com.s = 0.75f;
    com.v = 0.80f;
    ind.h = 35.0f;
    ind.s = 0.78f;
    ind.v = 0.86f;
    park.h = 145.0f;
    park.s = 0.70f;
    park.v = 0.76f;

    // Roads/bridges read as dark metal.
    asphalt1.h = 215.0f;
    asphalt1.s = 0.12f;
    asphalt1.v = 0.34f;
    asphalt2.h = 215.0f;
    asphalt2.s = 0.10f;
    asphalt2.v = 0.30f;
    asphalt3.h = 215.0f;
    asphalt3.s = 0.08f;
    asphalt3.v = 0.26f;

    deck1.h = 210.0f;
    deck1.s = 0.10f;
    deck1.v = 0.60f;
    deck2.h = 210.0f;
    deck2.s = 0.07f;
    deck2.v = 0.62f;
    deck3.h = 210.0f;
    deck3.s = 0.05f;
    deck3.v = 0.56f;
    break;

  case GfxTheme::Procedural:
  case GfxTheme::ProceduralMuted:
  case GfxTheme::ProceduralVibrant: {
    // Procedurally synthesize a coherent palette from the seed.
    //
    // Strategy:
    //  - pick a "biome hue" for vegetation (grass)
    //  - derive water and sand hues relative to that hue so terrain feels cohesive
    //  - pick zone overlay hues using a simple harmony scheme (triad/complementary/analogous)
    //  - keep roads mostly neutral with a subtle cool/warm tint
    const float terrainSat = ThemeSat(theme);

    // Controls that shape the overall "feel".
    const float dry = rng.nextF01();     // 0 = lush, 1 = arid
    const float strange = rng.nextF01(); // pushes into more alien hues sometimes
    const float coolWarm = rng.nextF01();
    const float season = rng.nextF01();

    // Vegetation hue buckets.
    if (strange < 0.10f) {
      grass.h = rng.rangeFloat(290.0f, 340.0f); // magenta/purple alien flora
    } else if (strange < 0.20f) {
      grass.h = rng.rangeFloat(150.0f, 210.0f); // teal/cyan flora
    } else if (season < 0.25f) {
      grass.h = rng.rangeFloat(60.0f, 95.0f);   // yellow-green / autumn
    } else {
      grass.h = rng.rangeFloat(95.0f, 155.0f);  // classic greens
    }

    // Terrain saturation/value tuned by aridity.
    grass.s = std::clamp(terrainSat * (0.42f + 0.40f * (1.0f - dry)), 0.0f, 1.0f);
    grass.v = std::clamp(0.60f + 0.22f * (1.0f - dry) + rng.rangeFloat(-0.02f, 0.02f), 0.0f, 1.0f);

    // Derive sand/water hues from vegetation hue so the palette feels coherent.
    sand.h = WrapHue(grass.h - 80.0f + rng.rangeFloat(-12.0f, 12.0f));
    sand.s = std::clamp(terrainSat * (0.10f + 0.22f * dry), 0.0f, 1.0f);
    sand.v = std::clamp(0.86f + 0.10f * dry, 0.0f, 1.0f);

    water.h = WrapHue(grass.h + 90.0f + rng.rangeFloat(-24.0f, 24.0f));
    water.s = std::clamp(terrainSat * (0.70f + 0.20f * (1.0f - dry)), 0.0f, 1.0f);
    water.v = std::clamp(0.58f + 0.28f * (1.0f - dry), 0.0f, 1.0f);

    // --- Zone overlays ---
    const float baseOverlaySat =
        (theme == GfxTheme::ProceduralMuted) ? 0.45f :
        (theme == GfxTheme::ProceduralVibrant) ? 0.85f : 0.65f;

    const float baseOverlayVal =
        (theme == GfxTheme::ProceduralMuted) ? 0.86f :
        (theme == GfxTheme::ProceduralVibrant) ? 0.82f : 0.78f;

    // Harmony scheme.
    const float baseHue = rng.rangeFloat(0.0f, 360.0f);
    const std::uint32_t scheme = rng.rangeU32(3); // 0=triadic, 1=complementary, 2=analogous

    float hA = baseHue;
    float hB = baseHue;
    float hC = baseHue;

    if (scheme == 0) {
      // Triadic: strong separation.
      hB = WrapHue(hA + 120.0f);
      hC = WrapHue(hA + 240.0f);
    } else if (scheme == 1) {
      // Complementary + split complement.
      hB = WrapHue(hA + 180.0f);
      hC = WrapHue(hA + 210.0f);
      hA = WrapHue(hA + 30.0f);
    } else {
      // Analogous: keep closer hues, but push value differences to keep readability.
      hB = WrapHue(hA + 35.0f);
      hC = WrapHue(hA + 70.0f);
    }

    res = Hsv{hA, baseOverlaySat, baseOverlayVal};
    com = Hsv{hB, baseOverlaySat, std::clamp(baseOverlayVal * 0.96f, 0.0f, 1.0f)};
    ind = Hsv{hC, std::clamp(baseOverlaySat + 0.05f, 0.0f, 1.0f), std::clamp(baseOverlayVal + 0.04f, 0.0f, 1.0f)};

    // Parks stay tied to vegetation hue but with enough saturation/value to read.
    park.h = WrapHue(grass.h + rng.rangeFloat(-8.0f, 8.0f));
    park.s = std::clamp(baseOverlaySat * 0.90f, 0.30f, 1.0f);
    park.v = std::clamp(baseOverlayVal * 0.96f, 0.0f, 1.0f);

    // --- Roads / bridges ---
    const float roadHue = WrapHue(water.h + ((coolWarm < 0.5f) ? -10.0f : 10.0f) + rng.rangeFloat(-6.0f, 6.0f));
    const float roadV = std::clamp(0.40f - 0.08f * dry, 0.20f, 0.55f);

    asphalt1.h = roadHue;
    asphalt1.s = 0.10f;
    asphalt1.v = roadV;

    asphalt2.h = roadHue;
    asphalt2.s = 0.08f;
    asphalt2.v = std::clamp(roadV - 0.04f, 0.18f, 0.55f);

    asphalt3.h = roadHue;
    asphalt3.s = 0.06f;
    asphalt3.v = std::clamp(roadV - 0.10f, 0.16f, 0.55f);

    // Bridge decks: pick either warm wood-ish or cool metal-ish.
    if (rng.chance(0.55f)) {
      deck1.h = WrapHue(sand.h + 4.0f);
      deck1.s = std::clamp(0.35f * (0.75f + 0.25f * (1.0f - dry)), 0.0f, 1.0f);
      deck1.v = 0.66f;
    } else {
      deck1.h = roadHue;
      deck1.s = 0.08f;
      deck1.v = 0.62f;
    }

    deck2 = deck1;
    deck2.s *= 0.55f;
    deck2.v = std::min(0.70f, deck1.v + 0.02f);

    deck3 = deck1;
    deck3.s *= 0.70f;
    deck3.v = std::max(0.48f, deck1.v - 0.06f);

    break;
  }
  }

  // Per-theme, per-seed micro-shifts.
  auto jitterHue = [&](Hsv& c, float degrees) {
    c.h = WrapHue(c.h + hueJitter + rng.rangeFloat(-degrees, degrees));
  };

  jitterHue(water, 8.0f);
  jitterHue(sand, 6.0f);
  jitterHue(grass, 8.0f);
  jitterHue(res, 5.0f);
  jitterHue(com, 5.0f);
  jitterHue(ind, 4.0f);
  jitterHue(park, 5.0f);

  // Asphalt hue jitter kept small to avoid “colored roads” unless the theme wants it.
  jitterHue(asphalt1, (theme == GfxTheme::Neon) ? 6.0f : 2.5f);
  jitterHue(asphalt2, (theme == GfxTheme::Neon) ? 6.0f : 2.0f);
  jitterHue(asphalt3, (theme == GfxTheme::Neon) ? 6.0f : 1.5f);

  jitterHue(deck1, 4.0f);
  jitterHue(deck2, 2.0f);
  jitterHue(deck3, 2.0f);

  // Build palette.
  GfxPalette pal{};
  pal.water = HsvToRgba(water, 255);
  pal.sand = HsvToRgba(sand, 255);
  pal.grass = HsvToRgba(grass, 255);

  pal.roadAsphalt1 = HsvToRgba(asphalt1, 230);
  pal.roadAsphalt2 = HsvToRgba(asphalt2, 235);
  pal.roadAsphalt3 = HsvToRgba(asphalt3, 240);

  // Markings: keep close to white/yellow, but nudge to match themes.
  const float markTint = (theme == GfxTheme::Noir) ? 0.90f : 1.00f;
  pal.roadMarkWhite = MulRgb(Rgba8{245, 245, 242, 245}, markTint);
  pal.roadMarkYellow = MulRgb(Rgba8{250, 215, 95, 245}, (theme == GfxTheme::Noir) ? 0.80f : 1.0f);

  pal.bridgeDeck1 = HsvToRgba(deck1, 235);
  pal.bridgeDeck2 = HsvToRgba(deck2, 240);
  pal.bridgeDeck3 = HsvToRgba(deck3, 240);

  // Overlays: theme-saturate and ensure readability.
  auto overlayFrom = [&](const Hsv& base) -> Rgba8 {
    Hsv c = base;
    if (theme == GfxTheme::Noir) {
      c.s = 0.0f;
    } else if (theme == GfxTheme::Pastel) {
      c.s *= 0.55f;
      c.v = std::min(0.92f, c.v + 0.10f);
    } else if (theme == GfxTheme::Neon) {
      c.s = std::min(1.0f, c.s * 1.10f);
      c.v = std::min(0.92f, c.v + 0.08f);
    }
    return HsvToRgba(c, 255);
  };

  pal.overlayResidential = overlayFrom(res);
  pal.overlayCommercial = overlayFrom(com);
  pal.overlayIndustrial = overlayFrom(ind);
  pal.overlayPark = overlayFrom(park);

  // Accents.
  pal.shorelineFoam = (theme == GfxTheme::Noir)
                          ? Rgba8{200, 200, 200, 200}
                          : Lerp(Rgba8{245, 245, 245, 210}, pal.water, 0.12f);

  pal.treeDark = (theme == GfxTheme::Noir)
                     ? Rgba8{70, 70, 70, 240}
                     : MulRgb(pal.overlayPark, 0.55f);

  return pal;
}

const char* GfxThemeName(GfxTheme t)
{
  switch (t) {
  case GfxTheme::Classic: return "classic";
  case GfxTheme::Autumn: return "autumn";
  case GfxTheme::Desert: return "desert";
  case GfxTheme::Noir: return "noir";
  case GfxTheme::Neon: return "neon";
  case GfxTheme::Pastel: return "pastel";
  case GfxTheme::SpaceColony: return "space_colony";
  case GfxTheme::Procedural: return "procedural";
  case GfxTheme::ProceduralMuted: return "procedural_muted";
  case GfxTheme::ProceduralVibrant: return "procedural_vibrant";
  default: return "classic";
  }
}

bool ParseGfxTheme(const std::string& s, GfxTheme& out)
{
  auto normalize = [](const std::string& in) -> std::string {
    std::string t;
    t.reserve(in.size());
    for (unsigned char c : in) {
      const char lc = static_cast<char>(std::tolower(c));
      // Treat common separators as ignorable so users can write:
      // "space colony", "space-colony", "space_colony", etc.
      if (lc == ' ' || lc == '\t' || lc == '_' || lc == '-' || lc == '.') continue;
      t.push_back(lc);
    }
    return t;
  };

  const std::string t = normalize(s);

  if (t == "classic" || t == "default" || t == "orig") {
    out = GfxTheme::Classic;
    return true;
  }
  if (t == "autumn" || t == "fall") {
    out = GfxTheme::Autumn;
    return true;
  }
  if (t == "desert" || t == "arid") {
    out = GfxTheme::Desert;
    return true;
  }
  if (t == "noir" || t == "mono" || t == "monochrome" || t == "bw") {
    out = GfxTheme::Noir;
    return true;
  }
  if (t == "neon" || t == "cyber" || t == "cyberpunk") {
    out = GfxTheme::Neon;
    return true;
  }
  if (t == "pastel" || t == "soft") {
    out = GfxTheme::Pastel;
    return true;
  }

  if (t == "space" || t == "spacecolony" || t == "colony" ||
      t == "lunar" || t == "moon" || t == "mars") {
    out = GfxTheme::SpaceColony;
    return true;
  }

  if (t == "procedural" || t == "proc" || t == "generated" || t == "gen" || t == "random") {
    out = GfxTheme::Procedural;
    return true;
  }
  if (t == "proceduralmuted" || t == "procmuted" || t == "procmute" || t == "proceduralsoft") {
    out = GfxTheme::ProceduralMuted;
    return true;
  }
  if (t == "proceduralvibrant" || t == "procvibrant" || t == "procvivid" || t == "proceduralvivid") {
    out = GfxTheme::ProceduralVibrant;
    return true;
  }

  return false;
}

} // namespace isocity
