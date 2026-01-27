#include "isocity/SolarPotential.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace isocity {

namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float DegToRad(float d) { return d * (kPi / 180.0f); }

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// 8- and 16-direction integer step sets (grid coordinates: +x east, +y south).
// These are used for coarse horizon scanning.
struct DirStep {
  int dx = 0;
  int dy = 0;
  float distPerStep = 1.0f; // sqrt(dx*dx + dy*dy)
};

constexpr float kSqrt2 = 1.41421356237f;
constexpr float kSqrt5 = 2.23606797750f;

static constexpr DirStep kDirs8[8] = {
    { 1,  0, 1.0f},   // E
    { 1, -1, kSqrt2}, // NE
    { 0, -1, 1.0f},   // N
    {-1, -1, kSqrt2}, // NW
    {-1,  0, 1.0f},   // W
    {-1,  1, kSqrt2}, // SW
    { 0,  1, 1.0f},   // S
    { 1,  1, kSqrt2}, // SE
};

static constexpr DirStep kDirs16[16] = {
    { 1,  0, 1.0f},   // E
    { 2, -1, kSqrt5}, // ENE
    { 1, -1, kSqrt2}, // NE
    { 1, -2, kSqrt5}, // NNE
    { 0, -1, 1.0f},   // N
    {-1, -2, kSqrt5}, // NNW
    {-1, -1, kSqrt2}, // NW
    {-2, -1, kSqrt5}, // WNW
    {-1,  0, 1.0f},   // W
    {-2,  1, kSqrt5}, // WSW
    {-1,  1, kSqrt2}, // SW
    {-1,  2, kSqrt5}, // SSW
    { 0,  1, 1.0f},   // S
    { 1,  2, kSqrt5}, // SSE
    { 1,  1, kSqrt2}, // SE
    { 2,  1, kSqrt5}, // ESE
};

inline bool IsCivic(Overlay o)
{
  return o == Overlay::School || o == Overlay::Hospital || o == Overlay::PoliceStation || o == Overlay::FireStation;
}

inline float BuildingHeight(const Tile& t, const SolarPotentialConfig& cfg)
{
  if (!cfg.includeBuildings) return 0.0f;

  float h = 0.0f;

  switch (t.overlay) {
  case Overlay::Residential: h = cfg.residentialHeightPerLevel * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;
  case Overlay::Commercial:  h = cfg.commercialHeightPerLevel  * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;
  case Overlay::Industrial:  h = cfg.industrialHeightPerLevel  * static_cast<float>(std::clamp<int>(t.level, 1, 3)); break;
  case Overlay::Road:
  case Overlay::Park:
  case Overlay::None:
    h = 0.0f;
    break;
  default:
    if (IsCivic(t.overlay)) {
      h = cfg.civicHeightPerLevel * static_cast<float>(std::clamp<int>(t.level, 1, 3));
    }
    break;
  }

  if (h > 0.0f && cfg.occupantScale > 0 && cfg.occupantHeightBoost > 0.0f) {
    const float occ01 = Clamp01(static_cast<float>(t.occupants) / static_cast<float>(cfg.occupantScale));
    h += cfg.occupantHeightBoost * occ01;
  }

  return h;
}

inline float RoofAreaFactor(const Tile& t, const SolarPotentialConfig& cfg)
{
  float base = 0.0f;
  switch (t.overlay) {
  case Overlay::Residential: base = cfg.roofResidential; break;
  case Overlay::Commercial:  base = cfg.roofCommercial; break;
  case Overlay::Industrial:  base = cfg.roofIndustrial; break;
  default:
    if (IsCivic(t.overlay)) base = cfg.roofCivic;
    break;
  }

  if (base <= 0.0f) return 0.0f;

  const float level01 = Clamp01(static_cast<float>(std::clamp<int>(t.level, 1, 3)) / 3.0f);
  float v = base * level01;

  if (cfg.roofOccupantScale > 0 && cfg.roofOccupantBoost > 0.0f) {
    const float occ01 = Clamp01(static_cast<float>(t.occupants) / static_cast<float>(cfg.roofOccupantScale));
    v *= (1.0f + cfg.roofOccupantBoost * occ01);
  }

  return Clamp01(v);
}

} // namespace

SolarPotentialResult ComputeSolarPotential(const World& world, const SolarPotentialConfig& cfg)
{
  SolarPotentialResult out{};
  out.w = world.width();
  out.h = world.height();
  out.cfg = cfg;

  const int w = out.w;
  const int h = out.h;
  if (w <= 0 || h <= 0) return out;

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  out.exposure01.assign(n, 0.0f);
  out.roofArea01.assign(n, 0.0f);
  out.potential01.assign(n, 0.0f);

  auto idx = [&](int x, int y) -> std::size_t {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
  };

  // Effective height field: terrain height + (optional) building height.
  std::vector<float> effH(n, 0.0f);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = world.at(x, y);
      effH[idx(x, y)] = t.height + BuildingHeight(t, cfg);
    }
  }

  // Roof factor.
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float roof = RoofAreaFactor(world.at(x, y), cfg);
      out.roofArea01[idx(x, y)] = roof;
      if (roof > 0.0f) out.roofTileCount++;
      if (world.at(x, y).overlay == Overlay::Residential && world.at(x, y).occupants > 0) {
        out.residentPopulation += static_cast<int>(world.at(x, y).occupants);
      }
    }
  }

  const int maxR = std::max(0, cfg.maxHorizonRadius);

  const DirStep* dirs = (cfg.azimuthSamples <= 8) ? kDirs8 : kDirs16;
  const int dirCount = (cfg.azimuthSamples <= 8) ? 8 : 16;

  // Precompute altitude weights (sin of altitude).
  std::vector<float> altRad;
  std::vector<float> altWeight;
  altRad.reserve(cfg.altitudeDeg.size());
  altWeight.reserve(cfg.altitudeDeg.size());
  for (float d : cfg.altitudeDeg) {
    if (d <= 0.0f) continue;
    const float r = DegToRad(d);
    const float wgt = std::sin(std::clamp(r, 0.0f, 0.5f * kPi));
    if (wgt <= 0.0f) continue;
    altRad.push_back(r);
    altWeight.push_back(wgt);
  }

  // In single-sample mode we don't need the whole altitude list.
  const float singleAltRad = DegToRad(cfg.singleAltitudeDeg);

  int highCount = 0;
  double totalPotential = 0.0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i0 = idx(x, y);
      const float h0 = effH[i0];

      // Compute horizon angles in each sampled direction.
      float horizon[16] = {}; // maximum possible dirCount is 16
      for (int di = 0; di < dirCount; ++di) {
        float maxAng = 0.0f;
        const DirStep d = dirs[di];

        // Scan outward.
        for (int s = 1; s <= maxR; ++s) {
          const int xx = x + d.dx * s;
          const int yy = y + d.dy * s;
          if (xx < 0 || yy < 0 || xx >= w || yy >= h) break;

          const float dh = effH[idx(xx, yy)] - h0;
          if (dh <= 0.0f) continue;

          const float dist = d.distPerStep * static_cast<float>(s);
          if (dist <= 0.0f) continue;

          const float ang = std::atan2(dh, dist);
          if (ang > maxAng) maxAng = ang;
        }

        horizon[di] = maxAng;
      }

      float exposure = 0.0f;

      if (cfg.singleSample) {
        // Map the azimuth to our nearest discrete compass direction.
        const float az = DegToRad(cfg.singleAzimuthDeg);
        // Standard azimuth in world coords has +y pointing North. Our grid has +y = South,
        // so we invert Y (dy = -sin).
        const float vx = std::cos(az);
        const float vy = -std::sin(az);

        int bestDi = 0;
        float bestDot = -std::numeric_limits<float>::infinity();
        for (int di = 0; di < dirCount; ++di) {
          const DirStep d = dirs[di];
          const float invLen = (d.distPerStep > 0.0f) ? (1.0f / d.distPerStep) : 1.0f;
          const float dxn = static_cast<float>(d.dx) * invLen;
          const float dyn = static_cast<float>(d.dy) * invLen;
          const float dot = dxn * vx + dyn * vy;
          if (dot > bestDot) {
            bestDot = dot;
            bestDi = di;
          }
        }

        exposure = (singleAltRad > horizon[bestDi]) ? 1.0f : 0.0f;
      } else {
        // Uniform azimuth sampling; weight each altitude by sin(altitude) (horizontal irradiance proxy).
        double lit = 0.0;
        double total = 0.0;

        for (std::size_t ai = 0; ai < altRad.size(); ++ai) {
          const float a = altRad[ai];
          const float wgt = altWeight[ai];

          total += static_cast<double>(wgt) * static_cast<double>(dirCount);

          for (int di = 0; di < dirCount; ++di) {
            if (a > horizon[di]) {
              lit += static_cast<double>(wgt);
            }
          }
        }

        exposure = (total > 0.0) ? static_cast<float>(lit / total) : 0.0f;
      }

      exposure = Clamp01(exposure / std::max(0.0001f, cfg.clamp01));
      out.exposure01[i0] = exposure;
      out.maxExposure01 = std::max(out.maxExposure01, exposure);

      const float roof = out.roofArea01[i0];
      float pot = Clamp01(exposure * roof / std::max(0.0001f, cfg.clamp01));
      out.potential01[i0] = pot;
      out.maxPotential01 = std::max(out.maxPotential01, pot);

      totalPotential += static_cast<double>(pot);

      if (roof > 0.0f && pot >= out.highPotentialThreshold) {
        highCount++;
      }
    }
  }

  if (out.residentPopulation > 0) {
    out.perCapitaPotential = static_cast<float>(totalPotential / static_cast<double>(out.residentPopulation));
  } else {
    out.perCapitaPotential = 0.0f;
  }

  if (out.roofTileCount > 0) {
    out.roofHighPotentialFrac = static_cast<float>(static_cast<double>(highCount) / static_cast<double>(out.roofTileCount));
  } else {
    out.roofHighPotentialFrac = 0.0f;
  }

  return out;
}

} // namespace isocity
