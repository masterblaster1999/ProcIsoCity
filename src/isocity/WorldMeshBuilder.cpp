#include "isocity/WorldMeshBuilder.hpp"

#include "isocity/ZoneParcels.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline float ClampF(float v, float lo, float hi)
{
  return std::max(lo, std::min(hi, v));
}

inline int ClampI(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

inline std::uint8_t ToU8(float f01)
{
  const float c = ClampF(f01, 0.0f, 1.0f);
  const int v = static_cast<int>(std::lround(c * 255.0f));
  return static_cast<std::uint8_t>(ClampI(v, 0, 255));
}

inline MeshC4 RGB(float r, float g, float b)
{
  return MeshC4{ToU8(r), ToU8(g), ToU8(b), 255};
}

MeshMaterial SurfaceMaterialForTile(const Tile& t)
{
  // Overlays take precedence so exports are visually legible.
  switch (t.overlay) {
    case Overlay::Road: return MeshMaterial::Road;
    case Overlay::Residential: return MeshMaterial::Residential;
    case Overlay::Commercial: return MeshMaterial::Commercial;
    case Overlay::Industrial: return MeshMaterial::Industrial;
    case Overlay::Park: return MeshMaterial::Park;
    case Overlay::None: break;
  }

  switch (t.terrain) {
    case Terrain::Water: return MeshMaterial::Water;
    case Terrain::Sand: return MeshMaterial::Sand;
    case Terrain::Grass: return MeshMaterial::Grass;
  }

  return MeshMaterial::Grass;
}

bool ValidateAndComputeBounds(const World& world, const MeshExportConfig& cfg,
                              int& outX0, int& outY0, int& outX1, int& outY1,
                              int& outOriginX, int& outOriginY,
                              std::string* outError)
{
  const int w = world.width();
  const int h = world.height();
  if (w <= 0 || h <= 0) {
    if (outError) *outError = "world has invalid dimensions";
    return false;
  }

  int x0 = 0;
  int y0 = 0;
  int x1 = w;
  int y1 = h;

  if (cfg.hasCrop) {
    if (cfg.cropW <= 0 || cfg.cropH <= 0) {
      if (outError) *outError = "cropW/cropH must be positive";
      return false;
    }
    x0 = ClampI(cfg.cropX, 0, w);
    y0 = ClampI(cfg.cropY, 0, h);
    x1 = ClampI(cfg.cropX + cfg.cropW, 0, w);
    y1 = ClampI(cfg.cropY + cfg.cropH, 0, h);
    if (x1 <= x0 || y1 <= y0) {
      if (outError) *outError = "crop rectangle is empty after clamping";
      return false;
    }
  }

  outX0 = x0;
  outY0 = y0;
  outX1 = x1;
  outY1 = y1;
  outOriginX = (cfg.hasCrop && cfg.originAtCrop) ? x0 : 0;
  outOriginY = (cfg.hasCrop && cfg.originAtCrop) ? y0 : 0;
  return true;
}

inline float QuantizeHeight(float v, float step)
{
  if (!(step > 0.0f) || !std::isfinite(step)) {
    return v;
  }
  const double q = std::round(static_cast<double>(v) / static_cast<double>(step));
  return static_cast<float>(q * static_cast<double>(step));
}

} // namespace

bool ComputeMeshExportBounds(const World& world, const MeshExportConfig& cfg,
                             int& x0, int& y0, int& x1, int& y1,
                             int& originX, int& originY,
                             std::string* outError)
{
  return ValidateAndComputeBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError);
}

const char* ObjMaterialName(MeshMaterial m)
{
  switch (m) {
    case MeshMaterial::Water: return "mat_water";
    case MeshMaterial::Sand: return "mat_sand";
    case MeshMaterial::Grass: return "mat_grass";
    case MeshMaterial::Road: return "mat_road";
    case MeshMaterial::Residential: return "mat_res";
    case MeshMaterial::Commercial: return "mat_com";
    case MeshMaterial::Industrial: return "mat_ind";
    case MeshMaterial::Park: return "mat_park";
    case MeshMaterial::Cliff: return "mat_cliff";
    case MeshMaterial::Building: return "mat_building";
    case MeshMaterial::BuildingResidential: return "mat_building_res";
    case MeshMaterial::BuildingCommercial: return "mat_building_com";
    case MeshMaterial::BuildingIndustrial: return "mat_building_ind";
    default: return "mat_grass";
  }
}

MeshC4 MaterialColor(MeshMaterial m)
{
  // Palette matches MeshExport.cpp / GltfExport.cpp so exports are visually consistent.
  switch (m) {
    case MeshMaterial::Water: return RGB(0.10f, 0.35f, 0.90f);
    case MeshMaterial::Sand: return RGB(0.85f, 0.80f, 0.45f);
    case MeshMaterial::Grass: return RGB(0.20f, 0.70f, 0.20f);

    case MeshMaterial::Road: return RGB(0.20f, 0.20f, 0.22f);
    case MeshMaterial::Residential: return RGB(0.25f, 0.80f, 0.35f);
    case MeshMaterial::Commercial: return RGB(0.25f, 0.55f, 0.95f);
    case MeshMaterial::Industrial: return RGB(0.95f, 0.55f, 0.20f);
    case MeshMaterial::Park: return RGB(0.15f, 0.85f, 0.15f);

    case MeshMaterial::Cliff: return RGB(0.45f, 0.35f, 0.25f);

    case MeshMaterial::BuildingResidential: return RGB(0.70f, 0.90f, 0.75f);
    case MeshMaterial::BuildingCommercial: return RGB(0.65f, 0.75f, 0.95f);
    case MeshMaterial::BuildingIndustrial: return RGB(0.95f, 0.75f, 0.55f);
    case MeshMaterial::Building: return RGB(0.75f, 0.75f, 0.75f);

    default: break;
  }

  return RGB(0.75f, 0.75f, 0.75f);
}

bool BuildWorldMeshQuads(const World& world, const MeshExportConfig& cfg, IMeshSink& sink, std::string* outError)
{
  if (outError) outError->clear();

  int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  int originX = 0, originY = 0;
  if (!ValidateAndComputeBounds(world, cfg, x0, y0, x1, y1, originX, originY, outError)) {
    return false;
  }

  const float tileSize = (cfg.tileSize > 0.0f) ? cfg.tileSize : 1.0f;
  const float hScale = cfg.heightScale;
  const float overlayOff = cfg.overlayOffset;
  const float cliffThr = std::max(0.0f, cfg.cliffThreshold);
  const float qStep = cfg.heightQuantization;

  auto baseHAt = [&](int x, int y) -> float {
    if (!world.inBounds(x, y)) return 0.0f;
    const float raw = world.at(x, y).height * hScale;
    return QuantizeHeight(raw, qStep);
  };

  // --- Top surfaces ---
  if (cfg.includeTopSurfaces) {
    const MeshN3 up{0.0f, 1.0f, 0.0f};

    const int bw = x1 - x0;
    const int bh = y1 - y0;
    if (bw > 0 && bh > 0 && cfg.mergeTopSurfaces) {
      std::vector<std::uint8_t> visited(static_cast<std::size_t>(bw) * static_cast<std::size_t>(bh), 0);
      auto vidx = [&](int x, int y) -> std::size_t {
        return static_cast<std::size_t>(y - y0) * static_cast<std::size_t>(bw) +
               static_cast<std::size_t>(x - x0);
      };

      struct Key {
        MeshMaterial mat;
        float y;
      };

      auto keyAt = [&](int x, int y) -> Key {
        const Tile& t = world.at(x, y);
        const MeshMaterial mat = SurfaceMaterialForTile(t);
        const float baseY = baseHAt(x, y);
        const float topY = baseY + ((t.overlay != Overlay::None) ? overlayOff : 0.0f);
        return Key{mat, topY};
      };

      for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
          const std::size_t vi = vidx(x, y);
          if (visited[vi]) continue;

          const Key k = keyAt(x, y);

          // Greedy rectangle expansion (deterministic scanline order).
          int rectW = 1;
          while (x + rectW < x1) {
            const std::size_t vj = vidx(x + rectW, y);
            if (visited[vj]) break;
            const Key kk = keyAt(x + rectW, y);
            if (kk.mat != k.mat || kk.y != k.y) break;
            rectW++;
          }

          int rectH = 1;
          bool canGrow = true;
          while (canGrow && (y + rectH < y1)) {
            for (int dx = 0; dx < rectW; ++dx) {
              const int cx = x + dx;
              const int cy = y + rectH;
              const std::size_t vk = vidx(cx, cy);
              if (visited[vk]) {
                canGrow = false;
                break;
              }
              const Key kk = keyAt(cx, cy);
              if (kk.mat != k.mat || kk.y != k.y) {
                canGrow = false;
                break;
              }
            }
            if (canGrow) rectH++;
          }

          // Mark visited.
          for (int dy = 0; dy < rectH; ++dy) {
            for (int dx = 0; dx < rectW; ++dx) {
              visited[vidx(x + dx, y + dy)] = 1;
            }
          }

          const float fx0 = static_cast<float>(x - originX) * tileSize;
          const float fx1 = static_cast<float>(x + rectW - originX) * tileSize;
          const float fz0 = static_cast<float>(y - originY) * tileSize;
          const float fz1 = static_cast<float>(y + rectH - originY) * tileSize;

          MeshQuad q;
          q.a = MeshV3{fx0, k.y, fz0};
          q.b = MeshV3{fx1, k.y, fz0};
          q.c = MeshV3{fx1, k.y, fz1};
          q.d = MeshV3{fx0, k.y, fz1};
          q.n = up;
          q.material = k.mat;
          q.color = MaterialColor(k.mat);
          sink.addQuad(q);
        }
      }
    } else {
      for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
          const Tile& t = world.at(x, y);
          const MeshMaterial mat = SurfaceMaterialForTile(t);
          const float baseY = baseHAt(x, y);
          const float topY = baseY + ((t.overlay != Overlay::None) ? overlayOff : 0.0f);

          const float fx0 = static_cast<float>(x - originX) * tileSize;
          const float fx1 = static_cast<float>(x + 1 - originX) * tileSize;
          const float fz0 = static_cast<float>(y - originY) * tileSize;
          const float fz1 = static_cast<float>(y + 1 - originY) * tileSize;

          MeshQuad q;
          q.a = MeshV3{fx0, topY, fz0};
          q.b = MeshV3{fx1, topY, fz0};
          q.c = MeshV3{fx1, topY, fz1};
          q.d = MeshV3{fx0, topY, fz1};
          q.n = up;
          q.material = mat;
          q.color = MaterialColor(mat);
          sink.addQuad(q);
        }
      }
    }
  }

  // --- Cliffs (vertical walls on height discontinuities) ---
  if (cfg.includeCliffs) {
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const float h0 = baseHAt(x, y);

        const float fx0 = static_cast<float>(x - originX) * tileSize;
        const float fx1 = static_cast<float>(x + 1 - originX) * tileSize;
        const float fz0 = static_cast<float>(y - originY) * tileSize;
        const float fz1 = static_cast<float>(y + 1 - originY) * tileSize;

        const MeshMaterial mat = MeshMaterial::Cliff;
        const MeshC4 c = MaterialColor(mat);

        // Right boundary (x+1): vertical wall at X=fx1.
        if (x + 1 < x1) {
          const float h1 = baseHAt(x + 1, y);
          const float dh = h0 - h1;
          if (std::fabs(dh) > cliffThr) {
            const float top = std::max(h0, h1);
            const float bot = std::min(h0, h1);
            const float xp = fx1;
            const MeshN3 n{(dh > 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f};

            MeshQuad q;
            q.a = MeshV3{xp, top, fz0};
            q.b = MeshV3{xp, top, fz1};
            q.c = MeshV3{xp, bot, fz1};
            q.d = MeshV3{xp, bot, fz0};
            q.n = n;
            q.material = mat;
            q.color = c;
            sink.addQuad(q);
          }
        }

        // Bottom boundary (y+1): vertical wall at Z=fz1.
        if (y + 1 < y1) {
          const float h1 = baseHAt(x, y + 1);
          const float dh = h0 - h1;
          if (std::fabs(dh) > cliffThr) {
            const float top = std::max(h0, h1);
            const float bot = std::min(h0, h1);
            const float zp = fz1;
            const MeshN3 n{0.0f, 0.0f, (dh > 0.0f) ? 1.0f : -1.0f};

            MeshQuad q;
            q.a = MeshV3{fx0, top, zp};
            q.b = MeshV3{fx1, top, zp};
            q.c = MeshV3{fx1, bot, zp};
            q.d = MeshV3{fx0, bot, zp};
            q.n = n;
            q.material = mat;
            q.color = c;
            sink.addQuad(q);
          }
        }
      }
    }
  }

  // --- Buildings ---
  if (cfg.includeBuildings) {
    const float footprint = ClampF(cfg.buildingFootprint, 0.20f, 0.95f);
    const float margin = 0.5f * (1.0f - footprint) * tileSize;

    auto buildingMatForOverlay = [&](Overlay o) -> MeshMaterial {
      switch (o) {
        case Overlay::Residential: return MeshMaterial::BuildingResidential;
        case Overlay::Commercial: return MeshMaterial::BuildingCommercial;
        case Overlay::Industrial: return MeshMaterial::BuildingIndustrial;
        default: break;
      }
      return MeshMaterial::Building;
    };

    auto emitBuildingBox = [&](int bx0, int by0, int bw, int bh,
                               Overlay overlay,
                               std::uint8_t level,
                               int occupants,
                               int capacity,
                               float baseY,
                               float var01,
                               int areaTiles) {
      const int lvl = ClampZoneLevel(static_cast<int>(level));
      const float occ = (capacity > 0)
                            ? ClampF(static_cast<float>(occupants) / static_cast<float>(capacity), 0.0f, 1.0f)
                            : 0.0f;

      const int area = std::max(1, areaTiles);
      const float areaBoost = (cfg.buildingAreaHeight > 0.0f && area > 1)
                                  ? (cfg.buildingAreaHeight * static_cast<float>(std::log2(static_cast<double>(area))))
                                  : 0.0f;

      const float height = tileSize * (cfg.buildingBaseHeight + cfg.buildingPerLevelHeight * static_cast<float>(lvl) +
                                       cfg.buildingOccHeight * occ + areaBoost + 0.25f * ClampF(var01, 0.0f, 1.0f));

      const float fx0 = static_cast<float>(bx0 - originX) * tileSize + margin;
      const float fx1 = static_cast<float>(bx0 + bw - originX) * tileSize - margin;
      const float fz0 = static_cast<float>(by0 - originY) * tileSize + margin;
      const float fz1 = static_cast<float>(by0 + bh - originY) * tileSize - margin;

      if (!(fx1 > fx0) || !(fz1 > fz0)) return;

      const float y0b = baseY;
      const float y1b = baseY + std::max(0.05f * tileSize, height);

      const MeshMaterial mat = buildingMatForOverlay(overlay);
      const MeshC4 c = MaterialColor(mat);

      // Roof.
      {
        MeshQuad q;
        q.a = MeshV3{fx0, y1b, fz0};
        q.b = MeshV3{fx1, y1b, fz0};
        q.c = MeshV3{fx1, y1b, fz1};
        q.d = MeshV3{fx0, y1b, fz1};
        q.n = MeshN3{0.0f, 1.0f, 0.0f};
        q.material = mat;
        q.color = c;
        sink.addQuad(q);
      }

      // North (z0)
      {
        MeshQuad q;
        q.a = MeshV3{fx0, y1b, fz0};
        q.b = MeshV3{fx1, y1b, fz0};
        q.c = MeshV3{fx1, y0b, fz0};
        q.d = MeshV3{fx0, y0b, fz0};
        q.n = MeshN3{0.0f, 0.0f, -1.0f};
        q.material = mat;
        q.color = c;
        sink.addQuad(q);
      }
      // South (z1)
      {
        MeshQuad q;
        q.a = MeshV3{fx0, y1b, fz1};
        q.b = MeshV3{fx1, y1b, fz1};
        q.c = MeshV3{fx1, y0b, fz1};
        q.d = MeshV3{fx0, y0b, fz1};
        q.n = MeshN3{0.0f, 0.0f, 1.0f};
        q.material = mat;
        q.color = c;
        sink.addQuad(q);
      }
      // West (x0)
      {
        MeshQuad q;
        q.a = MeshV3{fx0, y1b, fz0};
        q.b = MeshV3{fx0, y1b, fz1};
        q.c = MeshV3{fx0, y0b, fz1};
        q.d = MeshV3{fx0, y0b, fz0};
        q.n = MeshN3{-1.0f, 0.0f, 0.0f};
        q.material = mat;
        q.color = c;
        sink.addQuad(q);
      }
      // East (x1)
      {
        MeshQuad q;
        q.a = MeshV3{fx1, y1b, fz0};
        q.b = MeshV3{fx1, y1b, fz1};
        q.c = MeshV3{fx1, y0b, fz1};
        q.d = MeshV3{fx1, y0b, fz0};
        q.n = MeshN3{1.0f, 0.0f, 0.0f};
        q.material = mat;
        q.color = c;
        sink.addQuad(q);
      }
    };

    const int bw = x1 - x0;
    const int bh = y1 - y0;

    auto withinBounds = [&](int x, int y) -> bool {
      return (x >= x0 && y >= y0 && x < x1 && y < y1);
    };

    auto covIdx = [&](int x, int y) -> std::size_t {
      return static_cast<std::size_t>(y - y0) * static_cast<std::size_t>(bw) + static_cast<std::size_t>(x - x0);
    };

    std::vector<std::uint8_t> covered;
    if (cfg.mergeBuildings && bw > 0 && bh > 0) {
      covered.assign(static_cast<std::size_t>(bw) * static_cast<std::size_t>(bh), 0);

      ZoneBuildingParcels parcels;
      BuildZoneBuildingParcels(world, parcels);

      for (const ZoneBuildingParcel& p : parcels.parcels) {
        // Defensive: only zones.
        if (p.overlay != Overlay::Residential && p.overlay != Overlay::Commercial && p.overlay != Overlay::Industrial) {
          continue;
        }

        // Only merge parcels fully contained in the export bounds.
        if (p.x0 < x0 || p.y0 < y0 || (p.x0 + p.w) > x1 || (p.y0 + p.h) > y1) {
          continue;
        }

        // Only merge when the parcel's base height range (after quantization) stays within a tolerance.
        float minBase = baseHAt(p.x0, p.y0);
        float maxBase = minBase;
        for (int yy = p.y0; yy < p.y0 + p.h; ++yy) {
          for (int xx = p.x0; xx < p.x0 + p.w; ++xx) {
            const float h = baseHAt(xx, yy);
            minBase = std::min(minBase, h);
            maxBase = std::max(maxBase, h);
          }
        }
        if ((maxBase - minBase) > (cfg.mergeBuildingsMaxBaseHeightRange + 1e-6f)) continue;
        const float baseY = minBase + overlayOff + 0.001f;
        const float var01 = static_cast<float>((p.styleSeed >> 4) & 0x0Fu) / 15.0f;
        emitBuildingBox(p.x0, p.y0, p.w, p.h, p.overlay, p.level,
                        p.occupants, p.capacity, baseY, var01, p.area());

        // Mark covered tiles so we don't double-export via per-tile fallback.
        for (int yy = p.y0; yy < p.y0 + p.h; ++yy) {
          for (int xx = p.x0; xx < p.x0 + p.w; ++xx) {
            if (!withinBounds(xx, yy)) continue;
            covered[covIdx(xx, yy)] = 1;
          }
        }
      }
    }

    // Per-tile export (either merge disabled, or as a fallback for tiles not covered by merged parcels).
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        if (!covered.empty() && covered[covIdx(x, y)]) continue;

        const Tile& t = world.at(x, y);
        if (t.overlay != Overlay::Residential && t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) {
          continue;
        }
        if (t.terrain == Terrain::Water) continue;

        const float baseY = baseHAt(x, y) + overlayOff + 0.001f;
        const int cap = CapacityForTile(t);
        const float var01 = static_cast<float>((t.variation >> 4) & 0x0Fu) / 15.0f;
        emitBuildingBox(x, y, 1, 1, t.overlay, t.level,
                        static_cast<int>(t.occupants), cap, baseY, var01, 1);
      }
    }
  }

  return true;
}

} // namespace isocity
