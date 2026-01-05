#include "isocity/Export.hpp"

#include "isocity/Soft3D.hpp"
#include "isocity/WorldMeshBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

namespace {

inline float QuantizeHeight(float v, float step)
{
  if (!(step > 0.0f) || !std::isfinite(step)) return v;
  const double q = std::round(static_cast<double>(v) / static_cast<double>(step));
  return static_cast<float>(q * static_cast<double>(step));
}

inline float BaseHeightAt(const World& world, int x, int y, const MeshExportConfig& mc)
{
  // Heights are authored as Tile::height in [0,1]. We scale to world units.
  const float raw = world.at(x, y).height * mc.heightScale;
  return QuantizeHeight(raw, mc.heightQuantization);
}

inline MeshN3 ApproxTerrainNormal(const World& world, int x, int y, const MeshExportConfig& mc)
{
  const int w = world.width();
  const int h = world.height();
  const int xl = std::max(0, x - 1);
  const int xr = std::min(w - 1, x + 1);
  const int yd = std::max(0, y - 1);
  const int yu = std::min(h - 1, y + 1);

  const float hl = BaseHeightAt(world, xl, y, mc);
  const float hr = BaseHeightAt(world, xr, y, mc);
  const float hd = BaseHeightAt(world, x, yd, mc);
  const float hu = BaseHeightAt(world, x, yu, mc);

  const float ds = std::max(1e-6f, mc.tileSize * 2.0f);
  const float sx = (hr - hl) / ds;
  const float sz = (hu - hd) / ds;

  // For a heightfield y = f(x,z), a normal can be approximated as (-df/dx, 1, -df/dz).
  const float nx = -sx;
  const float ny = 1.0f;
  const float nz = -sz;
  const float len = std::sqrt(std::max(1e-12f, nx * nx + ny * ny + nz * nz));
  return MeshN3{nx / len, ny / len, nz / len};
}

struct VecSink final : IMeshSink {
  explicit VecSink(std::vector<MeshQuad>& q) : quads(q) {}
  void addQuad(const MeshQuad& q) override { quads.push_back(q); }
  std::vector<MeshQuad>& quads;
};

} // namespace

PpmImage RenderWorld3D(const World& world, ExportLayer layer, const Render3DConfig& cfg,
                       const LandValueResult* landValue,
                       const TrafficResult* traffic,
                       const GoodsResult* goods)
{
  if (world.width() <= 0 || world.height() <= 0) return {};
  if (cfg.width <= 0 || cfg.height <= 0) return {};

  // Use the canonical 2D exporter as a stable source of per-tile colors.
  const PpmImage topColors = RenderPpmLayer(world, layer, landValue, traffic, goods);

  MeshExportConfig mc = cfg.meshCfg;
  // Provide safe defaults if caller left mc uninitialized.
  if (!(mc.tileSize > 0.0f)) mc.tileSize = 1.0f;
  if (!(mc.heightScale > 0.0f)) mc.heightScale = 8.0f;

  int x0 = 0, y0 = 0, x1 = world.width(), y1 = world.height();
  int originX = 0, originY = 0;
  std::string boundsErr;
  if (!ComputeMeshExportBounds(world, mc, x0, y0, x1, y1, originX, originY, &boundsErr)) {
    return {};
  }

  const float tileSize = mc.tileSize;
  const float overlayOff = mc.overlayOffset;

  std::vector<MeshQuad> quads;
  quads.reserve(static_cast<std::size_t>(x1 - x0) * static_cast<std::size_t>(y1 - y0));

  // --- Top surfaces (always per-tile so heatmaps render correctly) ---
  if (mc.includeTopSurfaces) {
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        const Tile& t = world.at(x, y);
        const float baseY = BaseHeightAt(world, x, y, mc);
        const float topY = baseY + ((t.overlay != Overlay::None) ? overlayOff : 0.0f);

        const float fx0 = static_cast<float>(x - originX) * tileSize;
        const float fx1 = static_cast<float>(x + 1 - originX) * tileSize;
        const float fz0 = static_cast<float>(y - originY) * tileSize;
        const float fz1 = static_cast<float>(y + 1 - originY) * tileSize;

        // Per-tile color source (RenderPpmLayer returns full-world pixels).
        MeshC4 c{0, 0, 0, 255};
        if (x >= 0 && y >= 0 && x < topColors.width && y < topColors.height &&
            topColors.rgb.size() >= static_cast<std::size_t>(topColors.width * topColors.height * 3)) {
          const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(topColors.width) +
                                 static_cast<std::size_t>(x)) * 3u;
          c.r = topColors.rgb[i + 0];
          c.g = topColors.rgb[i + 1];
          c.b = topColors.rgb[i + 2];
        }

        MeshQuad q;
        q.a = MeshV3{fx0, topY, fz0};
        q.b = MeshV3{fx1, topY, fz0};
        q.c = MeshV3{fx1, topY, fz1};
        q.d = MeshV3{fx0, topY, fz1};
        q.n = ApproxTerrainNormal(world, x, y, mc);
        q.material = MeshMaterial::Grass;
        q.color = c;
        quads.push_back(q);
      }
    }
  }

  // --- Cliffs + buildings from the mesh generator (but skip top surfaces to avoid duplicates) ---
  {
    MeshExportConfig extras = mc;
    extras.includeTopSurfaces = false;
    std::string err;
    VecSink sink(quads);
    (void)BuildWorldMeshQuads(world, extras, sink, &err);
    // Non-fatal: if BuildWorldMeshQuads fails due to unexpected config, we still have top surfaces.
  }

  // --- Software render ---
  Soft3DCamera cam;
  cam.yawDeg = cfg.yawDeg;
  cam.pitchDeg = cfg.pitchDeg;
  cam.rollDeg = cfg.rollDeg;
  cam.autoFit = cfg.autoFit;
  cam.fitMargin = cfg.fitMargin;
  cam.targetX = cfg.targetX;
  cam.targetY = cfg.targetY;
  cam.targetZ = cfg.targetZ;
  cam.distance = cfg.distance;
  cam.fovYDeg = cfg.fovYDeg;
  cam.orthoHalfHeight = cfg.orthoHalfHeight;
  cam.projection = (cfg.projection == Render3DConfig::Projection::Perspective)
                       ? Soft3DCamera::Projection::Perspective
                       : Soft3DCamera::Projection::Orthographic;

  Soft3DShading shading;
  shading.lightDirX = cfg.lightDirX;
  shading.lightDirY = cfg.lightDirY;
  shading.lightDirZ = cfg.lightDirZ;
  shading.ambient = cfg.ambient;
  shading.diffuse = cfg.diffuse;
  shading.bgR = cfg.bgR;
  shading.bgG = cfg.bgG;
  shading.bgB = cfg.bgB;
  shading.enableFog = cfg.fog;
  shading.fogStrength = cfg.fogStrength;
  shading.fogStart = cfg.fogStart;
  shading.fogEnd = cfg.fogEnd;

  Soft3DRenderConfig rc;
  rc.width = cfg.width;
  rc.height = cfg.height;
  rc.supersample = cfg.supersample;
  rc.drawOutlines = cfg.drawOutlines;
  rc.outlineR = cfg.outlineR;
  rc.outlineG = cfg.outlineG;
  rc.outlineB = cfg.outlineB;
  rc.outlineDepthEps = cfg.outlineDepthEps;

  std::string renderErr;
  PpmImage img = RenderQuadsSoft3D(quads, cam, shading, rc, nullptr, nullptr, &renderErr);
  (void)renderErr;
  return img;
}

} // namespace isocity
