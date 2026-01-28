#include "isocity/RenderPipeline.hpp"

#include "isocity/Elevation.hpp"
#include "isocity/Renderer.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/VisualPrefs.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <sstream>

namespace isocity {

namespace {

static std::string FormatMs(double ms)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);
  oss << ms << "ms";
  return oss.str();
}

static bool EnsureParentDirExists(const std::filesystem::path& p, std::string& outError)
{
  outError.clear();
  if (p.empty()) return true;

  std::filesystem::path parent = p.parent_path();
  if (parent.empty()) return true;

  std::error_code ec;
  std::filesystem::create_directories(parent, ec);
  if (ec) {
    outError = "render_overview: failed to create output directory '" + parent.string() + "': " + ec.message();
    return false;
  }
  return true;
}

static std::filesystem::path TempPathForImage(const std::filesystem::path& out)
{
  const std::filesystem::path dir = out.parent_path();
  const std::string stem = out.stem().string();
  const std::string ext = out.extension().string();
  if (!ext.empty()) {
    return dir / (stem + ".tmp" + ext);
  }
  return dir / (stem + ".tmp.png");
}

static void ApplyRenderSafeModeOverrides(VisualPrefs& prefs)
{
  // Keep the world readable and reduce reliance on optional shaders.
  prefs.weather.affectScreen = false;
  prefs.weather.drawParticles = false;

  // Disable shader-heavy optional effects.
  prefs.materialFx.enabled = false;
  prefs.volumetricClouds.enabled = false;
  prefs.cloudShadows.enabled = false;

  // Keep palette/theme choice; it should be safe.
}

static void ApplyVisualPrefsToRenderer(Renderer& renderer, const VisualPrefs& prefs)
{
  renderer.setGfxTheme(prefs.gfxTheme);
  renderer.setBaseCacheEnabled(prefs.baseCacheEnabled);
  renderer.setLayerMask(prefs.layerMask);
  renderer.setShadowSettings(prefs.shadows);
  renderer.setDayNightSettings(prefs.dayNight);
  renderer.setWeatherSettings(prefs.weather);
  renderer.setMaterialFxSettings(prefs.materialFx);
  renderer.setCloudShadowSettings(prefs.cloudShadows);
  renderer.setVolumetricCloudSettings(prefs.volumetricClouds);
  renderer.markBaseCacheDirtyAll();
}

static bool TryLoadAndApplyVisualPrefs(const RenderOverviewOptions& opt,
                                      Renderer& renderer,
                                      RenderOverviewResult& out,
                                      std::ostringstream& rep)
{
  out.visualPrefsApplied = false;
  out.visualPrefsPathUsed.clear();

  if (!opt.useVisualPrefs) {
    if (opt.safeMode) {
      VisualPrefs prefs;
      ApplyRenderSafeModeOverrides(prefs);
      ApplyVisualPrefsToRenderer(renderer, prefs);
      rep << "visual_prefs: disabled; applied safe renderer overrides\n";
    } else {
      rep << "visual_prefs: disabled\n";
    }
    return true;
  }

  std::filesystem::path path = opt.visualPrefsPath;
  if (path.empty()) {
    path = std::filesystem::path("isocity_visual.json");
  }

  std::error_code ec;
  const bool exists = std::filesystem::exists(path, ec) && !ec;
  if (!exists) {
    if (opt.safeMode) {
      VisualPrefs prefs;
      ApplyRenderSafeModeOverrides(prefs);
      ApplyVisualPrefsToRenderer(renderer, prefs);
      rep << "visual_prefs: (not found) " << path.string() << "; applied safe renderer overrides\n";
    } else {
      rep << "visual_prefs: (not found) " << path.string() << "\n";
    }
    return true;
  }

  VisualPrefs prefs;
  std::string err;
  if (!LoadVisualPrefsJsonFile(path.string(), prefs, err)) {
    if (opt.safeMode) {
      VisualPrefs safe;
      ApplyRenderSafeModeOverrides(safe);
      ApplyVisualPrefsToRenderer(renderer, safe);
      rep << "visual_prefs: failed to load " << path.string() << " (" << err
          << "); applied safe renderer overrides\n";
    } else {
      rep << "visual_prefs: failed to load " << path.string() << " (" << err << ")\n";
    }
    return true; // non-fatal
  }

  if (opt.safeMode) {
    ApplyRenderSafeModeOverrides(prefs);
  }

  // Apply to the renderer so the export matches in-game visuals.
  ApplyVisualPrefsToRenderer(renderer, prefs);

  out.visualPrefsApplied = true;
  out.visualPrefsPathUsed = path;
  rep << "visual_prefs: applied " << path.string() << (opt.safeMode ? " (safe overrides)" : "") << "\n";
  return true;
}

class ScopedRaylibWindow {
public:
  ScopedRaylibWindow(int w, int h, const char* title, bool hidden)
  {
    unsigned int flags = 0u;
    if (hidden) flags |= FLAG_WINDOW_HIDDEN;
    SetConfigFlags(flags);
    InitWindow(w, h, title);
    m_ready = IsWindowReady();
  }

  ~ScopedRaylibWindow()
  {
    if (m_ready) {
      CloseWindow();
    }
  }

  bool ready() const { return m_ready; }

private:
  bool m_ready = false;
};

static bool RenameOrCopy(const std::filesystem::path& src,
                         const std::filesystem::path& dst,
                         std::string& outError)
{
  outError.clear();

  std::error_code ec;

  // Best-effort: remove destination first so rename can succeed on Windows.
  std::filesystem::remove(dst, ec);
  ec.clear();

  std::filesystem::rename(src, dst, ec);
  if (!ec) return true;

  // Fallback: copy + remove.
  std::error_code ec2;
  std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing, ec2);
  if (ec2) {
    outError = "render_overview: failed to move output '" + src.string() + "' -> '" + dst.string() +
               "': " + ec.message() + "; copy fallback also failed: " + ec2.message();
    return false;
  }

  std::filesystem::remove(src, ec2);
  if (ec2) {
    outError = "render_overview: wrote '" + dst.string() + "' but failed to delete temp file '" + src.string() +
               "': " + ec2.message();
  }
  return true;
}

} // namespace

bool RenderWorldOverviewFromSave(const RenderOverviewOptions& opt,
                                RenderOverviewResult& out,
                                std::string& outError)
{
  outError.clear();
  out = RenderOverviewResult{};

  if (opt.savePath.empty()) {
    outError = "render_overview: save path is empty";
    return false;
  }
  if (opt.outImagePath.empty()) {
    outError = "render_overview: output path is empty";
    return false;
  }

  std::filesystem::path outPath = opt.outImagePath;
  if (outPath.extension().empty()) {
    outPath += ".png";
  }

  std::string dirErr;
  if (!EnsureParentDirExists(outPath, dirErr)) {
    outError = dirErr;
    return false;
  }

  // Load save (headless).
  World world;
  ProcGenConfig procCfg{};
  SimConfig simCfg{};
  std::string loadErr;
  if (!LoadWorldBinary(world, procCfg, simCfg, opt.savePath.string(), loadErr)) {
    outError = "render_overview: failed to load '" + opt.savePath.string() + "': " + loadErr;
    return false;
  }

  const int tileW = std::max(1, opt.tileWidth);
  const int tileH = std::max(1, opt.tileHeight);

  const int winW = std::max(1, opt.windowWidth);
  const int winH = std::max(1, opt.windowHeight);

  const std::filesystem::path tmpPath = TempPathForImage(outPath);

  // Clean up any stale temp file.
  {
    std::error_code ec;
    std::filesystem::remove(tmpPath, ec);
  }

  ScopedRaylibWindow window(winW, winH, "ProcIsoCity Render Overview", opt.hiddenWindow);
  if (!window.ready()) {
    outError = "render_overview: raylib InitWindow failed (window not ready)";
    return false;
  }

  Renderer renderer(tileW, tileH, world.seed());

  std::ostringstream rep;
  rep << "ProcIsoCity render overview\n";
  rep << "save: " << opt.savePath.string() << "\n";
  rep << "out: " << outPath.string() << "\n";
  rep << "tmp: " << tmpPath.string() << "\n";
  rep << "tile: " << tileW << "x" << tileH << "\n";

  // Best-effort: apply visual prefs (optional; non-fatal if missing or invalid).
  (void)TryLoadAndApplyVisualPrefs(opt, renderer, out, rep);

  ElevationSettings elev{};
  elev.maxPixels = static_cast<float>(tileH) * std::max(0.0f, opt.elevationScale);
  elev.quantizeSteps = std::max(0, opt.elevationSteps);
  elev.flattenWater = true;
  renderer.setElevationSettings(elev);

  const int maxSize = std::max(64, opt.maxSize);

  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = renderer.exportWorldOverview(
      world,
      tmpPath.string().c_str(),
      maxSize,
      opt.timeSec,
      opt.includeScreenFx);
  const auto t1 = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  rep << "elevation_scale: " << opt.elevationScale << "\n";
  rep << "elevation_steps: " << elev.quantizeSteps << "\n";
  rep << "max_size: " << maxSize << "\n";
  rep << "time_sec: " << opt.timeSec << "\n";
  rep << "screen_fx: " << (opt.includeScreenFx ? "yes" : "no") << "\n";
  rep << "render_ms: " << FormatMs(ms) << "\n";
  rep << "result: " << (ok ? "PASS" : "FAIL") << "\n";

  out.report = rep.str();

  if (!ok) {
    outError = "render_overview: Renderer::exportWorldOverview failed (see proc_isocity.log / raylib TraceLog)";
    // Best effort: do not leave a partial temp file around.
    std::error_code ec;
    std::filesystem::remove(tmpPath, ec);
    return false;
  }

  std::string moveErr;
  (void)RenameOrCopy(tmpPath, outPath, moveErr);
  if (!moveErr.empty()) {
    // Not fatal, but surface it in the report so tooling sees it.
    out.report += "warning: " + moveErr + "\n";
  }

  out.ok = true;
  out.outImagePath = outPath;
  return true;
}

} // namespace isocity
