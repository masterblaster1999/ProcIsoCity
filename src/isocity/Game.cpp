#include "isocity/Game.hpp"

#include "isocity/AutoBuild.hpp"
#include "isocity/OsmImport.hpp"
#include "isocity/DistrictStats.hpp"
#include "isocity/Districting.hpp"
#include "isocity/Export.hpp"
#include "isocity/Cartography.hpp"

#include "isocity/GltfExport.hpp"
#include "isocity/FloodFill.hpp"
#include "isocity/Hash.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/Random.hpp"
#include "isocity/Replay.hpp"
#include "isocity/Road.hpp"
#include "isocity/RoadGraphTraffic.hpp"
#include "isocity/RoadUpgradePlannerExport.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/FileSync.hpp"
#include "isocity/Script.hpp"
#include "isocity/TransitPlannerExport.hpp"
#include "isocity/TourPlanner.hpp"
#include "isocity/WorldTiling.hpp"
#include "isocity/Ui.hpp"
#include "isocity/ZoneAccess.hpp"
#include "isocity/ShaderUtil.hpp"
#include "isocity/GfxPalette.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstring>
#include <ctime>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

namespace isocity {

struct Game::DossierJob {
  CityDossierConfig cfg;
  ProcGenConfig procCfg;
  SimConfig simCfg;

  // Snapshot export (current interactive world).
  std::vector<Stats> ticks;
  World world;

  // Seed-bake export (generate+simulate in the worker thread).
  std::vector<std::uint64_t> seeds;
  int bakeDays = 0;
  int w = 0;
  int h = 0;

  std::thread worker;
  std::atomic<bool> cancel{false};
  std::atomic<bool> done{false};
  std::atomic<int> stepIndex{0};
  std::atomic<int> stepCount{0};

  // Batch progress (seed-bake mode).
  std::atomic<int> seedIndex{0};
  std::atomic<int> seedCount{0};
  int okSeeds = 0;
  int failSeeds = 0;

  std::mutex mutex;
  std::string stage;
  bool ok = false;
  CityDossierResult res;
  std::string err;

  ~DossierJob()
  {
    if (worker.joinable()) worker.join();
  }
};

struct Game::MeshJob {
  enum class Kind : std::uint8_t { Obj = 0, Gltf = 1, Glb = 2 };

  Kind kind = Kind::Obj;
  MeshExportConfig cfg;
  World world;

  // Output paths (OBJ uses both objPath + mtlPath).
  std::string outPath;
  std::string mtlPath;

  std::thread worker;
  std::atomic<bool> done{false};

  std::mutex mutex;
  bool ok = false;
  MeshExportStats stats{};
  std::string err;

  ~MeshJob()
  {
    if (worker.joinable()) worker.join();
  }
};


struct Game::PolicyOptJob {
  World world;
  SimConfig simCfg;
  PolicySearchSpace space;
  PolicyOptimizerConfig cfg;
  PolicyOptProgress* progress = nullptr;

  std::thread worker;
  std::atomic<bool> done{false};

  std::mutex mutex;
  bool ok = false;
  PolicyOptimizationResult res;
  std::string err;

  ~PolicyOptJob()
  {
    if (worker.joinable()) worker.join();
  }
};

namespace {
// Slot 1 uses the legacy filename so existing quick-saves keep working.
constexpr const char* kLegacyQuickSavePath = "isocity_save.bin";
constexpr int kSaveSlotMin = 1;
constexpr int kSaveSlotMax = 5;

// In-game blueprint library folder (stores .isobp blueprint stamps).
// This mirrors the CLI tool naming (`proc_isocity_blueprint`) so content can flow
// between headless tools and the interactive app.
constexpr const char* kBlueprintLibraryDir = "isocity_blueprints";
constexpr const char* kBlueprintExt = ".isobp";

// Autosaves rotate through a separate set of slots.
constexpr int kAutosaveSlotMin = 1;
constexpr int kAutosaveSlotMax = 3;
constexpr float kAutosaveIntervalSec = 60.0f;

// Limit of queued dossier jobs to avoid unbounded memory growth.
constexpr int kDossierQueueMax = 8;

// --- UI layout ---
// Panels are drawn in a virtual UI coordinate system (see Game::draw). These constants are in that
// same coordinate space.
constexpr int kUiPanelMargin = 12;
constexpr int kUiPanelSpacing = 12;
constexpr int kUiPanelTopY = 96;

// Standard right-docked panel sizes.
constexpr int kPolicyPanelW = 420;
constexpr int kPolicyPanelH = 620;
constexpr int kNewsPanelW = 460;
constexpr int kNewsPanelH = 420;
constexpr int kChallengesPanelW = 460;
constexpr int kChallengesPanelH = 360;
constexpr int kBondsPanelW = 460;
constexpr int kBondsPanelH = 420;
constexpr int kTrafficPanelW = 420;
constexpr int kTrafficPanelH = 314;
constexpr int kResiliencePanelW = 460;
constexpr int kResiliencePanelH = 420;
constexpr int kDistrictPanelW = 420;
constexpr int kDistrictPanelH = 308;
constexpr int kTransitPanelW = 420;
constexpr int kTransitPanelH = 480;
constexpr int kRoadUpgradePanelW = 460;
constexpr int kRoadUpgradePanelH = 360;
constexpr int kWayfindingPanelW = 420;
constexpr int kWayfindingPanelH = 386;
constexpr int kAutoBuildPanelW = 460;
constexpr int kAutoBuildPanelH = 420;
constexpr int kServicesPanelW = 460;
constexpr int kServicesPanelH = 460;
constexpr int kHeadlessLabPanelW = 460;
constexpr int kHeadlessLabPanelH = 520;


// Simple flow-dock for stacking panels on the right. When vertical space runs out it wraps to a new
// column to the left, keeping panels on-screen without overlap.
struct RightPanelDock {
  int uiW = 0;
  int uiH = 0;

  int rightEdgeX = 0; // right edge of current column
  int cursorY = kUiPanelTopY;
  int colWidth = 0;

  explicit RightPanelDock(int w, int h)
    : uiW(w)
    , uiH(h)
    , rightEdgeX(w - kUiPanelMargin)
    , cursorY(kUiPanelTopY)
    , colWidth(0)
  {
  }

  void wrapColumn()
  {
    rightEdgeX -= (colWidth + kUiPanelSpacing);
    cursorY = kUiPanelTopY;
    colWidth = 0;
  }

  Rectangle alloc(int panelW, int panelH)
  {
    if (uiH > 0) {
      const int maxY = uiH - kUiPanelMargin;
      // If this panel doesn't fit in the current column and the column already contains something,
      // wrap to the next column.
      if (cursorY != kUiPanelTopY && (cursorY + panelH) > maxY) {
        wrapColumn();
      }
    }

    // If the panel is still too tall to fit (e.g. tiny window), pin it as best as we can.
    int y0 = cursorY;
    if (uiH > 0) {
      const int maxY0 = uiH - panelH - kUiPanelMargin;
      if (y0 > maxY0) {
        y0 = std::max(kUiPanelMargin, maxY0);
      }
    }

    const int x0 = rightEdgeX - panelW;
    cursorY = y0 + panelH + kUiPanelSpacing;
    colWidth = std::max(colWidth, panelW);
    return Rectangle{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                     static_cast<float>(panelH)};
  }
};

// --- Vehicle micro-sim tuning ---
constexpr int kMaxCommuteVehicles = 160;
constexpr int kMaxGoodsVehicles = 120;
constexpr int kCommutersPerCar = 40; // how many commuters one visible car represents
constexpr int kGoodsPerTruck = 80;   // goods units represented by one visible truck
constexpr int kMaxSpawnPerFrame = 2;

std::string FileTimestamp()
{
  using namespace std::chrono;

  const auto now = system_clock::now();
  const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::time_t tt = system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif

  char base[32]{};
  std::strftime(base, sizeof(base), "%Y%m%d_%H%M%S", &tm);

  char out[40]{};
  std::snprintf(out, sizeof(out), "%s_%03d", base, static_cast<int>(ms.count()));
  return std::string(out);
}

inline float Rand01(std::uint64_t& state)
{
  // 24-bit mantissa float in [0,1)
  const std::uint64_t u = SplitMix64Next(state);
  return static_cast<float>((u >> 40) & 0x00FFFFFFu) / 16777216.0f;
}

inline float RandRange(std::uint64_t& state, float a, float b)
{
  return a + (b - a) * Rand01(state);
}

// Discrete sim speed presets (dt multiplier).
constexpr float kSimSpeeds[] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f};
constexpr int kSimSpeedCount = static_cast<int>(sizeof(kSimSpeeds) / sizeof(kSimSpeeds[0]));

// World render scaling (resolution scale) helpers.
constexpr float kWorldRenderScaleStep = 0.05f;
constexpr float kWorldRenderScaleAbsMin = 0.25f;
constexpr float kWorldRenderScaleAbsMax = 2.0f;
constexpr float kWorldRenderAutoAdjustInterval = 0.35f; // seconds
constexpr float kWorldRenderDtSmoothing = 0.10f;        // EMA factor
constexpr int kWorldRenderRTMaxDim = 8192;              // safety guard

// Software 3D preview update throttle (seconds). The preview is CPU rendered,
// so we update it infrequently and primarily on world changes.
constexpr float k3DPreviewUpdateInterval = 0.85f;

constexpr float kPi = 3.14159265358979323846f;

inline float SmoothStep(float edge0, float edge1, float x)
{
  if (std::abs(edge1 - edge0) < 1e-6f) {
    return (x < edge0) ? 0.0f : 1.0f;
  }
  float t = (x - edge0) / (edge1 - edge0);
  t = std::clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

struct SimpleDayNightState {
  float phase = 0.0f;
  float sun = 0.0f;
  float day = 1.0f;
  float night = 0.0f;
  float twilight = 0.0f;
};

SimpleDayNightState ComputeSimpleDayNightState(float timeSec, const Renderer::DayNightSettings& s)
{
  SimpleDayNightState out{};
  if (!s.enabled) {
    out.day = 1.0f;
    out.night = 0.0f;
    out.twilight = 0.0f;
    return out;
  }

  const float len = std::max(1.0f, s.dayLengthSec);
  float t = std::fmod(timeSec + s.timeOffsetSec, len);
  if (t < 0.0f) t += len;

  out.phase = t / len;
  out.sun = std::sin(out.phase * 2.0f * kPi);
  out.day = SmoothStep(-0.18f, 0.22f, out.sun);
  out.night = 1.0f - out.day;
  out.twilight = SmoothStep(0.28f, 0.0f, std::abs(out.sun));
  return out;
}

inline void BlendRgb(std::uint8_t& r, std::uint8_t& g, std::uint8_t& b,
                     std::uint8_t tr, std::uint8_t tg, std::uint8_t tb, float a)
{
  a = std::clamp(a, 0.0f, 1.0f);
  const float ia = 1.0f - a;
  r = static_cast<std::uint8_t>(std::clamp(std::lround(ia * r + a * tr), 0l, 255l));
  g = static_cast<std::uint8_t>(std::clamp(std::lround(ia * g + a * tg), 0l, 255l));
  b = static_cast<std::uint8_t>(std::clamp(std::lround(ia * b + a * tb), 0l, 255l));
}

void ApplyInGameAtmosphereGradeToPpm(PpmImage& img, float timeSec,
                                     const Renderer::DayNightSettings& dn,
                                     const Renderer::WeatherSettings& wx)
{
  if (img.width <= 0 || img.height <= 0 || img.rgb.empty()) return;

  const SimpleDayNightState st = ComputeSimpleDayNightState(timeSec, dn);
  const float nightStrength = std::clamp(st.night * std::clamp(dn.nightDarken, 0.0f, 1.0f), 0.0f, 1.0f);
  const float duskStrength = std::clamp(st.twilight * std::clamp(dn.duskTint, 0.0f, 1.0f), 0.0f, 1.0f);

  // Overcast is only user-adjustable when a weather mode is active.
  const float overcast = (wx.mode == Renderer::WeatherSettings::Mode::Clear)
                             ? 0.0f
                             : std::clamp(wx.overcast, 0.0f, 1.0f);

  // Blend targets tuned to roughly match the in-renderer grades.
  const std::uint8_t nightR = 8, nightG = 12, nightB = 45;
  const std::uint8_t duskR = 255, duskG = 150, duskB = 90;
  const std::uint8_t ocR = 85, ocG = 95, ocB = 108;

  const float nightA = nightStrength * (210.0f / 255.0f);
  const float duskA = duskStrength * (110.0f / 255.0f) * (1.0f - nightStrength);
  const float ocA = overcast * 0.32f;

  const int w = img.width;
  const int h = img.height;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3u;
      std::uint8_t& r = img.rgb[i + 0];
      std::uint8_t& g = img.rgb[i + 1];
      std::uint8_t& b = img.rgb[i + 2];

      if (ocA > 0.001f) {
        BlendRgb(r, g, b, ocR, ocG, ocB, ocA);
      }
      if (nightA > 0.001f) {
        BlendRgb(r, g, b, nightR, nightG, nightB, nightA);
      }
      if (duskA > 0.001f) {
        BlendRgb(r, g, b, duskR, duskG, duskB, duskA);
      }
    }
  }
}

void ApplyWeatherTo3DCfg(Render3DConfig& cfg, const Renderer::WeatherSettings& wx)
{
  // Drive the CPU renderer fog from the in-game weather settings.
  const float fog01 = (wx.mode == Renderer::WeatherSettings::Mode::Clear) ? 0.0f : std::clamp(wx.fog, 0.0f, 1.0f);
  cfg.fog = (fog01 > 0.01f);
  cfg.fogStrength = std::clamp(0.2f + 0.8f * fog01, 0.0f, 1.0f);

  // A soft cool-grey fog works well across day/night.
  cfg.fogR = 200;
  cfg.fogG = 210;
  cfg.fogB = 225;

  // Overcast reduces contrast a bit in the 3D renderer itself (in addition to the post-grade).
  const float overcast = (wx.mode == Renderer::WeatherSettings::Mode::Clear) ? 0.0f : std::clamp(wx.overcast, 0.0f, 1.0f);
  cfg.diffuse = std::clamp(cfg.diffuse * (1.0f - 0.35f * overcast), 0.0f, 1.0f);
  cfg.ambient = std::clamp(cfg.ambient * (1.0f + 0.15f * overcast), 0.0f, 1.0f);
}

bool UploadPpmToTexture(Texture2D& tex, int& texW, int& texH, const PpmImage& img)
{
  if (img.width <= 0 || img.height <= 0) return false;
  if (img.rgb.size() != static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u) return false;

  const int w = img.width;
  const int h = img.height;

  // Recreate the texture if the size changed.
  if (tex.id != 0 && (texW != w || texH != h)) {
    UnloadTexture(tex);
    tex = Texture2D{};
    texW = 0;
    texH = 0;
  }

  if (tex.id == 0) {
    Image im{};
    im.width = w;
    im.height = h;
    im.mipmaps = 1;
    im.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;
    const std::size_t bytes = static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u;
    im.data = MemAlloc(static_cast<int>(bytes));
    if (!im.data) return false;
    std::memcpy(im.data, img.rgb.data(), bytes);

    tex = LoadTextureFromImage(im);
    UnloadImage(im);

    if (tex.id == 0) return false;
    SetTextureFilter(tex, TEXTURE_FILTER_BILINEAR);
    texW = w;
    texH = h;
    return true;
  }

  UpdateTexture(tex, img.rgb.data());
  return true;
}
} // namespace

std::string Game::savePathForSlot(int slot) const
{
  const int s = std::clamp(slot, kSaveSlotMin, kSaveSlotMax);
  if (s == 1) return std::string(kLegacyQuickSavePath);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "isocity_save_slot%d.bin", s);
  return std::string(buf);
}

std::string Game::autosavePathForSlot(int slot) const
{
  const int s = std::clamp(slot, kAutosaveSlotMin, kAutosaveSlotMax);
  char buf[64];
  std::snprintf(buf, sizeof(buf), "isocity_autosave_slot%d.bin", s);
  return std::string(buf);
}

std::string Game::thumbPathForSavePath(const std::string& savePath) const
{
  // Convert "*.bin" -> "*.png" (thumbnail image).
  std::string p = savePath;
  const std::size_t dot = p.find_last_of('.');
  if (dot != std::string::npos) {
    p = p.substr(0, dot);
  }
  p += ".png";
  return p;
}

void Game::cycleSaveSlot(int delta)
{
  const int range = (kSaveSlotMax - kSaveSlotMin + 1);
  if (range <= 0) return;

  int s = m_saveSlot + delta;
  while (s < kSaveSlotMin) s += range;
  while (s > kSaveSlotMax) s -= range;
  m_saveSlot = s;
}

bool Game::saveToPath(const std::string& path, bool makeThumbnail, const char* toastLabel)
{
  endPaintStroke();

  std::string err;
  if (!SaveWorldBinary(m_world, m_procCfg, m_sim.config(), path, err)) {
    showToast(std::string("Save failed: ") + err, 4.0f);
    return false;
  }

  if (makeThumbnail) {
    const std::string thumb = thumbPathForSavePath(path);
    // Best effort: do not fail the save if thumbnail export fails.
    (void)m_renderer.exportMinimapThumbnail(m_world, thumb.c_str(), 256);
  }

  if (toastLabel) {
    showToast(TextFormat("Saved: %s", toastLabel));
  } else {
    showToast(TextFormat("Saved: %s", path.c_str()));
  }

  // If the slot browser is open, refresh it so metadata/thumbnails update.
  if (m_showSaveMenu) refreshSaveMenu();
  return true;
}

bool Game::loadFromFile(const std::string& path, const char* toastLabel)
{
  return loadFromPath(path, toastLabel);
}

bool Game::loadFromPath(const std::string& path, const char* toastLabel)
{
  endPaintStroke();

  namespace fs = std::filesystem;

  auto endsWith = [](const std::string& s, const char* suf) -> bool {
    const std::size_t n = std::strlen(suf);
    return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
  };

  const fs::path inputPath = fs::path(path);
  fs::path basePath = inputPath;

  const bool inputIsTmp = endsWith(path, ".tmp");
  const bool inputIsBak = (!inputIsTmp && endsWith(path, ".bak"));

  if (inputIsTmp || inputIsBak) {
    if (path.size() >= 4) {
      basePath = fs::path(path.substr(0, path.size() - 4));
    }
  }

  fs::path tmpPath = basePath;
  tmpPath += ".tmp";
  fs::path bakPath = basePath;
  bakPath += ".bak";

  auto fileTimestamp = [](const fs::path& p, fs::file_time_type& out) -> bool {
    std::error_code ec;
    if (!fs::exists(p, ec) || ec) return false;
    if (!fs::is_regular_file(p, ec) || ec) return false;
    out = fs::last_write_time(p, ec);
    return !ec;
  };

  // Attempt load into temporaries so a failed parse doesn't partially mutate outputs.
  World loaded;
  ProcGenConfig loadedProcCfg{};
  SimConfig loadedSimCfg{};

  auto tryLoad = [&](const fs::path& loadPath, std::string& outErr) -> bool {
    outErr.clear();
    World w;
    ProcGenConfig pc{};
    SimConfig sc{};
    if (!LoadWorldBinary(w, pc, sc, loadPath.string(), outErr)) {
      return false;
    }
    loaded = std::move(w);
    loadedProcCfg = pc;
    loadedSimCfg = sc;
    return true;
  };

  enum class Source : std::uint8_t { Primary = 0, Temp = 1, Backup = 2 };

  fs::path usedPath;
  Source usedSrc = Source::Primary;
  bool recovered = false;
  bool baseAttemptedAndFailed = false;

  std::string errPrimary; // error from the most relevant failure (requested path)
  std::string err;

  // Prefer a newer temp save when loading a canonical slot path.
  if (!inputIsTmp && !inputIsBak) {
    fs::file_time_type tBase{};
    fs::file_time_type tTmp{};
    const bool haveTmp = fileTimestamp(tmpPath, tTmp);
    const bool haveBase = fileTimestamp(basePath, tBase);

    if (haveTmp && (!haveBase || tTmp > tBase)) {
      if (tryLoad(tmpPath, err)) {
        usedPath = tmpPath;
        usedSrc = Source::Temp;
        recovered = true;
      }
    }
  }

  // If we didn't load a preferred temp save, load the requested path.
  if (usedPath.empty()) {
    if (tryLoad(inputPath, err)) {
      usedPath = inputPath;
      usedSrc = inputIsTmp ? Source::Temp : (inputIsBak ? Source::Backup : Source::Primary);
      recovered = inputIsTmp || inputIsBak;
    } else {
      errPrimary = err;
      if (!inputIsTmp && !inputIsBak) {
        baseAttemptedAndFailed = true;
      }
    }
  }

  // Fallback chain (best-effort):
  if (usedPath.empty()) {
    // If the user asked for a temp/backup explicitly, try the canonical path next.
    if ((inputIsTmp || inputIsBak) && inputPath != basePath) {
      if (tryLoad(basePath, err)) {
        usedPath = basePath;
        usedSrc = Source::Primary;
        recovered = true;
      }
    }
  }

  if (usedPath.empty() && !inputIsTmp) {
    if (tryLoad(tmpPath, err)) {
      usedPath = tmpPath;
      usedSrc = Source::Temp;
      recovered = true;
    }
  }

  if (usedPath.empty() && !inputIsBak) {
    if (tryLoad(bakPath, err)) {
      usedPath = bakPath;
      usedSrc = Source::Backup;
      recovered = true;
    }
  }

  if (usedPath.empty()) {
    const std::string bestErr = !errPrimary.empty() ? errPrimary : err;
    showToast(std::string("Load failed: ") + bestErr, 4.0f);
    return false;
  }

  // Toast message: if the caller already provided a "Recovered: ..." label, honor it.
  std::string toast;
  if (toastLabel) {
    const std::string lbl = toastLabel;
    const bool lblIsRecovered = (lbl.rfind("Recovered", 0) == 0);
    if (lblIsRecovered) {
      toast = lbl;
    } else {
      toast = std::string(TextFormat("%s: %s", recovered ? "Recovered" : "Loaded", toastLabel));
    }
  } else {
    toast = std::string(TextFormat("%s: %s", recovered ? "Recovered" : "Loaded", usedPath.string().c_str()));
  }

  if (recovered) {
    m_console.print(TextFormat("save: recovered load from %s", usedPath.string().c_str()));
  }

  adoptLoadedWorld(std::move(loaded), loadedProcCfg, loadedSimCfg, toast, nullptr);

  // --- Best-effort on-disk healing ---
  // If we successfully loaded from a temp/backup, try to promote it back to the canonical slot file.
  auto ensureDirSynced = [&](const fs::path& filePath) {
    fs::path dir = filePath.parent_path();
    if (dir.empty()) {
      std::error_code ec;
      dir = fs::current_path(ec);
      if (ec) dir.clear();
    }
    if (!dir.empty()) {
      BestEffortSyncDirectory(dir);
    }
  };

  auto utcStamp = []() -> std::string {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%SZ", &tm);
    return std::string(buf);
  };

  if (usedSrc == Source::Temp) {
    // Commit tmp -> base (and keep previous base as .bak if present).
    std::error_code ec;
    const bool tmpExists = fs::exists(tmpPath, ec) && !ec && fs::is_regular_file(tmpPath, ec);
    if (tmpExists) {
      ec.clear();
      const bool hadBase = fs::exists(basePath, ec) && !ec && fs::is_regular_file(basePath, ec);

      if (hadBase) {
        // Overwrite any prior backup (best-effort).
        fs::remove(bakPath, ec);
        ec.clear();

        fs::rename(basePath, bakPath, ec);
        if (ec) {
          m_console.print(TextFormat("save: warning: unable to backup stale save before tmp promotion: %s", ec.message().c_str()));
          // If we can't move the base aside, we can't atomically promote the temp file.
          // Leave the .tmp in place so the user can recover it manually.
          return true;
        }
      }

      ec.clear();
      fs::rename(tmpPath, basePath, ec);
      if (ec) {
        // Best-effort rollback.
        std::error_code ec2;
        if (hadBase && fs::exists(bakPath, ec2)) {
          fs::rename(bakPath, basePath, ec2);
        }
        m_console.print(TextFormat("save: warning: unable to promote temp save into place: %s", ec.message().c_str()));
      } else {
        ensureDirSynced(basePath);
      }
    }
  } else if (usedSrc == Source::Backup) {
    // If we fell back to a backup because the primary failed (or is missing), restore the slot file.
    // Keep the .bak as a last-known-good copy.
    std::error_code ec;
    const bool bakExists = fs::exists(bakPath, ec) && !ec && fs::is_regular_file(bakPath, ec);
    if (bakExists) {
      ec.clear();
      const bool baseExists = fs::exists(basePath, ec) && !ec && fs::is_regular_file(basePath, ec);

      const bool shouldRestore = (!baseExists) || baseAttemptedAndFailed;
      if (shouldRestore) {
        if (baseExists && baseAttemptedAndFailed) {
          // Preserve the corrupted file for debugging.
          fs::path corrupt = basePath;
          corrupt += ".corrupt_";
          corrupt += utcStamp();

          ec.clear();
          fs::rename(basePath, corrupt, ec);
          if (ec) {
            m_console.print(TextFormat("save: warning: unable to preserve corrupt save: %s", ec.message().c_str()));
          }
        }

        ec.clear();
        fs::copy_file(bakPath, basePath, fs::copy_options::overwrite_existing, ec);
        if (ec) {
          m_console.print(TextFormat("save: warning: unable to restore backup save: %s", ec.message().c_str()));
        } else {
          BestEffortSyncFile(basePath);
          ensureDirSynced(basePath);
        }
      }
    }
  }

  // Clean up a stale temp file if it's older than the canonical save.
  {
    std::error_code ec;
    fs::file_time_type tBase{};
    fs::file_time_type tTmp{};
    const bool haveBase = fileTimestamp(basePath, tBase);
    const bool haveTmp = fileTimestamp(tmpPath, tTmp);
    if (haveBase && haveTmp && tBase >= tTmp) {
      fs::remove(tmpPath, ec);
    }
  }

  return true;
}

void Game::adoptLoadedWorld(World&& loaded, const ProcGenConfig& loadedProcCfg, const SimConfig& loadedSimCfg,
                            const std::string& toastMessage, const std::vector<Stats>* tickStats)
{
  const std::uint64_t prevSeed = m_world.seed();

  // Any in-flight paint stroke must be ended by the caller, but defensively ensure we
  // aren't mid-stroke when replacing the world.
  m_painting = false;

  // Replacing the world mid-capture would produce a corrupt replay (the base save no longer
  // matches). If a capture is active, stop recording but preserve the captured replay so
  // the user can still save it afterwards.
  if (m_replayCapture.active()) {
    m_replayCapture.stop();
    m_console.print("replay: capture stopped (world replaced)");
  }

  m_world = std::move(loaded);
  m_procCfg = loadedProcCfg;
  m_sim.config() = loadedSimCfg;
  m_sim.resetTimer();

  // New world (different seed) => reset non-persistent gameplay overlays.
  // When the seed is unchanged (e.g., AutoBuild fast-forward), preserve challenges.
  if (m_world.seed() != prevSeed) {
    clearCityChallenges();
  }

  m_renderer.markMinimapDirty();
  m_renderer.markBaseCacheDirtyAll();
  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_tradeDirty = true;
  m_landValueDirty = true;
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;
  m_servicesHeatmapsDirty = true;
  m_fireRiskDirty = true;
  m_servicesPlanDirty = true;
  m_servicesPlanValid = false;
  m_roadUpgradePlanDirty = true;
  m_roadUpgradeSelectedMaskDirty = true;
  invalidateHydrology();
  invalidateAnalysisLayers();
  m_vehiclesDirty = true;
  m_vehicles.clear();

  // Road-resilience caches/suggestions are tied to the road graph.
  m_resilienceDirty = true;
  m_resilienceBypassesDirty = true;
  m_resilienceBypasses.clear();
  m_roadGraphTileToNode.clear();
  m_roadGraphTileToEdge.clear();

  // Keep flood overlay defaults in sync with the loaded proc-gen thresholds.
  m_seaLevel = std::clamp(m_procCfg.waterLevel, 0.0f, 1.0f);

  // Deterministic vehicle RNG seed per world seed.
  m_vehicleRngState = (m_world.seed() ^ 0x9E3779B97F4A7C15ULL);

// Reset policy advisor state (results are tied to the previous world snapshot).
m_policyOptHaveResult = false;
m_policyOptResultOk = false;
m_policyOptResultDay = -1;
m_policyOptResultErr.clear();
m_policyOptResult = PolicyOptimizationResult{};
m_policyOptTopSelection = 0;
m_policyOptTopFirst = 0;

  // Loaded world invalidates undo/redo history.
  m_history.clear();

  // Loaded world invalidates inspect selection/debug overlays.
  m_inspectSelected.reset();
  m_inspectPath.clear();
  m_inspectPathCost = 0;
  m_inspectInfo.clear();

  // Loaded world invalidates any road-drag preview.
  m_roadDragActive = false;
  m_roadDragStart.reset();
  m_roadDragEnd.reset();
  m_roadDragPath.clear();
  m_roadDragBuildCost = 0;
  m_roadDragUpgradeTiles = 0;
  m_roadDragBridgeTiles = 0;
  m_roadDragMoneyCost = 0;
  m_roadDragValid = false;

  // Keep config in sync with loaded world, so regen & camera recenter behave.
  m_cfg.mapWidth = m_world.width();
  m_cfg.mapHeight = m_world.height();
  m_cfg.seed = m_world.seed();

  m_renderer.rebuildTextures(m_cfg.seed);
  SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(m_cfg.seed)));

  // Loaded world invalidates the software 3D preview.
  m_3dPreviewDirty = true;
  m_3dPreviewTimer = 0.0f;

  // Recenter camera on loaded map.
  m_camera.target = TileToWorldCenterElevated(m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2,
                                              static_cast<float>(m_cfg.tileWidth),
                                              static_cast<float>(m_cfg.tileHeight), m_elev);

  // Make HUD stats immediately correct (without waiting for the next sim tick).
  m_sim.refreshDerivedStats(m_world);
  clearHistory();
  if (tickStats && !tickStats->empty()) {
    int moneyOffset = 0;
    for (const Stats& s : *tickStats) {
      Stats ss = s;
      ss.money += moneyOffset;
      const int grant = applyCityChallenges(ss);
      moneyOffset += grant;
      recordHistorySample(ss);
    }
    if (moneyOffset != 0) {
      m_world.stats().money += moneyOffset;
    }
  } else {
    {
      Stats s = m_world.stats();
      const int grant = applyCityChallenges(s);
      if (grant != 0) m_world.stats().money += grant;
      recordHistorySample(s);
    }
  }

  showToast(toastMessage);

  if (m_showSaveMenu) refreshSaveMenu();
}

RaylibContext::RaylibContext(const Config& cfg, const char* title)
{
  unsigned int flags = 0;
  if (cfg.vsync) flags |= FLAG_VSYNC_HINT;
  if (cfg.windowResizable) flags |= FLAG_WINDOW_RESIZABLE;
  if (cfg.windowHighDPI) flags |= FLAG_WINDOW_HIGHDPI;
  SetConfigFlags(flags);

  InitWindow(cfg.windowWidth, cfg.windowHeight, title);

  if (cfg.windowResizable) {
    SetWindowMinSize(std::max(1, cfg.windowMinWidth), std::max(1, cfg.windowMinHeight));
  }

  // You can tune this later or expose it as a config.
  SetTargetFPS(60);

  // Ensure vsync state matches config at runtime.
  if (cfg.vsync) {
    SetWindowState(FLAG_VSYNC_HINT);
  } else {
    ClearWindowState(FLAG_VSYNC_HINT);
  }
}

RaylibContext::~RaylibContext() { CloseWindow(); }

Game::~Game()
{
  // Ensure any last-minute visual settings changes are persisted.
  if (m_visualPrefsDirty) {
    saveVisualPrefsFile(m_visualPrefsPath, false);
  }

  if (m_3dPreviewTex.id != 0) {
    UnloadTexture(m_3dPreviewTex);
    m_3dPreviewTex = Texture2D{};
  }

  unloadWorldRenderTarget();
  m_postFxPipeline.shutdown();
  unloadSaveMenuThumbnails();

  ui::Shutdown();
}

void Game::invalidateHydrology()
{
  m_seaFloodDirty = true;
  m_pondingDirty = true;
  m_evacDirty = true;
}

void Game::invalidateAnalysisLayers()
{
  m_airPollutionDirty = true;
  m_skyViewDirty = true;
  m_solarDirty = true;
  m_energyDirty = true;
  m_carbonDirty = true;
  m_crimeDirty = true;
  m_trafficSafetyDirty = true;
  m_transitAccessDirty = true;
  m_walkabilityDirty = true;
  m_jobsDirty = true;
  m_roadHealthDirty = true;
  m_livabilityDirty = true;
  m_hotspotsDirty = true;
  m_runoffDirty = true;
  m_runoffMitigationDirty = true;
}


Game::Game(Config cfg, GameStartupOptions startup)
    : m_cfg(cfg)
    , m_rl(cfg, "ProcIsoCity")
    , m_world()
    , m_sim(SimConfig{})
    , m_renderer(cfg.tileWidth, cfg.tileHeight, cfg.seed)
{
  // Prevent accidental Alt+F4 style exits while testing.
  SetExitKey(KEY_NULL);

  // Startup overrides (used by the executable bootstrap).
  if (!startup.visualPrefsPath.empty()) {
    m_visualPrefsPath = startup.visualPrefsPath;
  }

  // Blueprint stamping: by default, disallow placing non-road overlays onto water.
  m_blueprintApplyOpt.force = false;

  // Track the initial window geometry so fullscreen/borderless toggles can
  // restore back to the original windowed size/position.
  {
    const Vector2 pos = GetWindowPosition();
    m_windowedX = static_cast<int>(pos.x);
    m_windowedY = static_cast<int>(pos.y);
    m_windowedW = GetScreenWidth();
    m_windowedH = GetScreenHeight();
  }

  // Initialize UI scaling.
  if (m_uiScaleAuto) {
    m_uiScale = computeAutoUiScale(m_windowedW, m_windowedH);
  }

  // Initialize world render scaling (resolution scale) from config.
  m_worldRenderScaleAuto = m_cfg.worldRenderScaleAuto;
  m_worldRenderScale = clampWorldRenderScale(m_cfg.worldRenderScale);
  m_worldRenderScaleMin = clampWorldRenderScale(m_cfg.worldRenderScaleMin);
  m_worldRenderScaleMax = clampWorldRenderScale(m_cfg.worldRenderScaleMax);
  if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
    std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
  }
  m_worldRenderTargetFps = std::max(15, m_cfg.worldRenderTargetFps);
  m_worldRenderFilterPoint = m_cfg.worldRenderFilterPoint;
  m_mergedZoneBuildings = m_cfg.mergedZoneBuildings;
  if (m_worldRenderScaleAuto) {
    // Prefer best quality first; let the auto-scaler reduce resolution only
    // if we can't hit the target FPS.
    m_worldRenderScale = m_worldRenderScaleMax;
  }

  // Elevation settings derived from config.
  m_elevDefault.maxPixels = static_cast<float>(m_cfg.tileHeight) * std::max(0.0f, m_cfg.elevationScale);
  m_elevDefault.quantizeSteps = std::max(0, m_cfg.elevationSteps);
  m_elevDefault.flattenWater = true;
  m_elev = m_elevDefault;
  m_renderer.setElevationSettings(m_elev);

  // Post FX shaders are tiny; initialize once so toggling is instant.
  m_postFxPipeline.init();

  // Load persisted display/visual preferences if present (defaults to isocity_visual.json).
  if (startup.loadVisualPrefs && !m_visualPrefsPath.empty() && std::filesystem::exists(m_visualPrefsPath)) {
    loadVisualPrefsFile(m_visualPrefsPath, false);
  }
  // Prime change detection for autosave (even if no prefs file exists yet).
  m_visualPrefsLastSnapshot = captureVisualPrefs();

  // --- Software 3D preview/export defaults ---
  // Keep the in-game defaults modest so the feature is usable without long stalls.
  // The dev console command (render3d) can request higher resolutions / SSAA.
  m_pendingRender3DCfg = Render3DConfig{};
  m_pendingRender3DCfg.width = 1600;
  m_pendingRender3DCfg.height = 900;
  m_pendingRender3DCfg.supersample = 1;
  m_pendingRender3DCfg.projection = Render3DConfig::Projection::IsometricOrtho;
  m_pendingRender3DCfg.yawDeg = 45.0f;
  m_pendingRender3DCfg.pitchDeg = 35.264f;
  m_pendingRender3DCfg.autoFit = true;
  m_pendingRender3DCfg.drawOutlines = true;

  m_3dPreviewCfg = m_pendingRender3DCfg;
  m_3dPreviewCfg.width = 384;
  m_3dPreviewCfg.height = 216;
  m_3dPreviewCfg.supersample = 1;
  m_3dPreviewCfg.outlineAlpha = 0.70f;

  // --- Public services optimizer defaults (experimental; in-game integration scaffolding) ---
  m_servicesPlanCfg = ServiceOptimizerConfig{};
  m_servicesPlanCfg.modelCfg = m_sim.servicesModel();
  m_servicesPlanCfg.modelCfg.enabled = true;
  // Keep the outside-connection rule consistent with the main sim by default.
  m_servicesPlanCfg.modelCfg.requireOutsideConnection = m_sim.config().requireOutsideConnection;
  m_servicesPlanCfg.type = ServiceType::Education;
  m_servicesPlanCfg.facilitiesToAdd = 8;
  m_servicesPlanCfg.facilityLevel = 1;
  m_servicesPlanCfg.candidateLimit = 700;
  m_servicesPlanCfg.requireEmptyLand = true;
  m_servicesPlanCfg.requireStableAccessRoad = true;
  m_servicesPlanCfg.minSeparationMilli = 0;
  m_servicesPlanCfg.considerOnlySameTypeExisting = true;


  // --- Policy advisor defaults (async policy optimizer) ---
  m_policyOptCfg = PolicyOptimizerConfig{};
  m_policyOptCfg.method = PolicyOptMethod::CEM;
  m_policyOptCfg.evalDays = 45;
  m_policyOptCfg.iterations = 18;
  m_policyOptCfg.population = 48;
  m_policyOptCfg.elites = 8;
  m_policyOptCfg.topK = 12;
  m_policyOptCfg.exploreProb = 0.12f;
  m_policyOptCfg.maxExhaustiveCandidates = 400000;

  m_policyOptPreset = 0;
  m_policyOptTaxRadius = 3;
  m_policyOptMaintRadius = 1;

  resetWorld(m_cfg.seed);

  // --- City Lab defaults (headless tools integrated into the interactive app) ---
  m_labMineCfg.w = m_world.width();
  m_labMineCfg.h = m_world.height();
  m_labMineCfg.samples = 200;
  m_labMineCfg.days = 120;
  m_labMineCfg.seedStart = 1;
  m_labMineCfg.seedStep = 1;
  m_labMineCfg.objective = m_mineObjective;
  m_labMineCfg.hydrologyEnabled = true;
  m_labMineCfg.seaLevelOverride = std::numeric_limits<float>::quiet_NaN();
  m_labMineBakeDays = m_labMineCfg.days;

  // Camera
  m_camera.zoom = 1.0f;
  m_camera.rotation = 0.0f;
  m_camera.offset = Vector2{m_windowedW * 0.5f, m_windowedH * 0.5f};

  const Vector2 center = TileToWorldCenterElevated(
      m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2, static_cast<float>(m_cfg.tileWidth),
      static_cast<float>(m_cfg.tileHeight), m_elev);
  m_camera.target = center;

  ui::Init(m_cfg.seed);

  setupDevConsole();
}

void Game::setupDevConsole()
{
  // Keep the console usable in Release builds: it is primarily a dev/debug
  // productivity tool, but also enables power-users to script common actions.
  m_console.clearLog();
  m_console.print("ProcIsoCity dev console (F4). Type 'help' for commands.");

  auto toLower = [](std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return s;
  };

  auto joinArgs = [](const DevConsole::Args& args, std::size_t start) {
    std::string out;
    for (std::size_t i = start; i < args.size(); ++i) {
      if (!out.empty()) out.push_back(' ');
      out += args[i];
    }
    return out;
  };

  auto parseI64 = [](const std::string& s, long long& out) -> bool {
    try {
      std::size_t idx = 0;
      out = std::stoll(s, &idx, 10);
      return idx == s.size();
    } catch (...) {
      return false;
    }
  };

  auto parseI32 = [](const std::string& s, int& out) -> bool {
    try {
      std::size_t idx = 0;
      const long long v = std::stoll(s, &idx, 10);
      if (idx != s.size()) return false;
      if (v < static_cast<long long>(std::numeric_limits<int>::min()) ||
          v > static_cast<long long>(std::numeric_limits<int>::max())) {
        return false;
      }
      out = static_cast<int>(v);
      return true;
    } catch (...) {
      return false;
    }
  };

  auto parseU64 = [](const std::string& s, std::uint64_t& out) -> bool {
    try {
      std::size_t idx = 0;
      const unsigned long long v = std::stoull(s, &idx, 10);
      if (idx != s.size()) return false;
      out = static_cast<std::uint64_t>(v);
      return true;
    } catch (...) {
      return false;
    }
  };

  auto parseF32 = [](const std::string& s, float& out) -> bool {
    try {
      std::size_t idx = 0;
      out = std::stof(s, &idx);
      return idx == s.size();
    } catch (...) {
      return false;
    }
  };

  auto heatmapName = [](HeatmapOverlay h) -> const char* {
    switch (h) {
    case HeatmapOverlay::Off: return "off";
    case HeatmapOverlay::LandValue: return "land";
    case HeatmapOverlay::ParkAmenity: return "park";
    case HeatmapOverlay::WaterAmenity: return "water";
    case HeatmapOverlay::Pollution: return "pollution";
    case HeatmapOverlay::TrafficSpill: return "traffic";
    case HeatmapOverlay::ServicesOverall: return "services";
    case HeatmapOverlay::ServicesEducation: return "services_edu";
    case HeatmapOverlay::ServicesHealth: return "services_health";
    case HeatmapOverlay::ServicesSafety: return "services_safety";
    case HeatmapOverlay::FireRisk: return "fire_risk";
    case HeatmapOverlay::FloodDepth: return "flood";
    case HeatmapOverlay::PondingDepth: return "pond";
    case HeatmapOverlay::EvacuationTime: return "evac";
    case HeatmapOverlay::EvacuationUnreachable: return "evac_unreach";
    case HeatmapOverlay::EvacuationFlow: return "evac_flow";
    case HeatmapOverlay::AirPollution: return "air_pollution";
    case HeatmapOverlay::AirPollutionEmission: return "air_emission";
    case HeatmapOverlay::SkyView: return "sky_view";
    case HeatmapOverlay::CanyonConfinement: return "canyon_confinement";
    case HeatmapOverlay::SolarExposure: return "solar_exposure";
    case HeatmapOverlay::SolarPotential: return "solar_potential";
    case HeatmapOverlay::EnergyDemand: return "energy_demand";
    case HeatmapOverlay::EnergySolar: return "energy_solar";
    case HeatmapOverlay::EnergyBalance: return "energy_balance";
    case HeatmapOverlay::CarbonEmission: return "carbon_emission";
    case HeatmapOverlay::CarbonSequestration: return "carbon_sequestration";
    case HeatmapOverlay::CarbonBalance: return "carbon_balance";

    case HeatmapOverlay::CrimeRisk: return "crime_risk";
    case HeatmapOverlay::PoliceAccess: return "police_access";

    case HeatmapOverlay::TrafficCrashRisk: return "traffic_crash_risk";
    case HeatmapOverlay::TrafficCrashExposure: return "traffic_crash_exposure";
    case HeatmapOverlay::TrafficCrashPriority: return "traffic_crash_priority";


    case HeatmapOverlay::TransitAccess: return "transit_access";
    case HeatmapOverlay::TransitModeSharePotential: return "transit_mode_share_potential";
    case HeatmapOverlay::Walkability: return "walkability";
    case HeatmapOverlay::WalkabilityPark: return "walkability_park";
    case HeatmapOverlay::WalkabilityRetail: return "walkability_retail";
    case HeatmapOverlay::WalkabilityEducation: return "walkability_education";
    case HeatmapOverlay::WalkabilityHealth: return "walkability_health";
    case HeatmapOverlay::WalkabilitySafety: return "walkability_safety";
    case HeatmapOverlay::JobAccess: return "job_access";
    case HeatmapOverlay::JobOpportunity: return "job_opportunity";
    case HeatmapOverlay::RoadCentrality: return "road_centrality";
    case HeatmapOverlay::RoadVulnerability: return "road_vulnerability";
    case HeatmapOverlay::RoadBypass: return "road_bypass";
    case HeatmapOverlay::Livability: return "livability";
    case HeatmapOverlay::InterventionPriority: return "intervention_priority";
    case HeatmapOverlay::LivabilityHotspot: return "livability_hotspot";
    case HeatmapOverlay::InterventionHotspot: return "intervention_hotspot";
    case HeatmapOverlay::RunoffPollution: return "runoff_pollution";
    case HeatmapOverlay::RunoffPollutionLoad: return "runoff_load";
    case HeatmapOverlay::RunoffMitigationPriority: return "runoff_mitigation_priority";
    case HeatmapOverlay::RunoffMitigationPlan: return "runoff_mitigation_plan";
    default: return "?";
    }
  };

  // --- help/utility ---
  m_console.registerCommand(
      "help", "help [cmd]  - list commands or show help for one command",
      [toLower](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          const std::string key = toLower(args[0]);
          const auto it = c.commands().find(key);
          if (it == c.commands().end()) {
            c.print("Unknown command: " + args[0]);
            return;
          }
          c.print(it->first + "  - " + it->second.help);
          return;
        }

        c.print("Commands:");
        for (const std::string& name : c.commandOrder()) {
          const std::string key = toLower(name);
          const auto it = c.commands().find(key);
          if (it != c.commands().end()) {
            c.print("  " + name + "  - " + it->second.help);
          }
        }
      });

  m_console.registerCommand("clear", "clear      - clear the console output",
                            [](DevConsole& c, const DevConsole::Args&) { c.clearLog(); });

  m_console.registerCommand(
      "echo", "echo <text...>  - print text", [joinArgs](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) return;
        c.print(joinArgs(args, 0));
      });

  m_console.registerCommand(
      "shader_reload",
      "shader_reload  - reload on-disk GLSL overrides (postfx, taa, bloom_extract, bloom_blur, weatherfx, materialfx, volcloud, cloudmask)",
      [this](DevConsole& c, const DevConsole::Args&) {
        const ShaderOverrideSearch s = FindShaderOverrideDir();
        if (!s.dir.empty()) {
          c.print("Shader override dir: " + s.dir.string());
        } else {
          c.print("Shader override dir: (not found; using embedded fallbacks)");
        }

        const bool postOk = m_postFxPipeline.reload();
        c.print(std::string("PostFX: ") + (postOk ? "reloaded" : "reload failed"));

        m_renderer.reloadShaderOverrides();
        c.print("Renderer: shader overrides reloaded");
      });

  // ScriptRunner bridge: run the same deterministic headless scripts used by the CLI tools
  // directly inside the interactive app.
  //
  // This is intentionally implemented as a "transaction": we clone the current world/sim
  // into a ScriptRunner, run the script, and only commit the new state back to the game if
  // the script succeeds. This makes experimentation safer (a failing script won't leave the
  // world half-mutated).
  m_console.registerCommand(
      "script",
      "script [vars|clear|-e <cmd...>|[-v] <file>]  - run a ScriptRunner script and apply the result",
      [this, toLower, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        auto printUsage = [&]() {
          c.print("Usage:");
          c.print("  script <file>           Run a script file and apply the result");
          c.print("  script -v <file>        Same, but verbose (prints progress/info)");
          c.print("  script -e <cmd...>      Evaluate a single script command line");
          c.print("  script vars             Show persistent script variables");
          c.print("  script clear            Clear persistent script variables");
        };

        if (args.empty()) {
          printUsage();
          return;
        }

        const std::string a0 = toLower(args[0]);
        if (a0 == "vars") {
          if (m_consoleScriptVars.empty()) {
            c.print("(no script vars)");
            return;
          }
          c.print("script vars:");
          for (const auto& kv : m_consoleScriptVars) {
            c.print("  " + kv.first + "=" + kv.second);
          }
          return;
        }

        if (a0 == "clear" || a0 == "reset") {
          m_consoleScriptVars.clear();
          m_consoleScriptRunIndex = 0;
          c.print("script vars cleared");
          return;
        }

        bool eval = false;
        bool verbose = false;
        std::string path;
        std::string text;

        if (a0 == "-e" || a0 == "eval") {
          eval = true;
          if (args.size() < 2) {
            c.print("Usage: script -e <cmd...>");
            return;
          }
          text = joinArgs(args, 1);
        } else if (a0 == "-v" || a0 == "--verbose") {
          verbose = true;
          if (args.size() < 2) {
            c.print("Usage: script -v <file>");
            return;
          }
          path = args[1];
        } else {
          path = args[0];
        }

        // Ensure we don't run scripts mid-paint-stroke (keeps undo/state consistent).
        endPaintStroke();

        // Seed runner with a copy of the current game state so the script has a starting point.
        ScriptRunner runner;
        ScriptCallbacks cb;
        cb.print = [&c](const std::string& line) { c.print(line); };
        cb.info = [&c](const std::string& line) { c.print(line); };
        cb.error = [&c](const std::string& line) { c.print(std::string("[script] ") + line); };
        runner.setCallbacks(std::move(cb));

        ScriptRunOptions opt;
        opt.quiet = !verbose;
        runner.setOptions(opt);

        runner.state().vars = m_consoleScriptVars;
        runner.state().runIndex = m_consoleScriptRunIndex;

        runner.state().procCfg = m_procCfg;
        runner.state().simCfg = m_sim.config();
        runner.state().sim = m_sim;
        runner.state().world = m_world;
        runner.state().hasWorld = true;
        runner.state().dirtyDerived = true;

        auto commitState = [&](ScriptRunnerState& st, const char* toastLabel) {
          // Persist vars across runs.
          m_consoleScriptVars = st.vars;
          m_consoleScriptRunIndex = st.runIndex;

          // Commit the new world/sim config to the game.
          const std::uint64_t prevSeed = m_cfg.seed;

          m_world = std::move(st.world);
          m_procCfg = st.procCfg;
          m_sim = st.sim;
          m_sim.resetTimer();

          m_renderer.markMinimapDirty();
          m_renderer.markBaseCacheDirtyAll();
          m_roadGraphDirty = true;
          m_trafficDirty = true;
          m_goodsDirty = true;
          m_landValueDirty = true;
          m_transitPlanDirty = true;
          m_transitVizDirty = true;
          m_evacDirty = true;
          m_roadUpgradePlanDirty = true;
          m_roadUpgradeSelectedMaskDirty = true;
          invalidateHydrology();
          m_vehiclesDirty = true;
          m_vehicles.clear();

          // Road-resilience caches/suggestions are tied to the road graph.
          m_resilienceDirty = true;
          m_resilienceBypassesDirty = true;
          m_resilienceBypasses.clear();
          m_roadGraphTileToNode.clear();
          m_roadGraphTileToEdge.clear();

          // Keep flood overlay defaults in sync with the current proc-gen thresholds.
          m_seaLevel = std::clamp(m_procCfg.waterLevel, 0.0f, 1.0f);

          // Deterministic vehicle RNG seed per world seed.
          m_vehicleRngState = (m_world.seed() ^ 0x9E3779B97F4A7C15ULL);

          // Script commit invalidates undo/redo history (treat as a new session).
          m_history.clear();
          m_painting = false;

          // Clear inspect selection/debug overlays.
          m_inspectSelected.reset();
          m_inspectPath.clear();
          m_inspectPathCost = 0;
          m_inspectInfo.clear();

          // Clear any previously computed navigation route.
          m_wayfindingDirty = true;
          clearWayfindingRoute();

          // Clear any in-progress road drag preview.
          m_roadDragActive = false;
          m_roadDragStart.reset();
          m_roadDragEnd.reset();
          m_roadDragPath.clear();
          m_roadDragBuildCost = 0;
          m_roadDragUpgradeTiles = 0;
          m_roadDragBridgeTiles = 0;
          m_roadDragMoneyCost = 0;
          m_roadDragValid = false;

          // Keep config in sync with the new world, so regen & camera recenter behave.
          m_cfg.mapWidth = m_world.width();
          m_cfg.mapHeight = m_world.height();
          m_cfg.seed = m_world.seed();

          // If the seed changed, also vary procedural textures per seed.
          if (m_cfg.seed != prevSeed) {
            m_renderer.rebuildTextures(m_cfg.seed);
          }

          SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(m_cfg.seed)));

          // Any script commit invalidates the software 3D preview.
          m_3dPreviewDirty = true;
          m_3dPreviewTimer = 0.0f;

          // Recenter camera on the updated map.
          m_camera.target = TileToWorldCenterElevated(m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2,
                                                      static_cast<float>(m_cfg.tileWidth),
                                                      static_cast<float>(m_cfg.tileHeight), m_elev);

          // Make HUD stats immediately correct (without waiting for the next sim tick).
          m_sim.refreshDerivedStats(m_world);
          clearHistory();
          {
            Stats s = m_world.stats();
            const int grant = applyCityChallenges(s);
            if (grant != 0) m_world.stats().money += grant;
            recordHistorySample(s);
          }

          if (toastLabel && toastLabel[0] != '\0') {
            showToast(toastLabel);
          } else {
            showToast("Script applied");
          }

          if (m_showSaveMenu) refreshSaveMenu();
        };

        const bool ok = eval ? runner.runText(text, "<console>") : runner.runFile(path);
        if (!ok) {
          const std::string err = runner.lastError().empty() ? "Script failed" : runner.lastError();
          c.print(err);
          if (!runner.lastErrorPath().empty() && runner.lastErrorLine() > 0) {
            c.print("  at " + runner.lastErrorPath() + ":" + std::to_string(runner.lastErrorLine()));
          }
          showToast("Script failed", 4.0f);
          return;
        }

        if (eval) {
          commitState(runner.state(), "Script eval applied");
        } else {
          commitState(runner.state(), "Script applied");
        }
      });



  // --- deterministic AutoBuild "city bot" ---
  // Bridges the headless AutoBuild module into the interactive app.
  m_console.registerCommand(
    "bot",
    "bot <key> <value> | bot show | bot reset | bot ui  - configure AutoBuild (city bot)",
    [this, toLower](DevConsole& c, const DevConsole::Args& args)
      {
        auto printUsage = [&c]()
          {
            c.print("Usage:");
            c.print("  bot show            Show current bot config");
            c.print("  bot reset           Reset bot config to defaults");
            c.print("  bot ui              Toggle the AutoBuild panel");
            c.print("  bot <key> <value>   Set one config key");
          };

        if (args.empty()) {
          printUsage();
          return;
        }

        const std::string sub = toLower(args[0]);
        if (sub == "show") {
          const AutoBuildConfig& cfg = m_autoBuildCfg;
          c.print("bot config:");
          c.print(TextFormat("  zonesPerDay=%d", cfg.zonesPerDay));
          c.print(TextFormat("  zoneClusterMaxTiles=%d", cfg.zoneClusterMaxTiles));
          c.print(TextFormat("  roadsPerDay=%d", cfg.roadsPerDay));
          c.print(TextFormat("  parksPerDay=%d", cfg.parksPerDay));
          c.print(TextFormat("  useParkOptimizer=%s", cfg.useParkOptimizer ? "true" : "false"));
          c.print(TextFormat("  roadLevel=%d", cfg.roadLevel));
          c.print(TextFormat("  useRoadPlanner=%s", cfg.useRoadPlanner ? "true" : "false"));
          c.print(TextFormat("  allowBridges=%s", cfg.allowBridges ? "true" : "false"));
          c.print(TextFormat("  minMoneyReserve=%d", cfg.minMoneyReserve));
          c.print(TextFormat("  parkPerZoneTiles=%d", cfg.parkPerZoneTiles));
          c.print(TextFormat("  autoUpgradeRoads=%s", cfg.autoUpgradeRoads ? "true" : "false"));
          c.print(TextFormat("  congestionUpgradeThreshold=%.3f", cfg.congestionUpgradeThreshold));
          c.print(TextFormat("  roadUpgradesPerDay=%d", cfg.roadUpgradesPerDay));
          c.print(TextFormat("  autoBuildResilienceBypasses=%s", cfg.autoBuildResilienceBypasses ? "true" : "false"));
          c.print(TextFormat("  resilienceBypassCongestionThreshold=%.3f", cfg.resilienceBypassCongestionThreshold));
          c.print(TextFormat("  resilienceBypassesPerDay=%d", cfg.resilienceBypassesPerDay));
          c.print(TextFormat("  landValueRecalcDays=%d", cfg.landValueRecalcDays));
          c.print(TextFormat("  respectOutsideConnection=%s", cfg.respectOutsideConnection ? "true" : "false"));
          c.print(TextFormat("  ensureOutsideConnection=%s", cfg.ensureOutsideConnection ? "true" : "false"));
          c.print(TextFormat("  maxRoadSpurLength=%d", cfg.maxRoadSpurLength));
          return;
        }

        if (sub == "reset") {
          m_autoBuildCfg = AutoBuildConfig{};
          m_autoBuildPanelScroll = 0;
          c.print("bot: reset to defaults");
          showToast("Bot config reset", 2.0f);
          return;
        }

        if (sub == "ui" || sub == "panel") {
          m_showAutoBuildPanel = !m_showAutoBuildPanel;
          ui::ClearActiveWidget();
          showToast(m_showAutoBuildPanel ? "Bot panel: ON" : "Bot panel: OFF", 2.0f);
          return;
        }

        if (args.size() < 2) {
          printUsage();
          return;
        }

        std::string err;
        if (!ParseAutoBuildKey(args[0], args[1], m_autoBuildCfg, err)) {
          c.print("bot: " + err);
          showToast("Bot: invalid config", 2.0f);
          return;
        }

        c.print("bot: set " + args[0] + "=" + args[1]);
      });

  m_console.registerCommand(
    "autobuild",
    "autobuild <days> [key=value ...]  - run AutoBuild on the current world state",
    [this, toLower, parseI32](DevConsole& c, const DevConsole::Args& args)
      {
        auto printUsage = [&c]()
          {
            c.print("Usage:");
            c.print("  autobuild <days>");
            c.print("  autobuild <days> key=value ...");
            c.print("  autobuild ui");
            c.print("");
            c.print("Example:");
            c.print("  autobuild 120 zonesPerDay=4 roadsPerDay=2 useRoadPlanner=true");
            c.print("");
            c.print("Config keys are case-insensitive. See AutoBuild.hpp for the full list.");
          };

        if (args.empty()) {
          printUsage();
          return;
        }

        const std::string a0 = toLower(args[0]);
        if (a0 == "ui" || a0 == "panel") {
          m_showAutoBuildPanel = !m_showAutoBuildPanel;
          ui::ClearActiveWidget();
          showToast(m_showAutoBuildPanel ? "Bot panel: ON" : "Bot panel: OFF", 2.0f);
          return;
        }

        int days = 0;
        if (!parseI32(args[0], days) || days <= 0) {
          c.print("autobuild: invalid days: " + args[0]);
          return;
        }

        AutoBuildConfig cfg = m_autoBuildCfg;
        for (std::size_t i = 1; i < args.size(); ++i) {
          const std::string& tok = args[i];
          const std::size_t eq = tok.find('=');
          if (eq == std::string::npos) {
            c.print("autobuild: expected key=value, got: " + tok);
            return;
          }
          const std::string key = tok.substr(0, eq);
          const std::string val = tok.substr(eq + 1);
          std::string err;
          if (!ParseAutoBuildKey(key, val, cfg, err)) {
            c.print("autobuild: " + err);
            return;
          }
        }

        ui::ClearActiveWidget();
        runAutoBuild(days, cfg, "AutoBuild");

        const AutoBuildReport& rep = m_autoBuildLastReport;
        c.print(TextFormat(
          "autobuild: simulated %d/%d days | roads %d (+%d up) | zones %d | parks %d | fails %d",
          rep.daysSimulated, rep.daysRequested, rep.roadsBuilt, rep.roadsUpgraded, rep.zonesBuilt, rep.parksBuilt,
          rep.failedBuilds));
      });
  // --- turn-by-turn navigation / wayfinding ---
  //
  // This bridges the headless `StreetNames` + `Wayfinding` modules into the interactive app.
  // Routes are computed via the dev console and rendered as a path overlay.
  m_console.registerCommand(
    "route",
    "route <from> <to> | route clear | route panel | route overlay [on|off] | route rebuild",
    [this, toLower](DevConsole& c, const DevConsole::Args& args)
      {
        auto printUsage = [&c]()
          {
            c.print("Usage:");
            c.print("  route \"from\" \"to\"");
            c.print("  route clear");
            c.print("  route panel");
            c.print("  route overlay [on|off]");
            c.print("  route rebuild");
            c.print("\nEndpoints can be:");
            c.print("  - \"x,y\" tile coordinates");
            c.print("  - an address (e.g. \"120 Oak St\")");
            c.print("  - an intersection (e.g. \"Oak St & Pine Ave\")");
            c.print("Use quotes if your endpoint contains spaces.");
          };

        if (args.empty()) {
          printUsage();
          return;
        }

        const std::string sub = toLower(args[0]);
        if (sub == "clear") {
          clearWayfindingRoute();
          showToast("Route cleared", 2.0f);
          return;
        }
        if (sub == "panel") {
          m_showWayfindingPanel = !m_showWayfindingPanel;
          showToast(m_showWayfindingPanel ? "Navigator panel shown" : "Navigator panel hidden", 2.0f);
          return;
        }
        if (sub == "overlay") {
          if (args.size() >= 2) {
            const std::string v = toLower(args[1]);
            if (v == "on" || v == "1" || v == "true") {
              m_showWayfindingOverlay = true;
            } else if (v == "off" || v == "0" || v == "false") {
              m_showWayfindingOverlay = false;
            } else {
              c.print("route overlay: expected on/off");
              return;
            }
          } else {
            m_showWayfindingOverlay = !m_showWayfindingOverlay;
          }
          showToast(m_showWayfindingOverlay ? "Route overlay ON" : "Route overlay OFF", 2.0f);
          return;
        }
        if (sub == "rebuild") {
          m_wayfindingDirty = true;
          ensureWayfindingUpToDate();
          c.print(TextFormat("Wayfinding cache rebuilt: %zu addresses, %zu index street keys.",
                             m_wayfindingAddresses.size(),
                             m_wayfindingIndex.streetKeys.size()));
          showToast("Navigator cache rebuilt", 2.0f);
          return;
        }

        if (args.size() < 2) {
          printUsage();
          return;
        }

        m_wayfindingFromQuery = args[0];
        m_wayfindingToQuery = args[1];

        ensureWayfindingUpToDate();

        if (!m_wayfindingIndex.ok) {
          c.print(std::string("Address index error: ") + m_wayfindingIndex.error);
          showToast("Navigator index error", 4.0f);
          return;
        }

        const GeocodeMatch from = GeocodeEndpoint(m_world, m_wayfindingStreets, m_wayfindingIndex, m_wayfindingFromQuery);
        if (!from.ok) {
          c.print(std::string("From endpoint error: ") + from.error);
          for (size_t i = 0; i < std::min<size_t>(from.suggestions.size(), 8); ++i) {
            c.print(std::string("  suggestion: ") + from.suggestions[i]);
          }
          showToast("From geocode failed", 4.0f);
          return;
        }

        const GeocodeMatch to = GeocodeEndpoint(m_world, m_wayfindingStreets, m_wayfindingIndex, m_wayfindingToQuery);
        if (!to.ok) {
          c.print(std::string("To endpoint error: ") + to.error);
          for (size_t i = 0; i < std::min<size_t>(to.suggestions.size(), 8); ++i) {
            c.print(std::string("  suggestion: ") + to.suggestions[i]);
          }
          showToast("To geocode failed", 4.0f);
          return;
        }

        m_wayfindingRoute = RouteBetweenEndpoints(m_world, m_wayfindingStreets, from.endpoint, to.endpoint);
        m_wayfindingFocusManeuver = -1;
        m_wayfindingFocusPath.clear();
        m_wayfindingManeuverFirst = 0;

        if (!m_wayfindingRoute.ok) {
          c.print(std::string("Route error: ") + m_wayfindingRoute.error);
          showToast("Route failed", 4.0f);
          return;
        }

        m_showWayfindingOverlay = true;
        m_showWayfindingPanel = true;

        c.print("Route computed:");
        c.print(std::string("  From: ") + m_wayfindingRoute.from.full);
        c.print(std::string("  To:   ") + m_wayfindingRoute.to.full);
        c.print(TextFormat("  Tiles: %zu, Maneuvers: %zu", m_wayfindingRoute.pathTiles.size(), m_wayfindingRoute.maneuvers.size()));

        const size_t maxPrint = 60;
        for (size_t i = 0; i < std::min(maxPrint, m_wayfindingRoute.maneuvers.size()); ++i) {
          c.print(TextFormat("  %2zu. %s", i + 1, m_wayfindingRoute.maneuvers[i].instruction.c_str()));
        }
        if (m_wayfindingRoute.maneuvers.size() > maxPrint) {
          c.print(TextFormat("  ... (%zu more)", m_wayfindingRoute.maneuvers.size() - maxPrint));
        }

        showToast("Route computed", 2.0f);
      });

  // --- procedural POV camera ---
  //
  // A cinematic ride-along camera that can follow the current route, or generate
  // a small procedural tour (TourPlanner) and ride it.
  m_console.registerCommand(
      "pov",
      "POV camera. Usage: pov [route|tour|roam|off|speed <v>|3d on|3d off]",
      [this, toLower](DevConsole& c, const DevConsole::Args& args) {
        auto printUsage = [&c]() {
          c.print("pov route            - ride along the current route (if any)");
          c.print("pov tour [start]     - generate a procedural tour and ride it");
          c.print("pov roam [len] [seed] - generate a procedural cruising drive");
          c.print("pov off              - stop and restore camera");
          c.print("pov speed <tiles/s>  - set ride speed");
          c.print("pov 3d on|off        - drive the software 3D preview camera");
        };

        if (args.empty()) {
          // Toggle route POV.
          if (m_pov.isActive()) {
            stopPov();
            showToast("POV stopped", 1.6f);
          } else {
            if (!m_wayfindingRoute.ok) {
              c.print("No route to ride. Compute one via: route \"from\" \"to\"");
              showToast("No route", 2.0f);
              return;
            }

            std::vector<PovMarker> markers;
            markers.reserve(m_wayfindingRoute.maneuvers.size());
            for (const RouteManeuver& man : m_wayfindingRoute.maneuvers) {
              if (man.type == "turn") {
                PovMarker mk;
                mk.pathIndex = std::clamp(man.pathStart, 0, static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1);
                mk.label = man.instruction;
                mk.holdSec = 0.35f;
                markers.push_back(std::move(mk));
              } else if (man.type == "arrive") {
                PovMarker mk;
                mk.pathIndex = std::clamp(man.pathStart, 0, static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1);
                mk.label = man.instruction;
                mk.holdSec = 0.8f;
                markers.push_back(std::move(mk));
              }
            }

            m_pov.setTitle("Route");

            const bool ok = m_pov.startFromPath(
                m_world, m_wayfindingRoute.pathTiles, markers,
                static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), m_elev,
                m_camera, GetScreenWidth(), GetScreenHeight(), static_cast<std::uint32_t>(m_world.seed()));
            if (ok) {
              showToast("POV ride started", 2.0f);
            } else {
              showToast("POV failed", 3.0f);
            }
          }
          return;
        }

        const std::string sub = toLower(args[0]);
        if (sub == "help" || sub == "?" || sub == "h") {
          printUsage();
          return;
        }

        if (sub == "off" || sub == "stop") {
          stopPov();
          showToast("POV stopped", 1.6f);
          return;
        }

        if (sub == "speed" && args.size() >= 2) {
          try {
            const float v = std::stof(args[1]);
            m_pov.config().speedTilesPerSec = std::clamp(v, 0.5f, 120.0f);
            c.print(TextFormat("POV speed set: %.2f tiles/s", m_pov.config().speedTilesPerSec));
          } catch (...) {
            c.print("Invalid speed value");
          }
          return;
        }

        if (sub == "3d" && args.size() >= 2) {
          const std::string v = toLower(args[1]);
          if (v == "on" || v == "1" || v == "true") {
            m_povDrive3DPreview = true;
            c.print("POV 3D preview drive: ON");
          } else if (v == "off" || v == "0" || v == "false") {
            m_povDrive3DPreview = false;
            c.print("POV 3D preview drive: OFF");
          } else {
            c.print("Usage: pov 3d on|off");
          }
          return;
        }

        if (sub == "route") {
          if (!m_wayfindingRoute.ok) {
            c.print("No route to ride. Compute one via: route \"from\" \"to\"");
            showToast("No route", 2.0f);
            return;
          }

          std::vector<PovMarker> markers;
          markers.reserve(m_wayfindingRoute.maneuvers.size());
          for (const RouteManeuver& man : m_wayfindingRoute.maneuvers) {
            if (man.type == "turn") {
              PovMarker mk;
              mk.pathIndex = std::clamp(man.pathStart, 0, static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1);
              mk.label = man.instruction;
              mk.holdSec = 0.35f;
              markers.push_back(std::move(mk));
            } else if (man.type == "arrive") {
              PovMarker mk;
              mk.pathIndex = std::clamp(man.pathStart, 0, static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1);
              mk.label = man.instruction;
              mk.holdSec = 0.8f;
              markers.push_back(std::move(mk));
            }
          }

          stopPov();
          m_pov.setTitle("Route");
          const bool ok = m_pov.startFromPath(
              m_world, m_wayfindingRoute.pathTiles, markers,
              static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), m_elev,
              m_camera, GetScreenWidth(), GetScreenHeight(), static_cast<std::uint32_t>(m_world.seed()));
          showToast(ok ? "POV route started" : "POV failed", 2.0f);
          return;
        }

        if (sub == "tour") {
          stopPov();

          TourConfig cfg;
          cfg.maxStops = 6;

          std::string start = m_wayfindingFromQuery;
          if (args.size() >= 2) {
            start = args[1];
          }

          const TourPlan tour = BuildProceduralTour(m_world, start, cfg);
          const bool ok = m_pov.startFromTour(
              m_world, tour,
              static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), m_elev,
              m_camera, GetScreenWidth(), GetScreenHeight(), static_cast<std::uint32_t>(tour.seed ? tour.seed : m_world.seed()));
          showToast(ok ? "POV tour started" : "POV tour failed", 2.4f);
          return;
        }

        if (sub == "roam") {
          stopPov();

          int len = m_povRoamCfg.length;
          if (args.size() >= 2) {
            try {
              len = std::stoi(args[1]);
            } catch (...) {
              c.print("pov roam: invalid length");
              return;
            }
          }

          // Optional explicit seed for reproducible runs. Otherwise, we increment an internal seed.
          std::uint32_t seed = m_povRoamSeed + 1u;
          if (args.size() >= 3) {
            try {
              seed = static_cast<std::uint32_t>(std::stoul(args[2]));
            } catch (...) {
              c.print("pov roam: invalid seed");
              return;
            }
          }
          if (seed == 0) {
            seed = 1u;
          }

          const bool ok = startRoamPov(seed, len);
          if (ok) {
            c.print(std::string("pov roam: ") + m_povRoamDebug);
          } else {
            c.print("pov roam: failed to generate a path (no roads?)");
          }
          showToast(ok ? "POV roam started" : "POV roam failed", 2.4f);
          return;
        }


        printUsage();
      });

  // --- deterministic replay capture/playback ---
  //
  // The core replay system already exists for headless tooling/CI, but exposing it
  // inside the interactive app makes it dramatically easier to:
  //   - capture a "minimal repro" of a bug
  //   - share deterministic sessions
  //   - regression test simulation changes (hash asserts)
  m_console.registerCommand(
      "replay",
      "replay <start|stop|clear|save|note|assert|snapshot|status|play>  - capture or play deterministic replays",
      [this, toLower, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        auto printUsage = [&]() {
          c.print("Usage:");
          c.print("  replay start [force]                 Start capturing from the current world");
          c.print("  replay stop                          Stop capturing (keeps captured data)");
          c.print("  replay clear                         Discard captured replay data");
          c.print("  replay save <file.isoreplay>         Save captured replay to disk");
          c.print("  replay note <text...>                Record a note marker");
          c.print("  replay assert [stats|nostats] [lbl]  Record a deterministic world hash assert");
          c.print("  replay snapshot [lbl]                Record a full snapshot event");
          c.print("  replay status                        Print capture state summary");
          c.print("  replay play <file> [--nostrict] [--noassert] [--nostats]  Load + apply a replay");
        };

        if (args.empty()) {
          printUsage();
          return;
        }

        const std::string sub = toLower(args[0]);

        if (sub == "start") {
          bool force = false;
          if (args.size() >= 2) {
            const std::string a1 = toLower(args[1]);
            force = (a1 == "force" || a1 == "-f" || a1 == "--force");
          }

          const Replay& existing = m_replayCapture.replay();
          const bool haveExisting = !existing.baseSave.empty() || !existing.events.empty();

          if (haveExisting && !m_replayCapture.active() && !force) {
            c.print("replay: a previous capture exists (use: replay save ... or replay start force)");
            return;
          }

          if (m_replayCapture.active() && !force) {
            c.print("replay: already capturing (use: replay start force)");
            return;
          }

          endPaintStroke();

          std::string err;
          if (!m_replayCapture.startFromWorld(m_world, m_procCfg, m_sim.config(), err)) {
            c.print("replay start failed: " + err);
            showToast("Replay start failed", 4.0f);
            return;
          }

          const std::uint64_t h = HashWorld(m_world, true);
          c.print(TextFormat("replay: started (base hash 0x%llx)", static_cast<unsigned long long>(h)));
          showToast("Replay capture started");
          return;
        }

        if (sub == "stop") {
          if (!m_replayCapture.active()) {
            c.print("replay: not capturing");
            return;
          }
          m_replayCapture.stop();
          c.print(TextFormat("replay: stopped (%zu events captured)", m_replayCapture.replay().events.size()));
          showToast("Replay capture stopped");
          return;
        }

        if (sub == "clear" || sub == "discard" || sub == "reset") {
          const std::size_t prevEvents = m_replayCapture.replay().events.size();
          m_replayCapture.clear();
          c.print(TextFormat("replay: cleared (%zu events discarded)", prevEvents));
          showToast("Replay cleared");
          return;
        }

        if (sub == "save") {
          if (args.size() < 2) {
            c.print("Usage: replay save <file.isoreplay>");
            return;
          }
          const std::string path = args[1];
          std::string err;
          if (!m_replayCapture.saveToFile(path, err)) {
            c.print("replay save failed: " + err);
            showToast("Replay save failed", 4.0f);
            return;
          }
          c.print("wrote replay -> " + path);
          showToast("Replay saved");
          return;
        }

        if (sub == "note") {
          if (!m_replayCapture.active()) {
            c.print("replay: not capturing (start first)");
            return;
          }
          if (args.size() < 2) {
            c.print("Usage: replay note <text...>");
            return;
          }
          m_replayCapture.recordNote(joinArgs(args, 1));
          c.print("replay: note recorded");
          return;
        }

        if (sub == "assert") {
          if (!m_replayCapture.active()) {
            c.print("replay: not capturing (start first)");
            return;
          }

          bool includeStats = true;
          std::size_t labelStart = 1;
          if (args.size() >= 2) {
            const std::string a1 = toLower(args[1]);
            if (a1 == "nostats" || a1 == "no_stats" || a1 == "no-stats") {
              includeStats = false;
              labelStart = 2;
            } else if (a1 == "stats") {
              includeStats = true;
              labelStart = 2;
            }
          }
          const std::string label = (labelStart < args.size()) ? joinArgs(args, labelStart) : std::string();
          m_replayCapture.recordAssertHash(m_world, includeStats, label);
          const std::uint64_t h = HashWorld(m_world, includeStats);
          if (label.empty()) {
            c.print(TextFormat("replay: assert recorded (0x%llx)", static_cast<unsigned long long>(h)));
          } else {
            c.print(TextFormat("replay: assert recorded (0x%llx)  label=%s", static_cast<unsigned long long>(h),
                               label.c_str()));
          }
          return;
        }

        if (sub == "snapshot") {
          if (!m_replayCapture.active()) {
            c.print("replay: not capturing (start first)");
            return;
          }
          std::string err;
          if (!m_replayCapture.recordSnapshotFromWorld(m_world, m_procCfg, m_sim.config(), err)) {
            c.print("replay snapshot failed: " + err);
            showToast("Replay snapshot failed", 4.0f);
            return;
          }
          if (args.size() >= 2) {
            m_replayCapture.recordNote(std::string("snapshot: ") + joinArgs(args, 1));
          }
          c.print("replay: snapshot recorded");
          return;
        }

        if (sub == "status") {
          const Replay& r = m_replayCapture.replay();
          c.print(TextFormat("replay: active=%s  events=%zu  baseSave=%zu bytes", m_replayCapture.active() ? "true" : "false",
                             r.events.size(), r.baseSave.size()));
          const std::uint64_t h0 = HashWorld(m_world, false);
          const std::uint64_t h1 = HashWorld(m_world, true);
          c.print(TextFormat("world hash: tiles=0x%llx  tiles+stats=0x%llx", static_cast<unsigned long long>(h0),
                             static_cast<unsigned long long>(h1)));
          return;
        }

        if (sub == "play" || sub == "load") {
          if (args.size() < 2) {
            c.print("Usage: replay play <file.isoreplay> [--nostrict] [--noassert] [--nostats]");
            return;
          }

          const std::string path = args[1];
          bool strictPatches = true;
          bool strictAsserts = true;
          bool wantStats = true;

          for (std::size_t i = 2; i < args.size(); ++i) {
            const std::string f = toLower(args[i]);
            if (f == "--nostrict" || f == "nostrict" || f == "--force") {
              strictPatches = false;
            } else if (f == "--noassert" || f == "noassert" || f == "--no-assert") {
              strictAsserts = false;
            } else if (f == "--nostats" || f == "nostats" || f == "--no-stats") {
              wantStats = false;
            }
          }

          endPaintStroke();

          Replay replay;
          std::string err;
          if (!LoadReplayBinary(replay, path, err)) {
            c.print("replay load failed: " + err);
            showToast("Replay load failed", 4.0f);
            return;
          }

          World w;
          ProcGenConfig proc{};
          SimConfig sim{};
          std::vector<Stats> tickStats;
          std::vector<Stats>* outStats = (wantStats ? &tickStats : nullptr);

          if (!PlayReplay(replay, w, proc, sim, err, strictPatches, strictAsserts, outStats)) {
            c.print("replay play failed: " + err);
            showToast("Replay play failed", 4.0f);
            return;
          }

          const std::uint64_t finalHash = HashWorld(w, true);

          const std::string toast = std::string(TextFormat("Replay applied: %s", path.c_str()));
          adoptLoadedWorld(std::move(w), proc, sim, toast, outStats);

          c.print(TextFormat("replay: applied (%zu events) final_hash=0x%llx", replay.events.size(),
                             static_cast<unsigned long long>(finalHash)));
          return;
        }

        c.print("Unknown replay subcommand: " + args[0]);
        printUsage();
      });

  // --- world/simulation ---
  m_console.registerCommand(
      "seed", "seed <uint64>  - regenerate the world with a specific seed",
      [this, parseU64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: seed <uint64>");
          return;
        }
        std::uint64_t s = 0;
        if (!parseU64(args[0], s)) {
          c.print("Invalid seed: " + args[0]);
          return;
        }
        endPaintStroke();
        resetWorld(s);
        showToast(TextFormat("Seed: %llu", static_cast<unsigned long long>(s)));
        c.print(TextFormat("World regenerated with seed %llu", static_cast<unsigned long long>(s)));
      });



  // OpenStreetMap import bridge (mirrors the headless `proc_isocity_osmimport` tool).
  //
  // This lets you bring a real-world extract into the interactive app for:
  //  - quick visual inspection of imported geometry
  //  - running AutoBuild on top of real road networks
  //  - exporting replays / regression artifacts from a real map
  m_console.registerCommand(
      "osm",
      "osm import <file.osm> [options]  - import OpenStreetMap (OSM XML) into a new world (try: osm import extract.osm --full --autobuild-days 120)",
      [this, toLower, parseI32, parseU64](DevConsole& c, const DevConsole::Args& args) {
        auto printUsage = [&]() {
          c.print("Usage:");
          c.print("  osm import <file.osm> [--full] [--size WxH] [--seed u64] [--meters-per-tile f] [--padding n]");
          c.print("            [--roads 0|1] [--water 0|1] [--landuse 0|1] [--parks 0|1] [--buildings 0|1] [--overwrite 0|1]");
          c.print("            [--fixed-radius n] [--thicken-by-class 0|1] [--waterway-radius n]");
          c.print("            [--autobuild-days n] [--autobuild k=v]...");
          c.print("Notes:");
          c.print("  --full enables: water, landuse, parks, buildings.");
          c.print("  Paths with spaces must be quoted: osm import \"My Extract.osm\" --full");
          c.print("Examples:");
          c.print("  osm import extract.osm");
          c.print("  osm import extract.osm --full --size 512x512");
          c.print("  osm import extract.osm --full --autobuild-days 180 --autobuild roadLevel=2");
        };

        if (args.empty()) {
          printUsage();
          return;
        }

        const std::string sub = toLower(args[0]);
        if (sub != "import" && sub != "load") {
          c.print("Unknown osm subcommand: " + args[0]);
          printUsage();
          return;
        }

        // Defaults mirror the CLI tool.
        OsmImportConfig cfg;
        std::uint64_t seed = m_world.seed();

        int autobuildDays = 0;
        AutoBuildConfig abCfg;

        auto parseBool01 = [&](const std::string& s, bool& out) -> bool {
          int v = 0;
          if (!parseI32(s, v)) return false;
          if (v != 0 && v != 1) return false;
          out = (v != 0);
          return true;
        };

        auto parseWxH = [&](const std::string& s, int& outW, int& outH) -> bool {
          const std::size_t x = s.find_first_of("xX");
          if (x == std::string::npos) return false;
          int w = 0, h = 0;
          if (!parseI32(s.substr(0, x), w)) return false;
          if (!parseI32(s.substr(x + 1), h)) return false;
          if (w <= 0 || h <= 0) return false;
          outW = w;
          outH = h;
          return true;
        };

        auto parseF64 = [&](const std::string& s, double& out) -> bool {
          try {
            std::size_t idx = 0;
            const double v = std::stod(s, &idx);
            if (idx != s.size()) return false;
            if (!std::isfinite(v)) return false;
            out = v;
            return true;
          } catch (...) {
            return false;
          }
        };

        std::string osmPath;

        // Parse args: flags in any order; the first non-flag token becomes the OSM path.
        for (std::size_t i = 1; i < args.size(); ++i) {
          const std::string a = args[i];
          const std::string t = toLower(a);

          auto requireValue = [&](std::string& out) -> bool {
            if (i + 1 >= args.size()) return false;
            out = args[++i];
            return true;
          };

          if (t == "-h" || t == "--help" || t == "help") {
            printUsage();
            return;
          }

          // First positional token => path.
          if (!a.empty() && a[0] != '-' && osmPath.empty()) {
            osmPath = a;
            continue;
          }

          if (t == "--full") {
            cfg.importWater = true;
            cfg.importLanduse = true;
            cfg.importParks = true;
            cfg.importBuildings = true;
            continue;
          }

          if (t == "--seed") {
            std::string v;
            if (!requireValue(v) || !parseU64(v, seed)) {
              c.print("Invalid --seed");
              return;
            }
            continue;
          }

          if (t == "--size") {
            std::string v;
            int w = 0, h = 0;
            if (!requireValue(v) || !parseWxH(v, w, h)) {
              c.print("Invalid --size (expected WxH)");
              return;
            }
            cfg.width = w;
            cfg.height = h;
            continue;
          }

          if (t == "--meters-per-tile" || t == "--mpt") {
            std::string v;
            double mpt = 0.0;
            if (!requireValue(v) || !parseF64(v, mpt) || !(mpt > 0.0)) {
              c.print("Invalid --meters-per-tile");
              return;
            }
            cfg.metersPerTile = mpt;
            continue;
          }

          if (t == "--padding") {
            std::string v;
            int pad = 0;
            if (!requireValue(v) || !parseI32(v, pad) || pad < 0 || pad > 64) {
              c.print("Invalid --padding");
              return;
            }
            cfg.padding = pad;
            continue;
          }

          if (t == "--prefer-bounds") {
            std::string v;
            bool b = true;
            if (!requireValue(v) || !parseBool01(v, b)) {
              c.print("Invalid --prefer-bounds (expected 0|1)");
              return;
            }
            cfg.preferBoundsTag = b;
            continue;
          }

          auto parseFeatureBool = [&](bool& field) -> bool {
            std::string v;
            bool b = false;
            if (!requireValue(v) || !parseBool01(v, b)) return false;
            field = b;
            return true;
          };

          if (t == "--roads") {
            if (!parseFeatureBool(cfg.importRoads)) {
              c.print("Invalid --roads (expected 0|1)");
              return;
            }
            continue;
          }
          if (t == "--water") {
            if (!parseFeatureBool(cfg.importWater)) {
              c.print("Invalid --water (expected 0|1)");
              return;
            }
            continue;
          }
          if (t == "--landuse") {
            if (!parseFeatureBool(cfg.importLanduse)) {
              c.print("Invalid --landuse (expected 0|1)");
              return;
            }
            continue;
          }
          if (t == "--parks") {
            if (!parseFeatureBool(cfg.importParks)) {
              c.print("Invalid --parks (expected 0|1)");
              return;
            }
            continue;
          }
          if (t == "--buildings") {
            if (!parseFeatureBool(cfg.importBuildings)) {
              c.print("Invalid --buildings (expected 0|1)");
              return;
            }
            continue;
          }
          if (t == "--overwrite") {
            if (!parseFeatureBool(cfg.overwriteNonRoadOverlays)) {
              c.print("Invalid --overwrite (expected 0|1)");
              return;
            }
            continue;
          }

          if (t == "--fixed-radius") {
            std::string v;
            int r = 0;
            if (!requireValue(v) || !parseI32(v, r) || r < 0 || r > 16) {
              c.print("Invalid --fixed-radius");
              return;
            }
            cfg.fixedRadius = r;
            continue;
          }

          if (t == "--thicken-by-class") {
            std::string v;
            bool b = true;
            if (!requireValue(v) || !parseBool01(v, b)) {
              c.print("Invalid --thicken-by-class (expected 0|1)");
              return;
            }
            cfg.thickenByClass = b;
            continue;
          }

          if (t == "--waterway-radius") {
            std::string v;
            int r = 0;
            if (!requireValue(v) || !parseI32(v, r) || r < 0 || r > 16) {
              c.print("Invalid --waterway-radius");
              return;
            }
            cfg.waterwayRadius = r;
            continue;
          }

          if (t == "--autobuild-days") {
            std::string v;
            int d = 0;
            if (!requireValue(v) || !parseI32(v, d) || d < 0 || d > 5000) {
              c.print("Invalid --autobuild-days");
              return;
            }
            autobuildDays = d;
            continue;
          }

          if (t == "--autobuild") {
            std::string kv;
            if (!requireValue(kv)) {
              c.print("Invalid --autobuild (expected k=v)");
              return;
            }
            const std::size_t eq = kv.find('=');
            if (eq == std::string::npos) {
              c.print("Invalid --autobuild (expected k=v)");
              return;
            }
            const std::string key = kv.substr(0, eq);
            const std::string val = kv.substr(eq + 1);

            std::string err;
            if (!ParseAutoBuildKey(key, val, abCfg, err)) {
              c.print("autobuild: " + err);
              return;
            }
            continue;
          }

          // If we haven't consumed a path yet (e.g. it begins with '-' and was quoted), accept it.
          if (osmPath.empty()) {
            osmPath = a;
            continue;
          }

          c.print("Unknown option: " + a);
          printUsage();
          return;
        }

        if (osmPath.empty()) {
          c.print("osm: missing <file.osm>");
          printUsage();
          return;
        }

        endPaintStroke();

        World imported;
        OsmImportStats st;
        std::string err;

        if (!ImportOsmXmlRoadsToNewWorld(osmPath, seed, cfg, imported, &st, err)) {
          c.print("osm import failed: " + err);
          showToast("OSM import failed", 4.0f);
          return;
        }

        // Optional deterministic post-import growth.
        std::vector<Stats> tickStats;
        if (autobuildDays > 0) {
          Simulator sim = m_sim; // keep current runtime tuning (traffic/transit model)
          sim.resetTimer();
          const AutoBuildReport rep = RunAutoBuild(imported, sim, abCfg, autobuildDays, &tickStats);

          c.print(TextFormat("autobuild: requested=%d simulated=%d zones=%d roads=%d upgrades=%d parks=%d fails=%d",
                             rep.daysRequested, rep.daysSimulated, rep.zonesBuilt, rep.roadsBuilt, rep.roadsUpgraded,
                             rep.parksBuilt, rep.failedBuilds));
        }

        if (st.bounds.valid) {
          c.print(TextFormat("osm bounds: lat[%.6f..%.6f] lon[%.6f..%.6f]  out=%dx%d",
                             st.bounds.minLat, st.bounds.maxLat, st.bounds.minLon, st.bounds.maxLon,
                             st.outWidth, st.outHeight));
        } else {
          c.print(TextFormat("osm: imported  out=%dx%d", st.outWidth, st.outHeight));
        }

        const std::string toast = std::string(TextFormat("OSM imported: %dx%d  road:%zu water:%zu zone:%zu park:%zu",
                                                         st.outWidth, st.outHeight,
                                                         st.roadTilesPainted, st.waterTilesPainted,
                                                         st.zoneTilesPainted, st.parkTilesPainted));

        adoptLoadedWorld(std::move(imported), m_procCfg, m_sim.config(), toast,
                         (autobuildDays > 0 ? &tickStats : nullptr));

        c.print(TextFormat("osm stats: nodes=%zu ways=%zu rel=%zu  highway=%zu water=%zu landuse=%zu parks=%zu buildings=%zu",
                           st.nodesParsed, st.waysParsed, st.relationsParsed,
                           st.highwayWaysImported, st.waterWaysImported, st.landuseWaysImported,
                           st.parkWaysImported, st.buildingWaysImported));
      });

  m_console.registerCommand(
      "proc",
      "proc [key] [value]  - show/tweak ProcGen settings (try: proc list, proc preset island)",
      [this, toLower, parseF32, parseI32](DevConsole& c, const DevConsole::Args& args) {
        auto printCfg = [&]() {
          c.print("ProcGenConfig:");
          c.print(TextFormat("  terrain_scale = %.4f", m_procCfg.terrainScale));
          c.print(TextFormat("  water_level = %.3f", m_procCfg.waterLevel));
          c.print(TextFormat("  sand_level = %.3f", m_procCfg.sandLevel));
          c.print(TextFormat("  hubs = %d", m_procCfg.hubs));
          c.print(TextFormat("  extra_connections = %d", m_procCfg.extraConnections));
          c.print(TextFormat("  road_layout = %s", ToString(m_procCfg.roadLayout)));
          c.print(TextFormat("  zone_chance = %.3f", m_procCfg.zoneChance));
          c.print(TextFormat("  park_chance = %.3f", m_procCfg.parkChance));
          c.print(TextFormat("  terrain_preset = %s", ToString(m_procCfg.terrainPreset)));
          c.print(TextFormat("  terrain_preset_strength = %.3f", m_procCfg.terrainPresetStrength));
          c.print(TextFormat("  road_hierarchy_enabled = %s", m_procCfg.roadHierarchyEnabled ? "true" : "false"));
          c.print(TextFormat("  road_hierarchy_strength = %.3f", m_procCfg.roadHierarchyStrength));
          c.print(TextFormat("  districting_mode = %s", ToString(m_procCfg.districtingMode)));
          c.print(TextFormat("  erosion.enabled = %s", m_procCfg.erosion.enabled ? "true" : "false"));
          c.print(TextFormat("  rivers_enabled = %s", m_procCfg.erosion.riversEnabled ? "true" : "false"));
        };

        auto listPresets = [&]() {
          c.print("Terrain presets:");
          c.print("  classic");
          c.print("  island");
          c.print("  archipelago");
          c.print("  inland_sea");
          c.print("  river_valley");
          c.print("  mountain_ring");
          c.print("  fjords");
          c.print("  canyon");
          c.print("  volcano");
          c.print("  delta");
          c.print("  tectonic");
        };

        auto listRoadLayouts = [&]() {
          c.print("Road layouts:");
          c.print("  organic");
          c.print("  grid");
          c.print("  radial");
          c.print("  space_colonization");
        };

        auto listDistrictingModes = [&]() {
          c.print("Districting modes:");
          c.print("  voronoi");
          c.print("  road_flow");
          c.print("  block_graph");
        };

        auto listAll = [&]() {
          c.print("Keys:");
          c.print("  terrain_scale water_level sand_level hubs extra_connections zone_chance park_chance");
          c.print("  preset strength road_layout road_hierarchy road_hierarchy_strength districting_mode");
          c.print("  erosion rivers");
          listPresets();
          listRoadLayouts();
          listDistrictingModes();
          c.print("Examples:");
          c.print("  proc preset island");
          c.print("  proc strength 1.5");
          c.print("  proc road_layout grid");
          c.print("  proc districting_mode block_graph");
          c.print("  proc rivers 1");
          c.print("  regen same");
        };

        if (args.empty()) {
          printCfg();
          c.print("Usage: proc <key> <value>   (or 'proc list')");
          return;
        }

        const std::string key = toLower(args[0]);
        if (key == "list" || key == "help") {
          listAll();
          return;
        }
        if (key == "presets") {
          listPresets();
          return;
        }
        if (key == "layouts" || key == "road_layouts") {
          listRoadLayouts();
          return;
        }
        if (key == "districts" || key == "districting" || key == "districting_modes") {
          listDistrictingModes();
          return;
        }
        if (key == "show" || key == "get") {
          printCfg();
          return;
        }

        if (args.size() != 2) {
          c.print("Usage: proc <key> <value>   (try: proc list)");
          return;
        }

        const std::string val = args[1];

        if (key == "preset" || key == "terrain_preset") {
          ProcGenTerrainPreset p{};
          if (!ParseProcGenTerrainPreset(val, p)) {
            c.print("Unknown preset: " + val);
            listPresets();
            return;
          }
          m_procCfg.terrainPreset = p;
          showToast(TextFormat("Preset: %s", ToString(p)));
          c.print(std::string("terrain_preset = ") + ToString(p));
          return;
        }

        if (key == "road_layout" || key == "roadlayout" || key == "layout") {
          ProcGenRoadLayout l{};
          if (!ParseProcGenRoadLayout(val, l)) {
            c.print("Unknown road layout: " + val);
            listRoadLayouts();
            return;
          }
          m_procCfg.roadLayout = l;
          showToast(TextFormat("Road layout: %s", ToString(l)));
          c.print(std::string("road_layout = ") + ToString(l));
          return;
        }

        if (key == "strength" || key == "terrain_preset_strength") {
          float s = 1.0f;
          if (!parseF32(val, s)) {
            c.print("Invalid float: " + val);
            return;
          }
          m_procCfg.terrainPresetStrength = std::clamp(s, 0.0f, 5.0f);
          showToast(TextFormat("Preset strength: %.2f", m_procCfg.terrainPresetStrength));
          return;
        }

        auto setF32 = [&](float& target, float lo, float hi, const char* label) {
          float f = 0.0f;
          if (!parseF32(val, f)) {
            c.print(std::string("Invalid float: ") + val);
            return;
          }
          target = std::clamp(f, lo, hi);
          showToast(TextFormat("%s = %.3f", label, target));
        };

        auto setI32 = [&](int& target, int lo, int hi, const char* label) {
          int n = 0;
          if (!parseI32(val, n)) {
            c.print(std::string("Invalid int: ") + val);
            return;
          }
          target = std::clamp(n, lo, hi);
          showToast(TextFormat("%s = %d", label, target));
        };

        if (key == "terrain_scale") {
          setF32(m_procCfg.terrainScale, 0.005f, 1.0f, "terrain_scale");
        } else if (key == "water_level") {
          setF32(m_procCfg.waterLevel, 0.0f, 1.0f, "water_level");
        } else if (key == "sand_level") {
          setF32(m_procCfg.sandLevel, 0.0f, 1.0f, "sand_level");
        } else if (key == "hubs") {
          setI32(m_procCfg.hubs, 1, 64, "hubs");
        } else if (key == "extra_connections") {
          setI32(m_procCfg.extraConnections, 0, 256, "extra_connections");
        } else if (key == "zone_chance") {
          setF32(m_procCfg.zoneChance, 0.0f, 1.0f, "zone_chance");
        } else if (key == "park_chance") {
          setF32(m_procCfg.parkChance, 0.0f, 1.0f, "park_chance");
        } else if (key == "road_hierarchy" || key == "road_hierarchy_enabled" || key == "roadhierarchy") {
          int b = 0;
          if (!parseI32(val, b) || (b != 0 && b != 1)) {
            c.print("Usage: proc road_hierarchy <0|1>");
            return;
          }
          m_procCfg.roadHierarchyEnabled = (b != 0);
          showToast(m_procCfg.roadHierarchyEnabled ? "Road hierarchy: on" : "Road hierarchy: off");
        } else if (key == "road_hierarchy_strength" || key == "roadhierarchystrength") {
          setF32(m_procCfg.roadHierarchyStrength, 0.0f, 3.0f, "road_hierarchy_strength");
        } else if (key == "districting_mode" || key == "districting" || key == "district") {
          ProcGenDistrictingMode m{};
          if (!ParseProcGenDistrictingMode(val, m)) {
            c.print("Unknown districting mode: " + val);
            listDistrictingModes();
            return;
          }
          m_procCfg.districtingMode = m;
          showToast(TextFormat("Districting: %s", ToString(m)));
          c.print(std::string("districting_mode = ") + ToString(m));
        } else if (key == "rivers" || key == "rivers_enabled" || key == "erosion_rivers" || key == "riversenabled") {
          int b = 0;
          if (!parseI32(val, b) || (b != 0 && b != 1)) {
            c.print("Usage: proc rivers <0|1>");
            return;
          }
          m_procCfg.erosion.riversEnabled = (b != 0);
          showToast(m_procCfg.erosion.riversEnabled ? "Rivers: on" : "Rivers: off");
        } else if (key == "erosion" || key == "erosion_enabled") {
          int b = 0;
          if (!parseI32(val, b) || (b != 0 && b != 1)) {
            c.print("Usage: proc erosion <0|1>");
            return;
          }
          m_procCfg.erosion.enabled = (b != 0);
          showToast(m_procCfg.erosion.enabled ? "Erosion: on" : "Erosion: off");
        } else {
          c.print("Unknown proc key: " + args[0]);
          c.print("Try: proc show, proc list, proc preset <name>, proc strength <f>, proc road_layout <name>");
        }
      });

  m_console.registerCommand(
      "regen",
      "regen [same|<seed>] - regenerate the world (default: time-based seed)",
      [this, toLower, parseU64](DevConsole& c, const DevConsole::Args& args) {
        endPaintStroke();
        if (args.empty()) {
          resetWorld(0);
          c.print("World regenerated (time seed).");
          return;
        }

        const std::string a0 = toLower(args[0]);
        if (a0 == "same" || a0 == "current") {
          resetWorld(m_cfg.seed);
          c.print("World regenerated (same seed).");
          return;
        }

        std::uint64_t s = 0;
        if (!parseU64(args[0], s)) {
          c.print("Usage: regen [same|<uint64>]");
          return;
        }
        resetWorld(s);
        c.print(TextFormat("World regenerated with seed %llu", static_cast<unsigned long long>(s)));
      });

  m_console.registerCommand(
      "pause", "pause        - toggle simulation pause", [this](DevConsole& c, const DevConsole::Args&) {
        endPaintStroke();
        m_simPausedUser = !m_simPausedUser;
        m_sim.resetTimer();
        const bool paused = simPaused();
        showToast(paused ? "Sim paused" : "Sim running");
        c.print(paused ? "paused" : "running");
      });

  m_console.registerCommand(
      "step", "step         - advance the simulation by one day (like 'N' while paused)",
      [this](DevConsole& c, const DevConsole::Args&) {
        endPaintStroke();

        if (m_replayCapture.active()) {
          std::string err;
          (void)m_replayCapture.captureSettingsIfChanged(m_world, m_procCfg, m_sim, err);
        }

        m_sim.stepOnce(m_world);

        if (m_replayCapture.active()) {
          m_replayCapture.recordTicks(1);
        }
        {
          Stats s = m_world.stats();
          const int grant = applyCityChallenges(s);
          if (grant != 0) m_world.stats().money += grant;
          recordHistorySample(s);
        }
        m_trafficDirty = true;
        m_goodsDirty = true;
        m_landValueDirty = true;
        m_vehiclesDirty = true;
        m_transitPlanDirty = true;
        m_transitVizDirty = true;
        m_evacDirty = true;
        m_roadUpgradePlanDirty = true;
        m_roadUpgradeSelectedMaskDirty = true;
        invalidateAnalysisLayers();
        showToast("Sim step");
        c.print("stepped");
      });

  m_console.registerCommand(
      "speed", "speed <multiplier>  - set sim speed (e.g. 0.5, 1, 2, 4, 8)",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: speed <multiplier>");
          return;
        }
        float sp = 1.0f;
        if (!parseF32(args[0], sp)) {
          c.print("Invalid speed: " + args[0]);
          return;
        }

        // Pick nearest pre-defined speed.
        int best = 0;
        float bestDist = std::abs(kSimSpeeds[0] - sp);
        for (int i = 1; i < kSimSpeedCount; ++i) {
          const float d = std::abs(kSimSpeeds[i] - sp);
          if (d < bestDist) {
            bestDist = d;
            best = i;
          }
        }
        m_simSpeedIndex = best;
        showToast(TextFormat("Sim speed: x%.2f", static_cast<double>(kSimSpeeds[best])));
        c.print(TextFormat("sim speed set to x%.2f", static_cast<double>(kSimSpeeds[best])));
      });


  m_console.registerCommand(
      "loop",
      "loop [show|perf|dtmax <s>|simdtmax <s>|fps <focused> <unfocused>|throttle_unfocused <0|1>|sim <auto|fixed|ticks <n>>|budget <ms>|backlog <ticks>|pause_unfocused <0|1>|pause_minimized <0|1>] - tweak game loop pacing",
      [this, toLower, parseF32, parseI32](DevConsole& c, const DevConsole::Args& args) {
        auto parseBool01 = [&](const std::string& s, bool& out) -> bool {
          const std::string v = toLower(s);
          if (v == "1" || v == "on" || v == "true" || v == "yes") {
            out = true;
            return true;
          }
          if (v == "0" || v == "off" || v == "false" || v == "no") {
            out = false;
            return true;
          }
          return false;
        };

        auto printState = [&]() {
          c.print(TextFormat("dt clamp: frame %.3fs, sim %.3fs", static_cast<double>(m_loopCfg.maxFrameDt),
                             static_cast<double>(m_loopCfg.maxSimDt)));
          c.print(std::string("focus pause: unfocused ") + (m_loopCfg.pauseWhenUnfocused ? "on" : "off") +
                  ", minimized " + (m_loopCfg.pauseWhenMinimized ? "on" : "off"));
          c.print(TextFormat("fps cap: focused %d, unfocused %d | throttle %s | applied %d",
                             m_loopCfg.targetFpsFocused, m_loopCfg.targetFpsUnfocused,
                             m_loopCfg.throttleFpsWhenUnfocused ? "on" : "off", m_targetFpsApplied));

          const char* mode = !m_loopCfg.simTickLimiter ? "unlimited" : (m_loopCfg.simTickAuto ? "auto" : "fixed");
          c.print(TextFormat("sim limiter: %s | budget %.1fms | fixed %d ticks/frame | backlog cap %d ticks",
                             mode, static_cast<double>(m_loopCfg.simBudgetMs), m_loopCfg.simMaxTicksPerFrame,
                             m_loopCfg.simMaxBacklogTicks));
          c.print(std::string("perf overlay: ") + (m_showPerfOverlay ? "on (Ctrl+F3)" : "off (Ctrl+F3)"));

          c.print(TextFormat("last frame dt: %.2fms (raw %.2fms)%s | sim dt %.2fms",
                             static_cast<double>(m_loopStats.dtFrame * 1000.0f),
                             static_cast<double>(m_loopStats.dtRaw * 1000.0f),
                             m_loopStats.dtClamped ? " CLAMP" : "",
                             static_cast<double>(m_loopStats.dtSim * 1000.0f)));
          c.print(TextFormat("sim: ticks %d (limit %d) | backlog %d ticks (%.2fs) | tick %.2fms (ema %.2fms)",
                             m_loopStats.simTicks, m_loopStats.simTickLimit, m_loopStats.simBacklogTicks,
                             static_cast<double>(m_loopStats.simBacklogSec), m_loopStats.simMsPerTick,
                             m_loopStats.simMsPerTickEma));
          c.print(TextFormat("cpu ms: in %.2f  up %.2f  draw %.2f  total %.2f", m_loopStats.cpuInputMs,
                             m_loopStats.cpuUpdateMs, m_loopStats.cpuDrawMs, m_loopStats.cpuFrameMs));
        };

        if (args.empty()) {
          printState();
          return;
        }

        const std::string a0 = toLower(args[0]);
        if (a0 == "show" || a0 == "stats") {
          printState();
          return;
        }

        if (a0 == "perf") {
          if (args.size() == 1) {
            m_showPerfOverlay = !m_showPerfOverlay;
          } else {
            bool b = false;
            if (!parseBool01(args[1], b)) {
              c.print("Usage: loop perf [on|off|0|1|true|false]");
              return;
            }
            m_showPerfOverlay = b;
          }
          showToast(m_showPerfOverlay ? "Perf overlay: ON" : "Perf overlay: OFF");
          printState();
          return;
        }

        if (a0 == "dtmax") {
          if (args.size() != 2) {
            c.print("Usage: loop dtmax <seconds>");
            return;
          }
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Invalid float: " + args[1]);
            return;
          }
          m_loopCfg.maxFrameDt = std::max(0.0f, v);
          showToast(TextFormat("Loop dt clamp: %.3fs", static_cast<double>(m_loopCfg.maxFrameDt)));
          printState();
          return;
        }

        if (a0 == "simdtmax") {
          if (args.size() != 2) {
            c.print("Usage: loop simdtmax <seconds>");
            return;
          }
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Invalid float: " + args[1]);
            return;
          }
          m_loopCfg.maxSimDt = std::max(0.0f, v);
          showToast(TextFormat("Sim dt clamp: %.3fs", static_cast<double>(m_loopCfg.maxSimDt)));
          printState();
          return;
        }


        if (a0 == "throttle_unfocused") {
          if (args.size() != 2) {
            c.print("Usage: loop throttle_unfocused <0|1>");
            return;
          }
          bool b = false;
          if (!parseBool01(args[1], b)) {
            c.print("Usage: loop throttle_unfocused <0|1>");
            return;
          }
          m_loopCfg.throttleFpsWhenUnfocused = b;

          // Apply immediately.
          const bool focusedNow = IsWindowFocused();
          const bool minimizedNow = IsWindowState(FLAG_WINDOW_MINIMIZED);
          const int desired = (focusedNow && !minimizedNow) ? std::max(1, m_loopCfg.targetFpsFocused)
                                                            : std::max(1, m_loopCfg.targetFpsUnfocused);
          SetTargetFPS(desired);
          m_targetFpsApplied = desired;
          m_loopStats.targetFps = desired;

          showToast(std::string("Throttle when unfocused: ") + (b ? "ON" : "OFF"));
          printState();
          return;
        }

        if (a0 == "fps") {
          if (args.size() != 3) {
            c.print("Usage: loop fps <focused> <unfocused>");
            return;
          }
          int f = 0;
          int u = 0;
          if (!parseI32(args[1], f) || !parseI32(args[2], u)) {
            c.print("Usage: loop fps <focused> <unfocused>");
            return;
          }
          m_loopCfg.targetFpsFocused = std::clamp(f, 1, 1000);
          m_loopCfg.targetFpsUnfocused = std::clamp(u, 1, 1000);

          if (m_loopCfg.throttleFpsWhenUnfocused) {
            // Apply immediately based on current focus state.
            const bool focusedNow = IsWindowFocused();
            const bool minimizedNow = IsWindowState(FLAG_WINDOW_MINIMIZED);
            const int desired = (focusedNow && !minimizedNow) ? m_loopCfg.targetFpsFocused : m_loopCfg.targetFpsUnfocused;
            SetTargetFPS(desired);
            m_targetFpsApplied = desired;
            m_loopStats.targetFps = desired;
          }

          showToast(TextFormat("FPS cap: %d (focused), %d (unfocused)", m_loopCfg.targetFpsFocused, m_loopCfg.targetFpsUnfocused));
          printState();
          return;
        }

        if (a0 == "budget") {
          if (args.size() != 2) {
            c.print("Usage: loop budget <ms>");
            return;
          }
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Invalid float: " + args[1]);
            return;
          }
          m_loopCfg.simBudgetMs = std::max(0.0f, v);
          m_loopCfg.simTickLimiter = true;
          m_loopCfg.simTickAuto = true;
          showToast(TextFormat("Sim tick budget: %.1fms", static_cast<double>(m_loopCfg.simBudgetMs)));
          printState();
          return;
        }

        if (a0 == "backlog") {
          if (args.size() != 2) {
            c.print("Usage: loop backlog <ticks>");
            return;
          }
          int n = 0;
          if (!parseI32(args[1], n)) {
            c.print("Invalid int: " + args[1]);
            return;
          }
          m_loopCfg.simMaxBacklogTicks = std::max(0, n);
          showToast(TextFormat("Sim backlog cap: %d ticks", m_loopCfg.simMaxBacklogTicks));
          printState();
          return;
        }

        if (a0 == "pause_unfocused") {
          if (args.size() != 2) {
            c.print("Usage: loop pause_unfocused <0|1>");
            return;
          }
          bool b = false;
          if (!parseBool01(args[1], b)) {
            c.print("Usage: loop pause_unfocused <0|1>");
            return;
          }
          m_loopCfg.pauseWhenUnfocused = b;
          showToast(std::string("Pause when unfocused: ") + (b ? "ON" : "OFF"));
          printState();
          return;
        }

        if (a0 == "pause_minimized") {
          if (args.size() != 2) {
            c.print("Usage: loop pause_minimized <0|1>");
            return;
          }
          bool b = false;
          if (!parseBool01(args[1], b)) {
            c.print("Usage: loop pause_minimized <0|1>");
            return;
          }
          m_loopCfg.pauseWhenMinimized = b;
          showToast(std::string("Pause when minimized: ") + (b ? "ON" : "OFF"));
          printState();
          return;
        }

        if (a0 == "sim") {
          if (args.size() < 2) {
            c.print("Usage: loop sim <auto|fixed|ticks <n>>");
            return;
          }
          const std::string a1 = toLower(args[1]);
          if (a1 == "auto") {
            m_loopCfg.simTickLimiter = true;
            m_loopCfg.simTickAuto = true;
            showToast("Sim tick limiter: AUTO");
            printState();
            return;
          }
          if (a1 == "fixed") {
            m_loopCfg.simTickLimiter = true;
            m_loopCfg.simTickAuto = false;
            showToast("Sim tick limiter: FIXED");
            printState();
            return;
          }
          if (a1 == "ticks") {
            if (args.size() != 3) {
              c.print("Usage: loop sim ticks <n>");
              return;
            }
            int n = 0;
            if (!parseI32(args[2], n)) {
              c.print("Invalid int: " + args[2]);
              return;
            }
            m_loopCfg.simTickLimiter = true;
            m_loopCfg.simTickAuto = false;
            m_loopCfg.simMaxTicksPerFrame = std::max(0, n);
            showToast(TextFormat("Sim tick limit: %d", m_loopCfg.simMaxTicksPerFrame));
            printState();
            return;
          }
          c.print("Usage: loop sim <auto|fixed|ticks <n>>");
          return;
        }

        c.print("Unknown loop command. Try: loop show");
      });


  m_console.registerCommand(
      "money", "money <amount>  - set current money", [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: money <amount>");
          return;
        }
        long long v = 0;
        if (!parseI64(args[0], v)) {
          c.print("Invalid amount: " + args[0]);
          return;
        }
        m_world.stats().money = static_cast<int>(v);
        showToast(TextFormat("Money: %d", m_world.stats().money));
        c.print(TextFormat("money = %d", m_world.stats().money));
      });

  m_console.registerCommand(
      "give", "give <amount>   - add money", [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: give <amount>");
          return;
        }
        long long v = 0;
        if (!parseI64(args[0], v)) {
          c.print("Invalid amount: " + args[0]);
          return;
        }
        m_world.stats().money += static_cast<int>(v);
        showToast(TextFormat("Money: %d", m_world.stats().money));
        c.print(TextFormat("money = %d", m_world.stats().money));
      });

  // --- tools/rendering ---
  m_console.registerCommand(
      "tool",
      "tool <road|res|com|ind|park|bulldoze|inspect|raise|lower|smooth|district>  - select tool",
      [this, toLower, heatmapName](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: tool <name>");
          return;
        }
        const std::string t = toLower(args[0]);
        Tool newTool = m_tool;
        if (t == "road") newTool = Tool::Road;
        else if (t == "res" || t == "residential") newTool = Tool::Residential;
        else if (t == "com" || t == "commercial") newTool = Tool::Commercial;
        else if (t == "ind" || t == "industrial") newTool = Tool::Industrial;
        else if (t == "park") newTool = Tool::Park;
        else if (t == "bulldoze" || t == "doze" || t == "delete") newTool = Tool::Bulldoze;
        else if (t == "inspect") newTool = Tool::Inspect;
        else if (t == "raise") newTool = Tool::RaiseTerrain;
        else if (t == "lower") newTool = Tool::LowerTerrain;
        else if (t == "smooth") newTool = Tool::SmoothTerrain;
        else if (t == "district") newTool = Tool::District;
        else {
          c.print("Unknown tool: " + args[0]);
          return;
        }

        endPaintStroke();
        m_tool = newTool;
        // Cancel any road drag preview if we changed tools.
        if (m_tool != Tool::Road) {
          m_roadDragActive = false;
          m_roadDragStart.reset();
          m_roadDragEnd.reset();
          m_roadDragPath.clear();
          m_roadDragBuildCost = 0;
          m_roadDragUpgradeTiles = 0;
          m_roadDragBridgeTiles = 0;
          m_roadDragMoneyCost = 0;
          m_roadDragValid = false;
        }
        showToast(TextFormat("Tool: %s", ToString(m_tool)));
        c.print(TextFormat("tool = %s", ToString(m_tool)));
      });

  m_console.registerCommand(
      "brush", "brush <0..8>   - set brush radius (diamond)",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: brush <0..8>");
          return;
        }
        long long r = 0;
        if (!parseI64(args[0], r)) {
          c.print("Invalid radius: " + args[0]);
          return;
        }
        m_brushRadius = std::clamp(static_cast<int>(r), 0, 8);
        showToast(TextFormat("Brush radius: %d", m_brushRadius));
        c.print(TextFormat("brush = %d", m_brushRadius));
      });

  m_console.registerCommand(
      "roadlevel", "roadlevel <1..3> - set road build level", [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: roadlevel <1..3>");
          return;
        }
        long long lv = 0;
        if (!parseI64(args[0], lv)) {
          c.print("Invalid level: " + args[0]);
          return;
        }
        m_roadBuildLevel = std::clamp(static_cast<int>(lv), 1, 3);
        showToast(TextFormat("Road type: %s", RoadClassName(m_roadBuildLevel)));
        c.print(TextFormat("roadlevel = %d", m_roadBuildLevel));
      });

  m_console.registerCommand(
      "heatmap",
      "heatmap <name|list> - set heatmap overlay (try: heatmap list)",
      [this, toLower, heatmapName](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: heatmap <name|list>");
          c.print("Try: heatmap list");
          return;
        }

        const std::string h = toLower(args[0]);
        if (h == "list" || h == "ls" || h == "?" || h == "help") {
          c.print("Heatmaps:");
          static constexpr HeatmapOverlay kAll[] = {
              HeatmapOverlay::Off,
              HeatmapOverlay::LandValue,
              HeatmapOverlay::ParkAmenity,
              HeatmapOverlay::WaterAmenity,
              HeatmapOverlay::Pollution,
              HeatmapOverlay::TrafficSpill,
              HeatmapOverlay::ServicesOverall,
              HeatmapOverlay::ServicesEducation,
              HeatmapOverlay::ServicesHealth,
              HeatmapOverlay::ServicesSafety,
              HeatmapOverlay::FireRisk,
              HeatmapOverlay::FloodDepth,
              HeatmapOverlay::PondingDepth,
              HeatmapOverlay::EvacuationTime,
              HeatmapOverlay::EvacuationUnreachable,
              HeatmapOverlay::EvacuationFlow,

              HeatmapOverlay::AirPollution,
              HeatmapOverlay::AirPollutionEmission,
              HeatmapOverlay::SkyView,
              HeatmapOverlay::CanyonConfinement,
              HeatmapOverlay::SolarExposure,
              HeatmapOverlay::SolarPotential,
              HeatmapOverlay::EnergyDemand,
              HeatmapOverlay::EnergySolar,
              HeatmapOverlay::EnergyBalance,

              HeatmapOverlay::CarbonEmission,
              HeatmapOverlay::CarbonSequestration,
              HeatmapOverlay::CarbonBalance,

              HeatmapOverlay::CrimeRisk,
              HeatmapOverlay::PoliceAccess,

              HeatmapOverlay::TrafficCrashRisk,
              HeatmapOverlay::TrafficCrashExposure,
              HeatmapOverlay::TrafficCrashPriority,

              HeatmapOverlay::TransitAccess,
              HeatmapOverlay::TransitModeSharePotential,

              HeatmapOverlay::Walkability,
              HeatmapOverlay::WalkabilityPark,
              HeatmapOverlay::WalkabilityRetail,
              HeatmapOverlay::WalkabilityEducation,
              HeatmapOverlay::WalkabilityHealth,
              HeatmapOverlay::WalkabilitySafety,

              HeatmapOverlay::JobAccess,
              HeatmapOverlay::JobOpportunity,

              HeatmapOverlay::RoadCentrality,
              HeatmapOverlay::RoadVulnerability,
              HeatmapOverlay::RoadBypass,

              HeatmapOverlay::Livability,
              HeatmapOverlay::InterventionPriority,
              HeatmapOverlay::LivabilityHotspot,
              HeatmapOverlay::InterventionHotspot,

              HeatmapOverlay::RunoffPollution,
              HeatmapOverlay::RunoffPollutionLoad,
              HeatmapOverlay::RunoffMitigationPriority,
              HeatmapOverlay::RunoffMitigationPlan,
          };

          for (HeatmapOverlay hm : kAll) {
            c.print(std::string("  ") + heatmapName(hm));
          }
          return;
        }

        if (h == "off") m_heatmapOverlay = HeatmapOverlay::Off;
        else if (h == "land" || h == "landvalue") m_heatmapOverlay = HeatmapOverlay::LandValue;
        else if (h == "park") m_heatmapOverlay = HeatmapOverlay::ParkAmenity;
        else if (h == "water") m_heatmapOverlay = HeatmapOverlay::WaterAmenity;
        else if (h == "pollution") m_heatmapOverlay = HeatmapOverlay::Pollution;
        else if (h == "traffic") m_heatmapOverlay = HeatmapOverlay::TrafficSpill;
        else if (h == "services" || h == "svc" || h == "services_overall") m_heatmapOverlay = HeatmapOverlay::ServicesOverall;
        else if (h == "services_edu" || h == "services_education" || h == "edu" || h == "education") m_heatmapOverlay = HeatmapOverlay::ServicesEducation;
        else if (h == "services_health" || h == "services_hospital" || h == "health" || h == "hospital") m_heatmapOverlay = HeatmapOverlay::ServicesHealth;
        else if (h == "services_safety" || h == "services_police" || h == "services_fire" || h == "safety" || h == "police") m_heatmapOverlay = HeatmapOverlay::ServicesSafety;
        else if (h == "fire_risk" || h == "firerisk" || h == "firehazard" || h == "hazard_fire" || h == "fire") m_heatmapOverlay = HeatmapOverlay::FireRisk;
        else if (h == "flood") m_heatmapOverlay = HeatmapOverlay::FloodDepth;
        else if (h == "pond" || h == "ponding") m_heatmapOverlay = HeatmapOverlay::PondingDepth;
        else if (h == "evac" || h == "evactime" || h == "evac_time") m_heatmapOverlay = HeatmapOverlay::EvacuationTime;
        else if (h == "evac_unreach" || h == "evacunreach" || h == "evac_unreachable" || h == "unreach") m_heatmapOverlay = HeatmapOverlay::EvacuationUnreachable;
        else if (h == "evac_flow" || h == "evacflow" || h == "flow" || h == "evacuationflow") m_heatmapOverlay = HeatmapOverlay::EvacuationFlow;

        else if (h == "air_pollution" || h == "air" || h == "airquality" || h == "air_quality" || h == "smog" || h == "aq") m_heatmapOverlay = HeatmapOverlay::AirPollution;
        else if (h == "air_emission" || h == "air_source" || h == "air_sources" || h == "smog_sources" || h == "pollution_source") m_heatmapOverlay = HeatmapOverlay::AirPollutionEmission;

        else if (h == "sky_view" || h == "skyview" || h == "svf") m_heatmapOverlay = HeatmapOverlay::SkyView;
        else if (h == "canyon_confinement" || h == "canyon" || h == "urban_canyon" || h == "confinement") m_heatmapOverlay = HeatmapOverlay::CanyonConfinement;

        else if (h == "solar_exposure" || h == "solar") m_heatmapOverlay = HeatmapOverlay::SolarExposure;
        else if (h == "solar_potential" || h == "pv") m_heatmapOverlay = HeatmapOverlay::SolarPotential;

        else if (h == "energy_demand" || h == "power_demand" || h == "electricity_demand" || h == "demand_energy")
          m_heatmapOverlay = HeatmapOverlay::EnergyDemand;
        else if (h == "energy_solar" || h == "solar_supply" || h == "pv_supply" || h == "rooftop_solar")
          m_heatmapOverlay = HeatmapOverlay::EnergySolar;
        else if (h == "energy_balance" || h == "energy_net" || h == "net_energy" || h == "renewable_balance" || h == "energy")
          m_heatmapOverlay = HeatmapOverlay::EnergyBalance;



        else if (h == "carbon_emission" || h == "co2_emission" || h == "carbon_emissions")
          m_heatmapOverlay = HeatmapOverlay::CarbonEmission;
        else if (h == "carbon_sequestration" || h == "carbon_sink" || h == "co2_sink" || h == "sequestration")
          m_heatmapOverlay = HeatmapOverlay::CarbonSequestration;
        else if (h == "carbon_balance" || h == "carbon_net" || h == "co2_balance" || h == "net_carbon" || h == "carbon")
          m_heatmapOverlay = HeatmapOverlay::CarbonBalance;

        else if (h == "crime_risk" || h == "crime" || h == "crime_index" || h == "safety_risk")
          m_heatmapOverlay = HeatmapOverlay::CrimeRisk;
        else if (h == "police_access" || h == "police_response" || h == "police_coverage" || h == "response_time")
          m_heatmapOverlay = HeatmapOverlay::PoliceAccess;

        else if (h == "traffic_crash_risk" || h == "traffic_collision_risk" || h == "crash_risk" || h == "safety_crash")
          m_heatmapOverlay = HeatmapOverlay::TrafficCrashRisk;
        else if (h == "traffic_crash_exposure" || h == "crash_exposure" || h == "safety_exposure")
          m_heatmapOverlay = HeatmapOverlay::TrafficCrashExposure;
        else if (h == "traffic_crash_priority" || h == "crash_priority" || h == "safety_priority")
          m_heatmapOverlay = HeatmapOverlay::TrafficCrashPriority;

        else if (h == "walkability" || h == "walk" || h == "walkscore" || h == "15min") m_heatmapOverlay = HeatmapOverlay::Walkability;
        else if (h == "walkability_park" || h == "walk_park") m_heatmapOverlay = HeatmapOverlay::WalkabilityPark;
        else if (h == "walkability_retail" || h == "walk_retail") m_heatmapOverlay = HeatmapOverlay::WalkabilityRetail;
        else if (h == "walkability_education" || h == "walk_education") m_heatmapOverlay = HeatmapOverlay::WalkabilityEducation;
        else if (h == "walkability_health" || h == "walk_health") m_heatmapOverlay = HeatmapOverlay::WalkabilityHealth;
        else if (h == "walkability_safety" || h == "walk_safety") m_heatmapOverlay = HeatmapOverlay::WalkabilitySafety;

        else if (h == "job_access" || h == "jobs_access" || h == "employment_access" || h == "access_jobs" || h == "jobaccess")
          m_heatmapOverlay = HeatmapOverlay::JobAccess;
        else if (h == "job_opportunity" || h == "jobs" || h == "employment" || h == "job_market" || h == "jobopportunity")
          m_heatmapOverlay = HeatmapOverlay::JobOpportunity;

        else if (h == "road_centrality" || h == "centrality") m_heatmapOverlay = HeatmapOverlay::RoadCentrality;
        else if (h == "road_vulnerability" || h == "vulnerability") m_heatmapOverlay = HeatmapOverlay::RoadVulnerability;
        else if (h == "road_bypass" || h == "bypass") m_heatmapOverlay = HeatmapOverlay::RoadBypass;

        else if (h == "livability" || h == "live") m_heatmapOverlay = HeatmapOverlay::Livability;
        else if (h == "intervention_priority" || h == "priority") m_heatmapOverlay = HeatmapOverlay::InterventionPriority;

        else if (h == "livability_hotspot" || h == "live_hotspot") m_heatmapOverlay = HeatmapOverlay::LivabilityHotspot;
        else if (h == "intervention_hotspot" || h == "priority_hotspot") m_heatmapOverlay = HeatmapOverlay::InterventionHotspot;

        else if (h == "runoff_pollution" || h == "runoff" || h == "stormwater" || h == "water_quality") m_heatmapOverlay = HeatmapOverlay::RunoffPollution;
        else if (h == "runoff_load" || h == "runoff_source" || h == "stormwater_load") m_heatmapOverlay = HeatmapOverlay::RunoffPollutionLoad;
        else if (h == "runoff_mitigation_priority" || h == "mitigation_priority") m_heatmapOverlay = HeatmapOverlay::RunoffMitigationPriority;
        else if (h == "runoff_mitigation_plan" || h == "mitigation_plan") m_heatmapOverlay = HeatmapOverlay::RunoffMitigationPlan;

        else {
          c.print("Unknown heatmap: " + args[0]);
          c.print("Try: heatmap list");
          return;
        }

        // Mark derived fields dirty. Which ones get recomputed depends on which heatmap is active.
        m_landValueDirty = true;
        m_servicesHeatmapsDirty = true;
        m_fireRiskDirty = true;
        invalidateHydrology();
        invalidateAnalysisLayers();

        showToast(TextFormat("Heatmap: %s", heatmapName(m_heatmapOverlay)));
        c.print(TextFormat("heatmap = %s", heatmapName(m_heatmapOverlay)));
      });

  m_console.registerCommand(
      "sea",
      "sea [level] [edge|all] [4|8]  - configure sea-level flooding overlay (used by heatmap flood)",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() > 3) {
          c.print("Usage: sea [level] [edge|all] [4|8]");
          return;
        }

        bool changed = false;

        if (!args.empty()) {
          float lv = 0.0f;
          if (!parseF32(args[0], lv)) {
            c.print("Invalid sea level: " + args[0]);
            return;
          }
          m_seaLevel = std::clamp(lv, 0.0f, 1.0f);
          changed = true;
        }

        if (args.size() >= 2) {
          const std::string mode = toLower(args[1]);
          if (mode == "edge" || mode == "coast" || mode == "coastal") {
            m_seaFloodCfg.requireEdgeConnection = true;
          } else if (mode == "all" || mode == "any") {
            m_seaFloodCfg.requireEdgeConnection = false;
          } else {
            c.print("Unknown mode: " + args[1] + " (use edge|all)");
            return;
          }
          changed = true;
        }

        if (args.size() >= 3) {
          const std::string conn = toLower(args[2]);
          if (conn == "8" || conn == "8c" || conn == "eight") {
            m_seaFloodCfg.eightConnected = true;
          } else if (conn == "4" || conn == "4c" || conn == "four") {
            m_seaFloodCfg.eightConnected = false;
          } else {
            c.print("Unknown connectivity: " + args[2] + " (use 4|8)");
            return;
          }
          changed = true;
        }

        if (changed) {
          invalidateHydrology();
          showToast(TextFormat("Sea level: %.2f (%s,%s)", static_cast<double>(m_seaLevel),
                               m_seaFloodCfg.requireEdgeConnection ? "edge" : "all",
                               m_seaFloodCfg.eightConnected ? "8c" : "4c"));
        }

        c.print(TextFormat("sea = %.3f  mode=%s  conn=%s", static_cast<double>(m_seaLevel),
                           m_seaFloodCfg.requireEdgeConnection ? "edge" : "all",
                           m_seaFloodCfg.eightConnected ? "8" : "4"));
      });

  m_console.registerCommand(
      "pond",
      "pond [eps] [edge|noedge]  - configure depression-fill ponding overlay (used by heatmap pond)",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() > 2) {
          c.print("Usage: pond [eps] [edge|noedge]");
          return;
        }

        bool changed = false;

        if (!args.empty()) {
          float eps = 0.0f;
          if (!parseF32(args[0], eps)) {
            c.print("Invalid eps: " + args[0]);
            return;
          }
          m_pondingCfg.epsilon = std::max(0.0f, eps);
          changed = true;
        }

        if (args.size() >= 2) {
          const std::string mode = toLower(args[1]);
          if (mode == "edge" || mode == "edges" || mode == "open") {
            m_pondingCfg.includeEdges = true;
          } else if (mode == "noedge" || mode == "closed" || mode == "none") {
            m_pondingCfg.includeEdges = false;
          } else {
            c.print("Unknown mode: " + args[1] + " (use edge|noedge)");
            return;
          }
          changed = true;
        }

        if (changed) {
          m_pondingDirty = true;
  m_evacDirty = true;
          showToast(TextFormat("Ponding: eps %.4f (%s)", static_cast<double>(m_pondingCfg.epsilon),
                               m_pondingCfg.includeEdges ? "edge" : "noedge"));
        }

        c.print(TextFormat("pond = eps %.6f  edges=%s", static_cast<double>(m_pondingCfg.epsilon),
                           m_pondingCfg.includeEdges ? "edge" : "noedge"));
      });



  m_console.registerCommand(
      "evac",
      "evac [status|mode <none|sea|pond|both>|pondmin <depth>|bridges <0|1>|walk <milli>|cap <n>|weight <time|steps>|run|export]  - evacuation scenario analysis",
      [this, toLower, parseI64, parseF32](DevConsole& c, const DevConsole::Args& args) {
        auto printCfg = [&]() {
          c.print(TextFormat("evac = mode=%s  bridges=%s  pondMin=%.3f  weight=%s  walk=%d  cap=%d",
                             EvacuationHazardModeName(m_evacCfg.hazardMode),
                             m_evacCfg.bridgesPassable ? "on" : "off",
                             static_cast<double>(m_evacCfg.pondMinDepth),
                             m_evacCfg.evac.useTravelTime ? "time" : "steps",
                             m_evacCfg.evac.walkCostMilli,
                             m_evacCfg.evac.roadTileCapacity));
          c.print(TextFormat("  sea = %.3f (%s,%s)  pond = eps %.6f (%s)",
                             static_cast<double>(m_seaLevel),
                             m_seaFloodCfg.requireEdgeConnection ? "edge" : "all",
                             m_seaFloodCfg.eightConnected ? "8c" : "4c",
                             static_cast<double>(m_pondingCfg.epsilon),
                             m_pondingCfg.includeEdges ? "edge" : "noedge"));
        };

        auto printStatsIfReady = [&]() {
          if (m_evacDirty || m_evacScenario.w != m_world.width() || m_evacScenario.h != m_world.height() ||
              m_evacScenario.evac.residentialTiles == 0) {
            c.print("  (run: evac run)  (export: evac export)");
            return;
          }
          const EvacuationResult& r = m_evacScenario.evac;
          const double reach = (r.population > 0)
                                   ? (100.0 * static_cast<double>(r.reachablePopulation) /
                                      static_cast<double>(r.population))
                                   : 0.0;
          c.print(TextFormat(
              "  exits=%d  resTiles=%d  pop=%d  reach=%.1f%%  floodedPop=%d  unreachablePop=%d  avg=%.1f  p95=%.1f  maxFlow=%u",
              r.exitSources, r.residentialTiles, r.population, reach, r.floodedPopulation,
              r.unreachablePopulation, r.avgEvacTime, r.p95EvacTime, r.maxEvacRoadFlow));
        };

        if (args.empty() || toLower(args[0]) == "status" || toLower(args[0]) == "show") {
          printCfg();
          printStatsIfReady();
          return;
        }

        const std::string sub = toLower(args[0]);

        auto setMode = [&](EvacuationHazardMode m) {
          m_evacCfg.hazardMode = m;
          m_evacDirty = true;
          showToast(TextFormat("Evac mode: %s", EvacuationHazardModeName(m)), 2.0f);
        };

        if (sub == "mode" || sub == "hazard") {
          if (args.size() != 2) {
            c.print("Usage: evac mode <none|sea|pond|both>");
            return;
          }
          const std::string m = toLower(args[1]);
          if (m == "none") setMode(EvacuationHazardMode::None);
          else if (m == "sea") setMode(EvacuationHazardMode::Sea);
          else if (m == "pond" || m == "ponding") setMode(EvacuationHazardMode::Ponding);
          else if (m == "both") setMode(EvacuationHazardMode::Both);
          else {
            c.print("Unknown mode: " + args[1]);
            return;
          }
          printCfg();
          return;
        }

        if (sub == "none" || sub == "sea" || sub == "pond" || sub == "ponding" || sub == "both") {
          if (sub == "none") setMode(EvacuationHazardMode::None);
          else if (sub == "sea") setMode(EvacuationHazardMode::Sea);
          else if (sub == "pond" || sub == "ponding") setMode(EvacuationHazardMode::Ponding);
          else setMode(EvacuationHazardMode::Both);
          printCfg();
          return;
        }

        if (sub == "pondmin") {
          if (args.size() != 2) {
            c.print("Usage: evac pondmin <depth>");
            return;
          }
          float d = 0.0f;
          if (!parseF32(args[1], d)) {
            c.print("Invalid depth: " + args[1]);
            return;
          }
          m_evacCfg.pondMinDepth = std::max(0.0f, d);
          m_evacDirty = true;
          showToast(TextFormat("Evac pond depth >= %.3f", static_cast<double>(m_evacCfg.pondMinDepth)), 2.0f);
          printCfg();
          return;
        }

        if (sub == "bridges") {
          if (args.size() != 2) {
            c.print("Usage: evac bridges <0|1>");
            return;
          }
          long long v = 0;
          if (!parseI64(args[1], v)) {
            c.print("Invalid value: " + args[1]);
            return;
          }
          m_evacCfg.bridgesPassable = (v != 0);
          m_evacDirty = true;
          showToast(std::string("Evac bridges: ") + (m_evacCfg.bridgesPassable ? "passable" : "blocked"), 2.0f);
          printCfg();
          return;
        }

        if (sub == "walk") {
          if (args.size() != 2) {
            c.print("Usage: evac walk <milli>");
            return;
          }
          long long v = 0;
          if (!parseI64(args[1], v)) {
            c.print("Invalid millis: " + args[1]);
            return;
          }
          m_evacCfg.evac.walkCostMilli = std::clamp(static_cast<int>(v), 0, 600000);
          m_evacDirty = true;
          showToast(TextFormat("Evac walk cost: %d", m_evacCfg.evac.walkCostMilli), 2.0f);
          printCfg();
          return;
        }

        if (sub == "cap" || sub == "capacity") {
          if (args.size() != 2) {
            c.print("Usage: evac cap <vehiclesPerRoadTile>");
            return;
          }
          long long v = 0;
          if (!parseI64(args[1], v)) {
            c.print("Invalid cap: " + args[1]);
            return;
          }
          m_evacCfg.evac.roadTileCapacity = std::clamp(static_cast<int>(v), 1, 100000);
          m_evacDirty = true;
          showToast(TextFormat("Evac road capacity: %d", m_evacCfg.evac.roadTileCapacity), 2.0f);
          printCfg();
          return;
        }

        if (sub == "weight") {
          if (args.size() != 2) {
            c.print("Usage: evac weight <time|steps>");
            return;
          }
          const std::string w = toLower(args[1]);
          if (w == "time" || w == "traveltime" || w == "t") {
            m_evacCfg.evac.useTravelTime = true;
          } else if (w == "steps" || w == "tiles" || w == "s") {
            m_evacCfg.evac.useTravelTime = false;
          } else {
            c.print("Unknown weight: " + args[1]);
            return;
          }
          m_evacDirty = true;
          showToast(std::string("Evac weighting: ") + (m_evacCfg.evac.useTravelTime ? "time" : "steps"), 2.0f);
          printCfg();
          return;
        }

        if (sub == "run" || sub == "refresh") {
          ensureEvacuationScenarioUpToDate();
          printCfg();
          printStatsIfReady();
          if (!m_evacDirty) {
            const EvacuationResult& r = m_evacScenario.evac;
            const double reach = (r.population > 0)
                                     ? (100.0 * static_cast<double>(r.reachablePopulation) /
                                        static_cast<double>(r.population))
                                     : 0.0;
            showToast(TextFormat("Evac: %.0f%% reachable (p95 %.1f)", reach, r.p95EvacTime), 2.5f);
          }
          return;
        }

        if (sub == "export") {
          exportEvacuationArtifacts();
          return;
        }

        c.print(
            "Usage: evac [status|mode <none|sea|pond|both>|pondmin <depth>|bridges <0|1>|walk <milli>|cap <n>|weight <time|steps>|run|export]");
      });
  m_console.registerCommand(
      "floodapply",
      "floodapply [level] [edge|all] [4|8]  - apply sea flooding to the world (undoable)",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() > 3) {
          c.print("Usage: floodapply [level] [edge|all] [4|8]");
          return;
        }

        // Commit any in-progress user stroke so this is a clean, single undo step.
        endPaintStroke();

        float seaLevel = m_seaLevel;
        SeaFloodConfig cfg = m_seaFloodCfg;

        if (!args.empty()) {
          float lv = 0.0f;
          if (!parseF32(args[0], lv)) {
            c.print("Invalid sea level: " + args[0]);
            return;
          }
          seaLevel = std::clamp(lv, 0.0f, 1.0f);
        }

        if (args.size() >= 2) {
          const std::string mode = toLower(args[1]);
          if (mode == "edge" || mode == "coast" || mode == "coastal") {
            cfg.requireEdgeConnection = true;
          } else if (mode == "all" || mode == "any") {
            cfg.requireEdgeConnection = false;
          } else {
            c.print("Unknown mode: " + args[1] + " (use edge|all)");
            return;
          }
        }

        if (args.size() >= 3) {
          const std::string conn = toLower(args[2]);
          if (conn == "8" || conn == "8c" || conn == "eight") {
            cfg.eightConnected = true;
          } else if (conn == "4" || conn == "4c" || conn == "four") {
            cfg.eightConnected = false;
          } else {
            c.print("Unknown connectivity: " + args[2] + " (use 4|8)");
            return;
          }
        }

        // Make the overlay configuration reflect the parameters we just used.
        m_seaLevel = seaLevel;
        m_seaFloodCfg = cfg;
        invalidateHydrology();

        const int w = m_world.width();
        const int h = m_world.height();
        if (w <= 0 || h <= 0) {
          c.print("World is empty");
          return;
        }

        const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
        std::vector<float> heights(n, 0.0f);
        for (int y = 0; y < h; ++y) {
          for (int x = 0; x < w; ++x) {
            heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
                m_world.at(x, y).height;
          }
        }

        const SeaFloodResult r = ComputeSeaLevelFlood(heights, w, h, seaLevel, cfg);

        const auto idxToXY = [&](int idx, int& x, int& y) {
          x = idx % w;
          y = idx / w;
        };

        int changedTiles = 0;

        m_history.beginStroke(m_world);
        for (int i = 0; i < static_cast<int>(n); ++i) {
          if (r.flooded[static_cast<std::size_t>(i)] == 0) continue;

          int x = 0, y = 0;
          idxToXY(i, x, y);
          if (!m_world.inBounds(x, y)) continue;

          Tile& t = m_world.at(x, y);
          const bool needTerrain = (t.terrain != Terrain::Water);
          const bool needOverlayClear = (t.overlay != Overlay::None && t.overlay != Overlay::Road);
          if (!needTerrain && !needOverlayClear) continue;

          m_history.noteTilePreEdit(m_world, x, y);

          // Flooded land becomes water. Roads survive as bridges; everything else is removed.
          t.terrain = Terrain::Water;
          if (needOverlayClear) {
            m_world.setOverlay(Overlay::None, x, y);
          }

          ++changedTiles;
        }
        m_history.endStroke(m_world);

        if (changedTiles <= 0) {
          showToast(TextFormat("Flood apply: no changes (sea %.2f)", static_cast<double>(seaLevel)));
          c.print("no changes");
          return;
        }

        // A flood can invalidate many derived overlays/stats.
        m_renderer.markMinimapDirty();
        m_renderer.markBaseCacheDirtyAll();
        m_sim.refreshDerivedStats(m_world);
        m_trafficDirty = true;
        m_goodsDirty = true;
        m_landValueDirty = true;
        m_vehiclesDirty = true;
        m_roadGraphDirty = true;
        invalidateHydrology();

        showToast(TextFormat("Flood applied: %d tiles (sea %.2f)", changedTiles, static_cast<double>(seaLevel)), 3.0f);
        c.print(TextFormat("flooded %d tiles (sea %.3f)", changedTiles, static_cast<double>(seaLevel)));
      });

  m_console.registerCommand(
      "overlay",
      "overlay <minimap|vehicles|traffic|goods|outside|help|policy|report|cache|traffic_model|roadgraph|resilience|daynight|weather> [on|off|toggle]",
      [this, toLower](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print("Usage: overlay <name> [on|off|toggle]");
          return;
        }

        const std::string name = toLower(args[0]);
        const std::string mode = (args.size() >= 2) ? toLower(args[1]) : "toggle";

        auto want = [&](bool current) {
          if (mode == "on" || mode == "1" || mode == "true") return true;
          if (mode == "off" || mode == "0" || mode == "false") return false;
          return !current;
        };

        if (name == "minimap") {
          m_showMinimap = want(m_showMinimap);
          showToast(m_showMinimap ? "Minimap: ON" : "Minimap: OFF");
        } else if (name == "vehicles") {
          m_showVehicles = want(m_showVehicles);
          showToast(m_showVehicles ? "Vehicles: ON" : "Vehicles: OFF");
        } else if (name == "traffic") {
          m_showTrafficOverlay = want(m_showTrafficOverlay);
          showToast(m_showTrafficOverlay ? "Traffic overlay: ON" : "Traffic overlay: OFF");
        } else if (name == "goods") {
          m_showGoodsOverlay = want(m_showGoodsOverlay);
          showToast(m_showGoodsOverlay ? "Goods overlay: ON" : "Goods overlay: OFF");
        } else if (name == "outside") {
          m_showOutsideOverlay = want(m_showOutsideOverlay);
          showToast(m_showOutsideOverlay ? "Outside overlay: ON" : "Outside overlay: OFF");
        } else if (name == "roadgraph") {
          m_showRoadGraphOverlay = want(m_showRoadGraphOverlay);
          showToast(m_showRoadGraphOverlay ? "Road graph overlay: ON" : "Road graph overlay: OFF");
        } else if (name == "resilience" || name == "res") {
          m_showResilienceOverlay = want(m_showResilienceOverlay);
          if (m_showResilienceOverlay) {
            ensureRoadResilienceUpToDate();
            m_resilienceBypassesDirty = true;
            rebuildRoadResilienceBypasses();
          }
          showToast(m_showResilienceOverlay ? "Resilience overlay: ON" : "Resilience overlay: OFF");
        } else if (name == "help") {
          m_showHelp = want(m_showHelp);
          showToast(m_showHelp ? "Help: ON" : "Help: OFF");
        } else if (name == "policy" || name == "policies") {
          m_showPolicy = want(m_showPolicy);
          showToast(m_showPolicy ? "Policy panel: ON" : "Policy panel: OFF");
        } else if (name == "report") {
          m_showReport = want(m_showReport);
          showToast(m_showReport ? "City report: ON" : "City report: OFF");
        } else if (name == "traffic_model") {
          m_showTrafficModel = want(m_showTrafficModel);
          showToast(m_showTrafficModel ? "Traffic model: ON" : "Traffic model: OFF");
        } else if (name == "weather" || name == "wx") {
          auto s = m_renderer.weatherSettings();
          using M = Renderer::WeatherSettings::Mode;

          if (mode == "on" || mode == "1" || mode == "true") {
            if (s.mode == M::Clear) s.mode = M::Rain;
          } else if (mode == "off" || mode == "0" || mode == "false") {
            s.mode = M::Clear;
          } else { // toggle/cycle
            if (s.mode == M::Clear) s.mode = M::Rain;
            else if (s.mode == M::Rain) s.mode = M::Snow;
            else s.mode = M::Clear;
          }

          m_renderer.setWeatherSettings(s);
          showToast(std::string("Weather: ") + ((s.mode == M::Rain) ? "Rain" : (s.mode == M::Snow) ? "Snow" : "Clear"));
        } else if (name == "daynight" || name == "dn" || name == "lighting") {
          const bool enabled = want(m_renderer.dayNightEnabled());
          m_renderer.setDayNightEnabled(enabled);
          showToast(enabled ? "Day/night lighting: ON" : "Day/night lighting: OFF");
        } else if (name == "cache") {
          const bool enabled = want(m_renderer.baseCacheEnabled());
          m_renderer.setBaseCacheEnabled(enabled);
          m_renderer.markBaseCacheDirtyAll();
          showToast(enabled ? "Render cache: ON" : "Render cache: OFF");
        } else {
          c.print("Unknown overlay: " + args[0]);
          return;
        }
        c.print("ok");
      });


  m_console.registerCommand(
      "layer",
      "layer <terrain|decals|structures|overlays|all|none|status> [on|off|toggle]",
      [this, toLower](DevConsole& c, const DevConsole::Args& args) {
        auto printStatus = [&]() {
          c.print(TextFormat(
              "Layers: terrain=%s decals=%s structures=%s overlays=%s  mask=0x%X",
              m_renderer.layerEnabled(Renderer::RenderLayer::Terrain) ? "ON" : "OFF",
              m_renderer.layerEnabled(Renderer::RenderLayer::Decals) ? "ON" : "OFF",
              m_renderer.layerEnabled(Renderer::RenderLayer::Structures) ? "ON" : "OFF",
              m_renderer.layerEnabled(Renderer::RenderLayer::Overlays) ? "ON" : "OFF",
              static_cast<unsigned int>(m_renderer.layerMask())));
        };

        if (args.empty()) {
          printStatus();
          return;
        }

        const std::string name = toLower(args[0]);
        const std::string mode = (args.size() >= 2) ? toLower(args[1]) : "toggle";

        auto want = [&](bool current) {
          if (mode == "on" || mode == "1" || mode == "true") return true;
          if (mode == "off" || mode == "0" || mode == "false") return false;
          return !current;
        };

        auto applyAndToast = [&](const std::string& label) {
          // Layer changes can affect cached render content.
          m_renderer.markBaseCacheDirtyAll();
          showToast(label.c_str(), 2.0f);
        };

        if (name == "status") {
          printStatus();
          c.print("ok");
          return;
        }

        if (name == "all") {
          m_renderer.setLayerMask(Renderer::kLayerAll);
          applyAndToast("Layers: ALL");
          printStatus();
          c.print("ok");
          return;
        }

        if (name == "none") {
          m_renderer.setLayerMask(0u);
          applyAndToast("Layers: NONE");
          printStatus();
          c.print("ok");
          return;
        }

        auto setOne = [&](Renderer::RenderLayer layer, const char* label) {
          const bool enabled = want(m_renderer.layerEnabled(layer));
          m_renderer.setLayerEnabled(layer, enabled);
          applyAndToast(std::string(label) + (enabled ? ": ON" : ": OFF"));
          printStatus();
          c.print("ok");
        };

        if (name == "terrain") {
          setOne(Renderer::RenderLayer::Terrain, "Terrain");
        } else if (name == "decals") {
          setOne(Renderer::RenderLayer::Decals, "Decals");
        } else if (name == "structures") {
          setOne(Renderer::RenderLayer::Structures, "Structures");
        } else if (name == "overlays") {
          setOne(Renderer::RenderLayer::Overlays, "Overlays");
        } else {
          c.print("Unknown layer: " + args[0]);
        }
      });

  m_console.registerCommand(
      "daynight",
      "daynight [on|off|toggle] | daynight len <sec> | daynight strength <0..1> | daynight dusk <0..1> | "
      "daynight offset <sec> | daynight lights <on|off>",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        auto s = m_renderer.dayNightSettings();

        auto printStatus = [&]() {
          c.print(TextFormat("Day/night: %s  len=%0.1fs  offset=%0.1fs  strength=%0.2f  dusk=%0.2f  lights=%s",
                             s.enabled ? "ON" : "OFF", s.dayLengthSec, s.timeOffsetSec, s.nightDarken, s.duskTint,
                             s.drawLights ? "ON" : "OFF"));
        };

        if (args.empty()) {
          printStatus();
          return;
        }

        const std::string cmd = toLower(args[0]);

        if (cmd == "on" || cmd == "off" || cmd == "toggle") {
          if (cmd == "toggle") {
            s.enabled = !s.enabled;
          } else {
            s.enabled = (cmd == "on");
          }
          m_renderer.setDayNightSettings(s);
          showToast(s.enabled ? "Day/night lighting: ON" : "Day/night lighting: OFF");
          printStatus();
          c.print("ok");
          return;
        }

        if ((cmd == "len" || cmd == "length") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad number: " + args[1]);
            return;
          }
          s.dayLengthSec = std::max(1.0f, v);
          m_renderer.setDayNightSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if ((cmd == "strength" || cmd == "night") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad number: " + args[1]);
            return;
          }
          s.nightDarken = std::clamp(v, 0.0f, 1.0f);
          m_renderer.setDayNightSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if ((cmd == "dusk" || cmd == "twilight") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad number: " + args[1]);
            return;
          }
          s.duskTint = std::clamp(v, 0.0f, 1.0f);
          m_renderer.setDayNightSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if (cmd == "offset" && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad number: " + args[1]);
            return;
          }
          s.timeOffsetSec = v;
          m_renderer.setDayNightSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if (cmd == "lights" && args.size() >= 2) {
          const std::string v = toLower(args[1]);
          if (v == "on" || v == "1" || v == "true") {
            s.drawLights = true;
          } else if (v == "off" || v == "0" || v == "false") {
            s.drawLights = false;
          } else {
            c.print("Expected on/off, got: " + args[1]);
            return;
          }
          m_renderer.setDayNightSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        c.print("Usage:");
        c.print("  daynight");
        c.print("  daynight on|off|toggle");
        c.print("  daynight len <sec>");
        c.print("  daynight strength <0..1>");
        c.print("  daynight dusk <0..1>");
        c.print("  daynight offset <sec>");
        c.print("  daynight lights <on|off>");
      });




  // --- shadow ---
  m_console.registerCommand(
      "shadow",
      "shadow [on|off|toggle] | shadow strength <0..1> | shadow softness <0..1> | shadow dir <deg> | shadow maxlen <tiles> | shadow alt <minDeg> <maxDeg>",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        auto s = m_renderer.shadowSettings();

        auto clamp01 = [&](float v) { return std::max(0.0f, std::min(1.0f, v)); };

        auto printStatus = [&]() {
          c.print(TextFormat("Shadows: %s  strength=%.2f  softness=%.2f  dir=%.1fdeg  maxlen=%.1ftiles  alt=[%.1f..%.1f]deg",
                             s.enabled ? "ON" : "OFF", s.strength, s.softness, s.azimuthDeg, s.maxLengthTiles,
                             s.minAltitudeDeg, s.maxAltitudeDeg));
        };

        if (args.empty()) {
          printStatus();
          return;
        }

        const std::string a0 = toLower(args[0]);

        if (a0 == "on" || a0 == "off" || a0 == "toggle") {
          if (a0 == "toggle") s.enabled = !s.enabled;
          else s.enabled = (a0 == "on");

          m_renderer.setShadowSettings(s);
          showToast(s.enabled ? "Shadows: ON" : "Shadows: OFF");
          printStatus();
          c.print("ok");
          return;
        }

        if ((a0 == "strength" || a0 == "alpha") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad strength value."); return; }
          s.strength = clamp01(v);
          m_renderer.setShadowSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if ((a0 == "soft" || a0 == "softness") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad softness value."); return; }
          s.softness = clamp01(v);
          m_renderer.setShadowSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if ((a0 == "dir" || a0 == "azimuth" || a0 == "angle") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad direction angle."); return; }
          s.azimuthDeg = v;
          m_renderer.setShadowSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if ((a0 == "max" || a0 == "maxlen" || a0 == "length") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad max length."); return; }
          s.maxLengthTiles = std::max(0.0f, v);
          m_renderer.setShadowSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        if ((a0 == "alt" || a0 == "altitude") && args.size() >= 3) {
          float a = 0.0f, b = 0.0f;
          if (!parseF32(args[1], a) || !parseF32(args[2], b)) { c.print("Bad altitude range."); return; }
          if (b < a) { const float tmp = a; a = b; b = tmp; }
          s.minAltitudeDeg = std::clamp(a, 1.0f, 89.0f);
          s.maxAltitudeDeg = std::clamp(b, 1.0f, 89.0f);
          if (s.maxAltitudeDeg < s.minAltitudeDeg) s.maxAltitudeDeg = s.minAltitudeDeg;
          m_renderer.setShadowSettings(s);
          printStatus();
          c.print("ok");
          return;
        }

        c.print("Usage:");
        c.print("  shadow");
        c.print("  shadow on|off|toggle");
        c.print("  shadow strength <0..1>");
        c.print("  shadow softness <0..1>");
        c.print("  shadow dir <deg>");
        c.print("  shadow maxlen <tiles>");
        c.print("  shadow alt <minDeg> <maxDeg>");
      });
  // --- weather ---
  m_console.registerCommand(
      "weather",
      "weather [clear|rain|snow|toggle] | weather intensity <0..1> | weather wind <deg> [speed] | weather overcast <0..1> | "
      "weather fog <0..1> | weather ground <on|off> | weather particles <on|off> | weather reflect <on|off>",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        auto s = m_renderer.weatherSettings();
        using M = Renderer::WeatherSettings::Mode;

        auto modeName = [&](M m) -> const char* {
          switch (m) {
          case M::Rain: return "Rain";
          case M::Snow: return "Snow";
          default: return "Clear";
          }
        };

        auto clamp01 = [&](float v) { return std::max(0.0f, std::min(1.0f, v)); };

        auto parseOnOff = [&](const std::string& v, bool cur) -> bool {
          const std::string t = toLower(v);
          if (t == "on" || t == "1" || t == "true" || t == "yes") return true;
          if (t == "off" || t == "0" || t == "false" || t == "no") return false;
          return cur;
        };

        auto printStatus = [&]() {
          c.print(TextFormat("Weather: %s  intensity=%.2f  wind=%.1fdeg x%.2f  overcast=%.2f  fog=%.2f  ground=%s  particles=%s  reflect=%s",
                             modeName(s.mode), s.intensity, s.windAngleDeg, s.windSpeed, s.overcast, s.fog,
                             s.affectGround ? "ON" : "OFF", s.drawParticles ? "ON" : "OFF",
                             s.reflectLights ? "ON" : "OFF"));
        };

        if (args.empty()) {
          printStatus();
          return;
        }

        const std::string a0 = toLower(args[0]);

        if (a0 == "clear" || a0 == "rain" || a0 == "snow" || a0 == "toggle") {
          if (a0 == "clear") s.mode = M::Clear;
          else if (a0 == "rain") s.mode = M::Rain;
          else if (a0 == "snow") s.mode = M::Snow;
          else {
            if (s.mode == M::Clear) s.mode = M::Rain;
            else if (s.mode == M::Rain) s.mode = M::Snow;
            else s.mode = M::Clear;
          }

          if (s.mode != M::Clear && s.intensity < 0.05f) s.intensity = 0.80f;

          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if (a0 == "intensity" && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad intensity value."); return; }
          s.intensity = clamp01(v);
          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if (a0 == "wind" && args.size() >= 2) {
          float deg = 0.0f;
          if (!parseF32(args[1], deg)) { c.print("Bad wind angle."); return; }
          s.windAngleDeg = deg;

          if (args.size() >= 3) {
            float spd = 1.0f;
            if (!parseF32(args[2], spd)) { c.print("Bad wind speed."); return; }
            s.windSpeed = std::max(0.05f, spd);
          }

          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if (a0 == "overcast" && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad overcast value."); return; }
          s.overcast = clamp01(v);
          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if (a0 == "fog" && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) { c.print("Bad fog value."); return; }
          s.fog = clamp01(v);
          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "ground" || a0 == "affect") && args.size() >= 2) {
          s.affectGround = parseOnOff(args[1], s.affectGround);
          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if (a0 == "particles" && args.size() >= 2) {
          s.drawParticles = parseOnOff(args[1], s.drawParticles);
          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        if (a0 == "reflect" && args.size() >= 2) {
          s.reflectLights = parseOnOff(args[1], s.reflectLights);
          m_renderer.setWeatherSettings(s);
          printStatus();
          return;
        }

        c.print("Usage:");
        c.print("  weather                           (show status)");
        c.print("  weather <clear|rain|snow|toggle>");
        c.print("  weather intensity <0..1>");
        c.print("  weather wind <deg> [speed]");
        c.print("  weather overcast <0..1>");
        c.print("  weather fog <0..1>");
        c.print("  weather ground <on|off>");
        c.print("  weather particles <on|off>");
        c.print("  weather reflect <on|off>");
      });


  // --- organic material (animated procedural decal) ---
  m_console.registerCommand(
      "organic",
      "organic [on|off|toggle] | organic style <moss|slime|mycelium|bio> | organic alpha <0..1> | organic speed <0..4> | "
      "organic steps <1..16> | organic scale <0.25..4> | organic feed <0..0.1> | organic kill <0..0.1> | organic du <0..1> | "
      "organic dv <0..1> | organic glow <0..1> | organic reset [seed]",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        auto s = m_renderer.organicMaterialSettings();

        auto clamp01 = [&](float v) { return std::max(0.0f, std::min(1.0f, v)); };

        auto printStatus = [&]() {
          const char* styleStr = "moss";
          switch (s.style) {
            case OrganicMaterial::Style::Moss: styleStr = "moss"; break;
            case OrganicMaterial::Style::Slime: styleStr = "slime"; break;
            case OrganicMaterial::Style::Mycelium: styleStr = "mycelium"; break;
            case OrganicMaterial::Style::Bioluminescent: styleStr = "bio"; break;
          }

          c.print(TextFormat(
              "Organic: %s  style=%s  alpha=%.2f  speed=%.2f  steps=%d  scale=%.2f  feed=%.4f  kill=%.4f  du=%.3f  dv=%.3f  glow=%.2f",
              s.enabled ? "ON" : "OFF", styleStr, s.alpha, s.speed, s.stepsPerFrame, s.patternScale, s.feed, s.kill, s.diffusionU,
              s.diffusionV, s.glowStrength));
        };

        auto parseOnOffToggle = [&](const std::string& v, bool cur, bool& out) -> bool {
          const std::string t = toLower(v);
          if (t == "on" || t == "1" || t == "true" || t == "yes") {
            out = true;
            return true;
          }
          if (t == "off" || t == "0" || t == "false" || t == "no") {
            out = false;
            return true;
          }
          if (t == "toggle") {
            out = !cur;
            return true;
          }
          return false;
        };

        if (args.empty()) {
          printStatus();
          return;
        }

        const std::string a0 = toLower(args[0]);

        // on/off/toggle
        {
          bool v = s.enabled;
          if (parseOnOffToggle(a0, s.enabled, v)) {
            s.enabled = v;
            m_renderer.setOrganicMaterialSettings(s);
            printStatus();
            showToast(TextFormat("Organic material: %s", s.enabled ? "ON" : "OFF"), 1.5f);
            return;
          }
        }

        if ((a0 == "style" || a0 == "preset") && args.size() >= 2) {
          const std::string st = toLower(args[1]);

          // A couple of pleasing Gray-Scott parameter presets.
          auto applyPreset = [&](OrganicMaterial::Style style, float f, float k, float du, float dv) {
            s.style = style;
            s.feed = f;
            s.kill = k;
            s.diffusionU = du;
            s.diffusionV = dv;
          };

          if (st == "moss") {
            applyPreset(OrganicMaterial::Style::Moss, 0.035f, 0.065f, 0.16f, 0.08f);
          } else if (st == "slime") {
            applyPreset(OrganicMaterial::Style::Slime, 0.022f, 0.051f, 0.16f, 0.08f);
          } else if (st == "mycelium" || st == "myc") {
            applyPreset(OrganicMaterial::Style::Mycelium, 0.037f, 0.060f, 0.16f, 0.08f);
          } else if (st == "bio" || st == "biolume" || st == "bioluminescent") {
            applyPreset(OrganicMaterial::Style::Bioluminescent, 0.0367f, 0.0649f, 0.16f, 0.08f);
          } else {
            c.print("Unknown style. Use: moss | slime | mycelium | bio");
            return;
          }

          m_renderer.setOrganicMaterialSettings(s);
          // Reseed so the new preset feels immediately different.
          const std::uint32_t seed32 = static_cast<std::uint32_t>(m_cfg.seed) ^ static_cast<std::uint32_t>(m_cfg.seed >> 32) ^
                                      static_cast<std::uint32_t>(GetTime() * 1000.0);
          m_renderer.resetOrganicMaterial(seed32);
          printStatus();
          return;
        }

        if ((a0 == "alpha" || a0 == "a") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad alpha.");
            return;
          }
          s.alpha = clamp01(v);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "speed" || a0 == "spd") && args.size() >= 2) {
          float v = 1.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad speed.");
            return;
          }
          s.speed = std::clamp(v, 0.0f, 4.0f);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "steps" || a0 == "iters" || a0 == "iter") && args.size() >= 2) {
          int v = 4;
          try {
            v = std::stoi(args[1]);
          } catch (...) {
            c.print("Bad steps.");
            return;
          }
          s.stepsPerFrame = std::clamp(v, 1, 16);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "scale" || a0 == "freq") && args.size() >= 2) {
          float v = 1.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad scale.");
            return;
          }
          s.patternScale = std::clamp(v, 0.25f, 4.0f);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "feed" || a0 == "f") && args.size() >= 2) {
          float v = s.feed;
          if (!parseF32(args[1], v)) {
            c.print("Bad feed.");
            return;
          }
          s.feed = std::clamp(v, 0.0f, 0.1f);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "kill" || a0 == "k") && args.size() >= 2) {
          float v = s.kill;
          if (!parseF32(args[1], v)) {
            c.print("Bad kill.");
            return;
          }
          s.kill = std::clamp(v, 0.0f, 0.1f);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "du" || a0 == "diffu") && args.size() >= 2) {
          float v = s.diffusionU;
          if (!parseF32(args[1], v)) {
            c.print("Bad du.");
            return;
          }
          s.diffusionU = std::clamp(v, 0.0f, 1.0f);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "dv" || a0 == "diffv") && args.size() >= 2) {
          float v = s.diffusionV;
          if (!parseF32(args[1], v)) {
            c.print("Bad dv.");
            return;
          }
          s.diffusionV = std::clamp(v, 0.0f, 1.0f);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "glow" || a0 == "g") && args.size() >= 2) {
          float v = s.glowStrength;
          if (!parseF32(args[1], v)) {
            c.print("Bad glow.");
            return;
          }
          s.glowStrength = clamp01(v);
          m_renderer.setOrganicMaterialSettings(s);
          printStatus();
          return;
        }

        if (a0 == "reset") {
          std::uint32_t seed32 = static_cast<std::uint32_t>(m_cfg.seed) ^ static_cast<std::uint32_t>(m_cfg.seed >> 32) ^
                                static_cast<std::uint32_t>(GetTime() * 1000.0);
          if (args.size() >= 2) {
            try {
              const std::uint64_t parsed = static_cast<std::uint64_t>(std::stoull(args[1]));
              seed32 = static_cast<std::uint32_t>(parsed) ^ static_cast<std::uint32_t>(parsed >> 32);
            } catch (...) {
              c.print("Bad seed.");
              return;
            }
          }
          m_renderer.resetOrganicMaterial(seed32);
          c.print(TextFormat("Organic reset (seed=%u)", seed32));
          return;
        }

        c.print("Usage:");
        c.print("  organic                              (show status)");
        c.print("  organic <on|off|toggle>");
        c.print("  organic style <moss|slime|mycelium|bio>");
        c.print("  organic alpha <0..1>");
        c.print("  organic speed <0..4>");
        c.print("  organic steps <1..16>");
        c.print("  organic scale <0.25..4>");
        c.print("  organic feed <0..0.1>");
        c.print("  organic kill <0..0.1>");
        c.print("  organic du <0..1>");
        c.print("  organic dv <0..1>");
        c.print("  organic glow <0..1>");
        c.print("  organic reset [seed]");
      });


  // --- cloud shadows ---
  m_console.registerCommand(
      "cloud",
      "cloud [on|off|toggle] | cloud strength <0..1> | cloud scale <0.25..8> | cloud speed <0..3> | cloud evolve <0..1> | "
      "cloud coverage <0..1> | cloud softness <0..1>",
      [this, toLower, parseF32](DevConsole& c, const DevConsole::Args& args) {
        auto s = m_renderer.cloudShadowSettings();

        auto clamp01 = [&](float v) { return std::max(0.0f, std::min(1.0f, v)); };

        auto printStatus = [&]() {
          c.print(TextFormat("CloudShadows: %s  strength=%.2f  scale=%.2f  speed=%.2f  evolve=%.2f  coverage=%.2f  softness=%.2f",
                             s.enabled ? "ON" : "OFF", s.strength, s.scale, s.speed, s.evolve, s.coverage, s.softness));
        };

        auto parseOnOffToggle = [&](const std::string& v, bool cur, bool& out) -> bool {
          const std::string t = toLower(v);
          if (t == "on" || t == "1" || t == "true" || t == "yes") {
            out = true;
            return true;
          }
          if (t == "off" || t == "0" || t == "false" || t == "no") {
            out = false;
            return true;
          }
          if (t == "toggle") {
            out = !cur;
            return true;
          }
          return false;
        };

        if (args.empty()) {
          printStatus();
          return;
        }

        const std::string a0 = toLower(args[0]);

        // on/off/toggle
        {
          bool v = s.enabled;
          if (parseOnOffToggle(a0, s.enabled, v)) {
            s.enabled = v;
            m_renderer.setCloudShadowSettings(s);
            printStatus();
            showToast(TextFormat("Cloud shadows: %s", s.enabled ? "ON" : "OFF"), 1.5f);
            return;
          }
        }

        if (a0 == "strength" && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad strength value.");
            return;
          }
          s.strength = clamp01(v);
          m_renderer.setCloudShadowSettings(s);
          printStatus();
          return;
        }

        if (a0 == "scale" && args.size() >= 2) {
          float v = 1.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad scale value.");
            return;
          }
          s.scale = std::clamp(v, 0.25f, 8.0f);
          m_renderer.setCloudShadowSettings(s);
          printStatus();
          return;
        }

        if (a0 == "speed" && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad speed value.");
            return;
          }
          s.speed = std::clamp(v, 0.0f, 3.0f);
          m_renderer.setCloudShadowSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "evolve" || a0 == "morph") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad evolve value.");
            return;
          }
          s.evolve = clamp01(v);
          m_renderer.setCloudShadowSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "coverage" || a0 == "cover") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad coverage value.");
            return;
          }
          s.coverage = clamp01(v);
          m_renderer.setCloudShadowSettings(s);
          printStatus();
          return;
        }

        if ((a0 == "softness" || a0 == "soft") && args.size() >= 2) {
          float v = 0.0f;
          if (!parseF32(args[1], v)) {
            c.print("Bad softness value.");
            return;
          }
          s.softness = clamp01(v);
          m_renderer.setCloudShadowSettings(s);
          printStatus();
          return;
        }

        c.print("Usage:");
        c.print("  cloud                               (show status)");
        c.print("  cloud on|off|toggle");
        c.print("  cloud strength <0..1>");
        c.print("  cloud scale <0.25..8>");
        c.print("  cloud speed <0..3>");
        c.print("  cloud evolve <0..1>");
        c.print("  cloud coverage <0..1>");
        c.print("  cloud softness <0..1>");
      });



  // --- file export ---
  m_console.registerCommand(
      "shot", "shot          - capture a screenshot to captures/ (same as F12)",
      [this](DevConsole& c, const DevConsole::Args&) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");
        const std::string path =
            TextFormat("captures/screenshot_seed%llu_%s.png", static_cast<unsigned long long>(m_cfg.seed),
                       FileTimestamp().c_str());
        m_pendingScreenshot = true;
        m_pendingScreenshotPath = path;
        showToast(TextFormat("Queued screenshot: %s", path.c_str()), 2.0f);
        c.print("queued: " + path);
      });

  m_console.registerCommand(
      "map", "map [maxSize] [path] - export a world overview PNG to captures/ (queued)",
      [this, parseI64, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        // Defaults
        int maxSize = 4096;
        std::string path =
            TextFormat("captures/map_seed%llu_%s.png", static_cast<unsigned long long>(m_cfg.seed), FileTimestamp().c_str());

        auto clampSize = [](long long v) {
          // Keep this sane; exportWorldOverview may allocate a large render texture.
          return std::clamp(static_cast<int>(v), 64, 16384);
        };

        if (!args.empty()) {
          long long v = 0;
          // Allow either:
          //   map 4096
          //   map 4096 my.png
          //   map my.png
          //   map my.png 4096
          if (parseI64(args[0], v)) {
            maxSize = clampSize(v);
            if (args.size() >= 2) {
              path = joinArgs(args, 1);
            }
          } else {
            // Path first.
            path = joinArgs(args, 0);

            // If the last token is a number, treat it as maxSize.
            if (args.size() >= 2 && parseI64(args.back(), v)) {
              maxSize = clampSize(v);
              std::string p;
              for (std::size_t i = 0; i + 1 < args.size(); ++i) {
                if (!p.empty()) p.push_back(' ');
                p += args[i];
              }
              if (!p.empty()) path = p;
            }
          }
        }

        if (path.empty()) {
          c.print("Usage: map [maxSize] [path]");
          return;
        }

        m_pendingMapExport = true;
        m_pendingMapExportPath = path;
        m_pendingMapExportMaxSize = maxSize;
        showToast(TextFormat("Queued map export (%dpx): %s", maxSize, path.c_str()), 2.0f);
        c.print(TextFormat("queued: %s (maxSize=%d)", path.c_str(), maxSize));
      });

  m_console.registerCommand(
      "map_layers",
      "map_layers [maxSize] [prefix] - export layered world overview PNGs (queued; writes *_terrain/_decals/_structures/_overlays/_weather_fx)",
      [this, parseI64, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        // Defaults
        int maxSize = 4096;
        std::string prefix = TextFormat("captures/maplayers_seed%llu_%s.png",
                                        static_cast<unsigned long long>(m_cfg.seed),
                                        FileTimestamp().c_str());

        auto clampSize = [](long long v) {
          // Keep this sane; exportWorldOverview may allocate a large render texture.
          return std::clamp(static_cast<int>(v), 64, 16384);
        };

        if (!args.empty()) {
          long long v = 0;
          // Allow either:
          //   map_layers 4096
          //   map_layers 4096 myprefix.png
          //   map_layers myprefix.png
          //   map_layers myprefix.png 4096
          if (parseI64(args[0], v)) {
            maxSize = clampSize(v);
            if (args.size() >= 2) {
              prefix = joinArgs(args, 1);
            }
          } else {
            // Prefix first.
            prefix = joinArgs(args, 0);

            // If the last token is a number, treat it as maxSize.
            if (args.size() >= 2 && parseI64(args.back(), v)) {
              maxSize = clampSize(v);
              std::string p;
              for (std::size_t i = 0; i + 1 < args.size(); ++i) {
                if (!p.empty()) p.push_back(' ');
                p += args[i];
              }
              if (!p.empty()) prefix = p;
            }
          }
        }

        if (prefix.empty()) {
          c.print("Usage: map_layers [maxSize] [prefix]");
          return;
        }

        m_pendingMapLayersExport = true;
        m_pendingMapLayersPrefix = prefix;
        m_pendingMapLayersMaxSize = maxSize;
        showToast(TextFormat("Queued layer export (%dpx): %s", maxSize, prefix.c_str()), 2.0f);
        c.print(TextFormat("queued: %s (maxSize=%d)", prefix.c_str(), maxSize));
      });

  m_console.registerCommand(
      "render3d",
      "render3d [WxH] [layer] [iso|persp] [ssaa=N] [grade|nograde] [path] - export a software 3D render (queued; Ctrl+F11)",
      [this, joinArgs, toLower, parseI64](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        auto parseWxH = [&](const std::string& s, int& w, int& h) -> bool {
          const std::size_t x = s.find_first_of("xX");
          if (x == std::string::npos) return false;
          long long a = 0, b = 0;
          if (!parseI64(s.substr(0, x), a)) return false;
          if (!parseI64(s.substr(x + 1), b)) return false;
          w = static_cast<int>(a);
          h = static_cast<int>(b);
          return (w > 0 && h > 0);
        };

        auto parseSsaa = [&](const std::string& s, int& out) -> bool {
          const std::string t = toLower(s);
          const std::string k0 = "ssaa=";
          const std::string k1 = "ss=";
          const std::string k2 = "supersample=";
          std::string v;
          if (t.rfind(k0, 0) == 0) v = t.substr(k0.size());
          else if (t.rfind(k1, 0) == 0) v = t.substr(k1.size());
          else if (t.rfind(k2, 0) == 0) v = t.substr(k2.size());
          else return false;
          long long n = 0;
          if (!parseI64(v, n)) return false;
          out = std::clamp(static_cast<int>(n), 1, 4);
          return true;
        };

        auto parseLayer = [&](const std::string& s, ExportLayer& out) -> bool {
          return ParseExportLayer(s, out);
        };

        Render3DConfig cfg = m_pendingRender3DCfg;
        ExportLayer layer = m_pendingRender3DLayer;
        bool applyGrade = m_pendingRender3DApplyGrade;

        std::string path = TextFormat("captures/render3d_seed%llu_%s.png",
                                      static_cast<unsigned long long>(m_cfg.seed),
                                      FileTimestamp().c_str());

        // Parse args: first unrecognized token becomes the path.
        for (std::size_t i = 0; i < args.size(); ++i) {
          const std::string t = toLower(args[i]);

          int w = 0, h = 0;
          if (parseWxH(args[i], w, h)) {
            cfg.width = std::clamp(w, 64, 16384);
            cfg.height = std::clamp(h, 64, 16384);
            continue;
          }

          int ss = 0;
          if (parseSsaa(args[i], ss)) {
            cfg.supersample = ss;
            continue;
          }

          if (t == "iso" || t == "ortho" || t == "isometric") {
            cfg.projection = Render3DConfig::Projection::IsometricOrtho;
            continue;
          }

          if (t == "persp" || t == "perspective") {
            cfg.projection = Render3DConfig::Projection::Perspective;
            continue;
          }

          if (t == "grade" || t == "graded") {
            applyGrade = true;
            continue;
          }
          if (t == "nograde" || t == "raw") {
            applyGrade = false;
            continue;
          }

          ExportLayer parsedLayer;
          if (parseLayer(t, parsedLayer)) {
            layer = parsedLayer;
            continue;
          }

          // Treat the remaining tokens as a path.
          path = joinArgs(args, i);
          break;
        }

        if (path.empty()) {
          c.print("Usage: render3d [WxH] [layer] [iso|persp] [ssaa=N] [grade|nograde] [path]");
          return;
        }

        m_pendingRender3D = true;
        m_pendingRender3DPath = path;
        m_pendingRender3DCfg = cfg;
        m_pendingRender3DLayer = layer;
        m_pendingRender3DApplyGrade = applyGrade;

        showToast(TextFormat("Queued 3D render: %s", path.c_str()), 2.0f);
        c.print(TextFormat("queued: %s (%dx%d ssaa=%d %s %s)", path.c_str(), cfg.width, cfg.height, cfg.supersample,
                           (cfg.projection == Render3DConfig::Projection::Perspective) ? "persp" : "iso",
                           applyGrade ? "grade" : "raw"));
      });

  m_console.registerCommand(
      "dossier",
      "dossier [outDir] [scale=N] [fmt=png|ppm] [iso=0|1] [3d=0|1] [layers=...] [isolayers=...] - export a city dossier bundle (queued)",
      [this, toLower, parseI32](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        CityDossierConfig cfg;
        cfg.outDir = fs::path(TextFormat("captures/dossier_seed%llu_%s",
                                         static_cast<unsigned long long>(m_cfg.seed),
                                         FileTimestamp().c_str()));

        // Reasonable defaults for interactive use.
        cfg.exportScale = 2;
        cfg.format = "png";
        cfg.exportIso = true;
        cfg.export3d = false;

        // Make software 3D fast enough to be usable (users can override with 3d=1 and tweak later).
        cfg.render3dCfg.width = 960;
        cfg.render3dCfg.height = 540;
        cfg.render3dCfg.supersample = 2;
        cfg.render3dCfg.projection = Render3DConfig::Projection::IsometricOrtho;
        cfg.render3dCfg.timeOfDay = 0.42f;
        cfg.render3dCfg.windDirDeg = 125.0f;
        cfg.render3dCfg.rainStrength = 0.10f;
        cfg.render3dCfg.cloudStrength = 0.25f;
        cfg.render3dCfg.postGrade = true;
        cfg.render3dCfg.postVignette = true;
        cfg.render3dCfg.postBloom = true;
        cfg.render3dCfg.postBloomStrength = 0.12f;
        cfg.render3dCfg.postBloomRadius = 0.80f;
        cfg.render3dCfg.postDither = true;
        cfg.render3dCfg.postDitherStrength = 0.22f;

        bool outDirSet = false;

        auto parseBool = [&](const std::string& v, bool& out) -> bool {
          const std::string t = toLower(v);
          if (t == "1" || t == "true" || t == "on" || t == "yes") { out = true; return true; }
          if (t == "0" || t == "false" || t == "off" || t == "no") { out = false; return true; }
          return false;
        };

        auto parseLayerList = [&](const std::string& csv, std::vector<ExportLayer>& outLayers) -> bool {
          outLayers.clear();
          std::string token;
          for (std::size_t i = 0; i <= csv.size(); ++i) {
            if (i == csv.size() || csv[i] == ',') {
              if (!token.empty()) {
                ExportLayer l;
                if (!ParseExportLayer(token, l)) {
                  return false;
                }
                outLayers.push_back(l);
                token.clear();
              }
              continue;
            }
            token.push_back(csv[i]);
          }
          return !outLayers.empty();
        };

        for (std::size_t i = 0; i < args.size(); ++i) {
          const std::string a = args[i];
          const std::size_t eq = a.find('=');
          if (eq != std::string::npos) {
            const std::string k = toLower(a.substr(0, eq));
            const std::string v = a.substr(eq + 1);

            if (k == "scale" || k == "s") {
              int sc = 0;
              if (!parseI32(v, sc) || sc < 1 || sc > 16) {
                c.print("Bad scale. Expected 1..16");
                return;
              }
              cfg.exportScale = sc;
              continue;
            }

            if (k == "fmt" || k == "format") {
              cfg.format = toLower(v);
              continue;
            }

            if (k == "iso") {
              bool b = cfg.exportIso;
              if (!parseBool(v, b)) {
                c.print("Bad iso. Use iso=0|1");
                return;
              }
              cfg.exportIso = b;
              continue;
            }

            if (k == "3d" || k == "render3d") {
              bool b = cfg.export3d;
              if (!parseBool(v, b)) {
                c.print("Bad 3d. Use 3d=0|1");
                return;
              }
              cfg.export3d = b;
              continue;
            }

            if (k == "layers") {
              if (!parseLayerList(v, cfg.layers2d)) {
                c.print("Bad layers list. Example: layers=terrain,overlay,landvalue");
                return;
              }
              continue;
            }

            if (k == "isolayers" || k == "iso_layers") {
              if (!parseLayerList(v, cfg.layersIso)) {
                c.print("Bad isolayers list. Example: isolayers=overlay,landvalue");
                return;
              }
              continue;
            }

            if (k == "tilemetrics" || k == "tile_metrics") {
              bool b = cfg.writeTileMetricsCsv;
              if (!parseBool(v, b)) {
                c.print("Bad tilemetrics. Use tilemetrics=0|1");
                return;
              }
              cfg.writeTileMetricsCsv = b;
              continue;
            }

            if (k == "ticks") {
              bool b = cfg.writeTicksCsv;
              if (!parseBool(v, b)) {
                c.print("Bad ticks. Use ticks=0|1");
                return;
              }
              cfg.writeTicksCsv = b;
              continue;
            }

            if (k == "world" || k == "worldbin") {
              bool b = cfg.writeWorldBinary;
              if (!parseBool(v, b)) {
                c.print("Bad world. Use world=0|1");
                return;
              }
              cfg.writeWorldBinary = b;
              continue;
            }

            if (k == "html") {
              bool b = cfg.writeHtml;
              if (!parseBool(v, b)) {
                c.print("Bad html. Use html=0|1");
                return;
              }
              cfg.writeHtml = b;
              // Keep summary.json if html is requested.
              if (cfg.writeHtml) cfg.writeSummaryJson = true;
              continue;
            }

            c.print("Unknown option: " + a);
            continue;
          }

          // Positional: first non key=value token is treated as the output dir.
          if (!outDirSet) {
            cfg.outDir = fs::path(a);
            outDirSet = true;
            continue;
          }

          c.print("Unexpected arg: " + a);
          return;
        }

        if (cfg.outDir.empty()) {
          c.print("Usage: dossier [outDir] [scale=N] [fmt=png|ppm] [iso=0|1] [3d=0|1] [layers=...] [isolayers=...]");
          return;
        }

        endPaintStroke();
        const std::string outDirStr = cfg.outDir.string();
        queueDossierCurrentWorld(std::move(cfg));
        c.print(std::string("queued: ") + outDirStr);
      });


  m_console.registerCommand(
      "dossier_status",
      "dossier_status - show background dossier export status",
      [this](DevConsole& c, const DevConsole::Args& /*args*/) {
        if (m_dossierJob) {
          int si = m_dossierJob->stepIndex.load(std::memory_order_relaxed);
          int sc = m_dossierJob->stepCount.load(std::memory_order_relaxed);
          int seedI = m_dossierJob->seedIndex.load(std::memory_order_relaxed);
          int seedC = m_dossierJob->seedCount.load(std::memory_order_relaxed);
          bool cancel = m_dossierJob->cancel.load(std::memory_order_relaxed);
          std::string stage;
          {
            std::lock_guard<std::mutex> lock(m_dossierJob->mutex);
            stage = m_dossierJob->stage;
          }

          std::string seedPrefix;
          if (seedC > 0) {
            seedPrefix = TextFormat("[seed %d/%d] ", seedI, seedC);
          }

          if (sc > 0) {
            const float pct = 100.0f * (static_cast<float>(si) / static_cast<float>(sc));
            c.print(TextFormat("dossier: %s%s  %d/%d (%.1f%%)%s", seedPrefix.c_str(), stage.c_str(), si, sc,
                               static_cast<double>(pct), cancel ? "  [cancel requested]" : ""));
          } else {
            c.print(std::string("dossier: ") + seedPrefix + stage + (cancel ? "  [cancel requested]" : ""));
          }

          if (!m_dossierQueue.empty()) {
            c.print(TextFormat("dossier: %d job(s) queued", (int)m_dossierQueue.size()));
          }
          return;
        }

        if (!m_dossierQueue.empty()) {
          const auto& next = m_dossierQueue.front();
          c.print(TextFormat("dossier: queued %d job(s) (next outDir=%s)", (int)m_dossierQueue.size(),
                             next.cfg.outDir.string().c_str()));
          return;
        }

        if (!m_lastDossierMsg.empty()) {
          c.print(std::string("dossier: (idle)  last: ") + m_lastDossierMsg);
        } else {
          c.print("dossier: (idle)");
        }
      });

  

  m_console.registerCommand(
      "dossier_cancel",
      "dossier_cancel - request cancellation of the background dossier export",
      [this](DevConsole& c, const DevConsole::Args& /*args*/) {
        if (!m_dossierJob) {
          c.print("dossier: (idle)");
          return;
        }
        m_dossierJob->cancel.store(true, std::memory_order_relaxed);
        showToast("Dossier cancel requested", 1.8f);
        c.print("dossier: cancel requested");
        if (!m_dossierQueue.empty()) {
          c.print(TextFormat("dossier: %d job(s) still queued", (int)m_dossierQueue.size()));
        }
      });

  m_console.registerCommand(
      "dossier_clear",
      "dossier_clear - clear queued dossier requests (does not cancel the active job)",
      [this](DevConsole& c, const DevConsole::Args& /*args*/) {
        const int n = static_cast<int>(m_dossierQueue.size());
        m_dossierQueue.clear();
        c.print(TextFormat("dossier: cleared %d queued job(s)", n));
        showToast(TextFormat("Cleared %d dossier job(s)", n), 1.6f);
      });

  m_console.registerCommand(
      "mine",
      "mine <begin|stop|status|step|auto|top|load|export> ... - seed mining (headless) integrated into the app",
      [this, toLower, parseI32, parseU64, parseF32, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        auto usage = [&]() {
          c.print("Seed mining (headless) - uses the same core as proc_isocity_mine");
          c.print("Usage:");
          c.print("  mine begin <samples> [objective] [key=value...]  - start a new mining session (auto-runs)");
          c.print("       keys: days=N seed_start=U64 seed_step=U64 hydro=0|1 sea=0..1 topK=N diverse=0|1 pareto=0|1 crowding=0|1");
          c.print("  mine status                                 - show status / best seed so far");
          c.print("  mine auto on|off|toggle [steps=N] [budget=MS] - enable/disable auto-stepping each frame");
          c.print("  mine step [N]                               - synchronously mine N samples now");
          c.print("  mine top [K] [diverse=0|1] [pareto=0|1] [crowding=0|1] - list top seeds (score/mmr or pareto)");
          c.print("  mine load <rank>                            - reset the game world to the Nth best seed (rank starts at 1)");
          c.print("  mine export <path.csv|path.json>            - export mined records");
          c.print("  mine stop                                  - stop and discard the current session");
          c.print("Objectives: balanced, growth, resilient, chaos");
        };

        if (args.empty()) {
          usage();
          return;
        }

        const std::string sub = toLower(args[0]);

        auto parseBool = [&](const std::string& v, bool& out) -> bool {
          const std::string t = toLower(v);
          if (t == "1" || t == "true" || t == "on" || t == "yes") { out = true; return true; }
          if (t == "0" || t == "false" || t == "off" || t == "no") { out = false; return true; }
          if (t == "toggle") { out = !out; return true; }
          return false;
        };

        auto bestRecord = [&]() -> const MineRecord* {
          if (!m_mineSession) return nullptr;
          const auto& recs = m_mineSession->records();
          if (recs.empty()) return nullptr;
          const MineRecord* best = &recs[0];
          for (const MineRecord& r : recs) {
            if (r.score > best->score) best = &r;
          }
          return best;
        };

        if (sub == "help" || sub == "?" || sub == "-h" || sub == "--help") {
          usage();
          return;
        }

        if (sub == "stop" || sub == "cancel" || sub == "reset") {
          m_mineSession.reset();
          m_mineAutoRun = false;
          m_mineLastAutoStepMs = 0.0;
          showToast("Mining stopped", 1.5f);
          c.print("mine: stopped");
          return;
        }

        if (sub == "status") {
          if (!m_mineSession) {
            c.print("mine: (idle)");
            return;
          }

          const MineConfig& mc = m_mineSession->config();
          c.print(TextFormat(
              "mine: %d/%d  objective=%s  size=%dx%d  days=%d  hydro=%s  auto=%s  manual=%d  stepsFrame=%d  budget=%.1fms  last=%.2fms",
              m_mineSession->index(), m_mineSession->total(), MineObjectiveName(mc.objective), mc.w, mc.h, mc.days,
              mc.hydrologyEnabled ? "on" : "off", m_mineAutoRun ? "on" : "off", std::max(0, m_mineManualStepRequests),
              m_mineAutoStepsPerFrame, m_mineAutoBudgetMs, m_mineLastAutoStepMs));

          if (const MineRecord* best = bestRecord()) {
            c.print(TextFormat("best: score=%.4f seed=%llu pop=%d hap=%.1f land=%.1f commute=%.2fmin flood=%.2f%% pond=%.2f%%",
                               best->score, static_cast<unsigned long long>(best->seed), best->stats.population,
                               best->stats.happiness, best->stats.avgLandValue, best->stats.avgCommuteTime,
                               best->seaFloodFrac * 100.0, best->pondFrac * 100.0));
          }
          return;
        }

        if (sub == "auto") {
          if (args.size() == 1) {
            c.print(TextFormat("mine auto: %s  stepsFrame=%d  budget=%.1fms  last=%.2fms",
                               m_mineAutoRun ? "on" : "off", m_mineAutoStepsPerFrame, m_mineAutoBudgetMs,
                               m_mineLastAutoStepMs));
            return;
          }

          bool toggled = false;
          // First token may be on/off/toggle.
          {
            bool b = m_mineAutoRun;
            if (parseBool(args[1], b)) {
              m_mineAutoRun = b;
              toggled = true;
            }
          }

          for (std::size_t i = toggled ? 2 : 1; i < args.size(); ++i) {
            const std::string a = args[i];
            const std::size_t eq = a.find('=');
            if (eq == std::string::npos) {
              c.print("Expected key=value (steps=N budget=MS)");
              return;
            }
            const std::string k = toLower(a.substr(0, eq));
            const std::string v = a.substr(eq + 1);
            if (k == "steps" || k == "stepsframe") {
              int n = 0;
              if (!parseI32(v, n) || n < 1 || n > 64) {
                c.print("Bad steps. Expected 1..64");
                return;
              }
              m_mineAutoStepsPerFrame = n;
              continue;
            }
            if (k == "budget" || k == "budget_ms") {
              float ms = 0.0f;
              if (!parseF32(v, ms) || ms < 0.0f || ms > 1000.0f) {
                c.print("Bad budget. Expected 0..1000");
                return;
              }
              m_mineAutoBudgetMs = ms;
              continue;
            }
            c.print("Unknown key: " + k);
            return;
          }

          c.print(TextFormat("mine auto: %s  stepsFrame=%d  budget=%.1fms",
                             m_mineAutoRun ? "on" : "off", m_mineAutoStepsPerFrame, m_mineAutoBudgetMs));
          return;
        }

        if (sub == "step") {
          if (!m_mineSession) {
            c.print("mine: no active session. Use 'mine begin'.");
            return;
          }
          int n = 1;
          if (args.size() >= 2) {
            if (!parseI32(args[1], n) || n < 1) {
              c.print("Usage: mine step [N]");
              return;
            }
          }

          // Queue manual step requests so large values don't stall the current frame.
          const int maxQueue = 1000000;
          const int before = std::max(0, m_mineManualStepRequests);
          const int after = std::clamp(before + n, 0, maxQueue);
          m_mineManualStepRequests = after;

          c.print(TextFormat("mine: queued %d steps (pending=%d)  (%d/%d)", n, m_mineManualStepRequests,
                             m_mineSession->index(), m_mineSession->total()));
          showToast(TextFormat("Queued %d mine steps (pending=%d)", n, m_mineManualStepRequests), 1.5f);
          return;
        }

        if (sub == "top") {
          if (!m_mineSession) {
            c.print("mine: no active session.");
            return;
          }
          int k = m_mineTopK;
          bool diverse = m_mineTopDiverse;
          bool pareto = m_mineTopPareto;
          bool crowding = m_mineTopParetoCrowding;

          for (std::size_t i = 1; i < args.size(); ++i) {
            if (i == 1) {
              int v = 0;
              if (parseI32(args[i], v)) {
                k = v;
                continue;
              }
            }
            const std::size_t eq = args[i].find('=');
            if (eq == std::string::npos) {
              c.print("Usage: mine top [K] [diverse=0|1] [pareto=0|1] [crowding=0|1]");
              return;
            }
            const std::string key = toLower(args[i].substr(0, eq));
            const std::string val = args[i].substr(eq + 1);
            if (key == "diverse") {
              bool b = diverse;
              if (!parseBool(val, b)) {
                c.print("Bad diverse. Use diverse=0|1");
                return;
              }
              diverse = b;
              continue;
            }
            if (key == "pareto") {
              bool b = pareto;
              if (!parseBool(val, b)) {
                c.print("Bad pareto. Use pareto=0|1");
                return;
              }
              pareto = b;
              continue;
            }
            if (key == "crowding" || key == "pareto_crowding") {
              bool b = crowding;
              if (!parseBool(val, b)) {
                c.print("Bad crowding. Use crowding=0|1");
                return;
              }
              crowding = b;
              continue;
            }
            c.print("Unknown key: " + key);
            return;
          }

          k = std::clamp(k, 1, 1000);
          m_mineTopK = k;
          m_mineTopDiverse = diverse;
          m_mineTopPareto = pareto;
          m_mineTopParetoCrowding = crowding;

          auto& recs = m_mineSession->records();

          std::vector<int> idxs;
          if (pareto) {
            std::vector<ParetoObjective> obj;
            obj.push_back({MineMetric::Population, true});
            obj.push_back({MineMetric::Happiness, true});
            obj.push_back({MineMetric::AvgLandValue, true});
            obj.push_back({MineMetric::TrafficCongestion, false});
            if (m_mineSession->config().hydrologyEnabled) obj.push_back({MineMetric::FloodRisk, false});

            const ParetoResult pr = ComputePareto(recs, obj);
            for (std::size_t ii = 0; ii < recs.size(); ++ii) {
              recs[ii].paretoRank = pr.rank[ii];
              recs[ii].paretoCrowding = pr.crowding[ii];
            }

            idxs = SelectTopParetoIndices(pr, k, crowding);
          } else {
            idxs = SelectTopIndices(recs, k, diverse);
          }

          c.print(TextFormat("Top %d seeds (%s):", static_cast<int>(idxs.size()), pareto ? "pareto" : (diverse ? "diverse" : "best")));
          for (std::size_t i = 0; i < idxs.size(); ++i) {
            const MineRecord& r = recs[static_cast<std::size_t>(idxs[i])];
            if (pareto) {
              c.print(TextFormat(
                  "  %2d) pr=%d cd=%.3f score=%.4f seed=%llu pop=%d hap=%.1f land=%.1f commute=%.2fmin flood=%.2f%% pond=%.2f%%",
                  static_cast<int>(i + 1), r.paretoRank, r.paretoCrowding, r.score,
                  static_cast<unsigned long long>(r.seed), r.stats.population, r.stats.happiness, r.stats.avgLandValue,
                  r.stats.avgCommuteTime, r.seaFloodFrac * 100.0, r.pondFrac * 100.0));
            } else {
              c.print(TextFormat(
                  "  %2d) score=%.4f seed=%llu pop=%d hap=%.1f land=%.1f commute=%.2fmin flood=%.2f%% pond=%.2f%%",
                  static_cast<int>(i + 1), r.score, static_cast<unsigned long long>(r.seed), r.stats.population,
                  r.stats.happiness, r.stats.avgLandValue, r.stats.avgCommuteTime, r.seaFloodFrac * 100.0,
                  r.pondFrac * 100.0));
            }
          }
          return;
        }

        if (sub == "load") {
          if (!m_mineSession) {
            c.print("mine: no active session.");
            return;
          }
          if (args.size() < 2) {
            c.print("Usage: mine load <rank>");
            return;
          }
          int rank = 0;
          if (!parseI32(args[1], rank) || rank < 1) {
            c.print("Bad rank. Rank starts at 1.");
            return;
          }

          auto& recs = m_mineSession->records();
          if (recs.empty()) {
            c.print("mine: no records yet.");
            return;
          }

          const int k = std::max(1, m_mineTopK);
          std::vector<int> idxs;
          if (m_mineTopPareto) {
            std::vector<ParetoObjective> obj;
            obj.push_back({MineMetric::Population, true});
            obj.push_back({MineMetric::Happiness, true});
            obj.push_back({MineMetric::AvgLandValue, true});
            obj.push_back({MineMetric::TrafficCongestion, false});
            if (m_mineSession->config().hydrologyEnabled) obj.push_back({MineMetric::FloodRisk, false});

            const ParetoResult pr = ComputePareto(recs, obj);
            for (std::size_t ii = 0; ii < recs.size(); ++ii) {
              recs[ii].paretoRank = pr.rank[ii];
              recs[ii].paretoCrowding = pr.crowding[ii];
            }

            idxs = SelectTopParetoIndices(pr, k, m_mineTopParetoCrowding);
          } else {
            idxs = SelectTopIndices(recs, k, m_mineTopDiverse);
          }
          if (rank > static_cast<int>(idxs.size())) {
            c.print(TextFormat("Rank out of range. Have %d records in top list.", static_cast<int>(idxs.size())));
            return;
          }

          const MineRecord& r = recs[static_cast<std::size_t>(idxs[static_cast<std::size_t>(rank - 1)])];
          // Only allow load when dimensions match the current game map.
          if (r.w != m_world.width() || r.h != m_world.height()) {
            c.print(TextFormat("mine: cannot load (record size %dx%d != current %dx%d)", r.w, r.h, m_world.width(), m_world.height()));
            return;
          }

          const std::uint64_t seed = r.seed;
          c.print(TextFormat("mine: loading rank %d seed=%llu (score=%.4f)", rank, static_cast<unsigned long long>(seed), r.score));
          showToast(TextFormat("Loading mined seed %llu", static_cast<unsigned long long>(seed)), 2.0f);
          // Defer world mutation to the start of the next frame (safer for callbacks/UI).
          endPaintStroke();
          m_pendingMineLoadSeed = true;
          m_pendingMineLoadSeedValue = seed;
          return;
        }

        if (sub == "export") {
          if (!m_mineSession) {
            c.print("mine: no active session.");
            return;
          }
          auto& recs = m_mineSession->records();
          if (recs.empty()) {
            c.print("mine: no records yet.");
            return;
          }

          // If Pareto mode is enabled for the top list, annotate records so exports
          // carry paretoRank/paretoCrowding alongside the usual scalar score.
          if (m_mineTopPareto) {
            std::vector<ParetoObjective> obj;
            obj.push_back({MineMetric::Population, true});
            obj.push_back({MineMetric::Happiness, true});
            obj.push_back({MineMetric::AvgLandValue, true});
            obj.push_back({MineMetric::TrafficCongestion, false});
            if (m_mineSession->config().hydrologyEnabled) obj.push_back({MineMetric::FloodRisk, false});

            const ParetoResult pr = ComputePareto(recs, obj);
            for (std::size_t ii = 0; ii < recs.size(); ++ii) {
              recs[ii].paretoRank = pr.rank[ii];
              recs[ii].paretoCrowding = pr.crowding[ii];
            }
          }

          std::string pathOut;
          if (args.size() >= 2) {
            pathOut = joinArgs(args, 1);
          } else {
            pathOut = TextFormat("captures/mine_seed%llu_%s.csv", static_cast<unsigned long long>(m_cfg.seed), FileTimestamp().c_str());
          }

          namespace fs = std::filesystem;
          {
            std::error_code ec;
            const fs::path parent = fs::path(pathOut).parent_path();
            if (!parent.empty()) fs::create_directories(parent, ec);
          }

          const fs::path p(pathOut);
          const std::string ext = toLower(p.extension().string());

          if (ext == ".json") {
            JsonValue root = JsonValue::MakeObject();
            root.objectValue.push_back({"objective", JsonValue::MakeString(MineObjectiveName(m_mineSession->config().objective))});
            root.objectValue.push_back({"total", JsonValue::MakeNumber(m_mineSession->total())});
            JsonValue arr = JsonValue::MakeArray();
            arr.arrayValue.reserve(recs.size());
            for (const MineRecord& r : recs) {
              arr.arrayValue.push_back(MineRecordToJson(r));
            }
            root.objectValue.push_back({"records", std::move(arr)});

            std::string err;
            const bool ok = WriteJsonFile(pathOut, root, err, JsonWriteOptions{true, 2, true});
            c.print(ok ? (std::string("mine: wrote ") + pathOut)
                       : (std::string("mine: export failed: ") + (err.empty() ? pathOut : err)));
            showToast(ok ? (std::string("Exported: ") + pathOut)
                         : (std::string("Export failed: ") + (err.empty() ? pathOut : err)),
                     3.0f);
            return;
          }

          // Default: CSV.
          {
            std::ofstream f(pathOut);
            if (!f) {
              c.print("mine: could not open: " + pathOut);
              return;
            }
            WriteMineCsvHeader(f);
            for (const MineRecord& r : recs) {
              WriteMineCsvRow(f, r);
            }
          }

          c.print("mine: wrote " + pathOut);
          showToast("Exported: " + pathOut, 3.0f);
          return;
        }

        if (sub == "begin" || sub == "start") {
          if (args.size() < 2) {
            usage();
            return;
          }

          MineConfig mc;
          mc.w = m_world.width();
          mc.h = m_world.height();
          mc.objective = m_mineObjective;
          mc.samples = 0;

          // Defaults suitable for interactive exploration.
          mc.days = 120;
          mc.seedStart = 1;
          mc.seedStep = 1;
          mc.hydrologyEnabled = true;

          // First positional is samples; second positional may be objective.
          int samples = 0;
          if (!parseI32(args[1], samples) || samples < 1) {
            c.print("mine begin: bad samples");
            return;
          }
          mc.samples = samples;

          std::size_t i = 2;
          if (i < args.size()) {
            MineObjective obj;
            if (ParseMineObjective(args[i], obj)) {
              mc.objective = obj;
              m_mineObjective = obj;
              ++i;
            }
          }

          for (; i < args.size(); ++i) {
            const std::string a = args[i];
            const std::size_t eq = a.find('=');
            if (eq == std::string::npos) {
              c.print("mine begin: expected key=value");
              return;
            }
            const std::string k = toLower(a.substr(0, eq));
            const std::string v = a.substr(eq + 1);

            if (k == "days") {
              int d = 0;
              if (!parseI32(v, d) || d < 1 || d > 100000) {
                c.print("Bad days");
                return;
              }
              mc.days = d;
              continue;
            }
            if (k == "seed_start" || k == "seed" || k == "start") {
              std::uint64_t s0 = 0;
              if (!parseU64(v, s0)) {
                c.print("Bad seed_start");
                return;
              }
              mc.seedStart = s0;
              continue;
            }
            if (k == "seed_step" || k == "step") {
              std::uint64_t st = 0;
              if (!parseU64(v, st) || st == 0) {
                c.print("Bad seed_step");
                return;
              }
              mc.seedStep = st;
              continue;
            }
            if (k == "hydro" || k == "hydrology") {
              bool b = mc.hydrologyEnabled;
              if (!parseBool(v, b)) {
                c.print("Bad hydro. Use hydro=0|1");
                return;
              }
              mc.hydrologyEnabled = b;
              continue;
            }
            if (k == "sea" || k == "sealevel" || k == "sea_level") {
              float s = 0.0f;
              if (!parseF32(v, s)) {
                c.print("Bad sea. Use sea=0..1");
                return;
              }
              mc.seaLevelOverride = s;
              continue;
            }
            if (k == "topk") {
              int t = 0;
              if (!parseI32(v, t) || t < 1 || t > 1000) {
                c.print("Bad topK");
                return;
              }
              m_mineTopK = t;
              continue;
            }
            if (k == "diverse") {
              bool b = m_mineTopDiverse;
              if (!parseBool(v, b)) {
                c.print("Bad diverse. Use diverse=0|1");
                return;
              }
              m_mineTopDiverse = b;
              continue;
            }
            if (k == "pareto") {
              bool b = m_mineTopPareto;
              if (!parseBool(v, b)) {
                c.print("Bad pareto. Use pareto=0|1");
                return;
              }
              m_mineTopPareto = b;
              continue;
            }
            if (k == "crowding" || k == "pareto_crowding") {
              bool b = m_mineTopParetoCrowding;
              if (!parseBool(v, b)) {
                c.print("Bad crowding. Use crowding=0|1");
                return;
              }
              m_mineTopParetoCrowding = b;
              continue;
            }
            c.print("Unknown key: " + k);
            return;
          }

          // Create a new session.
          m_mineSession = std::make_unique<MineSession>(mc, m_procCfg, m_sim.config());
          m_mineAutoRun = true;
          m_mineLastAutoStepMs = 0.0;

          c.print(TextFormat(
              "mine: started  objective=%s  samples=%d  days=%d  seedStart=%llu  seedStep=%llu  size=%dx%d  hydro=%s",
              MineObjectiveName(mc.objective), mc.samples, mc.days, static_cast<unsigned long long>(mc.seedStart),
              static_cast<unsigned long long>(mc.seedStep), mc.w, mc.h, mc.hydrologyEnabled ? "on" : "off"));
          showToast("Seed mining started (auto)", 2.0f);
          return;
        }

        c.print("mine: unknown subcommand: " + sub);
        usage();
      });

  m_console.registerCommand(
      "tiles_csv",
      "tiles_csv [path] - export per-tile world data to CSV (x,y,terrain,overlay,level,district,height,variation,occupants)",
      [this, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;

        const std::string path = args.empty()
                                    ? TextFormat("captures/tiles_seed%llu_%s.csv",
                                                 static_cast<unsigned long long>(m_cfg.seed),
                                                 FileTimestamp().c_str())
                                    : joinArgs(args, 0);

        if (path.empty()) {
          c.print("Usage: tiles_csv [path]");
          return;
        }

        // Create parent directories if needed.
        std::error_code ec;
        const fs::path p(path);
        if (!p.parent_path().empty()) {
          fs::create_directories(p.parent_path(), ec);
        }

        std::string err;
        if (!WriteTilesCsv(m_world, path, err)) {
          c.print("Failed to write tiles CSV: " + path + (err.empty() ? "" : (" (" + err + ")")));
          showToast("Tiles CSV export failed", 2.5f);
          return;
        }

        showToast(TextFormat("Exported tiles CSV: %s", path.c_str()), 2.0f);
        c.print("wrote: " + path);
      });

  m_console.registerCommand(
      "report_csv", "report_csv [path] - export city history samples to CSV",
      [this, joinArgs](DevConsole& c, const DevConsole::Args& args) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        const std::string path = args.empty()
                                    ? TextFormat("captures/report_seed%llu_%s.csv",
                                                 static_cast<unsigned long long>(m_cfg.seed),
                                                 FileTimestamp().c_str())
                                    : joinArgs(args, 0);

        std::ofstream out(path, std::ios::out | std::ios::trunc);
        if (!out) {
          c.print("Failed to write: " + path);
          return;
        }

        out << "day,population,money,happiness,demandResidential,demandCommercial,demandIndustrial,avgLandValue,avgTaxPerCapita,income,expenses,taxRevenue,maintenanceCost,commuters,avgCommute,avgCommuteTime,trafficCongestion,goodsSatisfaction\n";
        for (const CityHistorySample& s : m_cityHistory) {
          out << s.day << ',' << s.population << ',' << s.money << ',' << s.happiness << ','
              << s.demandResidential << ',' << s.demandCommercial << ',' << s.demandIndustrial << ','
              << s.avgLandValue << ',' << s.avgTaxPerCapita << ',' << s.income << ','
              << s.expenses << ',' << s.taxRevenue << ',' << s.maintenanceCost << ',' << s.commuters << ','
              << s.avgCommute << ',' << s.avgCommuteTime << ',' << s.trafficCongestion << ','
              << s.goodsSatisfaction << '\n';
        }
        out.close();

        showToast(TextFormat("Exported report CSV: %s", path.c_str()), 2.0f);
        c.print("wrote: " + path);
      });

  // --- camera ---
  m_console.registerCommand(
      "goto", "goto <x> <y>   - center camera on tile coordinates",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 2) {
          c.print("Usage: goto <x> <y>");
          return;
        }
        long long x = 0;
        long long y = 0;
        if (!parseI64(args[0], x) || !parseI64(args[1], y)) {
          c.print("Invalid coordinates");
          return;
        }
        const int tx = std::clamp(static_cast<int>(x), 0, m_cfg.mapWidth - 1);
        const int ty = std::clamp(static_cast<int>(y), 0, m_cfg.mapHeight - 1);
        m_camera.target = TileToWorldCenterElevated(m_world, tx, ty, static_cast<float>(m_cfg.tileWidth),
                                                    static_cast<float>(m_cfg.tileHeight), m_elev);
        showToast(TextFormat("Camera -> (%d,%d)", tx, ty), 1.5f);
        c.print(TextFormat("camera centered on (%d,%d)", tx, ty));
      });

  m_console.registerCommand(
      "zoom", "zoom <0.25..4.0> - set camera zoom",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: zoom <value>");
          return;
        }
        float z = 1.0f;
        if (!parseF32(args[0], z)) {
          c.print("Invalid zoom: " + args[0]);
          return;
        }
        m_camera.zoom = std::clamp(z, 0.25f, 4.0f);
        showToast(TextFormat("Zoom: %.2f", static_cast<double>(m_camera.zoom)), 1.5f);
        c.print(TextFormat("zoom = %.2f", static_cast<double>(m_camera.zoom)));
      });

  // --- video/ui ---
  m_console.registerCommand(
      "ui_scale", "ui_scale [auto|value] - set UI scale (0.5..4.0)",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print(TextFormat("ui_scale = %.2f (%s)", static_cast<double>(m_uiScale), m_uiScaleAuto ? "auto" : "manual"));
          return;
        }

        if (args.size() != 1) {
          c.print("Usage: ui_scale [auto|value]");
          return;
        }

        if (args[0] == "auto") {
          m_uiScaleAuto = true;
          m_uiScale = computeAutoUiScale(GetScreenWidth(), GetScreenHeight());
          showToast(TextFormat("UI scale: auto (%.2f)", static_cast<double>(m_uiScale)), 1.5f);
          c.print("ui_scale -> auto");
          return;
        }

        float s = 1.0f;
        if (!parseF32(args[0], s)) {
          c.print("Invalid scale: " + args[0]);
          return;
        }

        m_uiScaleAuto = false;
        m_uiScale = std::clamp(s, 0.5f, 4.0f);
        m_uiScaleManual = m_uiScale;
        showToast(TextFormat("UI scale: %.2f", static_cast<double>(m_uiScale)), 1.5f);
        c.print(TextFormat("ui_scale -> %.2f", static_cast<double>(m_uiScale)));
      });

  m_console.registerCommand(
      "fullscreen", "fullscreen - toggle exclusive fullscreen (F11)",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          c.print("Usage: fullscreen");
          return;
        }
        toggleFullscreen();
        c.print("toggled fullscreen");
      });

  m_console.registerCommand(
      "borderless", "borderless - toggle borderless windowed fullscreen (Alt+Enter)",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          c.print("Usage: borderless");
          return;
        }
        toggleBorderlessWindowed();
        c.print("toggled borderless windowed");
      });

  m_console.registerCommand(
      "resolution", "resolution [w h] - print or set window resolution",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print(TextFormat("window %dx%d", GetScreenWidth(), GetScreenHeight()));
          return;
        }

        if (args.size() != 2) {
          c.print("Usage: resolution <w> <h>");
          return;
        }

        if (IsWindowFullscreen()) {
          c.print("Exit fullscreen first (F11)");
          return;
        }

        long long w = 0;
        long long h = 0;
        if (!parseI64(args[0], w) || !parseI64(args[1], h)) {
          c.print("Invalid size");
          return;
        }

        const int minW = std::max(320, m_cfg.windowMinWidth);
        const int minH = std::max(240, m_cfg.windowMinHeight);
        const int ww = std::max(minW, static_cast<int>(w));
        const int hh = std::max(minH, static_cast<int>(h));
        SetWindowSize(ww, hh);
        showToast(TextFormat("Window: %dx%d", ww, hh), 1.5f);
        c.print(TextFormat("window -> %dx%d", ww, hh));
      });

  m_console.registerCommand(
      "vsync", "vsync - toggle VSync hint",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (!args.empty()) {
          c.print("Usage: vsync");
          return;
        }
        toggleVsync();
        c.print(TextFormat("vsync -> %s", m_cfg.vsync ? "on" : "off"));
      });

  m_console.registerCommand(
      "prefs",
      "prefs [status|dump|save|load|autosave] - visual/display preferences\n"
      "  prefs status                     - show current prefs path + autosave state\n"
      "  prefs dump                       - print current prefs JSON\n"
      "  prefs save [path]                - write prefs to file (default: isocity_visual.json)\n"
      "  prefs load [path]                - load prefs from file and apply\n"
      "  prefs autosave on|off|toggle     - enable/disable autosave\n",
      [this, joinArgs, toLower](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty() || toLower(args[0]) == "status") {
          c.print(TextFormat("prefs path = %s", m_visualPrefsPath.c_str()));
          c.print(TextFormat("autosave = %s", m_visualPrefsAutosave ? "on" : "off"));
          c.print(TextFormat("dirty = %s", m_visualPrefsDirty ? "yes" : "no"));
          return;
        }

        const std::string sub = toLower(args[0]);
        if (sub == "dump") {
          c.print(VisualPrefsToJson(captureVisualPrefs()));
          return;
        }

        if (sub == "save") {
          const std::string path = (args.size() >= 2) ? joinArgs(args, 1) : m_visualPrefsPath;
          if (!saveVisualPrefsFile(path, true)) {
            c.print("save failed");
          } else {
            c.print("saved: " + path);
          }
          return;
        }

        if (sub == "load") {
          const std::string path = (args.size() >= 2) ? joinArgs(args, 1) : m_visualPrefsPath;
          if (!std::filesystem::exists(path)) {
            c.print("file not found: " + path);
            return;
          }
          if (!loadVisualPrefsFile(path, true)) {
            c.print("load failed");
          } else {
            c.print("loaded: " + path);
          }
          return;
        }

        if (sub == "autosave") {
          if (args.size() == 1) {
            m_visualPrefsAutosave = !m_visualPrefsAutosave;
          } else {
            const std::string mode = toLower(args[1]);
            if (mode == "on") m_visualPrefsAutosave = true;
            else if (mode == "off") m_visualPrefsAutosave = false;
            else if (mode == "toggle") m_visualPrefsAutosave = !m_visualPrefsAutosave;
            else {
              c.print("Usage: prefs autosave on|off|toggle");
              return;
            }
          }
          c.print(TextFormat("autosave -> %s", m_visualPrefsAutosave ? "on" : "off"));
          return;
        }

        c.print("Unknown subcommand. Try: prefs status");
      });

  m_console.registerCommand(
      "render_scale", "render_scale [auto|value] - set world render resolution scale",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.empty()) {
          c.print(TextFormat("render_scale = %.0f%% (%s)", m_worldRenderScale * 100.0f,
                             m_worldRenderScaleAuto ? "auto" : "manual"));
          if (m_worldRenderScaleAuto) {
            c.print(TextFormat("range: %.0f%%..%.0f%%  target: %dfps", m_worldRenderScaleMin * 100.0f,
                               m_worldRenderScaleMax * 100.0f, m_worldRenderTargetFps));
          }
          c.print(TextFormat("filter: %s", m_worldRenderFilterPoint ? "point" : "bilinear"));
          return;
        }

        if (args.size() != 1) {
          c.print("Usage: render_scale [auto|value]");
          return;
        }

        if (args[0] == "auto") {
          m_worldRenderScaleAuto = true;
          m_cfg.worldRenderScaleAuto = true;
          if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
            std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
          }
          m_worldRenderScale = std::clamp(m_worldRenderScaleMax, m_worldRenderScaleMin, m_worldRenderScaleMax);
          m_cfg.worldRenderScale = m_worldRenderScale;
          showToast(TextFormat("World render: auto (%.0f%%)", m_worldRenderScale * 100.0f), 1.5f);
          c.print("render_scale -> auto");
          return;
        }

        float s = 1.0f;
        if (!parseF32(args[0], s)) {
          c.print("Invalid scale: " + args[0]);
          return;
        }

        m_worldRenderScaleAuto = false;
        m_cfg.worldRenderScaleAuto = false;
        m_worldRenderScale = clampWorldRenderScale(s);
        m_cfg.worldRenderScale = m_worldRenderScale;
        showToast(TextFormat("World render scale: %.0f%%", m_worldRenderScale * 100.0f), 1.5f);
        c.print(TextFormat("render_scale -> %.0f%%", m_worldRenderScale * 100.0f));

        if (!wantsWorldRenderTarget()) {
          unloadWorldRenderTarget();
        }
      });

  m_console.registerCommand(
      "render_range", "render_range <min> <max> - set auto render-scale range",
      [this, parseF32](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 2) {
          c.print("Usage: render_range <min> <max>");
          return;
        }

        float mn = 0.7f;
        float mx = 1.0f;
        if (!parseF32(args[0], mn) || !parseF32(args[1], mx)) {
          c.print("Invalid range");
          return;
        }

        mn = clampWorldRenderScale(mn);
        mx = clampWorldRenderScale(mx);
        if (mn > mx) std::swap(mn, mx);

        m_worldRenderScaleMin = mn;
        m_worldRenderScaleMax = mx;
        m_cfg.worldRenderScaleMin = mn;
        m_cfg.worldRenderScaleMax = mx;

        if (m_worldRenderScaleAuto) {
          m_worldRenderScale = std::clamp(m_worldRenderScale, m_worldRenderScaleMin, m_worldRenderScaleMax);
          m_cfg.worldRenderScale = m_worldRenderScale;
        }

        showToast(TextFormat("Render range: %.0f%%..%.0f%%", mn * 100.0f, mx * 100.0f), 1.5f);
        c.print(TextFormat("render_range -> %.0f%%..%.0f%%", mn * 100.0f, mx * 100.0f));
      });

  m_console.registerCommand(
      "render_targetfps", "render_targetfps <fps> - set auto render-scale target fps",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: render_targetfps <fps>");
          return;
        }

        long long fps = 60;
        if (!parseI64(args[0], fps)) {
          c.print("Invalid fps");
          return;
        }

        m_worldRenderTargetFps = std::clamp(static_cast<int>(fps), 15, 240);
        m_cfg.worldRenderTargetFps = m_worldRenderTargetFps;
        showToast(TextFormat("Render target: %dfps", m_worldRenderTargetFps), 1.5f);
        c.print(TextFormat("render_targetfps -> %d", m_worldRenderTargetFps));
      });


  // Road resilience overlay + bypass planner.
  // Toggle overlay: Shift+T (in-game) or `res on/off`.
  m_console.registerCommand(
      "res",
      "res ... - road resilience overlay + bypass planner.\n"
      "Usage:\n"
      "  res status\n"
      "  res on|off|toggle\n"
      "  res suggest [topN] [money|tiles] [targetLevel] [allowBridges 0|1] [maxCost]\n"
      "  res list\n"
      "  res clear\n"
      "  res apply <i>",
      [this, parseI64](DevConsole& c, const DevConsole::Args& args) {
        auto lower = [](std::string s) {
          for (char& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
          return s;
        };

        auto printStatus = [&]() {
          ensureRoadResilienceUpToDate();
          c.print(TextFormat("res: overlay=%s  bridges=%d  articulations=%d  bypasses=%d  top=%d  obj=%s  lvl=%d  allowBridges=%d",
                             m_showResilienceOverlay ? "on" : "off",
                             static_cast<int>(m_roadResilience.bridgeEdges.size()),
                             static_cast<int>(m_roadResilience.articulationNodes.size()),
                             static_cast<int>(m_resilienceBypasses.size()),
                             m_resilienceBypassTop,
                             m_resilienceBypassMoney ? "money" : "tiles",
                             m_resilienceBypassTargetLevel,
                             m_resilienceBypassAllowBridges ? 1 : 0));
        };

        if (args.empty() || args[0] == "status" || args[0] == "help") {
          printStatus();
          if (args.empty() || args[0] == "help") {
            c.print("Try: res on | res suggest 5 money 2 0 2500 | res list | res apply 0");
          }
          return;
        }

        const std::string sub = lower(args[0]);

        if (sub == "on" || sub == "off" || sub == "toggle") {
          const bool want = (sub == "toggle") ? !m_showResilienceOverlay : (sub == "on");
          m_showResilienceOverlay = want;
          if (m_showResilienceOverlay) {
            ensureRoadResilienceUpToDate();
            m_resilienceBypassesDirty = true;
            rebuildRoadResilienceBypasses();
          }
          showToast(m_showResilienceOverlay ? "Resilience overlay: ON" : "Resilience overlay: OFF", 1.5f);
          printStatus();
          return;
        }

        if (sub == "suggest") {
          // Defaults: keep current settings.
          if (args.size() >= 2) {
            long long topN = m_resilienceBypassTop;
            if (parseI64(args[1], topN)) m_resilienceBypassTop = std::clamp(static_cast<int>(topN), 0, 64);
          }
          if (args.size() >= 3) {
            const std::string obj = lower(args[2]);
            if (obj == "money") m_resilienceBypassMoney = true;
            if (obj == "tiles" || obj == "newtiles") m_resilienceBypassMoney = false;
          }
          if (args.size() >= 4) {
            long long lvl = m_resilienceBypassTargetLevel;
            if (parseI64(args[3], lvl)) m_resilienceBypassTargetLevel = ClampRoadLevel(static_cast<int>(lvl));
          }
          if (args.size() >= 5) {
            long long ab = 0;
            if (parseI64(args[4], ab)) m_resilienceBypassAllowBridges = (ab != 0);
          }
          if (args.size() >= 6) {
            long long mc = 0;
            if (parseI64(args[5], mc)) m_resilienceBypassMaxCost = std::max(0, static_cast<int>(mc));
          }

          m_resilienceBypassesDirty = true;
          rebuildRoadResilienceBypasses();
          showToast(TextFormat("Res bypasses: %d", static_cast<int>(m_resilienceBypasses.size())), 1.5f);
          printStatus();
          return;
        }

        if (sub == "list") {
          if (m_resilienceBypassesDirty) {
            rebuildRoadResilienceBypasses();
          }
          if (m_resilienceBypasses.empty()) {
            c.print("No bypass suggestions. Try: res suggest 5 money 2 0");
            return;
          }
          for (std::size_t i = 0; i < m_resilienceBypasses.size(); ++i) {
            const RoadResilienceBypassSuggestion& s = m_resilienceBypasses[i];
            c.print(TextFormat("[%d] bridgeEdge=%d cut=%d cost=%d (%s) money=%d new=%d steps=%d lvl=%d bridges=%d",
                               static_cast<int>(i), s.bridgeEdge, s.cutSize, s.primaryCost,
                               s.moneyObjective ? "money" : "tiles", s.moneyCost, s.newTiles, s.steps,
                               s.targetLevel, s.allowBridges ? 1 : 0));
          }
          return;
        }

        if (sub == "clear") {
          m_resilienceBypasses.clear();
          m_resilienceBypassesDirty = false;
          c.print("Cleared bypass suggestions");
          return;
        }

        if (sub == "apply") {
          if (args.size() < 2) {
            c.print("Usage: res apply <index>");
            return;
          }
          long long idx = 0;
          if (!parseI64(args[1], idx)) {
            c.print("Invalid index: " + args[1]);
            return;
          }
          applyRoadResilienceBypass(static_cast<std::size_t>(std::max(0LL, idx)));
          return;
        }

        c.print("Unknown subcommand. Try: res help");
      });


  // Blueprint copy/paste stamping (interactive: toggle with J, console: bp ...)
  m_console.registerCommand(
      "bp",
      "bp <on|off|status|clear|capture|stamp|save|load|transform> ... - blueprint tooling",
      [this, joinArgs, parseI64](DevConsole& c, const DevConsole::Args& args) {
        auto printStatus = [&]() {
          const char* mode = "off";
          if (m_blueprintMode == BlueprintMode::Capture) mode = "capture";
          if (m_blueprintMode == BlueprintMode::Stamp) mode = "stamp";
          c.print(TextFormat("bp status: mode=%s hasBlueprint=%s size=%dx%d tiles=%d rot=%d mx=%d my=%d",
                             mode, m_hasBlueprint ? "yes" : "no", m_blueprintTransformed.width,
                             m_blueprintTransformed.height,
                             static_cast<int>(m_blueprintTransformed.tiles.size()),
                             m_blueprintTransform.rotateDeg, m_blueprintTransform.mirrorX ? 1 : 0,
                             m_blueprintTransform.mirrorY ? 1 : 0));
        };

        if (args.empty() || args[0] == "help") {
          c.print("bp on/off/status/clear");
          c.print("bp capture <x0> <y0> <w> <h>");
          c.print("bp stamp <x> <y>   (x/y are anchor tile; stamp is center-anchored)");
          c.print("bp tile <x0> <y0> <w> <h> [repeat|lib]   (top-left anchored, tiles in stamp dims)");
          c.print("bp tilemap [repeat|lib]                (top-left anchored)");
          c.print("bp transform <rotDeg> <mirrorX 0|1> <mirrorY 0|1>");
          c.print("bp save <path> | bp load <path>");
          printStatus();
          return;
        }

        const std::string sub = args[0];

        if (sub == "status") {
          updateBlueprintTransformed();
          printStatus();
          return;
        }

        if (sub == "on") {
          endPaintStroke();
          m_blueprintMode = BlueprintMode::Capture;
          c.print("bp -> on (capture)");
          showToast("Blueprint: CAPTURE (drag LMB to select)");
          return;
        }
        if (sub == "off") {
          m_blueprintMode = BlueprintMode::Off;
          m_blueprintSelecting = false;
          m_blueprintSelStart.reset();
          m_blueprintTilingSelecting = false;
          m_blueprintTileSelStart.reset();
          m_blueprintTileUseLibrary = false;
          c.print("bp -> off");
          showToast("Blueprint: OFF");
          return;
        }
        if (sub == "clear") {
          clearBlueprint();
          m_blueprintMode = BlueprintMode::Capture;
          c.print("bp -> cleared (capture)");
          showToast("Blueprint cleared");
          return;
        }

        if (sub == "capture") {
          if (args.size() != 5) {
            c.print("Usage: bp capture <x0> <y0> <w> <h>");
            return;
          }
          long long x0 = 0, y0 = 0, w = 0, h = 0;
          if (!parseI64(args[1], x0) || !parseI64(args[2], y0) || !parseI64(args[3], w) ||
              !parseI64(args[4], h)) {
            c.print("Invalid ints");
            return;
          }
          Blueprint bp;
          std::string err;
          if (!CaptureBlueprintRect(m_world, static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(w),
                                   static_cast<int>(h), bp, err, m_blueprintCaptureOpt)) {
            c.print("Capture failed: " + err);
            showToast(std::string("Blueprint capture failed: ") + err, 3.0f);
            return;
          }

          m_hasBlueprint = true;
          m_blueprint = std::move(bp);
          m_blueprintTransform = BlueprintTransform{};
          m_blueprintTransformedDirty = true;
          updateBlueprintTransformed();
          m_blueprintMode = BlueprintMode::Stamp;
          c.print(TextFormat("Captured blueprint %dx%d (%d tiles)", m_blueprintTransformed.width,
                             m_blueprintTransformed.height,
                             static_cast<int>(m_blueprintTransformed.tiles.size())));
          showToast(TextFormat("Blueprint captured (%dx%d) - STAMP mode", m_blueprintTransformed.width,
                               m_blueprintTransformed.height));
          return;
        }

        if (sub == "transform") {
          if (args.size() != 4) {
            c.print("Usage: bp transform <rotDeg> <mirrorX 0|1> <mirrorY 0|1>");
            return;
          }
          long long rot = 0, mx = 0, my = 0;
          if (!parseI64(args[1], rot) || !parseI64(args[2], mx) || !parseI64(args[3], my)) {
            c.print("Invalid ints");
            return;
          }
          m_blueprintTransform.rotateDeg = static_cast<int>((rot % 360 + 360) % 360);
          // Snap to multiples of 90 to match apply-time semantics.
          m_blueprintTransform.rotateDeg = (m_blueprintTransform.rotateDeg / 90) * 90;
          m_blueprintTransform.mirrorX = (mx != 0);
          m_blueprintTransform.mirrorY = (my != 0);
          m_blueprintTransformedDirty = true;
          updateBlueprintTransformed();
          printStatus();
          return;
        }

        if (sub == "save") {
          if (args.size() < 2) {
            c.print("Usage: bp save <path>");
            return;
          }
          if (!m_hasBlueprint) {
            c.print("No blueprint to save");
            return;
          }
          const std::string path = joinArgs(args, 1);
          std::string err;
          if (!SaveBlueprintBinary(m_blueprint, path, err)) {
            c.print("Save failed: " + err);
            showToast(std::string("Blueprint save failed: ") + err, 3.0f);
            return;
          }
          c.print("Saved blueprint: " + path);
          showToast("Blueprint saved", 1.5f);
          return;
        }

        if (sub == "load") {
          if (args.size() < 2) {
            c.print("Usage: bp load <path>");
            return;
          }
          const std::string path = joinArgs(args, 1);
          Blueprint bp;
          std::string err;
          if (!LoadBlueprintBinary(bp, path, err)) {
            c.print("Load failed: " + err);
            showToast(std::string("Blueprint load failed: ") + err, 3.0f);
            return;
          }
          m_hasBlueprint = true;
          m_blueprint = std::move(bp);
          m_blueprintTransform = BlueprintTransform{};
          m_blueprintTransformedDirty = true;
          updateBlueprintTransformed();
          m_blueprintMode = BlueprintMode::Stamp;
          c.print(TextFormat("Loaded blueprint %dx%d (%d tiles)", m_blueprintTransformed.width,
                             m_blueprintTransformed.height,
                             static_cast<int>(m_blueprintTransformed.tiles.size())));
          showToast("Blueprint loaded (STAMP mode)");
          return;
        }

        if (sub == "stamp") {
          if (args.size() != 3) {
            c.print("Usage: bp stamp <x> <y>");
            return;
          }
          long long x = 0, y = 0;
          if (!parseI64(args[1], x) || !parseI64(args[2], y)) {
            c.print("Invalid ints");
            return;
          }
          if (!m_hasBlueprint) {
            c.print("No blueprint captured/loaded");
            return;
          }
          if (!stampBlueprintAt(Point{static_cast<int>(x), static_cast<int>(y)})) {
            c.print("Stamp failed (see toast for details)");
            return;
          }
          c.print("Stamped blueprint");
          return;
        }

        if (sub == "tile") {
          if (args.size() < 5) {
            c.print("Usage: bp tile <x0> <y0> <w> <h> [repeat|lib]");
            return;
          }
          long long x0 = 0, y0 = 0, w = 0, h = 0;
          if (!parseI64(args[1], x0) || !parseI64(args[2], y0) || !parseI64(args[3], w) ||
              !parseI64(args[4], h)) {
            c.print("Invalid ints");
            return;
          }
          const std::string mode = (args.size() >= 6) ? args[5] : "repeat";
          const bool useLib = (mode == "lib" || mode == "library" || mode == "proc" || mode == "procedural");
          if (!tileBlueprintRect(static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(x0 + w - 1),
                                 static_cast<int>(y0 + h - 1), useLib)) {
            c.print("Tile failed (see toast for details)");
            return;
          }
          c.print(TextFormat("Tiled rect (%lld,%lld) size=%lldx%lld mode=%s", x0, y0, w, h, useLib ? "lib" : "repeat"));
          return;
        }

        if (sub == "tilemap") {
          const std::string mode = (args.size() >= 2) ? args[1] : "repeat";
          const bool useLib = (mode == "lib" || mode == "library" || mode == "proc" || mode == "procedural");
          if (!tileBlueprintRect(0, 0, m_world.width() - 1, m_world.height() - 1, useLib)) {
            c.print("Tile failed (see toast for details)");
            return;
          }
          c.print(TextFormat("Tiled entire map mode=%s", useLib ? "lib" : "repeat"));
          return;
        }

        c.print("Unknown bp subcommand: " + sub);
      });

  m_console.registerCommand(
      "render_filter", "render_filter <bilinear|point> - set world RT scaling filter",
      [this](DevConsole& c, const DevConsole::Args& args) {
        if (args.size() != 1) {
          c.print("Usage: render_filter <bilinear|point>");
          return;
        }

        const std::string mode = args[0];
        if (mode == "point") {
          m_worldRenderFilterPoint = true;
        } else if (mode == "bilinear") {
          m_worldRenderFilterPoint = false;
        } else {
          c.print("Unknown filter: " + mode);
          return;
        }

        m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;

        if (m_worldRenderRTValid) {
          SetTextureFilter(m_worldRenderRT.texture,
                           m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
        }

        showToast(TextFormat("Render filter: %s", m_worldRenderFilterPoint ? "point" : "bilinear"), 1.5f);
        c.print(TextFormat("render_filter -> %s", m_worldRenderFilterPoint ? "point" : "bilinear"));
      });
}

void Game::showToast(const std::string& msg, float seconds)
{
  m_toast = msg;
  m_toastTimer = std::max(0.0f, seconds);
}

float Game::computeAutoUiScale(int /*screenW*/, int screenH) const
{
  // Use screen height as a good proxy for overall UI readability and merge it
  // with any OS-reported DPI scaling.
  const float base = static_cast<float>(screenH) / 1080.0f;
  const Vector2 dpi = GetWindowScaleDPI();
  const float dpiScale = std::max(dpi.x, dpi.y);

  float scale = std::max(base, dpiScale);

  // Snap to a sensible step to avoid jitter while resizing.
  const float step = 0.25f;
  scale = std::round(scale / step) * step;
  scale = std::clamp(scale, 0.75f, 3.0f);
  return scale;
}

Vector2 Game::mouseUiPosition(float uiScale) const
{
  const Vector2 mp = GetMousePosition();
  if (uiScale <= 0.0f) return mp;
  return Vector2{mp.x / uiScale, mp.y / uiScale};
}

void Game::updateUiScaleHotkeys()
{
  const int screenW = GetScreenWidth();
  const int screenH = GetScreenHeight();
  const float autoScale = computeAutoUiScale(screenW, screenH);

  // Keep scale up-to-date when in auto mode (no toast spam).
  if (m_uiScaleAuto) {
    m_uiScale = autoScale;
  }

  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  if (!ctrl) return;

  // Reserve Ctrl+Alt combinations for other display hotkeys.
  const bool alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
  if (alt) return;

  bool userChanged = false;

  // Ctrl+0 => back to auto scaling.
  if (IsKeyPressed(KEY_ZERO)) {
    m_uiScaleAuto = true;
    m_uiScale = autoScale;
    userChanged = true;
  }

  // Ctrl+=' / Ctrl+'-' => manual adjustment.
  // NOTE: raylib maps both '=' and '+' to KEY_EQUAL.
  if (IsKeyPressed(KEY_EQUAL)) {
    if (m_uiScaleAuto) {
      m_uiScale = autoScale;
      m_uiScaleAuto = false;
    }
    m_uiScale = std::clamp(m_uiScale + 0.10f, 0.50f, 4.00f);
    userChanged = true;
  }
  if (IsKeyPressed(KEY_MINUS)) {
    if (m_uiScaleAuto) {
      m_uiScale = autoScale;
      m_uiScaleAuto = false;
    }
    m_uiScale = std::clamp(m_uiScale - 0.10f, 0.50f, 4.00f);
    userChanged = true;
  }

  if (userChanged) {
    if (!m_uiScaleAuto) {
      m_uiScaleManual = m_uiScale;
    }
    char buf[128];
    if (m_uiScaleAuto) {
      std::snprintf(buf, sizeof(buf), "UI scale: auto (%.2fx)", m_uiScale);
    } else {
      std::snprintf(buf, sizeof(buf), "UI scale: %.2fx (Ctrl+0 for auto)", m_uiScale);
    }
    showToast(buf, 2.0f);
  }
}

float Game::clampWorldRenderScale(float scale) const
{
  if (!std::isfinite(scale)) {
    return 1.0f;
  }

  return std::clamp(scale, kWorldRenderScaleAbsMin, kWorldRenderScaleAbsMax);
}

void Game::setWorldRenderScale(float scale)
{
  m_worldRenderScale = clampWorldRenderScale(scale);
  m_cfg.worldRenderScale = m_worldRenderScale;

  // If manual scaling ends up effectively "1x", drop the render target to save VRAM.
  if (!wantsWorldRenderTarget()) {
    unloadWorldRenderTarget();
  }
}

void Game::setWorldRenderScaleMin(float scaleMin)
{
  m_worldRenderScaleMin = clampWorldRenderScale(scaleMin);
  m_cfg.worldRenderScaleMin = m_worldRenderScaleMin;

  // Keep the active scale inside the new bounds (useful when auto-scaling is enabled).
  m_worldRenderScale = clampWorldRenderScale(m_worldRenderScale);
  m_cfg.worldRenderScale = m_worldRenderScale;
}

void Game::setWorldRenderScaleMax(float scaleMax)
{
  m_worldRenderScaleMax = clampWorldRenderScale(scaleMax);
  m_cfg.worldRenderScaleMax = m_worldRenderScaleMax;

  m_worldRenderScale = clampWorldRenderScale(m_worldRenderScale);
  m_cfg.worldRenderScale = m_worldRenderScale;
}

void Game::updateWorldRenderFilter()
{
  if (!m_worldRenderRTValid) return;
  SetTextureFilter(
    m_worldRenderRT.texture,
    m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
}

bool Game::wantsWorldRenderTarget() const
{
  if (m_postFx.enabled) return true;
  if (m_worldRenderScaleAuto) return true;
  return std::fabs(m_worldRenderScale - 1.0f) > 0.001f;
}

void Game::unloadWorldRenderTarget()
{
  if (!m_worldRenderRTValid) return;
  UnloadRenderTexture(m_worldRenderRT);
  m_worldRenderRT = {};
  m_worldRenderRTValid = false;
  m_worldRenderRTWidth = 0;
  m_worldRenderRTHeight = 0;
}

void Game::ensureWorldRenderTarget(int screenW, int screenH)
{
  if (!wantsWorldRenderTarget()) {
    unloadWorldRenderTarget();
    return;
  }

  float scale = clampWorldRenderScale(m_worldRenderScale);
  if (m_worldRenderScaleAuto) {
    const float lo = clampWorldRenderScale(m_worldRenderScaleMin);
    const float hi = clampWorldRenderScale(m_worldRenderScaleMax);
    scale = std::clamp(scale, std::min(lo, hi), std::max(lo, hi));
  }

  // Prevent absurdly large render targets on extreme resolutions.
  if (screenW > 0 && screenH > 0) {
    const float maxScaleByDim = std::min(kWorldRenderRTMaxDim / static_cast<float>(screenW),
                                        kWorldRenderRTMaxDim / static_cast<float>(screenH));
    scale = std::min(scale, maxScaleByDim);
  }

  // If we had to clamp the effective scale (for example due to max RT size),
  // keep the runtime value consistent so camera mapping stays correct.
  if (std::fabs(scale - m_worldRenderScale) > 0.0005f) {
    m_worldRenderScale = scale;
    m_cfg.worldRenderScale = scale;
  }

  const int desiredW = std::max(1, static_cast<int>(std::lround(screenW * scale)));
  const int desiredH = std::max(1, static_cast<int>(std::lround(screenH * scale)));

  if (m_worldRenderRTValid && (desiredW == m_worldRenderRTWidth) && (desiredH == m_worldRenderRTHeight)) {
    // Keep filter in sync (users can toggle it at runtime).
    SetTextureFilter(m_worldRenderRT.texture,
                     m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    return;
  }

  unloadWorldRenderTarget();
  m_worldRenderRT = LoadRenderTexture(desiredW, desiredH);
  m_worldRenderRTValid = (m_worldRenderRT.texture.id != 0);
  m_worldRenderRTWidth = desiredW;
  m_worldRenderRTHeight = desiredH;

  if (m_worldRenderRTValid) {
    SetTextureFilter(m_worldRenderRT.texture,
                     m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
  }
}

void Game::updateWorldRenderHotkeys()
{
  // Ctrl+Alt combinations are reserved for world render scaling, so they don't
  // clash with Ctrl +/- UI scaling.
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool alt = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);
  if (!ctrl || !alt) return;
  if (m_console.isOpen()) return;

  auto setManualScale = [&](float newScale) {
    m_worldRenderScaleAuto = false;
    m_worldRenderScale = clampWorldRenderScale(newScale);
    m_cfg.worldRenderScaleAuto = false;
    m_cfg.worldRenderScale = m_worldRenderScale;
    if (!wantsWorldRenderTarget()) {
      unloadWorldRenderTarget();
    }
    showToast(TextFormat("World scale: %.2fx", static_cast<double>(m_worldRenderScale)));
  };

  if (IsKeyPressed(KEY_EQUAL)) {
    setManualScale(m_worldRenderScale + kWorldRenderScaleStep);
  }
  if (IsKeyPressed(KEY_MINUS)) {
    setManualScale(m_worldRenderScale - kWorldRenderScaleStep);
  }
  if (IsKeyPressed(KEY_ZERO)) {
    setManualScale(1.0f);
  }
  if (IsKeyPressed(KEY_F)) {
    m_worldRenderFilterPoint = !m_worldRenderFilterPoint;
    m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;
    if (m_worldRenderRTValid) {
      SetTextureFilter(m_worldRenderRT.texture,
                       m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
    }
    showToast(m_worldRenderFilterPoint ? "World filter: POINT" : "World filter: BILINEAR");
  }
  if (IsKeyPressed(KEY_A)) {
    m_worldRenderScaleAuto = !m_worldRenderScaleAuto;
    m_cfg.worldRenderScaleAuto = m_worldRenderScaleAuto;
    if (m_worldRenderScaleAuto) {
      m_worldRenderScaleMin = clampWorldRenderScale(m_worldRenderScaleMin);
      m_worldRenderScaleMax = clampWorldRenderScale(m_worldRenderScaleMax);
      if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
        std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
      }
      m_worldRenderScale = std::clamp(m_worldRenderScaleMax, m_worldRenderScaleMin, m_worldRenderScaleMax);
      m_cfg.worldRenderScale = m_worldRenderScale;
      showToast("World scale: AUTO");
    } else {
      showToast("World scale: MANUAL");
      if (!wantsWorldRenderTarget()) {
        unloadWorldRenderTarget();
      }
    }
  }
}

void Game::updateDynamicWorldRenderScale(float dt)
{
  // Exponential smoothing for stability.
  m_frameTimeSmoothed = m_frameTimeSmoothed * (1.0f - kWorldRenderDtSmoothing) + dt * kWorldRenderDtSmoothing;

  if (!m_worldRenderScaleAuto) return;

  m_worldRenderAutoTimer += dt;
  if (m_worldRenderAutoTimer < kWorldRenderAutoAdjustInterval) return;
  m_worldRenderAutoTimer = 0.0f;

  float lo = clampWorldRenderScale(m_worldRenderScaleMin);
  float hi = clampWorldRenderScale(m_worldRenderScaleMax);
  if (lo > hi) std::swap(lo, hi);
  m_worldRenderScaleMin = lo;
  m_worldRenderScaleMax = hi;

  const int targetFps = std::max(15, m_worldRenderTargetFps);
  const float targetDt = 1.0f / static_cast<float>(targetFps);

  // Hysteresis bands to prevent oscillation.
  const float tooSlow = targetDt * 1.08f;  // 8% slower than target
  const float tooFast = targetDt * 0.92f;  // 8% faster than target

  float scale = std::clamp(m_worldRenderScale, lo, hi);
  if (m_frameTimeSmoothed > tooSlow && scale > lo + 0.001f) {
    scale = std::max(lo, scale - kWorldRenderScaleStep);
  } else if (m_frameTimeSmoothed < tooFast && scale < hi - 0.001f) {
    scale = std::min(hi, scale + kWorldRenderScaleStep);
  }

  // Quantize to our step to avoid constant reallocations.
  scale = std::round(scale / kWorldRenderScaleStep) * kWorldRenderScaleStep;
  scale = std::clamp(scale, lo, hi);

  if (std::fabs(scale - m_worldRenderScale) > 0.0001f) {
    m_worldRenderScale = scale;
    m_cfg.worldRenderScale = m_worldRenderScale;
    // No toast here: it would spam while auto-scaling.
  }
}

void Game::adjustVideoSettings(int dir)
{
  const bool shift = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT));
  const int d = (dir < 0) ? -1 : 1;

  auto toastScale = [&](float scale) {
    showToast(TextFormat("World render scale: %.0f%%", scale * 100.0f));
  };

  auto clamp01 = [](float v) { return std::clamp(v, 0.0f, 1.0f); };
  auto wrapDeg = [](float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
  };

  // ----------------------------
  // Display page (existing)
  // ----------------------------
  if (m_videoPage == 0) {
    switch (m_videoSelection) {
      case 0:
        toggleFullscreen();
        break;
      case 1:
        toggleBorderlessWindowed();
        break;
      case 2:
        m_cfg.vsync = !m_cfg.vsync;
        SetWindowState(m_cfg.vsync ? FLAG_VSYNC_HINT : 0);
        showToast(m_cfg.vsync ? "VSync: ON" : "VSync: OFF");
        break;
      case 3:
        m_uiScaleAuto = !m_uiScaleAuto;
        if (m_uiScaleAuto) {
          // Reset to 1x immediately; we'll compute a new best-fit scale next update.
          m_uiScale = 1.0f;
        }
        showToast(m_uiScaleAuto ? "UI scale: Auto" : "UI scale: Manual");
        break;
      case 4:
        if (!m_uiScaleAuto) {
          m_uiScale = std::clamp(m_uiScale + d * 0.1f, 0.5f, 3.0f);
          showToast(TextFormat("UI scale: %.2fx", m_uiScale));
        }
        break;

      case 5:
        m_worldRenderScaleAuto = !m_worldRenderScaleAuto;
        showToast(m_worldRenderScaleAuto ? "World render scale: Auto" : "World render scale: Manual");
        break;
      case 6:
        if (m_worldRenderScaleAuto) break;
        setWorldRenderScale(std::clamp(m_worldRenderScale + d * 0.05f, 0.25f, 2.0f));
        toastScale(m_worldRenderScale);
        break;
      case 7:
        if (!m_worldRenderScaleAuto) break;
        setWorldRenderScaleMin(std::clamp(m_worldRenderScaleMin + d * 0.05f, 0.25f, 2.0f));
        showToast(TextFormat("World scale min: %.0f%%", m_worldRenderScaleMin * 100.0f));
        break;
      case 8:
        if (!m_worldRenderScaleAuto) break;
        setWorldRenderScaleMax(std::clamp(m_worldRenderScaleMax + d * 0.05f, 0.25f, 2.0f));
        showToast(TextFormat("World scale max: %.0f%%", m_worldRenderScaleMax * 100.0f));
        break;
      case 9:
        if (!m_worldRenderScaleAuto) break;
        m_worldRenderTargetFps = std::clamp(m_worldRenderTargetFps + d * 5, 15, 240);
        showToast(TextFormat("World target FPS: %d", m_worldRenderTargetFps));
        break;
      case 10:
        m_worldRenderFilterPoint = !m_worldRenderFilterPoint;
        updateWorldRenderFilter();
        showToast(m_worldRenderFilterPoint ? "World filter: Point" : "World filter: Bilinear");
        break;
      default:
        break;
    }

    // Keep runtime values mirrored in config for consistency.
    m_cfg.worldRenderScaleAuto = m_worldRenderScaleAuto;
    m_cfg.worldRenderScale = m_worldRenderScale;
    m_cfg.worldRenderScaleMin = m_worldRenderScaleMin;
    m_cfg.worldRenderScaleMax = m_worldRenderScaleMax;
    m_cfg.worldRenderTargetFps = m_worldRenderTargetFps;
    m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;

    m_videoSelectionDisplay = m_videoSelection;
    return;
  }

  // ----------------------------
  // UI Theme page (new)
  // ----------------------------
  if (m_videoPage == 3) {
    ui::Settings s = ui::GetSettings();

    auto toastPct = [&](const char* label, float v) {
      showToast(TextFormat("%s: %.0f%%", label, v * 100.0f));
    };

    switch (m_videoSelection) {
      case 0:
        s.accentFromSeed = !s.accentFromSeed;
        showToast(s.accentFromSeed ? "UI accent: SEED" : "UI accent: MANUAL");
        break;
      case 1: {
        if (s.accentFromSeed) {
          showToast("UI accent hue: (seed)");
        } else {
          const float step = shift ? 15.0f : 5.0f;
          s.accentHueDeg = wrapDeg(s.accentHueDeg + static_cast<float>(d) * step);
          showToast(TextFormat("UI accent hue: %.0f°", s.accentHueDeg));
        }
      } break;
      case 2: {
        const float step = shift ? 0.10f : 0.03f;
        s.accentSaturation = clamp01(s.accentSaturation + static_cast<float>(d) * step);
        toastPct("UI accent sat", s.accentSaturation);
      } break;
      case 3: {
        const float step = shift ? 0.10f : 0.03f;
        s.accentValue = clamp01(s.accentValue + static_cast<float>(d) * step);
        toastPct("UI accent val", s.accentValue);
      } break;
      case 4: {
        const float step = shift ? 0.06f : 0.02f;
        s.roundness = std::clamp(s.roundness + static_cast<float>(d) * step, 0.0f, 1.0f);
        toastPct("UI roundness", s.roundness);
      } break;
      case 5: {
        const float step = shift ? 0.03f : 0.01f;
        s.noiseAlpha = clamp01(s.noiseAlpha + static_cast<float>(d) * step);
        toastPct("UI grain", s.noiseAlpha);
      } break;
      case 6: {
        const float step = shift ? 0.25f : 0.05f;
        s.noiseScale = std::clamp(s.noiseScale + static_cast<float>(d) * step, 0.05f, 4.0f);
        showToast(TextFormat("UI grain scale: %.2f", s.noiseScale));
      } break;
      case 7: {
        const float step = shift ? 0.10f : 0.03f;
        s.headerSheenStrength = clamp01(s.headerSheenStrength + static_cast<float>(d) * step);
        toastPct("UI header sheen", s.headerSheenStrength);
      } break;
      case 8: {
        const int next = std::clamp(s.fontAtlasScale + d, 2, 6);
        s.fontAtlasScale = next;
        showToast(TextFormat("UI font quality: %dx", s.fontAtlasScale));
      } break;
      case 9:
        s.fontFilterPoint = !s.fontFilterPoint;
        showToast(s.fontFilterPoint ? "UI font: PIXEL" : "UI font: SMOOTH (SDF)");
        break;
      case 10:
        ui::ResetSettings();
        showToast("UI theme: RESET");
        m_videoSelectionUiTheme = m_videoSelection;
        return;
      default:
        break;
    }

    ui::SetSettings(s);
    m_videoSelectionUiTheme = m_videoSelection;
    return;
  }

  // ----------------------------
  // Atmosphere page (new)
  // ----------------------------
  if (m_videoPage == 2) {
    auto cs = m_renderer.cloudShadowSettings();
    auto vc = m_renderer.volumetricCloudSettings();

    auto toastPct = [&](const char* label, float v) {
      showToast(TextFormat("%s: %.0f%%", label, v * 100.0f));
    };

    // Row indices must match drawVideoSettingsPanel() Atmosphere page (0..18).
    switch (m_videoSelection) {
      case 0:
        cs.enabled = !cs.enabled;
        m_renderer.setCloudShadowSettings(cs);
        showToast(cs.enabled ? "Cloud shadows: ON" : "Cloud shadows: OFF");
        break;
      case 1: {
        const float step = shift ? 0.10f : 0.03f;
        cs.strength = clamp01(cs.strength + static_cast<float>(d) * step);
        m_renderer.setCloudShadowSettings(cs);
        toastPct("Shadow strength", cs.strength);
      } break;
      case 2: {
        const float step = shift ? 0.50f : 0.10f;
        cs.scale = std::clamp(cs.scale + static_cast<float>(d) * step, 0.25f, 8.0f);
        m_renderer.setCloudShadowSettings(cs);
        showToast(TextFormat("Shadow scale: %.2fx", cs.scale));
      } break;
      case 3: {
        const float step = shift ? 0.50f : 0.10f;
        cs.speed = std::clamp(cs.speed + static_cast<float>(d) * step, 0.0f, 5.0f);
        m_renderer.setCloudShadowSettings(cs);
        showToast(TextFormat("Shadow speed: %.2fx", cs.speed));
      } break;
      case 4: {
        const float step = shift ? 0.10f : 0.03f;
        cs.evolve = clamp01(cs.evolve + static_cast<float>(d) * step);
        m_renderer.setCloudShadowSettings(cs);
        toastPct("Shadow evolve", cs.evolve);
      } break;
      case 5: {
        const float step = shift ? 0.10f : 0.03f;
        cs.coverage = clamp01(cs.coverage + static_cast<float>(d) * step);
        m_renderer.setCloudShadowSettings(cs);
        toastPct("Shadow coverage", cs.coverage);
      } break;
      case 6: {
        const float step = shift ? 0.10f : 0.03f;
        cs.softness = clamp01(cs.softness + static_cast<float>(d) * step);
        m_renderer.setCloudShadowSettings(cs);
        toastPct("Shadow softness", cs.softness);
      } break;
      case 7: {
        const float step = shift ? 0.10f : 0.05f;
        cs.clearAmount = clamp01(cs.clearAmount + static_cast<float>(d) * step);
        m_renderer.setCloudShadowSettings(cs);
        toastPct("Clear sky", cs.clearAmount);
      } break;

      case 8:
        vc.enabled = !vc.enabled;
        m_renderer.setVolumetricCloudSettings(vc);
        showToast(vc.enabled ? "Volumetric clouds: ON" : "Volumetric clouds: OFF");
        break;
      case 9: {
        const float step = shift ? 0.10f : 0.03f;
        vc.opacity = clamp01(vc.opacity + static_cast<float>(d) * step);
        m_renderer.setVolumetricCloudSettings(vc);
        toastPct("Cloud opacity", vc.opacity);
      } break;
      case 10: {
        const float step = shift ? 0.10f : 0.03f;
        vc.coverage = clamp01(vc.coverage + static_cast<float>(d) * step);
        m_renderer.setVolumetricCloudSettings(vc);
        toastPct("Cloud coverage", vc.coverage);
      } break;
      case 11: {
        const float step = shift ? 0.25f : 0.05f;
        vc.density = std::clamp(vc.density + static_cast<float>(d) * step, 0.0f, 2.0f);
        m_renderer.setVolumetricCloudSettings(vc);
        showToast(TextFormat("Cloud density: %.2fx", vc.density));
      } break;
      case 12: {
        const float step = shift ? 0.50f : 0.10f;
        vc.scale = std::clamp(vc.scale + static_cast<float>(d) * step, 0.25f, 8.0f);
        m_renderer.setVolumetricCloudSettings(vc);
        showToast(TextFormat("Cloud scale: %.2fx", vc.scale));
      } break;
      case 13: {
        const float step = shift ? 0.50f : 0.10f;
        vc.speed = std::clamp(vc.speed + static_cast<float>(d) * step, 0.0f, 5.0f);
        m_renderer.setVolumetricCloudSettings(vc);
        showToast(TextFormat("Cloud speed: %.2fx", vc.speed));
      } break;
      case 14: {
        const float step = shift ? 0.10f : 0.03f;
        vc.softness = clamp01(vc.softness + static_cast<float>(d) * step);
        m_renderer.setVolumetricCloudSettings(vc);
        toastPct("Cloud softness", vc.softness);
      } break;
      case 15: {
        const int step = shift ? 4 : 2;
        vc.steps = std::clamp(vc.steps + d * step, 8, 64);
        m_renderer.setVolumetricCloudSettings(vc);
        showToast(TextFormat("Cloud steps: %d", vc.steps));
      } break;
      case 16: {
        const float step = shift ? 0.10f : 0.03f;
        vc.bottomFade = clamp01(vc.bottomFade + static_cast<float>(d) * step);
        m_renderer.setVolumetricCloudSettings(vc);
        toastPct("Bottom fade", vc.bottomFade);
      } break;
      case 17: {
        const float step = shift ? 0.10f : 0.05f;
        vc.clearAmount = clamp01(vc.clearAmount + static_cast<float>(d) * step);
        m_renderer.setVolumetricCloudSettings(vc);
        toastPct("Clear sky", vc.clearAmount);
      } break;

      case 18:
        m_renderer.setCloudShadowSettings(Renderer::CloudShadowSettings{});
        m_renderer.setVolumetricCloudSettings(Renderer::VolumetricCloudSettings{});
        showToast("Atmosphere: RESET");
        break;

      default:
        break;
    }

    m_videoSelectionAtmos = m_videoSelection;
    return;
  }

  // ----------------------------
  // Visual FX page (new)
  // ----------------------------
  auto toggleLayer = [&](Renderer::RenderLayer layer, const char* name) {
    const bool en = !m_renderer.layerEnabled(layer);
    m_renderer.setLayerEnabled(layer, en);
    showToast(TextFormat("%s layer: %s", name, en ? "ON" : "OFF"));
  };

  switch (m_videoSelection) {
    case 0: {
      const bool en = !m_renderer.baseCacheEnabled();
      m_renderer.setBaseCacheEnabled(en);
      m_renderer.markBaseCacheDirtyAll();
      showToast(en ? "Render cache: ON" : "Render cache: OFF");
    } break;

    case 1:
      toggleLayer(Renderer::RenderLayer::Terrain, "Terrain");
      break;
    case 2:
      toggleLayer(Renderer::RenderLayer::Decals, "Decals");
      break;
    case 3:
      toggleLayer(Renderer::RenderLayer::Structures, "Structures");
      break;
    case 4:
      toggleLayer(Renderer::RenderLayer::Overlays, "Overlays");
      break;

    case 5: {
      m_mergedZoneBuildings = !m_mergedZoneBuildings;
      m_cfg.mergedZoneBuildings = m_mergedZoneBuildings;
      showToast(m_mergedZoneBuildings ? "Merged zone buildings: ON" : "Merged zone buildings: OFF");
    } break;

    case 6: {
      auto sh = m_renderer.shadowSettings();
      sh.enabled = !sh.enabled;
      m_renderer.setShadowSettings(sh);
      showToast(sh.enabled ? "Shadows: ON" : "Shadows: OFF");
    } break;

    case 7: {
      auto sh = m_renderer.shadowSettings();
      const float step = shift ? 0.10f : 0.03f;
      sh.strength = clamp01(sh.strength + static_cast<float>(d) * step);
      m_renderer.setShadowSettings(sh);
      showToast(TextFormat("Shadow strength: %.0f%%", sh.strength * 100.0f));
    } break;

    case 8: {
      auto sh = m_renderer.shadowSettings();
      const float step = shift ? 0.10f : 0.03f;
      sh.softness = clamp01(sh.softness + static_cast<float>(d) * step);
      m_renderer.setShadowSettings(sh);
      showToast(TextFormat("Shadow softness: %.0f%%", sh.softness * 100.0f));
    } break;

    case 9: {
      auto sh = m_renderer.shadowSettings();
      const float step = shift ? 1.0f : 0.25f;
      sh.maxLengthTiles = std::clamp(sh.maxLengthTiles + static_cast<float>(d) * step, 0.0f, 20.0f);
      m_renderer.setShadowSettings(sh);
      showToast(TextFormat("Shadow max len: %.1f tiles", sh.maxLengthTiles));
    } break;

    case 10: {
      auto sh = m_renderer.shadowSettings();
      const float step = shift ? 15.0f : 5.0f;
      sh.azimuthDeg = wrapDeg(sh.azimuthDeg + static_cast<float>(d) * step);
      m_renderer.setShadowSettings(sh);
      showToast(TextFormat("Shadow direction: %.0f°", sh.azimuthDeg));
    } break;

    case 11: {
      auto dn = m_renderer.dayNightSettings();
      dn.enabled = !dn.enabled;
      m_renderer.setDayNightSettings(dn);
      showToast(dn.enabled ? "Day/night: ON" : "Day/night: OFF");
    } break;

    case 12: {
      auto dn = m_renderer.dayNightSettings();
      const float step = shift ? 60.0f : 15.0f;
      dn.dayLengthSec = std::clamp(dn.dayLengthSec + static_cast<float>(d) * step, 30.0f, 1800.0f);
      dn.timeOffsetSec = std::clamp(dn.timeOffsetSec, 0.0f, std::max(0.0f, dn.dayLengthSec));
      m_renderer.setDayNightSettings(dn);
      showToast(TextFormat("Day length: %.0f s", dn.dayLengthSec));
    } break;

    case 13: {
      auto dn = m_renderer.dayNightSettings();
      const float step = shift ? 60.0f : 15.0f;
      dn.timeOffsetSec += static_cast<float>(d) * step;
      // Wrap within [0, dayLengthSec).
      while (dn.timeOffsetSec < 0.0f) dn.timeOffsetSec += dn.dayLengthSec;
      while (dn.timeOffsetSec >= dn.dayLengthSec) dn.timeOffsetSec -= dn.dayLengthSec;
      m_renderer.setDayNightSettings(dn);
      showToast(TextFormat("Time offset: %.0f s", dn.timeOffsetSec));
    } break;

    case 14: {
      auto dn = m_renderer.dayNightSettings();
      const float step = shift ? 0.10f : 0.03f;
      dn.nightDarken = clamp01(dn.nightDarken + static_cast<float>(d) * step);
      m_renderer.setDayNightSettings(dn);
      showToast(TextFormat("Night darken: %.0f%%", dn.nightDarken * 100.0f));
    } break;

    case 15: {
      auto dn = m_renderer.dayNightSettings();
      const float step = shift ? 0.10f : 0.03f;
      dn.duskTint = clamp01(dn.duskTint + static_cast<float>(d) * step);
      m_renderer.setDayNightSettings(dn);
      showToast(TextFormat("Dusk tint: %.0f%%", dn.duskTint * 100.0f));
    } break;

    case 16: {
      auto dn = m_renderer.dayNightSettings();
      dn.drawLights = !dn.drawLights;
      m_renderer.setDayNightSettings(dn);
      showToast(dn.drawLights ? "Night lights: ON" : "Night lights: OFF");
    } break;

    case 17: {
      auto wx = m_renderer.weatherSettings();
      using M = Renderer::WeatherSettings::Mode;
      const int cur = static_cast<int>(wx.mode);
      const int next = (cur + (d > 0 ? 1 : 2)) % 3;
      wx.mode = static_cast<M>(next);
      m_renderer.setWeatherSettings(wx);
      const char* modeStr = (wx.mode == M::Rain) ? "Rain" : (wx.mode == M::Snow) ? "Snow" : "Clear";
      showToast(TextFormat("Weather: %s", modeStr));
    } break;

    case 18: {
      auto wx = m_renderer.weatherSettings();
      const float step = shift ? 0.10f : 0.05f;
      wx.intensity = clamp01(wx.intensity + static_cast<float>(d) * step);
      m_renderer.setWeatherSettings(wx);
      showToast(TextFormat("Weather intensity: %.0f%%", wx.intensity * 100.0f));
    } break;

    case 19: {
      auto wx = m_renderer.weatherSettings();
      const float step = shift ? 15.0f : 5.0f;
      wx.windAngleDeg = wrapDeg(wx.windAngleDeg + static_cast<float>(d) * step);
      m_renderer.setWeatherSettings(wx);
      showToast(TextFormat("Wind dir: %.0f°", wx.windAngleDeg));
    } break;

    case 20: {
      auto wx = m_renderer.weatherSettings();
      const float step = shift ? 0.25f : 0.05f;
      wx.windSpeed = std::clamp(wx.windSpeed + static_cast<float>(d) * step, 0.10f, 5.0f);
      m_renderer.setWeatherSettings(wx);
      showToast(TextFormat("Wind speed: %.2fx", wx.windSpeed));
    } break;

    case 21: {
      auto wx = m_renderer.weatherSettings();
      const float step = shift ? 0.10f : 0.05f;
      wx.overcast = clamp01(wx.overcast + static_cast<float>(d) * step);
      m_renderer.setWeatherSettings(wx);
      showToast(TextFormat("Overcast: %.0f%%", wx.overcast * 100.0f));
    } break;

    case 22: {
      auto wx = m_renderer.weatherSettings();
      const float step = shift ? 0.10f : 0.05f;
      wx.fog = clamp01(wx.fog + static_cast<float>(d) * step);
      m_renderer.setWeatherSettings(wx);
      showToast(TextFormat("Fog: %.0f%%", wx.fog * 100.0f));
    } break;

    case 23: {
      auto wx = m_renderer.weatherSettings();
      wx.drawParticles = !wx.drawParticles;
      m_renderer.setWeatherSettings(wx);
      showToast(wx.drawParticles ? "Weather particles: ON" : "Weather particles: OFF");
    } break;

    case 24: {
      auto wx = m_renderer.weatherSettings();
      wx.affectGround = !wx.affectGround;
      m_renderer.setWeatherSettings(wx);
      showToast(wx.affectGround ? "Ground effects: ON" : "Ground effects: OFF");
    } break;

    case 25: {
      auto wx = m_renderer.weatherSettings();
      wx.reflectLights = !wx.reflectLights;
      m_renderer.setWeatherSettings(wx);
      showToast(wx.reflectLights ? "Reflect lights: ON" : "Reflect lights: OFF");
    } break;

    case 26: {
      m_postFx.enabled = !m_postFx.enabled;
      if (m_postFx.enabled) {
        showToast(m_postFxPipeline.isReady() ? "Post FX: ON" : "Post FX: ON (shader fallback)");
      } else {
        showToast("Post FX: OFF");
        if (!wantsWorldRenderTarget()) unloadWorldRenderTarget();
      }
    } break;

    case 27: {
      const int step = shift ? 2 : 1;
      m_postFx.colorBits = std::clamp(m_postFx.colorBits + (d * step), 2, 8);
      showToast(TextFormat("Post FX: bits %d", m_postFx.colorBits));
    } break;

    case 28: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.ditherStrength = clamp01(m_postFx.ditherStrength + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: dither %.0f%%", m_postFx.ditherStrength * 100.0f));
    } break;

    case 29: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.grain = clamp01(m_postFx.grain + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: grain %.0f%%", m_postFx.grain * 100.0f));
    } break;

    case 30: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.vignette = clamp01(m_postFx.vignette + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: vignette %.0f%%", m_postFx.vignette * 100.0f));
    } break;

    case 31: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.chroma = clamp01(m_postFx.chroma + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: chroma %.0f%%", m_postFx.chroma * 100.0f));
    } break;

    case 32: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.scanlines = clamp01(m_postFx.scanlines + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: scanlines %.0f%%", m_postFx.scanlines * 100.0f));
    } break;

    case 33: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.fxaa = clamp01(m_postFx.fxaa + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: FXAA %.0f%%", m_postFx.fxaa * 100.0f));
    } break;

    case 34: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.sharpen = clamp01(m_postFx.sharpen + (static_cast<float>(d) * step));
      showToast(TextFormat("Post FX: sharpen %.0f%%", m_postFx.sharpen * 100.0f));
    } break;

    case 35: {
      m_postFx.taaEnabled = !m_postFx.taaEnabled;
      if (m_postFx.taaEnabled) {
        showToast(m_postFxPipeline.taaReady() ? "TAA: ON" : "TAA: ON (shader fallback)");
      } else {
        showToast("TAA: OFF");
      }
    } break;

    case 36: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.taaHistory = clamp01(m_postFx.taaHistory + (static_cast<float>(d) * step));
      showToast(TextFormat("TAA history: %.0f%%", m_postFx.taaHistory * 100.0f));
    } break;

    case 37: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.taaJitter = clamp01(m_postFx.taaJitter + (static_cast<float>(d) * step));
      showToast(TextFormat("TAA jitter: %.0f%%", m_postFx.taaJitter * 100.0f));
    } break;

    case 38: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.taaResponse = clamp01(m_postFx.taaResponse + (static_cast<float>(d) * step));
      showToast(TextFormat("TAA response: %.0f%%", m_postFx.taaResponse * 100.0f));
    } break;

    case 39:
      m_postFx.includeWeather = !m_postFx.includeWeather;
      showToast(m_postFx.includeWeather ? "Post FX: include weather" : "Post FX: weather separate");
      break;

    case 40: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.bloom = clamp01(m_postFx.bloom + (static_cast<float>(d) * step));
      showToast(TextFormat("Bloom: %.0f%%", m_postFx.bloom * 100.0f));
    } break;

    case 41: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.bloomThreshold = clamp01(m_postFx.bloomThreshold + (static_cast<float>(d) * step));
      showToast(TextFormat("Bloom threshold: %.0f%%", m_postFx.bloomThreshold * 100.0f));
    } break;

    case 42: {
      const float step = shift ? 0.50f : 0.10f;
      m_postFx.bloomRadius = std::clamp(m_postFx.bloomRadius + (static_cast<float>(d) * step), 0.25f, 4.0f);
      showToast(TextFormat("Bloom radius: %.2fx", m_postFx.bloomRadius));
    } break;

    case 43: {
      const int opts[] = {1, 2, 4, 8};
      int idx = 2; // default (4x)
      for (int i = 0; i < 4; ++i) {
        if (opts[i] == m_postFx.bloomDownsample) {
          idx = i;
          break;
        }
      }

      idx = std::clamp(idx + d, 0, 3);
      m_postFx.bloomDownsample = opts[idx];
      showToast(TextFormat("Bloom downsample: %dx", m_postFx.bloomDownsample));
    } break;

    case 44: {
      m_postFx.tonemapEnabled = !m_postFx.tonemapEnabled;
      showToast(m_postFx.tonemapEnabled ? "Tonemap: ON" : "Tonemap: OFF");
    } break;

    case 45: {
      const float step = shift ? 0.25f : 0.05f;
      m_postFx.exposure = std::clamp(m_postFx.exposure + (static_cast<float>(d) * step), 0.0f, 4.0f);
      showToast(TextFormat("Exposure: %.2fx", m_postFx.exposure));
    } break;

    case 46: {
      const float step = shift ? 0.25f : 0.05f;
      m_postFx.contrast = std::clamp(m_postFx.contrast + (static_cast<float>(d) * step), 0.0f, 2.0f);
      showToast(TextFormat("Contrast: %.2fx", m_postFx.contrast));
    } break;

    case 47: {
      const float step = shift ? 0.25f : 0.05f;
      m_postFx.saturation = std::clamp(m_postFx.saturation + (static_cast<float>(d) * step), 0.0f, 2.0f);
      showToast(TextFormat("Saturation: %.2fx", m_postFx.saturation));
    } break;

    case 48: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.outline = clamp01(m_postFx.outline + (static_cast<float>(d) * step));
      showToast(TextFormat("Outlines: %.0f%%", m_postFx.outline * 100.0f));
    } break;

    case 49: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.outlineThreshold = clamp01(m_postFx.outlineThreshold + (static_cast<float>(d) * step));
      showToast(TextFormat("Outline threshold: %.0f%%", m_postFx.outlineThreshold * 100.0f));
    } break;

    case 50: {
      const float step = shift ? 0.50f : 0.10f;
      m_postFx.outlineThickness = std::clamp(m_postFx.outlineThickness + (static_cast<float>(d) * step), 0.5f, 4.0f);
      showToast(TextFormat("Outline thickness: %.2f px", m_postFx.outlineThickness));
    } break;

    // Lens precipitation (rain on lens / wet camera)
    case 51: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.lensWeather = clamp01(m_postFx.lensWeather + (static_cast<float>(d) * step));
      showToast(TextFormat("Lens rain: %.0f%%", m_postFx.lensWeather * 100.0f));
    } break;

    case 52: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.lensDistort = clamp01(m_postFx.lensDistort + (static_cast<float>(d) * step));
      showToast(TextFormat("Lens distort: %.0f%%", m_postFx.lensDistort * 100.0f));
    } break;

    case 53: {
      const float step = shift ? 0.25f : 0.05f;
      m_postFx.lensScale = std::clamp(m_postFx.lensScale + (static_cast<float>(d) * step), 0.5f, 2.0f);
      showToast(TextFormat("Lens scale: %.2fx", m_postFx.lensScale));
    } break;

    case 54: {
      const float step = shift ? 0.10f : 0.03f;
      m_postFx.lensDrips = clamp01(m_postFx.lensDrips + (static_cast<float>(d) * step));
      showToast(TextFormat("Lens drips: %.0f%%", m_postFx.lensDrips * 100.0f));
    } break;

    case 55:
      m_postFx = PostFxSettings{};
      // Reset any in-flight temporal history.
      m_taaActiveLastFrame = false;
      m_taaPrevCamValid = false;
      m_taaFrameIndex = 0;
      showToast("Post FX: RESET");
      if (!wantsWorldRenderTarget()) unloadWorldRenderTarget();
      break;

    case 56: {
      // Cycle procedural gfx palette theme.
      constexpr GfxTheme kThemes[] = {
        GfxTheme::Classic,
        GfxTheme::Autumn,
        GfxTheme::Desert,
        GfxTheme::Noir,
        GfxTheme::Neon,
        GfxTheme::Pastel,
        GfxTheme::SpaceColony,
        GfxTheme::Procedural,
        GfxTheme::ProceduralMuted,
        GfxTheme::ProceduralVibrant,
      };

      const int n = static_cast<int>(sizeof(kThemes) / sizeof(kThemes[0]));
      const GfxTheme cur = m_renderer.gfxTheme();
      int idx = 0;
      for (int i = 0; i < n; ++i) {
        if (kThemes[i] == cur) {
          idx = i;
          break;
        }
      }

      const int step = (dir >= 0) ? 1 : -1;
      const int nextIdx = (idx + step + n) % n;
      const GfxTheme next = kThemes[nextIdx];

      m_renderer.setGfxTheme(next);

      auto themeLabel = [](GfxTheme t) -> const char* {
        switch (t) {
          case GfxTheme::Classic: return "Classic";
          case GfxTheme::Autumn: return "Autumn";
          case GfxTheme::Desert: return "Desert";
          case GfxTheme::Noir: return "Noir";
          case GfxTheme::Neon: return "Neon";
          case GfxTheme::Pastel: return "Pastel";
          case GfxTheme::SpaceColony: return "Space Colony";
          case GfxTheme::Procedural: return "Procedural";
          case GfxTheme::ProceduralMuted: return "Procedural (Muted)";
          case GfxTheme::ProceduralVibrant: return "Procedural (Vibrant)";
          default: return "Classic";
        }
      };

      showToast(TextFormat("Gfx theme: %s", themeLabel(next)));
      break;
    }

    default:
      break;
  }

  m_videoSelectionVisual = m_videoSelection;
}
void Game::toggleFullscreen()
{
  // If we are in borderless-windowed mode, disable it first.
  if (m_borderlessWindowed) {
    toggleBorderlessWindowed();
  }

  if (!IsWindowFullscreen()) {
    // Store current windowed geometry before entering fullscreen.
    const Vector2 pos = GetWindowPosition();
    m_windowedX = static_cast<int>(pos.x);
    m_windowedY = static_cast<int>(pos.y);
    m_windowedW = GetScreenWidth();
    m_windowedH = GetScreenHeight();
  }

  ToggleFullscreen();

  if (!IsWindowFullscreen()) {
    // Restore the previous windowed geometry.
    SetWindowSize(m_windowedW, m_windowedH);
    SetWindowPosition(m_windowedX, m_windowedY);
  }

  showToast(IsWindowFullscreen() ? "Fullscreen: on (F11)" : "Fullscreen: off (F11)", 2.0f);
}

void Game::toggleBorderlessWindowed()
{
  // Borderless windowed mode is implemented by making the window undecorated
  // and sizing it to the current monitor.
  if (IsWindowFullscreen()) {
    ToggleFullscreen();
  }

  if (!m_borderlessWindowed) {
    const Vector2 pos = GetWindowPosition();
    m_windowedX = static_cast<int>(pos.x);
    m_windowedY = static_cast<int>(pos.y);
    m_windowedW = GetScreenWidth();
    m_windowedH = GetScreenHeight();

    SetWindowState(FLAG_WINDOW_UNDECORATED);
    const int monitor = GetCurrentMonitor();
    const int mw = GetMonitorWidth(monitor);
    const int mh = GetMonitorHeight(monitor);
    SetWindowPosition(0, 0);
    SetWindowSize(mw, mh);
    m_borderlessWindowed = true;
    showToast("Borderless fullscreen: on (Alt+Enter)", 2.0f);
  } else {
    ClearWindowState(FLAG_WINDOW_UNDECORATED);
    SetWindowSize(m_windowedW, m_windowedH);
    SetWindowPosition(m_windowedX, m_windowedY);
    m_borderlessWindowed = false;
    showToast("Borderless fullscreen: off (Alt+Enter)", 2.0f);
  }
}

void Game::toggleVsync()
{
  m_cfg.vsync = !m_cfg.vsync;

  if (m_cfg.vsync) {
    SetWindowState(FLAG_VSYNC_HINT);
    showToast("VSync: on", 1.5f);
  } else {
    ClearWindowState(FLAG_VSYNC_HINT);
    showToast("VSync: off", 1.5f);
  }
}


VisualPrefs Game::captureVisualPrefs() const
{
  VisualPrefs p{};
  p.vsync = m_cfg.vsync;

  p.uiScaleAuto = m_uiScaleAuto;
  p.uiScaleManual = m_uiScaleManual;

  {
    const ui::Settings s = ui::GetSettings();
    p.uiTheme.accentFromSeed = s.accentFromSeed;
    p.uiTheme.accentHueDeg = s.accentHueDeg;
    p.uiTheme.accentSaturation = s.accentSaturation;
    p.uiTheme.accentValue = s.accentValue;
    p.uiTheme.roundness = s.roundness;
    p.uiTheme.roundSegments = s.roundSegments;
    p.uiTheme.noiseAlpha = s.noiseAlpha;
    p.uiTheme.noiseScale = s.noiseScale;
    p.uiTheme.headerSheenStrength = s.headerSheenStrength;
    p.uiTheme.fontAtlasScale = s.fontAtlasScale;
    p.uiTheme.fontFilterPoint = s.fontFilterPoint;
  }

  p.worldRenderScaleAuto = m_worldRenderScaleAuto;
  p.worldRenderScale = m_worldRenderScale;
  p.worldRenderScaleMin = m_worldRenderScaleMin;
  p.worldRenderScaleMax = m_worldRenderScaleMax;
  p.worldRenderTargetFps = m_worldRenderTargetFps;
  p.worldRenderFilterPoint = m_worldRenderFilterPoint;

  p.mergedZoneBuildings = m_mergedZoneBuildings;

  p.baseCacheEnabled = m_renderer.baseCacheEnabled();
  p.layerMask = m_renderer.layerMask();
  p.shadows = m_renderer.shadowSettings();
  p.dayNight = m_renderer.dayNightSettings();
  p.weather = m_renderer.weatherSettings();
  p.materialFx = m_renderer.materialFxSettings();
  p.cloudShadows = m_renderer.cloudShadowSettings();
  p.volumetricClouds = m_renderer.volumetricCloudSettings();

  // Post FX
  p.postFx = m_postFx;
  p.elevation = m_elev;
  return p;
}

void Game::applyVisualPrefs(const VisualPrefs& prefs)
{
  // Display
  m_cfg.vsync = prefs.vsync;
  if (m_cfg.vsync) {
    SetWindowState(FLAG_VSYNC_HINT);
  } else {
    ClearWindowState(FLAG_VSYNC_HINT);
  }

  // UI scale
  m_uiScaleAuto = prefs.uiScaleAuto;
  m_uiScaleManual = std::clamp(prefs.uiScaleManual, 0.5f, 4.0f);
  if (m_uiScaleAuto) {
    m_uiScale = computeAutoUiScale(GetScreenWidth(), GetScreenHeight());
  } else {
    m_uiScale = m_uiScaleManual;
  }

  // UI theme + fonts
  {
    ui::Settings s = ui::GetSettings();
    s.accentFromSeed = prefs.uiTheme.accentFromSeed;
    s.accentHueDeg = prefs.uiTheme.accentHueDeg;
    s.accentSaturation = prefs.uiTheme.accentSaturation;
    s.accentValue = prefs.uiTheme.accentValue;
    s.roundness = prefs.uiTheme.roundness;
    s.roundSegments = prefs.uiTheme.roundSegments;
    s.noiseAlpha = prefs.uiTheme.noiseAlpha;
    s.noiseScale = prefs.uiTheme.noiseScale;
    s.headerSheenStrength = prefs.uiTheme.headerSheenStrength;
    s.fontAtlasScale = prefs.uiTheme.fontAtlasScale;
    s.fontFilterPoint = prefs.uiTheme.fontFilterPoint;
    ui::SetSettings(s);
  }

  // World render scaling
  m_worldRenderScaleAuto = prefs.worldRenderScaleAuto;
  m_worldRenderScale = clampWorldRenderScale(prefs.worldRenderScale);
  m_worldRenderScaleMin = clampWorldRenderScale(prefs.worldRenderScaleMin);
  m_worldRenderScaleMax = clampWorldRenderScale(prefs.worldRenderScaleMax);
  if (m_worldRenderScaleMin > m_worldRenderScaleMax) {
    std::swap(m_worldRenderScaleMin, m_worldRenderScaleMax);
  }

  m_worldRenderTargetFps = std::clamp(prefs.worldRenderTargetFps, 15, 240);
  m_worldRenderFilterPoint = prefs.worldRenderFilterPoint;

  // Post FX
  m_postFx = prefs.postFx;
  m_postFx.colorBits = std::clamp(m_postFx.colorBits, 2, 8);
  m_postFx.ditherStrength = std::clamp(m_postFx.ditherStrength, 0.0f, 1.0f);
  m_postFx.grain = std::clamp(m_postFx.grain, 0.0f, 1.0f);
  m_postFx.vignette = std::clamp(m_postFx.vignette, 0.0f, 1.0f);
  m_postFx.chroma = std::clamp(m_postFx.chroma, 0.0f, 1.0f);
  m_postFx.scanlines = std::clamp(m_postFx.scanlines, 0.0f, 1.0f);
  m_postFx.fxaa = std::clamp(m_postFx.fxaa, 0.0f, 1.0f);
  m_postFx.sharpen = std::clamp(m_postFx.sharpen, 0.0f, 1.0f);

  // Filmic tonemap / grade (safe clamps, even if the effect is disabled).
  m_postFx.exposure = std::clamp(m_postFx.exposure, 0.0f, 4.0f);
  m_postFx.contrast = std::clamp(m_postFx.contrast, 0.0f, 2.0f);
  m_postFx.saturation = std::clamp(m_postFx.saturation, 0.0f, 2.0f);

  // Screen-space outlines.
  m_postFx.outline = std::clamp(m_postFx.outline, 0.0f, 1.0f);
  m_postFx.outlineThreshold = std::clamp(m_postFx.outlineThreshold, 0.0f, 1.0f);
  m_postFx.outlineThickness = std::clamp(m_postFx.outlineThickness, 0.5f, 4.0f);

  // Temporal AA.
  m_postFx.taaHistory = std::clamp(m_postFx.taaHistory, 0.0f, 0.98f);
  m_postFx.taaJitter = std::clamp(m_postFx.taaJitter, 0.0f, 1.0f);
  m_postFx.taaResponse = std::clamp(m_postFx.taaResponse, 0.0f, 1.0f);

  // Bloom.
  m_postFx.bloom = std::clamp(m_postFx.bloom, 0.0f, 1.0f);
  m_postFx.bloomThreshold = std::clamp(m_postFx.bloomThreshold, 0.0f, 1.0f);
  m_postFx.bloomKnee = std::clamp(m_postFx.bloomKnee, 0.0f, 1.0f);
  m_postFx.bloomRadius = std::clamp(m_postFx.bloomRadius, 0.25f, 4.0f);
  m_postFx.bloomDownsample = std::clamp(m_postFx.bloomDownsample, 1, 8);


  if (m_worldRenderScaleAuto) {
    m_worldRenderScale = std::clamp(m_worldRenderScale, m_worldRenderScaleMin, m_worldRenderScaleMax);
  }

  // Mirror into cfg so console/info panels show consistent state.
  m_cfg.worldRenderScaleAuto = m_worldRenderScaleAuto;
  m_cfg.worldRenderScale = m_worldRenderScale;
  m_cfg.worldRenderScaleMin = m_worldRenderScaleMin;
  m_cfg.worldRenderScaleMax = m_worldRenderScaleMax;
  m_cfg.worldRenderTargetFps = m_worldRenderTargetFps;
  m_cfg.worldRenderFilterPoint = m_worldRenderFilterPoint;

  // Update / release the world render target if the mode changed.
  if (!wantsWorldRenderTarget()) {
    unloadWorldRenderTarget();
  } else if (m_worldRenderRTValid) {
    SetTextureFilter(
        m_worldRenderRT.texture,
        m_worldRenderFilterPoint ? TEXTURE_FILTER_POINT : TEXTURE_FILTER_BILINEAR);
  }

  // World visuals
  m_mergedZoneBuildings = prefs.mergedZoneBuildings;
  m_cfg.mergedZoneBuildings = m_mergedZoneBuildings;

  // Elevation
  m_elev = prefs.elevation;
  m_renderer.setElevationSettings(m_elev);

  // Renderer visuals
  m_renderer.setBaseCacheEnabled(prefs.baseCacheEnabled);
  m_renderer.setLayerMask(prefs.layerMask);
  m_renderer.setShadowSettings(prefs.shadows);
  m_renderer.setDayNightSettings(prefs.dayNight);
  m_renderer.setWeatherSettings(prefs.weather);
  m_renderer.setMaterialFxSettings(prefs.materialFx);
  m_renderer.setCloudShadowSettings(prefs.cloudShadows);
  m_renderer.setVolumetricCloudSettings(prefs.volumetricClouds);

  // Safe: force caches to rebuild under the new toggles.
  m_renderer.markBaseCacheDirtyAll();
}

bool Game::loadVisualPrefsFile(const std::string& path, bool toast)
{
  VisualPrefs prefs = captureVisualPrefs();
  std::string err;
  if (!LoadVisualPrefsJsonFile(path, prefs, err)) {
    if (toast) {
      showToast(std::string("Prefs load failed: ") + err, 3.0f);
    }
    return false;
  }

  applyVisualPrefs(prefs);

  m_visualPrefsPath = path;
  m_visualPrefsDirty = false;
  m_visualPrefsSaveTimer = 0.0f;
  m_visualPrefsLastSnapshot = captureVisualPrefs();

  if (toast) {
    showToast(std::string("Loaded prefs: ") + path, 1.5f);
  }
  return true;
}

bool Game::saveVisualPrefsFile(const std::string& path, bool toast)
{
  VisualPrefs prefs = captureVisualPrefs();
  std::string err;
  if (!WriteVisualPrefsJsonFile(path, prefs, err, 2)) {
    if (toast) {
      showToast(std::string("Prefs save failed: ") + err, 3.0f);
    }
    return false;
  }

  m_visualPrefsPath = path;
  m_visualPrefsDirty = false;
  m_visualPrefsSaveTimer = 0.0f;
  m_visualPrefsLastSnapshot = prefs;

  if (toast) {
    showToast(std::string("Saved prefs: ") + path, 1.5f);
  }
  return true;
}


void Game::queueDossierCurrentWorld(CityDossierConfig cfg, const char* toastLabel)
{
  if (cfg.outDir.empty()) {
    showToast("dossier: outDir is empty", 2.0f);
    m_console.print("dossier: outDir is empty");
    return;
  }

  if (m_dossierQueue.size() >= static_cast<std::size_t>(kDossierQueueMax)) {
    showToast("Dossier queue full", 2.0f);
    m_console.print("dossier: queue full");
    return;
  }

  DossierRequest req;
  req.kind = DossierRequest::Kind::CurrentWorld;
  req.cfg = std::move(cfg);
  req.procCfg = m_procCfg;
  req.simCfg = m_sim.config();

  // Snapshot tick history (prefer full Stats history so the dossier graphs match the in-game report).
  req.ticks = m_tickStatsHistory;
  if (req.ticks.empty()) {
    req.ticks.push_back(m_world.stats());
  }

  // Copy the world for thread safety.
  req.world = m_world;

  m_dossierQueue.push_back(std::move(req));

  const std::string p = m_dossierQueue.back().cfg.outDir.string();
  if (toastLabel) {
    showToast(std::string(toastLabel) + ": " + p, 2.0f);
  } else {
    showToast(std::string("Queued dossier: ") + p, 2.0f);
  }
  m_console.print(std::string("dossier: queued ") + p);
}

void Game::queueDossierSeeds(CityDossierConfig cfg, std::vector<std::uint64_t> seeds, int bakeDays,
                            const char* toastLabel)
{
  if (seeds.empty()) {
    showToast("dossier: no seeds to export", 2.0f);
    m_console.print("dossier: no seeds to export");
    return;
  }

  if (cfg.outDir.empty()) {
    showToast("dossier: outDir is empty", 2.0f);
    m_console.print("dossier: outDir is empty");
    return;
  }

  if (m_dossierQueue.size() >= static_cast<std::size_t>(kDossierQueueMax)) {
    showToast("Dossier queue full", 2.0f);
    m_console.print("dossier: queue full");
    return;
  }

  DossierRequest req;
  req.kind = DossierRequest::Kind::SeedBake;
  req.cfg = std::move(cfg);
  req.procCfg = m_procCfg;
  req.simCfg = m_sim.config();
  req.seeds = std::move(seeds);
  req.bakeDays = std::max(0, bakeDays);
  req.w = m_world.width();
  req.h = m_world.height();

  m_dossierQueue.push_back(std::move(req));

  const auto& back = m_dossierQueue.back();
  if (toastLabel) {
    showToast(std::string(toastLabel) + ": " + back.cfg.outDir.string(), 2.0f);
  } else if (back.seeds.size() == 1) {
    showToast(std::string("Queued seed dossier: ") + HexU64(back.seeds[0]), 2.0f);
  } else {
    showToast(std::string("Queued ") + std::to_string(back.seeds.size()) + " seed dossiers", 2.0f);
  }
  m_console.print(std::string("dossier: queued seeds -> ") + back.cfg.outDir.string());
}

void Game::updateDossierExportJob()
{
  // Surface completion first (so a completed job can chain into the next queued request on the same frame).
  if (m_dossierJob && m_dossierJob->done.load(std::memory_order_relaxed)) {
    if (m_dossierJob->worker.joinable()) {
      m_dossierJob->worker.join();
    }

    bool ok = false;
    bool cancelled = m_dossierJob->cancel.load(std::memory_order_relaxed);
    CityDossierResult res;
    std::string err;
    int okSeeds = 0;
    int failSeeds = 0;
    int seedCount = m_dossierJob->seedCount.load(std::memory_order_relaxed);

    {
      std::lock_guard<std::mutex> lock(m_dossierJob->mutex);
      ok = m_dossierJob->ok;
      res = m_dossierJob->res;
      err = m_dossierJob->err;
      okSeeds = m_dossierJob->okSeeds;
      failSeeds = m_dossierJob->failSeeds;
    }

    std::string msg;

    if (m_dossierJob->seeds.empty()) {
      // Single-world export.
      if (ok) {
        const std::string p = res.outDir.string();
        msg = std::string("Dossier exported: ") + p;
      } else if (cancelled || err == "Cancelled") {
        msg = "Dossier export cancelled";
      } else {
        msg = err.empty() ? std::string("Dossier export failed")
                          : (std::string("Dossier export failed: ") + err);
      }

      m_lastDossierOutDir = ok ? res.outDir.string() : m_dossierJob->cfg.outDir.string();
    } else {
      // Seed-bake/batch export.
      const int total = seedCount;
      const std::string base = m_dossierJob->cfg.outDir.string();

      if (cancelled) {
        msg = std::string("Seed dossier batch cancelled (") + std::to_string(okSeeds) + "/" +
              std::to_string(total) + "): " + base;
      } else if (failSeeds == 0) {
        msg = std::string("Seed dossiers exported (") + std::to_string(okSeeds) + "/" +
              std::to_string(total) + "): " + base;
      } else {
        msg = std::string("Seed dossier batch failed (") + std::to_string(okSeeds) + "/" +
              std::to_string(total) + ", " + std::to_string(failSeeds) +
              " failed): " + base;
        if (!err.empty()) {
          msg += std::string(" (e.g. ") + err + ")";
        }
      }

      m_lastDossierOutDir = base;
    }

    m_lastDossierOk = ok;
    m_lastDossierMsg = msg;

    showToast(msg, 4.0f);
    m_console.print(std::string("dossier: ") + msg);

    m_dossierJob.reset();
  }

  // Start a new job if we're idle and there is queued work.
  if (m_dossierJob || m_dossierQueue.empty()) return;

  DossierRequest req = std::move(m_dossierQueue.front());
  m_dossierQueue.pop_front();

  auto job = std::make_unique<DossierJob>();
  job->cfg = std::move(req.cfg);
  job->procCfg = req.procCfg;
  job->simCfg = req.simCfg;

  if (req.kind == DossierRequest::Kind::CurrentWorld) {
    job->ticks = std::move(req.ticks);
    job->world = std::move(req.world);
  } else {
    job->seeds = std::move(req.seeds);
    job->bakeDays = req.bakeDays;
    job->w = req.w;
    job->h = req.h;
    job->seedIndex.store(0, std::memory_order_relaxed);
    job->seedCount.store(static_cast<int>(job->seeds.size()), std::memory_order_relaxed);
  }

  DossierJob* jp = job.get();

  jp->worker = std::thread([jp]() {
    CityDossierResult lastRes;
    std::string firstErr;

    auto setStage = [jp](const std::string& s) {
      std::lock_guard<std::mutex> lock(jp->mutex);
      jp->stage = s;
    };

    // Export callback (runs in worker thread).
    auto progress = [jp](const CityDossierProgress& p) -> bool {
      jp->stepIndex.store(p.stepIndex, std::memory_order_relaxed);
      jp->stepCount.store(p.stepCount, std::memory_order_relaxed);
      {
        std::lock_guard<std::mutex> lock(jp->mutex);
        jp->stage = p.stage;
      }
      return !jp->cancel.load(std::memory_order_relaxed);
    };

    bool ok = true;

    if (jp->seeds.empty()) {
      std::string err;
      const bool wrote = WriteCityDossier(jp->world, jp->procCfg, jp->simCfg, jp->ticks, jp->cfg, &lastRes, err, progress);
      if (!wrote && !err.empty()) {
        firstErr = err;
      }
      ok = wrote;
    } else {
      const int total = static_cast<int>(jp->seeds.size());
      jp->seedCount.store(total, std::memory_order_relaxed);

      int okSeeds = 0;
      int failSeeds = 0;

      for (int i = 0; i < total; ++i) {
        if (jp->cancel.load(std::memory_order_relaxed)) {
          ok = false;
          break;
        }

        const std::uint64_t seed = jp->seeds[static_cast<std::size_t>(i)];
        jp->seedIndex.store(i + 1, std::memory_order_relaxed);

        // --- Bake / simulate ---
        const int bakeDays = std::max(0, jp->bakeDays);
        jp->stepIndex.store(0, std::memory_order_relaxed);
        jp->stepCount.store(std::max(1, bakeDays), std::memory_order_relaxed);
        setStage(std::string("seed ") + std::to_string(i + 1) + "/" + std::to_string(total) + ": baking");

        World w = GenerateWorld(jp->w, jp->h, seed, jp->procCfg);

        Simulator sim(jp->simCfg);
        sim.refreshDerivedStats(w);

        std::vector<Stats> ticks;
        ticks.reserve(static_cast<std::size_t>(bakeDays) + 1);
        ticks.push_back(w.stats());

        for (int d = 0; d < bakeDays; ++d) {
          if (jp->cancel.load(std::memory_order_relaxed)) break;
          sim.stepOnce(w);
          ticks.push_back(w.stats());
          jp->stepIndex.store(d + 1, std::memory_order_relaxed);
          jp->stepCount.store(std::max(1, bakeDays), std::memory_order_relaxed);

          // Update stage a bit less often to reduce mutex churn.
          if (d == 0 || d == bakeDays - 1 || ((d + 1) % 8 == 0)) {
            setStage(std::string("seed ") + std::to_string(i + 1) + "/" + std::to_string(total) + ": baking " +
                     std::to_string(d + 1) + "/" + std::to_string(bakeDays));
          }
        }

        if (jp->cancel.load(std::memory_order_relaxed)) {
          ok = false;
          break;
        }

        // --- Export dossier ---
        CityDossierConfig cfg = jp->cfg;
        if (total > 1) {
          cfg.outDir = jp->cfg.outDir / (std::string("seed_") + HexU64(seed));
        }

        auto progressSeed = [jp, i, total](const CityDossierProgress& p) -> bool {
          jp->stepIndex.store(p.stepIndex, std::memory_order_relaxed);
          jp->stepCount.store(p.stepCount, std::memory_order_relaxed);
          {
            std::lock_guard<std::mutex> lock(jp->mutex);
            jp->stage = std::string("seed ") + std::to_string(i + 1) + "/" + std::to_string(total) + ": " + p.stage;
          }
          return !jp->cancel.load(std::memory_order_relaxed);
        };

        std::string err;
        CityDossierResult res;
        const bool wrote = WriteCityDossier(w, jp->procCfg, jp->simCfg, ticks, cfg, &res, err, progressSeed);
        lastRes = res;

        if (wrote) {
          ++okSeeds;
        } else {
          ++failSeeds;
          ok = false;
          if (firstErr.empty()) {
            firstErr = err.empty() ? (std::string("seed ") + HexU64(seed) + ": export failed")
                                   : (std::string("seed ") + HexU64(seed) + ": " + err);
          }

          // Continue batch even if one fails; user can inspect partial results.
        }
      }

      {
        std::lock_guard<std::mutex> lock(jp->mutex);
        jp->okSeeds = okSeeds;
        jp->failSeeds = failSeeds;
      }

      // Consider it ok only if all seeds succeeded and we were not cancelled.
      ok = (failSeeds == 0) && !jp->cancel.load(std::memory_order_relaxed);
    }

    {
      std::lock_guard<std::mutex> lock(jp->mutex);
      jp->ok = ok;
      jp->res = lastRes;
      jp->err = firstErr;
    }

    jp->done.store(true, std::memory_order_relaxed);
  });

  m_dossierJob = std::move(job);

  if (m_dossierJob->seeds.empty()) {
    showToast(std::string("Dossier export started: ") + m_dossierJob->cfg.outDir.string(), 2.0f);
    m_console.print(std::string("dossier: started ") + m_dossierJob->cfg.outDir.string());
  } else {
    showToast(std::string("Seed dossier export started (") + std::to_string(m_dossierJob->seeds.size()) +
             " seeds): " + m_dossierJob->cfg.outDir.string(), 2.0f);
    m_console.print(std::string("dossier: started seeds -> ") + m_dossierJob->cfg.outDir.string());
  }
}


void Game::updateMeshExportJob()
{
  if (!m_meshJob) return;
  if (!m_meshJob->done.load(std::memory_order_relaxed)) return;

  // Join the worker before reading its results.
  if (m_meshJob->worker.joinable()) {
    m_meshJob->worker.join();
  }

  bool ok = false;
  MeshExportStats stats{};
  std::string err;
  std::string outPath;
  std::string mtlPath;

  {
    std::lock_guard<std::mutex> lock(m_meshJob->mutex);
    ok = m_meshJob->ok;
    stats = m_meshJob->stats;
    err = m_meshJob->err;
    outPath = m_meshJob->outPath;
    mtlPath = m_meshJob->mtlPath;
  }

  // Build a compact user-facing label (toasts are width-limited).
  std::string fileLabel = outPath;
  try {
    fileLabel = std::filesystem::path(outPath).filename().string();
  } catch (...) {
  }

  std::string msg;
  if (ok) {
    msg = TextFormat("Mesh exported: %s  (%llu v, %llu t)", fileLabel.c_str(),
                     static_cast<unsigned long long>(stats.vertices),
                     static_cast<unsigned long long>(stats.triangles));
  } else {
    msg = err.empty() ? std::string("Mesh export failed") : (std::string("Mesh export failed: ") + err);
  }

  m_lastMeshOk = ok;
  m_lastMeshPath = outPath;
  m_lastMeshMsg = msg;
  m_lastMeshStats = stats;

  showToast(msg, 4.0f);

  // Console gets full paths for easy copy/paste.
  if (ok) {
    if (!mtlPath.empty()) {
      m_console.print(std::string("mesh: exported ") + outPath + " (mtl " + mtlPath + ")");
    } else {
      m_console.print(std::string("mesh: exported ") + outPath);
    }
  } else {
    m_console.print(std::string("mesh: failed ") + (err.empty() ? std::string("(unknown)") : err));
  }

  m_meshJob.reset();
}

namespace {

constexpr int kPolicyOptPresetCount = 5;

inline const char* PolicyOptPresetName(int preset)
{
  switch (preset) {
  case 0: return "Balanced";
  case 1: return "Growth";
  case 2: return "Profit";
  case 3: return "Happiness";
  case 4: return "Traffic relief";
  default: return "Balanced";
  }
}

inline PolicyObjective PolicyOptPresetObjective(int preset)
{
  PolicyObjective o{};

  // The objective is a simple linear score. These weights are tuned to keep
  // the different terms in a comparable range for typical city sizes.
  //
  // moneyDelta is in dollars; population terms are in "people".
  // To avoid money completely dominating the score, we scale it down.
  switch (preset) {
  default:
  case 0: // Balanced
    o.wMoneyDelta = 0.002;
    o.wPopulation = 0.02;
    o.wHappyPop = 0.08;
    o.wUnemployed = 0.03;
    o.wCongestionPop = 0.06;
    o.minHappiness = 0.30;
    break;
  case 1: // Growth
    o.wMoneyDelta = 0.001;
    o.wPopulation = 0.05;
    o.wHappyPop = 0.06;
    o.wUnemployed = 0.05;
    o.wCongestionPop = 0.04;
    o.minHappiness = 0.25;
    break;
  case 2: // Profit
    o.wMoneyDelta = 0.005;
    o.wPopulation = 0.00;
    o.wHappyPop = 0.02;
    o.wUnemployed = 0.02;
    o.wCongestionPop = 0.02;
    o.minHappiness = 0.15;
    break;
  case 3: // Happiness
    o.wMoneyDelta = 0.001;
    o.wPopulation = 0.01;
    o.wHappyPop = 0.20;
    o.wUnemployed = 0.06;
    o.wCongestionPop = 0.08;
    o.minHappiness = 0.50;
    break;
  case 4: // Traffic relief
    o.wMoneyDelta = 0.001;
    o.wPopulation = 0.01;
    o.wHappyPop = 0.06;
    o.wUnemployed = 0.03;
    o.wCongestionPop = 0.20;
    o.minHappiness = 0.30;
    break;
  }

  return o;
}

} // namespace

void Game::applyPolicyCandidate(const PolicyCandidate& p, const char* toastLabel)
{
  ApplyPolicyToSimConfig(p, m_sim.config());

  // Updating policies affects derived stats and overlays.
  m_sim.refreshDerivedStats(m_world);
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_tradeDirty = true;
  m_landValueDirty = true;
  m_vehiclesDirty = true;
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;
  m_outsideOverlayRoadToEdge.clear();
  invalidateAnalysisLayers();

  if (toastLabel) {
    showToast(toastLabel, 2.0f);
  } else {
    showToast(TextFormat("Applied policy: R%d C%d I%d  RdM%d PkM%d",
                         p.taxResidential, p.taxCommercial, p.taxIndustrial,
                         p.maintenanceRoad, p.maintenancePark),
              2.0f);
  }
}

void Game::startPolicyOptimization()
{
  if (m_policyOptJob) {
    if (!m_policyOptJob->done.load(std::memory_order_relaxed)) {
      showToast("Policy advisor already running", 2.0f);
      return;
    }
    m_policyOptJob.reset();
  }

  // Reset progress (visible in the UI).
  m_policyOptProgress.iterationsTotal.store(0);
  m_policyOptProgress.iterationsCompleted.store(0);
  m_policyOptProgress.candidatesEvaluated.store(0);
  m_policyOptProgress.exhaustive.store(false);
  m_policyOptProgress.done.store(false);

  // Build a search space centered around the current policy.
  const SimConfig& baseCfg = m_sim.config();

  const auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

  const int taxRad = std::clamp(m_policyOptTaxRadius, 0, 10);
  const int maintRad = std::clamp(m_policyOptMaintRadius, 0, 10);

  PolicySearchSpace space{};
  space.taxResMin = clampI(baseCfg.taxResidential - taxRad, 0, 10);
  space.taxResMax = clampI(baseCfg.taxResidential + taxRad, 0, 10);
  space.taxComMin = clampI(baseCfg.taxCommercial - taxRad, 0, 10);
  space.taxComMax = clampI(baseCfg.taxCommercial + taxRad, 0, 10);
  space.taxIndMin = clampI(baseCfg.taxIndustrial - taxRad, 0, 10);
  space.taxIndMax = clampI(baseCfg.taxIndustrial + taxRad, 0, 10);

  space.maintRoadMin = clampI(baseCfg.maintenanceRoad - maintRad, 0, 5);
  space.maintRoadMax = clampI(baseCfg.maintenanceRoad + maintRad, 0, 5);
  space.maintParkMin = clampI(baseCfg.maintenancePark - maintRad, 0, 5);
  space.maintParkMax = clampI(baseCfg.maintenancePark + maintRad, 0, 5);

  PolicyOptimizerConfig cfg = m_policyOptCfg;
  cfg.objective = PolicyOptPresetObjective(m_policyOptPreset);

  // Deterministic seed per city-seed + day + preset so repeated runs are stable,
  // but different cities/days explore different parts of the space.
  cfg.rngSeed = static_cast<std::uint64_t>(m_world.seed()) ^
                (static_cast<std::uint64_t>(m_world.stats().day) * 0x9E3779B97F4A7C15ULL) ^
                (static_cast<std::uint64_t>(m_policyOptPreset) * 0xD1B54A32D192ED03ULL);

  auto job = std::make_unique<PolicyOptJob>();
  job->world = m_world;   // snapshot for thread safety
  job->simCfg = baseCfg;  // snapshot (policy fields will be overridden per candidate)
  job->space = space;
  job->cfg = cfg;
  job->progress = &m_policyOptProgress;

  PolicyOptJob* jp = job.get();
  jp->worker = std::thread([jp]() {
    bool ok = true;
    PolicyOptimizationResult res;
    std::string err;

    try {
      res = OptimizePolicies(jp->world, jp->simCfg, jp->space, jp->cfg, jp->progress);
      ok = true;
    } catch (const std::exception& e) {
      ok = false;
      err = e.what();
    } catch (...) {
      ok = false;
      err = "Unknown error";
    }

    {
      std::lock_guard<std::mutex> lock(jp->mutex);
      jp->ok = ok;
      jp->res = std::move(res);
      jp->err = std::move(err);
    }

    jp->done.store(true, std::memory_order_relaxed);
  });

  m_policyOptJob = std::move(job);
  showToast(TextFormat("Policy advisor: optimizing (%s)", PolicyOptPresetName(m_policyOptPreset)), 2.0f);
}

void Game::updatePolicyOptimizationJob()
{
  if (!m_policyOptJob) return;
  if (!m_policyOptJob->done.load(std::memory_order_relaxed)) return;

  // Join worker before reading results.
  if (m_policyOptJob->worker.joinable()) {
    m_policyOptJob->worker.join();
  }

  bool ok = false;
  PolicyOptimizationResult res;
  std::string err;

  {
    std::lock_guard<std::mutex> lock(m_policyOptJob->mutex);
    ok = m_policyOptJob->ok;
    res = std::move(m_policyOptJob->res);
    err = m_policyOptJob->err;
  }

  m_policyOptHaveResult = true;
  m_policyOptResultOk = ok;
  m_policyOptResultDay = m_world.stats().day;
  m_policyOptResultErr = err;
  if (ok) {
    m_policyOptResult = std::move(res);
    m_policyOptTopSelection = 0;
    m_policyOptTopFirst = 0;
    showToast("Policy advisor: done", 2.0f);
  } else {
    m_policyOptResult = PolicyOptimizationResult{};
    showToast(err.empty() ? "Policy advisor failed" : (std::string("Policy advisor failed: ") + err), 3.0f);
  }

  m_policyOptJob.reset();
}




void Game::updateVisualPrefsAutosave(float dt)
{
  if (!m_visualPrefsAutosave) return;

  const VisualPrefs current = captureVisualPrefs();
  if (!VisualPrefsEqual(current, m_visualPrefsLastSnapshot)) {
    m_visualPrefsLastSnapshot = current;
    m_visualPrefsDirty = true;
    // Debounce: wait until the user stops dragging sliders / spamming hotkeys.
    m_visualPrefsSaveTimer = 0.75f;
  }

  if (!m_visualPrefsDirty) return;

  m_visualPrefsSaveTimer -= dt;
  if (m_visualPrefsSaveTimer > 0.0f) return;

  // Try to write; if it fails keep dirty and back off.
  if (!saveVisualPrefsFile(m_visualPrefsPath, false)) {
    m_visualPrefsDirty = true;
    m_visualPrefsSaveTimer = 2.0f;
  }
}

void Game::updateSeedMining(float /*dt*/)
{
  if (!m_mineSession) return;

  // If the session is finished, ensure we stop auto-running and surface a message once.
  if (m_mineSession->done()) {
    if (m_mineAutoRun) {
      m_mineAutoRun = false;
      m_console.print("mine: complete. use 'mine top' to list results");
      showToast(TextFormat("Seed mining complete (%d samples)", m_mineSession->total()), 2.5f);
    }
    // Clear any pending manual steps.
    m_mineManualStepRequests = 0;
    return;
  }

  // Manual stepping: queued by console/UI so large requests don't stall a frame.
  int manualRemaining = std::max(0, m_mineManualStepRequests);
  int autoRemaining = m_mineAutoRun ? std::max(1, m_mineAutoStepsPerFrame) : 0;

  if (manualRemaining <= 0 && autoRemaining <= 0) return;

  const double budgetMs = std::max(0.0, static_cast<double>(m_mineAutoBudgetMs));
  // If no budget is configured, enforce a conservative hard cap to avoid long stalls.
  const int hardCap = (budgetMs <= 0.0) ? 256 : 1000000000;

  const auto t0 = std::chrono::steady_clock::now();

  int producedTotal = 0;
  int iterations = 0;
  while ((manualRemaining > 0 || autoRemaining > 0) && !m_mineSession->done()) {
    if (iterations >= hardCap) break;

    const bool consumeManual = (manualRemaining > 0);
    const int produced = m_mineSession->step(1);
    if (produced <= 0) break;

    producedTotal += produced;
    iterations += produced;

    if (consumeManual) {
      manualRemaining -= produced;
    } else {
      autoRemaining -= produced;
    }

    if (budgetMs > 0.0) {
      const auto tNow = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double, std::milli>(tNow - t0).count();
      if (elapsed >= budgetMs) break;
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  m_mineLastAutoStepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // Commit remaining manual requests.
  m_mineManualStepRequests = std::max(0, manualRemaining);

  if (m_mineSession->done()) {
    m_mineAutoRun = false;
    m_mineManualStepRequests = 0;
    m_console.print("mine: complete. use 'mine top' to list results");
    showToast(TextFormat("Seed mining complete (%d samples)", m_mineSession->total()), 2.5f);
  }
}

void Game::clearHistory()
{
  m_cityHistory.clear();
  m_tickStatsHistory.clear();
  clearCityNews();
}

void Game::recordHistorySample(const Stats& s)
{
  // Avoid recording duplicate days (can happen when resetting/loading).
  if (!m_tickStatsHistory.empty() && m_tickStatsHistory.back().day == s.day) return;

  CityHistorySample hs{};
  hs.day = s.day;
  hs.population = s.population;
  hs.money = s.money;
  hs.happiness = s.happiness;
  hs.demandResidential = s.demandResidential;
  hs.demandCommercial = s.demandCommercial;
  hs.demandIndustrial = s.demandIndustrial;
  hs.avgLandValue = s.avgLandValue;
  hs.avgTaxPerCapita = s.avgTaxPerCapita;
  hs.income = s.income;
  hs.expenses = s.expenses;
  hs.taxRevenue = s.taxRevenue;
  hs.maintenanceCost = s.maintenanceCost;
  hs.commuters = s.commuters;
  hs.avgCommute = s.avgCommute;
  hs.avgCommuteTime = s.avgCommuteTime;
  hs.trafficCongestion = s.trafficCongestion;
  hs.goodsSatisfaction = s.goodsSatisfaction;

  m_cityHistory.push_back(hs);
  m_tickStatsHistory.push_back(s);

  // Keep a bounded history window (simple ring behavior).
  const int maxDays = std::max(16, m_cityHistoryMax);
  while (static_cast<int>(m_cityHistory.size()) > maxDays) {
    m_cityHistory.erase(m_cityHistory.begin());
  }
  while (static_cast<int>(m_tickStatsHistory.size()) > maxDays) {
    m_tickStatsHistory.erase(m_tickStatsHistory.begin());
  }

  // Also append a City News headline for this (new) day.
  recordCityNews(s);
}


namespace {

enum class NewsKind : std::uint8_t {
  Welcome = 0,
  Quiet,
  Growth,
  Decline,
  Milestone,
  BudgetSurplus,
  BudgetDeficit,
  Traffic,
  Services,
  FireRisk,
  FireIncident,
  AirPollution,
  RunoffPollution,
  Goods,
  Trade,
  EconomyEvent,
  DemandShift,
};

struct NewsCandidate {
  NewsKind kind;
  float score;
};

inline float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline int Pct100(float v) { return static_cast<int>(std::round(static_cast<double>(Clamp01(v) * 100.0f))); }

inline const char* ToneTag(CityNewsTone t)
{
  switch (t) {
    case CityNewsTone::Good: return "GOOD";
    case CityNewsTone::Neutral: return "INFO";
    case CityNewsTone::Bad: return "WARN";
    case CityNewsTone::Alert: return "ALERT";
  }
  return "INFO";
}

inline std::string CityMoodLabel(float rating)
{
  if (rating >= 80.0f) return "Booming";
  if (rating >= 65.0f) return "Content";
  if (rating >= 50.0f) return "Uneasy";
  if (rating >= 35.0f) return "Angry";
  return "Crisis";
}

inline float ComputeMayorRating01(const Stats& s)
{
  // Weighted blend of a few citywide indicators. This is intentionally heuristic ("SimCity-ish"),
  // not a hard simulation output.
  const float happiness = Clamp01(s.happiness);
  const float services = Clamp01(s.servicesOverallSatisfaction);
  const float goods = Clamp01(s.goodsSatisfaction);
  const float trafficOk = 1.0f - Clamp01(s.trafficCongestion);
  const float land = Clamp01(s.avgLandValue);

  const float net = static_cast<float>(s.income - s.expenses);
  const float netScore = Clamp01((net + 250.0f) / 500.0f);  // net -250..+250 maps to 0..1

  const float money = static_cast<float>(s.money);
  const float moneyScore = Clamp01((money + 200.0f) / 1200.0f);  // -200..+1000 maps to 0..1

  float rating = 0.42f * happiness + 0.16f * services + 0.10f * goods + 0.12f * trafficOk + 0.08f * land +
                 0.06f * netScore + 0.06f * moneyScore;
  return Clamp01(rating);
}

inline CityNewsTone ToneForEconomyEvent(EconomyEventKind k)
{
  switch (k) {
    case EconomyEventKind::ExportBoom:
    case EconomyEventKind::TechBoom:
    case EconomyEventKind::TourismSurge:
      return CityNewsTone::Good;
    case EconomyEventKind::Recession:
    case EconomyEventKind::FuelSpike:
    case EconomyEventKind::ImportShock:
      return CityNewsTone::Bad;
    default:
      return CityNewsTone::Neutral;
  }
}

inline std::string PrettyEconomyEvent(EconomyEventKind k)
{
  switch (k) {
    case EconomyEventKind::Recession: return "Recession";
    case EconomyEventKind::FuelSpike: return "Fuel Spike";
    case EconomyEventKind::ImportShock: return "Import Shock";
    case EconomyEventKind::ExportBoom: return "Export Boom";
    case EconomyEventKind::TechBoom: return "Tech Boom";
    case EconomyEventKind::TourismSurge: return "Tourism Surge";
    default: return "Economic Shift";
  }
}

inline std::string JsonEscape(const std::string& s)
{
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned int>(static_cast<unsigned char>(c)));
          out += buf;
        } else {
          out.push_back(c);
        }
        break;
    }
  }
  return out;
}

struct DistrictExposureSummary {
  int population = 0;      // total residents included
  int tiles = 0;           // residential tiles with occupants > 0
  float avg01 = 0.0f;      // population-weighted mean
  float highFrac = 0.0f;   // share of residents above threshold
  int highPopulation = 0;  // residents above threshold

  int worstDistrict = 0;
  float worstAvg01 = 0.0f; // population-weighted mean in worst district
  int worstPopulation = 0; // residents in worst district
};

inline DistrictExposureSummary ComputeResidentialExposureByDistrict(const World& world,
                                                                   const std::vector<float>& field01,
                                                                   float highThreshold01)
{
  DistrictExposureSummary out;
  const int w = world.width();
  const int h = world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));
  if (w <= 0 || h <= 0) return out;
  if (field01.size() != n) return out;

  std::array<double, kDistrictCount> sum{};
  std::array<int, kDistrictCount> pop{};
  std::array<int, kDistrictCount> highPop{};

  double citySum = 0.0;
  int cityPop = 0;
  int cityHighPop = 0;
  int tiles = 0;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i = static_cast<std::size_t>(y * w + x);
      const Tile& t = world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants <= 0) continue;
      ++tiles;

      const int p = t.occupants;
      const float v = Clamp01(field01[i]);
      citySum += static_cast<double>(v) * static_cast<double>(p);
      cityPop += p;
      if (v >= highThreshold01) cityHighPop += p;

      const int d = static_cast<int>(t.district) % kDistrictCount;
      sum[d] += static_cast<double>(v) * static_cast<double>(p);
      pop[d] += p;
      if (v >= highThreshold01) highPop[d] += p;
    }
  }

  out.tiles = tiles;
  out.population = cityPop;
  out.highPopulation = cityHighPop;
  if (cityPop > 0) {
    out.avg01 = static_cast<float>(citySum / static_cast<double>(cityPop));
    out.highFrac = static_cast<float>(static_cast<double>(cityHighPop) / static_cast<double>(cityPop));
  }

  // Worst district by population-weighted mean exposure.
  int worstD = 0;
  double worstAvg = -1.0;
  for (int d = 0; d < kDistrictCount; ++d) {
    if (pop[d] <= 0) continue;
    const double a = sum[d] / static_cast<double>(pop[d]);
    if (a > worstAvg) {
      worstAvg = a;
      worstD = d;
    }
  }
  if (worstAvg >= 0.0) {
    out.worstDistrict = worstD;
    out.worstAvg01 = static_cast<float>(worstAvg);
    out.worstPopulation = pop[worstD];
  }

  return out;
}

} // namespace

void Game::clearCityNews()
{
  m_cityNews.clear();
  m_newsSelection = 0;
  m_newsFirst = 0;
  m_mayorRatingEma = 50.0f;
  m_mayorRatingPrev = 50.0f;
  m_newsEnvLastComputeSec = -1.0;
  m_newsEnvLastComputeDay = -1;
}

void Game::recordCityNews(const Stats& s)
{
  if (!m_cityNews.empty() && m_cityNews.back().day == s.day) return;

  const std::string cityName = GenerateCityName(m_world.seed());

  // Update (smoothed) mayor rating.
  const float rawRating = ComputeMayorRating01(s) * 100.0f;
  if (m_cityNews.empty()) {
    m_mayorRatingEma = rawRating;
    m_mayorRatingPrev = rawRating;
  } else {
    m_mayorRatingPrev = m_mayorRatingEma;
    constexpr float alpha = 0.35f;
    m_mayorRatingEma = (1.0f - alpha) * m_mayorRatingEma + alpha * rawRating;
  }

  const float rating = std::clamp(m_mayorRatingEma, 0.0f, 100.0f);
  const std::string mood = CityMoodLabel(rating);

  const Stats* prev = nullptr;
  if (m_tickStatsHistory.size() >= 2) {
    prev = &m_tickStatsHistory[m_tickStatsHistory.size() - 2];
  }

  // Deterministic RNG: seed + day (stable across reloads).
  const std::uint64_t newsSeed = static_cast<std::uint64_t>(m_world.seed()) ^ (static_cast<std::uint64_t>(s.day) * 0x9E3779B97F4A7C15ull);
  RNG rng(newsSeed);

  CityNewsEntry e;
  e.day = s.day;
  e.mayorRating = rating;

  // First day / no baseline: welcome headline.
  if (!prev) {
    e.tone = CityNewsTone::Neutral;
    e.headline = TextFormat("Welcome to %s", cityName.c_str());
    e.body = TextFormat(
      "A new city charter is signed and the first roads are surveyed.\n"
      "Tip: 1=Road, 2/3/4=RCI zones, 5=Parks. Watch demand (RCI) in the Report panel (F1).\n"
      "Ctrl+O opens City Challenges - short goals that nudge you toward solving real city problems.\n"
      "Mayor approval: %.0f%% (%s).",
      static_cast<double>(rating), mood.c_str());

    // Inject any addendum (e.g., challenge bulletins).
    if (auto it = m_cityNewsAddendum.find(s.day); it != m_cityNewsAddendum.end()) {
      if (!it->second.empty()) {
        e.body += "\n\n";
        e.body += it->second;
      }
      m_cityNewsAddendum.erase(it);
    }

    m_cityNews.push_back(std::move(e));
    m_newsSelection = static_cast<int>(m_cityNews.size()) - 1;
    return;
  }

  // --- Story scoring (pick the strongest "headline" of the day). ---
  std::vector<NewsCandidate> cands;
  cands.reserve(18);

  auto add = [&](NewsKind k, float score) {
    if (score > 0.0f) cands.push_back(NewsCandidate{k, score});
  };

  const int popDelta = s.population - prev->population;
  const int net = s.income - s.expenses;

  // A real disaster should dominate the day's headline.
  const int fireIncidentTotal = s.fireIncidentDamaged + s.fireIncidentDestroyed;
  if (fireIncidentTotal > 0) {
    // Score is intentionally much larger than other stories.
    add(NewsKind::FireIncident, 12.0f + 0.04f * static_cast<float>(fireIncidentTotal));
  }

  const float traffic = Clamp01(s.trafficCongestion);
  const float services = Clamp01(s.servicesOverallSatisfaction);
  const float goods = Clamp01(s.goodsSatisfaction);

  // Fire risk score (heuristic): high in dense, contiguous areas with weak fire station response.
  float fireAvgRisk = 0.0f;
  float fireWorstDistrictRisk = 0.0f;
  int fireWorstDistrict = 0;
  int fireZoneCount = 0;
  if (s.population > 0) {
    ensureFireRiskUpToDate();
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));
    if (m_fireRisk.risk01.size() == n) {
      std::array<float, kDistrictCount> sum{};
      std::array<int, kDistrictCount> cnt{};
      float tot = 0.0f;
      int totN = 0;
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t i = static_cast<std::size_t>(y * w + x);
          const Tile& t = m_world.at(x, y);
          if (!IsZoneOverlay(t.overlay)) continue;
          const float r = m_fireRisk.risk01[i];
          tot += r;
          ++totN;
          const int d = static_cast<int>(t.district) % kDistrictCount;
          sum[d] += r;
          cnt[d] += 1;
        }
      }
      fireZoneCount = totN;
      if (totN > 0) {
        fireAvgRisk = tot / static_cast<float>(totN);
        // Worst district average.
        for (int d = 0; d < kDistrictCount; ++d) {
          if (cnt[d] <= 0) continue;
          const float avg = sum[d] / static_cast<float>(cnt[d]);
          if (avg > fireWorstDistrictRisk) {
            fireWorstDistrictRisk = avg;
            fireWorstDistrict = d;
          }
        }
      }
    }
  }

  // --- Environmental advisor snapshots (air + runoff), computed opportunistically ---
  // These layers are intentionally recomputed on a small wall-clock cooldown so
  // the City News feed doesn't stall the sim when running at high speed.
  DistrictExposureSummary airExp{};
  DistrictExposureSummary runoffExp{};
  {
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));
    const bool wantEnv = (s.population >= 60);

    if (wantEnv && w > 0 && h > 0) {
      const double now = GetTime();
      constexpr double kEnvCooldownSec = 0.45;
      const bool cooldownOk = (m_newsEnvLastComputeSec < 0.0) || ((now - m_newsEnvLastComputeSec) >= kEnvCooldownSec);

      const bool haveAir = (m_airPollution.pollution01.size() == n);
      const bool haveRunoff = (m_runoff.pollution01.size() == n);

      if (cooldownOk && (m_airPollutionDirty || m_runoffDirty || !haveAir || !haveRunoff)) {
        // Use the most recent traffic/goods caches if available (even if marked dirty):
        // they are cheap to reuse and provide a more informative spatial pattern.
        const TrafficResult* tptr = (!m_traffic.roadTraffic.empty() && m_traffic.roadTraffic.size() == n) ? &m_traffic : nullptr;
        const GoodsResult* gptr = (!m_goods.roadGoodsTraffic.empty() && m_goods.roadGoodsTraffic.size() == n) ? &m_goods : nullptr;

        // Air pollution.
        if (!haveAir || m_airPollutionDirty) {
          m_airPollution = ComputeAirPollution(m_world, m_airPollutionCfg, tptr, gptr);
          m_airPollutionDirty = false;
        }

        // Runoff pollution.
        if (!haveRunoff || m_runoffDirty) {
          m_runoff = ComputeRunoffPollution(m_world, m_runoffCfg, tptr);
          m_runoffDirty = false;
        }

        m_newsEnvLastComputeSec = now;
        m_newsEnvLastComputeDay = s.day;
      }

      if (m_airPollution.pollution01.size() == n) {
        airExp = ComputeResidentialExposureByDistrict(m_world, m_airPollution.pollution01, m_airPollution.highExposureThreshold);
      }
      if (m_runoff.pollution01.size() == n) {
        runoffExp = ComputeResidentialExposureByDistrict(m_world, m_runoff.pollution01, m_runoff.cfg.highExposureThreshold01);
      }
    }
  }

  // Economy events (start-of-event gets a big boost so it headlines immediately).
  if (s.economyEventKind != 0) {
    const bool newEvent = (prev->economyEventKind != s.economyEventKind);
    add(NewsKind::EconomyEvent, newEvent ? 3.2f : 1.6f);
  }

  // Trade health.
  if (s.tradeImportDisrupted || s.tradeExportDisrupted) {
    add(NewsKind::Trade, 2.6f);
  } else {
    const float importScore = (s.tradeImportCapacityPct < 85) ? (0.8f + (85 - s.tradeImportCapacityPct) / 40.0f) : 0.0f;
    const float exportScore = (s.tradeExportCapacityPct < 85) ? (0.8f + (85 - s.tradeExportCapacityPct) / 40.0f) : 0.0f;
    add(NewsKind::Trade, std::max(importScore, exportScore));
  }

  // Budget.
  if (net < -80) {
    add(NewsKind::BudgetDeficit, 1.9f + static_cast<float>(-net) / 400.0f);
  } else if (net > 120) {
    add(NewsKind::BudgetSurplus, 1.2f + static_cast<float>(net) / 600.0f);
  }

  // Traffic.
  if (traffic > 0.55f) {
    add(NewsKind::Traffic, 1.0f + traffic);
  } else if (traffic > 0.35f) {
    add(NewsKind::Traffic, 0.6f + traffic);
  }

  // Fire risk (hazard advisory).
  if (fireZoneCount > 0 && fireAvgRisk > 0.60f) {
    const float highFrac = (fireZoneCount > 0) ? (static_cast<float>(m_fireRisk.highRiskZones) / static_cast<float>(fireZoneCount)) : 0.0f;
    float score = 0.85f + (fireAvgRisk - 0.60f) * 3.0f;
    score += highFrac * 1.6f;
    if (fireWorstDistrictRisk > 0.75f) score += 0.35f;
    add(NewsKind::FireRisk, score);
  }

  // Air quality advisory (derived from the air pollution layer).
  if (airExp.population > 0) {
    const float avg = Clamp01(airExp.avg01);
    const float high = Clamp01(airExp.highFrac);
    if (avg > 0.52f || high > 0.18f) {
      float score = 0.80f + (avg - 0.52f) * 3.0f + high * 2.2f;
      score += Clamp01(static_cast<float>(airExp.population) / 1200.0f) * 0.35f;
      if (airExp.worstAvg01 > 0.72f) score += 0.35f;
      add(NewsKind::AirPollution, score);
    } else if (avg > 0.44f && high > 0.10f) {
      const float score = 0.55f + (avg - 0.44f) * 2.5f + high * 1.6f;
      add(NewsKind::AirPollution, score);
    }
  }

  // Runoff / stormwater pollution advisory.
  if (runoffExp.population > 0) {
    const float avg = Clamp01(runoffExp.avg01);
    const float high = Clamp01(runoffExp.highFrac);
    if (avg > 0.52f || high > 0.18f) {
      float score = 0.75f + (avg - 0.52f) * 2.8f + high * 2.0f;
      score += Clamp01(static_cast<float>(runoffExp.population) / 1200.0f) * 0.30f;
      if (runoffExp.worstAvg01 > 0.72f) score += 0.30f;
      add(NewsKind::RunoffPollution, score);
    } else if (avg > 0.44f && high > 0.10f) {
      const float score = 0.50f + (avg - 0.44f) * 2.2f + high * 1.4f;
      add(NewsKind::RunoffPollution, score);
    }
  }

  // Services / goods.
  if (services < 0.55f) {
    add(NewsKind::Services, 0.9f + (0.55f - services) * 2.0f);
  }
  if (goods < 0.75f) {
    add(NewsKind::Goods, 0.8f + (0.75f - goods) * 2.0f);
  }

  // Population.
  if (popDelta > 25) {
    add(NewsKind::Growth, 0.7f + static_cast<float>(popDelta) / 250.0f);
  }
  if (popDelta < -10) {
    add(NewsKind::Decline, 1.3f + static_cast<float>(-popDelta) / 120.0f);
  }

  // Milestones.
  const int prevMilestone = std::max(0, prev->population / 500);
  const int curMilestone = std::max(0, s.population / 500);
  if (curMilestone > prevMilestone && s.population >= 500) {
    add(NewsKind::Milestone, 1.1f + 0.05f * static_cast<float>(curMilestone));
  }

  // Demand shifts.
  const float dR = s.demandResidential - prev->demandResidential;
  const float dC = s.demandCommercial - prev->demandCommercial;
  const float dI = s.demandIndustrial - prev->demandIndustrial;
  const float maxAbs = std::max({std::fabs(dR), std::fabs(dC), std::fabs(dI)});
  if (maxAbs > 0.22f) {
    add(NewsKind::DemandShift, 0.9f + maxAbs);
  }

  // Fallback.
  if (cands.empty()) {
    add(NewsKind::Quiet, 0.1f);
  }

  // Pick best by score; if tied, choose deterministically.
  float bestScore = -1.0f;
  for (const auto& c : cands) bestScore = std::max(bestScore, c.score);

  std::vector<NewsKind> best;
  for (const auto& c : cands) {
    if (c.score >= bestScore - 1.0e-4f) best.push_back(c.kind);
  }
  const NewsKind kind = best.empty() ? NewsKind::Quiet : best[static_cast<std::size_t>(rng.rangeU32(best.size()))];

  auto appendMayorLine = [&]() {
    e.body += TextFormat("\nMayor approval: %.0f%% (%s).", static_cast<double>(rating), mood.c_str());
  };

  // --- Text generation (headline + body + a small actionable tip). ---
  switch (kind) {
    case NewsKind::Quiet: {
      e.tone = CityNewsTone::Neutral;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = TextFormat("A quiet day in %s", cityName.c_str());
      else if (pick == 1) e.headline = "City hall reports steady progress";
      else e.headline = "No major incidents reported";

      e.body = TextFormat(
        "Population: %d (delta %+d) | Money: %d | Happiness: %d%%\n"
        "Tip: use L to cycle heatmaps, and F1 for trends (RCI, money, congestion).",
        s.population, popDelta, s.money, Pct100(s.happiness));
      appendMayorLine();
    } break;

    case NewsKind::Growth: {
      e.tone = CityNewsTone::Good;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = TextFormat("%s grows: population reaches %d", cityName.c_str(), s.population);
      else if (pick == 1) e.headline = TextFormat("Newcomers arrive: %+d residents today", popDelta);
      else e.headline = TextFormat("Housing demand climbs as city hits %d residents", s.population);

      e.body = TextFormat(
        "Population change: %+d\n"
        "RCI demand: R %d%% | C %d%% | I %d%%\n"
        "Tip: keep roads connected and balance R/C/I to sustain growth.",
        popDelta, Pct100(s.demandResidential), Pct100(s.demandCommercial), Pct100(s.demandIndustrial));
      appendMayorLine();
    } break;

    case NewsKind::Decline: {
      e.tone = CityNewsTone::Bad;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = "Residents leave in search of opportunity";
      else if (pick == 1) e.headline = TextFormat("Population dips: %+d today", popDelta);
      else e.headline = "City growth stalls amid citizen concerns";

      e.body = TextFormat(
        "Population change: %+d\n"
        "Happiness: %d%% | Services: %d%% | Goods: %d%%\n"
        "Tip: address the loudest complaints first (traffic, services, goods shortages).",
        popDelta, Pct100(s.happiness), Pct100(s.servicesOverallSatisfaction), Pct100(s.goodsSatisfaction));
      appendMayorLine();
    } break;

    case NewsKind::Milestone: {
      e.tone = CityNewsTone::Good;
      const int milestone = (s.population / 500) * 500;
      e.headline = TextFormat("Milestone reached: %d residents", milestone);
      e.body = TextFormat(
        "%s crosses a new milestone as the population reaches %d.\n"
        "Tip: as density rises, parks and services become more important.",
        cityName.c_str(), s.population);
      appendMayorLine();
    } break;

    case NewsKind::BudgetSurplus: {
      e.tone = CityNewsTone::Good;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = "Treasury reports a budget surplus";
      else if (pick == 1) e.headline = TextFormat("Income beats expenses by %+d today", net);
      else e.headline = "City coffers grow as revenues rise";

      e.body = TextFormat(
        "Income: %d | Expenses: %d | Net: %+d | Money: %d\n"
        "Tip: invest surplus into roads, parks, and services to boost land value.",
        s.income, s.expenses, net, s.money);
      appendMayorLine();
    } break;

    case NewsKind::BudgetDeficit: {
      e.tone = (net < -250 || s.money < 0) ? CityNewsTone::Alert : CityNewsTone::Bad;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = "Budget warning: expenses outpace income";
      else if (pick == 1) e.headline = TextFormat("City runs a deficit: %+d today", net);
      else e.headline = "Council debates cuts amid shrinking treasury";

      e.body = TextFormat(
        "Income: %d | Expenses: %d | Net: %+d | Money: %d\n"
        "Tip: open Policy (P) to adjust taxes/maintenance. The Policy Advisor can auto-tune trade-offs.",
        s.income, s.expenses, net, s.money);
      appendMayorLine();
    } break;

    case NewsKind::Traffic: {
      const int congPct = Pct100(s.trafficCongestion);
      e.tone = (congPct >= 70) ? CityNewsTone::Alert : CityNewsTone::Bad;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = TextFormat("Traffic snarls: congestion hits %d%%", congPct);
      else if (pick == 1) e.headline = "Commuters complain about gridlock";
      else e.headline = TextFormat("Congestion rising on major roads (%d%%)", congPct);

      e.body = TextFormat(
        "Avg commute time: %.1f | Congestion: %d%%\n"
        "Tip: Ctrl+T opens transit planning. Ctrl+U suggests road upgrades. More connections reduce chokepoints.",
        static_cast<double>(s.avgCommuteTime), congPct);
      appendMayorLine();
    } break;

    case NewsKind::Services: {
      const int svcPct = Pct100(s.servicesOverallSatisfaction);
      e.tone = (svcPct < 35) ? CityNewsTone::Alert : CityNewsTone::Bad;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = "Residents demand better public services";
      else if (pick == 1) e.headline = TextFormat("Service coverage concerns (%d%% satisfied)", svcPct);
      else e.headline = "City services stretched thin";

      e.body = TextFormat(
        "Overall service satisfaction: %d%%\n"
        "Tip: cycle heatmaps with L to find gaps, then place schools/hospitals/police/fire stations near dense zones.",
        svcPct);
      appendMayorLine();
    } break;

    case NewsKind::FireRisk: {
      // This is a heuristic hazard advisory computed from:
      //  - development density/contiguity
      //  - travel-time response coverage from fire stations
      ensureFireRiskUpToDate();

      const int avgPct = static_cast<int>(std::round(fireAvgRisk * 100.0f));
      const int worstPct = static_cast<int>(std::round(fireWorstDistrictRisk * 100.0f));
      const int pick = static_cast<int>(rng.rangeU32(3));

      // Tone based on severity.
      if (avgPct >= 70 || worstPct >= 80) e.tone = CityNewsTone::Alert;
      else e.tone = CityNewsTone::Bad;

      if (pick == 0) {
        e.headline = TextFormat("Fire safety advisory: District %d flagged", fireWorstDistrict);
      } else if (pick == 1) {
        e.headline = TextFormat("Fire risk rising (%d%% citywide)", avgPct);
      } else {
        e.headline = "Fire Chief urges better coverage";
      }

      // Count stations for context.
      int stations = 0;
      for (int y = 0; y < m_world.height(); ++y) {
        for (int x = 0; x < m_world.width(); ++x) {
          if (m_world.getTile(x, y).overlay == Overlay::FireStation) {
            ++stations;
          }
        }
      }

      e.body = TextFormat(
        "Estimated fire risk (avg): %d%% | High-risk zones: %d | Fire stations: %d\n"
        "Worst district: %d (%d%%)\n"
        "Tip: press L until 'Fire risk' appears, then add/upgrade Fire Stations near dense industrial/commercial blocks. "
        "Roads and parks can act as simple firebreaks.",
        avgPct, m_fireRisk.highRiskZones, stations, fireWorstDistrict, worstPct);
      appendMayorLine();
    } break;

    case NewsKind::FireIncident: {
      e.tone = CityNewsTone::Alert;

      const int damaged = s.fireIncidentDamaged;
      const int destroyed = s.fireIncidentDestroyed;
      const int total = damaged + destroyed;
      const int displaced = s.fireIncidentDisplaced;
      const int cost = s.fireIncidentCost;
      const int d = s.fireIncidentDistrict;

      const int pick = static_cast<int>(rng.rangeU32(3));
      if (d >= 0) {
        if (pick == 0) e.headline = TextFormat("Fire breaks out in District %d", d);
        else if (pick == 1) e.headline = TextFormat("Major fire damages %d buildings (District %d)", total, d);
        else e.headline = TextFormat("Fire response teams deployed to District %d", d);
      } else {
        if (pick == 0) e.headline = "Major fire reported";
        else if (pick == 1) e.headline = TextFormat("Fire damages %d buildings", total);
        else e.headline = "Fire crews respond to city incident";
      }

      // Context: how prepared are we?
      int stations = 0;
      for (int y = 0; y < m_world.height(); ++y) {
        for (int x = 0; x < m_world.width(); ++x) {
          if (m_world.getTile(x, y).overlay == Overlay::FireStation) ++stations;
        }
      }

      e.body = TextFormat(
        "Buildings damaged: %d | Destroyed: %d | Displaced residents: %d\n"
        "Estimated response cost: %d | Fire stations: %d\n"
        "Tip: press L until 'Fire risk' appears, then strengthen coverage near dense industrial/commercial blocks. "
        "Upgrading/adding Fire Stations and improving road connectivity can reduce future losses.",
        damaged, destroyed, displaced, cost, stations);
      appendMayorLine();
    } break;

    case NewsKind::AirPollution: {
      const int avgPct = Pct100(airExp.avg01);
      const int worstPct = Pct100(airExp.worstAvg01);
      const int highPct = static_cast<int>(std::round(static_cast<double>(Clamp01(airExp.highFrac)) * 100.0));

      if (avgPct >= 65 || highPct >= 25) e.tone = CityNewsTone::Alert;
      else e.tone = CityNewsTone::Bad;

      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) {
        e.headline = TextFormat("Air quality advisory: District %d under watch", airExp.worstDistrict);
      } else if (pick == 1) {
        e.headline = TextFormat("Smog concerns rise (%d%% residential exposure)", avgPct);
      } else {
        e.headline = "Residents call for cleaner air";
      }

      WindDir wdir = m_airPollution.cfg.windFromSeed ? InferWindDirFromSeed(m_world.seed()) : m_airPollution.cfg.fixedWindDir;
      const char* windName = WindDirName(wdir);

      e.body = TextFormat(
        "Residential exposure (avg): %d%% | High exposure: %d residents (%d%%)\n"
        "Worst district: %d (%d%%) | Wind: %s\n"
        "Tip: press L until 'Air pollution' / 'Air emission' appears. Add parks/water buffers near busy roads, keep heavy industry away from housing, and reduce congestion with better connections or Ctrl+U upgrades.",
        avgPct, airExp.highPopulation, highPct,
        airExp.worstDistrict, worstPct, windName);
      appendMayorLine();
    } break;

    case NewsKind::RunoffPollution: {
      const int avgPct = Pct100(runoffExp.avg01);
      const int worstPct = Pct100(runoffExp.worstAvg01);
      const int highPct = static_cast<int>(std::round(static_cast<double>(Clamp01(runoffExp.highFrac)) * 100.0));

      if (avgPct >= 65 || highPct >= 25) e.tone = CityNewsTone::Alert;
      else e.tone = CityNewsTone::Bad;

      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) {
        e.headline = TextFormat("Stormwater warning: District %d flagged", runoffExp.worstDistrict);
      } else if (pick == 1) {
        e.headline = TextFormat("Runoff pollution rising (%d%% residential exposure)", avgPct);
      } else {
        e.headline = "Creeks run dirty after storms";
      }

      e.body = TextFormat(
        "Residential runoff exposure (avg): %d%% | High exposure: %d residents (%d%%)\n"
        "Worst district: %d (%d%%)\n"
        "Tip: press L until 'Runoff pollution' or 'Mitigation plan' appears. Parks and grass filter runoff; prioritize green buffers downhill from roads and industry.",
        avgPct, runoffExp.highPopulation, highPct,
        runoffExp.worstDistrict, worstPct);
      appendMayorLine();
    } break;

    case NewsKind::Goods: {
      const int gPct = Pct100(s.goodsSatisfaction);
      e.tone = (gPct < 50) ? CityNewsTone::Alert : CityNewsTone::Bad;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = "Shopkeepers report empty shelves";
      else if (pick == 1) e.headline = TextFormat("Goods shortages reported (%d%% satisfied)", gPct);
      else e.headline = "Supply concerns ripple through the city";

      e.body = TextFormat(
        "Goods satisfaction: %d%% | Import cap: %d%% | Export cap: %d%%\n"
        "Tip: expand industrial capacity, improve road connectivity, and keep trade routes healthy.",
        gPct, s.tradeImportCapacityPct, s.tradeExportCapacityPct);
      appendMayorLine();
    } break;

    case NewsKind::Trade: {
      const bool impD = s.tradeImportDisrupted;
      const bool expD = s.tradeExportDisrupted;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (impD && expD) e.headline = "Trade routes disrupted";
      else if (impD) e.headline = "Imports disrupted; merchants seek alternatives";
      else if (expD) e.headline = "Exports disrupted; industry feels the pinch";
      else if (pick == 0) e.headline = "Trade capacity strained";
      else if (pick == 1) e.headline = "Freight delays reported";
      else e.headline = "Markets react to shifting trade flows";

      e.tone = (impD || expD) ? CityNewsTone::Bad : CityNewsTone::Neutral;
      if ((impD || expD) && (s.tradeImportCapacityPct < 60 || s.tradeExportCapacityPct < 60)) {
        e.tone = CityNewsTone::Alert;
      }

      e.body = TextFormat(
        "Imports: %d%% cap (%s) | Exports: %d%% cap (%s)\n"
        "Market index: %.2f | Trade zones: %d\n"
        "Tip: ensure a road network reaches the map edge (outside connection), and avoid chokepoints.",
        s.tradeImportCapacityPct, impD ? "DISRUPTED" : "ok",
        s.tradeExportCapacityPct, expD ? "DISRUPTED" : "ok",
        static_cast<double>(s.tradeMarketIndex), s.tradeZoneTiles);
      appendMayorLine();
    } break;

    case NewsKind::EconomyEvent: {
      const EconomyEventKind ek = static_cast<EconomyEventKind>(s.economyEventKind);
      const std::string evName = PrettyEconomyEvent(ek);
      const CityNewsTone baseTone = ToneForEconomyEvent(ek);

      e.tone = baseTone;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = TextFormat("Regional economy: %s", evName.c_str());
      else if (pick == 1) e.headline = TextFormat("Economic bulletin: %s", evName.c_str());
      else e.headline = TextFormat("Markets shift amid %s", evName.c_str());

      // Escalate if the event is long or inflation is high.
      if (baseTone == CityNewsTone::Bad && (s.economyEventDaysLeft > 6 || s.economyInflation > 0.06f)) {
        e.tone = CityNewsTone::Alert;
      }

      e.body = TextFormat(
        "Event: %s (%d days left)\n"
        "Inflation: %.1f%% | Tax base: x%.2f | Import cost: x%.2f\n"
        "Tip: keep an eye on goods and budget; use Policy (P) to soften shocks or capture booms.",
        EconomyEventKindName(ek), s.economyEventDaysLeft,
        static_cast<double>(s.economyInflation * 100.0f),
        static_cast<double>(s.economyTaxBaseMult),
        static_cast<double>(s.economyImportCostMult));
      appendMayorLine();
    } break;

    case NewsKind::DemandShift: {
      // Find the biggest absolute shift.
      const float adR = std::fabs(dR);
      const float adC = std::fabs(dC);
      const float adI = std::fabs(dI);
      const char* bestName = "Residential";
      float bestD = adR;
      float bestDelta = dR;
      if (adC > bestD) { bestD = adC; bestDelta = dC; bestName = "Commercial"; }
      if (adI > bestD) { bestD = adI; bestDelta = dI; bestName = "Industrial"; }

      e.tone = CityNewsTone::Neutral;
      const int pick = static_cast<int>(rng.rangeU32(3));
      if (pick == 0) e.headline = TextFormat("Zoning winds shift: %s demand %+d%%", bestName, static_cast<int>(std::round(bestDelta * 100.0f)));
      else if (pick == 1) e.headline = "Developers eye new opportunities";
      else e.headline = "Demand patterns shift across the city";

      e.body = TextFormat(
        "Demand now: R %d%% | C %d%% | I %d%%\n"
        "Largest move: %s (%+d%%)\n"
        "Tip: respond gradually - overzoning one category can create long-term imbalance.",
        Pct100(s.demandResidential), Pct100(s.demandCommercial), Pct100(s.demandIndustrial),
        bestName, static_cast<int>(std::round(bestDelta * 100.0f)));
      appendMayorLine();
    } break;

    default: {
      e.tone = CityNewsTone::Neutral;
      e.headline = "City bulletin";
      e.body = "No details.";
      appendMayorLine();
    } break;
  }

  // Inject any addendum (e.g., challenge bulletins).
  if (auto it = m_cityNewsAddendum.find(s.day); it != m_cityNewsAddendum.end()) {
    if (!it->second.empty()) {
      e.body += "\n\n";
      e.body += it->second;
    }
    m_cityNewsAddendum.erase(it);
  }

  // --- Commit ---
  const int countBefore = static_cast<int>(m_cityNews.size());
  const bool followTail = (countBefore <= 0) ? true : (m_newsSelection >= countBefore - 1);

  m_cityNews.push_back(std::move(e));

  const int maxEntries = std::max(16, m_cityNewsMax);
  while (static_cast<int>(m_cityNews.size()) > maxEntries) {
    m_cityNews.pop_front();
    if (m_newsSelection > 0) --m_newsSelection;
    if (m_newsFirst > 0) --m_newsFirst;
  }

  if (followTail) {
    m_newsSelection = static_cast<int>(m_cityNews.size()) - 1;
  }
  m_newsSelection = std::clamp(m_newsSelection, 0, std::max(0, static_cast<int>(m_cityNews.size()) - 1));

  // Pop a toast for major alerts or for a brand-new economy event.
  const bool newEventToast = (prev && prev->economyEventKind != s.economyEventKind && s.economyEventKind != 0);
  if (m_cityNews.back().tone == CityNewsTone::Alert || newEventToast) {
    showToast(m_cityNews.back().headline, 3.5f);
  }
}

void Game::exportCityNews()
{
  namespace fs = std::filesystem;

  if (m_cityNews.empty()) {
    showToast("City News: nothing to export");
    return;
  }

  std::error_code ec;
  fs::create_directories(fs::path("captures"), ec);

  const std::string stamp = FileTimestamp();
  const std::string base = TextFormat("captures/city_news_seed%llu_%s", static_cast<unsigned long long>(m_world.seed()), stamp.c_str());

  const std::string cityName = GenerateCityName(m_world.seed());

  // JSON export
  {
    const std::string path = base + ".json";
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) {
      showToast("City News export failed (json)");
    } else {
      f << "{\n";
      f << "  \"seed\": " << static_cast<unsigned long long>(m_world.seed()) << ",\n";
      f << "  \"city\": \"" << JsonEscape(cityName) << "\",\n";
      f << "  \"exported_at\": \"" << JsonEscape(stamp) << "\",\n";
      f << "  \"entries\": [\n";
      for (std::size_t i = 0; i < m_cityNews.size(); ++i) {
        const CityNewsEntry& e = m_cityNews[i];
        f << "    {\"day\": " << e.day
          << ", \"tone\": \"" << ToneTag(e.tone) << "\""
          << ", \"mayor_rating\": " << static_cast<int>(std::round(static_cast<double>(e.mayorRating)))
          << ", \"headline\": \"" << JsonEscape(e.headline) << "\""
          << ", \"body\": \"" << JsonEscape(e.body) << "\"}";
        if (i + 1 < m_cityNews.size()) f << ",";
        f << "\n";
      }
      f << "  ]\n";
      f << "}\n";
    }
  }

  // Plain text export
  {
    const std::string path = base + ".txt";
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) {
      showToast("City News export failed (txt)");
    } else {
      f << "City News: " << cityName << "\n";
      f << "Seed: " << static_cast<unsigned long long>(m_world.seed()) << "\n";
      f << "Exported: " << stamp << "\n\n";
      for (const CityNewsEntry& e : m_cityNews) {
        f << "Day " << e.day << " [" << ToneTag(e.tone) << "] " << e.headline << "\n";
        f << e.body << "\n\n";
      }
    }
  }

  showToast(TextFormat("City News exported (%d entries)", static_cast<int>(m_cityNews.size())));
}

void Game::drawNewsPanel(const Rectangle& panelR)
{
  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  ui::DrawPanel(panelR, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panelR, "City News", uiTime, /*active=*/true, /*titleSizePx=*/22);

  // Close button.
  {
    const Rectangle br{panelR.x + panelR.width - 30.0f, panelR.y + 8.0f, 22.0f, 18.0f};
    if (ui::Button(8700, br, "X", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      m_showNewsPanel = false;
      return;
    }
  }

  // Export button.
  {
    const Rectangle br{panelR.x + 12.0f, panelR.y + 8.0f, 80.0f, 18.0f};
    if (ui::Button(8701, br, "Export", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      exportCityNews();
    }
  }

  const int pad = 10;
  const int x0 = static_cast<int>(panelR.x) + pad;
  int y = static_cast<int>(panelR.y) + 42;
  const int w = static_cast<int>(panelR.width) - pad * 2;

  // Mayor rating header.
  {
    const float rating = std::clamp(m_mayorRatingEma, 0.0f, 100.0f);
    const float delta = rating - m_mayorRatingPrev;
    const char* trend = (delta > 0.5f) ? "+" : (delta < -0.5f) ? "-" : "=";

    const std::string mood = CityMoodLabel(rating);
    Color rCol = uiTh.accent;
    if (rating >= 70.0f) rCol = uiTh.accentOk;
    if (rating < 40.0f) rCol = uiTh.accentBad;

    ui::Text(x0, y, 16, TextFormat("Mayor approval: %.0f%% (%s)  %s", static_cast<double>(rating), mood.c_str(), trend),
             rCol, /*bold=*/true, /*shadow=*/true, 1);
    y += 22;
  }

  // Layout: list (top) + selected article (bottom).
  const int listH = std::min(200, std::max(140, static_cast<int>(panelR.height) / 2 - 40));
  const Rectangle listR{static_cast<float>(x0), static_cast<float>(y), static_cast<float>(w), static_cast<float>(listH)};
  ui::DrawPanelInset(listR, uiTime, /*active=*/true);

  const int detailY = y + listH + 10;
  const Rectangle detailR{static_cast<float>(x0), static_cast<float>(detailY), static_cast<float>(w),
                          std::max(0.0f, panelR.y + panelR.height - static_cast<float>(detailY) - 10.0f)};
  ui::DrawPanelInset(detailR, uiTime * 0.9f, /*active=*/true);

  const int count = static_cast<int>(m_cityNews.size());
  if (count <= 0) {
    ui::Text(x0 + 8, y + 8, 14, "No headlines yet. (Unpause to generate daily news)", uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    return;
  }

  m_newsSelection = std::clamp(m_newsSelection, 0, std::max(0, count - 1));

  const int rowH = 22;
  const Rectangle viewR{listR.x + 6.0f, listR.y + 6.0f, listR.width - 12.0f, listR.height - 12.0f};
  constexpr float kSbW = 12.0f;
  const Rectangle barR{viewR.x + viewR.width - kSbW, viewR.y, kSbW, viewR.height};
  const Rectangle contentR = ui::ContentRectWithScrollbar(viewR, kSbW, 2.0f);

  const int visibleRows = std::max(1, static_cast<int>(std::floor(contentR.height / static_cast<float>(rowH))));

  int first = std::clamp(m_newsFirst, 0, std::max(0, count - visibleRows));

  // Mouse wheel scroll (moves selection; view follows).
  const bool hoverList = CheckCollisionPointRec(mouseUi, viewR);
  const float wheel = GetMouseWheelMove();
  if (hoverList && wheel != 0.0f && count > 0) {
    const int step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 3 : 1;
    const int delta = (wheel > 0.0f) ? -step : step;
    const int newSel = std::clamp(m_newsSelection + delta, 0, std::max(0, count - 1));
    if (newSel != m_newsSelection) {
      m_newsSelection = newSel;
    }
  }

  if (m_newsSelection < first) first = m_newsSelection;
  if (m_newsSelection >= first + visibleRows) first = m_newsSelection - visibleRows + 1;
  first = std::clamp(first, 0, std::max(0, count - visibleRows));
  m_newsFirst = first;

  const bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

  for (int i = 0; i < visibleRows; ++i) {
    const int idx = first + i;
    if (idx >= count) break;

    const CityNewsEntry& e = m_cityNews[static_cast<std::size_t>(idx)];
    const int ry = static_cast<int>(contentR.y) + i * rowH;
    const bool sel = (idx == m_newsSelection);

    const Rectangle rowR{contentR.x, static_cast<float>(ry), contentR.width, static_cast<float>(rowH)};
    if (leftPressed && CheckCollisionPointRec(mouseUi, rowR)) {
      m_newsSelection = idx;
    }

    if (sel) {
      ui::DrawSelectionHighlight(Rectangle{rowR.x - 2.0f, rowR.y + 1.0f, rowR.width + 4.0f, rowR.height - 2.0f},
                                 uiTime, /*strong=*/true);
    }

    Color tagCol = uiTh.textDim;
    if (e.tone == CityNewsTone::Good) tagCol = uiTh.accentOk;
    if (e.tone == CityNewsTone::Bad) tagCol = uiTh.accentBad;
    if (e.tone == CityNewsTone::Alert) tagCol = Color{255, 120, 120, 255};

    const Color headCol = sel ? uiTh.text : uiTh.textDim;

    ui::Text(static_cast<int>(contentR.x) + 6, ry + 4, 14,
             TextFormat("D%-3d [%s]", e.day, ToneTag(e.tone)), tagCol, /*bold=*/sel, /*shadow=*/true, 1);
    ui::Text(static_cast<int>(contentR.x) + 110, ry + 4, 14, e.headline, headCol, /*bold=*/sel, /*shadow=*/true, 1);
  }

  // Scrollbar.
  int newFirst = first;
  if (ui::ScrollbarV(8702, barR, count, visibleRows, newFirst, mouseUi, uiTime, /*enabled=*/true)) {
    newFirst = std::clamp(newFirst, 0, std::max(0, count - visibleRows));
    m_newsFirst = newFirst;
    if (m_newsSelection < newFirst) m_newsSelection = newFirst;
    if (m_newsSelection >= newFirst + visibleRows) m_newsSelection = std::clamp(newFirst + visibleRows - 1, 0, count - 1);
  }

  // Detail panel.
  {
    const CityNewsEntry& e = m_cityNews[static_cast<std::size_t>(m_newsSelection)];

    const int dx = static_cast<int>(detailR.x) + 10;
    int dy = static_cast<int>(detailR.y) + 8;

    ui::Text(dx, dy, 16, TextFormat("Day %d - %s", e.day, e.headline.c_str()), uiTh.text, /*bold=*/true, /*shadow=*/true, 1);
    dy += 22;

    const Rectangle bodyR{detailR.x + 10.0f, static_cast<float>(dy), detailR.width - 20.0f, detailR.height - (static_cast<float>(dy) - detailR.y) - 8.0f};
    ui::TextBox(bodyR, 14, e.body, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1, /*wrap=*/true, /*clip=*/true);
  }

  // Footer.
  ui::Text(x0, static_cast<int>(panelR.y + panelR.height) - 20, 13,
           "Ctrl+N: toggle  |  Ctrl+Shift+N: export  |  Wheel/Click: browse", uiTh.textFaint, /*bold=*/false,
           /*shadow=*/true, 1);
}

namespace {

inline int DaysLeftInclusive(int today, int deadline) { return std::max(0, deadline - today); }

inline float ProgressFromRange(int start, int cur, int target)
{
  if (target == start) return (cur >= target) ? 1.0f : 0.0f;
  const double denom = static_cast<double>(target - start);
  const double num = static_cast<double>(cur - start);
  return static_cast<float>(std::clamp(num / denom, 0.0, 1.0));
}

inline float ProgressFromRange(float start, float cur, float target)
{
  const float denom = (target - start);
  if (std::abs(denom) < 1e-6f) return (cur >= target) ? 1.0f : 0.0f;
  const float t = (cur - start) / denom;
  return Clamp01(t);
}

static float ChallengeProgress(const CityChallenge& c, const Stats& s)
{
  switch (c.kind) {
    case CityChallengeKind::GrowPopulation: {
      return ProgressFromRange(c.startInt, s.population, c.targetInt);
    }
    case CityChallengeKind::BuildParks: {
      return ProgressFromRange(c.startInt, s.parks, c.targetInt);
    }
    case CityChallengeKind::ReduceCongestion: {
      // Lower is better.
      if (c.startF <= c.targetF) return (s.trafficCongestion <= c.targetF) ? 1.0f : 0.0f;
      return ProgressFromRange(c.startF, s.trafficCongestion, c.targetF);
    }
    case CityChallengeKind::ImproveGoods: {
      return ProgressFromRange(c.startF, s.goodsSatisfaction, c.targetF);
    }
    case CityChallengeKind::ImproveServices: {
      return ProgressFromRange(c.startF, s.servicesOverallSatisfaction, c.targetF);
    }
    case CityChallengeKind::BalanceBudget: {
      const int req = std::max(1, c.targetInt);
      return Clamp01(static_cast<float>(std::clamp(c.stateInt, 0, req)) / static_cast<float>(req));
    }
    case CityChallengeKind::RestoreOutsideConnection: {
      const int denom = std::max(1, c.startInt);
      const int ok = std::max(0, denom - s.commutersUnreachable);
      return Clamp01(static_cast<float>(ok) / static_cast<float>(denom));
    }
  }
  return 0.0f;
}

static bool ChallengeComplete(const CityChallenge& c, const Stats& s)
{
  switch (c.kind) {
    case CityChallengeKind::GrowPopulation: return s.population >= c.targetInt;
    case CityChallengeKind::BuildParks: return s.parks >= c.targetInt;
    case CityChallengeKind::ReduceCongestion: return s.trafficCongestion <= c.targetF;
    case CityChallengeKind::ImproveGoods: return s.goodsSatisfaction >= c.targetF;
    case CityChallengeKind::ImproveServices: return s.servicesOverallSatisfaction >= c.targetF;
    case CityChallengeKind::BalanceBudget: return c.stateInt >= std::max(1, c.targetInt);
    case CityChallengeKind::RestoreOutsideConnection: return s.commutersUnreachable <= 0;
  }
  return false;
}

static CityChallengeKind PickWeighted(RNG& rng, const std::vector<std::pair<CityChallengeKind, float>>& weights)
{
  double sum = 0.0;
  for (const auto& w : weights) {
    if (w.second > 0.0f && std::isfinite(w.second)) sum += static_cast<double>(w.second);
  }
  if (!(sum > 0.0)) {
    return CityChallengeKind::GrowPopulation;
  }
  const double r = static_cast<double>(rng.nextF01()) * sum;
  double acc = 0.0;
  for (const auto& w : weights) {
    if (!(w.second > 0.0f) || !std::isfinite(w.second)) continue;
    acc += static_cast<double>(w.second);
    if (r <= acc) return w.first;
  }
  return weights.back().first;
}

static int RngRangeI(RNG& rng, int lo, int hi)
{
  if (hi <= lo) return lo;
  return rng.rangeInt(lo, hi);
}

static CityChallenge MakeChallenge(const World& world, const Stats& s, int day, std::uint32_t id, RNG& rng,
                                  const std::vector<CityChallengeKind>& avoidKinds)
{
  auto isAvoid = [&](CityChallengeKind k) {
    for (CityChallengeKind a : avoidKinds) {
      if (a == k) return true;
    }
    return false;
  };

  // Heuristic needs -> weights.
  const float congNeed = Clamp01((s.trafficCongestion - 0.25f) / 0.65f);
  const float goodsNeed = Clamp01((0.85f - s.goodsSatisfaction) / 0.85f);
  const float svcNeed = Clamp01((0.65f - s.servicesOverallSatisfaction) / 0.65f);
  const float outsideNeed = (s.commuters > 0) ? Clamp01(static_cast<float>(s.commutersUnreachable) / static_cast<float>(s.commuters)) : 0.0f;

  const float demandAvg = (s.demandResidential + s.demandCommercial + s.demandIndustrial) / 3.0f;
  const float growthNeed = Clamp01(0.35f + 0.85f * demandAvg);

  std::vector<std::pair<CityChallengeKind, float>> weights;
  weights.reserve(10);
  if (!isAvoid(CityChallengeKind::GrowPopulation)) weights.push_back({CityChallengeKind::GrowPopulation, 1.25f * growthNeed});
  if (!isAvoid(CityChallengeKind::BuildParks)) weights.push_back({CityChallengeKind::BuildParks, 0.55f + 0.75f * Clamp01(0.5f - s.happiness)});
  if (!isAvoid(CityChallengeKind::ReduceCongestion)) weights.push_back({CityChallengeKind::ReduceCongestion, 0.15f + 1.70f * congNeed});
  if (!isAvoid(CityChallengeKind::ImproveGoods)) weights.push_back({CityChallengeKind::ImproveGoods, 0.25f + 1.30f * goodsNeed});
  if (!isAvoid(CityChallengeKind::ImproveServices)) weights.push_back({CityChallengeKind::ImproveServices, 0.25f + 1.30f * svcNeed});
  if (!isAvoid(CityChallengeKind::BalanceBudget)) weights.push_back({CityChallengeKind::BalanceBudget, 0.25f + 0.8f * Clamp01(static_cast<float>(s.expenses - s.income + 150) / 450.0f)});
  if (!isAvoid(CityChallengeKind::RestoreOutsideConnection) && s.commutersUnreachable > 0) {
    weights.push_back({CityChallengeKind::RestoreOutsideConnection, 0.25f + 1.75f * outsideNeed});
  }

  // Early game: bias toward growth.
  if (s.day < 5 || s.population == 0) {
    for (auto& w : weights) {
      if (w.first == CityChallengeKind::GrowPopulation) w.second *= 3.0f;
    }
  }

  CityChallenge c;
  c.id = id;
  c.dayIssued = day;
  c.status = CityChallengeStatus::Active;

  c.kind = PickWeighted(rng, weights);

  // Build the actual parameters.
  switch (c.kind) {
    case CityChallengeKind::GrowPopulation: {
      c.startInt = s.population;
      const int base = std::max(20, std::min(900, 60 + s.population / 4));
      const int delta = RngRangeI(rng, base / 2, base);
      c.targetInt = c.startInt + delta;
      const int dur = RngRangeI(rng, 10, 38);
      c.dayDeadline = day + dur;
      c.rewardMoney = 80 + delta * 2 + dur * 3;
      c.title = TextFormat("Grow to %d residents", c.targetInt);
      c.description = TextFormat("Increase population from %d to %d by Day %d.\nTip: keep Residential demand high and connect homes to jobs.",
                                 c.startInt, c.targetInt, c.dayDeadline);
    } break;
    case CityChallengeKind::BuildParks: {
      c.startInt = s.parks;
      const int add = std::clamp(RngRangeI(rng, 2, 10) + (s.population / 200), 2, 18);
      c.targetInt = c.startInt + add;
      const int dur = RngRangeI(rng, 12, 45);
      c.dayDeadline = day + dur;
      c.rewardMoney = 60 + add * 40 + dur * 2;
      c.title = TextFormat("Build %d parks", add);
      c.description = TextFormat("Add %d park tiles by Day %d.\nParks boost happiness, land value, and help stabilize growth.",
                                 add, c.dayDeadline);
    } break;
    case CityChallengeKind::ReduceCongestion: {
      c.startF = std::clamp(s.trafficCongestion, 0.0f, 1.0f);
      const float drop = rng.rangeFloat(0.15f, 0.45f);
      c.targetF = std::clamp(c.startF - drop, 0.05f, 0.55f);
      const int dur = RngRangeI(rng, 8, 30);
      c.dayDeadline = day + dur;
      c.rewardMoney = 110 + static_cast<int>(std::round(static_cast<double>((c.startF - c.targetF) * 520.0f))) + dur * 4;
      c.title = TextFormat("Cut congestion below %d%%", static_cast<int>(std::round(c.targetF * 100.0f)));
      c.description = TextFormat("Lower traffic congestion to <= %d%% by Day %d.\nTip: add bypass routes, upgrade key corridors, or build transit.",
                                 static_cast<int>(std::round(c.targetF * 100.0f)), c.dayDeadline);
    } break;
    case CityChallengeKind::ImproveGoods: {
      c.startF = std::clamp(s.goodsSatisfaction, 0.0f, 1.0f);
      c.targetF = std::clamp(std::max(0.75f, c.startF + rng.rangeFloat(0.15f, 0.35f)), 0.0f, 1.0f);
      const int dur = RngRangeI(rng, 8, 28);
      c.dayDeadline = day + dur;
      c.rewardMoney = 90 + static_cast<int>(std::round(static_cast<double>((c.targetF - c.startF) * 420.0f))) + dur * 3;
      c.title = TextFormat("Deliver goods at %d%%", static_cast<int>(std::round(c.targetF * 100.0f)));
      c.description = TextFormat("Raise goods satisfaction to >= %d%% by Day %d.\nTip: ensure industrial and commercial zones are connected to the road network.",
                                 static_cast<int>(std::round(c.targetF * 100.0f)), c.dayDeadline);
    } break;
    case CityChallengeKind::ImproveServices: {
      c.startF = std::clamp(s.servicesOverallSatisfaction, 0.0f, 1.0f);
      c.targetF = std::clamp(std::max(0.55f, c.startF + rng.rangeFloat(0.15f, 0.35f)), 0.0f, 0.95f);
      const int dur = RngRangeI(rng, 12, 40);
      c.dayDeadline = day + dur;
      c.rewardMoney = 100 + static_cast<int>(std::round(static_cast<double>((c.targetF - c.startF) * 520.0f))) + dur * 3;
      c.title = TextFormat("Improve services to %d%%", static_cast<int>(std::round(c.targetF * 100.0f)));
      c.description = TextFormat("Raise overall public services satisfaction to >= %d%% by Day %d.\nTip: place schools/hospitals/police/fire near residents.",
                                 static_cast<int>(std::round(c.targetF * 100.0f)), c.dayDeadline);
    } break;
    case CityChallengeKind::BalanceBudget: {
      c.stateInt = 0;
      c.targetInt = std::clamp(RngRangeI(rng, 3, 5), 3, 7);
      const int dur = RngRangeI(rng, 10, 42);
      c.dayDeadline = day + dur;
      c.rewardMoney = 120 + c.targetInt * 60 + dur * 3;
      c.title = TextFormat("Run a surplus for %d days", c.targetInt);
      c.description = TextFormat("Achieve %d consecutive days with Net >= 0 by Day %d.\nTip: adjust taxes/maintenance and watch demand.",
                                 c.targetInt, c.dayDeadline);
    } break;
    case CityChallengeKind::RestoreOutsideConnection: {
      const int unreachable = std::max(0, s.commutersUnreachable);
      c.startInt = std::max(1, unreachable);
      c.targetInt = 0;
      const int dur = RngRangeI(rng, 7, 25);
      c.dayDeadline = day + dur;
      c.rewardMoney = 90 + std::min(500, unreachable * 2) + dur * 4;
      c.title = "Restore outside access";
      c.description = TextFormat("Reduce unreachable commuters to 0 by Day %d.\nTip: connect roads to the map edge and avoid isolated neighborhoods.",
                                 c.dayDeadline);
    } break;
  }

  // Ensure the title isn't empty in edge cases.
  if (c.title.empty()) c.title = "City challenge";
  if (c.dayDeadline < c.dayIssued) c.dayDeadline = c.dayIssued;
  if (c.rewardMoney < 0) c.rewardMoney = 0;

  (void)world;
  return c;
}

static Color ChallengeProgressColor(const ui::Theme& th, float p, int daysLeft)
{
  if (p >= 0.999f) return th.accentOk;
  if (daysLeft <= 3 && p < 0.4f) return th.accentBad;
  return th.accent;
}

} // namespace

void Game::clearCityChallenges()
{
  m_cityChallenges.clear();
  m_challengeLog.clear();
  m_cityNewsAddendum.clear();
  m_challengeSelection = 0;
  m_challengeFirst = 0;
  m_challengeRerolls = 0;
  m_challengeNextId = 1;
  m_challengeLastProcessedDay = -1;
}

int Game::applyCityChallenges(Stats& ioStats)
{
  const int day = ioStats.day;
  if (day == m_challengeLastProcessedDay) return 0;
  m_challengeLastProcessedDay = day;

  // Deterministic RNG: world seed + day.
  const std::uint64_t seed = (static_cast<std::uint64_t>(m_world.seed()) ^ 0xBADC0FFEE0DDF00DULL) +
                             (static_cast<std::uint64_t>(day) * 0x9E3779B97F4A7C15ULL);
  RNG rng(seed);

  auto addBullet = [&](std::string_view line) {
    std::string& a = m_cityNewsAddendum[day];
    if (a.empty()) {
      a = "Council Dispatch:\n";
    }
    a += "- ";
    a.append(line.data(), line.size());
    a += "\n";
  };

  // Update active challenges.
  int moneyAward = 0;
  for (CityChallenge& c : m_cityChallenges) {
    if (c.status != CityChallengeStatus::Active) continue;

    // Stateful kinds.
    if (c.kind == CityChallengeKind::BalanceBudget) {
      const int net = ioStats.income - ioStats.expenses;
      if (net >= 0) c.stateInt = std::max(0, c.stateInt) + 1;
      else c.stateInt = 0;
    }

    const bool complete = ChallengeComplete(c, ioStats);
    const bool expired = (day > c.dayDeadline);
    if (complete) {
      c.status = CityChallengeStatus::Completed;
      moneyAward += std::max(0, c.rewardMoney);

      addBullet(TextFormat("Challenge completed: %s (+$%d)", c.title.c_str(), std::max(0, c.rewardMoney)));
      m_challengeLog.push_back(CityChallengeLogEntry{day, c.status, std::max(0, c.rewardMoney), c.title});
    } else if (expired) {
      c.status = CityChallengeStatus::Failed;
      addBullet(TextFormat("Challenge failed: %s", c.title.c_str()));
      m_challengeLog.push_back(CityChallengeLogEntry{day, c.status, 0, c.title});
    }
  }

  // Prune logs.
  while (static_cast<int>(m_challengeLog.size()) > std::max(8, m_challengeLogMax)) {
    m_challengeLog.pop_front();
  }

  // Remove completed/failed challenges from the active list.
  m_cityChallenges.erase(std::remove_if(m_cityChallenges.begin(), m_cityChallenges.end(), [](const CityChallenge& c) {
                          return c.status != CityChallengeStatus::Active;
                        }),
                        m_cityChallenges.end());

  // Keep a few active challenges available.
  const int activeCount = static_cast<int>(m_cityChallenges.size());
  const int target = std::clamp(m_challengeTargetActive, 1, 6);
  if (activeCount < target) {
    // Avoid duplicate kinds already active.
    std::vector<CityChallengeKind> avoid;
    avoid.reserve(static_cast<std::size_t>(target));
    for (const CityChallenge& c : m_cityChallenges) {
      avoid.push_back(c.kind);
    }
    const int toCreate = target - activeCount;
    for (int i = 0; i < toCreate; ++i) {
      const std::uint32_t id = m_challengeNextId++;
      CityChallenge nc = MakeChallenge(m_world, ioStats, day, id, rng, avoid);
      avoid.push_back(nc.kind);
      m_cityChallenges.push_back(std::move(nc));
    }
  }

  // Apply reward.
  if (moneyAward > 0) {
    ioStats.money += moneyAward;
    showToast(TextFormat("Challenge reward: +$%d", moneyAward), 2.0f);
  }

  return moneyAward;
}

void Game::focusChallenge(const CityChallenge& c)
{
  switch (c.kind) {
    case CityChallengeKind::BuildParks:
      setTool(Tool::Park);
      m_heatmapOverlay = HeatmapOverlay::ParkAmenity;
      showToast("Focus: build parks (heatmap: Park amenity)", 2.0f);
      break;
    case CityChallengeKind::GrowPopulation:
      setTool(Tool::Residential);
      m_showReport = true;
      showToast("Focus: grow population (tool: Residential)", 2.0f);
      break;
    case CityChallengeKind::ReduceCongestion:
      m_showTrafficOverlay = true;
      m_heatmapOverlay = HeatmapOverlay::TrafficSpill;
      m_trafficDirty = true;
      showToast("Focus: reduce congestion (heatmap: Traffic spill)", 2.0f);
      break;
    case CityChallengeKind::ImproveGoods:
      m_showGoodsOverlay = true;
      m_goodsDirty = true;
      showToast("Focus: improve goods delivery (goods overlay)", 2.0f);
      break;
    case CityChallengeKind::ImproveServices:
      m_showServicesPanel = true;
      m_heatmapOverlay = HeatmapOverlay::ServicesOverall;
      m_servicesHeatmapsDirty = true;
      showToast("Focus: improve services (Services panel)", 2.0f);
      break;
    case CityChallengeKind::BalanceBudget:
      m_showPolicy = true;
      showToast("Focus: balance budget (Policy panel)", 2.0f);
      break;
    case CityChallengeKind::RestoreOutsideConnection:
      m_showOutsideOverlay = true;
      showToast("Focus: restore outside access (Outside overlay)", 2.0f);
      break;
  }
}

bool Game::rerollChallenge(std::size_t idx)
{
  if (idx >= m_cityChallenges.size()) return false;
  CityChallenge& c = m_cityChallenges[idx];
  if (c.status != CityChallengeStatus::Active) return false;

  const int base = 25;
  const int cost = base + 20 * std::clamp(m_challengeRerolls, 0, 20);
  if (m_world.stats().money < cost) {
    showToast(TextFormat("Reroll costs $%d (insufficient funds)", cost), 2.5f);
    return false;
  }

  m_world.stats().money -= cost;
  m_challengeRerolls += 1;

  // Cancel and replace (avoid repeating the same kind immediately).
  const int day = m_world.stats().day;
  m_challengeLog.push_back(CityChallengeLogEntry{day, CityChallengeStatus::Canceled, 0, c.title});

  std::vector<CityChallengeKind> avoid;
  avoid.reserve(m_cityChallenges.size());
  avoid.push_back(c.kind);
  for (std::size_t i = 0; i < m_cityChallenges.size(); ++i) {
    if (i == idx) continue;
    avoid.push_back(m_cityChallenges[i].kind);
  }

  // Deterministic reroll seed: day + reroll count.
  const std::uint64_t seed = (static_cast<std::uint64_t>(m_world.seed()) ^ 0xC0FFEE123456789ULL) +
                             (static_cast<std::uint64_t>(day) * 0x9E3779B97F4A7C15ULL) +
                             (static_cast<std::uint64_t>(m_challengeRerolls) * 0xD1B54A32D192ED03ULL);
  RNG rng(seed);

  CityChallenge nc = MakeChallenge(m_world, m_world.stats(), day, m_challengeNextId++, rng, avoid);
  c = std::move(nc);
  showToast(TextFormat("Challenge rerolled (-$%d)", cost), 2.0f);
  return true;
}

void Game::drawChallengesPanel(const Rectangle& panelR)
{
  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  ui::DrawPanel(panelR, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panelR, "City Challenges", uiTime, /*active=*/true, /*titleSizePx=*/22);

  // Close.
  {
    const Rectangle br{panelR.x + panelR.width - 30.0f, panelR.y + 8.0f, 22.0f, 18.0f};
    if (ui::Button(8800, br, "X", mouseUi, uiTime, true, false)) {
      m_showChallengesPanel = false;
      return;
    }
  }

  const int pad = 10;
  const int x0 = static_cast<int>(panelR.x) + pad;
  int y = static_cast<int>(panelR.y) + 40;
  const int w = static_cast<int>(panelR.width) - pad * 2;

  const Stats& st = m_world.stats();
  ui::Text(x0, y, 14,
           TextFormat("Tab: select  |  Enter: focus  |  Del: reroll  |  Active: %d", static_cast<int>(m_cityChallenges.size())),
           uiTh.textDim, false, true, 1);
  y += 18;

  const int listH = std::min(176, std::max(130, static_cast<int>(panelR.height) / 2 - 10));
  const Rectangle listR{static_cast<float>(x0), static_cast<float>(y), static_cast<float>(w), static_cast<float>(listH)};
  ui::DrawPanelInset(listR, uiTime, true);
  const int detailY = y + listH + 10;
  const Rectangle detailR{static_cast<float>(x0), static_cast<float>(detailY), static_cast<float>(w),
                          std::max(0.0f, panelR.y + panelR.height - static_cast<float>(detailY) - 10.0f)};
  ui::DrawPanelInset(detailR, uiTime * 0.9f, true);

  const int count = static_cast<int>(m_cityChallenges.size());
  if (count <= 0) {
    ui::Text(x0 + 8, y + 8, 14, "No active challenges (unpause to generate).", uiTh.textDim, false, true, 1);
    return;
  }

  m_challengeSelection = std::clamp(m_challengeSelection, 0, std::max(0, count - 1));

  const int rowH = 24;
  const Rectangle viewR{listR.x + 6.0f, listR.y + 6.0f, listR.width - 12.0f, listR.height - 12.0f};
  constexpr float kSbW = 12.0f;
  const Rectangle barR{viewR.x + viewR.width - kSbW, viewR.y, kSbW, viewR.height};
  const Rectangle contentR = ui::ContentRectWithScrollbar(viewR, kSbW, 2.0f);
  const int visibleRows = std::max(1, static_cast<int>(std::floor(contentR.height / static_cast<float>(rowH))));

  int first = std::clamp(m_challengeFirst, 0, std::max(0, count - visibleRows));
  const bool hoverList = CheckCollisionPointRec(mouseUi, viewR);
  const float wheel = GetMouseWheelMove();
  if (hoverList && wheel != 0.0f && count > 0) {
    const int step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 2 : 1;
    const int delta = (wheel > 0.0f) ? -step : step;
    const int newSel = std::clamp(m_challengeSelection + delta, 0, std::max(0, count - 1));
    if (newSel != m_challengeSelection) m_challengeSelection = newSel;
  }

  if (m_challengeSelection < first) first = m_challengeSelection;
  if (m_challengeSelection >= first + visibleRows) first = m_challengeSelection - visibleRows + 1;
  first = std::clamp(first, 0, std::max(0, count - visibleRows));
  m_challengeFirst = first;

  const bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
  for (int i = 0; i < visibleRows; ++i) {
    const int idx = first + i;
    if (idx >= count) break;
    const CityChallenge& c = m_cityChallenges[static_cast<std::size_t>(idx)];
    const int ry = static_cast<int>(contentR.y) + i * rowH;
    const bool sel = (idx == m_challengeSelection);

    const Rectangle rowR{contentR.x, static_cast<float>(ry), contentR.width, static_cast<float>(rowH)};
    if (leftPressed && CheckCollisionPointRec(mouseUi, rowR)) {
      m_challengeSelection = idx;
    }
    if (sel) {
      ui::DrawSelectionHighlight(Rectangle{rowR.x - 2.0f, rowR.y + 1.0f, rowR.width + 4.0f, rowR.height - 2.0f},
                                 uiTime, true);
    }

    const int dl = DaysLeftInclusive(st.day, c.dayDeadline);
    const float p = ChallengeProgress(c, st);
    const Color pc = ChallengeProgressColor(uiTh, p, dl);
    ui::Text(static_cast<int>(contentR.x) + 6, ry + 5, 14,
             TextFormat("D%3d  %2dd  $%d", c.dayDeadline, dl, std::max(0, c.rewardMoney)),
             pc, sel, true, 1);
    ui::Text(static_cast<int>(contentR.x) + 136, ry + 5, 14, c.title, sel ? uiTh.text : uiTh.textDim, sel, true, 1);
  }

  // Scrollbar.
  int newFirst = first;
  if (ui::ScrollbarV(8801, barR, count, visibleRows, newFirst, mouseUi, uiTime, true)) {
    newFirst = std::clamp(newFirst, 0, std::max(0, count - visibleRows));
    m_challengeFirst = newFirst;
    if (m_challengeSelection < newFirst) m_challengeSelection = newFirst;
    if (m_challengeSelection >= newFirst + visibleRows) m_challengeSelection = std::clamp(newFirst + visibleRows - 1, 0, count - 1);
  }

  // Detail.
  {
    const CityChallenge& c = m_cityChallenges[static_cast<std::size_t>(m_challengeSelection)];
    const int dx = static_cast<int>(detailR.x) + 10;
    int dy = static_cast<int>(detailR.y) + 8;

    const int dl = DaysLeftInclusive(st.day, c.dayDeadline);
    ui::Text(dx, dy, 16, c.title, uiTh.text, true, true, 1);
    dy += 20;

    ui::Text(dx, dy, 13, TextFormat("Due Day %d  (%d days left)   Reward: $%d", c.dayDeadline, dl, std::max(0, c.rewardMoney)),
             uiTh.textDim, false, true, 1);
    dy += 18;

    const float p = ChallengeProgress(c, st);
    const Color pc = ChallengeProgressColor(uiTh, p, dl);
    const Rectangle pr{detailR.x + 10.0f, static_cast<float>(dy), detailR.width - 20.0f, 14.0f};
    ui::ProgressBar(pr, p, pc, uiTime, true);
    ui::Text(dx, dy - 1, 13, TextFormat("%.0f%%", static_cast<double>(p * 100.0f)), uiTh.text, true, true, 1);
    dy += 22;

    // Description.
    const Rectangle bodyR{detailR.x + 10.0f, static_cast<float>(dy), detailR.width - 20.0f, detailR.height - (static_cast<float>(dy) - detailR.y) - 40.0f};
    ui::TextBox(bodyR, 14, c.description, uiTh.textDim, false, true, 1, true, true);

    // Buttons.
    const Rectangle focusR{detailR.x + 10.0f, detailR.y + detailR.height - 28.0f, 120.0f, 20.0f};
    const Rectangle rerollR{detailR.x + 140.0f, detailR.y + detailR.height - 28.0f, 120.0f, 20.0f};
    const int cost = 25 + 20 * std::clamp(m_challengeRerolls, 0, 20);

    if (ui::Button(8802, focusR, "Focus", mouseUi, uiTime, true, true)) {
      focusChallenge(c);
    }
    if (ui::Button(8803, rerollR, TextFormat("Reroll ($%d)", cost), mouseUi, uiTime, true, false)) {
      (void)rerollChallenge(static_cast<std::size_t>(m_challengeSelection));
    }

    // Recent log.
    if (!m_challengeLog.empty()) {
      const int lines = std::min(3, static_cast<int>(m_challengeLog.size()));
      int ly = static_cast<int>(detailR.y + detailR.height) - 52 - 14 * lines;
      ui::Text(dx, ly, 13, "Recent:", uiTh.textFaint, true, true, 1);
      ly += 14;
      for (int i = 0; i < lines; ++i) {
        const CityChallengeLogEntry& e = m_challengeLog[m_challengeLog.size() - 1 - i];
        const bool ok = (e.status == CityChallengeStatus::Completed);
        const Color lc = ok ? uiTh.accentOk : (e.status == CityChallengeStatus::Failed) ? uiTh.accentBad : uiTh.textFaint;
        std::string line = TextFormat("Day %d: %s", e.day, e.title.c_str());
        if (ok && e.rewardMoney > 0) {
          line += TextFormat(" (+$%d)", e.rewardMoney);
        }
        if (line.size() > 70) {
          line.resize(67);
          line += "...";
        }
        ui::Text(dx, ly, 13, line, lc, false, true, 1);
        ly += 14;
      }
    }
  }

  ui::Text(x0, static_cast<int>(panelR.y + panelR.height) - 20, 13,
           "Ctrl+O: toggle  |  Tab: cycle  |  Enter: focus  |  Del: reroll", uiTh.textFaint, false, true, 1);
}


void Game::issueBond(int principal, int termDays, int aprBasisPoints)
{
  principal = std::max(0, principal);
  termDays = std::max(1, termDays);
  aprBasisPoints = std::clamp(aprBasisPoints, 0, 50000);

  if (principal == 0) return;

  // Compute an amortized daily payment estimate. The sim will force a full payoff on the
  // final day, so small rounding drift is OK.
  int dailyPayment = 0;
  if (aprBasisPoints <= 0) {
    dailyPayment = (principal + termDays - 1) / termDays;
  } else {
    const double apr = static_cast<double>(aprBasisPoints) / 10000.0;
    const double r = apr / 365.0;
    if (r <= 0.0) {
      dailyPayment = (principal + termDays - 1) / termDays;
    } else {
      const double n = static_cast<double>(termDays);
      const double powv = std::pow(1.0 + r, n);
      const double denom = (powv - 1.0);
      const double pmt = (denom != 0.0) ? (static_cast<double>(principal) * r * powv) / denom : 0.0;
      if (!std::isfinite(pmt) || pmt <= 0.0) {
        dailyPayment = (principal + termDays - 1) / termDays;
      } else {
        dailyPayment = static_cast<int>(std::ceil(pmt));
      }
    }
  }
  dailyPayment = std::max(1, dailyPayment);

  // Pick a stable id (max+1).
  int nextId = 1;
  for (const DebtItem& d : m_world.debts()) {
    nextId = std::max(nextId, d.id + 1);
  }

  DebtItem debt{};
  debt.id = nextId;
  debt.principal = principal;
  debt.balance = principal;
  debt.termDays = termDays;
  debt.daysLeft = termDays;
  debt.aprBasisPoints = aprBasisPoints;
  debt.dailyPayment = dailyPayment;
  debt.issuedDay = m_world.stats().day;

  m_world.debts().push_back(debt);

  // Cash injection (kept separate from tax income so the graphs stay meaningful).
  if (principal > 0 && m_world.stats().money > (std::numeric_limits<int>::max() - principal)) {
    m_world.stats().money = std::numeric_limits<int>::max();
  } else {
    m_world.stats().money += principal;
  }

  showToast(TextFormat("Bond #%d issued: +$%d @ %.2f%%", debt.id, principal, aprBasisPoints / 100.0f));

  // Add a lightweight news entry so players notice the extra expense line item.
  CityNewsEntry e{};
  e.day = m_world.stats().day;
  e.tone = CityNewsTone::Neutral;
  e.headline = "City treasury: bond issued";
  e.body = TextFormat(
      "The city issued a $%d municipal bond to fund infrastructure. Debt service starts immediately.\n\n"
      "Tip: bonds boost cash now, but increase daily expenses until paid off.",
      principal);
  e.mayorRating = static_cast<float>(std::clamp(static_cast<int>(m_world.stats().happiness * 100.0f), 0, 100));
  m_cityNews.push_back(std::move(e));
  if (m_cityNews.size() > 250) {
    m_cityNews.erase(m_cityNews.begin(), m_cityNews.begin() + (m_cityNews.size() - 250));
  }
}

void Game::payDownBond(std::size_t idx, int payment)
{
  auto& debts = m_world.debts();
  if (idx >= debts.size()) return;

  payment = std::max(0, payment);
  if (payment == 0) return;

  DebtItem& d = debts[idx];
  if (d.balance <= 0) return;

  if (m_world.stats().money <= 0) {
    showToast("Not enough cash to pay down debt");
    return;
  }

  const int pay = std::min({payment, d.balance, m_world.stats().money});
  if (pay <= 0) return;

  m_world.stats().money -= pay;
  d.balance -= pay;

  if (d.balance <= 0) {
    const int id = d.id;
    debts.erase(debts.begin() + static_cast<std::ptrdiff_t>(idx));
    showToast(TextFormat("Bond #%d paid off (-$%d)", id, pay));
  } else {
    showToast(TextFormat("Paid $%d toward bond #%d", pay, d.id));
  }

  if (debts.empty()) {
    m_bondsSelection = 0;
    m_bondsFirst = 0;
  } else {
    m_bondsSelection = std::clamp(m_bondsSelection, 0, static_cast<int>(debts.size()) - 1);
    m_bondsFirst = std::clamp(m_bondsFirst, 0, std::max(0, static_cast<int>(debts.size()) - 1));
  }
}


void Game::drawBondsPanel(const Rectangle& rect)
{
  const float uiTime = GetTime();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);
  const ui::Theme& uiTh = ui::theme();

  ui::DrawPanel(rect, uiTime, true);
  ui::DrawPanelHeader(rect, "City Treasury", uiTime, true);

  // Close button.
  if (ui::Button(24100, {rect.x + rect.width - 28, rect.y + 6, 22, 20}, "x", mouseUi, uiTime, true, false)) {
    m_showBondsPanel = false;
    return;
  }

  const int pad = 10;
  const int x0 = static_cast<int>(rect.x) + pad;
  int y = static_cast<int>(rect.y) + 40;
  const int w = static_cast<int>(rect.width) - pad * 2;

  const Stats& st = m_world.stats();
  const int pop = st.population;
  const int net = st.income - st.expenses;
  const int money = st.money;

  int totalDebt = 0;
  int totalDaily = 0;
  for (const DebtItem& d : m_world.debts()) {
    totalDebt += std::max(0, d.balance);
    totalDaily += std::max(0, std::min(d.dailyPayment, d.balance));
  }

  // --- Market / credit scoring (lightweight, deterministic) ---
  const float happiness01 = std::clamp(st.happiness, 0.0f, 1.0f);

  const float netPerCap = (pop > 0) ? static_cast<float>(net) / static_cast<float>(pop) : static_cast<float>(net);
  const float netScore01 = std::clamp(0.5f + 0.5f * (netPerCap / 5.0f), 0.0f, 1.0f);

  const float cashScore01 = std::clamp(0.5f + 0.5f * (static_cast<float>(money) / 1000.0f), 0.0f, 1.0f);

  const float debtScale = static_cast<float>(std::max(500, pop * 80 + 500));
  const float debtScore01 = std::clamp(1.0f - (static_cast<float>(totalDebt) / debtScale), 0.0f, 1.0f);

  float credit01 = 0.15f;
  credit01 += 0.35f * happiness01;
  credit01 += 0.25f * netScore01;
  credit01 += 0.15f * cashScore01;
  credit01 += 0.10f * debtScore01;
  credit01 = std::clamp(credit01, 0.0f, 1.0f);

  const char* rating = "B";
  if (credit01 >= 0.85f)
    rating = "AAA";
  else if (credit01 >= 0.72f)
    rating = "AA";
  else if (credit01 >= 0.60f)
    rating = "A";
  else if (credit01 >= 0.48f)
    rating = "BBB";
  else if (credit01 >= 0.36f)
    rating = "BB";

  const int baseCap = 600 + pop * 35;
  const float capMult = 0.60f + 0.90f * credit01;
  const int maxOutstanding =
      std::clamp(static_cast<int>(std::round(static_cast<float>(baseCap) * capMult)), 200, 200000);
  const int available = std::max(0, maxOutstanding - totalDebt);

  // --- Summary ---
  ui::Text(x0, y, 14, TextFormat("Rating: %s   Credit: %.0f%%", rating, std::round(credit01 * 100.0f)), uiTh.text,
           /*bold=*/true, /*shadow=*/true, 1);
  y += 18;

  ui::Text(x0, y, 14, TextFormat("Cash: $%d   Net/day: $%d   Pop: %d", money, net, pop), uiTh.textDim, false, true, 1);
  y += 18;

  ui::Text(x0, y, 14, TextFormat("Debt: $%d   Debt service: ~$%d/day", totalDebt, totalDaily), uiTh.textDim, false, true,
           1);
  y += 18;

  ui::Text(x0, y, 14, TextFormat("Capacity: $%d   Available: $%d", maxOutstanding, available), uiTh.textDim, false, true,
           1);
  y += 10;

  DrawRectangle(x0, y, w, 1, uiTh.panelBorder);
  y += 10;

  // --- Issuance ---
  ui::Text(x0, y, 14, "Issue a bond", uiTh.text, true, true, 1);
  y += 18;

  constexpr int kBondTypeCount = 3;
  const int termDays[kBondTypeCount] = {30, 90, 180};
  const int baseAprBp[kBondTypeCount] = {450, 650, 850}; // 4.5%, 6.5%, 8.5%

  m_bondsType = std::clamp(m_bondsType, 0, kBondTypeCount - 1);

  // Type buttons (Short / Medium / Long).
  const int gap = 6;
  const float bw = static_cast<float>((w - gap * 2) / 3);

  const char* typeLabel[kBondTypeCount] = {"Short", "Medium", "Long"};
  for (int i = 0; i < kBondTypeCount; ++i) {
    Rectangle br{static_cast<float>(x0 + (static_cast<int>(bw) + gap) * i), static_cast<float>(y), bw, 20.0f};
    if (ui::Button(24110 + i, br, typeLabel[i], mouseUi, uiTime, true, (m_bondsType == i))) {
      m_bondsType = i;
    }
  }
  y += 26;

  const int penaltyBp = static_cast<int>(std::round((1.0f - credit01) * 600.0f)); // up to +6%
  const int offerAprBp = std::clamp(baseAprBp[m_bondsType] + penaltyBp, 200, 2500);

  const int maxAmount = available;
  const int minAmount = 100;
  const bool canBorrow = (maxAmount >= minAmount);

  if (canBorrow) {
    m_bondsAmount = std::clamp(m_bondsAmount, minAmount, maxAmount);
  } else {
    m_bondsAmount = minAmount;
  }

  // Payment preview.
  int previewPayment = 0;
  {
    const int principal = m_bondsAmount;
    const int tdays = termDays[m_bondsType];

    if (!canBorrow) {
      previewPayment = 0;
    } else if (offerAprBp <= 0) {
      previewPayment = (principal + tdays - 1) / tdays;
    } else {
      const double apr = static_cast<double>(offerAprBp) / 10000.0;
      const double r = apr / 365.0;
      const double n = static_cast<double>(tdays);
      const double powv = std::pow(1.0 + r, n);
      const double denom = (powv - 1.0);
      const double pmt = (denom != 0.0) ? (static_cast<double>(principal) * r * powv) / denom : 0.0;
      previewPayment =
          (!std::isfinite(pmt) || pmt <= 0.0) ? (principal + tdays - 1) / tdays : static_cast<int>(std::ceil(pmt));
    }
    previewPayment = std::max(1, previewPayment);
  }

  ui::Text(x0, y, 14,
           TextFormat("Offer: %d days @ %.2f%% APR  |  Est. payment: ~$%d/day", termDays[m_bondsType],
                      offerAprBp / 100.0f, previewPayment),
           uiTh.textDim, false, true, 1);
  y += 18;

  // Amount slider.
  {
    Rectangle sliderR{static_cast<float>(x0), static_cast<float>(y), static_cast<float>(w), 20.0f};
    const int maxSlider = std::max(minAmount, maxAmount);
    ui::SliderInt(24130, sliderR, m_bondsAmount, minAmount, maxSlider, mouseUi, uiTime, canBorrow);
    y += 24;
  }

  // Issue button.
  {
    Rectangle issueR{static_cast<float>(x0), static_cast<float>(y), static_cast<float>(w), 22.0f};
    const bool enabled = canBorrow && (m_bondsAmount >= minAmount) && (m_bondsAmount <= maxAmount);
    if (ui::Button(24140, issueR, enabled ? "Issue bond" : "Issue bond (unavailable)", mouseUi, uiTime, enabled,
                   /*primary=*/true)) {
      issueBond(m_bondsAmount, termDays[m_bondsType], offerAprBp);
    }
    y += 28;
  }

  DrawRectangle(x0, y, w, 1, uiTh.panelBorder);
  y += 10;

  // --- Debt list ---
  ui::Text(x0, y, 14, "Outstanding bonds", uiTh.text, true, true, 1);
  y += 18;

  Rectangle listR{static_cast<float>(x0), static_cast<float>(y), static_cast<float>(w), 110.0f};
  ui::DrawPanelInset(listR, uiTime, true);

  const auto& debts = m_world.debts();
  const int rowH = 18;
  const int visibleRows = std::max(1, static_cast<int>((listR.height - 8.0f) / static_cast<float>(rowH)));

  if (CheckCollisionPointRec(mouseUi, listR)) {
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
      m_bondsFirst -= static_cast<int>(wheel);
    }
  }

  m_bondsSelection = std::clamp(m_bondsSelection, 0, std::max(0, static_cast<int>(debts.size()) - 1));
  m_bondsFirst = std::clamp(m_bondsFirst, 0, std::max(0, static_cast<int>(debts.size()) - visibleRows));

  float ly = listR.y + 4.0f;
  if (debts.empty()) {
    ui::Text(x0 + 6, static_cast<int>(ly), 14, "No outstanding debt.", uiTh.textDim, false, false, 0);
  } else {
    const int start = m_bondsFirst;
    const int end = std::min(static_cast<int>(debts.size()), start + visibleRows);

    for (int i = start; i < end; ++i) {
      const DebtItem& d = debts[static_cast<std::size_t>(i)];
      Rectangle rr{listR.x + 4.0f, ly, listR.width - 8.0f, static_cast<float>(rowH - 2)};
      const bool selected = (i == m_bondsSelection);

      const int daily = std::max(0, std::min(d.dailyPayment, d.balance));
      const char* label =
          TextFormat("#%d  $%d  %dd  ~$%d/d  %.2f%%", d.id, d.balance, d.daysLeft, daily, d.aprBasisPoints / 100.0f);

      if (ui::Button(24160 + i, rr, label, mouseUi, uiTime, true, selected)) {
        m_bondsSelection = i;
      }

      ly += static_cast<float>(rowH);
    }
  }

  y += static_cast<int>(listR.height) + 10;

  // --- Actions ---
  if (!debts.empty()) {
    const int sel = std::clamp(m_bondsSelection, 0, static_cast<int>(debts.size()) - 1);
    const DebtItem d = debts[static_cast<std::size_t>(sel)]; // copy (safe if buttons mutate the vector)

    ui::Text(x0, y, 14, TextFormat("Selected: Bond #%d   Balance: $%d   Days left: %d", d.id, d.balance, d.daysLeft),
             uiTh.textDim, false, true, 1);
    y += 18;

    Rectangle btn1{static_cast<float>(x0), static_cast<float>(y), static_cast<float>((w - gap * 2) / 3), 22.0f};
    Rectangle btn2{btn1.x + btn1.width + static_cast<float>(gap), btn1.y, btn1.width, 22.0f};
    Rectangle btn3{btn2.x + btn2.width + static_cast<float>(gap), btn1.y, btn1.width, 22.0f};

    const bool canPay = (money > 0 && d.balance > 0);

    if (ui::Button(24180, btn1, "$100", mouseUi, uiTime, canPay, false)) {
      payDownBond(static_cast<std::size_t>(sel), 100);
    }
    if (ui::Button(24181, btn2, "$500", mouseUi, uiTime, canPay, false)) {
      payDownBond(static_cast<std::size_t>(sel), 500);
    }

    const bool canPayOff = (money >= d.balance && d.balance > 0);
    if (ui::Button(24182, btn3, "Pay off", mouseUi, uiTime, canPayOff, true)) {
      payDownBond(static_cast<std::size_t>(sel), d.balance);
    }

    y += 28;
  }

  // Footnote.
  ui::Text(x0, static_cast<int>(rect.y + rect.height) - 18, 12,
           "Debt service is applied each sim-day as an expense (see the Report panel).", uiTh.textFaint, false, false, 0);
}



void Game::unloadSaveMenuThumbnails()
{
  auto unloadVec = [&](std::vector<SaveMenuSlot>& v) {
    for (SaveMenuSlot& e : v) {
      if (e.thumbLoaded && e.thumb.id != 0) {
        UnloadTexture(e.thumb);
      }
      e.thumb = Texture2D{};
      e.thumbLoaded = false;
    }
  };

  unloadVec(m_saveMenuManual);
  unloadVec(m_saveMenuAutos);
}

void Game::refreshSaveMenu()
{
  namespace fs = std::filesystem;

  unloadSaveMenuThumbnails();
  m_saveMenuManual.clear();
  m_saveMenuAutos.clear();

  auto ageTextForPath = [&](const std::string& path) -> std::string {
    std::error_code ec;
    const fs::file_time_type ft = fs::last_write_time(fs::path(path), ec);
    if (ec) return std::string("(unknown time)");

    const auto now = fs::file_time_type::clock::now();
    auto d = (now >= ft) ? (now - ft) : (ft - now);
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(d).count();
    if (sec < 60) return std::string("just now");
    if (sec < 3600) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%lldm ago", static_cast<long long>(sec / 60));
      return std::string(buf);
    }
    if (sec < 86400) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%lldh ago", static_cast<long long>(sec / 3600));
      return std::string(buf);
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%lldd ago", static_cast<long long>(sec / 86400));
    return std::string(buf);
  };

  auto fill = [&](std::vector<SaveMenuSlot>& out, bool autosave, int minSlot, int maxSlot,
                  auto pathForSlot) {
    for (int slot = minSlot; slot <= maxSlot; ++slot) {
      SaveMenuSlot e;
      e.slot = slot;
      e.autosave = autosave;
      e.path = pathForSlot(slot);
      e.thumbPath = thumbPathForSavePath(e.path);

      std::error_code ec;
      e.exists = fs::exists(fs::path(e.path), ec) && !ec;

      if (e.exists) {
        std::string err;
        e.summaryOk = ReadSaveSummary(e.path, e.summary, err, true);
        e.crcChecked = e.summary.crcChecked;
        e.crcOk = e.summary.crcOk;
        e.timeText = ageTextForPath(e.path);
      } else {
        e.summaryOk = false;
        e.timeText = std::string("(empty)");
      }

      // Load thumbnail if present.
      if (fs::exists(fs::path(e.thumbPath), ec) && !ec) {
        e.thumb = LoadTexture(e.thumbPath.c_str());
        e.thumbLoaded = (e.thumb.id != 0);
      }

      out.push_back(std::move(e));
    }
  };

  fill(m_saveMenuManual, false, kSaveSlotMin, kSaveSlotMax, [&](int s) { return savePathForSlot(s); });
  fill(m_saveMenuAutos, true, kAutosaveSlotMin, kAutosaveSlotMax, [&](int s) { return autosavePathForSlot(s); });

  // Clamp selection indices.
  const int manualCount = static_cast<int>(m_saveMenuManual.size());
  const int autoCount = static_cast<int>(m_saveMenuAutos.size());
  if (m_saveMenuGroup == 0) {
    m_saveMenuSelection = std::clamp(m_saveMenuSelection, 0, std::max(0, manualCount - 1));
  } else {
    m_saveMenuSelection = std::clamp(m_saveMenuSelection, 0, std::max(0, autoCount - 1));
  }
}

void Game::drawSaveMenuPanel(int screenW, int screenH)
{
  if (!m_showSaveMenu) return;

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();

  const int panelW = 760;
  const int panelH = 420;
  const int x0 = (screenW - panelW) / 2;
  // Center vertically so the panel looks reasonable across different window sizes.
  const int y0 = std::max(24, (screenH - panelH) / 2);

  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                         static_cast<float>(panelH)};
  ui::DrawPanel(panelR, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panelR, "Save Manager", uiTime, /*active=*/true, /*titleSizePx=*/22);

  int x = x0 + 12;
  int y = y0 + 42;

  const char* tabName = (m_saveMenuGroup == 0) ? "Manual" : "Autosaves";
  ui::Text(
      x, y, 15,
      TextFormat("Tab: switch  |  Up/Down: select  |  Enter/F9: load  |  F5: save  |  Del: delete  |  Group: %s",
                 tabName),
      uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 22;

  const int listW = 470;
  const int previewX = x0 + listW + 24;
  const int previewY = y;
  const int previewW = panelW - listW - 36;
  const int previewH = panelH - (previewY - y0) - 14;

  const int listH = panelH - (y - y0) - 14;
  const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(listW),
                        static_cast<float>(listH)};
  ui::DrawPanelInset(listR, uiTime, /*active=*/true);

  const std::vector<SaveMenuSlot>& list = (m_saveMenuGroup == 0) ? m_saveMenuManual : m_saveMenuAutos;
  const int rows = static_cast<int>(list.size());
  const int rowH = 52;
  const int rowX = x0 + 18;
  int rowY = y + 6;

  for (int i = 0; i < rows; ++i) {
    const SaveMenuSlot& e = list[static_cast<std::size_t>(i)];
    const bool sel = (i == m_saveMenuSelection);
    if (sel) {
      ui::DrawSelectionHighlight(Rectangle{static_cast<float>(rowX - 4), static_cast<float>(rowY - 2),
                                           static_cast<float>(listW - 12), static_cast<float>(rowH - 2)},
                                 uiTime, /*strong=*/true);
    }

    const char* slotLabel = e.autosave ? "Auto" : "Slot";
    ui::Text(rowX, rowY, 18, TextFormat("%s %d", slotLabel, e.slot), sel ? uiTh.text : uiTh.textDim,
             /*bold=*/sel, /*shadow=*/true, 1);

    if (!e.exists) {
      ui::Text(rowX + 90, rowY + 2, 16, "(empty)", Color{180, 180, 180, 255}, /*bold=*/false, /*shadow=*/true, 1);
    } else if (!e.summaryOk) {
      ui::Text(rowX + 90, rowY + 2, 16, "(unreadable)", Color{255, 120, 120, 255}, /*bold=*/false, /*shadow=*/true, 1);
    } else {
      const Stats& s = e.summary.stats;
      ui::Text(rowX + 90, rowY + 2, 16,
               TextFormat("Day %d  Pop %d  $%d  Happy %.0f%%", s.day, s.population, s.money,
                          static_cast<double>(s.happiness * 100.0f)),
               Color{210, 210, 210, 255}, /*bold=*/false, /*shadow=*/true, 1);
    }

    // Right-aligned metadata.
    Color meta = Color{180, 180, 180, 255};
    if (e.crcChecked && !e.crcOk) meta = Color{255, 90, 90, 255};

    const char* crcText = (e.crcChecked && !e.crcOk) ? "CORRUPT" : nullptr;
    if (crcText) {
      ui::Text(x0 + listW - 40, rowY + 2, 14, crcText, meta, /*bold=*/true, /*shadow=*/true, 1);
    }
    ui::Text(x0 + listW - 140, rowY + 24, 14, e.timeText.c_str(), meta, /*bold=*/false, /*shadow=*/true, 1);

    rowY += rowH;
  }

  // Preview panel
  const Rectangle previewR{static_cast<float>(previewX), static_cast<float>(previewY), static_cast<float>(previewW),
                           static_cast<float>(previewH)};
  ui::DrawPanelInset(previewR, uiTime, /*active=*/true);
  ui::Text(previewX + 8, previewY + 6, 18, "Preview", uiTh.text, /*bold=*/true, /*shadow=*/true, 1);

  if (!list.empty()) {
    const int idx = std::clamp(m_saveMenuSelection, 0, static_cast<int>(list.size()) - 1);
    const SaveMenuSlot& e = list[static_cast<std::size_t>(idx)];

    int py = previewY + 30;
    ui::Text(previewX + 8, py, 14, TextFormat("Path: %s", e.path.c_str()), uiTh.textDim, /*bold=*/false,
             /*shadow=*/true, 1);
    py += 18;

    if (e.exists && e.summaryOk) {
      const Stats& s = e.summary.stats;
      ui::Text(previewX + 8, py, 14, TextFormat("Seed: %llu", static_cast<unsigned long long>(e.summary.seed)),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
      py += 18;
      ui::Text(previewX + 8, py, 14, TextFormat("Day %d | Pop %d | Money %d", s.day, s.population, s.money),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
      py += 18;
      ui::Text(previewX + 8, py, 14, TextFormat("Happiness: %.0f%%", static_cast<double>(s.happiness * 100.0f)),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
      py += 18;
    }

    if (e.thumbLoaded && e.thumb.id != 0) {
      const int margin = 12;
      Rectangle dst{static_cast<float>(previewX + margin), static_cast<float>(py + 8),
                    static_cast<float>(previewW - margin * 2),
                    static_cast<float>(previewH - (py - previewY) - 18)};

      const float sx = dst.width / static_cast<float>(e.thumb.width);
      const float sy = dst.height / static_cast<float>(e.thumb.height);
      const float s = std::min(sx, sy);
      const float w = static_cast<float>(e.thumb.width) * s;
      const float h = static_cast<float>(e.thumb.height) * s;
      const float dx = dst.x + (dst.width - w) * 0.5f;
      const float dy = dst.y + (dst.height - h) * 0.5f;

      DrawTextureEx(e.thumb, Vector2{dx, dy}, 0.0f, s, RAYWHITE);
      DrawRectangleLinesEx(Rectangle{dx, dy, w, h}, 1, Color{255, 255, 255, 80});
    } else {
      ui::Text(previewX + 8, py + 18, 14, "(no thumbnail)", Color{180, 180, 180, 255}, /*bold=*/false,
               /*shadow=*/true, 1);
    }
  }
}


namespace {

inline float U32ToUnitFloat(std::uint32_t u)
{
  // [0,1)
  return static_cast<float>(u) / 4294967296.0f;
}

// NOTE: Weight callback is a generic callable so we can pass lambdas directly.
// A function-pointer parameter would force callers to explicitly convert lambdas
// (MSVC/GCC do not consider that conversion during template argument deduction).
template <typename Item, typename GetWeight>
int PickWeightedIndex(std::uint64_t& rngState, const std::vector<Item>& items, std::uint64_t totalWeight,
                      GetWeight&& getWeight)
{
  if (items.empty()) return -1;
  if (totalWeight == 0) return -1;

  const std::uint64_t r = SplitMix64Next(rngState) % totalWeight;
  std::uint64_t acc = 0;
  for (int i = 0; i < static_cast<int>(items.size()); ++i) {
    const int w = std::max(0, static_cast<int>(getWeight(items[static_cast<std::size_t>(i)])));
    acc += static_cast<std::uint64_t>(w);
    if (r < acc) return i;
  }
  return static_cast<int>(items.size()) - 1;
}

bool BuildPathFollowingParents(int startRoadIdx, int w, int h, const std::vector<int>& parent,
                              std::vector<Point>& outPath)
{
  outPath.clear();
  if (w <= 0 || h <= 0) return false;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (parent.size() != n) return false;
  if (startRoadIdx < 0 || static_cast<std::size_t>(startRoadIdx) >= n) return false;

  int cur = startRoadIdx;
  int guard = 0;
  while (cur != -1 && guard++ < static_cast<int>(n) + 8) {
    const int x = cur % w;
    const int y = cur / w;
    outPath.push_back(Point{x, y});
    const std::size_t ui = static_cast<std::size_t>(cur);
    if (ui >= parent.size()) break;
    cur = parent[ui];
  }
  return outPath.size() >= 2;
}

} // namespace

void Game::rebuildVehiclesRoutingCache()
{
  m_vehiclesDirty = false;

  // Ensure road graph + routing helpers are ready for per-vehicle A* routes.
  ensureRoadGraphUpToDate();
  m_vehicleSpawnAccum = 0.0f;

  m_vehicles.clear();

  m_commuteJobSources.clear();
  m_commuteOrigins.clear();
  m_commuteOriginWeightTotal = 0;
  m_commuteField = RoadFlowField{};

  m_goodsProducerRoads.clear();
  m_goodsProducerSupply.clear();
  m_goodsProducerWeightTotal = 0;
  m_goodsProducerField = RoadFlowField{};

  m_goodsConsumers.clear();
  m_goodsConsumerWeightTotal = 0;

  m_goodsEdgeSources.clear();
  m_goodsEdgeField = RoadFlowField{};

  const int w = m_world.width();
  const int h = m_world.height();
  if (w <= 0 || h <= 0) return;
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  // Outside-connection constraint mirrors the core simulation.
  const bool requireOutside = m_sim.config().requireOutsideConnection;
  std::vector<std::uint8_t> roadToEdgeLocal;
  const std::vector<std::uint8_t>* roadToEdge = nullptr;
  if (requireOutside) {
    ComputeRoadsConnectedToEdge(m_world, roadToEdgeLocal);
    roadToEdge = &roadToEdgeLocal;
  }

  // Zone access: allows interior tiles of a connected zoned area to be reachable via a
  // road-adjacent boundary tile.
  const ZoneAccessMap zoneAccess = BuildZoneAccessMap(m_world, roadToEdge);

  auto isTraversableRoad = [&](int ridx) -> bool {
    if (ridx < 0 || static_cast<std::size_t>(ridx) >= n) return false;
    const int x = ridx % w;
    const int y = ridx / w;
    if (!m_world.inBounds(x, y)) return false;
    if (m_world.at(x, y).overlay != Overlay::Road) return false;
    if (requireOutside) {
      if (!roadToEdge || roadToEdge->size() != n) return false;
      if ((*roadToEdge)[static_cast<std::size_t>(ridx)] == 0) return false;
    }
    return true;
  };

  auto zoneHasAccess = [&](int zx, int zy) -> bool {
    return HasZoneAccess(zoneAccess, zx, zy);
  };


  // --- Commute routing: sources are road tiles adjacent to commercial/industrial zones ---
  std::vector<std::uint8_t> isJobSource(n, std::uint8_t{0});
  m_commuteJobSources.reserve(n / 16);

  const int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Commercial && t.overlay != Overlay::Industrial) continue;
      if (!zoneHasAccess(x, y)) continue;

      bool addedAdjacent = false;

      // Prefer direct road adjacency when present (keeps visuals close to the core traffic model).
      for (const auto& d : dirs) {
        const int rx = x + d[0];
        const int ry = y + d[1];
        if (!m_world.inBounds(rx, ry)) continue;
        if (m_world.at(rx, ry).overlay != Overlay::Road) continue;
        const int ridx = ry * w + rx;
        if (!isTraversableRoad(ridx)) continue;
        const std::size_t ui = static_cast<std::size_t>(ridx);
        if (ui >= isJobSource.size()) continue;
        if (isJobSource[ui]) continue;
        isJobSource[ui] = 1;
        m_commuteJobSources.push_back(ridx);
        addedAdjacent = true;
      }

      // If the job tile is interior (no adjacent road), fall back to its propagated access road.
      if (!addedAdjacent) {
        const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
        if (zidx < zoneAccess.roadIdx.size()) {
          const int ridx = zoneAccess.roadIdx[zidx];
          if (isTraversableRoad(ridx)) {
            const std::size_t ui = static_cast<std::size_t>(ridx);
            if (ui < isJobSource.size() && !isJobSource[ui]) {
              isJobSource[ui] = 1;
              m_commuteJobSources.push_back(ridx);
            }
          }
        }
      }
    }
  }

  RoadFlowFieldConfig commuteCfg;
  commuteCfg.requireOutsideConnection = requireOutside;
  commuteCfg.computeOwner = true;
  commuteCfg.useTravelTime = true;
  m_commuteField = BuildRoadFlowField(m_world, m_commuteJobSources, commuteCfg, roadToEdge);

  // Origins: residential zones with occupants.
  const float employedShare = (m_world.stats().population > 0)
                                  ? (static_cast<float>(m_world.stats().employed) /
                                     static_cast<float>(m_world.stats().population))
                                  : 0.0f;

  const std::uint32_t seedMix = static_cast<std::uint32_t>(m_world.seed() ^ (m_world.seed() >> 32));
  m_commuteOrigins.reserve(n / 16);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Residential) continue;
      if (t.occupants == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      int ridx = -1;
      Point road{};
      if (PickAdjacentRoadTile(m_world, roadToEdge, x, y, road)) {
        ridx = road.y * w + road.x;
      } else if (zidx < zoneAccess.roadIdx.size()) {
        ridx = zoneAccess.roadIdx[zidx];
      }
      if (!isTraversableRoad(ridx)) continue;

      if (m_commuteField.dist.empty() || static_cast<std::size_t>(ridx) >= m_commuteField.dist.size()) continue;
      if (m_commuteField.dist[static_cast<std::size_t>(ridx)] < 0) continue; // unreachable to any job

      const float desired = static_cast<float>(t.occupants) * std::clamp(employedShare, 0.0f, 1.0f);
      int commuters = static_cast<int>(std::floor(desired));
      const float frac = desired - static_cast<float>(commuters);
      if (frac > 0.0f) {
        const std::uint32_t h32 = HashCoords32(x, y, seedMix);
        if (U32ToUnitFloat(h32) < frac) commuters += 1;
      }
      commuters = std::clamp(commuters, 0, static_cast<int>(t.occupants));
      if (commuters <= 0) continue;

      m_commuteOrigins.emplace_back(ridx, commuters);
      m_commuteOriginWeightTotal += static_cast<std::uint64_t>(commuters);
    }
  }

  // --- Goods routing (mirrors the core goods model closely enough for visuals) ---
  GoodsConfig gc;
  gc.requireOutsideConnection = requireOutside;
  // Keep allowImports/allowExports as defaults.

  std::vector<int> supplyPerRoad(n, 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Industrial) continue;
      if (t.level == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      int ridx = -1;
      Point road{};
      if (PickAdjacentRoadTile(m_world, roadToEdge, x, y, road)) {
        ridx = road.y * w + road.x;
      } else if (zidx < zoneAccess.roadIdx.size()) {
        ridx = zoneAccess.roadIdx[zidx];
      }
      if (!isTraversableRoad(ridx)) continue;

      const float raw = static_cast<float>(12 * std::clamp(static_cast<int>(t.level), 0, 3)) * gc.supplyScale;
      const int supply = std::max(0, static_cast<int>(std::lround(raw)));
      if (supply <= 0) continue;
      supplyPerRoad[static_cast<std::size_t>(ridx)] += supply;
    }
  }

  for (int ridx = 0; ridx < static_cast<int>(n); ++ridx) {
    const int supply = supplyPerRoad[static_cast<std::size_t>(ridx)];
    if (supply <= 0) continue;
    if (!isTraversableRoad(ridx)) continue;
    m_goodsProducerRoads.push_back(ridx);
    m_goodsProducerSupply.push_back(supply);
    m_goodsProducerWeightTotal += static_cast<std::uint64_t>(supply);
  }

  RoadFlowFieldConfig prodCfg;
  prodCfg.requireOutsideConnection = requireOutside;
  prodCfg.computeOwner = true;
  prodCfg.useTravelTime = true;
  m_goodsProducerField = BuildRoadFlowField(m_world, m_goodsProducerRoads, prodCfg, roadToEdge);

  m_goodsConsumers.reserve(n / 16);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Commercial) continue;
      if (t.level == 0) continue;
      if (!zoneHasAccess(x, y)) continue;

      const float raw = static_cast<float>(8 * std::clamp(static_cast<int>(t.level), 0, 3)) * gc.demandScale;
      const int demand = std::max(0, static_cast<int>(std::lround(raw)));
      if (demand <= 0) continue;

      const std::size_t zidx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      int ridx = -1;
      Point road{};
      if (PickAdjacentRoadTile(m_world, roadToEdge, x, y, road)) {
        ridx = road.y * w + road.x;
      } else if (zidx < zoneAccess.roadIdx.size()) {
        ridx = zoneAccess.roadIdx[zidx];
      }
      if (!isTraversableRoad(ridx)) continue;

      const int d = (!m_goodsProducerRoads.empty() && static_cast<std::size_t>(ridx) < m_goodsProducerField.dist.size())
                        ? m_goodsProducerField.dist[static_cast<std::size_t>(ridx)]
                        : -1;
      const int own = (d >= 0 && static_cast<std::size_t>(ridx) < m_goodsProducerField.owner.size())
                          ? m_goodsProducerField.owner[static_cast<std::size_t>(ridx)]
                          : -1;

      m_goodsConsumers.push_back(GoodsConsumerLite{ridx, demand, d, own});
      m_goodsConsumerWeightTotal += static_cast<std::uint64_t>(demand);
    }
  }

  // Edge routing (imports/exports) uses border roads as sources.
  m_goodsEdgeSources.reserve(static_cast<std::size_t>(w + h) * 2u);
  auto pushEdge = [&](int ex, int ey) {
    const int ridx = ey * w + ex;
    if (!isTraversableRoad(ridx)) return;
    m_goodsEdgeSources.push_back(ridx);
  };

  for (int x = 0; x < w; ++x) {
    pushEdge(x, 0);
    if (h > 1) pushEdge(x, h - 1);
  }
  for (int y = 1; y < h - 1; ++y) {
    pushEdge(0, y);
    if (w > 1) pushEdge(w - 1, y);
  }

  if (gc.allowImports || gc.allowExports) {
    RoadFlowFieldConfig edgeCfg;
    edgeCfg.requireOutsideConnection = requireOutside;
    edgeCfg.computeOwner = false;
    edgeCfg.useTravelTime = true;
    m_goodsEdgeField = BuildRoadFlowField(m_world, m_goodsEdgeSources, edgeCfg, roadToEdge);
  }
}

namespace {

// Removes points that don't change direction (collinear steps), to keep vehicle polylines short.
// This is purely visual and assumes grid-adjacent steps.
static std::vector<Point> simplifyPath(const std::vector<Point>& path)
{
  if (path.size() < 3) {
    return path;
  }

  auto sgn = [](int v) { return (v > 0) - (v < 0); };

  std::vector<Point> out;
  out.reserve(path.size());
  out.push_back(path.front());

  int prevDx = sgn(path[1].x - path[0].x);
  int prevDy = sgn(path[1].y - path[0].y);

  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const int dx = sgn(path[i + 1].x - path[i].x);
    const int dy = sgn(path[i + 1].y - path[i].y);
    if (dx != prevDx || dy != prevDy) {
      out.push_back(path[i]);
      prevDx = dx;
      prevDy = dy;
    }
  }

  out.push_back(path.back());
  return out;
}

} // namespace

void Game::updateVehicles(float dt)
{
  if (!m_showVehicles) return;

  if (m_vehiclesDirty) {
    rebuildVehiclesRoutingCache();
  }

  // --- Integrate movement ---
  if (dt > 0.0f) {
    std::vector<Vehicle> alive;
    alive.reserve(m_vehicles.size());

    for (Vehicle& v : m_vehicles) {
      if (v.path.size() < 2) continue;

      const float maxS = static_cast<float>(static_cast<int>(v.path.size()) - 1);
      v.s += v.dir * v.speed * dt;

      bool keep = true;
      if (v.s >= maxS) {
        v.s = maxS;
        if (v.kind == VehicleKind::Commute && v.turnsRemaining > 0) {
          v.dir = -1.0f;
          v.turnsRemaining -= 1;
        } else {
          keep = false;
        }
      } else if (v.s <= 0.0f) {
        v.s = 0.0f;
        // Commute vehicles despawn when they return to the origin.
        if (v.kind == VehicleKind::Commute && v.dir < 0.0f) {
          keep = false;
        }
      }

      if (keep) alive.push_back(std::move(v));
    }

    m_vehicles.swap(alive);
  }

  // Don't spawn while paused / painting (dt==0).
  if (dt <= 0.0f) return;

  // --- Targets ---
  int targetCommute = std::clamp(m_world.stats().commuters / kCommutersPerCar, 0, kMaxCommuteVehicles);
  int targetGoods = std::clamp((m_world.stats().goodsDelivered + m_world.stats().goodsExported) / kGoodsPerTruck,
                               0, kMaxGoodsVehicles);

  if (m_commuteJobSources.empty() || m_commuteOrigins.empty()) targetCommute = 0;
  if (m_goodsConsumers.empty()) targetGoods = 0;

  int curCommute = 0;
  int curGoods = 0;
  for (const Vehicle& v : m_vehicles) {
    if (v.kind == VehicleKind::Commute)
      ++curCommute;
    else
      ++curGoods;
  }

  auto makeVehicle = [&](VehicleKind kind, std::vector<Point>&& path, float baseSpeed, int turns) {
    Vehicle v;
    v.kind = kind;
    v.path = std::move(path);
    v.s = 0.0f;
    v.dir = 1.0f;
    v.speed = std::max(0.5f, baseSpeed + RandRange(m_vehicleRngState, -0.75f, 0.75f));
    v.laneOffset = RandRange(m_vehicleRngState, -5.0f, 5.0f);
    v.style = static_cast<int>(SplitMix64Next(m_vehicleRngState) & 0x7FFFFFFFu);
    v.turnsRemaining = turns;
    m_vehicles.push_back(std::move(v));
  };

  auto speedMultForPath = [&](const std::vector<Point>& path) -> float {
    float sum = 0.0f;
    int count = 0;
    for (const Point& p : path) {
      if (!m_world.inBounds(p.x, p.y)) continue;
      const Tile& t = m_world.at(p.x, p.y);
      if (t.overlay != Overlay::Road) continue;
      sum += RoadSpeedMultiplierForLevel(static_cast<int>(t.level));
      ++count;
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 1.0f;
  };

  auto getOriginWeight = [](const std::pair<int, int>& p) -> int { return p.second; };
  auto getConsumerWeight = [](const GoodsConsumerLite& c) -> int { return c.demand; };
  auto getProducerWeight = [&](const int& roadIdx) -> int {
    (void)roadIdx;
    return 1;
  };

  // Vehicle route geometry: use A* on the road graph to build point-to-point routes.
  const int worldW = m_world.width();
  const int worldH = m_world.height();
  if (worldW <= 0 || worldH <= 0) return;

  ensureRoadGraphUpToDate();

  RoadRouteConfig routeCfg;
  routeCfg.metric = RoadRouteMetric::TravelTime;

  auto routeRoadIdx = [&](int startIdx, int goalIdx, std::vector<Point>& outPath) -> bool {
    const int n = worldW * worldH;
    if (startIdx < 0 || goalIdx < 0 || startIdx >= n || goalIdx >= n) return false;
    if (startIdx == goalIdx) return false;

    const Point start{startIdx % worldW, startIdx / worldW};
    const Point goal{goalIdx % worldW, goalIdx / worldW};
    RoadRouteResult rr = FindRoadRouteAStar(m_world, m_roadGraph, m_roadGraphIndex, m_roadGraphWeights, start, goal, routeCfg);
    if (rr.path.size() < 2) return false;

    outPath = std::move(rr.path);
    return true;
  };

  auto traceRoot = [&](int startIdx, const std::vector<int>& parent) -> int {
    const int n = worldW * worldH;
    if (static_cast<int>(parent.size()) != n) return -1;
    int cur = startIdx;
    for (int guard = 0; guard < n + 8; ++guard) {
      if (cur < 0 || cur >= n) return -1;
      const int p = parent[static_cast<std::size_t>(cur)];
      if (p < 0) return cur;
      cur = p;
    }
    return -1;
  };

  int commuteVehiclesSpawnedThisFrame = 0;

  auto spawnCommute = [&]() -> bool {
    if (m_commuteField.dist.empty() || m_commuteField.parent.empty()) return false;

    const int w = m_commuteField.w;
    const int h = m_commuteField.h;
    const auto& parent = m_commuteField.parent;

    const int idx = PickWeightedIndex(m_vehicleRngState, m_commuteOrigins, m_commuteOriginWeightTotal, getOriginWeight);
    if (idx < 0 || static_cast<std::size_t>(idx) >= m_commuteOrigins.size()) return false;

    const int startRoadIdx = m_commuteOrigins[static_cast<std::size_t>(idx)].first;

    // Preferred destination: nearest job source for this origin (flow-field owner).
    int goalRoadIdx = -1;
    if (!m_commuteField.owner.empty()) {
      const int owner = m_commuteField.owner[static_cast<std::size_t>(startRoadIdx)];
      if (owner >= 0 && owner < static_cast<int>(m_commuteJobSources.size())) {
        goalRoadIdx = m_commuteJobSources[static_cast<std::size_t>(owner)];
      }
    }

    std::vector<Point> path;
    if (goalRoadIdx >= 0) {
      if (!routeRoadIdx(startRoadIdx, goalRoadIdx, path)) {
        // Fallback: follow flow-field parents (still reaches a job source).
        if (!BuildPathFollowingParents(startRoadIdx, w, h, parent, path)) return false;
      }
    } else {
      if (!BuildPathFollowingParents(startRoadIdx, w, h, parent, path)) return false;
    }

    if (path.size() < 2) return false;
    path = simplifyPath(path);

    const float baseSpeed = 7.5f * speedMultForPath(path);
    makeVehicle(VehicleKind::Commute, std::move(path), baseSpeed, 1);
    ++commuteVehiclesSpawnedThisFrame;
    return true;
  };

  auto spawnGoods = [&]() -> bool {
    const int delivered = std::max(0, m_world.stats().goodsDelivered);
    const int imported = std::max(0, m_world.stats().goodsImported);
    const int exported = std::max(0, m_world.stats().goodsExported);
    const int goodsTotal = delivered + exported;
    if (goodsTotal <= 0) return false;

    const float exportFrac = (goodsTotal > 0) ? (static_cast<float>(exported) / static_cast<float>(goodsTotal)) : 0.0f;
    const float importFrac = (delivered > 0) ? (static_cast<float>(imported) / static_cast<float>(delivered)) : 0.0f;

    const bool wantExport = (Rand01(m_vehicleRngState) < exportFrac);

    // Export: producer -> edge.
    if (wantExport) {
      if (m_goodsProducerRoads.empty()) return false;
      if (m_goodsEdgeField.parent.empty() || m_goodsEdgeField.dist.empty()) return false;

      // Pick producer weighted by supply.
      if (m_goodsProducerSupply.size() != m_goodsProducerRoads.size() || m_goodsProducerWeightTotal == 0) return false;
      // Build a temporary view of producer indices for weighted picking.
      struct ProducerRef { int idx; int w; };
      std::vector<ProducerRef> refs;
      refs.reserve(m_goodsProducerRoads.size());
      for (int i = 0; i < static_cast<int>(m_goodsProducerRoads.size()); ++i) {
        refs.push_back(ProducerRef{i, m_goodsProducerSupply[static_cast<std::size_t>(i)]});
      }
      auto getW = [](const ProducerRef& r) -> int { return r.w; };
      const int pi = PickWeightedIndex(m_vehicleRngState, refs, m_goodsProducerWeightTotal, getW);
      if (pi < 0 || pi >= static_cast<int>(refs.size())) return false;
      const int pidx = refs[static_cast<std::size_t>(pi)].idx;
      if (pidx < 0 || pidx >= static_cast<int>(m_goodsProducerRoads.size())) return false;
      const int start = m_goodsProducerRoads[static_cast<std::size_t>(pidx)];
      if (start < 0 || static_cast<std::size_t>(start) >= m_goodsEdgeField.dist.size()) return false;
      if (m_goodsEdgeField.dist[static_cast<std::size_t>(start)] < 0) return false;

      const int end = traceRoot(start, m_goodsEdgeField.parent);
      if (end < 0) return false;

      std::vector<Point> path;
      if (!routeRoadIdx(start, end, path)) {
        if (!BuildPathFollowingParents(start, m_goodsEdgeField.w, m_goodsEdgeField.h, m_goodsEdgeField.parent, path)) return false;
      }
      if (path.size() < 2) return false;
      const float baseSpeed = 5.5f * speedMultForPath(path);
      makeVehicle(VehicleKind::GoodsExport, std::move(path), baseSpeed, 0);
      return true;
    }

    // Delivery: (producer or edge) -> consumer.
    if (m_goodsConsumers.empty()) return false;
    const int ci = PickWeightedIndex(m_vehicleRngState, m_goodsConsumers, m_goodsConsumerWeightTotal, getConsumerWeight);
    if (ci < 0 || ci >= static_cast<int>(m_goodsConsumers.size())) return false;
    const GoodsConsumerLite& c = m_goodsConsumers[static_cast<std::size_t>(ci)];
    if (c.roadIdx < 0) return false;

    const bool preferImport = (Rand01(m_vehicleRngState) < importFrac);

    auto tryImport = [&]() -> bool {
      if (m_goodsEdgeField.parent.empty() || m_goodsEdgeField.dist.empty()) return false;
      if (static_cast<std::size_t>(c.roadIdx) >= m_goodsEdgeField.dist.size()) return false;
      if (m_goodsEdgeField.dist[static_cast<std::size_t>(c.roadIdx)] < 0) return false;
      const int start = traceRoot(c.roadIdx, m_goodsEdgeField.parent);
      if (start < 0) return false;

      std::vector<Point> path;
      if (!routeRoadIdx(start, c.roadIdx, path)) {
        if (!BuildPathFollowingParents(c.roadIdx, m_goodsEdgeField.w, m_goodsEdgeField.h, m_goodsEdgeField.parent, path)) return false;
        std::reverse(path.begin(), path.end());
      }
      if (path.size() < 2) return false;
      const float baseSpeed = 5.0f * speedMultForPath(path);
      makeVehicle(VehicleKind::GoodsImport, std::move(path), baseSpeed, 0);
      return true;
    };

    auto tryLocal = [&]() -> bool {
      if (m_goodsProducerRoads.empty()) return false;
      if (m_goodsProducerField.parent.empty() || m_goodsProducerField.dist.empty() || m_goodsProducerField.owner.empty()) return false;
      if (static_cast<std::size_t>(c.roadIdx) >= m_goodsProducerField.dist.size()) return false;
      if (m_goodsProducerField.dist[static_cast<std::size_t>(c.roadIdx)] < 0) return false;
      const int own = m_goodsProducerField.owner[static_cast<std::size_t>(c.roadIdx)];
      if (own < 0 || own >= static_cast<int>(m_goodsProducerRoads.size())) return false;

      const int start = m_goodsProducerRoads[static_cast<std::size_t>(own)];
      if (start < 0) return false;

      std::vector<Point> path;
      if (!routeRoadIdx(start, c.roadIdx, path)) {
        if (!BuildPathFollowingParents(c.roadIdx, m_goodsProducerField.w, m_goodsProducerField.h, m_goodsProducerField.parent, path)) return false;
        std::reverse(path.begin(), path.end());
      }
      if (path.size() < 2) return false;
      const float baseSpeed = 5.2f * speedMultForPath(path);
      makeVehicle(VehicleKind::GoodsDelivery, std::move(path), baseSpeed, 0);
      return true;
    };

    if (preferImport) {
      if (tryImport()) return true;
      return tryLocal();
    }

    if (tryLocal()) return true;
    return tryImport();
  };

  int spawnBudget = kMaxSpawnPerFrame;

  while (spawnBudget > 0 && curCommute < targetCommute) {
    if (!spawnCommute()) break;
    ++curCommute;
    --spawnBudget;
  }

  while (spawnBudget > 0 && curGoods < targetGoods) {
    if (!spawnGoods()) break;
    ++curGoods;
    --spawnBudget;
  }
}

void Game::appendVehicleSprites(const Camera2D& camera, std::vector<Renderer::WorldSprite>& out)
{
  if (!m_showVehicles) return;
  if (m_vehicles.empty()) return;

  const float zoom = std::max(0.25f, camera.zoom);
  const float invZoom = 1.0f / zoom;

  const float tileScreenW = static_cast<float>(m_cfg.tileWidth) * zoom;

  // Emissive contribution (headlights) based on the renderer's day/night cycle.
  const float night = m_renderer.nightFactor(m_timeSec);
  const bool doEmissive = (night > 0.02f);

  // Worst-case: 2 sprites per vehicle (color + emissive).
  out.reserve(out.size() + m_vehicles.size() * (doEmissive ? 2u : 1u));

  for (const Vehicle& v : m_vehicles) {
    if (v.path.size() < 2) continue;

    const float maxS = static_cast<float>(static_cast<int>(v.path.size()) - 1);
    const float s = std::clamp(v.s, 0.0f, maxS);
    int seg = static_cast<int>(std::floor(s));
    float t = s - static_cast<float>(seg);
    if (seg >= static_cast<int>(v.path.size()) - 1) {
      seg = static_cast<int>(v.path.size()) - 2;
      t = 1.0f;
    }

    const Point a = v.path[static_cast<std::size_t>(seg)];
    const Point b = v.path[static_cast<std::size_t>(seg + 1)];

    const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                 static_cast<float>(m_cfg.tileHeight), m_elev);
    const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                 static_cast<float>(m_cfg.tileHeight), m_elev);

    Vector2 pos{wa.x + (wb.x - wa.x) * t, wa.y + (wb.y - wa.y) * t};
    Vector2 dir{wb.x - wa.x, wb.y - wa.y};

    const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (len > 1e-3f) {
      const Vector2 nrm{-dir.y / len, dir.x / len};
      const float off = (v.laneOffset * invZoom); // keep lane separation roughly constant in screen pixels
      pos.x += nrm.x * off;
      pos.y += nrm.y * off;
    }

    const bool isTruck = (v.kind != VehicleKind::Commute);

    // Sprite scale: keep vehicles readable, but clamp growth at high zoom.
    // (k is a *screen-space* factor, so scale = k/zoom maps it into world space for BeginMode2D.)
    const float k = std::clamp(tileScreenW / 120.0f, 0.16f, 0.55f);
    const float scale = k * invZoom;

    const bool slopePositive = (dir.x * dir.y >= 0.0f); // +45° diag vs -45° diag on screen
    const bool flipX = (dir.x < 0.0f);

    const Renderer::VehicleSprite* spr =
        isTruck ? m_renderer.truckSprite(slopePositive, v.style) : m_renderer.carSprite(slopePositive, v.style);
    if (!spr || spr->color.id == 0) {
      // Texture generation failed (should be rare) - skip.
      continue;
    }

    // Depth anchor: snap to the nearest path node so sprites interleave with the renderer's tile order.
    int anchorIdx = (t > 0.5f) ? (seg + 1) : seg;
    anchorIdx = std::clamp(anchorIdx, 0, static_cast<int>(v.path.size()) - 1);
    const Point anchor = v.path[static_cast<std::size_t>(anchorIdx)];

    const int sortSum = anchor.x + anchor.y;
    const float sortX = static_cast<float>(anchor.x);

    auto pushSprite = [&](const Texture2D& tex, bool emissive) {
      if (tex.id == 0) return;

      Renderer::WorldSprite ws;
      ws.sortSum = sortSum;
      ws.sortX = sortX;
      ws.tex = &tex;
      ws.emissive = emissive;

      Rectangle src{0.0f, 0.0f, static_cast<float>(tex.width), static_cast<float>(tex.height)};
      if (flipX) {
        src.x = static_cast<float>(tex.width);
        src.width = -src.width;
      }

      const int pivotX = flipX ? (tex.width - spr->pivotX) : spr->pivotX;
      const float px = static_cast<float>(pivotX) * scale;
      const float py = static_cast<float>(spr->pivotY) * scale;

      ws.src = src;
      ws.dst = Rectangle{pos.x - px, pos.y - py, static_cast<float>(tex.width) * scale,
                         static_cast<float>(tex.height) * scale};
      ws.origin = Vector2{0.0f, 0.0f};
      ws.rotation = 0.0f;

      Color tint = WHITE;
      if (!emissive) {
        tint.a = 230;
      } else {
        const int a8 = static_cast<int>(std::round(255.0f * std::clamp(night, 0.0f, 1.0f)));
        tint.a = static_cast<unsigned char>(std::clamp(a8, 0, 255));
      }
      ws.tint = tint;

      out.push_back(ws);
    };

    // Base color sprite.
    pushSprite(spr->color, /*emissive=*/false);

    // Optional emissive headlights sprite.
    if (doEmissive) {
      pushSprite(spr->emissive, /*emissive=*/true);
    }
  }
}

void Game::applyToolBrush(int centerX, int centerY)
{
  if (m_tool == Tool::Inspect) return;

  // Terrain editing (Raise/Lower/Smooth) uses modifier keys for strength.
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  const int r = std::max(0, m_brushRadius);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      // Diamond brush (fits iso grid nicely).
      if (std::abs(dx) + std::abs(dy) > r) continue;
      const int tx = centerX + dx;
      const int ty = centerY + dy;

      // Skip out-of-bounds early.
      if (!m_world.inBounds(tx, ty)) continue;

      // Within a single paint stroke, apply at most once per tile. This avoids
      // accidental "multi-upgrades" (zones) if the cursor is held still.
      if (!m_strokeApplied.empty() && m_strokeApplyW == m_world.width() && m_strokeApplyH == m_world.height()) {
        const int idx = ty * m_strokeApplyW + tx;
        if (idx >= 0) {
          const std::size_t uidx = static_cast<std::size_t>(idx);
          if (uidx < m_strokeApplied.size()) {
            if (m_strokeApplied[uidx]) continue;
            m_strokeApplied[uidx] = 1;
          }
        }
      }

      // Capture pre-edit state for undo/redo.
      //
      // Road auto-tiling masks are fixed up locally by EditHistory (undo/redo) so it's
      // sufficient to track the edited tile itself.
      const Overlay beforeOverlay = m_world.at(tx, ty).overlay;
      const Terrain beforeTerrain = m_world.at(tx, ty).terrain;
      const float beforeHeight = m_world.at(tx, ty).height;
      m_history.noteTilePreEdit(m_world, tx, ty);

      bool applied = false;
      ToolApplyResult res = ToolApplyResult::Noop;

      // --- Terraforming tools are handled at the game layer (they need ProcGenConfig thresholds). ---
      if (m_tool == Tool::RaiseTerrain || m_tool == Tool::LowerTerrain || m_tool == Tool::SmoothTerrain) {
        Tile& t = m_world.at(tx, ty);

        auto classifyTerrain = [&](float h) -> Terrain {
          const float wl = std::clamp(m_procCfg.waterLevel, 0.0f, 1.0f);
          const float sl = std::clamp(m_procCfg.sandLevel, 0.0f, 1.0f);
          if (h < wl) return Terrain::Water;
          if (h < std::max(wl, sl)) return Terrain::Sand;
          return Terrain::Grass;
        };

        // Strength modifiers:
        //  - default: medium
        //  - Shift: stronger
        //  - Ctrl: finer
        float delta = 0.05f;
        if (shift) delta = 0.10f;
        if (ctrl) delta = 0.02f;

        float newH = t.height;
        if (m_tool == Tool::RaiseTerrain) {
          newH = std::clamp(t.height + delta, 0.0f, 1.0f);
        } else if (m_tool == Tool::LowerTerrain) {
          newH = std::clamp(t.height - delta, 0.0f, 1.0f);
        } else if (m_tool == Tool::SmoothTerrain) {
          const int w = m_world.width();
          const int h = m_world.height();
          const std::size_t n = static_cast<std::size_t>(w * h);
          if (w > 0 && h > 0 && m_heightSnapshot.size() == n) {
            auto sample = [&](int sx, int sy) -> float {
              if (sx < 0 || sy < 0 || sx >= w || sy >= h) return m_heightSnapshot[static_cast<std::size_t>(ty * w + tx)];
              return m_heightSnapshot[static_cast<std::size_t>(sy * w + sx)];
            };

            // 3x3 neighborhood average from the snapshot so smoothing is order-independent.
            float sum = 0.0f;
            int count = 0;
            for (int oy = -1; oy <= 1; ++oy) {
              for (int ox = -1; ox <= 1; ++ox) {
                sum += sample(tx + ox, ty + oy);
                ++count;
              }
            }
            const float avg = (count > 0) ? (sum / static_cast<float>(count)) : t.height;

            float alpha = 0.5f;
            if (shift) alpha = 0.75f;
            if (ctrl) alpha = 0.25f;

            newH = std::clamp(m_heightSnapshot[static_cast<std::size_t>(ty * w + tx)] + (avg - m_heightSnapshot[static_cast<std::size_t>(ty * w + tx)]) * alpha,
                              0.0f, 1.0f);
          }
        }

        // Apply height.
        t.height = newH;

        // Derive terrain from height thresholds.
        const Terrain newTerrain = classifyTerrain(t.height);
        if (newTerrain == Terrain::Water) {
          // When a tile becomes water we must clear most overlays.
          // Roads are the exception: a Road overlay on a Water tile is treated as a bridge.
          if (t.overlay != Overlay::None && t.overlay != Overlay::Road) {
            m_world.setOverlay(Overlay::None, tx, ty);
            t.overlay = Overlay::None;
            t.level = 1;
            t.occupants = 0;
          }
        }
        t.terrain = newTerrain;

        const Overlay afterOverlay = t.overlay;
        const bool overlayChanged = (afterOverlay != beforeOverlay);
        const bool terrainChanged = (t.terrain != beforeTerrain);
        const bool heightChanged = (t.height != beforeHeight);

        applied = overlayChanged || terrainChanged || heightChanged;

        if (heightChanged) {
          // Flood overlay is derived purely from the heightfield.
          invalidateHydrology();
        }

        if (applied) {
          m_landValueDirty = true;
        }

        const bool roadRelevant = (beforeOverlay == Overlay::Road || afterOverlay == Overlay::Road);
        const bool roadCostChanged = terrainChanged && roadRelevant;

        if (overlayChanged || roadCostChanged) {
          m_trafficDirty = true;
          m_goodsDirty = true;
          m_vehiclesDirty = true;
        }

        // Road-graph routing depends on both topology (roads) and whether they're bridges.
        if ((overlayChanged || roadCostChanged) && roadRelevant) {
          m_roadGraphDirty = true;
        }
      } else {
        if (m_tool == Tool::District) {
          // Districts are a lightweight label layer; they do not run through the economy rules.
          res = m_world.applyDistrict(tx, ty, m_activeDistrict);
          applied = (res == ToolApplyResult::Applied);
        } else {
          // --- Regular tools go through World::applyTool (economy + rules). ---
          res = (m_tool == Tool::Road) ? m_world.applyRoad(tx, ty, m_roadBuildLevel)
                                       : m_world.applyTool(m_tool, tx, ty);
          switch (res) {
          case ToolApplyResult::InsufficientFunds: m_strokeFeedback.noMoney = true; break;
          case ToolApplyResult::BlockedNoRoad: m_strokeFeedback.noRoad = true; break;
          case ToolApplyResult::BlockedWater: m_strokeFeedback.water = true; break;
          case ToolApplyResult::BlockedOccupied: m_strokeFeedback.occupied = true; break;
          default: break;
          }

          applied = (res == ToolApplyResult::Applied);
          if (applied) {
            m_landValueDirty = true;
            // Traffic depends on roads + zones + occupancy.
            m_trafficDirty = true;
            // Goods logistics depend on roads + industrial/commercial zoning.
            m_goodsDirty = true;
            // Moving vehicles (visualization) also depend on roads + zones + occupancy.
            m_vehiclesDirty = true;

            if (m_tool == Tool::Road || (m_tool == Tool::Bulldoze && beforeOverlay == Overlay::Road)) {
              m_roadGraphDirty = true;
            }
          }
        }
      }

      if (applied) {
        // District edits do not affect cached terrain/overlays, so avoid base-cache rebuild churn.
        if (m_tool != Tool::District) {
          m_tilesEditedThisStroke.push_back(Point{tx, ty});
        }
      }
    }
  }
}

void Game::beginPaintStroke()
{
  if (m_painting) return;
  m_painting = true;
  m_strokeFeedback.clear();
  m_tilesEditedThisStroke.clear();
  m_history.beginStroke(m_world);

  if (m_replayCapture.active()) {
    // World hash before any stroke modifications (used as the patch base hash).
    m_replayStrokeBaseHash = HashWorld(m_world, true);
  }

  // Snapshot heights for order-independent smoothing.
  m_heightSnapshot.clear();
  if (m_tool == Tool::SmoothTerrain) {
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));
    m_heightSnapshot.resize(n, 0.0f);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        m_heightSnapshot[static_cast<std::size_t>(y * w + x)] = m_world.at(x, y).height;
      }
    }
  }

  // Per-stroke applied tile mask.
  m_strokeApplyW = m_world.width();
  m_strokeApplyH = m_world.height();
  const std::size_t n =
      static_cast<std::size_t>(std::max(0, m_strokeApplyW) * std::max(0, m_strokeApplyH));
  m_strokeApplied.assign(n, 0);
}

void Game::endPaintStroke()
{
  if (!m_painting) return;
  m_painting = false;

  EditHistory::Command cmd;
  const bool committed = m_history.endStroke(m_world, m_replayCapture.active() ? &cmd : nullptr);

  // A stroke potentially changes many tiles; update the minimap lazily.
  m_renderer.markMinimapDirty();

  // Also refresh the (optional) cached base render for any edited tiles.
  m_renderer.markBaseCacheDirtyForTiles(m_tilesEditedThisStroke, m_world.width(), m_world.height());
  m_tilesEditedThisStroke.clear();

  // The software 3D preview is an expensive but useful "sanity view".
  // Only refresh it when the world changes (stroke ends) or when its throttle
  // timer expires.
  m_3dPreviewDirty = true;
  m_3dPreviewTimer = 0.0f;

  // Height snapshot is only valid for the current stroke.
  m_heightSnapshot.clear();

  m_strokeApplied.clear();
  m_strokeApplyW = 0;
  m_strokeApplyH = 0;

  // Keep HUD numbers (roads/parks/capacities) responsive even before the next sim tick.
  m_sim.refreshDerivedStats(m_world);

  if (committed && m_replayCapture.active()) {
    std::string err;
    (void)m_replayCapture.recordTileCommandPatch(m_world, cmd, m_replayStrokeBaseHash, /*useBeforeAsTarget=*/false,
                                                 err);
  }
  m_replayStrokeBaseHash = 0;

  // Any edit stroke potentially changes travel demand and the road graph.
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;
  m_servicesHeatmapsDirty = true;
  m_fireRiskDirty = true;
  m_servicesPlanDirty = true;
  m_servicesPlanValid = false;
  invalidateAnalysisLayers();

  // Street naming + address geocoding depend on roads/parcels; invalidate them and clear any
  // existing navigation route so we never render stale turn-by-turn guidance.
  m_wayfindingDirty = true;
  clearWayfindingRoute();

  // Provide one toast per stroke for common build failures (no money, no road access, etc.).
  if (m_strokeFeedback.any()) {
    std::string msg = "Some placements failed: ";
    bool first = true;
    auto add = [&](const char* s) {
      if (!first) msg += ", ";
      msg += s;
      first = false;
    };

    if (m_strokeFeedback.noMoney) add("not enough money");
    if (m_strokeFeedback.noRoad) add("need road access");
    if (m_strokeFeedback.water) add("can't build on water");
    if (m_strokeFeedback.occupied) add("tile occupied");

    showToast(msg, 3.0f);
  }
}


// -----------------------------------------------------------------------------
// Blueprint tool helpers
// -----------------------------------------------------------------------------
void Game::clearBlueprint()
{
  m_hasBlueprint = false;
  m_blueprint = Blueprint{};
  m_blueprintTransformed = Blueprint{};
  m_blueprintTransform = BlueprintTransform{};
  m_blueprintTransformedDirty = false;
  m_blueprintSelecting = false;
  m_blueprintSelStart.reset();
  m_blueprintSelEnd = Point{0, 0};

  m_blueprintTilingSelecting = false;
  m_blueprintTileSelStart.reset();
  m_blueprintTileSelEnd = Point{0, 0};
  m_blueprintTileUseLibrary = false;

  // Clearing the blueprint also clears any stale preview.
  m_blueprintLibraryPreviewIndex = -1;
  m_blueprintLibraryPreviewOk = false;
  m_blueprintLibraryPreviewError.clear();
  m_blueprintLibraryPreview = Blueprint{};
}

void Game::updateBlueprintTransformed()
{
  if (!m_hasBlueprint) {
    m_blueprintTransformed = Blueprint{};
    m_blueprintTransformedDirty = false;
    return;
  }

  if (!m_blueprintTransformedDirty) return;

  Blueprint out;
  std::string err;
  if (!TransformBlueprint(m_blueprint, m_blueprintTransform, out, err)) {
    // Fail safe: revert to identity transform.
    m_blueprintTransform = BlueprintTransform{};
    m_blueprintTransformed = m_blueprint;
    m_blueprintTransformedDirty = false;
    showToast(std::string("Blueprint transform failed: ") + err, 3.0f);
    return;
  }

  m_blueprintTransformed = std::move(out);
  m_blueprintTransformedDirty = false;
}

bool Game::stampBlueprintAt(const Point& anchorTile)
{
  if (!m_hasBlueprint) {
    showToast("No blueprint captured", 2.0f);
    return false;
  }

  updateBlueprintTransformed();
  const Blueprint& bp = m_blueprintTransformed;
  if (bp.width <= 0 || bp.height <= 0) {
    showToast("Blueprint is empty", 2.0f);
    return false;
  }

  // Center-anchor the blueprint on the hovered tile.
  const int dstX = anchorTile.x - (bp.width / 2);
  const int dstY = anchorTile.y - (bp.height / 2);

  // Commit any in-progress stroke, then make stamping undoable as a single stroke.
  endPaintStroke();
  beginPaintStroke();

  // Pre-mark tiles for undo/redo tracking and base-cache invalidation.
  for (const auto& d : bp.tiles) {
    const int lx = d.index % bp.width;
    const int ly = d.index / bp.width;
    const int tx = dstX + lx;
    const int ty = dstY + ly;
    if (!m_world.inBounds(tx, ty)) continue;
    m_history.noteTilePreEdit(m_world, tx, ty);
    m_tilesEditedThisStroke.push_back(Point{tx, ty});
  }

  std::string err;
  BlueprintApplyOptions opt = m_blueprintApplyOpt;
  opt.transform = BlueprintTransform{}; // already baked into bp
  const bool ok = ApplyBlueprint(m_world, bp, dstX, dstY, opt, err);
  if (!ok) {
    showToast(std::string("Blueprint stamp failed: ") + err, 3.0f);
    endPaintStroke();
    return false;
  }

  // Stamps can affect everything.
  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_landValueDirty = true;
  invalidateHydrology();
  invalidateAnalysisLayers();
  m_vehiclesDirty = true;

  endPaintStroke();
  showToast(TextFormat("Stamped blueprint (%dx%d, %d tiles)", bp.width, bp.height,
                       static_cast<int>(bp.tiles.size())));
  return true;
}

// -----------------------------------------------------------------------------
// Blueprint library (file-backed stamp collection)
// -----------------------------------------------------------------------------

void Game::refreshBlueprintLibrary()
{
  namespace fs = std::filesystem;

  // Try to preserve the currently selected entry across refreshes.
  std::string prevSelectedPath;
  if (m_blueprintLibrarySelection >= 0 &&
      m_blueprintLibrarySelection < static_cast<int>(m_blueprintLibrary.size())) {
    prevSelectedPath =
        m_blueprintLibrary[static_cast<std::size_t>(m_blueprintLibrarySelection)].path;
  }

  m_blueprintLibrary.clear();
  m_blueprintLibrarySelection = 0;
  m_blueprintLibraryDeleteArmed = false;
  m_blueprintLibraryDeleteTimer = 0.0f;

  m_blueprintLibraryPreviewIndex = -1;
  m_blueprintLibraryPreviewOk = false;
  m_blueprintLibraryPreviewError.clear();
  m_blueprintLibraryPreview = Blueprint{};

  // Ensure directory exists (ignore errors; we can still run without a folder).
  {
    std::error_code ec;
    fs::create_directories(fs::path(kBlueprintLibraryDir), ec);
  }

  auto ageTextForPath = [&](const fs::path& p) -> std::string {
    std::error_code ec;
    const fs::file_time_type ft = fs::last_write_time(p, ec);
    if (ec) return std::string("(unknown time)");

    const auto now = fs::file_time_type::clock::now();
    auto d = (now >= ft) ? (now - ft) : (ft - now);
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(d).count();
    if (sec < 60) return std::string("just now");
    if (sec < 3600) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%lldm ago", static_cast<long long>(sec / 60));
      return std::string(buf);
    }
    if (sec < 86400) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%lldh ago", static_cast<long long>(sec / 3600));
      return std::string(buf);
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%lldd ago", static_cast<long long>(sec / 86400));
    return std::string(buf);
  };

  auto sortKeyForPath = [&](const fs::path& p) -> std::int64_t {
    std::error_code ec;
    const fs::file_time_type ft = fs::last_write_time(p, ec);
    if (ec) return 0;

    // Convert file clock -> system_clock best-effort.
    const auto nowFile = fs::file_time_type::clock::now();
    const auto nowSys = std::chrono::system_clock::now();
    const auto sysTp = nowSys + (ft - nowFile);
    return static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(sysTp.time_since_epoch()).count());
  };

  std::error_code ec;
  const fs::path dir(kBlueprintLibraryDir);
  if (!fs::exists(dir, ec) || ec) return;

  for (const auto& it : fs::directory_iterator(dir, ec)) {
    if (ec) break;
    if (!it.is_regular_file(ec) || ec) continue;

    const fs::path p = it.path();
    const std::string ext = p.extension().string();
    if (ext != kBlueprintExt) continue;

    BlueprintLibraryEntry e;
    e.path = p.string();
    e.name = p.filename().string();
    e.timeText = ageTextForPath(p);
    e.sortKey = sortKeyForPath(p);

    Blueprint bp;
    std::string err;
    if (LoadBlueprintBinary(bp, e.path, err)) {
      e.ok = true;
      e.width = bp.width;
      e.height = bp.height;
      e.deltas = static_cast<int>(bp.tiles.size());
      e.version = bp.version;
    } else {
      e.ok = false;
      e.err = err;
    }

    m_blueprintLibrary.push_back(std::move(e));
  }

  std::sort(m_blueprintLibrary.begin(), m_blueprintLibrary.end(),
            [&](const BlueprintLibraryEntry& a, const BlueprintLibraryEntry& b) {
              // Newest first; tie-break on name.
              if (a.sortKey != b.sortKey) return a.sortKey > b.sortKey;
              return a.name < b.name;
            });

  // Restore selection if possible.
  if (!prevSelectedPath.empty()) {
    for (std::size_t i = 0; i < m_blueprintLibrary.size(); ++i) {
      if (m_blueprintLibrary[i].path == prevSelectedPath) {
        m_blueprintLibrarySelection = static_cast<int>(i);
        break;
      }
    }
  }

  m_blueprintLibrarySelection =
      std::clamp(m_blueprintLibrarySelection, 0,
                 std::max(0, static_cast<int>(m_blueprintLibrary.size()) - 1));

  m_blueprintLibraryFirst =
      std::clamp(m_blueprintLibraryFirst, 0,
                 std::max(0, static_cast<int>(m_blueprintLibrary.size()) - 1));
}

void Game::ensureBlueprintLibraryPreviewUpToDate()
{
  if (!m_blueprintLibraryOpen) return;
  if (m_blueprintLibrarySelection < 0 || m_blueprintLibrarySelection >= static_cast<int>(m_blueprintLibrary.size())) {
    m_blueprintLibraryPreviewIndex = -1;
    m_blueprintLibraryPreviewOk = false;
    m_blueprintLibraryPreviewError.clear();
    m_blueprintLibraryPreview = Blueprint{};
    return;
  }

  if (m_blueprintLibraryPreviewIndex == m_blueprintLibrarySelection) return;

  m_blueprintLibraryPreviewIndex = m_blueprintLibrarySelection;
  m_blueprintLibraryPreviewOk = false;
  m_blueprintLibraryPreviewError.clear();
  m_blueprintLibraryPreview = Blueprint{};

  const BlueprintLibraryEntry& e = m_blueprintLibrary[static_cast<std::size_t>(m_blueprintLibrarySelection)];
  Blueprint bp;
  std::string err;
  if (LoadBlueprintBinary(bp, e.path, err)) {
    m_blueprintLibraryPreviewOk = true;
    m_blueprintLibraryPreview = std::move(bp);
  } else {
    m_blueprintLibraryPreviewOk = false;
    m_blueprintLibraryPreviewError = err;
  }
}

bool Game::saveBlueprintToLibrary()
{
  namespace fs = std::filesystem;

  if (!m_hasBlueprint) {
    showToast("No blueprint to save", 2.0f);
    return false;
  }

  updateBlueprintTransformed();
  const Blueprint& bp = m_blueprintTransformed;
  if (bp.width <= 0 || bp.height <= 0) {
    showToast("Blueprint is empty", 2.0f);
    return false;
  }

  // Ensure directory exists.
  {
    std::error_code ec;
    fs::create_directories(fs::path(kBlueprintLibraryDir), ec);
  }

  const std::string stamp = FileTimestamp();
  const std::string fname = TextFormat("bp_%s_%dx%d_%d%s", stamp.c_str(), bp.width, bp.height,
                                       static_cast<int>(bp.tiles.size()), kBlueprintExt);
  const fs::path outPath = fs::path(kBlueprintLibraryDir) / fs::path(fname);

  std::string err;
  if (!SaveBlueprintBinary(bp, outPath.string(), err)) {
    showToast(std::string("Blueprint save failed: ") + err, 3.0f);
    return false;
  }

  showToast(std::string("Saved blueprint: ") + outPath.string(), 2.0f);
  refreshBlueprintLibrary();

  // Try to select the freshly written file.
  for (std::size_t i = 0; i < m_blueprintLibrary.size(); ++i) {
    if (m_blueprintLibrary[i].path == outPath.string()) {
      m_blueprintLibrarySelection = static_cast<int>(i);
      m_blueprintLibraryPreviewIndex = -1; // force reload
      break;
    }
  }
  return true;
}

bool Game::loadBlueprintFromLibrarySelection()
{
  if (m_blueprintLibrarySelection < 0 || m_blueprintLibrarySelection >= static_cast<int>(m_blueprintLibrary.size())) {
    showToast("Blueprint library is empty", 2.0f);
    return false;
  }

  const BlueprintLibraryEntry& e = m_blueprintLibrary[static_cast<std::size_t>(m_blueprintLibrarySelection)];

  Blueprint bp;
  std::string err;
  if (!LoadBlueprintBinary(bp, e.path, err)) {
    showToast(std::string("Blueprint load failed: ") + err, 3.0f);
    return false;
  }

  m_hasBlueprint = true;
  m_blueprint = std::move(bp);
  m_blueprintTransform = BlueprintTransform{};
  m_blueprintTransformedDirty = true;
  updateBlueprintTransformed();
  m_blueprintMode = BlueprintMode::Stamp;

  showToast(TextFormat("Loaded blueprint: %s (%dx%d)", e.name.c_str(), m_blueprintTransformed.width,
                       m_blueprintTransformed.height),
            2.0f);
  return true;
}

bool Game::deleteBlueprintFromLibrarySelection()
{
  namespace fs = std::filesystem;

  if (m_blueprintLibrarySelection < 0 || m_blueprintLibrarySelection >= static_cast<int>(m_blueprintLibrary.size())) {
    showToast("No blueprint selected", 2.0f);
    return false;
  }

  const BlueprintLibraryEntry& e = m_blueprintLibrary[static_cast<std::size_t>(m_blueprintLibrarySelection)];
  std::error_code ec;
  const bool ok = fs::remove(fs::path(e.path), ec) && !ec;
  if (!ok) {
    showToast(std::string("Delete failed: ") + e.path, 3.0f);
    return false;
  }

  showToast(std::string("Deleted blueprint: ") + e.name, 2.0f);
  refreshBlueprintLibrary();
  return true;
}

bool Game::tileBlueprintRect(int x0, int y0, int x1, int y1, bool useLibraryTileset)
{
  if (!m_hasBlueprint) {
    showToast("Blueprint: nothing to tile (capture a stamp first)");
    return false;
  }

  updateBlueprintTransformed();
  const int cellW = m_blueprintTransformed.width;
  const int cellH = m_blueprintTransformed.height;
  if (cellW <= 0 || cellH <= 0) {
    showToast("Blueprint: invalid stamp size");
    return false;
  }

  const int wx = m_world.width();
  const int wy = m_world.height();
  if (wx <= 0 || wy <= 0) return false;

  const int rx0 = std::clamp(std::min(x0, x1), 0, wx - 1);
  const int ry0 = std::clamp(std::min(y0, y1), 0, wy - 1);
  const int rx1 = std::clamp(std::max(x0, x1), 0, wx - 1);
  const int ry1 = std::clamp(std::max(y0, y1), 0, wy - 1);

  const int regionW = rx1 - rx0 + 1;
  const int regionH = ry1 - ry0 + 1;
  const int cellsX = regionW / cellW;
  const int cellsY = regionH / cellH;

  if (cellsX <= 0 || cellsY <= 0) {
    showToast(TextFormat("Tile fill: selection too small for %dx%d stamp", cellW, cellH));
    return false;
  }

  // Load blueprint sources (current stamp + optional library). We keep this local so
  // the solver can generate transformed variants as needed.
  std::vector<std::pair<std::string, Blueprint>> sources;
  sources.reserve(1u + m_blueprintLibrary.size());
  sources.emplace_back("current", m_blueprintTransformed);

  int libLoaded = 0;
  int libSkipped = 0;
  if (useLibraryTileset) {
    refreshBlueprintLibrary();
    for (const auto& e : m_blueprintLibrary) {
      if (!e.ok) {
        libSkipped++;
        continue;
      }
      Blueprint bp;
      std::string err;
      if (!LoadBlueprintBinary(bp, e.path, err)) {
        libSkipped++;
        continue;
      }
      sources.emplace_back(e.name, std::move(bp));
      libLoaded++;
    }
  }

  BlueprintTileset tileset;
  std::string err;
  if (!BuildBlueprintTileset(sources, cellW, cellH, /*generateTransforms=*/useLibraryTileset, tileset, err)) {
    showToast(TextFormat("Tile fill failed: %s", err.c_str()));
    return false;
  }

  BlueprintTilingConfig cfg;
  cfg.matchRoadEdges = useLibraryTileset;
  cfg.allowFallback = true;
  cfg.seed = static_cast<std::uint32_t>(m_cfg.seed ^ (m_cfg.seed >> 32));

  BlueprintTilingSolution sol;
  if (!SolveBlueprintTiling(tileset, cellsX, cellsY, cfg, sol, err)) {
    showToast(TextFormat("Tiling solve failed: %s", err.c_str()));
    return false;
  }

  // Apply in a single undoable stroke.
  endPaintStroke();
  beginPaintStroke();

  BlueprintApplyOptions opt = m_blueprintApplyOpt;
  opt.transform = BlueprintTransform{}; // variants are pre-transformed
  const bool wantRecomputeRoadMasks = opt.recomputeRoadMasks;
  opt.recomputeRoadMasks = false;

  int placed = 0;
  int applyFailed = 0;
  std::vector<std::uint8_t> used(tileset.variants.size(), std::uint8_t{0});

  for (int cy = 0; cy < cellsY; ++cy) {
    for (int cx = 0; cx < cellsX; ++cx) {
      const int vidx = sol.chosen[static_cast<std::size_t>(cy * cellsX + cx)];
      if (vidx < 0 || vidx >= static_cast<int>(tileset.variants.size())) continue;
      used[static_cast<std::size_t>(vidx)] = 1;

      const Blueprint& bp = tileset.variants[static_cast<std::size_t>(vidx)].bp;
      const int dstX = rx0 + cx * cellW;
      const int dstY = ry0 + cy * cellH;

      // Mark history for tiles that might be changed.
      for (const auto& d : bp.tiles) {
        const int lx = static_cast<int>(d.index % static_cast<std::uint32_t>(bp.width));
        const int ly = static_cast<int>(d.index / static_cast<std::uint32_t>(bp.width));
        const int tx = dstX + lx;
        const int ty = dstY + ly;
        if (!m_world.inBounds(tx, ty)) continue;
        m_history.noteTilePreEdit(m_world, tx, ty);
        m_tilesEditedThisStroke.push_back({tx, ty});
      }

      std::string aerr;
      if (!ApplyBlueprint(m_world, bp, dstX, dstY, opt, aerr)) {
        applyFailed++;
        continue;
      }

      placed++;
    }
  }

  if (wantRecomputeRoadMasks) {
    m_world.recomputeRoadMasks();
  }

  // Invalidate systems that depend on tile fields.
  m_roadGraphDirty = true;
  m_resilienceDirty = true;
  m_resilienceBypassesDirty = true;
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_roadUpgradePlanDirty = true;
  m_evacDirty = true;
  m_roadUpgradeSelectedMaskDirty = true;
  invalidateHydrology();

  // Rendering base cache invalidation is handled by m_tilesEditedThisStroke.
  endPaintStroke();

  int usedCount = 0;
  for (std::uint8_t b : used) usedCount += (b ? 1 : 0);

  const int coveredW = cellsX * cellW;
  const int coveredH = cellsY * cellH;
  const int leftoverW = regionW - coveredW;
  const int leftoverH = regionH - coveredH;

  if (useLibraryTileset) {
    showToast(TextFormat("Tiled %dx%d cells (%d stamps)  variants=%d  fallbacks=%d  apply_fail=%d  leftover=%dx%d  lib=%d loaded/%d skipped",
                         cellsX, cellsY, placed, usedCount, sol.fallbacks, applyFailed, leftoverW, leftoverH,
                         libLoaded, libSkipped));
  } else {
    showToast(TextFormat("Tiled %dx%d (%d stamps)  leftover=%dx%d", cellsX, cellsY, placed, leftoverW, leftoverH));
  }

  return placed > 0;
}

void Game::drawBlueprintOverlay()
{
  if (m_blueprintMode == BlueprintMode::Off) return;

  const float tileW = static_cast<float>(m_cfg.tileWidth);
  const float tileH = static_cast<float>(m_cfg.tileHeight);
  const float thickness = 2.0f / std::max(0.35f, m_camera.zoom);

  auto drawOutline = [&](int tx, int ty, Color c) {
    if (!m_world.inBounds(tx, ty)) return;
    const Vector2 center =
        TileToWorldCenterElevated(m_world, tx, ty, tileW, tileH, m_elev);
    Vector2 corners[4];
    TileDiamondCorners(center, tileW, tileH, corners);
    for (int i = 0; i < 4; ++i) {
      const int j = (i + 1) % 4;
      DrawLineEx(corners[i], corners[j], thickness, c);
    }
  };

  BeginMode2D(m_camera);

  if (m_blueprintMode == BlueprintMode::Capture && m_blueprintSelecting && m_blueprintSelStart) {
    const Point a = *m_blueprintSelStart;
    const Point b = m_blueprintSelEnd;
    const int x0 = std::min(a.x, b.x);
    const int y0 = std::min(a.y, b.y);
    const int x1 = std::max(a.x, b.x);
    const int y1 = std::max(a.y, b.y);
    const Color col = Color{60, 255, 120, 200};
    for (int x = x0; x <= x1; ++x) {
      drawOutline(x, y0, col);
      drawOutline(x, y1, col);
    }
    for (int y = y0; y <= y1; ++y) {
      drawOutline(x0, y, col);
      drawOutline(x1, y, col);
    }
  }

  if (m_blueprintMode == BlueprintMode::Stamp && m_blueprintTilingSelecting && m_blueprintTileSelStart) {
    const Point a = *m_blueprintTileSelStart;
    const Point b = m_blueprintTileSelEnd;
    const int x0 = std::min(a.x, b.x);
    const int y0 = std::min(a.y, b.y);
    const int x1 = std::max(a.x, b.x);
    const int y1 = std::max(a.y, b.y);
    const Color col = m_blueprintTileUseLibrary ? Color{200, 120, 255, 210} : Color{80, 170, 255, 200};
    for (int x = x0; x <= x1; ++x) {
      drawOutline(x, y0, col);
      drawOutline(x, y1, col);
    }
    for (int y = y0; y <= y1; ++y) {
      drawOutline(x0, y, col);
      drawOutline(x1, y, col);
    }
  }

  if (m_blueprintMode == BlueprintMode::Stamp && m_hasBlueprint && m_hovered) {
    updateBlueprintTransformed();
    const Blueprint& bp = m_blueprintTransformed;
    if (bp.width > 0 && bp.height > 0) {
      const int dstX = m_hovered->x - (bp.width / 2);
      const int dstY = m_hovered->y - (bp.height / 2);

      bool oob = false;
      for (const auto& d : bp.tiles) {
        const int lx = d.index % bp.width;
        const int ly = d.index / bp.width;
        const int tx = dstX + lx;
        const int ty = dstY + ly;
        if (!m_world.inBounds(tx, ty)) {
          oob = true;
          break;
        }
      }

      const Color border = oob ? Color{255, 80, 80, 220} : Color{80, 170, 255, 220};
      const Color tileCol = oob ? Color{255, 150, 80, 200} : Color{255, 240, 120, 220};

      // Draw transformed blueprint bounds.
      const int x0 = dstX;
      const int y0 = dstY;
      const int x1 = dstX + bp.width - 1;
      const int y1 = dstY + bp.height - 1;
      for (int x = x0; x <= x1; ++x) {
        drawOutline(x, y0, border);
        drawOutline(x, y1, border);
      }
      for (int y = y0; y <= y1; ++y) {
        drawOutline(x0, y, border);
        drawOutline(x1, y, border);
      }

      // Draw actual affected tiles.
      for (const auto& d : bp.tiles) {
        const int lx = d.index % bp.width;
        const int ly = d.index / bp.width;
        const int tx = dstX + lx;
        const int ty = dstY + ly;
        drawOutline(tx, ty, tileCol);
      }
    }
  }

  EndMode2D();
}

void Game::drawBlueprintPanel(int uiW, int uiH)
{
  if (m_blueprintMode == BlueprintMode::Off) return;

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  const bool lib = m_blueprintLibraryOpen;

  // Responsive sizing (keeps the panel on-screen on small windows).
  const int x = kUiPanelMargin;
  const int y = kUiPanelTopY;
  int w = lib ? 560 : 420;
  int h = lib ? 440 : 170;
  if (uiW > 0) w = std::min(w, uiW - x - kUiPanelMargin);
  if (uiH > 0) h = std::min(h, uiH - y - kUiPanelMargin);
  w = std::max(320, w);
  h = std::max(150, h);

  const Rectangle panelR{static_cast<float>(x), static_cast<float>(y), static_cast<float>(w), static_cast<float>(h)};
  ui::DrawPanel(panelR, uiTime, /*active=*/true);

  const char* mode = (m_blueprintMode == BlueprintMode::Capture) ? "CAPTURE" : "STAMP";
  ui::Text(x + 10, y + 8, 20, TextFormat("Blueprint Tool [%s]%s", mode, lib ? "  [LIB]" : ""), uiTh.text,
           /*bold=*/true, /*shadow=*/true, 1);

  int ty = y + 34;
  if (m_blueprintMode == BlueprintMode::Capture) {
    ui::Text(x + 10, ty, 18, "LMB drag: select region to capture", uiTh.text, /*bold=*/false, /*shadow=*/true, 1);
    ty += 22;
    ui::Text(x + 10, ty, 14, "Enter: STAMP | Backspace: clear | Ctrl+S: save | Ctrl+L: library | J/Esc: exit",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    ty += 20;

    if (m_blueprintSelecting && m_blueprintSelStart) {
      const Point a = *m_blueprintSelStart;
      const Point b = m_blueprintSelEnd;
      const int sx = std::min(a.x, b.x);
      const int sy = std::min(a.y, b.y);
      const int ex = std::max(a.x, b.x);
      const int ey = std::max(a.y, b.y);
      ui::Text(x + 10, ty, 16,
               TextFormat("Selecting: (%d,%d) -> (%d,%d)  size=%dx%d", sx, sy, ex, ey, (ex - sx + 1),
                          (ey - sy + 1)),
               Color{150, 255, 170, 255}, /*bold=*/false, /*shadow=*/true, 1);
    } else if (m_hasBlueprint) {
      updateBlueprintTransformed();
      ui::Text(x + 10, ty, 16,
               TextFormat("Current stamp: %dx%d (%d tiles)", m_blueprintTransformed.width,
                          m_blueprintTransformed.height, static_cast<int>(m_blueprintTransformed.tiles.size())),
               Color{200, 220, 255, 255}, /*bold=*/false, /*shadow=*/true, 1);
    }
  } else {
    ui::Text(x + 10, ty, 18, "LMB: stamp at hovered tile (center anchored)", uiTh.text, /*bold=*/false,
             /*shadow=*/true, 1);
    ty += 22;
    ui::Text(x + 10, ty, 14,
             "Shift+drag: tile fill (repeat) | Ctrl+Shift+drag: proc tile (library+road seams)",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    ty += 18;
    ui::Text(x + 10, ty, 14,
             "Z/X/Y: rotate/mirror | Enter: CAPTURE | Backspace: clear | Ctrl+S: save | Ctrl+L: library | J/Esc: exit",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    ty += 20;

    if (m_blueprintTilingSelecting && m_blueprintTileSelStart) {
      const Point a = *m_blueprintTileSelStart;
      const Point b = m_blueprintTileSelEnd;
      const int sx = std::min(a.x, b.x);
      const int sy = std::min(a.y, b.y);
      const int ex = std::max(a.x, b.x);
      const int ey = std::max(a.y, b.y);
      const int selW = (ex - sx + 1);
      const int selH = (ey - sy + 1);
      updateBlueprintTransformed();
      const int cellW = std::max(1, m_blueprintTransformed.width);
      const int cellH = std::max(1, m_blueprintTransformed.height);
      const int cellsX = selW / cellW;
      const int cellsY = selH / cellH;
      const Color col = m_blueprintTileUseLibrary ? Color{220, 180, 255, 255} : Color{180, 220, 255, 255};
      ui::Text(x + 10, ty, 16,
               TextFormat("Tiling: (%d,%d)->(%d,%d)  size=%dx%d  cells=%dx%d  mode=%s", sx, sy, ex, ey, selW,
                          selH, cellsX, cellsY, m_blueprintTileUseLibrary ? "LIB" : "REPEAT"),
               col, /*bold=*/false, /*shadow=*/true, 1);
    } else if (m_hasBlueprint) {
      updateBlueprintTransformed();
      ui::Text(x + 10, ty, 16,
               TextFormat("Stamp: %dx%d (%d tiles) rot=%d mx=%d my=%d", m_blueprintTransformed.width,
                          m_blueprintTransformed.height,
                          static_cast<int>(m_blueprintTransformed.tiles.size()), m_blueprintTransform.rotateDeg,
                          m_blueprintTransform.mirrorX ? 1 : 0, m_blueprintTransform.mirrorY ? 1 : 0),
               Color{255, 240, 190, 255}, /*bold=*/false, /*shadow=*/true, 1);
    } else {
      ui::Text(x + 10, ty, 16, "No stamp captured yet - press Enter or switch to CAPTURE", Color{255, 200, 200, 255},
               /*bold=*/false, /*shadow=*/true, 1);
    }
  }

  // Library UI
  if (lib) {
    // Layout: left list, right preview.
    const int pad = 10;
    const int headerH = 22;
    const int rowH = 34;

    const int areaY = ty + 26;
    const int areaX = x + pad;
    const int areaW = w - pad * 2;
    const int areaH = h - (areaY - y) - pad;

    ui::Text(areaX, areaY - 18, 14, TextFormat("Library (%s/*%s)", kBlueprintLibraryDir, kBlueprintExt),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

    const int previewW = std::min(200, std::max(140, areaW / 3));
    const int listW = areaW - previewW - pad;
    const int listX = areaX;
    const int listY = areaY;
    const int prevX = areaX + listW + pad;
    const int prevY = areaY;

    // List box
    const Rectangle listR{static_cast<float>(listX), static_cast<float>(listY), static_cast<float>(listW),
                          static_cast<float>(areaH)};
    ui::DrawPanelInset(listR, uiTime, /*active=*/true);

    ui::Text(listX + 8, listY + 6, 14, "Up/Down: select | Enter: load | Del: delete | Ctrl+L/Esc: close",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

    const int listTop = listY + headerH;

    const int count = static_cast<int>(m_blueprintLibrary.size());

    // Scrollable view rect (rows only).
    const Rectangle viewR{static_cast<float>(listX + 6), static_cast<float>(listTop + 2),
                          static_cast<float>(listW - 12), static_cast<float>(areaH - headerH - 6)};

    constexpr float kSbW = 12.0f;
    const Rectangle barR{viewR.x + viewR.width - kSbW, viewR.y, kSbW, viewR.height};
    const Rectangle contentR = ui::ContentRectWithScrollbar(viewR, kSbW, 2.0f);

    const int visibleRows = std::max(1, static_cast<int>(std::floor(contentR.height / static_cast<float>(rowH))));

    // Maintain a persistent scroll position (first visible row) and keep selection visible.
    int first = std::clamp(m_blueprintLibraryFirst, 0, std::max(0, count - visibleRows));

    // Mouse wheel scroll (moves selection; view follows selection).
    const bool hoverList = CheckCollisionPointRec(mouseUi, viewR);
    const float wheel = GetMouseWheelMove();
    if (hoverList && wheel != 0.0f && count > 0) {
      const int step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 3 : 1;
      const int delta = (wheel > 0.0f) ? -step : step;
      const int newSel = std::clamp(m_blueprintLibrarySelection + delta, 0, std::max(0, count - 1));
      if (newSel != m_blueprintLibrarySelection) {
        m_blueprintLibrarySelection = newSel;
        m_blueprintLibraryDeleteArmed = false;
        m_blueprintLibraryDeleteTimer = 0.0f;
      }
    }

    if (m_blueprintLibrarySelection < first) first = m_blueprintLibrarySelection;
    if (m_blueprintLibrarySelection >= first + visibleRows) first = m_blueprintLibrarySelection - visibleRows + 1;
    first = std::clamp(first, 0, std::max(0, count - visibleRows));
    m_blueprintLibraryFirst = first;

    const bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    for (int i = 0; i < visibleRows; ++i) {
      const int idx = first + i;
      if (idx >= count) break;

      const BlueprintLibraryEntry& e = m_blueprintLibrary[static_cast<std::size_t>(idx)];
      const int ry = static_cast<int>(contentR.y) + i * rowH;
      const bool sel = (idx == m_blueprintLibrarySelection);

      const Rectangle rowR{contentR.x, static_cast<float>(ry), contentR.width, static_cast<float>(rowH)};
      if (leftPressed && CheckCollisionPointRec(mouseUi, rowR)) {
        m_blueprintLibrarySelection = idx;
        m_blueprintLibraryDeleteArmed = false;
        m_blueprintLibraryDeleteTimer = 0.0f;
      }

      if (sel) {
        ui::DrawSelectionHighlight(Rectangle{rowR.x - 2.0f, rowR.y + 2.0f, rowR.width + 4.0f, rowR.height - 4.0f},
                                   uiTime, /*strong=*/true);
      }

      const Color nameCol = sel ? uiTh.text : uiTh.textDim;
      const Color metaCol = sel ? Color{210, 230, 255, 255} : Color{170, 170, 170, 255};

      const char* okTag = e.ok ? "" : "[!] ";
      ui::Text(static_cast<int>(contentR.x) + 6, ry + 4, 14, TextFormat("%s%s", okTag, e.name.c_str()), nameCol,
               /*bold=*/sel, /*shadow=*/true, 1);
      if (e.ok) {
        ui::Text(static_cast<int>(contentR.x) + 6, ry + 20, 12,
                 TextFormat("%dx%d  %d tiles  %s", e.width, e.height, e.deltas, e.timeText.c_str()),
                 metaCol, /*bold=*/false, /*shadow=*/true, 1);
      } else {
        ui::Text(static_cast<int>(contentR.x) + 6, ry + 20, 12, TextFormat("load error: %s", e.err.c_str()),
                 Color{255, 160, 160, 255}, /*bold=*/false, /*shadow=*/true, 1);
      }
    }

    // Scrollbar (drives `first`).
    int newFirst = first;
    if (ui::ScrollbarV(8601, barR, count, visibleRows, newFirst, mouseUi, uiTime, /*enabled=*/true)) {
      newFirst = std::clamp(newFirst, 0, std::max(0, count - visibleRows));
      m_blueprintLibraryFirst = newFirst;

      // Keep selection inside the new view window.
      if (count > 0) {
        if (m_blueprintLibrarySelection < newFirst) m_blueprintLibrarySelection = newFirst;
        if (m_blueprintLibrarySelection >= newFirst + visibleRows)
          m_blueprintLibrarySelection = std::clamp(newFirst + visibleRows - 1, 0, count - 1);
      } else {
        m_blueprintLibrarySelection = 0;
      }

      m_blueprintLibraryDeleteArmed = false;
      m_blueprintLibraryDeleteTimer = 0.0f;
    }

    ensureBlueprintLibraryPreviewUpToDate();

    // Preview box
    const Rectangle prevR{static_cast<float>(prevX), static_cast<float>(prevY), static_cast<float>(previewW),
                          static_cast<float>(areaH)};
    ui::DrawPanelInset(prevR, uiTime, /*active=*/true);

    const int prevPad = 8;
    const int prevTitleY = prevY + 6;
    ui::Text(prevX + prevPad, prevTitleY, 14, "Preview", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);

    const int square = std::min(previewW - prevPad * 2, areaH - 46);
    const int sqX = prevX + (previewW - square) / 2;
    const int sqY = prevY + 26;

    const Rectangle sqR{static_cast<float>(sqX), static_cast<float>(sqY), static_cast<float>(square),
                        static_cast<float>(square)};
    ui::DrawPanelInset(sqR, uiTime * 0.7f, /*active=*/false);

    auto colorForOverlay = [](Overlay o) -> Color {
      switch (o) {
      case Overlay::Road: return Color{130, 130, 130, 255};
      case Overlay::Residential: return Color{80, 200, 120, 255};
      case Overlay::Commercial: return Color{90, 160, 255, 255};
      case Overlay::Industrial: return Color{240, 200, 60, 255};
      case Overlay::Park: return Color{60, 220, 70, 255};
      default: return Color{35, 35, 35, 255};
      }
    };

    // Draw a very small "stamp thumbnail" using just the overlay field.
    if (m_blueprintLibraryPreviewOk && m_blueprintLibraryPreview.width > 0 &&
        m_blueprintLibraryPreview.height > 0) {
      const Blueprint& bp = m_blueprintLibraryPreview;
      const int maxDim = std::max(1, std::max(bp.width, bp.height));
      const int cell = std::max(1, square / maxDim);
      const int drawW = cell * bp.width;
      const int drawH = cell * bp.height;
      const int ox = sqX + (square - drawW) / 2;
      const int oy = sqY + (square - drawH) / 2;

      std::vector<Overlay> grid(static_cast<std::size_t>(bp.width * bp.height), Overlay::None);
      for (const auto& d : bp.tiles) {
        if (d.index >= grid.size()) continue;
        if (d.mask & static_cast<std::uint8_t>(TileFieldMask::Overlay)) {
          grid[d.index] = d.value.overlay;
        }
      }

      for (int by = 0; by < bp.height; ++by) {
        for (int bx = 0; bx < bp.width; ++bx) {
          const Overlay o = grid[static_cast<std::size_t>(by * bp.width + bx)];
          DrawRectangle(ox + bx * cell, oy + by * cell, cell, cell, colorForOverlay(o));
        }
      }

      ui::Text(prevX + prevPad, prevY + areaH - 18, 14, TextFormat("%dx%d", bp.width, bp.height), uiTh.textDim,
               /*bold=*/false, /*shadow=*/true, 1);
    } else {
      const char* msg = m_blueprintLibraryPreviewError.empty() ? "(no preview)" : "(preview load failed)";
      ui::Text(sqX + 8, sqY + 8, 14, msg, Color{220, 180, 180, 255}, /*bold=*/false, /*shadow=*/true, 1);
    }
  }
}


// -----------------------------------------------------------------------------
// Road resilience overlay + bypass planner
// -----------------------------------------------------------------------------

void Game::ensureRoadGraphUpToDate()
{
  if (!m_roadGraphDirty) return;

  m_roadGraph = BuildRoadGraph(m_world);
  m_roadGraphDirty = false;

  // Build fast tile->(node/edge) lookups + precompute edge traversal costs for A* routing.
  m_roadGraphIndex = BuildRoadGraphIndex(m_world, m_roadGraph);
  m_roadGraphWeights = BuildRoadGraphWeights(m_world, m_roadGraph);

  // Reuse the same lookup tables for road-graph debug overlays.
  m_roadGraphTileToNode = m_roadGraphIndex.tileToNode;
  m_roadGraphTileToEdge = m_roadGraphIndex.tileToEdge;

  // Any road-graph change invalidates downstream road-resilience caches.
  m_resilienceDirty = true;
  m_resilienceBypassesDirty = true;
}

void Game::ensureRoadResilienceUpToDate()
{
  ensureRoadGraphUpToDate();
  if (!m_resilienceDirty) return;
  m_roadResilience = ComputeRoadGraphResilience(m_roadGraph);
  m_resilienceDirty = false;
}

void Game::rebuildRoadResilienceBypasses()
{
  ensureRoadResilienceUpToDate();

  m_resilienceBypasses.clear();
  m_resilienceBypassesDirty = false;

  if (m_resilienceBypassTop <= 0) return;
  if (m_roadResilience.bridgeEdges.empty()) return;

  const int mapW = m_world.width();
  const int mapH = m_world.height();
  if (mapW <= 0 || mapH <= 0) return;

  const bool haveTraffic = (!m_trafficDirty && !m_traffic.roadTraffic.empty() &&
                            static_cast<int>(m_traffic.roadTraffic.size()) == mapW * mapH);

  RoadResilienceBypassConfig cfg;
  cfg.top = m_resilienceBypassTop;
  cfg.moneyObjective = m_resilienceBypassMoney;
  cfg.targetLevel = ClampRoadLevel(m_resilienceBypassTargetLevel);
  cfg.allowBridges = m_resilienceBypassAllowBridges;
  cfg.maxPrimaryCost = m_resilienceBypassMaxCost;
  cfg.maxNodesPerSide = m_resilienceBypassMaxNodesPerSide;
  cfg.rankByTraffic = true;

  m_resilienceBypasses = SuggestRoadResilienceBypasses(m_world, m_roadGraph, m_roadResilience, cfg,
                                                       haveTraffic ? &m_traffic : nullptr);
}

bool Game::applyRoadResilienceBypass(std::size_t idx)
{
  if (idx >= m_resilienceBypasses.size()) {
    showToast("No such bypass suggestion");
    return false;
  }

  const RoadResilienceBypassSuggestion& s = m_resilienceBypasses[idx];
  if (s.path.size() < 2) {
    showToast("Bypass path is empty");
    return false;
  }

  // Validate buildability and compute the current money cost (world may have changed since planning).
  int moneyCost = 0;
  bool anyChange = false;
  for (const Point& p : s.path) {
    if (!m_world.inBounds(p.x, p.y)) {
      showToast("Bypass path is out of bounds (re-suggest)");
      return false;
    }
    const Tile& t = m_world.at(p.x, p.y);
    if (t.overlay != Overlay::None && t.overlay != Overlay::Road) {
      showToast("Bypass path is blocked (re-suggest)");
      return false;
    }
    if (t.terrain == Terrain::Water && !s.allowBridges) {
      showToast("Bypass would require bridges (enable allowBridges)");
      return false;
    }

    const bool isBridge = (t.terrain == Terrain::Water);
    if (t.overlay == Overlay::Road) {
      const int cur = ClampRoadLevel(static_cast<int>(t.level));
      const int c = RoadPlacementCost(cur, s.targetLevel, /*alreadyRoad=*/true, isBridge);
      moneyCost += c;
      if (c > 0) anyChange = true;
    } else {
      const int c = RoadPlacementCost(1, s.targetLevel, /*alreadyRoad=*/false, isBridge);
      moneyCost += c;
      if (c > 0) anyChange = true;
    }
  }

  if (!anyChange) {
    showToast("Bypass already built (no changes)");
    return false;
  }

  if (moneyCost > m_world.stats().money) {
    showToast(TextFormat("Insufficient funds (%d needed)", moneyCost));
    return false;
  }

  endPaintStroke();

  const int moneyBefore = m_world.stats().money;
  m_history.beginStroke(m_world);

  std::vector<Point> changed;
  changed.reserve(s.path.size());

  for (const Point& p : s.path) {
    // Always note the pre-edit state so undo/redo can restore tiles.
    m_history.noteTilePreEdit(m_world, p.x, p.y);
    const ToolApplyResult r = m_world.applyRoad(p.x, p.y, s.targetLevel);
    if (r == ToolApplyResult::Applied) {
      changed.push_back(p);
    }
  }

  m_history.endStroke(m_world);

  // Invalidate render caches and derived stats.
  m_sim.refreshDerivedStats(m_world);
  m_renderer.markMinimapDirty();
  m_renderer.markBaseCacheDirtyForTiles(changed, m_world.width(), m_world.height());

  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_landValueDirty = true;
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;
  m_vehiclesDirty = true;

  // Suggestions are now stale.
  m_resilienceDirty = true;
  m_resilienceBypassesDirty = true;
  m_resilienceBypasses.clear();

  const int spent = moneyBefore - m_world.stats().money;
  showToast(TextFormat("Bypass applied: %d tiles, spent %d", static_cast<int>(changed.size()), spent));
  return true;
}

void Game::drawRoadResilienceOverlay()
{
  if (!m_showResilienceOverlay) return;

  ensureRoadResilienceUpToDate();
  if (m_resilienceBypassesDirty) {
    rebuildRoadResilienceBypasses();
  }

  if (m_roadGraph.nodes.empty()) return;

  BeginMode2D(m_camera);

  const float zoom = std::max(0.25f, m_camera.zoom);
  const float thicknessBridge = 3.5f / zoom;
  const float thicknessBypass = 2.5f / zoom;
  const float radius = 4.0f / zoom;

  // Hover highlight (optional).
  int hoveredEdge = -1;
  int hoveredNode = -1;
  if (m_hovered) {
    const int w = m_world.width();
    const int h = m_world.height();
    if (w > 0 && h > 0) {
      const int idx = m_hovered->y * w + m_hovered->x;
      if (idx >= 0 && idx < static_cast<int>(m_roadGraphTileToNode.size())) {
        hoveredNode = m_roadGraphTileToNode[static_cast<std::size_t>(idx)];
      }
      if (idx >= 0 && idx < static_cast<int>(m_roadGraphTileToEdge.size())) {
        hoveredEdge = m_roadGraphTileToEdge[static_cast<std::size_t>(idx)];
      }
    }
  }

  // Draw bridge edges.
  for (int ei : m_roadResilience.bridgeEdges) {
    if (ei < 0 || ei >= static_cast<int>(m_roadGraph.edges.size())) continue;
    const RoadGraphEdge& e = m_roadGraph.edges[static_cast<std::size_t>(ei)];
    const bool hi = (ei == hoveredEdge);
    const Color c = hi ? Color{255, 80, 80, 230} : Color{255, 80, 80, 170};

    for (std::size_t i = 1; i < e.tiles.size(); ++i) {
      const Point& a = e.tiles[i - 1];
      const Point& b = e.tiles[i];
      const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
      const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
      DrawLineEx(wa, wb, thicknessBridge, c);
    }
  }

  // Draw articulation nodes.
  for (int ni : m_roadResilience.articulationNodes) {
    if (ni < 0 || ni >= static_cast<int>(m_roadGraph.nodes.size())) continue;
    const RoadGraphNode& n = m_roadGraph.nodes[static_cast<std::size_t>(ni)];
    const bool hi = (ni == hoveredNode);
    const Color c = hi ? Color{255, 235, 60, 255} : Color{255, 235, 60, 200};
    const Vector2 wpos = TileToWorldCenterElevated(m_world, n.pos.x, n.pos.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
    DrawCircleV(wpos, radius, c);
  }

  // Draw bypass suggestions as translucent polylines.
  for (const RoadResilienceBypassSuggestion& s : m_resilienceBypasses) {
    if (s.path.size() < 2) continue;
    const Color c = Color{80, 255, 140, 140};
    for (std::size_t i = 1; i < s.path.size(); ++i) {
      const Point& a = s.path[i - 1];
      const Point& b = s.path[i];
      const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
      const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
      DrawLineEx(wa, wb, thicknessBypass, c);
    }
  }

  EndMode2D();
}

// -----------------------------------------------------------------------------
// Transit planner (auto "bus line" suggestions)
// -----------------------------------------------------------------------------

namespace {

Color TransitLineColor(int idx, unsigned char alpha)
{
  // A small cycle of high-contrast colors for map overlays.
  constexpr Color kColors[] = {
      {230,  70,  70, 255},
      { 70, 210, 120, 255},
      { 70, 140, 230, 255},
      {240, 190,  60, 255},
      {190,  90, 220, 255},
      { 60, 210, 220, 255},
      {240, 120,  40, 255},
      {140, 230,  70, 255},
  };
  constexpr int kN = static_cast<int>(sizeof(kColors) / sizeof(kColors[0]));
  const int i = (kN > 0) ? (idx % kN) : 0;
  Color c = kColors[i];
  c.a = alpha;
  return c;
}

} // namespace

void Game::ensureTransitPlanUpToDate()
{
  if (!m_transitPlanDirty) return;

  ensureRoadGraphUpToDate();
  m_transitEdgeDemand.clear();

  if (m_roadGraph.edges.empty()) {
    m_transitPlan = TransitPlan{};
    m_transitViz.clear();
    m_transitVizDirty = false;
    m_transitPlanDirty = false;
    m_transitSelectedLine = -1;
    return;
  }

  const int w = m_world.width();
  const int h = m_world.height();
  const int n = std::max(0, w) * std::max(0, h);
  if (n <= 0) {
    m_transitPlan = TransitPlan{};
    m_transitViz.clear();
    m_transitVizDirty = false;
    m_transitPlanDirty = false;
    m_transitSelectedLine = -1;
    return;
  }

  // Outside-connection mask (optional, reused by traffic/goods demand computation).
  const bool requireOutside = m_sim.config().requireOutsideConnection;
  const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
  if (requireOutside) {
    // Reuse cached outside-connection mask when possible (also used by other overlays).
    if (static_cast<int>(m_outsideOverlayRoadToEdge.size()) != n) {
      ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
    }
    roadToEdgeMask = &m_outsideOverlayRoadToEdge;
  }

  const TransitModelSettings& tm = m_sim.transitModel();
  const TransitDemandMode dm = tm.demandMode;

  const bool needTraffic = (dm == TransitDemandMode::Commute || dm == TransitDemandMode::Combined);
  const bool needGoods = (dm == TransitDemandMode::Goods || dm == TransitDemandMode::Combined);

  // Traffic (commute)
  if (needTraffic) {
    if (m_trafficDirty || static_cast<int>(m_traffic.roadTraffic.size()) != n) {
      const float share = (m_world.stats().population > 0)
                              ? (static_cast<float>(m_world.stats().employed) /
                                 static_cast<float>(m_world.stats().population))
                              : 0.0f;

      TrafficConfig tc;
      tc.requireOutsideConnection = requireOutside;
      {
        const TrafficModelSettings& trafficModel = m_sim.trafficModel();
        tc.congestionAwareRouting = trafficModel.congestionAwareRouting;
        tc.congestionIterations = trafficModel.congestionIterations;
        tc.congestionAlpha = trafficModel.congestionAlpha;
        tc.congestionBeta = trafficModel.congestionBeta;
        tc.congestionCapacityScale = trafficModel.congestionCapacityScale;
        tc.congestionRatioClamp = trafficModel.congestionRatioClamp;
        tc.capacityAwareJobs = trafficModel.capacityAwareJobs;
        tc.jobAssignmentIterations = trafficModel.jobAssignmentIterations;
        tc.jobPenaltyBaseMilli = trafficModel.jobPenaltyBaseMilli;
      }

      const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
      m_trafficDirty = false;
      m_carbonDirty = true;
    }
  }

  // Goods
  if (needGoods) {
    if (m_goodsDirty || static_cast<int>(m_goods.roadGoodsTraffic.size()) != n) {
      GoodsConfig gc;
      gc.requireOutsideConnection = requireOutside;

      const Stats& st = m_world.stats();
      const TradeModelSettings& ts = m_sim.tradeModel();
      if (ts.enabled) {
        gc.allowImports = ts.allowImports;
        gc.allowExports = ts.allowExports;
        gc.importCapacityPct = std::clamp(st.tradeImportCapacityPct, 0, 100);
        gc.exportCapacityPct = std::clamp(st.tradeExportCapacityPct, 0, 100);
      }
      const std::vector<std::uint8_t>* pre = (gc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;
      m_carbonDirty = true;
      m_carbonDirty = true;
      m_carbonDirty = true;
    }
  }

  // Build a per-road-tile demand signal.
  std::vector<std::uint32_t> roadFlow(static_cast<std::size_t>(n), 0);
  if (needTraffic && m_traffic.roadTraffic.size() == static_cast<std::size_t>(n)) {
    for (int i = 0; i < n; ++i) {
      roadFlow[static_cast<std::size_t>(i)] += static_cast<std::uint32_t>(m_traffic.roadTraffic[static_cast<std::size_t>(i)]);
    }
  }
  if (needGoods && m_goods.roadGoodsTraffic.size() == static_cast<std::size_t>(n)) {
    for (int i = 0; i < n; ++i) {
      roadFlow[static_cast<std::size_t>(i)] += static_cast<std::uint32_t>(m_goods.roadGoodsTraffic[static_cast<std::size_t>(i)]);
    }
  }

  // Aggregate road-tile demand onto the compressed RoadGraph edges.
  const RoadGraphTrafficResult agg = AggregateFlowOnRoadGraph(m_world, m_roadGraph, roadFlow);
  m_transitEdgeDemand.resize(m_roadGraph.edges.size(), 0);

  // Prefer interior demand so endpoints don't get double-counted across adjacent edges.
  for (std::size_t ei = 0; ei < m_roadGraph.edges.size() && ei < agg.edges.size(); ++ei) {
    m_transitEdgeDemand[ei] = agg.edges[ei].sumTrafficInterior;
  }

  // Keep the planner deterministic per-world unless the user tweaks seedSalt explicitly.
  TransitPlannerConfig cfg = tm.plannerCfg;
  if (cfg.seedSalt == 0) {
    cfg.seedSalt = (m_world.seed() ^ 0xA2B3C4D5E6F70911ULL) ^ (static_cast<std::uint64_t>(dm) * 0x9E3779B97F4A7C15ULL);
  }

  m_transitPlan = PlanTransitLines(m_roadGraph, m_transitEdgeDemand, cfg, &m_world);
  m_transitPlan.cfg = cfg;

  // Clamp selected line to the new plan.
  if (m_transitSelectedLine >= static_cast<int>(m_transitPlan.lines.size())) {
    m_transitSelectedLine = static_cast<int>(m_transitPlan.lines.size()) - 1;
  }
  if (m_transitPlan.lines.empty()) {
    m_transitSelectedLine = -1;
  }

  m_transitPlanDirty = false;
  m_transitVizDirty = true;
  m_evacDirty = true;
}

void Game::ensureTransitVizUpToDate()
{
  ensureTransitPlanUpToDate();
  if (!m_transitVizDirty) return;

  m_transitViz.clear();
  m_transitViz.reserve(m_transitPlan.lines.size());

  const int stopSpacing = std::max(2, m_sim.transitModel().stopSpacingTiles);

  for (std::size_t i = 0; i < m_transitPlan.lines.size(); ++i) {
    const TransitLine& line = m_transitPlan.lines[i];

    TransitLineViz viz;
    viz.lineIndex = static_cast<int>(i);
    viz.id = line.id;
    viz.sumDemand = line.sumDemand;
    viz.baseCost = line.baseCost;

    if (!BuildTransitLineTilePolyline(m_roadGraph, line, viz.tiles)) continue;
    (void)BuildTransitLineStopTiles(m_roadGraph, line, stopSpacing, viz.stops);

    m_transitViz.push_back(std::move(viz));
  }

  m_transitVizDirty = false;
}

void Game::ensureTradeMarketUpToDate()
{
  const Stats& st = m_world.stats();
  const TradeModelSettings& ts = m_sim.tradeModel();

  // World edits that affect goods flow should also invalidate the trade breakdown.
  if (m_goodsDirty) m_tradeDirty = true;

  if (!m_tradeDirty && m_tradeCachedDay == st.day && ts == m_tradeCachedSettings) return;

  m_tradeCachedDay = st.day;
  m_tradeCachedSettings = ts;

  // The trade breakdown UI only needs the aggregate imported/exported totals.
  GoodsResult gg;
  gg.goodsImported = st.goodsImported;
  gg.goodsExported = st.goodsExported;

  const TradeMarketSummary plan = PlanTradeMarket(m_world, st.day, ts);
  m_tradeMarket = ComputeTradeMarket(m_world, st.day, ts, gg, plan);

  m_tradeDirty = false;
}

void Game::ensureServicesHeatmapsUpToDate()
{
  const int w = m_world.width();
  const int h = m_world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));

  const bool sizeOk = (m_servicesHeatmaps.overall.size() == n &&
                       m_servicesHeatmaps.education.size() == n &&
                       m_servicesHeatmaps.health.size() == n &&
                       m_servicesHeatmaps.safety.size() == n);

  if (!m_servicesHeatmapsDirty && sizeOk) return;

  ServicesModelSettings cfg = m_sim.servicesModel();
  cfg.enabled = true;

  const std::vector<ServiceFacility> facilities = ExtractServiceFacilitiesFromWorld(m_world);

  // Recompute on-demand for visualization.
  m_servicesHeatmaps = ComputeServices(m_world, cfg, facilities);

  m_servicesHeatmapsDirty = false;
  m_servicesHeatmapsTimer = 0.0f;
}

void Game::ensureFireRiskUpToDate()
{
  const int w = m_world.width();
  const int h = m_world.height();
  const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));

  const bool sizeOk = (m_fireRisk.risk01.size() == n && m_fireRisk.coverage01.size() == n && m_fireRisk.responseCostMilli.size() == n);
  if (!m_fireRiskDirty && sizeOk) return;

  FireRiskConfig cfg = m_fireRiskCfg;
  // Keep the outside-connection rule consistent with current policy.
  cfg.requireOutsideConnection = m_sim.config().requireOutsideConnection;
  cfg.responseRadiusSteps = std::clamp(cfg.responseRadiusSteps, 6, 128);

  m_fireRisk = ComputeFireRisk(m_world, cfg);

  m_fireRiskDirty = false;
  m_fireRiskTimer = 0.0f;
}



namespace {

int ServiceBuildCostForLevel(int level)
{
  level = std::clamp(level, 0, 3);
  if (level <= 0) return 0;
  return 25 + 20 * (level - 1);
}

Tool ToolForServiceType(ServiceType t, Tool safetyTool)
{
  switch (t) {
    case ServiceType::Education: return Tool::School;
    case ServiceType::Health: return Tool::Hospital;
    case ServiceType::Safety:
      return (safetyTool == Tool::FireStation) ? Tool::FireStation : Tool::PoliceStation;
    default: return Tool::School;
  }
}

const char* ServiceTypeName(ServiceType t)
{
  switch (t) {
    case ServiceType::Education: return "Education";
    case ServiceType::Health: return "Health";
    case ServiceType::Safety: return "Safety";
    default: return "Service";
  }
}

Color ServiceTypeColor(ServiceType t, unsigned char alpha)
{
  // Semantic colors (roughly: blue=education, green=health, red=safety).
  Color c = WHITE;
  switch (t) {
    case ServiceType::Education: c = Color{80, 170, 255, 255}; break;
    case ServiceType::Health: c = Color{90, 220, 150, 255}; break;
    case ServiceType::Safety: c = Color{255, 110, 90, 255}; break;
    default: c = Color{220, 220, 220, 255}; break;
  }
  c.a = alpha;
  return c;
}

bool IsEmptyBuildableLand(const World& world, int x, int y)
{
  if (!world.inBounds(x, y)) return false;
  const Tile& t = world.at(x, y);
  return (t.terrain != Terrain::Water) && (t.overlay == Overlay::None);
}

} // namespace

void Game::ensureServicesPlanUpToDate()
{
  const int w = m_world.width();
  const int h = m_world.height();
  const int n = std::max(0, w) * std::max(0, h);

  // Sync the planner's outside-connection rule to the active policy.
  const bool requireOutside = m_sim.config().requireOutsideConnection;
  if (m_servicesPlanCfg.modelCfg.requireOutsideConnection != requireOutside) {
    m_servicesPlanCfg.modelCfg.requireOutsideConnection = requireOutside;
    m_servicesPlanDirty = true;
  }

  // Keep the planner's model weights in sync with the simulation model (so
  // the suggested facilities match the currently simulated service demand).
  {
    const ServicesModelSettings& sm = m_sim.servicesModel();
    ServicesModelSettings& pm = m_servicesPlanCfg.modelCfg;

    // The optimizer doesn't use `enabled`, but for clarity we treat planning as
    // "enabled" so it always produces suggestions.
    pm.enabled = true;

    pm.catchmentRadiusSteps = sm.catchmentRadiusSteps;
    pm.decayGamma = sm.decayGamma;
    pm.decayBetaBaseMilli = sm.decayBetaBaseMilli;
    pm.includeResidential = sm.includeResidential;
    pm.includeCommercial = sm.includeCommercial;
    pm.includeIndustrial = sm.includeIndustrial;
    pm.demandWeightResidential = sm.demandWeightResidential;
    pm.demandWeightCommercial = sm.demandWeightCommercial;
    pm.demandWeightIndustrial = sm.demandWeightIndustrial;
    pm.satisfactionK = sm.satisfactionK;
    pm.satisfactionMid = sm.satisfactionMid;

    // Outside-connection rule is driven by policy.
    pm.requireOutsideConnection = requireOutside;
  }

  // If nothing is dirty and dimensions match, keep the cached plan.
  if (!m_servicesPlanDirty && m_servicesPlanValid && m_servicesPlan.w == w && m_servicesPlan.h == h) {
    return;
  }

  if (n <= 0) {
    m_servicesPlan = ServiceOptimizerResult{};
    m_servicesPlanValid = false;
    m_servicesPlanDirty = false;
    m_servicesPlanSelection = -1;
    m_servicesPlanFirst = 0;
    return;
  }

  // Outside-connection mask (reused by other overlays).
  const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
  if (requireOutside) {
    if (static_cast<int>(m_outsideOverlayRoadToEdge.size()) != n) {
      ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
    }
    if (static_cast<int>(m_outsideOverlayRoadToEdge.size()) == n) {
      roadToEdgeMask = &m_outsideOverlayRoadToEdge;
    }
  }

  const std::vector<ServiceFacility> facilities = ExtractServiceFacilitiesFromWorld(m_world);

  ServiceOptimizerConfig cfg = m_servicesPlanCfg;

  // Sanitize config.
  cfg.facilitiesToAdd = std::clamp(cfg.facilitiesToAdd, 1, 64);
  cfg.facilityLevel = static_cast<std::uint8_t>(std::clamp<int>(cfg.facilityLevel, 1, 3));
  cfg.candidateLimit = std::clamp(cfg.candidateLimit, 50, 5000);
  cfg.minSeparationMilli = std::max(0, cfg.minSeparationMilli);

  m_servicesPlan = SuggestServiceFacilities(m_world, cfg, facilities, /*precomputedZoneAccess=*/nullptr, roadToEdgeMask);
  m_servicesPlanValid = (m_servicesPlan.w == w && m_servicesPlan.h == h);
  m_servicesPlanDirty = false;

  const int count = static_cast<int>(m_servicesPlan.placements.size());
  if (count <= 0) {
    m_servicesPlanSelection = -1;
    m_servicesPlanFirst = 0;
  } else {
    if (m_servicesPlanSelection < 0 || m_servicesPlanSelection >= count) {
      m_servicesPlanSelection = 0;
    }
    const int maxFirst = std::max(0, count - 1);
    m_servicesPlanFirst = std::clamp(m_servicesPlanFirst, 0, maxFirst);
  }
}

void Game::drawServicesOverlay()
{
  const bool show = (m_showServicesPanel || m_showServicesOverlay);
  if (!show) return;

  ensureServicesPlanUpToDate();
  if (!m_servicesPlanValid || m_servicesPlan.placements.empty()) return;

  const ServiceType type = m_servicesPlan.cfg.type;

  BeginMode2D(m_camera);

  const float zoom = std::max(0.25f, m_camera.zoom);
  const float rBase = 4.0f / zoom;
  const float lineThick = 1.6f / zoom;
  const float ringThick = 1.2f / zoom;

  // Hover highlight via the already-tracked hovered tile.
  int hoverIdx = -1;
  if (m_hovered.has_value()) {
    const Point ht = m_hovered.value();
    for (int i = 0; i < static_cast<int>(m_servicesPlan.placements.size()); ++i) {
      const ServicePlacement& p = m_servicesPlan.placements[static_cast<std::size_t>(i)];
      if ((p.facility.tile.x == ht.x && p.facility.tile.y == ht.y) ||
          (p.accessRoad.x == ht.x && p.accessRoad.y == ht.y)) {
        hoverIdx = i;
        break;
      }
    }
  }

  const int selected = m_servicesPlanSelection;

  for (int i = 0; i < static_cast<int>(m_servicesPlan.placements.size()); ++i) {
    const ServicePlacement& p = m_servicesPlan.placements[static_cast<std::size_t>(i)];
    const Point ft = p.facility.tile;
    const Point ar = p.accessRoad;

    const Vector2 wf = TileToWorldCenterElevated(m_world, ft.x, ft.y,
                                                 static_cast<float>(m_cfg.tileWidth),
                                                 static_cast<float>(m_cfg.tileHeight),
                                                 m_elev);
    const Vector2 wr = TileToWorldCenterElevated(m_world, ar.x, ar.y,
                                                 static_cast<float>(m_cfg.tileWidth),
                                                 static_cast<float>(m_cfg.tileHeight),
                                                 m_elev);

    const bool isSel = (i == selected);
    const bool isHover = (i == hoverIdx);

    const unsigned char alpha = isSel ? 225 : (isHover ? 190 : 120);
    const Color c = ServiceTypeColor(type, alpha);

    const Color lineC = Color{c.r, c.g, c.b, static_cast<unsigned char>(std::min<int>(255, alpha / 2 + 30))};
    DrawLineEx(wf, wr, lineThick, lineC);

    float rr = rBase;
    if (isSel) rr *= 1.9f;
    if (isHover) rr *= 1.5f;

    DrawCircleV(wf, rr, c);

    if (isSel) {
      const Color ringC = Color{255, 255, 255, 180};
      DrawRing(wf, rr + 1.5f / zoom, rr + 1.5f / zoom + ringThick, 0.0f, 360.0f, 20, ringC);
    }
  }

  EndMode2D();
}

bool Game::applyServicePlacement(std::size_t idx)
{
  ensureServicesPlanUpToDate();
  if (!m_servicesPlanValid) return false;
  if (idx >= m_servicesPlan.placements.size()) return false;

  // Determine the tool and target level.
  const ServiceType type = m_servicesPlan.cfg.type;
  const Tool tool = ToolForServiceType(type, m_servicesPlanTool);
  const int targetLevel = std::clamp<int>(static_cast<int>(m_servicesPlan.cfg.facilityLevel), 1, 3);

  const ServicePlacement& p = m_servicesPlan.placements[idx];

  // Determine the build tile (normally `facility.tile`, but if the plan is in
  // "place on road tile" mode we try to pick an adjacent empty land tile).
  Point build = p.facility.tile;
  if (!m_world.inBounds(build.x, build.y)) return false;

  const Tile& t0 = m_world.at(build.x, build.y);
  if (t0.overlay == Overlay::Road) {
    // Find empty adjacent land around the access road.
    constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    bool found = false;
    for (const auto& d : dirs) {
      const int nx = p.accessRoad.x + d[0];
      const int ny = p.accessRoad.y + d[1];
      if (IsEmptyBuildableLand(m_world, nx, ny)) {
        build = Point{nx, ny};
        found = true;
        break;
      }
    }
    if (!found) {
      showToast("Services: no adjacent empty land to build", 2.5f);
      return false;
    }
  }

  if (!IsEmptyBuildableLand(m_world, build.x, build.y)) {
    // Allow upgrading an existing matching facility if present.
    const Tile& t = m_world.at(build.x, build.y);
    bool match = false;
    if (type == ServiceType::Education && t.overlay == Overlay::School) match = true;
    if (type == ServiceType::Health && t.overlay == Overlay::Hospital) match = true;
    if (type == ServiceType::Safety &&
        (t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation)) {
      // For safety, allow upgrading either kind, regardless of selected tool.
      match = true;
    }
    if (!match) {
      showToast("Services: can't build here", 2.0f);
      return false;
    }
  }

  const Tile& before = m_world.at(build.x, build.y);
  const int currentLevel =
      (before.overlay == Overlay::School || before.overlay == Overlay::Hospital ||
       before.overlay == Overlay::PoliceStation || before.overlay == Overlay::FireStation)
          ? before.level
          : 0;

  if (currentLevel >= targetLevel) {
    showToast("Services: already at target level", 1.8f);
    return false;
  }

  const int needCost = ServiceBuildCostForLevel(targetLevel) - ServiceBuildCostForLevel(currentLevel);
  if (needCost > std::max(0, m_world.stats().money)) {
    showToast("Services: not enough money", 2.0f);
    return false;
  }

  endPaintStroke();
  m_history.beginStroke(m_world);

  // Only one tile changes for service placement.
  m_history.noteTilePreEdit(build.x, build.y, before);

  bool anyApplied = false;
  for (int step = currentLevel; step < targetLevel; ++step) {
    const ToolApplyResult r = m_world.applyTool(tool, build.x, build.y);
    if (r != ToolApplyResult::Applied) {
      if (!anyApplied) {
        // The stroke will be empty (no tile edits / money), so undo won't add a step.
        if (r == ToolApplyResult::InsufficientFunds) showToast("Services: not enough money", 2.0f);
        else if (r == ToolApplyResult::BlockedNoRoad) showToast("Services: needs road access", 2.0f);
        else if (r == ToolApplyResult::BlockedNotEmpty || r == ToolApplyResult::BlockedOccupied)
          showToast("Services: tile not empty", 2.0f);
        else if (r == ToolApplyResult::BlockedWater) showToast("Services: can't build on water", 2.0f);
        else showToast("Services: build failed", 2.0f);
      }
      break;
    }
    anyApplied = true;
  }

  (void)m_history.endStroke(m_world, /*outCmd=*/nullptr);

  if (!anyApplied) {
    // Nothing changed.
    return false;
  }

  // One-off edit: update render caches + derived models like endPaintStroke().
  m_renderer.markMinimapDirty();
  m_renderer.markBaseCacheDirtyForTiles(std::vector<Point>{build}, m_world.width(), m_world.height());

  m_3dPreviewDirty = true;
  m_3dPreviewTimer = 0.0f;

  m_sim.refreshDerivedStats(m_world);

  m_trafficDirty = true;
  m_goodsDirty = true;
  m_tradeDirty = true;
  m_landValueDirty = true;
  m_vehiclesDirty = true;

  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;

  m_servicesHeatmapsDirty = true;
  m_servicesPlanDirty = true;
  m_servicesPlanValid = false;
  invalidateAnalysisLayers();

  m_wayfindingDirty = true;
  clearWayfindingRoute();

  showToast(TextFormat("Built %s (lvl %d)", ServiceTypeName(type), targetLevel), 2.0f);
  return true;
}

void Game::exportServicesArtifacts()
{
  ensureServicesPlanUpToDate();
  if (!m_servicesPlanValid || m_servicesPlan.placements.empty()) {
    showToast("Services export: no plan", 2.5f);
    return;
  }

  namespace fs = std::filesystem;
  fs::create_directories("captures");

  const std::string base = TextFormat("captures/services_seed%llu_%s",
                                      static_cast<unsigned long long>(m_world.seed()),
                                      FileTimestamp().c_str());

  const ServiceType type = m_servicesPlan.cfg.type;

  // JSON
  {
    const std::string path = base + ".json";
    std::ofstream f(path, std::ios::out | std::ios::trunc);
    if (!f) {
      showToast("Services export failed: can't write JSON", 3.0f);
    } else {
      f << "{\n";
      f << "  \"seed\": " << static_cast<unsigned long long>(m_world.seed()) << ",\n";
      f << "  \"type\": \"" << ServiceTypeName(type) << "\",\n";
      f << "  \"facilityLevel\": " << static_cast<int>(m_servicesPlan.cfg.facilityLevel) << ",\n";
      f << "  \"facilitiesToAdd\": " << m_servicesPlan.cfg.facilitiesToAdd << ",\n";
      f << "  \"candidateLimit\": " << m_servicesPlan.cfg.candidateLimit << ",\n";
      f << "  \"requireEmptyLand\": " << (m_servicesPlan.cfg.requireEmptyLand ? "true" : "false") << ",\n";
      f << "  \"requireStableAccessRoad\": " << (m_servicesPlan.cfg.requireStableAccessRoad ? "true" : "false") << ",\n";
      f << "  \"minSeparationMilli\": " << m_servicesPlan.cfg.minSeparationMilli << ",\n";
      f << "  \"considerOnlySameTypeExisting\": "
        << (m_servicesPlan.cfg.considerOnlySameTypeExisting ? "true" : "false") << ",\n";
      f << "  \"totalDemandWeight\": " << static_cast<unsigned long long>(m_servicesPlan.totalDemandWeight) << ",\n";
      f << "  \"placements\": [\n";
      for (std::size_t i = 0; i < m_servicesPlan.placements.size(); ++i) {
        const ServicePlacement& p = m_servicesPlan.placements[i];
        f << "    {\n";
        f << "      \"rank\": " << (i + 1) << ",\n";
        f << "      \"facility\": {\"x\": " << p.facility.tile.x << ", \"y\": " << p.facility.tile.y
          << ", \"level\": " << p.facility.level << "},\n";
        f << "      \"accessRoad\": {\"x\": " << p.accessRoad.x << ", \"y\": " << p.accessRoad.y << "},\n";
        f << "      \"marginalGain\": " << p.marginalGain << ",\n";
        f << "      \"localDemandSum\": " << p.localDemandSum << ",\n";
        f << "      \"demandGainRatio\": " << p.demandGainRatio << "\n";
        f << "    }" << (i + 1 < m_servicesPlan.placements.size() ? "," : "") << "\n";
      }
      f << "  ]\n";
      f << "}\n";
      showToast(std::string("Services export: ") + path, 3.0f);
    }
  }

  // Quick-look raster overlay (tile-space, scaled): placements on top of a dark background.
  {
    const int w = m_world.width();
    const int h = m_world.height();
    PpmImage img;
    img.width = w;
    img.height = h;
    img.rgb.assign(static_cast<std::size_t>(std::max(0, w) * std::max(0, h) * 3), 0);

    auto put = [&](int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
      if (x < 0 || y < 0 || x >= w || y >= h) return;
      const std::size_t idx3 = static_cast<std::size_t>((y * w + x) * 3);
      img.rgb[idx3 + 0] = r;
      img.rgb[idx3 + 1] = g;
      img.rgb[idx3 + 2] = b;
    };

    // Background: slightly brighter for land than water.
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const Tile& t = m_world.at(x, y);
        if (t.terrain == Terrain::Water) put(x, y, 10, 12, 18);
        else put(x, y, 18, 18, 22);
      }
    }

    // Placements.
    for (const ServicePlacement& p : m_servicesPlan.placements) {
      const Point ft = p.facility.tile;
      const Point ar = p.accessRoad;
      const Color c = ServiceTypeColor(type, 255);
      put(ar.x, ar.y, static_cast<std::uint8_t>(c.r / 2), static_cast<std::uint8_t>(c.g / 2),
          static_cast<std::uint8_t>(c.b / 2));
      put(ft.x, ft.y, c.r, c.g, c.b);
    }

    const int maxDim = std::max(1, std::max(w, h));
    const int scale = std::clamp(2048 / maxDim, 1, 8);
    if (scale > 1) {
      img = ScaleNearest(img, scale);
    }

    std::string err;
    const std::string path = base + ".png";
    const bool ok = WriteImageAuto(path, img, err);
    if (!ok) {
      showToast(std::string("Services PNG failed: ") + err, 3.0f);
    }
  }
}

void Game::drawServicesPanel(int x0, int y0)
{
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);
  const float uiTime = static_cast<float>(GetTime());

  const int panelW = kServicesPanelW;
  const int panelH = kServicesPanelH;
  const Rectangle panel{static_cast<float>(x0), static_cast<float>(y0),
                        static_cast<float>(panelW), static_cast<float>(panelH)};

  ui::DrawPanel(panel, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panel, "Services Planner", uiTime, /*active=*/true, /*titleSizePx=*/20);

  const int x = x0 + 12;
  int y = y0 + 42;

  const Stats& st = m_world.stats();

  ui::Text(x, y, 14,
           TextFormat("Satisfaction: %.0f%%  (E %.0f / H %.0f / S %.0f)",
                      static_cast<double>(st.servicesOverallSat * 100.0f),
                      static_cast<double>(st.servicesEducationSat * 100.0f),
                      static_cast<double>(st.servicesHealthSat * 100.0f),
                      static_cast<double>(st.servicesSafetySat * 100.0f)),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 16;
  ui::Text(x, y, 14,
           TextFormat("Facilities: E %d  H %d  S %d   Maint: $%d",
                      st.servicesEducationFacilities,
                      st.servicesHealthFacilities,
                      st.servicesSafetyFacilities,
                      st.servicesMaintenanceCost),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 18;

  ensureServicesPlanUpToDate();

  // Buttons row.
  const int btnH = 20;
  const int gap = 6;
  const int bw = 100;
  const Rectangle br0{static_cast<float>(x), static_cast<float>(y), static_cast<float>(bw), static_cast<float>(btnH)};
  const Rectangle br1{static_cast<float>(x + (bw + gap) * 1), static_cast<float>(y), static_cast<float>(bw), static_cast<float>(btnH)};
  const Rectangle br2{static_cast<float>(x + (bw + gap) * 2), static_cast<float>(y), static_cast<float>(bw), static_cast<float>(btnH)};
  const Rectangle br3{static_cast<float>(x + (bw + gap) * 3), static_cast<float>(y), static_cast<float>(bw), static_cast<float>(btnH)};

  if (ui::Button(24000, br0, "Replan", mouseUi, uiTime, /*enabled=*/true, /*primary=*/true)) {
    m_servicesPlanDirty = true;
    ensureServicesPlanUpToDate();
    showToast("Services: replanned", 1.8f);
  }

  auto applyAll = [&]() {
    ensureServicesPlanUpToDate();
    if (!m_servicesPlanValid || m_servicesPlan.placements.empty()) {
      showToast("Services: no plan", 2.0f);
      return;
    }

    const ServiceType type = m_servicesPlan.cfg.type;
    const Tool tool = ToolForServiceType(type, m_servicesPlanTool);
    const int targetLevel = std::clamp<int>(static_cast<int>(m_servicesPlan.cfg.facilityLevel), 1, 3);

    // Build tiles list (resolve road-tile placements to adjacent empty land). Filter to tiles
    // that are either empty buildable land, or an existing service facility we can upgrade.
    std::vector<Point> buildTiles;
    buildTiles.reserve(m_servicesPlan.placements.size());

    for (const ServicePlacement& p : m_servicesPlan.placements) {
      Point build = p.facility.tile;
      if (!m_world.inBounds(build.x, build.y)) continue;

      const Tile& t0 = m_world.at(build.x, build.y);
      if (t0.overlay == Overlay::Road) {
        // If the suggested facility tile is a road, place adjacent to the access road instead.
        constexpr int dirs[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
        bool found = false;
        for (const auto& d : dirs) {
          const int nx = p.accessRoad.x + d[0];
          const int ny = p.accessRoad.y + d[1];
          if (IsEmptyBuildableLand(m_world, nx, ny)) {
            build = Point{nx, ny};
            found = true;
            break;
          }
        }
        if (!found) continue;
      }

      if (!m_world.inBounds(build.x, build.y)) continue;

      const Tile& t = m_world.at(build.x, build.y);

      const bool empty = IsEmptyBuildableLand(m_world, build.x, build.y);
      const bool isSvc =
          (t.overlay == Overlay::School || t.overlay == Overlay::Hospital ||
           t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation);

      bool match = empty;
      if (isSvc) {
        if (type == ServiceType::Education && t.overlay == Overlay::School) match = true;
        if (type == ServiceType::Health && t.overlay == Overlay::Hospital) match = true;
        if (type == ServiceType::Safety &&
            (t.overlay == Overlay::PoliceStation || t.overlay == Overlay::FireStation)) {
          match = true;
        }
      }

      if (!match) continue;

      const int cur = isSvc ? t.level : 0;
      if (cur >= targetLevel) continue; // already at target

      buildTiles.push_back(build);
    }

    if (buildTiles.empty()) {
      showToast("Services: no buildable placements", 2.0f);
      return;
    }

    // De-dupe in case the plan yields duplicates.
    std::sort(buildTiles.begin(), buildTiles.end(), [](const Point& a, const Point& b) {
      if (a.y != b.y) return a.y < b.y;
      return a.x < b.x;
    });
    buildTiles.erase(std::unique(buildTiles.begin(), buildTiles.end(), [](const Point& a, const Point& b) {
                      return a.x == b.x && a.y == b.y;
                    }),
                    buildTiles.end());

    endPaintStroke();
    m_history.beginStroke(m_world);

    // Note pre-edit for all tiles we might touch.
    for (const Point& bt : buildTiles) {
      m_history.noteTilePreEdit(bt.x, bt.y, m_world.at(bt.x, bt.y));
    }

    int builtToTarget = 0;
    int upgradedOrPlaced = 0;
    bool outOfFunds = false;

    std::vector<Point> changedTiles;
    changedTiles.reserve(buildTiles.size());

    for (const Point& bt : buildTiles) {
      const Tile& before = m_world.at(bt.x, bt.y);
      const bool isSvc =
          (before.overlay == Overlay::School || before.overlay == Overlay::Hospital ||
           before.overlay == Overlay::PoliceStation || before.overlay == Overlay::FireStation);
      const int cur = isSvc ? before.level : 0;

      bool changed = false;

      for (int step = cur; step < targetLevel; ++step) {
        const ToolApplyResult r = m_world.applyTool(tool, bt.x, bt.y);
        if (r == ToolApplyResult::Applied) {
          changed = true;
          continue;
        }

        if (r == ToolApplyResult::InsufficientFunds) {
          outOfFunds = true;
        }
        break;
      }

      if (changed) {
        upgradedOrPlaced += 1;
        changedTiles.push_back(bt);
      }

      const Tile& after = m_world.at(bt.x, bt.y);
      const bool isSvcAfter =
          (after.overlay == Overlay::School || after.overlay == Overlay::Hospital ||
           after.overlay == Overlay::PoliceStation || after.overlay == Overlay::FireStation);
      if (isSvcAfter && after.level >= targetLevel) {
        builtToTarget += 1;
      }

      if (outOfFunds) break;
    }

    (void)m_history.endStroke(m_world, /*outCmd=*/nullptr);

    if (changedTiles.empty()) {
      showToast("Services: build failed", 2.0f);
      return;
    }

    m_renderer.markMinimapDirty();
    m_renderer.markBaseCacheDirtyForTiles(changedTiles, m_world.width(), m_world.height());

    m_3dPreviewDirty = true;
    m_3dPreviewTimer = 0.0f;

    m_sim.refreshDerivedStats(m_world);

    m_trafficDirty = true;
    m_goodsDirty = true;
    m_tradeDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
    m_servicesHeatmapsDirty = true;
    m_fireRiskDirty = true;
    m_servicesPlanDirty = true;
    m_servicesPlanValid = false;
    m_wayfindingDirty = true;
    clearWayfindingRoute();

    if (outOfFunds) {
      showToast(TextFormat("Built %d/%d %s (ran out of funds)", builtToTarget, upgradedOrPlaced, ServiceTypeName(type)),
                3.0f);
    } else {
      showToast(TextFormat("Built %d/%d %s", builtToTarget, upgradedOrPlaced, ServiceTypeName(type)), 2.5f);
    }
  };


  const bool hasSelected = (m_servicesPlanSelection >= 0);
  if (ui::Button(24001, br1, "Build sel", mouseUi, uiTime, /*enabled=*/hasSelected, /*primary=*/false)) {
    if (m_servicesPlanSelection >= 0) {
      (void)applyServicePlacement(static_cast<std::size_t>(m_servicesPlanSelection));
    }
  }

  if (ui::Button(24002, br2, "Build all", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
    applyAll();
  }

  if (ui::Button(24003, br3, "Export", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
    exportServicesArtifacts();
  }

  y += btnH + 10;

  // Config list.
  ServiceOptimizerConfig& cfg = m_servicesPlanCfg;
  ServicesModelSettings& sm = m_sim.servicesModel();

  auto markPlanDirty = [&]() {
    m_servicesPlanDirty = true;
    m_servicesPlanValid = false;
  };

  auto refreshSim = [&]() {
    m_sim.refreshDerivedStats(m_world);
    m_servicesHeatmapsDirty = true;
    m_fireRiskDirty = true;
    markPlanDirty();

    // Demand/happiness affects downstream models.
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_tradeDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
  };

  constexpr int rowH = 20;
  constexpr int rows = 10;
  const int listH = rowH * rows + 8;
  const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y),
                        static_cast<float>(panelW - 24), static_cast<float>(listH)};
  ui::DrawPanelInset(listR, uiTime, /*active=*/true);

  const int listX = static_cast<int>(listR.x) + 6;
  const int xValueRight = static_cast<int>(listR.x + listR.width) - 8;
  int rowY = static_cast<int>(listR.y) + 4;

  auto rowRectFor = [&](int yy) -> Rectangle {
    return Rectangle{static_cast<float>(listR.x + 4), static_cast<float>(yy),
                     static_cast<float>(listR.width - 8), static_cast<float>(rowH)};
  };

  auto selectRowOnClick = [&](int idx, Rectangle rr) {
    if (CheckCollisionPointRec(mouseUi, rr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_servicesSelection = idx;
    }
  };

  auto drawToggleRow = [&](int idx, std::string_view label, bool& v, bool refresh, bool replan) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selectedRow = (m_servicesSelection == idx);
    if (selectedRow) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selectedRow ? uiTh.text : uiTh.textDim;
    ui::Text(listX, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
    if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_servicesSelection = idx;
    }

    bool vv = v;
    if (ui::Toggle(24100 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
      v = vv;
      if (refresh) refreshSim();
      if (replan) markPlanDirty();
    }

    rowY += rowH;
  };

  auto drawIntSliderRow = [&](int idx, std::string_view label, int& v, int vMin, int vMax, int step,
                              bool refresh, bool replan, const char* fmt) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selectedRow = (m_servicesSelection == idx);
    if (selectedRow) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, v);

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const int sliderW = 120;
    const int sliderH = 14;
    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};

    if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_servicesSelection = idx;
    }

    const Color c = selectedRow ? uiTh.text : uiTh.textDim;
    ui::Text(listX, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    int vv = v;
    if (ui::SliderInt(24200 + idx, sr, vv, vMin, vMax, step, mouseUi, uiTime, /*enabled=*/true)) {
      v = std::clamp(vv, vMin, vMax);
      if (replan) markPlanDirty();
      if (refresh) refreshSim();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);

    rowY += rowH;
  };

  auto drawCycleRow = [&](int idx, std::string_view label, std::string_view value, bool enabled, auto onClick) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selectedRow = (m_servicesSelection == idx);
    if (selectedRow) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selectedRow ? uiTh.text : uiTh.textDim;
    ui::Text(listX, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    const int bw2 = std::max(120, ui::MeasureTextWidth(value, 15, /*bold=*/true, 1) + 18);
    const Rectangle br{static_cast<float>(xValueRight - bw2),
                       static_cast<float>(rowY + (rowH - 18) / 2),
                       static_cast<float>(bw2),
                       18.0f};

    if (ui::Button(24300 + idx, br, value, mouseUi, uiTime, /*enabled=*/enabled, /*primary=*/false)) {
      onClick();
    }

    rowY += rowH;
  };

  // Effective overlay state.
  const bool overlayEffective = (m_showServicesPanel || m_showServicesOverlay);
  const char* overlayLabel = m_showServicesOverlay ? "ON" : (overlayEffective ? "AUTO (panel)" : "OFF");

  // Row indices must match adjustServicesPanel().
  // 0: Overlay
  {
    const int idx = 0;
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selectedRow = (m_servicesSelection == idx);
    if (selectedRow) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selectedRow ? uiTh.text : uiTh.textDim;
    ui::Text(listX, rowY, 15, "Overlay", c, /*bold=*/false, /*shadow=*/true, 1);

    const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
    if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_servicesSelection = idx;
    }

    bool vv = m_showServicesOverlay;
    if (ui::Toggle(24600 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
      m_showServicesOverlay = vv;
      showToast(m_showServicesOverlay ? "Services overlay: ON" : "Services overlay: OFF", 2.0f);
    }

    ui::Text(xValueRight - 52 - ui::MeasureTextWidth(overlayLabel, 13, true, 1), rowY + 2,
             13, overlayLabel, uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);

    rowY += rowH;
  }

  // 1: Optimize type
  drawCycleRow(1, "Optimize", ServiceTypeName(cfg.type), /*enabled=*/true, [&]() {
    const int idx = static_cast<int>(cfg.type);
    const int next = (idx + 1) % 3;
    cfg.type = static_cast<ServiceType>(next);
    markPlanDirty();

    // Default tool for non-safety types.
    if (cfg.type == ServiceType::Education) m_servicesPlanTool = Tool::School;
    if (cfg.type == ServiceType::Health) m_servicesPlanTool = Tool::Hospital;

    showToast(std::string("Services: optimize ") + ServiceTypeName(cfg.type), 2.0f);
  });

  // 2: Tool (only meaningful for Safety; other types are fixed)
  {
    const bool isSafety = (cfg.type == ServiceType::Safety);
    const Tool effectiveTool = ToolForServiceType(cfg.type, m_servicesPlanTool);

    const char* toolName = "School";
    if (effectiveTool == Tool::Hospital) toolName = "Hospital";
    if (effectiveTool == Tool::PoliceStation) toolName = "Police";
    if (effectiveTool == Tool::FireStation) toolName = "Fire";

    drawCycleRow(2, "Facility", toolName, /*enabled=*/isSafety, [&]() {
      if (m_servicesPlanTool == Tool::FireStation) m_servicesPlanTool = Tool::PoliceStation;
      else m_servicesPlanTool = Tool::FireStation;
      showToast(std::string("Services tool: ") +
                    ((m_servicesPlanTool == Tool::FireStation) ? "Fire" : "Police"),
                2.0f);
    });
  }

  // 3: facilities to add
  drawIntSliderRow(3, "Add", cfg.facilitiesToAdd, 1, 32, 1, /*refresh=*/false, /*replan=*/true, "%d");

  // 4: facility level
  {
    int v = static_cast<int>(cfg.facilityLevel);
    drawIntSliderRow(4, "Level", v, 1, 3, 1, /*refresh=*/false, /*replan=*/true, "%d");
    cfg.facilityLevel = static_cast<std::uint8_t>(std::clamp(v, 1, 3));
  }

  // 5: candidate cap
  drawIntSliderRow(5, "Candidates", cfg.candidateLimit, 100, 2000, 50, /*refresh=*/false, /*replan=*/true, "%d");

  // 6: require empty land
  drawToggleRow(6, "Empty land", cfg.requireEmptyLand, /*refresh=*/false, /*replan=*/true);

  // 7: stable access
  drawToggleRow(7, "Stable access", cfg.requireStableAccessRoad, /*refresh=*/false, /*replan=*/true);

  // 8: min separation
  {
    int tiles = std::max(0, cfg.minSeparationMilli) / 1000;
    drawIntSliderRow(8, "Min sep", tiles, 0, 30, 1, /*refresh=*/false, /*replan=*/true, "%d tiles");
    cfg.minSeparationMilli = std::max(0, tiles) * 1000;
  }

  // 9: model enabled (simulation)
  drawToggleRow(9, "Model", sm.enabled, /*refresh=*/true, /*replan=*/true);

  y = static_cast<int>(listR.y + listR.height) + 10;

  // Placements list.
  ensureServicesPlanUpToDate();

  const int count = (m_servicesPlanValid) ? static_cast<int>(m_servicesPlan.placements.size()) : 0;

  ui::Text(x, y, 15,
           (count > 0)
               ? TextFormat("Suggestions: %d   Cost(each): $%d",
                            count,
                            ServiceBuildCostForLevel(static_cast<int>(cfg.facilityLevel)))
               : "Suggestions: (none)",
           uiTh.text, /*bold=*/false, /*shadow=*/true, 1);
  y += 18;

  const int bottomMargin = 12;
  const int availH = (y0 + panelH - bottomMargin) - y;
  const Rectangle plR{static_cast<float>(x0 + 12), static_cast<float>(y),
                      static_cast<float>(panelW - 24), static_cast<float>(std::max(50, availH))};

  ui::DrawPanelInset(plR, uiTime, /*active=*/true);

  if (count <= 0) {
    ui::Text(static_cast<int>(plR.x) + 8, static_cast<int>(plR.y) + 10, 14,
             "No placements (try adding roads/zones or raising candidate cap).",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    return;
  }

  // Scrollbar.
  const int rowH2 = 18;
  const int viewRows = std::max(1, static_cast<int>((plR.height - 8) / rowH2));
  const int contentRows = count;
  const int maxFirst = std::max(0, contentRows - viewRows);
  m_servicesPlanFirst = std::clamp(m_servicesPlanFirst, 0, maxFirst);

  Rectangle barR{plR.x + plR.width - 12.0f, plR.y + 4.0f, 10.0f, plR.height - 8.0f};
  ui::ScrollbarV(24500, barR, contentRows, viewRows, m_servicesPlanFirst, mouseUi, uiTime, /*enabled=*/true);

  Rectangle contentR = ui::ContentRectWithScrollbar(plR);
  const int rx = static_cast<int>(contentR.x) + 6;
  int ry = static_cast<int>(contentR.y) + 4;

  for (int r = 0; r < viewRows; ++r) {
    const int i = m_servicesPlanFirst + r;
    if (i < 0 || i >= count) break;

    const ServicePlacement& p = m_servicesPlan.placements[static_cast<std::size_t>(i)];
    const Rectangle rr{static_cast<float>(contentR.x + 4),
                       static_cast<float>(ry),
                       static_cast<float>(contentR.width - 8),
                       static_cast<float>(rowH2)};

    const bool sel = (i == m_servicesPlanSelection);
    if (sel) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    if (CheckCollisionPointRec(mouseUi, rr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_servicesPlanSelection = i;
    }

    const Color c = sel ? uiTh.text : uiTh.textDim;

    ui::Text(rx, ry, 14,
             TextFormat("%2d) (%d,%d)  +%.2f  ratio %.2f",
                        i + 1,
                        p.facility.tile.x, p.facility.tile.y,
                        p.marginalGain,
                        p.demandGainRatio),
             c, /*bold=*/false, /*shadow=*/true, 1);

    ry += rowH2;
  }
}

void Game::adjustServicesPanel(int dir, bool bigStep)
{
  const int stepI = bigStep ? 5 : 1;
  const int stepCand = bigStep ? 200 : 50;
  const int stepSep = bigStep ? 5 : 1;

  ServiceOptimizerConfig& cfg = m_servicesPlanCfg;
  ServicesModelSettings& sm = m_sim.servicesModel();

  auto markPlanDirty = [&]() {
    m_servicesPlanDirty = true;
    m_servicesPlanValid = false;
  };

  auto refreshSim = [&]() {
    m_sim.refreshDerivedStats(m_world);
    m_servicesHeatmapsDirty = true;
    m_fireRiskDirty = true;
    markPlanDirty();

    m_trafficDirty = true;
    m_goodsDirty = true;
    m_tradeDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
  };

  switch (m_servicesSelection) {
    case 0: {
      // Overlay toggle
      if (dir != 0) {
        m_showServicesOverlay = !m_showServicesOverlay;
        showToast(m_showServicesOverlay ? "Services overlay: ON" : "Services overlay: OFF", 2.0f);
      }
    } break;
    case 1: {
      const int idx = static_cast<int>(cfg.type);
      const int next = (idx + (dir >= 0 ? 1 : -1) + 3) % 3;
      cfg.type = static_cast<ServiceType>(next);
      if (cfg.type == ServiceType::Education) m_servicesPlanTool = Tool::School;
      if (cfg.type == ServiceType::Health) m_servicesPlanTool = Tool::Hospital;
      markPlanDirty();
      showToast(std::string("Services: optimize ") + ServiceTypeName(cfg.type), 2.0f);
    } break;
    case 2: {
      if (cfg.type == ServiceType::Safety) {
        m_servicesPlanTool = (m_servicesPlanTool == Tool::FireStation) ? Tool::PoliceStation : Tool::FireStation;
        showToast(std::string("Services tool: ") +
                      ((m_servicesPlanTool == Tool::FireStation) ? "Fire" : "Police"),
                  2.0f);
      }
    } break;
    case 3: {
      cfg.facilitiesToAdd = std::clamp(cfg.facilitiesToAdd + dir * stepI, 1, 32);
      markPlanDirty();
    } break;
    case 4: {
      int v = static_cast<int>(cfg.facilityLevel);
      v = std::clamp(v + dir, 1, 3);
      cfg.facilityLevel = static_cast<std::uint8_t>(v);
      markPlanDirty();
    } break;
    case 5: {
      cfg.candidateLimit = std::clamp(cfg.candidateLimit + dir * stepCand, 100, 2000);
      markPlanDirty();
    } break;
    case 6: {
      if (dir != 0) {
        cfg.requireEmptyLand = !cfg.requireEmptyLand;
        markPlanDirty();
      }
    } break;
    case 7: {
      if (dir != 0) {
        cfg.requireStableAccessRoad = !cfg.requireStableAccessRoad;
        markPlanDirty();
      }
    } break;
    case 8: {
      int tiles = std::max(0, cfg.minSeparationMilli) / 1000;
      tiles = std::clamp(tiles + dir * stepSep, 0, 30);
      cfg.minSeparationMilli = tiles * 1000;
      markPlanDirty();
    } break;
    case 9: {
      if (dir != 0) {
        sm.enabled = !sm.enabled;
        refreshSim();
      }
    } break;
    default: break;
  }
}


void Game::drawTransitOverlay()
{
  const bool show = (m_showTransitPanel || m_showTransitOverlay);
  if (!show) return;

  ensureTransitVizUpToDate();
  if (m_transitViz.empty()) return;

  BeginMode2D(m_camera);

  const float zoom = std::max(0.25f, m_camera.zoom);
  const float thickness = 2.8f / zoom;
  const float stopRadius = 3.8f / zoom;

  const bool solo = (m_transitShowOnlySelectedLine && m_transitSelectedLine >= 0);

  for (std::size_t li = 0; li < m_transitViz.size(); ++li) {
    const TransitLineViz& l = m_transitViz[li];
    if (solo && l.lineIndex != m_transitSelectedLine) continue;

    const bool isSelected = (m_transitSelectedLine >= 0 && l.lineIndex == m_transitSelectedLine);
    const unsigned char alpha = (solo || isSelected || m_transitSelectedLine < 0) ? 210 : 90;
    const Color c = TransitLineColor(static_cast<int>(li), alpha);

    for (std::size_t i = 1; i < l.tiles.size(); ++i) {
      const Point& a = l.tiles[i - 1];
      const Point& b = l.tiles[i];
      const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
      const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                   static_cast<float>(m_cfg.tileHeight), m_elev);
      DrawLineEx(wa, wb, thickness, c);
    }

    if (m_transitShowStops) {
      const Color stopC = Color{c.r, c.g, c.b, static_cast<unsigned char>(std::min(255, alpha + 40))};
      for (const Point& s : l.stops) {
        const Vector2 ws = TileToWorldCenterElevated(m_world, s.x, s.y, static_cast<float>(m_cfg.tileWidth),
                                                     static_cast<float>(m_cfg.tileHeight), m_elev);
        DrawCircleV(ws, stopRadius, stopC);
      }
    }
  }

  EndMode2D();
}

void Game::drawTransitPanel(int x0, int y0)
{
  if (!m_showTransitPanel) return;

  ensureTransitVizUpToDate();

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  const int panelW = kTransitPanelW;
  const int panelH = kTransitPanelH;

  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW), static_cast<float>(panelH)};
  ui::DrawPanel(panelR, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panelR, "Transit Planner", uiTime, /*active=*/true, /*titleSizePx=*/22);

  // Local helpers for applying changes in-place.
  TransitModelSettings& tm = m_sim.transitModel();
  TransitPlannerConfig& cfg = tm.plannerCfg;
  auto markPlanDirty = [&]() {
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
  };
  auto refreshSim = [&]() {
    // Update the derived stats immediately so the panel reflects the new setting without requiring a sim tick.
    m_sim.refreshDerivedStats(m_world);
  };

  int x = x0 + 12;
  int y = y0 + 42;

  ui::Text(x, y, 15, "Tab: select  |  [ / ]: adjust  |  Enter: replan", uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
  y += 18;
  ui::Text(x, y, 15, "Ctrl+T: toggle panel  |  Ctrl+Shift+T: export", uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);

  // Quick action buttons (mouse friendly).
  {
    const Rectangle br1{static_cast<float>(x0 + panelW - 170), static_cast<float>(y0 + 40), 74.0f, 18.0f};
    const Rectangle br2{static_cast<float>(x0 + panelW - 90), static_cast<float>(y0 + 40), 78.0f, 18.0f};

    if (ui::Button(6001, br1, "Replan", mouseUi, uiTime, /*enabled=*/true, /*primary=*/true)) {
      m_transitPlanDirty = true;
      ensureTransitVizUpToDate();
      showToast("Transit: replanned", 2.0f);
    }
    if (ui::Button(6002, br2, "Export", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      exportTransitArtifacts();
    }
  }

  y += 22;

  auto rowRectFor = [&](int yRow, int rowH) -> Rectangle {
    return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                     static_cast<float>(rowH)};
  };

  const int rowH = 20;
  const int rows = 19;
  const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                        static_cast<float>(rows * rowH + 8)};
  ui::DrawPanelInset(listR, uiTime, /*active=*/true);

  int rowY = y + 4;

  const int xValueRight = x0 + panelW - 14;
  const int sliderH = 11;
  const int sliderW = 150;

  // Click-to-select helper.
  auto selectRowOnClick = [&](int idx, Rectangle rr) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
      m_transitSelection = idx;
    }
  };

  auto drawToggleRow = [&](int idx, std::string_view label, bool& v, bool refresh, bool replan) {
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
    if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_transitSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    bool vv = v;
    if (ui::Toggle(6100 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
      v = vv;
      if (replan) markPlanDirty();
      if (refresh) refreshSim();
    }

    rowY += rowH;
  };

  auto drawIntSliderRow = [&](int idx, std::string_view label, int& v, int vMin, int vMax, int step,
                              bool refresh, bool replan, const char* fmt = "%d") {
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, v);

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_transitSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    int vv = v;
    if (ui::SliderInt(6200 + idx, sr, vv, vMin, vMax, step, mouseUi, uiTime, /*enabled=*/true)) {
      v = std::clamp(vv, vMin, vMax);
      if (replan) markPlanDirty();
      if (refresh) refreshSim();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  };

  auto drawFloatSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax, bool refresh, bool replan,
                                const char* fmt) {
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, static_cast<double>(v));

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_transitSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    float vv = v;
    if (ui::SliderFloat(6300 + idx, sr, vv, vMin, vMax, mouseUi, uiTime, /*enabled=*/true)) {
      v = std::clamp(vv, vMin, vMax);
      if (replan) markPlanDirty();
      if (refresh) refreshSim();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  };

  auto drawCycleRow = [&](int idx, std::string_view label, std::string_view value, auto onClick) {
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    const int bw = std::max(110, ui::MeasureTextWidth(value, 15, /*bold=*/true, 1) + 18);
    const Rectangle br{static_cast<float>(xValueRight - bw),
                       static_cast<float>(rowY + (rowH - 18) / 2),
                       static_cast<float>(bw),
                       18.0f};
    if (ui::Button(6400 + idx, br, value, mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      onClick();
    }

    rowY += rowH;
  };

  // Effective overlay state.
  const bool overlayEffective = (m_showTransitPanel || m_showTransitOverlay);
  const char* overlayLabel = m_showTransitOverlay ? "ON" : (overlayEffective ? "AUTO (panel)" : "OFF");

  // Keep indices in sync with adjustTransitPanel().
  // 0: Overlay
  {
    const int idx = 0;
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, "Overlay", c, /*bold=*/false, /*shadow=*/true, 1);

    const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
    if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_transitSelection = idx;
    }

    bool vv = m_showTransitOverlay;
    if (ui::Toggle(6100 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
      m_showTransitOverlay = vv;
      showToast(m_showTransitOverlay ? "Transit overlay: ON" : "Transit overlay: OFF", 2.0f);
    }

    // Show the effective label.
    ui::Text(xValueRight - 52 - ui::MeasureTextWidth(overlayLabel, 13, true, 1), rowY + 2,
             13, overlayLabel, uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);

    rowY += rowH;
  }

  // 1: System
  drawToggleRow(1, "System", tm.enabled, /*refresh=*/true, /*replan=*/false);

  // 2..6: simulation knobs
  drawFloatSliderRow(2, "Service", tm.serviceLevel, 0.0f, 3.0f, /*refresh=*/true, /*replan=*/false, "%.2f");
  // Max shift is stored as 0..1; show and edit it as a percentage for readability.
  {
    const int idx = 3;
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    float pct = std::clamp(tm.maxModeShare * 100.0f, 0.0f, 100.0f);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.0f%%", static_cast<double>(pct));

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_transitSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, "Max shift", c, /*bold=*/false, /*shadow=*/true, 1);

    float vv = pct;
    if (ui::SliderFloat(6300 + idx, sr, vv, 0.0f, 100.0f, mouseUi, uiTime, /*enabled=*/true)) {
      pct = std::clamp(vv, 0.0f, 100.0f);
      tm.maxModeShare = std::clamp(pct / 100.0f, 0.0f, 1.0f);
      refreshSim();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  }
  drawFloatSliderRow(4, "Time mult", tm.travelTimeMultiplier, 0.25f, 2.5f, /*refresh=*/true, /*replan=*/false, "%.2f");
  drawIntSliderRow(5, "Cost/tile", tm.costPerTile, 0, 200, 1, /*refresh=*/true, /*replan=*/false);
  drawIntSliderRow(6, "Cost/stop", tm.costPerStop, 0, 500, 1, /*refresh=*/true, /*replan=*/false);

  // 7: Demand mode
  drawCycleRow(7, "Demand", TransitDemandModeName(tm.demandMode), [&]() {
    const int idx = static_cast<int>(tm.demandMode);
    const int next = (idx + 1 + 3) % 3;
    tm.demandMode = static_cast<TransitDemandMode>(next);
    markPlanDirty();
    refreshSim();
    showToast(std::string("Transit demand: ") + TransitDemandModeName(tm.demandMode), 2.0f);
  });

  // 8..9: planner caps
  drawIntSliderRow(8, "Max lines", cfg.maxLines, 1, 32, 1, /*refresh=*/true, /*replan=*/true);
  drawIntSliderRow(9, "Endpoints", cfg.endpointCandidates, 4, 128, 2, /*refresh=*/true, /*replan=*/true);

  // 10: edge weight
  drawCycleRow(10, "Edge weight", TransitEdgeWeightModeName(cfg.weightMode), [&]() {
    cfg.weightMode = (cfg.weightMode == TransitEdgeWeightMode::Steps) ? TransitEdgeWeightMode::TravelTime
                                                                      : TransitEdgeWeightMode::Steps;
    markPlanDirty();
    refreshSim();
  });

  // 11..13: planner floats
  {
    // Demand bias lives as double; bridge to float slider for UI.
    float v = static_cast<float>(cfg.demandBias);
    drawFloatSliderRow(11, "Demand bias", v, 0.0f, 10.0f, /*refresh=*/true, /*replan=*/true, "%.2f");
    cfg.demandBias = static_cast<double>(v);
  }
  {
    float v = static_cast<float>(cfg.maxDetour);
    drawFloatSliderRow(12, "Max detour", v, 1.0f, 3.0f, /*refresh=*/true, /*replan=*/true, "%.2f");
    cfg.maxDetour = static_cast<double>(v);
  }
  {
    float v = static_cast<float>(cfg.coverFraction);
    drawFloatSliderRow(13, "Cover frac", v, 0.0f, 1.0f, /*refresh=*/true, /*replan=*/true, "%.2f");
    cfg.coverFraction = static_cast<double>(v);
  }

  // 14: min line demand (uint64)
  {
    int v = static_cast<int>(std::min<std::uint64_t>(cfg.minLineDemand, 2000ull));
    drawIntSliderRow(14, "Min line", v, 0, 2000, 10, /*refresh=*/true, /*replan=*/true, "%d");
    cfg.minLineDemand = static_cast<std::uint64_t>(std::max(0, v));
  }

  // 15: stop spacing
  drawIntSliderRow(15, "Stop spacing", tm.stopSpacingTiles, 2, 64, 1, /*refresh=*/true, /*replan=*/false, "%d tiles");

  // 16..18 toggles/selection
  drawToggleRow(16, "Show stops", m_transitShowStops, /*refresh=*/false, /*replan=*/false);

  {
    const int count = static_cast<int>(m_transitPlan.lines.size());
    int v = m_transitSelectedLine;
    const int vMax = std::max(-1, count - 1);
    if (v > vMax) v = -1;

    const int idx = 17;
    const Rectangle rr = rowRectFor(rowY, rowH);
    selectRowOnClick(idx, rr);
    const bool selected = (m_transitSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, "Highlight", c, /*bold=*/false, /*shadow=*/true, 1);

    const char* valueText = (count <= 0) ? "None" : ((v < 0) ? "All" : TextFormat("%d / %d", v + 1, count));
    const int valW = ui::MeasureTextWidth(valueText, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;
    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_transitSelection = idx;
    }

    int vv = v;
    if (count > 0 && ui::SliderInt(6200 + idx, sr, vv, -1, vMax, 1, mouseUi, uiTime, /*enabled=*/true)) {
      // Snap: -1 means All.
      if (vv < -1) vv = -1;
      if (vv > vMax) vv = -1;
      m_transitSelectedLine = vv;
    }

    ui::Text(valX, rowY, 15, valueText, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  }

  drawToggleRow(18, "Solo", m_transitShowOnlySelectedLine, /*refresh=*/false, /*replan=*/false);

  // Footer / summary.
  y = rowY + 4;
  DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
  y += 10;

  const int lineCount = static_cast<int>(m_transitPlan.lines.size());
  const double cov =
      (m_transitPlan.totalDemand > 0)
          ? (100.0 * static_cast<double>(m_transitPlan.coveredDemand) / static_cast<double>(m_transitPlan.totalDemand))
          : 0.0;

  ui::Text(x, y, 16, TextFormat("Lines: %d   Coverage: %.0f%%", lineCount, cov), uiTh.text,
           /*bold=*/false, /*shadow=*/true, 1);
  y += 18;
  ui::Text(x, y, 14, TextFormat("Demand: %llu  Covered: %llu",
                                static_cast<unsigned long long>(m_transitPlan.totalDemand),
                                static_cast<unsigned long long>(m_transitPlan.coveredDemand)),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 16;

  // Simulation feedback.
  {
    const Stats& st = m_world.stats();
    ui::Text(x, y, 14,
             TextFormat("Sim riders: %d  Share: %.0f%%  Cost: $%d",
                        st.transitRiders,
                        static_cast<double>(st.transitModeShare * 100.0f),
                        st.transitCost),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 16;
    ui::Text(x, y, 14, TextFormat("Commute coverage: %.0f%%", static_cast<double>(st.transitCommuteCoverage * 100.0f)),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 16;
  }

  if (m_transitSelectedLine >= 0 && m_transitSelectedLine < static_cast<int>(m_transitPlan.lines.size())) {
    const TransitLine& l = m_transitPlan.lines[static_cast<std::size_t>(m_transitSelectedLine)];
    // Find the viz cache for stop count.
    int stopCount = 0;
    for (const TransitLineViz& v : m_transitViz) {
      if (v.lineIndex == m_transitSelectedLine) {
        stopCount = static_cast<int>(v.stops.size());
        break;
      }
    }

    ui::Text(x, y, 13,
             TextFormat("Line %d: demand %llu  cost %llu  stops %d",
                        m_transitSelectedLine + 1,
                        static_cast<unsigned long long>(l.sumDemand),
                        static_cast<unsigned long long>(l.baseCost),
                        stopCount),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  }
}

void Game::adjustTransitPanel(int dir, bool bigStep)
{
  const int stepI = bigStep ? 2 : 1;
  const int stepBigI = bigStep ? 8 : 2;
  const double stepF = bigStep ? 0.25 : 0.05;
  const double stepBias = bigStep ? 0.50 : 0.10;

  const int stepCost = bigStep ? 5 : 1;
  const float stepService = bigStep ? 0.25f : 0.05f;
  const float stepShare = bigStep ? 0.10f : 0.02f;
  const float stepTime = bigStep ? 0.10f : 0.02f;

  TransitModelSettings& tm = m_sim.transitModel();
  TransitPlannerConfig& cfg = tm.plannerCfg;

  auto markPlanDirty = [&]() {
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
  m_evacDirty = true;
  };

  auto refreshSim = [&]() {
    // Update the derived stats immediately so the panel reflects the new setting without requiring a sim tick.
    m_sim.refreshDerivedStats(m_world);
  };

  switch (m_transitSelection) {
  case 0: {
    // Overlay persist toggle.
    if (dir != 0) {
      m_showTransitOverlay = !m_showTransitOverlay;
      showToast(m_showTransitOverlay ? "Transit overlay: ON" : "Transit overlay: OFF", 2.0f);
    }
    break;
  }
  case 1: {
    // Simulation on/off.
    if (dir != 0) {
      tm.enabled = !tm.enabled;
      refreshSim();
      showToast(tm.enabled ? "Transit system: ON" : "Transit system: OFF", 2.0f);
    }
    break;
  }
  case 2: {
    // Service intensity.
    tm.serviceLevel = std::clamp(tm.serviceLevel + static_cast<float>(dir) * stepService, 0.0f, 3.0f);
    refreshSim();
    break;
  }
  case 3: {
    // Maximum mode-shift cap.
    tm.maxModeShare = std::clamp(tm.maxModeShare + static_cast<float>(dir) * stepShare, 0.0f, 1.0f);
    refreshSim();
    break;
  }
  case 4: {
    // Transit travel time multiplier.
    tm.travelTimeMultiplier = std::clamp(tm.travelTimeMultiplier + static_cast<float>(dir) * stepTime, 0.25f, 2.5f);
    refreshSim();
    break;
  }
  case 5: {
    tm.costPerTile = std::clamp(tm.costPerTile + dir * stepCost, 0, 200);
    refreshSim();
    break;
  }
  case 6: {
    tm.costPerStop = std::clamp(tm.costPerStop + dir * stepCost, 0, 500);
    refreshSim();
    break;
  }
  case 7: {
    // Planner demand mode.
    if (dir != 0) {
      const int idx = static_cast<int>(tm.demandMode);
      const int next = (idx + (dir > 0 ? 1 : -1) + 3) % 3;
      tm.demandMode = static_cast<TransitDemandMode>(next);
      markPlanDirty();
      refreshSim();
      showToast(std::string("Transit demand: ") + TransitDemandModeName(tm.demandMode), 2.0f);
    }
    break;
  }
  case 8: {
    cfg.maxLines = std::clamp(cfg.maxLines + dir * stepI, 1, 32);
    markPlanDirty();
    refreshSim();
    break;
  }
  case 9: {
    cfg.endpointCandidates = std::clamp(cfg.endpointCandidates + dir * stepBigI, 4, 128);
    markPlanDirty();
    refreshSim();
    break;
  }
  case 10: {
    if (dir != 0) {
      cfg.weightMode = (cfg.weightMode == TransitEdgeWeightMode::Steps) ? TransitEdgeWeightMode::TravelTime
                                                                        : TransitEdgeWeightMode::Steps;
      markPlanDirty();
      refreshSim();
    }
    break;
  }
  case 11: {
    cfg.demandBias = std::clamp(cfg.demandBias + static_cast<double>(dir) * stepBias, 0.0, 10.0);
    markPlanDirty();
    refreshSim();
    break;
  }
  case 12: {
    cfg.maxDetour = std::clamp(cfg.maxDetour + static_cast<double>(dir) * stepF, 1.0, 3.0);
    markPlanDirty();
    refreshSim();
    break;
  }
  case 13: {
    cfg.coverFraction = std::clamp(cfg.coverFraction + static_cast<double>(dir) * stepF, 0.0, 1.0);
    markPlanDirty();
    refreshSim();
    break;
  }
  case 14: {
    const std::uint64_t step = static_cast<std::uint64_t>(bigStep ? 50 : 10);
    if (dir > 0) {
      cfg.minLineDemand += step;
    } else if (dir < 0) {
      cfg.minLineDemand = (cfg.minLineDemand > step) ? (cfg.minLineDemand - step) : 0;
    }
    markPlanDirty();
    refreshSim();
    break;
  }
  case 15: {
    tm.stopSpacingTiles = std::clamp(tm.stopSpacingTiles + dir * (bigStep ? 4 : 1), 2, 64);
    m_transitVizDirty = true;
  m_evacDirty = true;
    refreshSim();
    break;
  }
  case 16: {
    if (dir != 0) m_transitShowStops = !m_transitShowStops;
    break;
  }
  case 17: {
    const int count = static_cast<int>(m_transitPlan.lines.size());
    if (count <= 0) {
      m_transitSelectedLine = -1;
      break;
    }

    // Cycle: All (-1) then 0..count-1.
    int idx = m_transitSelectedLine;
    if (idx < -1) idx = -1;
    if (idx >= count) idx = count - 1;

    if (dir > 0) {
      idx = (idx < 0) ? 0 : (idx + 1);
      if (idx >= count) idx = -1;
    } else if (dir < 0) {
      idx = (idx < 0) ? (count - 1) : (idx - 1);
      if (idx < 0) idx = -1;
    }

    m_transitSelectedLine = idx;
    break;
  }
  case 18: {
    if (dir != 0) m_transitShowOnlySelectedLine = !m_transitShowOnlySelectedLine;
    break;
  }
  default:
    break;
  }
}

void Game::exportTransitArtifacts()
{
  ensureTransitVizUpToDate();
  if (m_transitPlan.lines.empty() || m_roadGraph.edges.empty()) {
    showToast("Transit export: no plan", 2.5f);
    return;
  }

  namespace fs = std::filesystem;
  fs::create_directories("captures");

  const std::string base = TextFormat("captures/transit_seed%llu_%s",
                                     static_cast<unsigned long long>(m_world.seed()),
                                      FileTimestamp().c_str());

  TransitPlanExportConfig ec;
  ec.includeStops = true;
  ec.includeTiles = true;
  ec.stopMode = TransitStopMode::Tiles;
  ec.stopSpacingTiles = std::max(2, m_sim.transitModel().stopSpacingTiles);

  // GeoJSON + JSON
  {
    std::string err;
    const std::string path = base + ".geojson";
    const bool ok = ExportTransitPlanGeoJson(path, m_roadGraph, m_transitPlan, ec, &err);
    showToast(ok ? (std::string("Transit export: ") + path) : (std::string("Transit export failed: ") + err), 3.0f);
  }
  {
    std::string err;
    const std::string path = base + ".json";
    const bool ok = ExportTransitPlanJson(path, m_roadGraph, m_transitPlan, ec, &err);
    if (!ok) {
      showToast(std::string("Transit JSON failed: ") + err, 3.0f);
    }
  }

  // Quick-look raster overlay (tile-space, scaled).
  {
    std::string err;
    PpmImage img = RenderTransitOverlayTile(m_world, ExportLayer::Overlay, m_roadGraph, m_transitPlan, ec);
    const int maxDim = std::max(1, std::max(m_world.width(), m_world.height()));
    const int scale = std::clamp(2048 / maxDim, 1, 8);
    if (scale > 1) {
      img = ScaleNearest(img, scale);
    }

    const std::string path = base + ".png";
    const bool ok = WriteImageAuto(path, img, err);
    if (!ok) {
      showToast(std::string("Transit PNG failed: ") + err, 3.0f);
    }
  }
}

// -----------------------------------------------------------------------------
// Road upgrade planner (street->avenue/highway suggestions)
// -----------------------------------------------------------------------------

namespace {

const char* RoadUpgradeObjectiveLabel(RoadUpgradeObjective obj)
{
  switch (obj) {
    case RoadUpgradeObjective::Congestion: return "Congestion";
    case RoadUpgradeObjective::Time: return "Time";
    case RoadUpgradeObjective::Hybrid: return "Hybrid";
  }
  return "Congestion";
}

Color RoadUpgradeLevelColor(int level, unsigned char alpha)
{
  // Match export colors (teal for level 2, warm orange-red for level 3), but use
  // alpha-tinted overlays for in-game readability.
  Color c = WHITE;
  switch (ClampRoadLevel(level)) {
    case 2: c = Color{40, 220, 200, 255}; break;
    case 3: c = Color{255, 110, 70, 255}; break;
    default: c = Color{255, 255, 255, 255}; break;
  }
  c.a = alpha;
  return c;
}

} // namespace

void Game::ensureRoadUpgradePlanUpToDate()
{
  // If the plan isn't explicitly dirty, still replan if any of the upstream
  // inputs changed (roads, traffic/goods caches, or auto-budget money).
  const bool needCommute = (m_roadUpgradeDemandMode == TransitDemandMode::Commute) ||
                           (m_roadUpgradeDemandMode == TransitDemandMode::Combined);
  const bool needGoods = (m_roadUpgradeDemandMode == TransitDemandMode::Goods) ||
                         (m_roadUpgradeDemandMode == TransitDemandMode::Combined);

  const int w = m_world.width();
  const int h = m_world.height();
  const int n = std::max(0, w) * std::max(0, h);

  if (!m_roadUpgradePlanDirty) {
    if (m_roadGraphDirty) m_roadUpgradePlanDirty = true;
    if (needCommute && (m_trafficDirty || static_cast<int>(m_traffic.roadTraffic.size()) != n)) m_roadUpgradePlanDirty = true;
    if (needGoods && (m_goodsDirty || static_cast<int>(m_goods.roadGoodsTraffic.size()) != n)) m_roadUpgradePlanDirty = true;
    if (m_roadUpgradeBudgetAuto && (m_roadUpgradePlan.cfg.budget != std::max(0, m_world.stats().money))) m_roadUpgradePlanDirty = true;
    if ((m_roadUpgradePlan.w != w) || (m_roadUpgradePlan.h != h)) m_roadUpgradePlanDirty = true;
  }

  if (!m_roadUpgradePlanDirty) return;

  ensureRoadGraphUpToDate();

  if (m_roadGraph.edges.empty() || n <= 0) {
    m_roadUpgradePlan = RoadUpgradePlan{};
    m_roadUpgradePlanDirty = false;
    m_roadUpgradeSelectedEdge = -1;
    m_roadUpgradeSelectedMaskDirty = true;
    return;
  }

  // Outside-connection rule (optionally cached so we can reuse it across planners/overlays).
  const bool requireOutside = m_sim.config().requireOutsideConnection;
  const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
  if (requireOutside) {
    if (static_cast<int>(m_outsideOverlayRoadToEdge.size()) != n) {
      ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
    }
    if (static_cast<int>(m_outsideOverlayRoadToEdge.size()) == n) {
      roadToEdgeMask = &m_outsideOverlayRoadToEdge;
    }
  }

  // Ensure derived flows exist for the selected demand mode.
  if (needCommute) {
    if (m_trafficDirty || static_cast<int>(m_traffic.roadTraffic.size()) != n) {
      TrafficConfig tc;
      tc.requireOutsideConnection = requireOutside;
      {
        const TrafficModelSettings& tms = m_sim.trafficModel();
        tc.congestionAwareRouting = tms.congestionAwareRouting;
        tc.congestionIterations = tms.congestionIterations;
        tc.congestionAlpha = tms.congestionAlpha;
        tc.congestionBeta = tms.congestionBeta;
        tc.congestionCapacityScale = tms.congestionCapacityScale;
        tc.congestionRatioClamp = tms.congestionRatioClamp;
        tc.capacityAwareJobs = tms.capacityAwareJobs;
        tc.jobAssignmentIterations = tms.jobAssignmentIterations;
        tc.jobPenaltyBaseMilli = tms.jobPenaltyBaseMilli;
      }

      const float employedShare = (m_world.stats().population > 0)
                                      ? (static_cast<float>(m_world.stats().employed) /
                                         static_cast<float>(m_world.stats().population))
                                      : 1.0f;
      const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_traffic = ComputeCommuteTraffic(m_world, tc, employedShare, pre);
      m_trafficDirty = false;
      m_carbonDirty = true;
    }
  }

  if (needGoods) {
    if (m_goodsDirty || static_cast<int>(m_goods.roadGoodsTraffic.size()) != n) {
      GoodsConfig gc;
      gc.requireOutsideConnection = requireOutside;

      const Stats& st = m_world.stats();
      const TradeModelSettings& ts = m_sim.tradeModel();
      if (ts.enabled) {
        gc.allowImports = ts.allowImports;
        gc.allowExports = ts.allowExports;
        gc.importCapacityPct = std::clamp(st.tradeImportCapacityPct, 0, 100);
        gc.exportCapacityPct = std::clamp(st.tradeExportCapacityPct, 0, 100);
      }
      const std::vector<std::uint8_t>* pre = (gc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;
      m_carbonDirty = true;
    }
  }

  // Compose a per-road-tile flow map.
  std::vector<std::uint32_t> roadFlow;
  roadFlow.assign(static_cast<std::size_t>(n), 0u);

  if (needCommute && static_cast<int>(m_traffic.roadTraffic.size()) == n) {
    for (int i = 0; i < n; ++i) {
      roadFlow[static_cast<std::size_t>(i)] += static_cast<std::uint32_t>(m_traffic.roadTraffic[static_cast<std::size_t>(i)]);
    }
  }
  if (needGoods && static_cast<int>(m_goods.roadGoodsTraffic.size()) == n) {
    const double wgt = std::max(0.0, m_roadUpgradeGoodsWeight);
    for (int i = 0; i < n; ++i) {
      const double add = wgt * static_cast<double>(m_goods.roadGoodsTraffic[static_cast<std::size_t>(i)]);
      const std::uint32_t inc = static_cast<std::uint32_t>(std::clamp<long long>(
          static_cast<long long>(std::llround(add)), 0ll, static_cast<long long>(std::numeric_limits<std::uint32_t>::max())));
      roadFlow[static_cast<std::size_t>(i)] += inc;
    }
  }

  // Apply outside-connection pruning at the tile level (extra safety in case the derived models
  // didn't fully suppress disconnected subnets).
  if (roadToEdgeMask && static_cast<int>(roadToEdgeMask->size()) == n) {
    for (int i = 0; i < n; ++i) {
      if ((*roadToEdgeMask)[static_cast<std::size_t>(i)] == 0) {
        roadFlow[static_cast<std::size_t>(i)] = 0u;
      }
    }
  }

  RoadUpgradePlannerConfig cfg = m_roadUpgradeCfg;

  // Keep maxTargetLevel sane for gameplay (no point planning "upgrade to street").
  cfg.maxTargetLevel = std::clamp(cfg.maxTargetLevel, 2, 3);

  // Budget: either manual (cfg.budget) or dynamic based on current money.
  if (m_roadUpgradeBudgetAuto) {
    cfg.budget = std::max(0, m_world.stats().money);
  } else {
    cfg.budget = m_roadUpgradeBudget;
  }

  m_roadUpgradePlan = PlanRoadUpgrades(m_world, m_roadGraph, roadFlow, cfg);

  // Determine if the planned upgrades are already reflected in the current world.
  //
  // Note: World intentionally keeps its tile storage private; we use at(x,y) here instead of
  // exposing a raw tile array just for this comparison.
  m_roadUpgradePlanApplied = true;
  if (static_cast<int>(m_roadUpgradePlan.tileTargetLevel.size()) == n && w > 0 && h > 0) {
    for (int i = 0; i < n; ++i) {
      const int x = i % w;
      const int y = i / w;
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int target = static_cast<int>(m_roadUpgradePlan.tileTargetLevel[static_cast<std::size_t>(i)]);
      if (target > t.level) {
        m_roadUpgradePlanApplied = false;
        break;
      }
    }
  } else {
    m_roadUpgradePlanApplied = false;
  }
  m_roadUpgradePlanDirty = false;
  m_roadUpgradeSelectedMaskDirty = true;

  // Clamp selection to valid range.
  if (m_roadUpgradePlan.edges.empty()) {
    m_roadUpgradeSelectedEdge = -1;
  } else if (m_roadUpgradeSelectedEdge >= static_cast<int>(m_roadUpgradePlan.edges.size())) {
    m_roadUpgradeSelectedEdge = -1;
  }
}

void Game::ensureRoadUpgradeSelectedMaskUpToDate()
{
  if (!m_roadUpgradeSelectedMaskDirty) return;

  const int w = m_world.width();
  const int h = m_world.height();
  const int n = std::max(0, w) * std::max(0, h);

  m_roadUpgradeSelectedMask.assign(static_cast<std::size_t>(n), 0u);

  ensureRoadUpgradePlanUpToDate();
  if (m_roadUpgradeSelectedEdge < 0 || m_roadUpgradeSelectedEdge >= static_cast<int>(m_roadUpgradePlan.edges.size()) || n <= 0) {
    m_roadUpgradeSelectedMaskDirty = false;
    return;
  }

  const int gEdge = m_roadUpgradePlan.edges[static_cast<std::size_t>(m_roadUpgradeSelectedEdge)].edgeIndex;
  if (gEdge < 0 || gEdge >= static_cast<int>(m_roadGraph.edges.size())) {
    m_roadUpgradeSelectedMaskDirty = false;
    return;
  }

  const int planW = m_roadUpgradePlan.w;
  const int planH = m_roadUpgradePlan.h;
  if (planW != w || planH != h) {
    m_roadUpgradeSelectedMaskDirty = false;
    return;
  }
  if (static_cast<int>(m_roadUpgradePlan.tileTargetLevel.size()) != n) {
    m_roadUpgradeSelectedMaskDirty = false;
    return;
  }

  const RoadGraphEdge& e = m_roadGraph.edges[static_cast<std::size_t>(gEdge)];
  for (const Point& p : e.tiles) {
    if (!m_world.inBounds(p.x, p.y)) continue;
    const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w) +
                            static_cast<std::size_t>(p.x);
    if (idx >= m_roadUpgradeSelectedMask.size()) continue;
    if (m_roadUpgradePlan.tileTargetLevel[idx] > 0) {
      m_roadUpgradeSelectedMask[idx] = 1u;
    }
  }

  m_roadUpgradeSelectedMaskDirty = false;
}

void Game::drawRoadUpgradeOverlay()
{
  const bool show = m_showRoadUpgradePanel || m_showRoadUpgradeOverlay;
  if (!show) return;

  ensureRoadUpgradePlanUpToDate();
  if (m_roadUpgradePlan.tileTargetLevel.empty()) return;

  const int w = m_world.width();
  const int h = m_world.height();
  if (w <= 0 || h <= 0) return;
  if (m_roadUpgradePlan.w != w || m_roadUpgradePlan.h != h) return;
  const int n = w * h;
  if (static_cast<int>(m_roadUpgradePlan.tileTargetLevel.size()) != n) return;

  ensureRoadUpgradeSelectedMaskUpToDate();

  BeginMode2D(m_camera);

  const float zoom = std::max(0.25f, m_camera.zoom);
  const float outline = 1.25f / zoom;

  const bool haveSelectionMask = (m_roadUpgradeSelectedEdge >= 0) &&
                                 (static_cast<int>(m_roadUpgradeSelectedMask.size()) == n);

  Vector2 corners[4];

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      const int lvl = static_cast<int>(m_roadUpgradePlan.tileTargetLevel[idx]);
      if (lvl <= 0) continue;

      const bool isSelectedTile = haveSelectionMask && (m_roadUpgradeSelectedMask[idx] != 0u);
      if (m_roadUpgradeShowOnlySelectedEdge && haveSelectionMask && !isSelectedTile) continue;

      const unsigned char alpha = isSelectedTile ? 170u : (haveSelectionMask ? 70u : 130u);
      const Color fill = RoadUpgradeLevelColor(lvl, alpha);

      const Vector2 c = TileToWorldCenterElevated(m_world, x, y, static_cast<float>(m_cfg.tileWidth),
                                                  static_cast<float>(m_cfg.tileHeight), m_elev);
      TileDiamondCorners(c, static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), corners);

      // Filled diamond (two triangles).
      DrawTriangle(corners[0], corners[1], corners[3], fill);
      DrawTriangle(corners[1], corners[2], corners[3], fill);

      if (isSelectedTile) {
        const Color oc = Color{255, 255, 255, 200};
        DrawLineEx(corners[0], corners[1], outline, oc);
        DrawLineEx(corners[1], corners[2], outline, oc);
        DrawLineEx(corners[2], corners[3], outline, oc);
        DrawLineEx(corners[3], corners[0], outline, oc);
      }
    }
  }

  EndMode2D();
}

void Game::drawRoadUpgradePanel(int x0, int y0)
{
  if (!m_showRoadUpgradePanel) return;

  ensureRoadUpgradePlanUpToDate();
  ensureRoadUpgradeSelectedMaskUpToDate();

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  const int panelW = kRoadUpgradePanelW;
  const int panelH = kRoadUpgradePanelH;

  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW), static_cast<float>(panelH)};
  ui::DrawPanel(panelR, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panelR, "Road Upgrade Planner", uiTime, /*active=*/true, /*titleSizePx=*/22);

  auto markDirty = [&]() {
    m_roadUpgradePlanDirty = true;
    m_roadUpgradeSelectedMaskDirty = true;
  };

  int x = x0 + 12;
  int y = y0 + 42;

  ui::Text(x, y, 15, "Tab: select  |  [ / ]: adjust  |  Enter: plan", uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
  y += 18;
  ui::Text(x, y, 15, "Ctrl+U: toggle panel  |  Ctrl+Shift+U: export  |  Shift+Enter: apply plan", uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);

  // Mouse-friendly actions.
  {
    const Rectangle br1{static_cast<float>(x0 + panelW - 244), static_cast<float>(y0 + 40), 70.0f, 18.0f};
    const Rectangle br2{static_cast<float>(x0 + panelW - 170), static_cast<float>(y0 + 40), 70.0f, 18.0f};
    const Rectangle br3{static_cast<float>(x0 + panelW - 96), static_cast<float>(y0 + 40), 84.0f, 18.0f};

    if (ui::Button(7001, br1, "Plan", mouseUi, uiTime, /*enabled=*/true, /*primary=*/true)) {
      m_roadUpgradePlanDirty = true;
      ensureRoadUpgradePlanUpToDate();
      ensureRoadUpgradeSelectedMaskUpToDate();
      showToast("Road upgrades: replanned", 2.0f);
    }
    if (ui::Button(7002, br2, "Apply", mouseUi, uiTime,
                   /*enabled=*/(!m_roadUpgradePlanApplied && m_roadUpgradePlan.totalCost > 0),
                   /*primary=*/false)) {
      applyRoadUpgradePlan();
      showToast("Road upgrades: applied", 2.0f);
    }
    if (ui::Button(7003, br3, "Export", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      exportRoadUpgradeArtifacts();
    }
  }

  y += 22;

  const int rowH = 20;
  const int rows = 13;
  auto rowRectFor = [&](int yRow) -> Rectangle {
    return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                     static_cast<float>(rowH)};
  };

  const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                        static_cast<float>(rows * rowH + 8)};
  ui::DrawPanelInset(listR, uiTime, /*active=*/true);

  int rowY = y + 4;

  const int xValueRight = x0 + panelW - 14;
  const int sliderH = 11;
  const int sliderW = 150;

  const Stats& st = m_world.stats();

  auto selectRowOnClick = [&](int idx, Rectangle rr) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
      m_roadUpgradeSelection = idx;
    }
  };

  auto drawToggleRow = [&](int idx, std::string_view label, bool& v, bool affectsPlan) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selected = (m_roadUpgradeSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
    if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_roadUpgradeSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    bool vv = v;
    if (ui::Toggle(7100 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
      v = vv;
      if (affectsPlan) markDirty();
    }

    rowY += rowH;
  };

  auto drawIntSliderRow = [&](int idx, std::string_view label, int& v, int vMin, int vMax, int step,
                              bool affectsPlan, const char* fmt = "%d", bool enabled = true) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selected = (m_roadUpgradeSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, v);

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    if (enabled && CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_roadUpgradeSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    int vv = v;
    if (ui::SliderInt(7200 + idx, sr, vv, vMin, vMax, step, mouseUi, uiTime, /*enabled=*/enabled)) {
      v = std::clamp(vv, vMin, vMax);
      if (affectsPlan) markDirty();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  };

  auto drawFloatSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax,
                                bool affectsPlan, const char* fmt, bool enabled = true) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selected = (m_roadUpgradeSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    char buf[64];
    std::snprintf(buf, sizeof(buf), fmt, static_cast<double>(v));

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    if (enabled && CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_roadUpgradeSelection = idx;
    }

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    float vv = v;
    if (ui::SliderFloat(7300 + idx, sr, vv, vMin, vMax, mouseUi, uiTime, /*enabled=*/enabled)) {
      v = std::clamp(vv, vMin, vMax);
      if (affectsPlan) markDirty();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  };

  auto drawCycleRow = [&](int idx, std::string_view label, std::string_view value, auto onClick) {
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selected = (m_roadUpgradeSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, label, c, /*bold=*/false, /*shadow=*/true, 1);

    const int bw = std::max(110, ui::MeasureTextWidth(value, 15, /*bold=*/true, 1) + 18);
    const Rectangle br{static_cast<float>(xValueRight - bw),
                       static_cast<float>(rowY + (rowH - 18) / 2),
                       static_cast<float>(bw),
                       18.0f};
    if (ui::Button(7400 + idx, br, value, mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      onClick();
      markDirty();
    }

    rowY += rowH;
  };

  // 0 overlay toggle
  drawToggleRow(0, "Overlay", m_showRoadUpgradeOverlay, /*affectsPlan=*/false);

  // 1 demand mode
  drawCycleRow(1, "Demand", TransitDemandModeName(m_roadUpgradeDemandMode), [&]() {
    const int idx = static_cast<int>(m_roadUpgradeDemandMode);
    const int next = (idx + 1 + 3) % 3;
    m_roadUpgradeDemandMode = static_cast<TransitDemandMode>(next);
    showToast(std::string("Road upgrade demand: ") + TransitDemandModeName(m_roadUpgradeDemandMode), 2.0f);
  });

  // 2 objective
  drawCycleRow(2, "Objective", RoadUpgradeObjectiveName(m_roadUpgradeCfg.objective), [&]() {
    const int idx = static_cast<int>(m_roadUpgradeCfg.objective);
    const int next = (idx + 1 + 3) % 3;
    m_roadUpgradeCfg.objective = static_cast<RoadUpgradeObjective>(next);
    showToast(std::string("Road upgrade objective: ") + RoadUpgradeObjectiveName(m_roadUpgradeCfg.objective), 2.0f);
  });

  // 3 budget row: auto toggle + slider.
  {
    const int idx = 3;
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selected = (m_roadUpgradeSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, "Budget", c, /*bold=*/false, /*shadow=*/true, 1);

    // Auto toggle.
    const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
    bool autoV = m_roadUpgradeBudgetAuto;
    if (ui::Toggle(7100 + idx, tr, autoV, mouseUi, uiTime, /*enabled=*/true)) {
      m_roadUpgradeBudgetAuto = autoV;
      markDirty();
      showToast(m_roadUpgradeBudgetAuto ? "Upgrade budget: AUTO" : "Upgrade budget: manual", 2.0f);
    }

    const bool manual = !m_roadUpgradeBudgetAuto;
    const int maxBudget = std::max(0, std::max(st.money, 20000));
    int budgetV = std::clamp(m_roadUpgradeBudget, -1, maxBudget);

    // Slider (manual only).
    char buf[64];
    const int shownBudget = manual ? budgetV : std::max(0, st.money);
    const bool unlimited = manual && (budgetV < 0);
    std::snprintf(buf, sizeof(buf), "%s", manual ? (unlimited ? "UNLIM" : TextFormat("$%d", shownBudget)) : "AUTO");

    const int valW = ui::MeasureTextWidth(buf, 15, /*bold=*/false, 1);
    const int valX = xValueRight - 46 - 8 - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};
    int vv = budgetV;
    if (ui::SliderInt(7200 + idx, sr, vv, -1, maxBudget, 1, mouseUi, uiTime, /*enabled=*/manual)) {
      m_roadUpgradeBudget = std::clamp(vv, -1, maxBudget);
      markDirty();
    }

    ui::Text(valX, rowY, 15, buf, c, /*bold=*/false, /*shadow=*/true, 1);

    rowY += rowH;
  }

  // 4 max target
  drawIntSliderRow(4, "Max level", m_roadUpgradeCfg.maxTargetLevel, 2, 3, 1, /*affectsPlan=*/true);

  // 5 min util
  {
    float v = static_cast<float>(m_roadUpgradeCfg.minUtilConsider);
    drawFloatSliderRow(5, "Min util", v, 0.0f, 5.0f, /*affectsPlan=*/true, "%.2f");
    m_roadUpgradeCfg.minUtilConsider = static_cast<double>(v);
  }

  // 6 endpoints
  drawToggleRow(6, "Endpoints", m_roadUpgradeCfg.upgradeEndpoints, /*affectsPlan=*/true);

  // 7 base cap
  drawIntSliderRow(7, "Base cap", m_roadUpgradeCfg.baseTileCapacity, 4, 200, 2, /*affectsPlan=*/true);

  // 8..10 hybrid weights (floats)
  {
    float v = static_cast<float>(m_roadUpgradeCfg.hybridExcessWeight);
    drawFloatSliderRow(8, "Excess w", v, 0.0f, 10.0f, /*affectsPlan=*/true, "%.2f");
    m_roadUpgradeCfg.hybridExcessWeight = static_cast<double>(v);
  }
  {
    float v = static_cast<float>(m_roadUpgradeCfg.hybridTimeWeight);
    drawFloatSliderRow(9, "Time w", v, 0.0f, 10.0f, /*affectsPlan=*/true, "%.2f");
    m_roadUpgradeCfg.hybridTimeWeight = static_cast<double>(v);
  }
  {
    float v = static_cast<float>(m_roadUpgradeGoodsWeight);
    drawFloatSliderRow(10, "Goods w", v, 0.0f, 10.0f, /*affectsPlan=*/true, "%.2f");
    m_roadUpgradeGoodsWeight = static_cast<double>(v);
  }

  // 11 selected edge (per-plan)
  {
    const int idx = 11;
    const Rectangle rr = rowRectFor(rowY);
    selectRowOnClick(idx, rr);
    const bool selected = (m_roadUpgradeSelection == idx);
    if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

    const Color c = selected ? uiTh.text : uiTh.textDim;
    ui::Text(x, rowY, 15, "Edge", c, /*bold=*/false, /*shadow=*/true, 1);

    const int count = static_cast<int>(m_roadUpgradePlan.edges.size());
    const int vMax = std::max(-1, count - 1);
    int v = m_roadUpgradeSelectedEdge;
    if (v > vMax) v = -1;

    const char* valueText = (count <= 0) ? "None" : ((v < 0) ? "All" : TextFormat("%d / %d", v + 1, count));
    const int valW = ui::MeasureTextWidth(valueText, 15, /*bold=*/false, 1);
    const int valX = xValueRight - valW;

    const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                       static_cast<float>(rowY + (rowH - sliderH) / 2),
                       static_cast<float>(sliderW),
                       static_cast<float>(sliderH)};

    int vv = v;
    if (count > 0 && ui::SliderInt(7200 + idx, sr, vv, -1, vMax, 1, mouseUi, uiTime, /*enabled=*/true)) {
      if (vv < -1) vv = -1;
      if (vv > vMax) vv = -1;
      m_roadUpgradeSelectedEdge = vv;
      m_roadUpgradeSelectedMaskDirty = true;
    }

    ui::Text(valX, rowY, 15, valueText, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  }

  // 12 solo
  drawToggleRow(12, "Solo", m_roadUpgradeShowOnlySelectedEdge, /*affectsPlan=*/false);

  // Footer / plan stats.
  y = rowY + 4;
  DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
  y += 10;

  const RoadUpgradePlan& p = m_roadUpgradePlan;

  // Selection mask stats.
  int selectedEdges = 0;
  for (std::uint8_t b : m_roadUpgradeSelectedMask) selectedEdges += (b != 0);

  // Approx tile-upgrade count (target level > current road level).
  int plannedTiles = 0;
  if (static_cast<int>(p.tileTargetLevel.size()) == m_world.width() * m_world.height()) {
    const int ww = m_world.width();
    const int hh = m_world.height();
    for (int yy = 0; yy < hh; ++yy) {
      for (int xx = 0; xx < ww; ++xx) {
        const int idx = yy * ww + xx;
        const Tile& t = m_world.at(xx, yy);
        if (t.overlay == Overlay::Road && idx >= 0 && idx < static_cast<int>(p.tileTargetLevel.size())) {
          if (p.tileTargetLevel[static_cast<std::size_t>(idx)] > static_cast<std::uint8_t>(t.level)) {
            ++plannedTiles;
          }
        }
      }
    }
  }

  ui::Text(x, y, 15,
           TextFormat("Edges: %d (sel %d)  Planned tiles: %d  Cost: $%d",
                      static_cast<int>(p.edges.size()),
                      selectedEdges,
                      plannedTiles,
                      p.totalCost),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 16;

  ui::Text(x, y, 14,
           TextFormat("Excess reduced: %.0f   Time saved: %.0f   Runtime: %.3fs",
                      p.totalExcessReduced,
                      p.totalTimeSaved,
                      p.runtimeSec),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 16;

  ui::Text(x, y, 14,
           TextFormat("Applied: %s  Money: $%d  Budget: %s",
                      m_roadUpgradePlanApplied ? "YES" : "no",
                      st.money,
                      m_roadUpgradeBudgetAuto ? "AUTO" : (m_roadUpgradeBudget < 0 ? "UNLIM" : TextFormat("$%d", m_roadUpgradeBudget))),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
}

void Game::adjustRoadUpgradePanel(int dir, bool bigStep)
{
  if (dir == 0) return;
  ensureRoadUpgradePlanUpToDate();

  bool planChanged = false;

  const int stepMoney = bigStep ? 500 : 100;
  const int stepCap = bigStep ? 8 : 2;
  const double stepF = bigStep ? 0.25 : 0.05;

  switch (m_roadUpgradeSelection) {
    case 0: {
      m_showRoadUpgradeOverlay = !m_showRoadUpgradeOverlay;
      showToast(m_showRoadUpgradeOverlay ? "Road upgrades overlay: ON" : "Road upgrades overlay: OFF", 2.0f);
    } break;

    case 1: {
      int m = static_cast<int>(m_roadUpgradeDemandMode);
      m += (dir > 0) ? 1 : -1;
      if (m < 0) m = 2;
      if (m > 2) m = 0;
      m_roadUpgradeDemandMode = static_cast<TransitDemandMode>(m);
      showToast(std::string("Upgrade demand: ") + TransitDemandModeName(m_roadUpgradeDemandMode), 2.0f);
      planChanged = true;
    } break;

    case 2: {
      int o = static_cast<int>(m_roadUpgradeCfg.objective);
      o += (dir > 0) ? 1 : -1;
      if (o < 0) o = 2;
      if (o > 2) o = 0;
      m_roadUpgradeCfg.objective = static_cast<RoadUpgradeObjective>(o);
      showToast(std::string("Upgrade objective: ") + RoadUpgradeObjectiveLabel(m_roadUpgradeCfg.objective), 2.0f);
      planChanged = true;
    } break;

    case 3: {
      if (bigStep) {
        m_roadUpgradeBudgetAuto = !m_roadUpgradeBudgetAuto;
        if (!m_roadUpgradeBudgetAuto && m_roadUpgradeBudget < 0) {
          m_roadUpgradeBudget = std::max(0, m_world.stats().money);
        }
        showToast(m_roadUpgradeBudgetAuto ? "Upgrade budget: AUTO" : "Upgrade budget: MANUAL", 2.0f);
        planChanged = true;
      } else {
        if (!m_roadUpgradeBudgetAuto) {
          // If budget currently unlimited, snap to current money first so adjustments are intuitive.
          if (m_roadUpgradeBudget < 0) {
            m_roadUpgradeBudget = std::max(0, m_world.stats().money);
          }
          m_roadUpgradeBudget += dir * stepMoney;
          if (m_roadUpgradeBudget < 0) m_roadUpgradeBudget = -1; // allow going back to unlimited
          planChanged = true;
        }
      }
    } break;

    case 4: {
      m_roadUpgradeCfg.maxTargetLevel = std::clamp(m_roadUpgradeCfg.maxTargetLevel + dir, 2, 3);
      planChanged = true;
    } break;

    case 5: {
      m_roadUpgradeCfg.minUtilConsider = std::clamp(m_roadUpgradeCfg.minUtilConsider + static_cast<double>(dir) * stepF, 0.0, 5.0);
      planChanged = true;
    } break;

    case 6: {
      m_roadUpgradeCfg.upgradeEndpoints = !m_roadUpgradeCfg.upgradeEndpoints;
      planChanged = true;
    } break;

    case 7: {
      m_roadUpgradeCfg.baseTileCapacity = std::clamp(m_roadUpgradeCfg.baseTileCapacity + dir * stepCap, 4, 200);
      planChanged = true;
    } break;

    case 8: {
      m_roadUpgradeCfg.hybridExcessWeight = std::clamp(m_roadUpgradeCfg.hybridExcessWeight + static_cast<double>(dir) * stepF, 0.0, 10.0);
      planChanged = true;
    } break;

    case 9: {
      m_roadUpgradeCfg.hybridTimeWeight = std::clamp(m_roadUpgradeCfg.hybridTimeWeight + static_cast<double>(dir) * stepF, 0.0, 10.0);
      planChanged = true;
    } break;

    case 10: {
      m_roadUpgradeGoodsWeight = std::clamp(m_roadUpgradeGoodsWeight + static_cast<double>(dir) * stepF, 0.0, 10.0);
      planChanged = true;
    } break;

    case 11: {
      const int count = static_cast<int>(m_roadUpgradePlan.edges.size());
      if (count <= 0) {
        m_roadUpgradeSelectedEdge = -1;
      } else {
        int cur = m_roadUpgradeSelectedEdge;
        if (cur < 0) {
          cur = (dir > 0) ? 0 : (count - 1);
        } else {
          cur += (dir > 0) ? 1 : -1;
          if (cur >= count) cur = -1;
          if (cur < -1) cur = count - 1;
        }
        m_roadUpgradeSelectedEdge = cur;
      }
      m_roadUpgradeSelectedMaskDirty = true;
    } break;

    case 12: {
      m_roadUpgradeShowOnlySelectedEdge = !m_roadUpgradeShowOnlySelectedEdge;
    } break;

    default:
      break;
  }

  if (planChanged) {
    m_roadUpgradePlanDirty = true;
    m_roadUpgradeSelectedMaskDirty = true;
  }
}

bool Game::applyRoadUpgradePlan()
{
  ensureRoadUpgradePlanUpToDate();
  if (m_roadUpgradePlan.tileTargetLevel.empty() || m_roadUpgradePlan.w <= 0 || m_roadUpgradePlan.h <= 0) {
    showToast("No upgrade plan", 2.0f);
    return false;
  }

  const int w = m_world.width();
  const int h = m_world.height();
  if (w != m_roadUpgradePlan.w || h != m_roadUpgradePlan.h) {
    showToast("Upgrade plan mismatch (replan)", 2.5f);
    return false;
  }

  const int n = w * h;
  if (static_cast<int>(m_roadUpgradePlan.tileTargetLevel.size()) != n) {
    showToast("Upgrade plan invalid (replan)", 2.5f);
    return false;
  }

  // Compute the current money cost (world may have changed since planning).
  int moneyCost = 0;
  int upgradeTiles = 0;
  bool anyChange = false;

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      const int tgt = ClampRoadLevel(static_cast<int>(m_roadUpgradePlan.tileTargetLevel[idx]));
      if (tgt <= 0) continue;
      const Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int cur = ClampRoadLevel(static_cast<int>(t.level));
      if (tgt <= cur) continue;
      const bool isBridge = (t.terrain == Terrain::Water);
      const int c = RoadPlacementCost(cur, tgt, /*alreadyRoad=*/true, isBridge);
      moneyCost += c;
      if (c > 0) {
        anyChange = true;
        ++upgradeTiles;
      }
    }
  }

  if (!anyChange) {
    showToast("Upgrade plan already applied (no changes)", 2.5f);
    return false;
  }

  if (moneyCost > m_world.stats().money) {
    showToast(TextFormat("Insufficient funds (%d needed)", moneyCost), 3.0f);
    return false;
  }

  endPaintStroke();

  const int moneyBefore = m_world.stats().money;
  m_history.beginStroke(m_world);

  std::vector<Point> changed;
  changed.reserve(static_cast<std::size_t>(upgradeTiles));

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                              static_cast<std::size_t>(x);
      const int tgt = ClampRoadLevel(static_cast<int>(m_roadUpgradePlan.tileTargetLevel[idx]));
      if (tgt <= 0) continue;
      Tile& t = m_world.at(x, y);
      if (t.overlay != Overlay::Road) continue;
      const int cur = ClampRoadLevel(static_cast<int>(t.level));
      if (tgt <= cur) continue;

      m_history.noteTilePreEdit(m_world, x, y);
      t.level = static_cast<std::uint8_t>(tgt);
      changed.push_back(Point{x, y});
    }
  }

  // Spend money once, atomically.
  m_world.stats().money = moneyBefore - moneyCost;

  m_history.endStroke(m_world);

  // Invalidate render caches and derived stats.
  m_sim.refreshDerivedStats(m_world);
  m_renderer.markMinimapDirty();
  m_renderer.markBaseCacheDirtyForTiles(changed, m_world.width(), m_world.height());

  m_trafficDirty = true;
  m_goodsDirty = true;
  m_landValueDirty = true;
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;
  m_vehiclesDirty = true;

  // Our plan is now stale.
  m_roadUpgradePlanDirty = true;
  m_roadUpgradeSelectedMaskDirty = true;

  // Resilience suggestions depend on road class (bridge travel-time/maintenance), so invalidate.
  m_resilienceDirty = true;
  m_resilienceBypassesDirty = true;
  m_resilienceBypasses.clear();

  const int spent = moneyBefore - m_world.stats().money;
  showToast(TextFormat("Road upgrades applied: %d tiles, spent %d", static_cast<int>(changed.size()), spent));
  m_roadUpgradePlanApplied = true;
  return true;
}

void Game::exportRoadUpgradeArtifacts()
{
  ensureRoadUpgradePlanUpToDate();
  if (m_roadUpgradePlan.edges.empty()) {
    showToast("Road upgrade export: no plan", 2.5f);
    return;
  }

  namespace fs = std::filesystem;
  fs::create_directories("captures");

  const std::string base = TextFormat("captures/roadup_seed%llu_%s",
                                      static_cast<unsigned long long>(m_world.seed()),
                                      FileTimestamp().c_str());

  RoadUpgradePlanExportConfig ec;
  ec.includeEdgeTiles = true;
  ec.includeTileUpgrades = true;

  // GeoJSON + JSON
  {
    std::string err;
    const std::string path = base + ".geojson";
    const bool ok = ExportRoadUpgradePlanGeoJson(path, m_roadGraph, m_roadUpgradePlan, ec, &err);
    showToast(ok ? (std::string("Road upgrades export: ") + path)
                 : (std::string("Road upgrades export failed: ") + err),
              3.0f);
  }
  {
    std::string err;
    const std::string path = base + ".json";
    const bool ok = ExportRoadUpgradePlanJson(path, m_roadGraph, m_roadUpgradePlan, ec, &err);
    if (!ok) {
      showToast(std::string("Road upgrades JSON failed: ") + err, 3.0f);
    }
  }

  // Quick-look raster overlay (tile-space, scaled).
  {
    std::string err;
    PpmImage img = RenderRoadUpgradeOverlayTile(m_world, ExportLayer::Overlay, m_roadUpgradePlan);
    const int maxDim = std::max(1, std::max(m_world.width(), m_world.height()));
    const int scale = std::clamp(2048 / maxDim, 1, 8);
    if (scale > 1) {
      img = ScaleNearest(img, scale);
    }

    const std::string path = base + ".png";
    const bool ok = WriteImageAuto(path, img, err);
    if (!ok) {
      showToast(std::string("Road upgrades PNG failed: ") + err, 3.0f);
    }
  }
}

void Game::adjustResiliencePanel(int dir, bool bigStep)
{
  if (dir == 0) return;

  auto clamp01 = [](float v) -> float { return std::clamp(v, 0.0f, 1.0f); };
  auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };
  auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

  auto heatmapLabel = [](HeatmapOverlay h) -> const char* {
    switch (h) {
    case HeatmapOverlay::Off: return "Off";
    case HeatmapOverlay::FloodDepth: return "Flood depth";
    case HeatmapOverlay::PondingDepth: return "Ponding depth";
    case HeatmapOverlay::EvacuationTime: return "Evac time";
    case HeatmapOverlay::EvacuationUnreachable: return "Evac unreachable";
    case HeatmapOverlay::EvacuationFlow: return "Evac flow";
    default: return "Heatmap";
    }
  };

  const float seaStep = bigStep ? 0.05f : 0.01f;
  const float pondMinDepthStep = bigStep ? 0.02f : 0.005f;
  const float epsStep = bigStep ? 1.0e-5f : 1.0e-6f;
  const int walkStep = bigStep ? 1000 : 250;
  const int capStep = bigStep ? 5 : 1;

  switch (m_resilienceSelection) {
  case 0: { // heatmap overlay subset (resilience-related)
    static constexpr HeatmapOverlay kOverlays[] = {
        HeatmapOverlay::Off,
        HeatmapOverlay::FloodDepth,
        HeatmapOverlay::PondingDepth,
        HeatmapOverlay::EvacuationTime,
        HeatmapOverlay::EvacuationUnreachable,
        HeatmapOverlay::EvacuationFlow,
    };
    constexpr int count = static_cast<int>(sizeof(kOverlays) / sizeof(kOverlays[0]));
    int idx = 0;
    for (int i = 0; i < count; ++i) {
      if (kOverlays[i] == m_heatmapOverlay) {
        idx = i;
        break;
      }
    }
    idx = (idx + dir + count) % count;
    m_heatmapOverlay = kOverlays[idx];

    m_landValueDirty = true;
    invalidateHydrology();
    showToast(TextFormat("Heatmap: %s", heatmapLabel(m_heatmapOverlay)));
    break;
  }
  case 1: { // hazard mode
    static constexpr EvacuationHazardMode kModes[] = {
        EvacuationHazardMode::None,
        EvacuationHazardMode::Sea,
        EvacuationHazardMode::Ponding,
        EvacuationHazardMode::Both,
    };
    constexpr int count = static_cast<int>(sizeof(kModes) / sizeof(kModes[0]));
    int idx = 0;
    for (int i = 0; i < count; ++i) {
      if (kModes[i] == m_evacCfg.hazardMode) {
        idx = i;
        break;
      }
    }
    idx = (idx + dir + count) % count;
    m_evacCfg.hazardMode = kModes[idx];
    m_evacDirty = true;
    showToast(TextFormat("Evac hazards: %s", EvacuationHazardModeName(m_evacCfg.hazardMode)));
    break;
  }
  case 2: // bridges passable
    m_evacCfg.bridgesPassable = !m_evacCfg.bridgesPassable;
    m_evacDirty = true;
    showToast(m_evacCfg.bridgesPassable ? "Evac bridges: passable" : "Evac bridges: blocked");
    break;
  case 3: // sea level
    m_seaLevel = clamp01(m_seaLevel + static_cast<float>(dir) * seaStep);
    invalidateHydrology();
    showToast(TextFormat("Sea level: %.2f", static_cast<double>(m_seaLevel)));
    break;
  case 4: // sea edge connectivity
    m_seaFloodCfg.requireEdgeConnection = !m_seaFloodCfg.requireEdgeConnection;
    invalidateHydrology();
    showToast(m_seaFloodCfg.requireEdgeConnection ? "Flood connectivity: edge-connected" : "Flood connectivity: any low cell");
    break;
  case 5: // sea 4/8-connected
    m_seaFloodCfg.eightConnected = !m_seaFloodCfg.eightConnected;
    invalidateHydrology();
    showToast(m_seaFloodCfg.eightConnected ? "Flood connectivity: 8-neighbor" : "Flood connectivity: 4-neighbor");
    break;
  case 6: // pond include edges
    m_pondingCfg.includeEdges = !m_pondingCfg.includeEdges;
    invalidateHydrology();
    showToast(m_pondingCfg.includeEdges ? "Ponding: edges drain" : "Ponding: sealed edges");
    break;
  case 7: // pond epsilon
    m_pondingCfg.epsilon = clampF(m_pondingCfg.epsilon + static_cast<float>(dir) * epsStep, 0.0f, 0.1f);
    invalidateHydrology();
    showToast(TextFormat("Pond epsilon: %.2g", static_cast<double>(m_pondingCfg.epsilon)));
    break;
  case 8: // pond min depth (for evac hazard mask)
    m_evacCfg.pondMinDepth = clamp01(m_evacCfg.pondMinDepth + static_cast<float>(dir) * pondMinDepthStep);
    m_evacDirty = true;
    showToast(TextFormat("Evac pond min depth: %.3f", static_cast<double>(m_evacCfg.pondMinDepth)));
    break;
  case 9: // evac weighting
    m_evacCfg.evac.useTravelTime = !m_evacCfg.evac.useTravelTime;
    m_evacDirty = true;
    showToast(m_evacCfg.evac.useTravelTime ? "Evac weighting: time" : "Evac weighting: steps");
    break;
  case 10: // walk cost
    m_evacCfg.evac.walkCostMilli = clampI(m_evacCfg.evac.walkCostMilli + dir * walkStep, 0, 100000);
    m_evacDirty = true;
    showToast(TextFormat("Evac walk cost: %d", m_evacCfg.evac.walkCostMilli));
    break;
  case 11: // road tile capacity
    m_evacCfg.evac.roadTileCapacity = clampI(m_evacCfg.evac.roadTileCapacity + dir * capStep, 1, 100000);
    m_evacDirty = true;
    showToast(TextFormat("Evac road cap: %d", m_evacCfg.evac.roadTileCapacity));
    break;
  case 12: { // run/refresh
    m_evacDirty = true;
    ensureEvacuationScenarioUpToDate();
    const EvacuationResult& r = m_evacScenario.evac;
    const double reachPct = (r.population > 0)
                                ? (100.0 * static_cast<double>(r.reachablePopulation) / static_cast<double>(r.population))
                                : 0.0;
    const char* unit = m_evacCfg.evac.useTravelTime ? "time" : "steps";
    showToast(TextFormat("Evac run (%s): reach %.0f%%  p95 %.1f %s",
                         EvacuationHazardModeName(m_evacCfg.hazardMode),
                         reachPct, static_cast<double>(r.p95EvacTime), unit),
              3.0f);
    break;
  }
  case 13: // export
    exportEvacuationArtifacts();
    break;
  default: break;
  }
}

void Game::ensureEvacuationScenarioUpToDate()
{
  const int w = m_world.width();
  const int h = m_world.height();
  if (w <= 0 || h <= 0) {
    m_evacScenario = EvacuationScenarioResult{};
    m_evacHeatmaps = EvacuationScenarioHeatmaps{};
    m_evacDirty = false;
    return;
  }

  // Keep hazard settings synced with the flood/ponding overlay configs.
  m_evacCfg.seaLevel = m_seaLevel;
  m_evacCfg.seaCfg = m_seaFloodCfg;
  m_evacCfg.pondCfg = m_pondingCfg;
  m_evacCfg.pondMinDepth = std::clamp(m_evacCfg.pondMinDepth, 0.0f, 1.0f);

  if (!m_evacDirty && (m_evacScenario.w == w) && (m_evacScenario.h == h)) return;

  m_evacScenario = ComputeEvacuationScenario(m_world, m_evacCfg);
  m_evacHeatmaps = BuildEvacuationScenarioHeatmaps(m_world, m_evacScenario);
  m_evacDirty = false;
}

void Game::exportEvacuationArtifacts()
{
  ensureEvacuationScenarioUpToDate();

  if (m_evacScenario.w <= 0 || m_evacScenario.h <= 0) {
    showToast("Evac export: nothing to export", 2.0f);
    return;
  }

  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories("captures", ec);

  const std::string base = TextFormat("captures/evac_seed%llu_day%d_%s",
                                      static_cast<unsigned long long>(m_world.seed()),
                                      m_world.stats().day,
                                      FileTimestamp().c_str());

  // Write JSON summary.
  {
    const std::string path = base + ".json";
    std::ofstream out(path, std::ios::binary);
    if (!out) {
      showToast("Evac export failed: can't write json", 3.0f);
    } else {
      const EvacuationScenarioConfig& cfg = m_evacCfg;
      const EvacuationResult& r = m_evacScenario.evac;

      auto b = [](bool v) -> const char* { return v ? "true" : "false"; };

      out << "{\n";
      out << "  \"seed\": " << static_cast<unsigned long long>(m_world.seed()) << ",\n";
      out << "  \"day\": " << m_world.stats().day << ",\n";
      out << "  \"hazardMode\": \"" << EvacuationHazardModeName(cfg.hazardMode) << "\",\n";
      out << "  \"bridgesPassable\": " << b(cfg.bridgesPassable) << ",\n";
      out << "  \"pondMinDepth\": " << cfg.pondMinDepth << ",\n";
      out << "  \"seaLevel\": " << cfg.seaLevel << ",\n";
      out << "  \"seaRequireEdge\": " << b(cfg.seaCfg.requireEdgeConnection) << ",\n";
      out << "  \"seaEightConnected\": " << b(cfg.seaCfg.eightConnected) << ",\n";
      out << "  \"pondEpsilon\": " << cfg.pondCfg.epsilon << ",\n";
      out << "  \"pondIncludeEdges\": " << b(cfg.pondCfg.includeEdges) << ",\n";
      out << "  \"evacConfig\": {\n";
      out << "    \"useTravelTime\": " << b(cfg.evac.useTravelTime) << ",\n";
      out << "    \"walkCostMilli\": " << cfg.evac.walkCostMilli << ",\n";
      out << "    \"roadTileCapacity\": " << cfg.evac.roadTileCapacity << "\n";
      out << "  },\n";
      out << "  \"hazards\": {\n";
      out << "    \"seaFloodedCells\": " << m_evacScenario.sea.floodedCells << ",\n";
      out << "    \"seaMaxDepth\": " << m_evacScenario.sea.maxDepth << ",\n";
      out << "    \"pondFilledCells\": " << m_evacScenario.pond.filledCells << ",\n";
      out << "    \"pondMaxDepth\": " << m_evacScenario.pond.maxDepth << ",\n";
      out << "    \"pondVolume\": " << m_evacScenario.pond.volume << "\n";
      out << "  },\n";
      out << "  \"evacSummary\": {\n";
      out << "    \"exitSources\": " << r.exitSources << ",\n";
      out << "    \"residentialTiles\": " << r.residentialTiles << ",\n";
      out << "    \"totalResidentialPopulation\": " << r.population << ",\n";
      out << "    \"reachableResidentialPopulation\": " << r.reachablePopulation << ",\n";
      out << "    \"floodedResidentialPopulation\": " << r.floodedPopulation << ",\n";
      out << "    \"unreachableResidentialPopulation\": " << r.unreachablePopulation << ",\n";
      out << "    \"avgEvacTime\": " << r.avgEvacTime << ",\n";
      out << "    \"p95EvacTime\": " << r.p95EvacTime << ",\n";
      out << "    \"maxEvacRoadFlow\": " << r.maxEvacRoadFlow << "\n";
      out << "  }\n";
      out << "}\n";
    }
  }

  // Helper: overlay a per-tile value onto a base layer.
  auto blendOver = [](std::uint8_t base, std::uint8_t over, std::uint8_t a) -> std::uint8_t {
    const unsigned int ib = base;
    const unsigned int io = over;
    const unsigned int ia = a;
    return static_cast<std::uint8_t>((ib * (255u - ia) + io * ia) / 255u);
  };

  auto overlayPixel = [&](PpmImage& img, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b,
                          std::uint8_t a) {
    const int w = img.width;
    const std::size_t idx = static_cast<std::size_t>(y * w + x) * 3u;
    if (idx + 2u >= img.rgb.size()) return;
    img.rgb[idx + 0u] = blendOver(img.rgb[idx + 0u], r, a);
    img.rgb[idx + 1u] = blendOver(img.rgb[idx + 1u], g, a);
    img.rgb[idx + 2u] = blendOver(img.rgb[idx + 2u], b, a);
  };

  const int w = m_evacScenario.w;
  const int h = m_evacScenario.h;
  const int maxDim = std::max(w, h);
  const int scale = std::clamp(2048 / std::max(1, maxDim), 1, 8);

  auto write = [&](const std::string& suffix, const PpmImage& imgIn) {
    std::string err;
    PpmImage outImg = imgIn;
    if (scale > 1) outImg = ScaleNearest(outImg, scale);
    const std::string path = base + suffix;
    if (!WriteImageAuto(path, outImg, err)) {
      showToast(std::string("Export failed: ") + err, 4.0f);
    }
  };

  // Hazard mask overlay (blue)
  {
    PpmImage img = RenderPpmLayer(m_world, ExportLayer::Overlay);
    const std::vector<std::uint8_t>& mask = m_evacScenario.hazardMask;
    if (static_cast<int>(mask.size()) == w * h) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const std::size_t i = static_cast<std::size_t>(y * w + x);
          if (mask[i]) overlayPixel(img, x, y, 0u, 160u, 255u, 180u);
        }
      }
    }
    write("_hazard.png", img);
  }

  auto rampColor = [](float v, std::uint8_t& outR, std::uint8_t& outG, std::uint8_t& outB) {
    v = std::clamp(v, 0.0f, 1.0f);
    outR = static_cast<std::uint8_t>(std::lround(255.0f * v));
    outG = static_cast<std::uint8_t>(std::lround(255.0f * (1.0f - v)));
    outB = 0u;
  };

  // Evacuation time overlay (residential tiles)
  {
    PpmImage img = RenderPpmLayer(m_world, ExportLayer::Overlay);
    const auto& hm = m_evacHeatmaps.evacTime;
    if (static_cast<int>(hm.size()) == w * h) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const float v = hm[static_cast<std::size_t>(y * w + x)];
          if (v <= 0.0f) continue;
          std::uint8_t r, g, b;
          rampColor(v, r, g, b);
          overlayPixel(img, x, y, r, g, b, 210u);
        }
      }
    }
    write("_time.png", img);
  }

  // Evac unreachable overlay (red)
  {
    PpmImage img = RenderPpmLayer(m_world, ExportLayer::Overlay);
    const auto& hm = m_evacHeatmaps.evacUnreachable;
    if (static_cast<int>(hm.size()) == w * h) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const float v = hm[static_cast<std::size_t>(y * w + x)];
          if (v <= 0.0f) continue;
          overlayPixel(img, x, y, 255u, 30u, 30u, 220u);
        }
      }
    }
    write("_unreachable.png", img);
  }

  // Evac road flow overlay (roads)
  {
    PpmImage img = RenderPpmLayer(m_world, ExportLayer::Overlay);
    const auto& hm = m_evacHeatmaps.evacFlow;
    if (static_cast<int>(hm.size()) == w * h) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          const float v = hm[static_cast<std::size_t>(y * w + x)];
          if (v <= 0.0f) continue;
          std::uint8_t r, g, b;
          rampColor(v, r, g, b);
          overlayPixel(img, x, y, r, g, b, 200u);
        }
      }
    }
    write("_flow.png", img);
  }

  showToast(std::string("Exported evac artifacts to: ") + base + "*", 3.0f);
}

void Game::doUndo()
{
  // Commit any in-progress stroke before undoing.
  endPaintStroke();

  std::uint64_t baseHash = 0;
  EditHistory::Command cmd;
  if (m_replayCapture.active()) {
    baseHash = HashWorld(m_world, true);
  }

  if (m_history.undo(m_world, m_replayCapture.active() ? &cmd : nullptr)) {
    m_sim.refreshDerivedStats(m_world);

    if (m_replayCapture.active()) {
      std::string err;
      (void)m_replayCapture.recordTileCommandPatch(m_world, cmd, baseHash, /*useBeforeAsTarget=*/true, err);
    }

    m_renderer.markMinimapDirty();
    m_renderer.markBaseCacheDirtyAll();
    m_roadGraphDirty = true;
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_landValueDirty = true;
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
    m_roadUpgradePlanDirty = true;
    m_roadUpgradeSelectedMaskDirty = true;
    invalidateHydrology();
    m_vehiclesDirty = true;
    showToast(TextFormat("Undo (%d left)", static_cast<int>(m_history.undoSize())));
  } else {
    showToast("Nothing to undo");
  }
}

void Game::doRedo()
{
  endPaintStroke();

  std::uint64_t baseHash = 0;
  EditHistory::Command cmd;
  if (m_replayCapture.active()) {
    baseHash = HashWorld(m_world, true);
  }

  if (m_history.redo(m_world, m_replayCapture.active() ? &cmd : nullptr)) {
    m_sim.refreshDerivedStats(m_world);

    if (m_replayCapture.active()) {
      std::string err;
      (void)m_replayCapture.recordTileCommandPatch(m_world, cmd, baseHash, /*useBeforeAsTarget=*/false, err);
    }

    m_renderer.markMinimapDirty();
    m_renderer.markBaseCacheDirtyAll();
    m_roadGraphDirty = true;
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_landValueDirty = true;
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
    m_roadUpgradePlanDirty = true;
    m_roadUpgradeSelectedMaskDirty = true;
    invalidateHydrology();
    m_vehiclesDirty = true;
    showToast(TextFormat("Redo (%d left)", static_cast<int>(m_history.redoSize())));
  } else {
    showToast("Nothing to redo");
  }
}

void Game::resetWorld(std::uint64_t newSeed)
{
  if (newSeed == 0) newSeed = TimeSeed();

  m_cfg.seed = newSeed;

  if (ui::IsReady()) {
    ui::SetSeed(newSeed);
  }
  m_world = GenerateWorld(m_cfg.mapWidth, m_cfg.mapHeight, newSeed, m_procCfg);
  m_renderer.markMinimapDirty();
  m_roadGraphDirty = true;
  m_trafficDirty = true;
  m_goodsDirty = true;
  m_landValueDirty = true;
  m_transitPlanDirty = true;
  m_transitVizDirty = true;
  m_evacDirty = true;
  invalidateHydrology();
  invalidateAnalysisLayers();
  m_vehiclesDirty = true;
  m_vehicles.clear();

  // Default flood overlay sea level tracks the current proc-gen water threshold.
  m_seaLevel = std::clamp(m_procCfg.waterLevel, 0.0f, 1.0f);

  // Deterministic vehicle RNG seed per world seed.
  m_vehicleRngState = (newSeed ^ 0x9E3779B97F4A7C15ULL);

// Reset policy advisor state (results are tied to the previous world snapshot).
m_policyOptHaveResult = false;
m_policyOptResultOk = false;
m_policyOptResultDay = -1;
m_policyOptResultErr.clear();
m_policyOptResult = PolicyOptimizationResult{};
m_policyOptTopSelection = 0;
m_policyOptTopFirst = 0;


  // New world invalidates history.
  m_history.clear();
  m_painting = false;

  // Clear inspect selection/debug overlays.
  m_inspectSelected.reset();
  m_inspectPath.clear();
  m_inspectPathCost = 0;
  m_inspectInfo.clear();

  // Any loaded world invalidates cached geocoding/wayfinding state.
  m_wayfindingDirty = true;
  clearWayfindingRoute();

  // Clear any in-progress road drag preview.
  m_roadDragActive = false;
  m_roadDragStart.reset();
  m_roadDragEnd.reset();
  m_roadDragPath.clear();
  m_roadDragBuildCost = 0;
  m_roadDragUpgradeTiles = 0;
  m_roadDragBridgeTiles = 0;
  m_roadDragMoneyCost = 0;
  m_roadDragValid = false;

  // Optional: vary procedural textures per seed (still no assets-from-disk).
  m_renderer.rebuildTextures(newSeed);
  m_renderer.markBaseCacheDirtyAll();

  // Any world reset invalidates the software 3D preview.
  m_3dPreviewDirty = true;
  m_3dPreviewTimer = 0.0f;

  // Make HUD stats immediately correct (without waiting for the first sim tick).
  m_sim.refreshDerivedStats(m_world);

  clearCityChallenges();
  clearHistory();
  {
    Stats s = m_world.stats();
    const int grant = applyCityChallenges(s);
    if (grant != 0) m_world.stats().money += grant;
    recordHistorySample(s);
  }

  // Update title with seed.
  SetWindowTitle(TextFormat("ProcIsoCity  |  seed: %llu", static_cast<unsigned long long>(newSeed)));

  // Recenter camera.
  m_camera.target = TileToWorldCenterElevated(m_world, m_cfg.mapWidth / 2, m_cfg.mapHeight / 2,
                                              static_cast<float>(m_cfg.tileWidth),
                                              static_cast<float>(m_cfg.tileHeight), m_elev);
}

void Game::run()
{
  using clock = std::chrono::steady_clock;
  auto toMs = [](clock::duration d) -> double {
    return std::chrono::duration<double, std::milli>(d).count();
  };

  while (!WindowShouldClose()) {
    const auto t0 = clock::now();

    // Raw delta time as measured by raylib.
    float dtRaw = GetFrameTime();
    if (!(dtRaw >= 0.0f)) dtRaw = 0.0f;

    m_loopStats.dtRaw = dtRaw;

    // Pause the real-time clock when the window is not interactive (optional).
    const bool focused = IsWindowFocused();
    const bool minimized = IsWindowState(FLAG_WINDOW_MINIMIZED);
    const bool focusPaused =
        (m_loopCfg.pauseWhenMinimized && minimized) || (m_loopCfg.pauseWhenUnfocused && !focused);

    m_loopStats.focused = focused;
    m_loopStats.minimized = minimized;
    m_loopStats.focusPaused = focusPaused;

    // Treat focus/minimize pauses as an automatic simulation pause (separate from user pause).
    m_simPausedAuto = focusPaused;


    // Optionally throttle frame rate when unfocused/minimized to reduce CPU/GPU churn.
    if (m_loopCfg.throttleFpsWhenUnfocused) {
      int desired = m_targetFpsApplied;
      if (focused && !minimized) desired = std::max(1, m_loopCfg.targetFpsFocused);
      else desired = std::max(1, m_loopCfg.targetFpsUnfocused);

      if (desired != m_targetFpsApplied) {
        SetTargetFPS(desired);
        m_targetFpsApplied = desired;
      }
    }
    m_loopStats.targetFps = m_targetFpsApplied;

    // Clamp dt used for input/camera/UI timers.
    float dtFrame = dtRaw;
    m_loopStats.dtClamped = false;
    if (m_loopCfg.maxFrameDt > 0.0f && dtFrame > m_loopCfg.maxFrameDt) {
      dtFrame = m_loopCfg.maxFrameDt;
      m_loopStats.dtClamped = true;
    }

    // Clamp dt fed to the simulation clock separately.
    float dtSimBase = dtRaw;
    if (m_loopCfg.maxSimDt > 0.0f && dtSimBase > m_loopCfg.maxSimDt) {
      dtSimBase = m_loopCfg.maxSimDt;
    }

    // If we paused due to focus/minimize, freeze clocks and avoid a catch-up burst on resume.
    // (dtRaw can be huge when returning from alt-tab / breakpoint / window drag.)
    if (focusPaused) {
      dtFrame = 0.0f;
      dtSimBase = 0.0f;

      // Clear pending sim time on the first paused frame.
      if (!m_focusPausePrev) {
        m_sim.resetTimer();
      }
    } else if (m_focusPausePrev) {
      // First frame after unpausing: swallow the resume hitch.
      dtFrame = 0.0f;
      dtSimBase = 0.0f;
      m_sim.resetTimer();
    }
    m_focusPausePrev = focusPaused;

    m_loopStats.dtFrame = dtFrame;

    m_timeSec += dtFrame;

    const auto t1 = clock::now();
    handleInput(dtFrame);
    const auto t2 = clock::now();

    // Compute dt for the simulation after input, so speed hotkeys apply immediately.
    const float simSpeed =
        kSimSpeeds[static_cast<std::size_t>(std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1))];
    m_loopStats.dtSim = dtSimBase * simSpeed;

    update(dtFrame);
    const auto t3 = clock::now();

    draw();
    const auto t4 = clock::now();

    // CPU timing for the perf overlay (note: draw() may include vsync waits).
    m_loopStats.cpuInputMs = toMs(t2 - t1);
    m_loopStats.cpuUpdateMs = toMs(t3 - t2);
    m_loopStats.cpuDrawMs = toMs(t4 - t3);
    m_loopStats.cpuFrameMs = toMs(t4 - t0);
  }
}
void Game::floodFillDistrict(Point start, bool includeRoads)
{
  if (!m_world.inBounds(start.x, start.y)) return;

  beginPaintStroke();

  const int w = m_world.width();
  const int h = m_world.height();
  const int n = w * h;

  const std::uint8_t targetDistrict = static_cast<std::uint8_t>(std::clamp(m_activeDistrict, 0, kDistrictCount - 1));
  const Tile& seed = m_world.at(start.x, start.y);

  enum class FillMode { RoadComponent, WaterBody, LandBlock };
  const FillMode mode = (seed.overlay == Overlay::Road) ? FillMode::RoadComponent :
                        (seed.terrain == Terrain::Water) ? FillMode::WaterBody :
                                                          FillMode::LandBlock;

  auto canFill = [&](int x, int y) -> bool {
    if (!m_world.inBounds(x, y)) return false;
    const Tile& t = m_world.at(x, y);
    switch (mode) {
    case FillMode::RoadComponent:
      return t.overlay == Overlay::Road;
    case FillMode::WaterBody:
      // Water fill excludes bridges (road overlay).
      return t.terrain == Terrain::Water && t.overlay != Overlay::Road;
    case FillMode::LandBlock:
    default:
      if (t.terrain == Terrain::Water) return false;
      if (!includeRoads && t.overlay == Overlay::Road) return false;
      return true;
    }
  };

  std::vector<std::uint8_t> visited(static_cast<std::size_t>(n), std::uint8_t{0});
  std::vector<Point> stack;
  stack.reserve(std::min(n, 4096));

  auto push = [&](int x, int y) {
    const int idx = y * w + x;
    if (visited[static_cast<std::size_t>(idx)]) return;
    visited[static_cast<std::size_t>(idx)] = 1;
    stack.push_back({x, y});
  };

  if (canFill(start.x, start.y)) push(start.x, start.y);

  int changed = 0;
  while (!stack.empty()) {
    const Point p = stack.back();
    stack.pop_back();

    Tile& t = m_world.at(p.x, p.y);
    if (t.district != targetDistrict) {
      m_history.noteTilePreEdit(m_world, p.x, p.y);
      t.district = targetDistrict;
      ++changed;
    }

    const int x = p.x;
    const int y = p.y;
    if (x > 0 && canFill(x - 1, y)) push(x - 1, y);
    if (x + 1 < w && canFill(x + 1, y)) push(x + 1, y);
    if (y > 0 && canFill(x, y - 1)) push(x, y - 1);
    if (y + 1 < h && canFill(x, y + 1)) push(x, y + 1);
  }

  endPaintStroke();
  if (changed > 0) {
    showToast(TextFormat("District fill: %d tiles", changed));
  }
}

void Game::floodFillTool(Point start, bool includeRoads)
{
  if (!m_world.inBounds(start.x, start.y)) return;

  // Only a subset of tools make sense for flood fill.
  const bool supported = (m_tool == Tool::Residential || m_tool == Tool::Commercial || m_tool == Tool::Industrial ||
                          m_tool == Tool::Park || m_tool == Tool::Bulldoze);
  if (!supported) return;

  // Region selection matches the district fill behavior:
  //  - click a road: select that connected road component
  //  - click water: select that connected water body (excluding bridges)
  //  - click land: select the land block bounded by water and (optionally) roads
  const FloodFillResult region = FloodFillAuto(m_world, start, includeRoads);
  if (region.tiles.empty()) return;

  const int moneyBefore = m_world.stats().money;

  std::vector<Point> targets;
  targets.reserve(region.tiles.size());

  int estimatedCost = 0;

  auto labelForTool = [&]() -> const char* {
    switch (m_tool) {
    case Tool::Residential: return "Residential";
    case Tool::Commercial: return "Commercial";
    case Tool::Industrial: return "Industrial";
    case Tool::Park: return "Park";
    case Tool::Bulldoze: return "Bulldoze";
    default: return "Tool";
    }
  };

  if (m_tool == Tool::Park) {
    // Parks only place on empty non-water tiles.
    for (const Point& p : region.tiles) {
      if (!m_world.inBounds(p.x, p.y)) continue;
      const Tile& t = m_world.at(p.x, p.y);
      if (t.terrain == Terrain::Water) continue;
      if (t.overlay != Overlay::None) continue;
      targets.push_back(p);
      estimatedCost += 3;
    }
  } else if (m_tool == Tool::Bulldoze) {
    // Bulldoze clears any non-empty overlay. (Cost is always 0.)
    for (const Point& p : region.tiles) {
      if (!m_world.inBounds(p.x, p.y)) continue;
      const Tile& t = m_world.at(p.x, p.y);
      if (t.overlay == Overlay::None) continue;
      targets.push_back(p);
    }
  } else {
    // Zoning tools: place or upgrade where possible.
    const Overlay zone = (m_tool == Tool::Residential) ? Overlay::Residential :
                         (m_tool == Tool::Commercial) ? Overlay::Commercial :
                                                       Overlay::Industrial;

    for (const Point& p : region.tiles) {
      if (!m_world.inBounds(p.x, p.y)) continue;
      const Tile& t = m_world.at(p.x, p.y);
      if (t.terrain == Terrain::Water) continue;

      // Zoning rules: must have adjacent road.
      if (!m_world.hasAdjacentRoad(p.x, p.y)) continue;

      if (t.overlay == Overlay::None) {
        targets.push_back(p);
        estimatedCost += 5;
      } else if (t.overlay == zone) {
        // Upgrade only if not already max level.
        if (static_cast<int>(t.level) >= 3) continue;
        targets.push_back(p);
        estimatedCost += 5;
      }
    }
  }

  if (targets.empty()) {
    showToast(TextFormat("%s fill: no eligible tiles", labelForTool()));
    return;
  }

  // Make flood fill atomic for costed tools (parks + zoning): if we can't afford the whole
  // plan, don't partially build.
  if (estimatedCost > moneyBefore) {
    showToast(TextFormat("Not enough funds for %s fill: need $%d (short $%d)",
                         labelForTool(), estimatedCost, estimatedCost - moneyBefore),
              3.0f);
    return;
  }

  beginPaintStroke();

  const int savedRadius = m_brushRadius;
  m_brushRadius = 0; // flood fills are always single-tile wide

  for (const Point& p : targets) {
    applyToolBrush(p.x, p.y);
  }

  m_brushRadius = savedRadius;

  const bool hadFailures = m_strokeFeedback.any();
  endPaintStroke();

  if (!hadFailures) {
    const int spent = moneyBefore - m_world.stats().money;
    if (m_tool == Tool::Bulldoze) {
      showToast(TextFormat("Bulldozed %d tiles", static_cast<int>(targets.size())));
    } else if (spent > 0) {
      showToast(TextFormat("%s fill: %d tiles (cost %d)", labelForTool(), static_cast<int>(targets.size()), spent));
    } else {
      showToast(TextFormat("%s fill: %d tiles", labelForTool(), static_cast<int>(targets.size())));
    }
  }
}

void Game::ensureWayfindingUpToDate() {
  if (!m_wayfindingDirty) {
    return;
  }

  // NOTE: This is intentionally derived, not persisted. It can be moderately expensive on huge
  // maps, so we compute it lazily and invalidate it on any edit stroke.
  m_wayfindingStreets = BuildStreetNames(m_world);
  m_wayfindingAddresses = BuildParcelAddresses(m_world, m_wayfindingStreets);
  m_wayfindingIndex = BuildAddressIndex(m_wayfindingAddresses);

  m_wayfindingDirty = false;
}

void Game::clearWayfindingRoute() {
  m_wayfindingFromQuery.clear();
  m_wayfindingToQuery.clear();

  m_wayfindingRoute = {};
  m_wayfindingFocusManeuver = -1;
  m_wayfindingFocusPath.clear();
  m_wayfindingManeuverFirst = 0;

  // Keep panel state, but hide the overlay by default so a cleared route doesn't leave artifacts.
  m_showWayfindingOverlay = false;
}

void Game::rebuildWayfindingFocusPath() {
  m_wayfindingFocusPath.clear();

  if (!m_wayfindingRoute.ok) {
    return;
  }
  if (m_wayfindingRoute.pathTiles.empty()) {
    return;
  }

  const int n = static_cast<int>(m_wayfindingRoute.maneuvers.size());
  if (m_wayfindingFocusManeuver < 0 || m_wayfindingFocusManeuver >= n) {
    return;
  }

  const RouteManeuver& m = m_wayfindingRoute.maneuvers[m_wayfindingFocusManeuver];
  const int last = static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1;

  int a = std::clamp(m.pathStart, 0, last);
  int b = std::clamp(m.pathEnd, 0, last);
  if (a > b) {
    std::swap(a, b);
  }

  // Inclusive range [a, b].
  m_wayfindingFocusPath.assign(m_wayfindingRoute.pathTiles.begin() + a, m_wayfindingRoute.pathTiles.begin() + b + 1);
}

void Game::drawWayfindingOverlay() {
  if (!m_showWayfindingOverlay || !m_wayfindingRoute.ok) {
    return;
  }

  const ParcelAddress& from = m_wayfindingRoute.from;
  const ParcelAddress& to = m_wayfindingRoute.to;

  const Vector2 fromWs = TileToWorldCenterElevated(
      m_world, from.roadTile.x, from.roadTile.y,
      static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), m_elev);
  const Vector2 toWs = TileToWorldCenterElevated(
      m_world, to.roadTile.x, to.roadTile.y,
      static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), m_elev);

  BeginMode2D(m_camera);

  // Keep marker size roughly stable relative to zoom.
  const float r = std::clamp(6.0f / m_camera.zoom, 2.0f, 12.0f);

  DrawCircleV(fromWs, r, Color{0, 220, 120, 220});
  DrawCircleV(toWs, r, Color{255, 80, 80, 220});

  EndMode2D();
}

void Game::drawWayfindingPanel(int x0, int y0) {
  const ui::Theme& uiTh = ui::GetTheme();
  const float uiTime = static_cast<float>(GetTime());
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  const int panelW = kWayfindingPanelW;
  const int panelH = kWayfindingPanelH;
  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW), static_cast<float>(panelH)};

  ui::DrawPanel(panelR, uiTime, m_showWayfindingPanel);
  ui::DrawPanelHeader(panelR, "Navigator", uiTime, m_showWayfindingPanel);

  // Header buttons.
  const bool hasRoute = m_wayfindingRoute.ok;
  if (ui::Button(7600, Rectangle{static_cast<float>(x0 + panelW - 162), static_cast<float>(y0 + 40), 72.0f, 18.0f}, "Clear", mouseUi, uiTime, hasRoute, false)) {
    clearWayfindingRoute();
    showToast("Route cleared", 2.0f);
  }

  const bool canToggleOverlay = hasRoute || m_showWayfindingOverlay;
  if (ui::Button(7601, Rectangle{static_cast<float>(x0 + panelW - 82), static_cast<float>(y0 + 40), 72.0f, 18.0f},
                 m_showWayfindingOverlay ? "Hide" : "Show", mouseUi, uiTime, canToggleOverlay, false)) {
    m_showWayfindingOverlay = !m_showWayfindingOverlay;
  }

  int x = x0 + 12;
  int y = y0 + 42;

  ui::Text(x, y, 15, "Console: route \"from\" \"to\"   |   route panel", uiTh.textDim, false, true, 1);
  y += 22;

  if (!hasRoute) {
    const char* tip = "No active route.\n"
                      "Try: route \"x,y\" \"x,y\"\n"
                      "Or: route \"street1 & street2\" \"street3 & street4\"";
    ui::TextBox(Rectangle{static_cast<float>(x), static_cast<float>(y), static_cast<float>(panelW - 24), 54.0f},
                14, tip, uiTh.text, /*bold=*/false, /*shadow=*/true, /*spacingPx=*/1,
                /*wrap=*/true, /*clip=*/true);
    y += 62;
  }

  // Summary.
  if (hasRoute) {
    const std::string summary = TextFormat("From: %s\nTo: %s\nSteps: %d    Tiles: %d", m_wayfindingRoute.from.full.c_str(),
                                           m_wayfindingRoute.to.full.c_str(), static_cast<int>(m_wayfindingRoute.maneuvers.size()),
                                           static_cast<int>(m_wayfindingRoute.pathTiles.size()));
    ui::TextBox(Rectangle{static_cast<float>(x), static_cast<float>(y), static_cast<float>(panelW - 24), 56.0f},
                14, summary, uiTh.text, /*bold=*/false, /*shadow=*/true, /*spacingPx=*/1,
                /*wrap=*/true, /*clip=*/true);
    y += 64;
  }

  // Procedural POV controls (works even without a route).
  {
    // This panel is intentionally "dense"; we keep it self-contained so the
    // maneuver list below remains stable.
    const int boxH = 300;
    const Rectangle boxR{static_cast<float>(x0 + 10), static_cast<float>(y), static_cast<float>(panelW - 20), static_cast<float>(boxH)};
    ui::DrawPanelInset(boxR, uiTime);

    int bx = x0 + 18;
    int by = y + 8;

    ui::Text(bx, by, 15, "Procedural POV", uiTh.textDim, true, true, 1);

    const bool povOn = m_pov.isActive();
    const float btnH = 20.0f;
    const float btnW = (boxR.width - 10.0f) * 0.5f;

    const std::string rideLabel = povOn ? "Stop POV" : (hasRoute ? "Ride route" : "Ride route (no route)");

    const Rectangle rideBtn{boxR.x + 8.0f, boxR.y + 26.0f, btnW, btnH};
    const Rectangle tourBtn{boxR.x + 8.0f + btnW + 6.0f, boxR.y + 26.0f, btnW, btnH};
    const Rectangle roamBtn{boxR.x + 8.0f, boxR.y + 26.0f + btnH + 6.0f, boxR.width - 16.0f, btnH};

    if (ui::Button(7610, rideBtn, rideLabel, mouseUi, uiTime, povOn || hasRoute, povOn)) {
      if (povOn) {
        stopPov();
      } else if (hasRoute) {
        std::vector<PovMarker> markers;
        markers.reserve(m_wayfindingRoute.maneuvers.size());
        for (const RouteManeuver& man : m_wayfindingRoute.maneuvers) {
          if (man.type == "turn") {
            PovMarker mk;
            mk.pathIndex = std::clamp(man.pathStart, 0, static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1);
            mk.label = man.instruction;
            mk.holdSec = 0.35f;
            markers.push_back(std::move(mk));
          } else if (man.type == "arrive") {
            PovMarker mk;
            mk.pathIndex = std::clamp(man.pathStart, 0, static_cast<int>(m_wayfindingRoute.pathTiles.size()) - 1);
            mk.label = man.instruction;
            mk.holdSec = 0.8f;
            markers.push_back(std::move(mk));
          }
        }

        m_pov.setTitle("Route");
        const bool ok = m_pov.startFromPath(
            m_world, m_wayfindingRoute.pathTiles, markers, static_cast<float>(m_cfg.tileWidth),
            static_cast<float>(m_cfg.tileHeight), m_elev, m_camera, GetScreenWidth(), GetScreenHeight(),
            static_cast<std::uint32_t>(m_world.seed()));

        showToast(ok ? "POV ride started" : "POV failed to start", 2.0f);
      }
    }

    if (ui::Button(7611, tourBtn, "Tour POV", mouseUi, uiTime, !povOn, false)) {
      stopPov();

      // Build a small procedural tour and ride it.
      TourConfig cfg;
      cfg.maxStops = 6;
      const std::string startQuery = m_wayfindingFromQuery.empty() ? std::string{} : m_wayfindingFromQuery;
      const TourPlan tour = BuildProceduralTour(m_world, startQuery, cfg);

      const bool ok = m_pov.startFromTour(
          m_world, tour, static_cast<float>(m_cfg.tileWidth), static_cast<float>(m_cfg.tileHeight), m_elev,
          m_camera, GetScreenWidth(), GetScreenHeight(), static_cast<std::uint32_t>(tour.seed ? tour.seed : m_world.seed()));

      showToast(ok ? "POV tour started" : "POV tour could not be generated", 2.0f);
    }

    if (ui::Button(7614, roamBtn, "Roam POV", mouseUi, uiTime, !povOn, false)) {
      stopPov();
      const bool ok = startRoamPov(m_povRoamSeed + 1u, m_povRoamCfg.length);
      showToast(ok ? "POV roam started" : "POV roam failed", 2.0f);
    }

    // Sliders.
    const int sx = x0 + 18;
    const int sw = panelW - 44;
    int sy = y + 78;

    // Speed.
    ui::Text(sx, sy, 13, TextFormat("Speed: %.1f tiles/s", m_pov.config().speedTilesPerSec), uiTh.textDim, false, true, 1);
    const Rectangle speedR{static_cast<float>(sx), static_cast<float>(sy + 16), static_cast<float>(sw), 14.0f};
    ui::SliderFloat(7612, speedR, m_pov.config().speedTilesPerSec, 2.0f, 40.0f, mouseUi, uiTime, true);
    sy += 34;

    // Roam length.
    ui::Text(sx, sy, 13, TextFormat("Roam length: %d tiles", m_povRoamCfg.length), uiTh.textDim, false, true, 1);
    const Rectangle lenR{static_cast<float>(sx), static_cast<float>(sy + 16), static_cast<float>(sw), 14.0f};
    ui::SliderInt(7615, lenR, m_povRoamCfg.length, 200, 2500, 50, mouseUi, uiTime, true);
    sy += 34;

    // Straight bias.
    ui::Text(sx, sy, 13, TextFormat("Straight bias: %.2f", m_povRoamCfg.straightBias), uiTh.textDim, false, true, 1);
    const Rectangle straightR{static_cast<float>(sx), static_cast<float>(sy + 16), static_cast<float>(sw), 14.0f};
    ui::SliderFloat(7616, straightR, m_povRoamCfg.straightBias, 0.0f, 1.0f, mouseUi, uiTime, true);
    sy += 34;

    // Scenic bias.
    ui::Text(sx, sy, 13, TextFormat("Scenic bias: %.2f", m_povRoamCfg.scenicBias), uiTh.textDim, false, true, 1);
    const Rectangle scenicR{static_cast<float>(sx), static_cast<float>(sy + 16), static_cast<float>(sw), 14.0f};
    ui::SliderFloat(7617, scenicR, m_povRoamCfg.scenicBias, 0.0f, 1.0f, mouseUi, uiTime, true);
    sy += 34;

    // Revisit penalty.
    ui::Text(sx, sy, 13, TextFormat("Revisit penalty: %.2f", m_povRoamCfg.revisitPenalty), uiTh.textDim, false, true, 1);
    const Rectangle revisitR{static_cast<float>(sx), static_cast<float>(sy + 16), static_cast<float>(sw), 14.0f};
    ui::SliderFloat(7618, revisitR, m_povRoamCfg.revisitPenalty, 0.0f, 1.25f, mouseUi, uiTime, true);
    sy += 34;

    // Dead-end avoidance toggle.
    const Rectangle deadR{static_cast<float>(sx), static_cast<float>(sy), 34.0f, 16.0f};
    const Rectangle deadLabelR{deadR.x + deadR.width + 6.0f, deadR.y, 120.0f, deadR.height};
    const bool deadChanged = ui::Toggle(7619, deadR, m_povRoamCfg.avoidDeadEnds, mouseUi, uiTime, true);
    ui::Text(static_cast<int>(std::lround(deadLabelR.x)), static_cast<int>(std::lround(deadLabelR.y + 1.0f)),
             13, "Avoid dead ends", uiTh.textDim, false, true, 1);
    if (!deadChanged && CheckCollisionPointRec(mouseUi, deadLabelR) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_povRoamCfg.avoidDeadEnds = !m_povRoamCfg.avoidDeadEnds;
    }
    sy += 22;

    // 3D preview toggle.
    // ui::Toggle is a "switch" widget; we draw a label alongside it and make the label clickable.
    const Rectangle togR{static_cast<float>(sx), static_cast<float>(sy), 34.0f, 16.0f};
    const Rectangle togLabelR{togR.x + togR.width + 6.0f, togR.y, 86.0f, togR.height};
    const bool togChanged = ui::Toggle(7613, togR, m_povDrive3DPreview, mouseUi, uiTime, true);
    ui::Text(static_cast<int>(std::lround(togLabelR.x)), static_cast<int>(std::lround(togLabelR.y + 1.0f)),
             13, "3D preview", uiTh.textDim, false, true, 1);
    if (!togChanged && CheckCollisionPointRec(mouseUi, togLabelR) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
      m_povDrive3DPreview = !m_povDrive3DPreview;
    }

    y += boxH + 10;
  }

  if (!hasRoute) {
    return;
  }

  // Maneuver list with scrolling.
  const int rowH = 20;
  const int listX = x0 + 10;
  const int listY = y;
  const int listW = panelW - 20;
  const int listH = panelH - (listY - y0) - 12;
  const Rectangle listR{static_cast<float>(listX), static_cast<float>(listY), static_cast<float>(listW), static_cast<float>(listH)};
  ui::DrawPanelInset(listR, uiTime);

  const int total = static_cast<int>(m_wayfindingRoute.maneuvers.size());
  const int view = std::max(1, listH / rowH);
  const int maxFirst = std::max(0, total - view);
  m_wayfindingManeuverFirst = std::clamp(m_wayfindingManeuverFirst, 0, maxFirst);

  if (CheckCollisionPointRec(mouseUi, listR) && total > view) {
    const float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
      m_wayfindingManeuverFirst = std::clamp(m_wayfindingManeuverFirst - static_cast<int>(wheel), 0, maxFirst);
    }
  }

  // Scrollbar.
  const Rectangle scrollR{static_cast<float>(listX + listW - 12), static_cast<float>(listY), 12.0f, static_cast<float>(listH)};
  (void)ui::ScrollbarV(7602, scrollR, total, view, m_wayfindingManeuverFirst, mouseUi, uiTime);

  const int textW = listW - 14;
  for (int i = 0; i < view; ++i) {
    const int idx = m_wayfindingManeuverFirst + i;
    if (idx >= total) {
      break;
    }
    const RouteManeuver& m = m_wayfindingRoute.maneuvers[idx];

    std::string label = TextFormat("%2d. %s", idx + 1, m.instruction.c_str());
    if (label.size() > 92) {
      label.resize(89);
      label += "...";
    }

    const Rectangle rowR{static_cast<float>(listX + 2), static_cast<float>(listY + 2 + i * rowH), static_cast<float>(textW), static_cast<float>(rowH - 2)};
    const bool isFocus = (idx == m_wayfindingFocusManeuver);
    if (ui::Button(7700 + idx, rowR, label, mouseUi, uiTime, true, isFocus)) {
      // Toggle focus; when focused we render the segment only.
      if (m_wayfindingFocusManeuver == idx) {
        m_wayfindingFocusManeuver = -1;
        m_wayfindingFocusPath.clear();
      } else {
        m_wayfindingFocusManeuver = idx;
        rebuildWayfindingFocusPath();
      }
    }
  }
}



void Game::runAutoBuild(int days, const AutoBuildConfig& cfg, const char* toastPrefix)
{
  endPaintStroke();

  if (days <= 0) {
    showToast("AutoBuild: days must be >= 1", 2.0f);
    return;
  }

  World w = m_world;
  Simulator sim = m_sim;

  std::vector<Stats> tickStats;
  const AutoBuildReport rep = RunAutoBuild(w, sim, cfg, days, &tickStats);
  m_autoBuildLastReport = rep;

  // Preserve history so the report panel remains continuous after fast-forwarding.
  std::vector<CityHistorySample> prevHistory = std::move(m_cityHistory);

  std::string toast;
  if (toastPrefix && toastPrefix[0] != '\0') {
    toast = TextFormat("%s: %d days | roads %d (+%d) | zones %d | parks %d | fails %d",
                       toastPrefix, rep.daysSimulated, rep.roadsBuilt, rep.roadsUpgraded, rep.zonesBuilt,
                       rep.parksBuilt, rep.failedBuilds);
  } else {
    toast = TextFormat("AutoBuild: %d days | roads %d (+%d) | zones %d | parks %d | fails %d",
                       rep.daysSimulated, rep.roadsBuilt, rep.roadsUpgraded, rep.zonesBuilt,
                       rep.parksBuilt, rep.failedBuilds);
  }

  // Commit new world state (recomputes caches and clears transient selections).
  adoptLoadedWorld(std::move(w), m_procCfg, m_sim.config(), toast, nullptr);

  // Restore and append simulation history samples.
  m_cityHistory = std::move(prevHistory);
  if (!tickStats.empty()) {
    int moneyOffset = 0;
    for (const Stats& s : tickStats) {
      Stats ss = s;
      ss.money += moneyOffset;
      const int grant = applyCityChallenges(ss);
      moneyOffset += grant;
      recordHistorySample(ss);
    }
    if (moneyOffset != 0) {
      m_world.stats().money += moneyOffset;
    }
  } else {
    Stats s = m_world.stats();
    const int grant = applyCityChallenges(s);
    if (grant != 0) m_world.stats().money += grant;
    recordHistorySample(s);
  }
}


void Game::drawAutoBuildPanel(int x0, int y0)
{
  const float uiScale = m_uiScale;
  const Vector2 mouseUi = mouseUiPosition(uiScale);
  const float uiTime = static_cast<float>(GetTime());
  const auto& uiTh = ui::GetTheme();

  const int panelW = kAutoBuildPanelW;
  const int panelH = kAutoBuildPanelH;

  Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                   static_cast<float>(panelH)};

  ui::DrawPanel(panelR, uiTime, true);
  ui::DrawPanelHeader(panelR, "AutoBuild Bot", uiTime, true);

  // Close button.
  const Rectangle closeBtn{panelR.x + panelR.width - 28.0f, panelR.y + 8.0f, 20.0f, 20.0f};
  if (ui::Button(6101, closeBtn, "X", mouseUi, uiTime)) {
    m_showAutoBuildPanel = false;
    ui::ClearActiveWidget();
    return;
  }

  int x = x0 + 12;
  int y = y0 + 36;

  ui::Text(x, y, 14, "Deterministic city growth automation for the current world.", uiTh.textDim);
  y += 18;

  // Run length.
  ui::Text(x, y, 14, "Days", uiTh.text);
  {
    const float valueW = 52.0f;
    Rectangle sliderR{static_cast<float>(x + 56), static_cast<float>(y - 4),
                      static_cast<float>(panelW - 56 - 12) - valueW, 18.0f};
    ui::SliderInt(6102, sliderR, m_autoBuildRunDays, 1, 365, 1, mouseUi, uiTime);
    ui::Text(x0 + panelW - 12 - static_cast<int>(valueW), y, 14,
             TextFormat("%d", m_autoBuildRunDays), uiTh.text);
  }
  y += 24;

  // Buttons.
  const int btnH = 22;
  const int btnW = 84;
  Rectangle runBtn{static_cast<float>(x), static_cast<float>(y), static_cast<float>(btnW),
                   static_cast<float>(btnH)};
  Rectangle stepBtn{static_cast<float>(x + btnW + 8), static_cast<float>(y), static_cast<float>(btnW),
                    static_cast<float>(btnH)};
  Rectangle resetBtn{static_cast<float>(x + 2 * (btnW + 8)), static_cast<float>(y),
                     static_cast<float>(btnW), static_cast<float>(btnH)};

  if (ui::Button(6103, runBtn, "Run", mouseUi, uiTime, true, true)) {
    ui::ClearActiveWidget();
    runAutoBuild(m_autoBuildRunDays, m_autoBuildCfg, "AutoBuild");
  }

  if (ui::Button(6104, stepBtn, "Step", mouseUi, uiTime, true, true)) {
    ui::ClearActiveWidget();
    runAutoBuild(1, m_autoBuildCfg, "AutoBuild");
  }

  if (ui::Button(6105, resetBtn, "Reset", mouseUi, uiTime, true, false)) {
    m_autoBuildCfg = AutoBuildConfig{};
    m_autoBuildPanelScroll = 0;
    ui::ClearActiveWidget();
    showToast("Bot config reset", 2.0f);
  }

  y += btnH + 10;

  // Scrollable config list.
  const int bottomH = 64;
  const int listX = x0 + 12;
  const int listY = y;
  const int listW = panelW - 24;
  const int listH = panelH - (listY - y0) - bottomH;
  Rectangle listR{static_cast<float>(listX), static_cast<float>(listY), static_cast<float>(listW),
                  static_cast<float>(listH)};

  ui::DrawPanelInset(listR, uiTime);

  constexpr int rowH = 24;
  constexpr int contentRows = 26;
  const int viewRows = std::max(1, static_cast<int>(listR.height) / rowH);

  m_autoBuildPanelScroll = std::clamp(m_autoBuildPanelScroll, 0, std::max(0, contentRows - viewRows));

  // Scrollbar.
  const Rectangle scrollbarR{listR.x + listR.width - 12.0f, listR.y, 12.0f, listR.height};
  ui::ScrollbarV(6106, scrollbarR, contentRows, viewRows, m_autoBuildPanelScroll, mouseUi, uiTime, true);

  const Rectangle contentR = ui::ContentRectWithScrollbar(listR);
  const float rowX = contentR.x + 6.0f;
  const float widgetW = 160.0f;
  const float widgetX = contentR.x + contentR.width - widgetW - 6.0f;

  auto drawHeading = [&](int yRow, const char* label) {
    ui::Text(static_cast<int>(rowX), yRow + 4, 15, label, uiTh.textDim, true, false, 1);
  };

  auto drawBoolRow = [&](int id, int yRow, const char* label, bool& v, bool enabled) {
    ui::Text(static_cast<int>(rowX), yRow + 4, 14, label, enabled ? uiTh.text : uiTh.textDim);
    Rectangle tR{widgetX + widgetW - 44.0f, static_cast<float>(yRow + 2), 44.0f,
                 static_cast<float>(rowH - 4)};
    ui::Toggle(id, tR, v, mouseUi, uiTime, enabled);
  };

  auto drawIntRow = [&](int id, int yRow, const char* label, int& v, int lo, int hi, int step,
                        bool enabled) {
    ui::Text(static_cast<int>(rowX), yRow + 4, 14, TextFormat("%s: %d", label, v),
             enabled ? uiTh.text : uiTh.textDim);
    Rectangle sR{widgetX, static_cast<float>(yRow + 3), widgetW, static_cast<float>(rowH - 6)};
    ui::SliderInt(id, sR, v, lo, hi, step, mouseUi, uiTime, enabled);
  };

  auto drawFloatRow = [&](int id, int yRow, const char* label, float& v, float lo, float hi, float step,
                          bool enabled) {
    ui::Text(static_cast<int>(rowX), yRow + 4, 14, TextFormat("%s: %.2f", label, v),
             enabled ? uiTh.text : uiTh.textDim);
    Rectangle sR{widgetX, static_cast<float>(yRow + 3), widgetW, static_cast<float>(rowH - 6)};
    ui::SliderFloat(id, sR, v, lo, hi, step, mouseUi, uiTime, enabled);
  };

  const int first = m_autoBuildPanelScroll;
  const int last = std::min(contentRows, first + viewRows);

  for (int r = first; r < last; ++r) {
    const int yRow = listY + (r - first) * rowH;
    const int idBase = 6200 + r * 10;

    switch (r) {
      case 0:
        drawHeading(yRow, "Growth");
        break;
      case 1:
        drawIntRow(idBase, yRow, "Zones/day", m_autoBuildCfg.zonesPerDay, 0, 8, 1, true);
        break;
      case 2:
        drawIntRow(idBase, yRow, "Zone cluster", m_autoBuildCfg.zoneClusterMaxTiles, 1, 12, 1, true);
        break;
      case 3:
        drawIntRow(idBase, yRow, "Roads/day", m_autoBuildCfg.roadsPerDay, 0, 8, 1, true);
        break;
      case 4:
        drawBoolRow(idBase, yRow, "Use road planner", m_autoBuildCfg.useRoadPlanner, true);
        break;
      case 5:
        drawIntRow(idBase, yRow, "Road level", m_autoBuildCfg.roadLevel, 1, 3, 1, true);
        break;
      case 6:
        drawIntRow(idBase, yRow, "Road spur length", m_autoBuildCfg.maxRoadSpurLength, 2, 60, 1, true);
        break;
      case 7:
        drawBoolRow(idBase, yRow, "Allow bridges", m_autoBuildCfg.allowBridges, true);
        break;
      case 8:
        drawIntRow(idBase, yRow, "Parks/day", m_autoBuildCfg.parksPerDay, 0, 8, 1, true);
        break;
      case 9:
        drawBoolRow(idBase, yRow, "Use park optimizer", m_autoBuildCfg.useParkOptimizer, true);
        break;
      case 10:
        drawIntRow(idBase, yRow, "Park per zone tiles", m_autoBuildCfg.parkPerZoneTiles, 1, 80, 1, true);
        break;
      case 11:
        drawIntRow(idBase, yRow, "Min money reserve", m_autoBuildCfg.minMoneyReserve, 0, 250, 1, true);
        break;
      case 12:
        drawIntRow(idBase, yRow, "Land value recalc", m_autoBuildCfg.landValueRecalcDays, 1, 60, 1, true);
        break;
      case 13:
        drawHeading(yRow, "Outside connection");
        break;
      case 14:
        drawBoolRow(idBase, yRow, "Respect existing", m_autoBuildCfg.respectOutsideConnection, true);
        break;
      case 15:
        drawBoolRow(idBase, yRow, "Ensure exists", m_autoBuildCfg.ensureOutsideConnection, true);
        break;
      case 16:
        drawHeading(yRow, "Road upgrades");
        break;
      case 17:
        drawBoolRow(idBase, yRow, "Auto-upgrade roads", m_autoBuildCfg.autoUpgradeRoads, true);
        break;
      case 18:
        drawFloatRow(idBase, yRow, "Congestion", m_autoBuildCfg.congestionUpgradeThreshold, 0.0f, 1.0f, 0.01f,
                     m_autoBuildCfg.autoUpgradeRoads);
        break;
      case 19:
        drawIntRow(idBase, yRow, "Upgrades/day", m_autoBuildCfg.roadUpgradesPerDay, 0, 20, 1,
                   m_autoBuildCfg.autoUpgradeRoads);
        break;
      case 20:
        drawHeading(yRow, "Resilience bypass");
        break;
      case 21:
        drawBoolRow(idBase, yRow, "Auto-build bypasses", m_autoBuildCfg.autoBuildResilienceBypasses, true);
        break;
      case 22:
        drawFloatRow(idBase, yRow, "Bypass congestion", m_autoBuildCfg.resilienceBypassCongestionThreshold, 0.0f,
                     1.0f, 0.01f, m_autoBuildCfg.autoBuildResilienceBypasses);
        break;
      case 23:
        drawIntRow(idBase, yRow, "Bypasses/day", m_autoBuildCfg.resilienceBypassesPerDay, 0, 20, 1,
                   m_autoBuildCfg.autoBuildResilienceBypasses);
        break;
      case 24:
        drawBoolRow(idBase, yRow, "Bypass bridges", m_autoBuildCfg.resilienceBypassAllowBridges,
                    m_autoBuildCfg.autoBuildResilienceBypasses);
        break;
      case 25:
        drawIntRow(idBase, yRow, "Bypass target", m_autoBuildCfg.resilienceBypassTargetLevel, 1, 3, 1,
                   m_autoBuildCfg.autoBuildResilienceBypasses);
        break;
      default:
        break;
    }
  }

  // Bottom status.
  const AutoBuildReport& rep = m_autoBuildLastReport;
  const int by = y0 + panelH - 50;
  ui::Text(x0 + 12, by, 14,
           TextFormat("Last: days %d/%d  roads %d (+%d)  zones %d  parks %d  fails %d", rep.daysSimulated,
                      rep.daysRequested, rep.roadsBuilt, rep.roadsUpgraded, rep.zonesBuilt, rep.parksBuilt,
                      rep.failedBuilds),
           uiTh.textDim);

  ui::Text(x0 + 12, by + 16, 12,
           "Console: bot show | bot <key> <value> | autobuild <days> key=value ...",
           uiTh.textDim);
}


void Game::stopPov()
{
  // Restore 2D camera.
  if (m_pov.isActive()) {
    m_pov.stop(m_camera);
  }

  // Restore 3D preview state if POV was driving it.
  if (m_povSaved3DPreviewValid) {
    m_show3DPreview = m_povSavedShow3DPreview;
    m_3dPreviewCfg = m_povSaved3DPreviewCfg;
    m_povSaved3DPreviewValid = false;
    m_pov3DPreviewAccumSec = 0.0f;

    // Force a refresh so the restored camera takes effect.
    m_3dPreviewDirty = true;
    m_3dPreviewTimer = 0.0f;
  }
}


bool Game::startRoamPov(std::uint32_t seed, int lengthTiles)
{
  if (m_world.width() <= 0 || m_world.height() <= 0) {
    return false;
  }

  PovRoamConfig roam = m_povRoamCfg;
  roam.length = std::max(2, lengthTiles);

  // Use selected tile if possible, otherwise use the camera target tile.
  Point hint{m_world.width() / 2, m_world.height() / 2};
  if (m_inspectSelected) {
    hint = *m_inspectSelected;
  } else {
    if (const auto camTile =
            WorldToTileElevated(m_camera.target, m_world, static_cast<float>(m_cfg.tileWidth),
                                static_cast<float>(m_cfg.tileHeight), m_elev)) {
      hint = *camTile;
    }
  }

  std::string debug;
  const std::vector<Point> path = GeneratePovRoamPath(m_world, hint, roam, seed, &debug);
  m_povRoamDebug = debug;
  m_povRoamSeed = seed;

  if (path.size() < 2) {
    return false;
  }

  // Insert lightweight "turn markers" so the ride occasionally breathes at corners.
  std::vector<PovMarker> markers;
  markers.reserve(64);

  int lastMarker = -999999;
  for (int i = 1; i + 1 < static_cast<int>(path.size()); ++i) {
    const Point a = path[static_cast<std::size_t>(i - 1)];
    const Point b = path[static_cast<std::size_t>(i)];
    const Point c = path[static_cast<std::size_t>(i + 1)];
    const Point d1{b.x - a.x, b.y - a.y};
    const Point d2{c.x - b.x, c.y - b.y};

    if (d1.x == d2.x && d1.y == d2.y) {
      continue;
    }

    // Space markers out so we don't stutter on dense intersections.
    if (i - lastMarker < 18) {
      continue;
    }
    lastMarker = i;

    PovMarker mk;
    mk.pathIndex = i;
    mk.holdSec = 0.25f;
    mk.label = "Turn";
    markers.push_back(std::move(mk));
  }

  // Make POV start.
  m_pov.config().loop = true;
  m_pov.setTitle("Roam");

  m_pov.startFromPath(m_world, path, markers, static_cast<float>(m_cfg.tileWidth),
                      static_cast<float>(m_cfg.tileHeight), m_elev, m_camera,
                      GetScreenWidth(), GetScreenHeight(), seed);

  // Optional: start driving the software 3D preview if enabled by the user.
  if (m_povDrive3DPreview) {
    // Save current preview state.
    if (!m_povSaved3DPreviewValid) {
      m_povSavedShow3DPreview = m_show3DPreview;
      m_povSaved3DPreviewCfg = m_3dPreviewCfg;
      m_povSaved3DPreviewValid = true;
    }
    m_show3DPreview = true;
    m_pov3DPreviewAccumSec = 0.0f;
    m_3dPreviewDirty = true;
    m_3dPreviewTimer = 0.0f;
  }

  return true;
}

void Game::updatePov(float dt)
{
  if (!m_pov.isActive()) {
    // If the user toggled 3D driving off while POV isn't active, ensure any saved
    // 3D state is restored.
    if (m_povSaved3DPreviewValid && !m_povDrive3DPreview) {
      m_show3DPreview = m_povSavedShow3DPreview;
      m_3dPreviewCfg = m_povSaved3DPreviewCfg;
      m_povSaved3DPreviewValid = false;
      m_pov3DPreviewAccumSec = 0.0f;
      m_3dPreviewDirty = true;
      m_3dPreviewTimer = 0.0f;
    }
    return;
  }

  const bool ok = m_pov.update(
      dt,
      m_world,
      static_cast<float>(m_cfg.tileWidth),
      static_cast<float>(m_cfg.tileHeight),
      m_elev,
      m_camera,
      GetScreenWidth(),
      GetScreenHeight());

  if (!ok) {
    // Controller stopped itself; also restore any preview override.
    if (m_povSaved3DPreviewValid) {
      m_show3DPreview = m_povSavedShow3DPreview;
      m_3dPreviewCfg = m_povSaved3DPreviewCfg;
      m_povSaved3DPreviewValid = false;
      m_pov3DPreviewAccumSec = 0.0f;
      m_3dPreviewDirty = true;
      m_3dPreviewTimer = 0.0f;
    }
    return;
  }

  updatePov3DPreview(dt);
}

void Game::updatePov3DPreview(float dt)
{
  if (!m_pov.isActive()) {
    return;
  }

  if (!m_povDrive3DPreview) {
    // If we were driving the preview and the user toggled it off mid-ride,
    // restore the old preview settings.
    if (m_povSaved3DPreviewValid) {
      m_show3DPreview = m_povSavedShow3DPreview;
      m_3dPreviewCfg = m_povSaved3DPreviewCfg;
      m_povSaved3DPreviewValid = false;
      m_pov3DPreviewAccumSec = 0.0f;
      m_3dPreviewDirty = true;
      m_3dPreviewTimer = 0.0f;
    }
    return;
  }

  // Save preview state once, so we can restore it on stop.
  if (!m_povSaved3DPreviewValid) {
    m_povSaved3DPreviewValid = true;
    m_povSavedShow3DPreview = m_show3DPreview;
    m_povSaved3DPreviewCfg = m_3dPreviewCfg;
  }

  // Ensure the panel is visible if the user wants 3D POV driving.
  m_show3DPreview = true;

  // Throttle updates: the software renderer is expensive.
  constexpr float kPov3DPreviewFps = 8.0f;
  const float interval = 1.0f / kPov3DPreviewFps;
  m_pov3DPreviewAccumSec += dt;
  if (m_pov3DPreviewAccumSec < interval) {
    return;
  }
  m_pov3DPreviewAccumSec = 0.0f;

  Vector2 posTiles{0, 0};
  Vector2 dirTiles{1, 0};
  if (!m_pov.getTilePose(posTiles, dirTiles)) {
    return;
  }

  // --- Convert tile-space pose -> orbit camera (Soft3D) ---
  const float tileSize = m_3dPreviewCfg.meshCfg.tileSize;
  const float heightScale = m_3dPreviewCfg.meshCfg.heightScale;

  const int tx = std::clamp(static_cast<int>(std::floor(posTiles.x)), 0, m_world.width() - 1);
  const int ty = std::clamp(static_cast<int>(std::floor(posTiles.y)), 0, m_world.height() - 1);
  const float groundY = m_world.at(tx, ty).height * heightScale;

  // Eye is slightly above the road surface.
  const float eyeH = 1.35f * tileSize;
  const float lookAhead = std::max(1.5f * tileSize, m_pov.config().lookAheadTiles * tileSize);
  const float lookDown = 0.45f * tileSize;

  const float eyeX = posTiles.x * tileSize;
  const float eyeZ = posTiles.y * tileSize;
  const float eyeY = groundY + eyeH;

  // Normalize direction in tile space.
  float dl = std::sqrt(dirTiles.x * dirTiles.x + dirTiles.y * dirTiles.y);
  float dx = (dl > 1e-3f) ? (dirTiles.x / dl) : 1.0f;
  float dz = (dl > 1e-3f) ? (dirTiles.y / dl) : 0.0f;

  const float targetX = eyeX + dx * lookAhead;
  const float targetZ = eyeZ + dz * lookAhead;
  const float targetY = eyeY - lookDown;

  const float offX = eyeX - targetX;
  const float offY = eyeY - targetY;
  const float offZ = eyeZ - targetZ;
  const float dist = std::max(0.1f, std::sqrt(offX * offX + offY * offY + offZ * offZ));

  const float odx = offX / dist;
  const float ody = offY / dist;
  const float odz = offZ / dist;

  const float yawRad = std::atan2(odz, odx);
  const float pitchRad = std::asin(std::clamp(ody, -1.0f, 1.0f));

  m_3dPreviewCfg.projection = Render3DConfig::Projection::Perspective;
  m_3dPreviewCfg.autoFit = false;
  m_3dPreviewCfg.fovYDeg = 75.0f;
  m_3dPreviewCfg.targetX = targetX;
  m_3dPreviewCfg.targetY = targetY;
  m_3dPreviewCfg.targetZ = targetZ;
  m_3dPreviewCfg.distance = dist;
  m_3dPreviewCfg.yawDeg = yawRad * (180.0f / 3.1415926535f);
  m_3dPreviewCfg.pitchDeg = pitchRad * (180.0f / 3.1415926535f);
  m_3dPreviewCfg.rollDeg = 0.0f;

  m_3dPreviewDirty = true;
  m_3dPreviewTimer = 0.0f;
}

void Game::handleInput(float dt)
{
  // Keep UI scaling in sync with monitor DPI and any window resizes.
  updateUiScaleHotkeys();
  updateWorldRenderHotkeys();

  const int screenW = GetScreenWidth();
  const int screenH = GetScreenHeight();
  const float uiScale = m_uiScale;
  const int uiW = static_cast<int>(std::round(static_cast<float>(screenW) / uiScale));
  const int uiH = static_cast<int>(std::round(static_cast<float>(screenH) / uiScale));

  const Vector2 mouse = GetMousePosition();
  const Vector2 mouseUi = mouseUiPosition(uiScale);

  // Update hovered tile from mouse.
  const Vector2 mouseWorld = GetScreenToWorld2D(mouse, m_camera);
  m_hovered = WorldToTileElevated(mouseWorld, m_world, static_cast<float>(m_cfg.tileWidth),
                                 static_cast<float>(m_cfg.tileHeight), m_elev);

  // Undo/redo
  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  // Fullscreen/borderless toggles (common PC shortcuts).
  if (IsKeyPressed(KEY_F11)) {
    if (ctrl) {
      // Ctrl+F11: queue a software 3D render export (CPU renderer).
      namespace fs = std::filesystem;
      fs::create_directories("captures");

      Render3DConfig cfg = m_pendingRender3DCfg;
      // Ctrl+Shift+F11 requests a higher-quality export (SSAA bump).
      if (shift) cfg.supersample = std::clamp(std::max(2, cfg.supersample), 1, 4);

      const std::string path =
          TextFormat("captures/render3d_seed%llu_%s.png", static_cast<unsigned long long>(m_cfg.seed),
                     FileTimestamp().c_str());

      m_pendingRender3D = true;
      m_pendingRender3DPath = path;
      m_pendingRender3DCfg = cfg;
      // Keep current defaults for layer/grade.
      showToast(TextFormat("Queued 3D render (%dx%d ssaa=%d): %s", cfg.width, cfg.height, cfg.supersample,
                           path.c_str()),
                2.5f);
    } else if (shift) {
      // Shift+F11: toggle the in-game 3D preview panel.
      m_show3DPreview = !m_show3DPreview;
      m_3dPreviewDirty = true;
      m_3dPreviewTimer = 0.0f;
      showToast(m_show3DPreview ? "3D preview: ON" : "3D preview: OFF", 2.0f);
    } else {
      toggleFullscreen();
    }
  }
  if ((IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) && IsKeyPressed(KEY_ENTER)) {
    toggleBorderlessWindowed();
  }

  // 3D preview camera nudge controls (hold Ctrl while the preview panel is open).
  // Keeping this modifier-heavy prevents accidental conflicts with normal play.
  if (m_show3DPreview && ctrl && !m_showVideoSettings && !m_showSaveMenu && !m_console.isOpen()) {
    bool changed = false;
    const float stepYaw = shift ? 12.0f : 4.0f;
    const float stepPitch = shift ? 8.0f : 3.0f;

    if (IsKeyPressed(KEY_LEFT)) {
      m_3dPreviewCfg.yawDeg -= stepYaw;
      changed = true;
    }
    if (IsKeyPressed(KEY_RIGHT)) {
      m_3dPreviewCfg.yawDeg += stepYaw;
      changed = true;
    }
    if (IsKeyPressed(KEY_UP)) {
      m_3dPreviewCfg.pitchDeg = std::clamp(m_3dPreviewCfg.pitchDeg + stepPitch, 10.0f, 80.0f);
      changed = true;
    }
    if (IsKeyPressed(KEY_DOWN)) {
      m_3dPreviewCfg.pitchDeg = std::clamp(m_3dPreviewCfg.pitchDeg - stepPitch, 10.0f, 80.0f);
      changed = true;
    }
    if (IsKeyPressed(KEY_R)) {
      m_3dPreviewCfg.yawDeg = 45.0f;
      m_3dPreviewCfg.pitchDeg = 35.264f;
      m_3dPreviewCfg.projection = Render3DConfig::Projection::IsometricOrtho;
      m_3dPreviewCfg.autoFit = true;
      changed = true;
    }
    if (IsKeyPressed(KEY_P)) {
      // Toggle projection between isometric orthographic and a mild perspective.
      m_3dPreviewCfg.projection =
        (m_3dPreviewCfg.projection == Render3DConfig::Projection::IsometricOrtho)
          ? Render3DConfig::Projection::Perspective
          : Render3DConfig::Projection::IsometricOrtho;
      changed = true;
    }

    if (changed) {
      m_3dPreviewDirty = true;
      m_3dPreviewTimer = 0.0f;
    }
  }

  // Developer console (toggle with F4). When open it captures keyboard input.
  if (IsKeyPressed(KEY_F4)) {
    endPaintStroke();

    // Avoid overlapping input-capturing UIs.
    if (!m_console.isOpen() && m_showSaveMenu) {
      unloadSaveMenuThumbnails();
      m_showSaveMenu = false;
      m_saveMenuDeleteArmed = false;
    }

    m_console.toggle();
    showToast(m_console.isOpen() ? "Console: ON" : "Console: OFF");
  }

  if (m_console.isOpen()) {
    (void)m_console.update(dt, uiW, uiH, mouseUi.x, mouseUi.y);
    return;
  }

  if (ctrl && shift && IsKeyPressed(KEY_Z)) {
    doRedo();
  } else if (ctrl && IsKeyPressed(KEY_Z)) {
    doUndo();
  } else if (ctrl && IsKeyPressed(KEY_Y)) {
    doRedo();
  }

  // Save manager UI (toggle with F10). When open, it captures most input.
  if (IsKeyPressed(KEY_F10)) {
    endPaintStroke();
    m_showSaveMenu = !m_showSaveMenu;
    if (m_showSaveMenu) {
      m_saveMenuDeleteArmed = false;
      m_saveMenuRefreshTimer = 0.0f;
      refreshSaveMenu();
      showToast("Save menu: ON");
    } else {
      unloadSaveMenuThumbnails();
      m_saveMenuDeleteArmed = false;
      showToast("Save menu: OFF");
    }
  }

  // Capture controls
  // - F12: window screenshot
  // - Ctrl+F12: full city overview export (off-screen render)
  if (IsKeyPressed(KEY_F12)) {
    endPaintStroke();

    std::error_code ec;
    const std::filesystem::path outDir = std::filesystem::path("captures");
    std::filesystem::create_directories(outDir, ec);

    const std::string stamp = FileTimestamp();
    const unsigned long long seed = static_cast<unsigned long long>(m_world.seed());
    const int day = m_world.stats().day;

    auto makeFileName = [&](const char* prefix) {
      char buf[256];
      std::snprintf(buf, sizeof(buf), "%s_seed%llu_day%d_%s.png", prefix, seed, day, stamp.c_str());
      return outDir / buf;
    };

    if (ctrl) {
      if (shift) {
        const std::filesystem::path prefixPath = makeFileName("maplayers");
        m_pendingMapLayersExport = true;
        m_pendingMapLayersPrefix = prefixPath.string();
        m_pendingMapLayersMaxSize = 4096;
        showToast(std::string("Queued layer export: ") + prefixPath.string(), 2.5f);
      } else {
        const std::filesystem::path outPath = makeFileName("map");
        m_pendingMapExport = true;
        m_pendingMapExportPath = outPath.string();
        m_pendingMapExportMaxSize = 4096;
        showToast(std::string("Queued map export: ") + outPath.string(), 2.5f);
      }
    } else {
      // Queue the screenshot so it's captured after the frame is drawn.
      m_pendingScreenshotPath = makeFileName("screenshot").string();
      m_pendingScreenshot = true;
      showToast(std::string("Queued screenshot: ") + m_pendingScreenshotPath, 2.5f);
    }
  }

  if (m_showSaveMenu) {
    // Group switch.
    if (IsKeyPressed(KEY_TAB)) {
      m_saveMenuGroup = (m_saveMenuGroup == 0) ? 1 : 0;
      m_saveMenuSelection = 0;
      m_saveMenuDeleteArmed = false;
    }

    const std::vector<SaveMenuSlot>& list = (m_saveMenuGroup == 0) ? m_saveMenuManual : m_saveMenuAutos;
    const int count = static_cast<int>(list.size());

    if (IsKeyPressed(KEY_UP)) m_saveMenuSelection = std::max(0, m_saveMenuSelection - 1);
    if (IsKeyPressed(KEY_DOWN)) m_saveMenuSelection = std::min(std::max(0, count - 1), m_saveMenuSelection + 1);

    if (count > 0) {
      const int idx = std::clamp(m_saveMenuSelection, 0, count - 1);
      const SaveMenuSlot& e = list[static_cast<std::size_t>(idx)];

      // Load selected (Enter or F9).
      if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER) || IsKeyPressed(KEY_F9)) {
        if (e.exists) {
          const char* label = e.autosave ? TextFormat("Autosave %d", e.slot) : TextFormat("Slot %d", e.slot);
          loadFromPath(e.path, label);
        } else {
          showToast("No save in that slot", 2.0f);
        }
      }

      // Save into selected manual slot (F5).
      if (IsKeyPressed(KEY_F5)) {
        if (!e.autosave) {
          m_saveSlot = e.slot;
          saveToPath(e.path, true, TextFormat("Slot %d", e.slot));
        } else {
          showToast("Autosaves are read-only", 2.0f);
        }
      }

      // Delete selected (Del twice to confirm).
      if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
        if (!e.exists) {
          showToast("Slot is already empty", 2.0f);
        } else if (!m_saveMenuDeleteArmed) {
          m_saveMenuDeleteArmed = true;
          m_saveMenuDeleteTimer = 1.5f;
          showToast("Press Del again to delete", 1.5f);
        } else {
          namespace fs = std::filesystem;
          std::error_code ec;
          fs::remove(fs::path(e.path), ec);
          const std::string tp = thumbPathForSavePath(e.path);
          fs::remove(fs::path(tp), ec);
          m_saveMenuDeleteArmed = false;
          refreshSaveMenu();
          showToast("Deleted save", 1.5f);
        }
      }
    }

    // While the save menu is open we don't want other gameplay inputs to fire.
    return;
  }

  // Simulation controls
  auto simSpeed = [&]() -> float {
    const int si = std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1);
    return kSimSpeeds[static_cast<std::size_t>(si)];
  };

  if (IsKeyPressed(KEY_SPACE)) {
    endPaintStroke();
    m_simPausedUser = !m_simPausedUser;
    m_sim.resetTimer();
    const bool paused = simPaused();
    // If we're paused due to focus/minimize, keep messaging clear.
    if (paused && !m_simPausedUser && m_simPausedAuto) showToast("Sim paused (unfocused)");
    else showToast(paused ? "Sim paused" : "Sim running");
  }

  if (simPaused() && IsKeyPressed(KEY_N)) {
    endPaintStroke();

    if (m_replayCapture.active()) {
      std::string err;
      (void)m_replayCapture.captureSettingsIfChanged(m_world, m_procCfg, m_sim, err);
    }

    m_sim.stepOnce(m_world);

    if (m_replayCapture.active()) {
      m_replayCapture.recordTicks(1);
    }
    recordHistorySample(m_world.stats());
    m_trafficDirty = true;
    m_goodsDirty = true;
    m_landValueDirty = true;
    m_vehiclesDirty = true;
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
    m_evacDirty = true;
    m_roadUpgradePlanDirty = true;
    m_roadUpgradeSelectedMaskDirty = true;
    invalidateAnalysisLayers();
    showToast("Sim step");
  }

  if (IsKeyPressed(KEY_KP_ADD) || (IsKeyPressed(KEY_EQUAL) && shift)) {
    const int before = m_simSpeedIndex;
    m_simSpeedIndex = std::clamp(m_simSpeedIndex + 1, 0, kSimSpeedCount - 1);
    if (m_simSpeedIndex != before) {
      m_sim.resetTimer();
      showToast(TextFormat("Sim speed: x%.2f", static_cast<double>(simSpeed())));
    }
  }

  if (IsKeyPressed(KEY_KP_SUBTRACT) || IsKeyPressed(KEY_MINUS)) {
    const int before = m_simSpeedIndex;
    m_simSpeedIndex = std::clamp(m_simSpeedIndex - 1, 0, kSimSpeedCount - 1);
    if (m_simSpeedIndex != before) {
      m_sim.resetTimer();
      showToast(TextFormat("Sim speed: x%.2f", static_cast<double>(simSpeed())));
    }
  }

  // Toggle UI
  if (IsKeyPressed(KEY_H)) m_showHelp = !m_showHelp;
  if (IsKeyPressed(KEY_G)) m_drawGrid = !m_drawGrid;

  if (IsKeyPressed(KEY_F1)) {
    m_showReport = !m_showReport;
    showToast(m_showReport ? "City report: ON" : "City report: OFF");
  }

  if (IsKeyPressed(KEY_F2)) {
    if (ctrl) {
      // Ctrl+F2 toggles the City Lab panel. Ctrl+Shift+F2 opens it directly on the Mesh tab.
      if (shift) {
        m_showHeadlessLab = true;
        m_headlessLabTab = 2; // Mesh
        ui::ClearActiveWidget();
        showToast("City Lab: Mesh", 1.5f);
      } else {
        m_showHeadlessLab = !m_showHeadlessLab;
        ui::ClearActiveWidget();
        showToast(m_showHeadlessLab ? "City Lab: ON" : "City Lab: OFF");
      }
    } else if (shift) {
      const bool enabled = !m_renderer.dayNightEnabled();
      m_renderer.setDayNightEnabled(enabled);
      showToast(enabled ? "Day/night lighting: ON" : "Day/night lighting: OFF");
    } else {
      const bool enabled = !m_renderer.baseCacheEnabled();
      m_renderer.setBaseCacheEnabled(enabled);
      m_renderer.markBaseCacheDirtyAll();
      showToast(enabled ? "Render cache: ON" : "Render cache: OFF");
    }
  }

  if (IsKeyPressed(KEY_I)) {
    m_mergedZoneBuildings = !m_mergedZoneBuildings;
    m_cfg.mergedZoneBuildings = m_mergedZoneBuildings;
    showToast(std::string("Merged zone buildings: ") + (m_mergedZoneBuildings ? "ON" : "OFF"));
  }

  if (IsKeyPressed(KEY_F3)) {
    // Ctrl+F3: perf overlay. Ctrl+Shift+F3: sim tick limiter mode (auto vs fixed).
    if (ctrl && shift) {
      m_loopCfg.simTickLimiter = true;
      m_loopCfg.simTickAuto = !m_loopCfg.simTickAuto;
      showToast(std::string("Sim tick limiter: ") + (m_loopCfg.simTickAuto ? "AUTO" : "FIXED"));
    } else if (ctrl) {
      m_showPerfOverlay = !m_showPerfOverlay;
      showToast(m_showPerfOverlay ? "Perf overlay: ON" : "Perf overlay: OFF");
    } else if (shift) {
      auto s = m_renderer.weatherSettings();
      using M = Renderer::WeatherSettings::Mode;

      if (s.mode == M::Clear) s.mode = M::Rain;
      else if (s.mode == M::Rain) s.mode = M::Snow;
      else s.mode = M::Clear;

      // Ensure intensity isn't accidentally near-zero when enabling.
      if (s.mode != M::Clear && s.intensity < 0.05f) s.intensity = 0.80f;

      m_renderer.setWeatherSettings(s);
      showToast(std::string("Weather: ") + ((s.mode == M::Rain) ? "Rain" : (s.mode == M::Snow) ? "Snow" : "Clear"));
    } else {
      m_showTrafficModel = !m_showTrafficModel;
      showToast(m_showTrafficModel ? "Traffic model: ON" : "Traffic model: OFF");
    }
    endPaintStroke();
  }

  if (IsKeyPressed(KEY_F7)) {
    m_showDistrictPanel = !m_showDistrictPanel;
    showToast(m_showDistrictPanel ? "Districts panel: ON" : "Districts panel: OFF");
    endPaintStroke();
  }

  if (IsKeyPressed(KEY_F8)) {
    endPaintStroke();

    // Video settings is one of the few places we allow click/drag UI widgets,
    // so ensure we don't leave a stale drag active when toggling pages.
    ui::ClearActiveWidget();

    if (!m_showVideoSettings) {
      m_showVideoSettings = true;
      m_showHelp = false;

      // Shift+F8 opens directly on the Visual FX page.
      // Ctrl+F8 opens directly on the UI Theme page.
      // Ctrl+Shift+F8 opens directly on the Atmosphere page.
      if (ctrl && shift) m_videoPage = 2;
      else if (ctrl) m_videoPage = 3;
      else if (shift) m_videoPage = 1;
      else m_videoPage = 0;

      if (m_videoPage == 0) m_videoSelection = m_videoSelectionDisplay;
      else if (m_videoPage == 1) m_videoSelection = m_videoSelectionVisual;
      else if (m_videoPage == 2) m_videoSelection = m_videoSelectionAtmos;
      else m_videoSelection = m_videoSelectionUiTheme;

      showToast((m_videoPage == 0) ? "Video settings: Display"
                                  : (m_videoPage == 1) ? "Video settings: Visual FX"
                                  : (m_videoPage == 2) ? "Video settings: Atmosphere"
                                                      : "Video settings: UI Theme");
    } else {
      // When the panel is already open:
      //  - F8 closes it.
      //  - Shift+F8 cycles pages without closing.
      //  - Ctrl+F8 jumps to UI Theme (toggles back to Display if already there).
      if (shift || ctrl) {
        if (m_videoPage == 0) m_videoSelectionDisplay = m_videoSelection;
        else if (m_videoPage == 1) m_videoSelectionVisual = m_videoSelection;
        else if (m_videoPage == 2) m_videoSelectionAtmos = m_videoSelection;
        else m_videoSelectionUiTheme = m_videoSelection;

        if (ctrl && shift) {
          m_videoPage = 2;
        } else if (ctrl) {
          m_videoPage = (m_videoPage == 3) ? 0 : 3;
        } else {
          m_videoPage = (m_videoPage + 1) % 4;
        }

        if (m_videoPage == 0) m_videoSelection = m_videoSelectionDisplay;
        else if (m_videoPage == 1) m_videoSelection = m_videoSelectionVisual;
        else if (m_videoPage == 2) m_videoSelection = m_videoSelectionAtmos;
        else m_videoSelection = m_videoSelectionUiTheme;

        showToast((m_videoPage == 0) ? "Video settings: Display"
                                    : (m_videoPage == 1) ? "Video settings: Visual FX"
                                    : (m_videoPage == 2) ? "Video settings: Atmosphere"
                                                        : "Video settings: UI Theme");
      } else {
        if (m_videoPage == 0) m_videoSelectionDisplay = m_videoSelection;
        else if (m_videoPage == 1) m_videoSelectionVisual = m_videoSelection;
        else if (m_videoPage == 2) m_videoSelectionAtmos = m_videoSelection;
        else m_videoSelectionUiTheme = m_videoSelection;

        m_showVideoSettings = false;
        showToast("Video settings: OFF");
      }
    }
  }

  if (IsKeyPressed(KEY_P) && !ctrl && !alt && !m_show3DPreview) {
    m_showPolicy = !m_showPolicy;
    showToast(m_showPolicy ? "Policy: ON" : "Policy: OFF");
  }


  if (IsKeyPressed(KEY_TAB)) {
    // Hold Shift to cycle backwards.
    const int delta = shift ? -1 : 1;

    if (m_showReport) {
      constexpr int kPages = 5;
      m_reportPage = (m_reportPage + delta + kPages) % kPages;
    } else if (m_showPolicy) {
      const int count = 13;
      m_policySelection = (m_policySelection + delta + count) % count;
    } else if (m_showNewsPanel) {
      const int count = static_cast<int>(m_cityNews.size());
      if (count > 0) {
        m_newsSelection = (m_newsSelection + delta + count) % count;
      }
    } else if (m_showChallengesPanel) {
      const int count = static_cast<int>(m_cityChallenges.size());
      if (count > 0) {
        m_challengeSelection = (m_challengeSelection + delta + count) % count;
      }
    } else if (m_showTrafficModel) {
      const int count = 9;
      m_trafficModelSelection = (m_trafficModelSelection + delta + count) % count;
    } else if (m_showResiliencePanel) {
      const int count = 14;
      m_resilienceSelection = (m_resilienceSelection + delta + count) % count;
    } else if (m_showDistrictPanel) {
      const int count = 9;
      m_districtSelection = (m_districtSelection + delta + count) % count;
    } else if (m_showServicesPanel) {
      const int count = 10;
      m_servicesSelection = (m_servicesSelection + delta + count) % count;
    } else if (m_showTransitPanel) {
      const int count = 19;
      m_transitSelection = (m_transitSelection + delta + count) % count;
    } else if (m_showRoadUpgradePanel) {
      const int count = 13;
      m_roadUpgradeSelection = (m_roadUpgradeSelection + delta + count) % count;
    } else if (m_showVideoSettings) {
      const int count = (m_videoPage == 0)   ? 11
                        : (m_videoPage == 1) ? 57
                        : (m_videoPage == 2) ? 19
                                             : 11;
      m_videoSelection = (m_videoSelection + delta + count) % count;
      if (m_videoPage == 0) m_videoSelectionDisplay = m_videoSelection;
      else if (m_videoPage == 1) m_videoSelectionVisual = m_videoSelection;
      else if (m_videoPage == 2) m_videoSelectionAtmos = m_videoSelection;
      else m_videoSelectionUiTheme = m_videoSelection;
    }
  }

  // City Challenges: keyboard shortcuts while the panel is open.
  if (m_showChallengesPanel && m_blueprintMode == BlueprintMode::Off) {
    // Avoid fighting with other "Enter" panels (transit/upgrades) when multiple docks are visible.
    const bool enterOk = !(m_showTransitPanel || m_showRoadUpgradePanel);
    if (enterOk && (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))) {
      endPaintStroke();
      if (!m_cityChallenges.empty()) {
        m_challengeSelection = std::clamp(m_challengeSelection, 0, std::max(0, static_cast<int>(m_cityChallenges.size()) - 1));
        focusChallenge(m_cityChallenges[static_cast<std::size_t>(m_challengeSelection)]);
      }
    }
    if (IsKeyPressed(KEY_DELETE)) {
      endPaintStroke();
      if (!m_cityChallenges.empty()) {
        m_challengeSelection = std::clamp(m_challengeSelection, 0, std::max(0, static_cast<int>(m_cityChallenges.size()) - 1));
        (void)rerollChallenge(static_cast<std::size_t>(m_challengeSelection));
      }
    }
  }

  // Transit panel: Enter replans with the current settings.
  if (m_showTransitPanel && m_blueprintMode == BlueprintMode::Off &&
      (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))) {
    m_transitPlanDirty = true;
    m_transitVizDirty = true;
  m_evacDirty = true;
    ensureTransitVizUpToDate();

    const int lineCount = static_cast<int>(m_transitPlan.lines.size());
    const double cov = (m_transitPlan.totalDemand > 0)
                           ? (100.0 * static_cast<double>(m_transitPlan.coveredDemand) /
                              static_cast<double>(m_transitPlan.totalDemand))
                           : 0.0;
    showToast(TextFormat("Transit replanned (%d lines, %.0f%% coverage)", lineCount, cov));
  }

  // Road upgrade panel: Enter replans, Ctrl+Enter applies the upgrades.
  if (m_showRoadUpgradePanel && m_blueprintMode == BlueprintMode::Off &&
      (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER))) {
    if (ctrl) {
      applyRoadUpgradePlan();
    } else {
      m_roadUpgradePlanDirty = true;
      ensureRoadUpgradePlanUpToDate();

      const int edgeCount = static_cast<int>(m_roadUpgradePlan.edges.size());
      showToast(TextFormat("Upgrades replanned (%d edges)", edgeCount));
    }
  }

  if (IsKeyPressed(KEY_M)) {
    m_showMinimap = !m_showMinimap;
    if (m_showMinimap) m_renderer.markMinimapDirty();
    showToast(m_showMinimap ? "Minimap: ON" : "Minimap: OFF");
  }

  if (IsKeyPressed(KEY_C)) {
    m_showVehicles = !m_showVehicles;
    m_vehiclesDirty = true;
    if (!m_showVehicles) m_vehicles.clear();
    showToast(m_showVehicles ? "Vehicles: ON" : "Vehicles: OFF");
  }

  // Toggle elevation rendering (flat <-> elevated). This is purely visual; terraforming is separate.
  if (IsKeyPressed(KEY_E)) {
    endPaintStroke();
    if (m_elev.maxPixels > 0.0f) {
      m_elev.maxPixels = 0.0f;
      showToast("Elevation: OFF");
    } else {
      m_elev = m_elevDefault;
      showToast(TextFormat("Elevation: ON (max %.0fpx)", static_cast<double>(m_elev.maxPixels)));
    }
    m_renderer.setElevationSettings(m_elev);
  }
  if (IsKeyPressed(KEY_O) && !ctrl) {
    m_showOutsideOverlay = !m_showOutsideOverlay;
    showToast(m_showOutsideOverlay ? "Outside overlay: ON" : "Outside overlay: OFF");
  }

  // City Challenges panel: Ctrl+O (keeps plain 'O' reserved for the outside overlay).
  if (ctrl && IsKeyPressed(KEY_O)) {
    endPaintStroke();
    m_showChallengesPanel = !m_showChallengesPanel;
    showToast(m_showChallengesPanel ? "City challenges: ON" : "City challenges: OFF");
  }

  // City Treasury panel (municipal bonds): Ctrl+B.
  if (ctrl && IsKeyPressed(KEY_B)) {
    endPaintStroke();
    m_showBondsPanel = !m_showBondsPanel;
    showToast(m_showBondsPanel ? "City treasury: ON" : "City treasury: OFF");
  }

  if (IsKeyPressed(KEY_T)) {
    // Ctrl+T toggles transit panel; Ctrl+Shift+T exports transit artifacts.
    // Shift+T toggles road resilience; plain T toggles road graph.
    if (ctrl) {
      endPaintStroke();
      if (shift) {
        exportTransitArtifacts();
      } else {
        m_showTransitPanel = !m_showTransitPanel;
        if (m_showTransitPanel) {
          ensureTransitVizUpToDate();
          const int lineCount = static_cast<int>(m_transitPlan.lines.size());
          const double cov = (m_transitPlan.totalDemand > 0)
                                 ? (100.0 * static_cast<double>(m_transitPlan.coveredDemand) /
                                    static_cast<double>(m_transitPlan.totalDemand))
                                 : 0.0;
          showToast(TextFormat("Transit: ON (%d lines, %.0f%% coverage)", lineCount, cov));
        } else {
          showToast("Transit: OFF");
        }
      }
    } else if (shift) {
      m_showResilienceOverlay = !m_showResilienceOverlay;
      if (m_showResilienceOverlay) {
        ensureRoadResilienceUpToDate();
        m_resilienceBypassesDirty = true;
        rebuildRoadResilienceBypasses();
        showToast(TextFormat("Resilience: ON (%d bridges, %d articulations)",
                             static_cast<int>(m_roadResilience.bridgeEdges.size()),
                             static_cast<int>(m_roadResilience.articulationNodes.size())));
      } else {
        showToast("Resilience: OFF");
      }
    } else {
      m_showRoadGraphOverlay = !m_showRoadGraphOverlay;
      if (m_showRoadGraphOverlay) {
        ensureRoadGraphUpToDate();
        showToast(TextFormat("Road graph: ON (%d nodes, %d edges)",
                             static_cast<int>(m_roadGraph.nodes.size()),
                             static_cast<int>(m_roadGraph.edges.size())));
      } else {
        showToast("Road graph: OFF");
      }
    }
  }



  if (IsKeyPressed(KEY_N) && ctrl) {
    // Ctrl+N toggles City News; Ctrl+Shift+N exports the log.
    endPaintStroke();
    if (shift) {
      exportCityNews();
    } else {
      m_showNewsPanel = !m_showNewsPanel;
      showToast(m_showNewsPanel ? "City News: ON" : "City News: OFF");
    }
  }


  if (IsKeyPressed(KEY_X) && ctrl) {
    // Ctrl+X toggles Services planner; Ctrl+Shift+X exports the current plan.
    endPaintStroke();
    if (shift) {
      exportServicesArtifacts();
    } else {
      m_showServicesPanel = !m_showServicesPanel;
      if (m_showServicesPanel) {
        ensureServicesPlanUpToDate();
        const int n = m_servicesPlanValid ? static_cast<int>(m_servicesPlan.placements.size()) : 0;
        showToast(TextFormat("Services planner: ON (%s, %d suggestions)", ServiceTypeName(m_servicesPlanCfg.type), n));
      } else {
        showToast("Services planner: OFF");
      }
    }
  }

  if (IsKeyPressed(KEY_U) && ctrl) {
    endPaintStroke();
    if (shift) {
      exportRoadUpgradeArtifacts();
    } else {
      m_showRoadUpgradePanel = !m_showRoadUpgradePanel;
      if (m_showRoadUpgradePanel) {
        ensureRoadUpgradePlanUpToDate();
        const int edgeCount = static_cast<int>(m_roadUpgradePlan.edges.size());
        showToast(TextFormat("Road upgrades: ON (%d edges)", edgeCount));
      } else {
        showToast("Road upgrades: OFF");
      }

    }
  }

  if (IsKeyPressed(KEY_V)) {
    m_showTrafficOverlay = !m_showTrafficOverlay;
    m_trafficDirty = true;

    if (m_showTrafficOverlay) {
      const float share = (m_world.stats().population > 0)
                              ? (static_cast<float>(m_world.stats().employed) /
                                 static_cast<float>(m_world.stats().population))
                              : 0.0f;

      TrafficConfig tc;
      tc.requireOutsideConnection = m_sim.config().requireOutsideConnection;
      {
        const TrafficModelSettings& trafficModel = m_sim.trafficModel();
        tc.congestionAwareRouting = trafficModel.congestionAwareRouting;
        tc.congestionIterations = trafficModel.congestionIterations;
        tc.congestionAlpha = trafficModel.congestionAlpha;
        tc.congestionBeta = trafficModel.congestionBeta;
        tc.congestionCapacityScale = trafficModel.congestionCapacityScale;
        tc.congestionRatioClamp = trafficModel.congestionRatioClamp;
        tc.capacityAwareJobs = trafficModel.capacityAwareJobs;
        tc.jobAssignmentIterations = trafficModel.jobAssignmentIterations;
        tc.jobPenaltyBaseMilli = trafficModel.jobPenaltyBaseMilli;
      }

      // Traffic overlay should respect the sim's outside-connection rule even
      // if the connectivity overlay itself is not being drawn.
      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* pre = nullptr;
      if (tc.requireOutsideConnection) {
        ComputeRoadsConnectedToEdge(m_world, roadToEdge);
        pre = &roadToEdge;
      }

      m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
      m_trafficDirty = false;
      m_carbonDirty = true;

      showToast(TextFormat(
          "Traffic overlay: ON (%d commuters, avg %.1f (t %.1f), cong %.0f%%, %s x%d)", m_traffic.totalCommuters,
          static_cast<double>(m_traffic.avgCommute), static_cast<double>(m_traffic.avgCommuteTime),
          static_cast<double>(m_traffic.congestion * 100.0f),
          m_traffic.usedCongestionAwareRouting ? "cong" : "free", m_traffic.routingPasses));
    } else {
      showToast("Traffic overlay: OFF");
    }
  }


  if (!ctrl && !alt && IsKeyPressed(KEY_B)) {
    m_showGoodsOverlay = !m_showGoodsOverlay;
    m_goodsDirty = true;

    if (m_showGoodsOverlay) {
      GoodsConfig gc;
      gc.requireOutsideConnection = m_sim.config().requireOutsideConnection;

      // Reflect the trade market's current day capacities so the overlay matches
      // the active simulation rules.
      const Stats& st = m_world.stats();
      const TradeModelSettings& ts = m_sim.tradeModel();
      if (ts.enabled) {
        gc.allowImports = ts.allowImports;
        gc.allowExports = ts.allowExports;
        gc.importCapacityPct = std::clamp(st.tradeImportCapacityPct, 0, 100);
        gc.exportCapacityPct = std::clamp(st.tradeExportCapacityPct, 0, 100);
      }

      // Goods overlay should respect the sim's outside-connection rule even
      // if the connectivity overlay itself is not being drawn.
      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* pre = nullptr;
      if (gc.requireOutsideConnection) {
        ComputeRoadsConnectedToEdge(m_world, roadToEdge);
        pre = &roadToEdge;
      }

      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;
      m_carbonDirty = true;

      showToast(TextFormat("Goods overlay: ON (deliv %d/%d, sat %.0f%%, imp %d, exp %d)",
                           m_goods.goodsDelivered, m_goods.goodsDemand,
                           static_cast<double>(m_goods.satisfaction * 100.0f),
                           m_goods.goodsImported, m_goods.goodsExported));
    } else {
      showToast("Goods overlay: OFF");
    }
  }

  if (IsKeyPressed(KEY_F)) {
    endPaintStroke();
    m_showResiliencePanel = !m_showResiliencePanel;
    showToast(m_showResiliencePanel ? "Resilience panel: ON" : "Resilience panel: OFF");
  }

  // Heatmap overlay: cycle through all built-in and analysis heatmaps.
  if (IsKeyPressed(KEY_L)) {
    auto labelOf = [&](HeatmapOverlay m) -> const char* {
      switch (m) {
      case HeatmapOverlay::Off: return "OFF";
      case HeatmapOverlay::LandValue: return "Land value";
      case HeatmapOverlay::ParkAmenity: return "Park amenity";
      case HeatmapOverlay::WaterAmenity: return "Water amenity";
      case HeatmapOverlay::Pollution: return "Pollution";
      case HeatmapOverlay::TrafficSpill: return "Traffic spill";
      case HeatmapOverlay::ServicesOverall: return "Services";
      case HeatmapOverlay::ServicesEducation: return "Services (edu)";
      case HeatmapOverlay::ServicesHealth: return "Services (health)";
      case HeatmapOverlay::ServicesSafety: return "Services (safety)";
      case HeatmapOverlay::FireRisk: return "Fire risk";
      case HeatmapOverlay::FloodDepth: return "Flood depth";
      case HeatmapOverlay::PondingDepth: return "Ponding depth";
      case HeatmapOverlay::EvacuationTime: return "Evac time";
      case HeatmapOverlay::EvacuationUnreachable: return "Evac unreachable";
      case HeatmapOverlay::EvacuationFlow: return "Evac flow";

      case HeatmapOverlay::AirPollution: return "Air pollution";
      case HeatmapOverlay::AirPollutionEmission: return "Air emission";
      case HeatmapOverlay::SkyView: return "Sky view";
      case HeatmapOverlay::CanyonConfinement: return "Canyon confinement";
      case HeatmapOverlay::SolarExposure: return "Solar exposure";
      case HeatmapOverlay::SolarPotential: return "Solar potential";
      case HeatmapOverlay::EnergyDemand: return "Energy demand";
      case HeatmapOverlay::EnergySolar: return "Rooftop solar";
      case HeatmapOverlay::EnergyBalance: return "Energy balance";
      case HeatmapOverlay::CarbonEmission: return "Carbon emission";
      case HeatmapOverlay::CarbonSequestration: return "Carbon sequestration";
      case HeatmapOverlay::CarbonBalance: return "Carbon balance";

      case HeatmapOverlay::CrimeRisk: return "Crime risk";
      case HeatmapOverlay::PoliceAccess: return "Police access";

      case HeatmapOverlay::TrafficCrashRisk: return "Traffic crash risk";
      case HeatmapOverlay::TrafficCrashExposure: return "Traffic crash exposure";
      case HeatmapOverlay::TrafficCrashPriority: return "Traffic crash priority";

      case HeatmapOverlay::TransitAccess: return "Transit access";
      case HeatmapOverlay::TransitModeSharePotential: return "Transit mode share";

      case HeatmapOverlay::Walkability: return "Walkability";
      case HeatmapOverlay::WalkabilityPark: return "Walkability (parks)";
      case HeatmapOverlay::WalkabilityRetail: return "Walkability (retail)";
      case HeatmapOverlay::WalkabilityEducation: return "Walkability (education)";
      case HeatmapOverlay::WalkabilityHealth: return "Walkability (health)";
      case HeatmapOverlay::WalkabilitySafety: return "Walkability (safety)";

      case HeatmapOverlay::JobAccess: return "Job access";
      case HeatmapOverlay::JobOpportunity: return "Job opportunity";

      case HeatmapOverlay::RoadCentrality: return "Road centrality";
      case HeatmapOverlay::RoadVulnerability: return "Road vulnerability";
      case HeatmapOverlay::RoadBypass: return "Road bypass plan";

      case HeatmapOverlay::Livability: return "Livability";
      case HeatmapOverlay::InterventionPriority: return "Intervention priority";
      case HeatmapOverlay::LivabilityHotspot: return "Livability hotspot";
      case HeatmapOverlay::InterventionHotspot: return "Priority hotspot";

      case HeatmapOverlay::RunoffPollution: return "Runoff pollution";
      case HeatmapOverlay::RunoffPollutionLoad: return "Runoff load";
      case HeatmapOverlay::RunoffMitigationPriority: return "Mitigation priority";
      case HeatmapOverlay::RunoffMitigationPlan: return "Mitigation plan";
      default: return "Heatmap";
      }
    };

    static constexpr HeatmapOverlay kCycle[] = {
        HeatmapOverlay::Off,
        HeatmapOverlay::LandValue,
        HeatmapOverlay::ParkAmenity,
        HeatmapOverlay::WaterAmenity,
        HeatmapOverlay::Pollution,
        HeatmapOverlay::TrafficSpill,
        HeatmapOverlay::ServicesOverall,
        HeatmapOverlay::ServicesEducation,
        HeatmapOverlay::ServicesHealth,
        HeatmapOverlay::ServicesSafety,
        HeatmapOverlay::FireRisk,
        HeatmapOverlay::FloodDepth,
        HeatmapOverlay::PondingDepth,
        HeatmapOverlay::EvacuationTime,
        HeatmapOverlay::EvacuationUnreachable,
        HeatmapOverlay::EvacuationFlow,

        HeatmapOverlay::AirPollution,
        HeatmapOverlay::AirPollutionEmission,
        HeatmapOverlay::SkyView,
        HeatmapOverlay::CanyonConfinement,
        HeatmapOverlay::SolarExposure,
        HeatmapOverlay::SolarPotential,
        HeatmapOverlay::EnergyDemand,
        HeatmapOverlay::EnergySolar,
        HeatmapOverlay::EnergyBalance,

        HeatmapOverlay::CarbonEmission,
        HeatmapOverlay::CarbonSequestration,
        HeatmapOverlay::CarbonBalance,

        HeatmapOverlay::CrimeRisk,
        HeatmapOverlay::PoliceAccess,

        HeatmapOverlay::TrafficCrashRisk,
        HeatmapOverlay::TrafficCrashExposure,
        HeatmapOverlay::TrafficCrashPriority,

        HeatmapOverlay::TransitAccess,
        HeatmapOverlay::TransitModeSharePotential,

        HeatmapOverlay::Walkability,
        HeatmapOverlay::WalkabilityPark,
        HeatmapOverlay::WalkabilityRetail,
        HeatmapOverlay::WalkabilityEducation,
        HeatmapOverlay::WalkabilityHealth,
        HeatmapOverlay::WalkabilitySafety,

        HeatmapOverlay::JobAccess,
        HeatmapOverlay::JobOpportunity,

        HeatmapOverlay::RoadCentrality,
        HeatmapOverlay::RoadVulnerability,
        HeatmapOverlay::RoadBypass,

        HeatmapOverlay::Livability,
        HeatmapOverlay::InterventionPriority,
        HeatmapOverlay::LivabilityHotspot,
        HeatmapOverlay::InterventionHotspot,

        HeatmapOverlay::RunoffPollution,
        HeatmapOverlay::RunoffPollutionLoad,
        HeatmapOverlay::RunoffMitigationPriority,
        HeatmapOverlay::RunoffMitigationPlan,
    };

    constexpr int count = static_cast<int>(sizeof(kCycle) / sizeof(kCycle[0]));
    int idx = 0;
    for (int i = 0; i < count; ++i) {
      if (kCycle[i] == m_heatmapOverlay) {
        idx = i;
        break;
      }
    }

    const int delta = shift ? -1 : 1;
    idx = (idx + delta + count) % count;
    m_heatmapOverlay = kCycle[idx];

    // Mark derived fields dirty (they will be recomputed on demand).
    m_landValueDirty = true;
    m_servicesHeatmapsDirty = true;
    m_fireRiskDirty = true;
    invalidateHydrology();
    invalidateAnalysisLayers();

    showToast(TextFormat("Heatmap: %s", labelOf(m_heatmapOverlay)));
  }


// Brush radius
  if (IsKeyPressed(KEY_LEFT_BRACKET)) {
    if (m_showPolicy) {
      // Policy adjustments
      const int delta = shift ? -5 : -1;
      SimConfig& cfg = m_sim.config();
      TradeModelSettings& tm = m_sim.tradeModel();

      auto clampInt = [&](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

      switch (m_policySelection) {
      case 0: cfg.taxResidential = clampInt(cfg.taxResidential + delta, 0, 10); break;
      case 1: cfg.taxCommercial = clampInt(cfg.taxCommercial + delta, 0, 10); break;
      case 2: cfg.taxIndustrial = clampInt(cfg.taxIndustrial + delta, 0, 10); break;
      case 3: cfg.maintenanceRoad = clampInt(cfg.maintenanceRoad + (shift ? -2 : -1), 0, 5); break;
      case 4: cfg.maintenancePark = clampInt(cfg.maintenancePark + (shift ? -2 : -1), 0, 5); break;
      case 5: cfg.requireOutsideConnection = !cfg.requireOutsideConnection; break;
      case 6: cfg.parkInfluenceRadius = clampInt(cfg.parkInfluenceRadius + (shift ? -2 : -1), 0, 20); break;
      case 7: tm.enabled = !tm.enabled; break;
      case 8: tm.allowImports = !tm.allowImports; break;
      case 9: tm.allowExports = !tm.allowExports; break;
      case 10: tm.tariffPct = clampInt(tm.tariffPct + delta, 0, 30); break;
      case 11: {
        const int count = kDefaultTradePartnerCount;
        int v = tm.importPartner;
        if (v < -1) v = -1;
        v -= 1;
        if (v < -1) v = count - 1;
        tm.importPartner = v;
      } break;
      case 12: {
        const int count = kDefaultTradePartnerCount;
        int v = tm.exportPartner;
        if (v < -1) v = -1;
        v -= 1;
        if (v < -1) v = count - 1;
        tm.exportPartner = v;
      } break;
      default: break;
      }

      // Updating policies affects derived stats and overlays.
      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_tradeDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;
      m_servicesHeatmapsDirty = true;
      m_fireRiskDirty = true;
      m_servicesPlanDirty = true;
      m_servicesPlanValid = false;
      m_outsideOverlayRoadToEdge.clear();
    } else if (m_showNewsPanel) {
      const int delta = shift ? -5 : -1;
      const int count = static_cast<int>(m_cityNews.size());
      if (count > 0) {
        m_newsSelection = std::clamp(m_newsSelection + delta, 0, std::max(0, count - 1));
      }
    } else if (m_showTrafficModel) {
      // Traffic model adjustments
      const float fdelta = shift ? -0.20f : -0.05f;
      TrafficModelSettings& tm = m_sim.trafficModel();

      auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };
      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      switch (m_trafficModelSelection) {
      case 0: tm.congestionAwareRouting = !tm.congestionAwareRouting; break;
      case 1: tm.congestionIterations = clampI(tm.congestionIterations + (shift ? -2 : -1), 1, 16); break;
      case 2: tm.congestionAlpha = clampF(tm.congestionAlpha + fdelta, 0.0f, 2.0f); break;
      case 3: tm.congestionBeta = clampF(tm.congestionBeta + (shift ? -2.0f : -1.0f), 1.0f, 8.0f); break;
      case 4: tm.congestionCapacityScale = clampF(tm.congestionCapacityScale + (shift ? -0.25f : -0.10f), 0.25f, 4.0f);
        break;
      case 5: tm.congestionRatioClamp = clampF(tm.congestionRatioClamp + (shift ? -1.0f : -0.5f), 1.0f, 10.0f);
        break;
      case 6: tm.capacityAwareJobs = !tm.capacityAwareJobs; break;
      case 7: tm.jobAssignmentIterations = clampI(tm.jobAssignmentIterations + (shift ? -2 : -1), 1, 32); break;
      case 8: tm.jobPenaltyBaseMilli = clampI(tm.jobPenaltyBaseMilli + (shift ? -4000 : -1000), 0, 50000); break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_tradeDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;
    } else if (m_showResiliencePanel) {
      adjustResiliencePanel(-1, shift);
    } else if (m_showDistrictPanel) {
      const int deltaI = shift ? -2 : -1;
      const float deltaF = shift ? -0.25f : -0.05f;
      SimConfig& cfg = m_sim.config();

      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      const int d = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
      DistrictPolicy& pol = cfg.districtPolicies[static_cast<std::size_t>(d)];

      switch (m_districtSelection) {
      case 0:
        cfg.districtPoliciesEnabled = !cfg.districtPoliciesEnabled;
        showToast(cfg.districtPoliciesEnabled ? "District policies: ON" : "District policies: OFF");
        break;
      case 1:
        m_activeDistrict = (m_activeDistrict + deltaI + kDistrictCount) % kDistrictCount;
        showToast(TextFormat("Active district: %d", m_activeDistrict));
        break;
      case 2:
        m_showDistrictOverlay = !m_showDistrictOverlay;
        showToast(m_showDistrictOverlay ? "District overlay: ON" : "District overlay: OFF");
        break;
      case 3:
        m_showDistrictBorders = !m_showDistrictBorders;
        showToast(m_showDistrictBorders ? "District borders: ON" : "District borders: OFF");
        break;
      case 4:
        pol.taxResidentialMult = clampF(pol.taxResidentialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d res tax mult: %.2f", d, pol.taxResidentialMult));
        break;
      case 5:
        pol.taxCommercialMult = clampF(pol.taxCommercialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d com tax mult: %.2f", d, pol.taxCommercialMult));
        break;
      case 6:
        pol.taxIndustrialMult = clampF(pol.taxIndustrialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d ind tax mult: %.2f", d, pol.taxIndustrialMult));
        break;
      case 7:
        pol.roadMaintenanceMult = clampF(pol.roadMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d road maint mult: %.2f", d, pol.roadMaintenanceMult));
        break;
      case 8:
        pol.parkMaintenanceMult = clampF(pol.parkMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d park maint mult: %.2f", d, pol.parkMaintenanceMult));
        break;
      default: break;
      }

      // Policies affect derived stats and budget.
      m_sim.refreshDerivedStats(m_world);
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
  m_evacDirty = true;
    } else if (m_showServicesPanel) {
      adjustServicesPanel(-1, shift);
    } else if (m_showTransitPanel) {
      adjustTransitPanel(-1, shift);
    } else if (m_showRoadUpgradePanel) {
      adjustRoadUpgradePanel(-1, shift);
    } else if (m_showVideoSettings) {
      adjustVideoSettings(-1);
    } else {
      m_brushRadius = std::max(0, m_brushRadius - 1);
      showToast(TextFormat("Brush radius: %d", m_brushRadius));
    }
  }
  if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
    if (m_showPolicy) {
      const int delta = shift ? 5 : 1;
      SimConfig& cfg = m_sim.config();
      TradeModelSettings& tm = m_sim.tradeModel();

      auto clampInt = [&](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

      switch (m_policySelection) {
      case 0: cfg.taxResidential = clampInt(cfg.taxResidential + delta, 0, 10); break;
      case 1: cfg.taxCommercial = clampInt(cfg.taxCommercial + delta, 0, 10); break;
      case 2: cfg.taxIndustrial = clampInt(cfg.taxIndustrial + delta, 0, 10); break;
      case 3: cfg.maintenanceRoad = clampInt(cfg.maintenanceRoad + (shift ? 2 : 1), 0, 5); break;
      case 4: cfg.maintenancePark = clampInt(cfg.maintenancePark + (shift ? 2 : 1), 0, 5); break;
      case 5: cfg.requireOutsideConnection = !cfg.requireOutsideConnection; break;
      case 6: cfg.parkInfluenceRadius = clampInt(cfg.parkInfluenceRadius + (shift ? 2 : 1), 0, 20); break;
      case 7: tm.enabled = !tm.enabled; break;
      case 8: tm.allowImports = !tm.allowImports; break;
      case 9: tm.allowExports = !tm.allowExports; break;
      case 10: tm.tariffPct = clampInt(tm.tariffPct + delta, 0, 30); break;
      case 11: {
        const int count = kDefaultTradePartnerCount;
        int v = tm.importPartner;
        if (v < -1) v = -1;
        v += 1;
        if (v >= count) v = -1;
        tm.importPartner = v;
      } break;
      case 12: {
        const int count = kDefaultTradePartnerCount;
        int v = tm.exportPartner;
        if (v < -1) v = -1;
        v += 1;
        if (v >= count) v = -1;
        tm.exportPartner = v;
      } break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_tradeDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
  m_evacDirty = true;
      m_outsideOverlayRoadToEdge.clear();
    } else if (m_showNewsPanel) {
      const int delta = shift ? 5 : 1;
      const int count = static_cast<int>(m_cityNews.size());
      if (count > 0) {
        m_newsSelection = std::clamp(m_newsSelection + delta, 0, count - 1);
      }
    } else if (m_showTrafficModel) {
      // Traffic model adjustments
      const float fdelta = shift ? 0.20f : 0.05f;
      TrafficModelSettings& tm = m_sim.trafficModel();

      auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };
      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      switch (m_trafficModelSelection) {
      case 0: tm.congestionAwareRouting = !tm.congestionAwareRouting; break;
      case 1: tm.congestionIterations = clampI(tm.congestionIterations + (shift ? 2 : 1), 1, 16); break;
      case 2: tm.congestionAlpha = clampF(tm.congestionAlpha + fdelta, 0.0f, 2.0f); break;
      case 3: tm.congestionBeta = clampF(tm.congestionBeta + (shift ? 2.0f : 1.0f), 1.0f, 8.0f); break;
      case 4: tm.congestionCapacityScale = clampF(tm.congestionCapacityScale + (shift ? 0.25f : 0.10f), 0.25f, 4.0f);
        break;
      case 5: tm.congestionRatioClamp = clampF(tm.congestionRatioClamp + (shift ? 1.0f : 0.5f), 1.0f, 10.0f);
        break;
      case 6: tm.capacityAwareJobs = !tm.capacityAwareJobs; break;
      case 7: tm.jobAssignmentIterations = clampI(tm.jobAssignmentIterations + (shift ? 2 : 1), 1, 32); break;
      case 8: tm.jobPenaltyBaseMilli = clampI(tm.jobPenaltyBaseMilli + (shift ? 4000 : 1000), 0, 50000); break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_tradeDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;
    } else if (m_showResiliencePanel) {
      adjustResiliencePanel(+1, shift);
    } else if (m_showDistrictPanel) {
      const int deltaI = shift ? 2 : 1;
      const float deltaF = shift ? 0.25f : 0.05f;
      SimConfig& cfg = m_sim.config();

      const int d = ((m_activeDistrict % kDistrictCount) + kDistrictCount) % kDistrictCount;
      DistrictPolicy& pol = cfg.districtPolicies[static_cast<std::size_t>(d)];
      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };

      switch (m_districtSelection) {
      case 0:
        cfg.districtPoliciesEnabled = !cfg.districtPoliciesEnabled;
        showToast(cfg.districtPoliciesEnabled ? "District policies: ON" : "District policies: OFF");
        break;
      case 1:
        m_activeDistrict = (m_activeDistrict + deltaI) % kDistrictCount;
        showToast(TextFormat("Active district: %d", m_activeDistrict));
        break;
      case 2:
        m_showDistrictOverlay = !m_showDistrictOverlay;
        showToast(m_showDistrictOverlay ? "District overlay: ON" : "District overlay: OFF");
        break;
      case 3:
        m_showDistrictBorders = !m_showDistrictBorders;
        showToast(m_showDistrictBorders ? "District borders: ON" : "District borders: OFF");
        break;
      case 4:
        pol.taxResidentialMult = clampF(pol.taxResidentialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d res tax mult: %.2f", d, pol.taxResidentialMult));
        break;
      case 5:
        pol.taxCommercialMult = clampF(pol.taxCommercialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d com tax mult: %.2f", d, pol.taxCommercialMult));
        break;
      case 6:
        pol.taxIndustrialMult = clampF(pol.taxIndustrialMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d ind tax mult: %.2f", d, pol.taxIndustrialMult));
        break;
      case 7:
        pol.roadMaintenanceMult = clampF(pol.roadMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d road maint mult: %.2f", d, pol.roadMaintenanceMult));
        break;
      case 8:
        pol.parkMaintenanceMult = clampF(pol.parkMaintenanceMult + deltaF, 0.0f, 3.0f);
        showToast(TextFormat("District %d park maint mult: %.2f", d, pol.parkMaintenanceMult));
        break;
      default: break;
      }

      m_sim.refreshDerivedStats(m_world);
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
  m_evacDirty = true;
    } else if (m_showServicesPanel) {
      adjustServicesPanel(+1, shift);
    } else if (m_showTransitPanel) {
      adjustTransitPanel(+1, shift);
    } else if (m_showRoadUpgradePanel) {
      adjustRoadUpgradePanel(+1, shift);
    } else if (m_showVideoSettings) {
      adjustVideoSettings(+1);
    } else {
      m_brushRadius = std::min(8, m_brushRadius + 1);
      showToast(TextFormat("Brush radius: %d", m_brushRadius));
    }
  }

  // Save slot selection
  if (IsKeyPressed(KEY_F6)) {
    endPaintStroke();
    // Hold Shift to cycle backwards.
    cycleSaveSlot(shift ? -1 : 1);
    showToast(TextFormat("Save slot: %d", m_saveSlot));
  }

  // Save / Load (quick save)
  if (IsKeyPressed(KEY_F5)) {
    const std::string path = savePathForSlot(m_saveSlot);
    saveToPath(path, true, TextFormat("Slot %d", m_saveSlot));
  }

  if (IsKeyPressed(KEY_F9)) {
    const std::string path = savePathForSlot(m_saveSlot);
    loadFromPath(path, TextFormat("Slot %d", m_saveSlot));
  }

  // Regenerate
  if (IsKeyPressed(KEY_R)) {
    endPaintStroke();
    resetWorld(TimeSeed());
  }

  // Tool selection
  auto setTool = [&](Tool t) {
    if (m_tool == t) return;
    endPaintStroke();
    m_tool = t;

    // Switching tools clears any inspect selection/path.
    m_inspectSelected.reset();
    m_inspectPath.clear();
    m_inspectPathCost = 0;
    m_inspectInfo.clear();

    // Switching tools also cancels any road-drag preview.
    m_roadDragActive = false;
    m_roadDragStart.reset();
    m_roadDragEnd.reset();
    m_roadDragPath.clear();
    m_roadDragBuildCost = 0;
    m_roadDragUpgradeTiles = 0;
    m_roadDragBridgeTiles = 0;
    m_roadDragMoneyCost = 0;
    m_roadDragValid = false;
  };

  if (IsKeyPressed(KEY_Q)) setTool(Tool::Inspect);
  if (IsKeyPressed(KEY_ONE)) setTool(Tool::Road);
  if (IsKeyPressed(KEY_TWO)) setTool(Tool::Residential);
  if (IsKeyPressed(KEY_THREE)) setTool(Tool::Commercial);
  if (IsKeyPressed(KEY_FOUR)) setTool(Tool::Industrial);
  if (IsKeyPressed(KEY_FIVE)) setTool(Tool::Park);
  if (IsKeyPressed(KEY_ZERO)) setTool(Tool::Bulldoze);

  // Road tool: cycle the road class used for placement/upgrade (Street/Avenue/Highway).
  if (IsKeyPressed(KEY_U) && !ctrl) {
    const int delta = shift ? -1 : 1;
    m_roadBuildLevel += delta;
    if (m_roadBuildLevel < 1) m_roadBuildLevel = 3;
    if (m_roadBuildLevel > 3) m_roadBuildLevel = 1;
    showToast(TextFormat("Road type: %s", RoadClassName(m_roadBuildLevel)));
  }
  if (IsKeyPressed(KEY_SIX)) setTool(Tool::RaiseTerrain);
  if (IsKeyPressed(KEY_SEVEN)) setTool(Tool::LowerTerrain);
  if (IsKeyPressed(KEY_EIGHT)) setTool(Tool::SmoothTerrain);
  if (IsKeyPressed(KEY_NINE)) setTool(Tool::District);

  // ---------------------------------------------------------------------
  // Blueprint tool: capture a rectangular stamp and paste it elsewhere.
  // Toggle with J. In Capture mode: LMB drag selects a rect. In Stamp mode:
  // LMB stamps at the hovered tile (center-anchored).
  // ---------------------------------------------------------------------
  if (IsKeyPressed(KEY_J)) {
    endPaintStroke();
    if (m_blueprintMode == BlueprintMode::Off) {
      m_blueprintMode = BlueprintMode::Capture;
      showToast("Blueprint: CAPTURE (drag LMB to select)");
    } else {
      m_blueprintMode = BlueprintMode::Off;
      m_blueprintSelecting = false;
      m_blueprintSelStart.reset();
      m_blueprintTilingSelecting = false;
      m_blueprintTileSelStart.reset();
      m_blueprintTileUseLibrary = false;
      m_blueprintLibraryOpen = false;
      showToast("Blueprint: OFF");
    }
  }

  if (m_blueprintMode != BlueprintMode::Off) {
    // When the library is open, Escape closes the library first.
    if (IsKeyPressed(KEY_ESCAPE)) {
      if (m_blueprintLibraryOpen) {
        m_blueprintLibraryOpen = false;
        m_blueprintLibraryDeleteArmed = false;
        m_blueprintLibraryDeleteTimer = 0.0f;
        showToast("Blueprint library: CLOSED");
      } else {
        m_blueprintMode = BlueprintMode::Off;
        m_blueprintSelecting = false;
        m_blueprintSelStart.reset();
        m_blueprintTilingSelecting = false;
        m_blueprintTileSelStart.reset();
        m_blueprintTileUseLibrary = false;
        showToast("Blueprint: OFF");
      }
    }

    // Ctrl+L toggles the file-backed blueprint library browser.
    if (ctrl && IsKeyPressed(KEY_L)) {
      m_blueprintLibraryOpen = !m_blueprintLibraryOpen;
      m_blueprintLibraryDeleteArmed = false;
      m_blueprintLibraryDeleteTimer = 0.0f;

      if (m_blueprintLibraryOpen) {
        refreshBlueprintLibrary();
        showToast("Blueprint library: OPEN (Up/Down, Enter=load)");
      } else {
        showToast("Blueprint library: CLOSED");
      }
    }

    // Ctrl+S saves the current (transformed) blueprint stamp to disk.
    if (ctrl && IsKeyPressed(KEY_S)) {
      saveBlueprintToLibrary();
    }

    // Library navigation consumes common keys.
    if (m_blueprintLibraryOpen) {
      // Simple expiry timer for the delete arm.
      if (m_blueprintLibraryDeleteArmed) {
        m_blueprintLibraryDeleteTimer -= dt;
        if (m_blueprintLibraryDeleteTimer <= 0.0f) {
          m_blueprintLibraryDeleteArmed = false;
          m_blueprintLibraryDeleteTimer = 0.0f;
        }
      }

      const int count = static_cast<int>(m_blueprintLibrary.size());

      if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_KP_8)) {
        m_blueprintLibrarySelection = std::clamp(m_blueprintLibrarySelection - 1, 0, std::max(0, count - 1));
      }
      if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_KP_2)) {
        m_blueprintLibrarySelection = std::clamp(m_blueprintLibrarySelection + 1, 0, std::max(0, count - 1));
      }
      if (IsKeyPressed(KEY_HOME)) {
        m_blueprintLibrarySelection = 0;
      }
      if (IsKeyPressed(KEY_END)) {
        m_blueprintLibrarySelection = std::max(0, count - 1);
      }

      if (IsKeyPressed(KEY_DELETE)) {
        if (!m_blueprintLibraryDeleteArmed) {
          m_blueprintLibraryDeleteArmed = true;
          m_blueprintLibraryDeleteTimer = 1.25f;
          showToast("Press Delete again to remove blueprint", 2.0f);
        } else {
          m_blueprintLibraryDeleteArmed = false;
          m_blueprintLibraryDeleteTimer = 0.0f;
          deleteBlueprintFromLibrarySelection();
        }
      }

      // Enter loads the selected blueprint into the stamp tool.
      if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
        loadBlueprintFromLibrarySelection();
        // Keep the library open so you can quickly iterate.
      }

    } else {
      // Enter toggles between Capture and Stamp (if a blueprint is available).
      if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
        if (m_hasBlueprint) {
          m_blueprintMode = (m_blueprintMode == BlueprintMode::Capture) ? BlueprintMode::Stamp
                                                                        : BlueprintMode::Capture;
          showToast(m_blueprintMode == BlueprintMode::Capture ? "Blueprint: CAPTURE"
                                                             : "Blueprint: STAMP");
        } else {
          m_blueprintMode = BlueprintMode::Capture;
          showToast("Blueprint: CAPTURE (no stamp yet)");
        }
      }
    }

    if (IsKeyPressed(KEY_BACKSPACE)) {
      clearBlueprint();
      m_blueprintMode = BlueprintMode::Capture;
      showToast("Blueprint cleared");
    }

    // Transform keys (Stamp mode)
    if (m_blueprintMode == BlueprintMode::Stamp && m_hasBlueprint) {
      if (!ctrl && IsKeyPressed(KEY_Z)) {
        m_blueprintTransform.rotateDeg = (m_blueprintTransform.rotateDeg + 90) % 360;
        m_blueprintTransformedDirty = true;
        showToast(TextFormat("Blueprint rot: %d", m_blueprintTransform.rotateDeg));
      }
      if (!ctrl && IsKeyPressed(KEY_X)) {
        m_blueprintTransform.mirrorX = !m_blueprintTransform.mirrorX;
        m_blueprintTransformedDirty = true;
        showToast(m_blueprintTransform.mirrorX ? "Blueprint mirrorX: ON" : "Blueprint mirrorX: OFF");
      }
      if (!ctrl && IsKeyPressed(KEY_Y)) {
        m_blueprintTransform.mirrorY = !m_blueprintTransform.mirrorY;
        m_blueprintTransformedDirty = true;
        showToast(m_blueprintTransform.mirrorY ? "Blueprint mirrorY: ON" : "Blueprint mirrorY: OFF");
      }
    }
  }

  // Auto-generate administrative districts based on the current road network.
  // K = use all roads. Shift+K = use only roads connected to the map edge.
  if (IsKeyPressed(KEY_K)) {
    endPaintStroke();

    AutoDistrictConfig dc;
    dc.districts = kDistrictCount;
    dc.fillAllTiles = true;
    dc.useTravelTime = true;
    dc.requireOutsideConnection = shift;

    const AutoDistrictResult r = AutoAssignDistricts(m_world, dc);
    m_sim.refreshDerivedStats(m_world);

    showToast(TextFormat("Auto districts: used %d/%d (seeds=%d)%s",
                         r.districtsUsed, r.districtsRequested,
                         static_cast<int>(r.seedRoadIdx.size()),
                         dc.requireOutsideConnection ? " [outside]" : ""));
  }

  if (m_tool == Tool::District) {
    if (IsKeyPressed(KEY_COMMA)) {
      m_activeDistrict = (m_activeDistrict + kDistrictCount - 1) % kDistrictCount;
      showToast(TextFormat("Active district: %d", m_activeDistrict));
    }
    if (IsKeyPressed(KEY_PERIOD)) {
      m_activeDistrict = (m_activeDistrict + 1) % kDistrictCount;
      showToast(TextFormat("Active district: %d", m_activeDistrict));
    }
  }

  // Camera pan: right mouse drag (raylib example style).
  if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
    endPaintStroke();
    Vector2 delta = GetMouseDelta();
    delta.x *= -1.0f / std::max(0.001f, m_camera.zoom);
    delta.y *= -1.0f / std::max(0.001f, m_camera.zoom);
    m_camera.target.x += delta.x;
    m_camera.target.y += delta.y;
  }

  // Keyboard pan (optional). When a text-based UI (console) or a modal menu (save manager)
  // is open, don't let movement keys inadvertently move the camera.
  const float panSpeed = 650.0f * dt / std::max(0.25f, m_camera.zoom);
  const bool blockPanKeys = (m_console.isOpen() || m_showSaveMenu);
  const bool blockArrowPan = blockPanKeys ||
                             (m_blueprintLibraryOpen && m_blueprintMode != BlueprintMode::Off);

  if (!blockPanKeys) {
    if (IsKeyDown(KEY_A)) m_camera.target.x -= panSpeed;
    if (IsKeyDown(KEY_D)) m_camera.target.x += panSpeed;
    if (IsKeyDown(KEY_W)) m_camera.target.y -= panSpeed;
    if (IsKeyDown(KEY_S)) m_camera.target.y += panSpeed;
  }

  if (!blockArrowPan) {
    if (IsKeyDown(KEY_LEFT)) m_camera.target.x -= panSpeed;
    if (IsKeyDown(KEY_RIGHT)) m_camera.target.x += panSpeed;
    if (IsKeyDown(KEY_UP)) m_camera.target.y -= panSpeed;
    if (IsKeyDown(KEY_DOWN)) m_camera.target.y += panSpeed;
  }

  // Zoom around mouse cursor (raylib example style).
  // If a modal UI is open (e.g. the blueprint library list), let the UI own the wheel.
  const bool uiWantsWheel = (m_blueprintLibraryOpen || m_showVideoSettings);
  float wheel = 0.0f;
  if (!uiWantsWheel) {
    wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
      const Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), m_camera);
      m_camera.offset = GetMousePosition();
      m_camera.target = mouseWorldPos;

      const float zoomIncrement = 0.125f;
      m_camera.zoom += wheel * zoomIncrement;
      m_camera.zoom = std::clamp(m_camera.zoom, 0.25f, 4.0f);
    }
  }

  // Build/paint with left mouse.
  const bool leftPressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
  const bool leftDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
  const bool leftReleased = IsMouseButtonReleased(MOUSE_BUTTON_LEFT);

  // --- Minimap interaction (UI consumes left mouse so we don't accidentally paint the world). ---
  bool consumeLeft = false;

  // UI panels should consume left mouse so we do not click-through and
  // accidentally paint/inspect the world while interacting with widgets.
  const bool uiConsumesLeft = [&]() -> bool {
    // Report panel (top-left).
    if (m_showReport) {
      int w = 520;
      int h = 420;
      const int x0 = kUiPanelMargin;
      const int y0 = kUiPanelTopY;
      if (uiW > 0) {
        w = std::min(w, uiW - x0 - kUiPanelMargin);
      }
      if (uiH > 0) {
        h = std::min(h, uiH - y0 - kUiPanelMargin);
      }
      if (w > 0 && h > 0) {
        const Rectangle r{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(w), static_cast<float>(h)};
        if (CheckCollisionPointRec(mouseUi, r)) {
          return true;
        }
      }
    }

    // Video settings panel (below report if visible).
    if (m_showVideoSettings) {
      int w = 432;
      int h = 276;
      const int x0 = kUiPanelMargin;
      int y0 = kUiPanelTopY;
      if (m_showReport) {
        y0 += 420 + kUiPanelMargin;
      }
      if (uiW > 0) {
        w = std::min(w, uiW - x0 - kUiPanelMargin);
      }
      if (uiH > 0) {
        h = std::min(h, uiH - y0 - kUiPanelMargin);
      }
      if (w > 0 && h > 0) {
        const Rectangle r{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(w), static_cast<float>(h)};
        if (CheckCollisionPointRec(mouseUi, r)) {
          return true;
        }
      }
    }

    // Right-side docked panels.
    {
      RightPanelDock dock(uiW, uiH);
      auto checkDock = [&](bool show, int pw, int ph) -> bool {
        if (!show) return false;
        const Rectangle r = dock.alloc(pw, ph);
        return CheckCollisionPointRec(mouseUi, r);
      };
      if (checkDock(m_showPolicy, kPolicyPanelW, kPolicyPanelH)) return true;
      if (checkDock(m_showNewsPanel, kNewsPanelW, kNewsPanelH)) return true;
      if (checkDock(m_showChallengesPanel, kChallengesPanelW, kChallengesPanelH)) return true;
      if (checkDock(m_showBondsPanel, kBondsPanelW, kBondsPanelH)) return true;
      if (checkDock(m_showTrafficModel, kTrafficPanelW, kTrafficPanelH)) return true;
      if (checkDock(m_showResiliencePanel, kResiliencePanelW, kResiliencePanelH)) return true;
      if (checkDock(m_showDistrictPanel, kDistrictPanelW, kDistrictPanelH)) return true;
      if (checkDock(m_showTransitPanel, kTransitPanelW, kTransitPanelH)) return true;
      if (checkDock(m_showWayfindingPanel, kWayfindingPanelW, kWayfindingPanelH)) return true;
      if (checkDock(m_showRoadUpgradePanel, kRoadUpgradePanelW, kRoadUpgradePanelH)) return true;
      if (checkDock(m_showAutoBuildPanel, kAutoBuildPanelW, kAutoBuildPanelH)) return true;
    }

    return false;
  }();
  if (uiConsumesLeft && (leftDown || leftPressed)) {
    consumeLeft = true;
  }
  bool overMinimap = false;
  if (m_showMinimap && m_world.width() > 0 && m_world.height() > 0) {
    const Renderer::MinimapLayout mini = m_renderer.minimapLayout(m_world, uiW, uiH);
    const Vector2 mp = mouseUi;
    const bool over = CheckCollisionPointRec(mp, mini.rect);
    overMinimap = over;

    if (leftPressed && over) {
      // Cancel any in-progress stroke before moving the camera.
      endPaintStroke();
      m_minimapDragActive = true;
    }

    if (leftReleased) {
      m_minimapDragActive = false;
    }

    if (leftDown && m_minimapDragActive) {
      const float lx = std::clamp(mp.x - mini.rect.x, 0.0f, std::max(1.0f, mini.rect.width - 1.0f));
      const float ly = std::clamp(mp.y - mini.rect.y, 0.0f, std::max(1.0f, mini.rect.height - 1.0f));

      const float s = std::max(1.0e-3f, mini.pixelsPerTile);
      const int tx = std::clamp(static_cast<int>(std::floor(lx / s)), 0, m_world.width() - 1);
      const int ty = std::clamp(static_cast<int>(std::floor(ly / s)), 0, m_world.height() - 1);

      m_camera.target = TileToWorldCenterElevated(m_world, tx, ty, static_cast<float>(m_cfg.tileWidth),
                                                  static_cast<float>(m_cfg.tileHeight), m_elev);
      consumeLeft = true;
    }

    // If the cursor is over the minimap, don't start any world interactions on press.
    if (over && leftPressed) consumeLeft = true;
  } else {
    m_minimapDragActive = false;
  }


  // Blueprint interaction: capture/stamp consumes LMB while active.
  if (m_blueprintMode != BlueprintMode::Off) {
    // Prevent other tools from reacting to left mouse while blueprint mode is active.
    consumeLeft = true;

    // Minimap gets priority if the cursor is over it.
    if (!overMinimap && !m_blueprintLibraryOpen) {
      if (m_blueprintMode == BlueprintMode::Capture) {
        if (leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
          endPaintStroke();
          m_blueprintSelecting = true;
          m_blueprintSelStart = *m_hovered;
          m_blueprintSelEnd = *m_hovered;
        }

        if (leftDown && m_blueprintSelecting && m_hovered) {
          m_blueprintSelEnd = *m_hovered;
        }

        if (leftReleased && m_blueprintSelecting) {
          m_blueprintSelecting = false;
          if (m_blueprintSelStart) {
            const Point a = *m_blueprintSelStart;
            const Point b = m_blueprintSelEnd;
            const int x0 = std::min(a.x, b.x);
            const int y0 = std::min(a.y, b.y);
            const int x1 = std::max(a.x, b.x);
            const int y1 = std::max(a.y, b.y);
            const int w = x1 - x0 + 1;
            const int h = y1 - y0 + 1;

            Blueprint bp;
            std::string err;
            if (CaptureBlueprintRect(m_world, x0, y0, w, h, bp, err, m_blueprintCaptureOpt)) {
              m_hasBlueprint = true;
              m_blueprint = std::move(bp);
              m_blueprintTransform = BlueprintTransform{};
              m_blueprintTransformedDirty = true;
              updateBlueprintTransformed();
              m_blueprintMode = BlueprintMode::Stamp;
              showToast(TextFormat("Blueprint captured (%dx%d, %d tiles) - click to stamp",
                                   m_blueprintTransformed.width, m_blueprintTransformed.height,
                                   static_cast<int>(m_blueprintTransformed.tiles.size())));
            } else {
              showToast(std::string("Blueprint capture failed: ") + err, 3.0f);
            }
          }
        }
      } else if (m_blueprintMode == BlueprintMode::Stamp) {
        // Shift+drag: tile fill with current stamp (macro-tiling)
        // Ctrl+Shift+drag: procedural tiling using the blueprint library tileset, with road-edge matching.

        if (m_blueprintTilingSelecting) {
          if (leftDown && m_hovered) {
            m_blueprintTileSelEnd = *m_hovered;
          }

          if (leftReleased) {
            const bool useLib = m_blueprintTileUseLibrary;
            m_blueprintTilingSelecting = false;
            m_blueprintTileUseLibrary = false;

            if (m_blueprintTileSelStart) {
              const Point a = *m_blueprintTileSelStart;
              const Point b = m_blueprintTileSelEnd;
              tileBlueprintRect(a.x, a.y, b.x, b.y, useLib);
            }
            m_blueprintTileSelStart.reset();
          }
        } else {
          if (leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            if (!m_hasBlueprint) {
              showToast("No blueprint captured (switch to CAPTURE)", 2.0f);
            } else if (shift) {
              // Start selection.
              endPaintStroke();
              m_blueprintTilingSelecting = true;
              m_blueprintTileUseLibrary = ctrl;
              m_blueprintTileSelStart = *m_hovered;
              m_blueprintTileSelEnd = *m_hovered;
            } else {
              stampBlueprintAt(*m_hovered);
            }
          }
        }
      }
    }
  }

  // Road tool: Shift+drag plans a cheapest (money cost) road path (includes upgrades/bridges)
  // and commits the whole path on release (single undoable stroke).
  const bool roadDragMode = (m_tool == Tool::Road) && shift && !m_painting && !consumeLeft;

  if (roadDragMode) {
    auto computePathEconomy = [&](const std::vector<Point>& path, int& outNewTiles, int& outUpgrades, int& outBridgeTiles, int& outCost) {
      outNewTiles = 0;
      outUpgrades = 0;
      outBridgeTiles = 0;
      outCost = 0;

      const int targetLevel = ClampRoadLevel(m_roadBuildLevel);

      for (const Point& p : path) {
        if (!m_world.inBounds(p.x, p.y)) continue;
        const Tile& t = m_world.at(p.x, p.y);
        const bool isBridge = (t.terrain == Terrain::Water);

        if (t.overlay == Overlay::Road) {
          const int cur = ClampRoadLevel(static_cast<int>(t.level));
          if (cur < targetLevel) {
            outUpgrades += 1;
            if (isBridge) outBridgeTiles += 1;
            outCost += RoadPlacementCost(cur, targetLevel, /*alreadyRoad=*/true, isBridge);
          }
        } else if (t.overlay == Overlay::None) {
          outNewTiles += 1;
          if (isBridge) outBridgeTiles += 1;
          outCost += RoadPlacementCost(1, targetLevel, /*alreadyRoad=*/false, isBridge);
        }
      }
    };

    // Road planner config: money-aware and bridge-aware.
    RoadBuildPathConfig planCfg;
    planCfg.allowBridges = true;
    planCfg.costModel = RoadBuildPathConfig::CostModel::Money;

    // Start drag.
    if (leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Point start = *m_hovered;

      std::vector<Point> tmp;
      planCfg.targetLevel = m_roadBuildLevel;
      if (!FindRoadBuildPath(m_world, start, start, tmp, nullptr, planCfg)) {
        showToast("Can't start a road path here", 2.5f);
      } else {
        endPaintStroke();
        m_roadDragActive = true;
        m_roadDragStart = start;
        m_roadDragEnd = start;
        m_roadDragPath = std::move(tmp);
        computePathEconomy(m_roadDragPath, m_roadDragBuildCost, m_roadDragUpgradeTiles, m_roadDragBridgeTiles, m_roadDragMoneyCost);
        m_roadDragValid = true;
      }
    }

    // Update preview.
    if (leftDown && m_roadDragActive && m_roadDragStart && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Point end = *m_hovered;
      if (!m_roadDragEnd || (m_roadDragEnd->x != end.x) || (m_roadDragEnd->y != end.y)) {
        m_roadDragEnd = end;

        std::vector<Point> tmp;
        planCfg.targetLevel = m_roadBuildLevel;
        const bool ok = FindRoadBuildPath(m_world, *m_roadDragStart, end, tmp, nullptr, planCfg);
        if (ok && !tmp.empty()) {
          m_roadDragValid = true;
          m_roadDragPath = std::move(tmp);
          computePathEconomy(m_roadDragPath, m_roadDragBuildCost, m_roadDragUpgradeTiles, m_roadDragBridgeTiles, m_roadDragMoneyCost);
        } else {
          m_roadDragValid = false;
          m_roadDragPath.clear();
          m_roadDragBuildCost = 0;
          m_roadDragUpgradeTiles = 0;
          m_roadDragBridgeTiles = 0;
          m_roadDragMoneyCost = 0;
        }
      }
    }

    // Commit on release.
    if (leftReleased && m_roadDragActive) {
      if (m_roadDragValid && !m_roadDragPath.empty()) {
        const int moneyBefore = m_world.stats().money;

        // Make the road-drag tool atomic: if we cannot afford the whole plan, don't build a partial path.
        if (m_roadDragMoneyCost > moneyBefore) {
          showToast(TextFormat("Not enough funds for planned path: need $%d (short $%d)",
                               m_roadDragMoneyCost, m_roadDragMoneyCost - moneyBefore),
                    3.0f);
        } else {
          beginPaintStroke();
          const int savedRadius = m_brushRadius;
          m_brushRadius = 0; // path tool is always 1-tile wide

          for (const Point& p : m_roadDragPath) {
            applyToolBrush(p.x, p.y);
          }

          m_brushRadius = savedRadius;

          const bool hadFailures = m_strokeFeedback.any();
          endPaintStroke();

          if (!hadFailures) {
            const int spent = moneyBefore - m_world.stats().money;
            const int bridges = m_roadDragBridgeTiles;
            if (spent > 0) {
              if (bridges > 0) {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded, %d bridge tiles, cost %d)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles, bridges,
                                     spent));
              } else {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded, cost %d)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles, spent));
              }
            } else {
              if (bridges > 0) {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded, %d bridge tiles)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles, bridges));
              } else {
                showToast(TextFormat("Built road path (%s: %d new, %d upgraded)",
                                     RoadClassName(m_roadBuildLevel), m_roadDragBuildCost, m_roadDragUpgradeTiles));
              }
            }
          }
        }
      } else {
        showToast("No valid road path", 2.5f);
      }

      // Clear drag state.
      m_roadDragActive = false;
      m_roadDragStart.reset();
      m_roadDragEnd.reset();
      m_roadDragPath.clear();
      m_roadDragBuildCost = 0;
      m_roadDragUpgradeTiles = 0;
      m_roadDragBridgeTiles = 0;
      m_roadDragMoneyCost = 0;
      m_roadDragValid = false;
    }
  }

  // Inspect click:
  //  - Default: select tile and (if possible) compute the shortest road path to the map edge (outside connection).
  //  - When an evacuation heatmap is active: show evacuation-to-edge cost and highlight the evacuation route.
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::Inspect) {
    m_inspectSelected = *m_hovered;
    m_inspectPath.clear();
    m_inspectPathCost = 0;
    m_inspectInfo.clear();

    const Point sel = *m_inspectSelected;
    const Tile& t = m_world.at(sel.x, sel.y);

    const bool evacHeatmap =
        (m_heatmapOverlay == HeatmapOverlay::EvacuationTime) ||
        (m_heatmapOverlay == HeatmapOverlay::EvacuationUnreachable) ||
        (m_heatmapOverlay == HeatmapOverlay::EvacuationFlow);

    bool handled = false;
    if (evacHeatmap) {
      ensureEvacuationScenarioUpToDate();
      const EvacuationResult& er = m_evacScenario.evac;
      const int w = m_world.width();
      const int h = m_world.height();
      const int idx0 = (w > 0 && h > 0) ? (sel.y * w + sel.x) : -1;
      const char* unit = m_evacCfg.evac.useTravelTime ? "time" : "steps";

      auto isHazard = [&](int idx) -> bool {
        return (idx >= 0 && idx < static_cast<int>(m_evacScenario.hazardMask.size()) && m_evacScenario.hazardMask[idx] != 0);
      };

      auto buildRoadRouteToExit = [&](int startIdx) {
        m_inspectPath.clear();
        if (w <= 0 || h <= 0) return;
        if (startIdx < 0 || startIdx >= static_cast<int>(er.roadParent.size())) return;

        int cur = startIdx;
        for (int iter = 0; iter < w * h && cur >= 0; ++iter) {
          const int rx = cur % w;
          const int ry = cur / w;
          m_inspectPath.push_back(Point{rx, ry});

          const int next = er.roadParent[cur];
          if (next == cur) break; // safety against accidental self-loops
          cur = next;
        }
      };

      if (t.overlay == Overlay::Residential) {
        handled = true;

        if (idx0 < 0 || idx0 >= static_cast<int>(er.resCostMilli.size()) ||
            idx0 >= static_cast<int>(er.resAccessRoad.size())) {
          m_inspectInfo = TextFormat("Evac (%d,%d): (no data)", sel.x, sel.y);
          showToast(m_inspectInfo, 3.0f);
        } else if (isHazard(idx0)) {
          m_inspectInfo = TextFormat("Evac (%d,%d): HAZARD (flooded)", sel.x, sel.y);
          showToast(m_inspectInfo, 3.0f);
        } else {
          const int costMilli = er.resCostMilli[idx0];
          const int accessRoad = er.resAccessRoad[idx0];
          if (costMilli < 0 || accessRoad < 0) {
            m_inspectInfo = TextFormat("Evac (%d,%d): unreachable", sel.x, sel.y);
            showToast(m_inspectInfo, 3.0f);
          } else {
            buildRoadRouteToExit(accessRoad);
            m_inspectPathCost = costMilli;

            const double c = static_cast<double>(costMilli) / 1000.0;
            const int arx = accessRoad % w;
            const int ary = accessRoad / w;
            const int flow = (accessRoad >= 0 && accessRoad < static_cast<int>(er.evacRoadFlow.size()))
                                 ? er.evacRoadFlow[accessRoad]
                                 : 0;
            m_inspectInfo = TextFormat("Evac (%d,%d): %.1f %s  access(%d,%d)  flow %d", sel.x, sel.y, c, unit, arx, ary,
                                       flow);
            showToast(m_inspectInfo);
          }
        }
      } else if (t.overlay == Overlay::Road) {
        handled = true;

        if (idx0 < 0 || idx0 >= static_cast<int>(er.roadCostMilli.size())) {
          m_inspectInfo = TextFormat("Evac road (%d,%d): (no data)", sel.x, sel.y);
          showToast(m_inspectInfo, 3.0f);
        } else if (isHazard(idx0)) {
          m_inspectInfo = TextFormat("Evac road (%d,%d): HAZARD (blocked)", sel.x, sel.y);
          showToast(m_inspectInfo, 3.0f);
        } else {
          const int costMilli = er.roadCostMilli[idx0];
          if (costMilli < 0) {
            m_inspectInfo = TextFormat("Evac road (%d,%d): unreachable", sel.x, sel.y);
            showToast(m_inspectInfo, 3.0f);
          } else {
            buildRoadRouteToExit(idx0);
            m_inspectPathCost = costMilli;

            const double c = static_cast<double>(costMilli) / 1000.0;
            const int flow = (idx0 >= 0 && idx0 < static_cast<int>(er.evacRoadFlow.size())) ? er.evacRoadFlow[idx0] : 0;
            const float cong = (idx0 >= 0 && idx0 < static_cast<int>(m_evacHeatmaps.roadCongestionFrac.size()))
                                   ? m_evacHeatmaps.roadCongestionFrac[idx0]
                                   : 0.0f;
            m_inspectInfo =
                TextFormat("Evac road (%d,%d): %.1f %s  flow %d  cong %.0f%%", sel.x, sel.y, c, unit, flow,
                           static_cast<double>(cong * 100.0f));
            showToast(m_inspectInfo);
          }
        }
      }
    }

    if (!handled) {
      auto pickAdjacentRoad = [&](Point& outRoad) -> bool {
        // Deterministic neighbor order.
        constexpr int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& d : dirs) {
          const int nx = sel.x + d[0];
          const int ny = sel.y + d[1];
          if (!m_world.inBounds(nx, ny)) continue;
          if (m_world.at(nx, ny).overlay == Overlay::Road) {
            outRoad = Point{nx, ny};
            return true;
          }
        }
        return false;
      };

      Point startRoad = sel;
      bool hasStartRoad = false;

      if (t.overlay == Overlay::Road) {
        hasStartRoad = true;
      } else {
        hasStartRoad = pickAdjacentRoad(startRoad);
      }

      if (!hasStartRoad) {
        m_inspectInfo = TextFormat("Inspect (%d,%d): no adjacent road", sel.x, sel.y);
        showToast(m_inspectInfo);
      } else {
        const bool ok = FindRoadPathToEdge(m_world, startRoad, m_inspectPath, &m_inspectPathCost);
        if (ok) {
          m_inspectInfo = TextFormat("Inspect (%d,%d): outside YES (road dist %d)", sel.x, sel.y, m_inspectPathCost);
          showToast(m_inspectInfo);
        } else {
          m_inspectInfo = TextFormat("Inspect (%d,%d): outside NO", sel.x, sel.y);
          showToast(m_inspectInfo, 3.0f);
        }
      }
    }
  }

  // District tool: Alt+click to pick the hovered tile's district ID (avoids accidental repainting).
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::District
      && (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))) {
    const Tile& t = m_world.at(m_hovered->x, m_hovered->y);
    m_activeDistrict = static_cast<int>(t.district) % kDistrictCount;
    showToast(TextFormat("Picked district: %d", m_activeDistrict));
    consumeLeft = true;
  }

  // District tool: Shift+click flood fills a region.
  // Ctrl+Shift allows the flood to cross roads when filling land blocks.
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool == Tool::District
      && shift && !(IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT))) {
    floodFillDistrict(*m_hovered, ctrl);
    consumeLeft = true;
  }

  // --- Block operations for build tools ---
  // Shift+click flood-fills the region under the cursor and applies the current tool:
  //   - Land: fills the land block bounded by water and roads
  //   - Road: fills the connected road component
  //   - Water: fills the connected water body (excluding bridges)
  // Ctrl+Shift allows the fill to cross roads (land-block mode). Use carefully.
  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && shift
      && (m_tool == Tool::Residential || m_tool == Tool::Commercial || m_tool == Tool::Industrial || m_tool == Tool::Park
          || m_tool == Tool::Bulldoze)) {
    floodFillTool(*m_hovered, ctrl);
    consumeLeft = true;
  }

  if (!consumeLeft && !roadDragMode && leftPressed && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    beginPaintStroke();
  }

  if (!consumeLeft && !roadDragMode && leftDown && m_painting && m_hovered && !IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && m_tool != Tool::Inspect) {
    applyToolBrush(m_hovered->x, m_hovered->y);
  }

  if (!consumeLeft && !roadDragMode && leftReleased) {
    endPaintStroke();
  }

  // If the window is resizable, keep the camera offset sane.
  if (IsWindowResized()) {
    // Only reset if not in the middle of a "zoom around cursor" moment.
    // (This is a simple heuristic; you can refine later.)
    if (wheel == 0.0f) {
      m_camera.offset = Vector2{GetScreenWidth() * 0.5f, GetScreenHeight() * 0.5f};
    }
  }
}

void Game::updateAutosave(float dt)
{
  if (!m_autosaveEnabled) return;
  if (m_painting) return;

  m_autosaveTimer += dt;
  if (m_autosaveTimer < kAutosaveIntervalSec) return;

  // Avoid spamming identical autosaves (e.g., if the sim is paused).
  const int day = m_world.stats().day;
  if (day == m_lastAutosaveDay) {
    m_autosaveTimer = kAutosaveIntervalSec;
    return;
  }

  // Rotate through autosave slots.
  const int slot = std::clamp(m_autosaveNextSlot, kAutosaveSlotMin, kAutosaveSlotMax);
  const std::string path = autosavePathForSlot(slot);

  // Best effort: autosaves should never disrupt gameplay.
  std::string err;
  if (SaveWorldBinary(m_world, m_procCfg, m_sim.config(), path, err)) {
    const std::string thumb = thumbPathForSavePath(path);
    (void)m_renderer.exportMinimapThumbnail(m_world, thumb.c_str(), 256);

    m_lastAutosaveDay = day;
    m_autosaveNextSlot = (slot >= kAutosaveSlotMax) ? kAutosaveSlotMin : (slot + 1);
    m_autosaveTimer = 0.0f;

    // Avoid toasts when the save menu is open; the list itself is feedback.
    if (!m_showSaveMenu) {
      showToast(TextFormat("Autosaved (slot %d)", slot), 1.5f);
    }

    if (m_showSaveMenu) refreshSaveMenu();
  } else {
    // If autosave fails, back off a bit to avoid hammering the filesystem.
    m_autosaveTimer = kAutosaveIntervalSec * 0.75f;
  }
}

void Game::update(float dt)
{
  // Deferred mine loads are applied at the start of the next frame.
  if (m_pendingMineLoadSeed) {
    const std::uint64_t seed = m_pendingMineLoadSeedValue;
    m_pendingMineLoadSeed = false;
    m_pendingMineLoadSeedValue = 0;
    resetWorld(seed);
  }


  // Pause simulation while actively painting so an undoable "stroke" doesn't
  // accidentally include sim-driven money changes.
  //
  // Game loop note: the simulator maintains its own fixed-tick accumulator and can
  // process multiple ticks back-to-back if dt spikes. For real-time play we cap the
  // amount of catch-up work per frame to avoid "spiral of death" hitches.
  m_loopStats.simTicks = 0;
  m_loopStats.simCpuMs = 0.0;
  m_loopStats.simMsPerTick = 0.0;
  m_loopStats.simTickLimit = 0;

  if (!m_painting && !simPaused()) {
    if (m_replayCapture.active()) {
      std::string err;
      (void)m_replayCapture.captureSettingsIfChanged(m_world, m_procCfg, m_sim, err);
    }

    // Choose how many sim ticks we're willing to process this frame.
    // (0 means "unlimited" when the limiter is disabled.)
    int tickLimit = 0;
    if (m_loopCfg.simTickLimiter) {
      if (m_loopCfg.simTickAuto) {
        if (m_loopStats.simMsPerTickEma > 0.0 && m_loopCfg.simBudgetMs > 0.0f) {
          tickLimit = static_cast<int>(
              std::floor(static_cast<double>(m_loopCfg.simBudgetMs) / m_loopStats.simMsPerTickEma));
        } else {
          tickLimit = m_loopCfg.simMaxTicksPerFrame;
        }
        tickLimit = std::clamp(tickLimit, m_loopCfg.simAutoMinTicks, m_loopCfg.simAutoMaxTicks);
      } else {
        tickLimit = std::max(0, m_loopCfg.simMaxTicksPerFrame);
      }
    }
    m_loopStats.simTickLimit = tickLimit;

    std::vector<Stats> tickStats;
    tickStats.reserve(4);

    const auto t0 = std::chrono::steady_clock::now();
    const int ticks = m_sim.updateLimited(m_world, m_loopStats.dtSim, tickLimit, m_loopCfg.simMaxBacklogTicks,
                                         &tickStats);
    const auto t1 = std::chrono::steady_clock::now();

    m_loopStats.simTicks = ticks;
    m_loopStats.simCpuMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (ticks > 0) {
      m_loopStats.simMsPerTick = m_loopStats.simCpuMs / static_cast<double>(ticks);
      constexpr double kAlpha = 0.10;
      if (m_loopStats.simMsPerTickEma <= 0.0) m_loopStats.simMsPerTickEma = m_loopStats.simMsPerTick;
      else m_loopStats.simMsPerTickEma =
          (1.0 - kAlpha) * m_loopStats.simMsPerTickEma + kAlpha * m_loopStats.simMsPerTick;
    }

    if (ticks > 0) {
      if (m_replayCapture.active()) {
        m_replayCapture.recordTicks(static_cast<std::uint32_t>(ticks));
      }
      // The sim advanced 1..N ticks. These derived overlays depend on occupants/jobs.
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;

      // Public services heatmaps depend on occupants + zones; refresh periodically while the sim runs.
      m_servicesHeatmapsTimer += dt;
      constexpr float kServicesHeatmapUpdateInterval = 0.75f;
      if (m_servicesHeatmapsTimer >= kServicesHeatmapUpdateInterval) {
        m_servicesHeatmapsDirty = true;
        m_servicesHeatmapsTimer = kServicesHeatmapUpdateInterval;
      }

      // Fire risk depends on occupancy + fire station placement.
      m_fireRiskTimer += dt;
      constexpr float kFireRiskUpdateInterval = 1.25f;
      if (m_fireRiskTimer >= kFireRiskUpdateInterval) {
        m_fireRiskDirty = true;
        m_fireRiskTimer = kFireRiskUpdateInterval;
      }

      // Advanced analysis layers (air/solar/walkability/livability/runoff/etc) can be expensive.
      // Refresh them periodically while the sim runs so heatmap previews stay reasonably current.
      m_analysisHeatmapsTimer += dt;
      constexpr float kAnalysisUpdateInterval = 1.25f;
      if (m_analysisHeatmapsTimer >= kAnalysisUpdateInterval) {
        invalidateAnalysisLayers();
        m_analysisHeatmapsTimer = kAnalysisUpdateInterval;
      }

      // Keep the software 3D preview in sync with sim-driven changes.
      m_3dPreviewDirty = true;

      // Apply gameplay layer (challenges) and then record per-tick snapshots.
      // When multiple ticks run in one frame, we apply challenge rewards deterministically
      // in tick order so later snapshots reflect earlier grants.
      int moneyOffset = 0;
      for (const Stats& s : tickStats) {
        Stats ss = s;
        ss.money += moneyOffset;
        const int grant = applyCityChallenges(ss);
        moneyOffset += grant;
        recordHistorySample(ss);
      }
      if (moneyOffset != 0) {
        m_world.stats().money += moneyOffset;
      }
    }
  }

  // Sim backlog stats for debug overlays.
  m_loopStats.simBacklogSec = m_sim.accumulatedSeconds();
  m_loopStats.simBacklogTicks = m_sim.accumulatedTicks();

  if (m_toastTimer > 0.0f) {
    m_toastTimer -= dt;
    if (m_toastTimer < 0.0f) m_toastTimer = 0.0f;
  }

  // Update vehicle visualization (movement pauses when sim is paused or while painting).
  const float vdt = (!m_painting && !simPaused()) ? dt : 0.0f;
  updateVehicles(vdt);

  // Autosave uses wall-clock time (so it works regardless of sim speed).
  updateAutosave(dt);

  // Save menu housekeeping.
  if (m_saveMenuDeleteArmed) {
    m_saveMenuDeleteTimer -= dt;
    if (m_saveMenuDeleteTimer <= 0.0f) {
      m_saveMenuDeleteTimer = 0.0f;
      m_saveMenuDeleteArmed = false;
    }
  }

  if (m_showSaveMenu) {
    m_saveMenuRefreshTimer += dt;
    if (m_saveMenuRefreshTimer >= 1.0f) {
      m_saveMenuRefreshTimer = 0.0f;
      refreshSaveMenu();
    }
  } else {
    m_saveMenuRefreshTimer = 0.0f;
  }

  // Throttle the software 3D preview (CPU renderer) so it doesn't constantly
  // rebuild every frame. We rebuild on world changes (endPaintStroke / sim tick)
  // and also periodically while the panel is visible so day/night/weather
  // updates are reflected.
  if (m_show3DPreview) {
    m_3dPreviewTimer += dt;
    if (m_3dPreviewTimer >= k3DPreviewUpdateInterval) {
      m_3dPreviewTimer = 0.0f;
      m_3dPreviewDirty = true;
    }
  } else {
    m_3dPreviewTimer = 0.0f;
  }

  // Procedural POV camera (ride-along). This runs after sim/vehicle updates so
  // the camera can react to the latest world state.
  updatePov(dt);

  // Optional dynamic resolution scaling for the world layer.
  updateDynamicWorldRenderScale(dt);
  updateDossierExportJob();
  updateMeshExportJob();
  updatePolicyOptimizationJob();
  updateSeedMining(dt);
  updateVisualPrefsAutosave(dt);
}


namespace {

template <typename F>
void DrawHistoryGraph(const std::vector<CityHistorySample>& samples, Rectangle r, const char* title, F getValue,
                      float fixedMin, float fixedMax, bool fixedRange, const char* valueFmt, bool percent = false)
{
  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& th = ui::GetTheme();

  // Background.
  ui::DrawPanelInset(r, uiTime, /*active=*/true);

  const int pad = 10;
  const int titleSize = 18;
  const int fontSmall = 13;

  // Title.
  ui::Text(static_cast<int>(r.x) + pad, static_cast<int>(r.y) + 6, titleSize, title, th.text,
           /*bold=*/true, /*shadow=*/true, 1);

  const std::size_t n = samples.size();
  if (n < 2) {
    ui::Text(static_cast<int>(r.x) + pad, static_cast<int>(r.y) + 34, 14, "(no data yet)", th.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    return;
  }

  // Range.
  float vmin = fixedMin;
  float vmax = fixedMax;
  if (!fixedRange) {
    vmin = std::numeric_limits<float>::infinity();
    vmax = -std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      const float v = getValue(samples[i]);
      vmin = std::min(vmin, v);
      vmax = std::max(vmax, v);
    }
    if (std::abs(vmax - vmin) < 1e-6f) {
      vmax = vmin + 1.0f;
    } else {
      const float padv = 0.06f * (vmax - vmin);
      vmin -= padv;
      vmax += padv;
    }
  } else {
    if (std::abs(vmax - vmin) < 1e-6f) vmax = vmin + 1.0f;
  }

  // Graph area.
  Rectangle gr = r;
  gr.x += static_cast<float>(pad);
  gr.width -= static_cast<float>(pad * 2);
  gr.y += 34.0f;
  gr.height -= 52.0f;

  // Grid lines.
  const int gridLines = 3;
  for (int i = 0; i <= gridLines; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(gridLines);
    const int yy = static_cast<int>(gr.y + t * gr.height);
    DrawLine(static_cast<int>(gr.x), yy, static_cast<int>(gr.x + gr.width), yy, Fade(th.grid, 0.35f));
  }

  auto mapX = [&](std::size_t i) -> float {
    const float t = static_cast<float>(i) / static_cast<float>(n - 1);
    return gr.x + t * gr.width;
  };
  auto mapY = [&](float v) -> float {
    const float t = (v - vmin) / (vmax - vmin);
    return gr.y + (1.0f - std::clamp(t, 0.0f, 1.0f)) * gr.height;
  };

  // Line.
  for (std::size_t i = 1; i < n; ++i) {
    const float x0 = mapX(i - 1);
    const float y0 = mapY(getValue(samples[i - 1]));
    const float x1 = mapX(i);
    const float y1 = mapY(getValue(samples[i]));
    DrawLineEx(Vector2{x0, y0}, Vector2{x1, y1}, 2.0f, th.accent);
  }

  // Latest label.
  char buf[96];
  const float latest = getValue(samples.back());
  if (percent) {
    std::snprintf(buf, sizeof(buf), valueFmt, static_cast<double>(latest * 100.0f));
  } else {
    std::snprintf(buf, sizeof(buf), valueFmt, static_cast<double>(latest));
  }
  ui::Text(static_cast<int>(r.x) + pad, static_cast<int>(r.y + r.height) - 22, fontSmall, buf, th.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
}

void DrawHistoryGraphRCI(const std::vector<CityHistorySample>& samples, Rectangle r)
{
  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& th = ui::GetTheme();

  ui::DrawPanelInset(r, uiTime, /*active=*/true);

  const int pad = 10;
  const int titleSize = 18;
  const int fontSmall = 13;

  ui::Text(static_cast<int>(r.x) + pad, static_cast<int>(r.y) + 6, titleSize, "RCI demand", th.text,
           /*bold=*/true, /*shadow=*/true, 1);

  const std::size_t n = samples.size();
  if (n < 2) {
    ui::Text(static_cast<int>(r.x) + pad, static_cast<int>(r.y) + 34, 14, "(no data yet)", th.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    return;
  }

  // Fixed 0..1 range.
  const float vmin = 0.0f;
  const float vmax = 1.0f;

  Rectangle gr = r;
  gr.x += static_cast<float>(pad);
  gr.width -= static_cast<float>(pad * 2);
  gr.y += 34.0f;
  gr.height -= 52.0f;

  const int gridLines = 3;
  for (int i = 0; i <= gridLines; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(gridLines);
    const int yy = static_cast<int>(gr.y + t * gr.height);
    DrawLine(static_cast<int>(gr.x), yy, static_cast<int>(gr.x + gr.width), yy, Fade(th.grid, 0.35f));
  }

  auto mapX = [&](std::size_t i) -> float {
    const float t = static_cast<float>(i) / static_cast<float>(n - 1);
    return gr.x + t * gr.width;
  };
  auto mapY = [&](float v) -> float {
    const float t = (v - vmin) / (vmax - vmin);
    return gr.y + (1.0f - std::clamp(t, 0.0f, 1.0f)) * gr.height;
  };

  // SimCity-style colors.
  const Color cR{95, 220, 120, 255};
  const Color cC{95, 180, 255, 255};
  const Color cI{255, 190, 95, 255};

  auto drawSeries = [&](auto getV, Color c) {
    for (std::size_t i = 1; i < n; ++i) {
      const float x0 = mapX(i - 1);
      const float y0 = mapY(getV(samples[i - 1]));
      const float x1 = mapX(i);
      const float y1 = mapY(getV(samples[i]));
      DrawLineEx(Vector2{x0, y0}, Vector2{x1, y1}, 2.0f, c);
    }
  };

  drawSeries([](const CityHistorySample& s) { return s.demandResidential; }, cR);
  drawSeries([](const CityHistorySample& s) { return s.demandCommercial; }, cC);
  drawSeries([](const CityHistorySample& s) { return s.demandIndustrial; }, cI);

  // Legend.
  const int lx = static_cast<int>(r.x + r.width) - pad - 92;
  const int ly = static_cast<int>(r.y) + 10;
  DrawRectangle(lx + 0, ly + 3, 10, 10, cR);
  ui::Text(lx + 14, ly, 14, "R", th.textDim, /*bold=*/true, /*shadow=*/true, 1);
  DrawRectangle(lx + 28, ly + 3, 10, 10, cC);
  ui::Text(lx + 42, ly, 14, "C", th.textDim, /*bold=*/true, /*shadow=*/true, 1);
  DrawRectangle(lx + 56, ly + 3, 10, 10, cI);
  ui::Text(lx + 70, ly, 14, "I", th.textDim, /*bold=*/true, /*shadow=*/true, 1);

  // Latest label.
  const CityHistorySample& last = samples.back();
  char buf[128];
  std::snprintf(buf, sizeof(buf), "Latest: R %.0f%%  C %.0f%%  I %.0f%%",
                static_cast<double>(last.demandResidential * 100.0f),
                static_cast<double>(last.demandCommercial * 100.0f),
                static_cast<double>(last.demandIndustrial * 100.0f));

  ui::Text(static_cast<int>(r.x) + pad, static_cast<int>(r.y + r.height) - 22, fontSmall, buf, th.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
}

constexpr int kReportPages = 5;

const char* ReportPageName(int page)
{
  switch (page) {
    default:
    case 0: return "Overview";
    case 1: return "Economy";
    case 2: return "Traffic";
    case 3: return "Land & Goods";
    case 4: return "Districts";
  }
}

} // namespace

void Game::drawReportPanel(int /*screenW*/, int /*screenH*/)
{
  if (!m_showReport) return;

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  const int panelW = 520;
  const int panelH = 420;
  const int x0 = 12;
  const int y0 = 96;

  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                         static_cast<float>(panelH)};
  ui::DrawPanel(panelR, uiTime, /*active=*/true);
  ui::DrawPanelHeader(panelR, "Report", uiTime, /*active=*/true, /*titleSizePx=*/22);

  // Close button.
  {
    const Rectangle br{panelR.x + panelR.width - 30.0f, panelR.y + 8.0f, 22.0f, 18.0f};
    if (ui::Button(8000, br, "X", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
      m_showReport = false;
      return;
    }
  }

  // Tabs (mouse).
  const int tabH = 18;
  const int gap = 6;
  const int tabW = (panelW - 24 - gap * (kReportPages - 1)) / kReportPages;
  const int tabY = y0 + 42;

  for (int p = 0; p < kReportPages; ++p) {
    const Rectangle br{static_cast<float>(x0 + 12 + p * (tabW + gap)),
                       static_cast<float>(tabY),
                       static_cast<float>(tabW),
                       static_cast<float>(tabH)};
    const bool selected = (m_reportPage == p);
    if (ui::Button(8010 + p, br, ReportPageName(p), mouseUi, uiTime, /*enabled=*/true, /*primary=*/selected)) {
      m_reportPage = p;
    }
  }

  // Display a fixed window: last N days (bounded by stored history).
  const int maxPoints = 120;
  const int count = static_cast<int>(m_cityHistory.size());
  const int start = (count > maxPoints) ? (count - maxPoints) : 0;
  const std::vector<CityHistorySample> view(m_cityHistory.begin() + start, m_cityHistory.end());

  const int graphY = tabY + tabH + 6;
  Rectangle r1{static_cast<float>(x0 + 12), static_cast<float>(graphY), static_cast<float>(panelW - 24), 96};
  Rectangle r2{static_cast<float>(x0 + 12), static_cast<float>(graphY + 104), static_cast<float>(panelW - 24), 96};
  Rectangle r3{static_cast<float>(x0 + 12), static_cast<float>(graphY + 208), static_cast<float>(panelW - 24), 96};

  if (m_reportPage == 0) {
    DrawHistoryGraph(view, r1, "Population", [](const CityHistorySample& s) { return static_cast<float>(s.population); },
                     0.0f, 0.0f, false, "Latest: %.0f");
    DrawHistoryGraph(view, r2, "Happiness", [](const CityHistorySample& s) { return s.happiness; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
    DrawHistoryGraphRCI(view, r3);
  } else if (m_reportPage == 1) {
    DrawHistoryGraph(view, r1, "Money", [](const CityHistorySample& s) { return static_cast<float>(s.money); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
    DrawHistoryGraph(view, r2, "Income", [](const CityHistorySample& s) { return static_cast<float>(s.income); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
    DrawHistoryGraph(view, r3, "Expenses", [](const CityHistorySample& s) { return static_cast<float>(s.expenses); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
  } else if (m_reportPage == 2) {
    DrawHistoryGraph(view, r1, "Commuters", [](const CityHistorySample& s) { return static_cast<float>(s.commuters); }, 0.0f, 0.0f,
                     false, "Latest: %.0f");
    DrawHistoryGraph(view, r2, "Avg commute (time)", [](const CityHistorySample& s) { return s.avgCommuteTime; }, 0.0f, 0.0f,
                     false, "Latest: %.1f");
    DrawHistoryGraph(view, r3, "Congestion", [](const CityHistorySample& s) { return s.trafficCongestion; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
  } else if (m_reportPage == 3) {
    DrawHistoryGraph(view, r1, "Avg land value", [](const CityHistorySample& s) { return s.avgLandValue; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
    DrawHistoryGraph(view, r2, "Tax per capita", [](const CityHistorySample& s) { return s.avgTaxPerCapita; }, 0.0f, 0.0f, false,
                     "Latest: %.2f");
    DrawHistoryGraph(view, r3, "Goods satisfaction", [](const CityHistorySample& s) { return s.goodsSatisfaction; }, 0.0f, 1.0f, true,
                     "Latest: %.0f%%", true);
  } else {
    // Districts table
    const SimConfig& cfg = m_sim.config();
    const int w = m_world.width();
    const int h = m_world.height();
    const int n = w * h;

    const std::vector<float>* lv = (static_cast<int>(m_landValue.value.size()) == n) ? &m_landValue.value : nullptr;
    const DistrictStatsResult ds = ComputeDistrictStats(m_world, cfg, lv, nullptr);

    const int headerY = graphY;
    const int tableX = x0 + 12;
    const int rowH = 20;
    const int font = 15;

    auto drawR = [&](int xRight, int yDraw, std::string_view text, Color c) {
      const int tw = ui::MeasureTextWidth(text, font, /*bold=*/false, 1);
      ui::Text(xRight - tw, yDraw, font, text, c, /*bold=*/false, /*shadow=*/true, 1);
    };

    ui::Text(tableX, headerY, font, "ID", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);
    ui::Text(tableX + 40, headerY, font, "Pop", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);
    ui::Text(tableX + 120, headerY, font, "Emp", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);
    ui::Text(tableX + 200, headerY, font, "Net", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);
    ui::Text(tableX + 280, headerY, font, "LV", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);
    ui::Text(tableX + 350, headerY, font, "Acc", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);

    const int rowStartY = headerY + 20;
    for (int d = 0; d < kDistrictCount; ++d) {
      const DistrictSummary& s = ds.districts[static_cast<std::size_t>(d)];
      const int rowY = rowStartY + d * rowH;

      if (d == std::clamp(m_activeDistrict, 0, kDistrictCount - 1)) {
        const Rectangle rr{static_cast<float>(x0 + 8), static_cast<float>(rowY - 2),
                           static_cast<float>(panelW - 16), static_cast<float>(rowH)};
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/false);
      }

      ui::Text(tableX, rowY, font, TextFormat("%d", d), uiTh.text, /*bold=*/false, /*shadow=*/true, 1);
      drawR(tableX + 40 + 70, rowY, TextFormat("%d", s.population), uiTh.text);
      drawR(tableX + 120 + 70, rowY, TextFormat("%d", s.employed), uiTh.text);
      drawR(tableX + 200 + 70, rowY, TextFormat("%+d", s.net), (s.net < 0) ? uiTh.accentBad : uiTh.accentOk);
      drawR(tableX + 280 + 50, rowY, TextFormat("%.0f%%", static_cast<double>(s.avgLandValue) * 100.0), uiTh.text);
      if (s.zoneTiles > 0) {
        const double accPct = 100.0 * static_cast<double>(s.zoneTilesAccessible) / static_cast<double>(s.zoneTiles);
        drawR(tableX + 350 + 60, rowY, TextFormat("%.0f%%", accPct), uiTh.text);
      } else {
        drawR(tableX + 350 + 60, rowY, "--", uiTh.textFaint);
      }
    }

    // Totals row
    const int totalsY = rowStartY + kDistrictCount * rowH + 8;
    DrawLine(x0 + 10, totalsY - 6, x0 + panelW - 10, totalsY - 6, Fade(uiTh.panelBorderHi, 0.6f));
    ui::Text(tableX, totalsY, font, "All", uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);
    drawR(tableX + 40 + 70, totalsY, TextFormat("%d", ds.total.population), uiTh.textDim);
    drawR(tableX + 120 + 70, totalsY, TextFormat("%d", ds.total.employed), uiTh.textDim);
    drawR(tableX + 200 + 70, totalsY, TextFormat("%+d", ds.total.net), (ds.total.net < 0) ? uiTh.accentBad : uiTh.accentOk);
    drawR(tableX + 280 + 50, totalsY, TextFormat("%.0f%%", static_cast<double>(ds.total.avgLandValue) * 100.0), uiTh.textDim);

    // Detail line for selected district
    const int dSel = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
    const DistrictSummary& sel = ds.districts[static_cast<std::size_t>(dSel)];
    const int detailY = totalsY + 24;
    ui::Text(x0 + 12, detailY, 13,
             TextFormat("D%d: tax %d  maint %d (roads %d, parks %d)", dSel, sel.taxRevenue, sel.maintenanceCost,
                        sel.roadMaintenanceCost, sel.parkMaintenanceCost),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    ui::Text(x0 + 12, detailY + 16, 13, "Note: district budget excludes trade, upgrades, and one-off build costs.",
             uiTh.textFaint, /*bold=*/false, /*shadow=*/true, 1);
  }

  // Footer: day range / help
  if (!view.empty()) {
    const int d0 = view.front().day;
    const int d1 = view.back().day;
    ui::Text(x0 + 12, y0 + panelH - 22, 13,
             TextFormat("Days: %d..%d  (showing %d / stored %d)   Tab: cycle   F1: toggle",
                        d0, d1, static_cast<int>(view.size()), static_cast<int>(m_cityHistory.size())),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  }
}

void Game::drawVideoSettingsPanel(int uiW, int uiH)
{
  if (!m_showVideoSettings) return;

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();

  const int panelMaxW = 620;
  const int panelMinW = 420;
  const int availW = std::max(0, uiW - 24);
  const int panelW = std::max(std::min(panelMaxW, availW), std::min(panelMinW, availW));
  const int rowH = 22;
  const int rows = (m_videoPage == 0)   ? 11
                   : (m_videoPage == 1) ? 57
                   : (m_videoPage == 2) ? 19
                                        : 11;
  const int panelH = 64 + rows * rowH + 28;

  const int x0 = 12;
  int y0 = 96;

  // Avoid overlapping the report panel (which also lives on the left).
  if (m_showReport) {
    y0 += 420 + 12;
  }

  // Clamp to screen height.
  if (y0 + panelH > uiH - 12) {
    y0 = std::max(12, uiH - panelH - 12);
  }

  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                         static_cast<float>(panelH)};
  ui::DrawPanel(panelR, uiTime, /*active=*/true);

  const char* title = (m_videoPage == 0)   ? "Video / Display"
                      : (m_videoPage == 1) ? "Video / Visual FX"
                      : (m_videoPage == 2) ? "Video / Atmosphere"
                                           : "Video / UI Theme";
  ui::DrawPanelHeader(panelR, title, uiTime, /*active=*/true, /*titleSizePx=*/22);

  const int x = x0 + 12;
  int y = y0 + 42;

  ui::Text(x, y, 15,
           "Tab: select  |  [ / ]: adjust/toggle  |  Shift: coarse  |  F8: close  |  Shift+F8: next page  |  Ctrl+F8: UI Theme  |  Ctrl+Shift+F8: Atmosphere",
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 22;

  // List inset
  const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                        static_cast<float>(rows * rowH + 8)};
  ui::DrawPanelInset(listR, uiTime, /*active=*/true);

  const Vector2 mouseUi = mouseUiPosition(m_uiScale);
  int rowY = y + 4;

  auto rowRectFor = [&](int yRow) -> Rectangle {
    return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                     static_cast<float>(rowH)};
  };

  auto drawRow = [&](int idx, std::string_view label, std::string_view value, bool dim = false) {
    const bool selected = (m_videoSelection == idx);
    const Rectangle rr = rowRectFor(rowY);

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
      m_videoSelection = idx;
      if (m_videoPage == 0) m_videoSelectionDisplay = idx;
      if (m_videoPage == 1) m_videoSelectionVisual = idx;
      if (m_videoPage == 2) m_videoSelectionAtmos = idx;
      if (m_videoPage == 3) m_videoSelectionUiTheme = idx;
    }

    if (selected) {
      ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
    }

    Color c = dim ? uiTh.textFaint : uiTh.textDim;
    if (selected) {
      c = uiTh.text;
    }

    ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);
    const int valW = ui::MeasureTextWidth(value, 16, /*bold=*/false, 1);
    ui::Text(x0 + panelW - 12 - valW, rowY, 16, value, c, /*bold=*/false, /*shadow=*/true, 1);
    rowY += rowH;
  };

  if (m_videoPage == 0) {
    // 0..10 must match adjustVideoSettings() and Tab cycling.
    drawRow(0, "Fullscreen", IsWindowFullscreen() ? "On" : "Off");
    drawRow(1, "Borderless windowed", m_borderlessWindowed ? "On" : "Off");
    drawRow(2, "VSync", m_cfg.vsync ? "On" : "Off");
    drawRow(3, "UI scale mode", m_uiScaleAuto ? "Auto" : "Manual");
    drawRow(4, "UI scale", TextFormat("%.2fx", m_uiScale), !m_uiScaleAuto);

    drawRow(5, "World render mode", m_worldRenderScaleAuto ? "Auto" : "Manual");
    drawRow(6, "World render scale", TextFormat("%.0f%%", m_worldRenderScale * 100.0f), m_worldRenderScaleAuto);
    drawRow(7, "World scale min", TextFormat("%.0f%%", m_worldRenderScaleMin * 100.0f), !m_worldRenderScaleAuto);
    drawRow(8, "World scale max", TextFormat("%.0f%%", m_worldRenderScaleMax * 100.0f), !m_worldRenderScaleAuto);
    drawRow(9, "World target FPS", TextFormat("%d", m_worldRenderTargetFps), !m_worldRenderScaleAuto);
    drawRow(10, "World filter", m_worldRenderFilterPoint ? "Point" : "Bilinear");
  } else if (m_videoPage == 1) {
    const Renderer::ShadowSettings sh = m_renderer.shadowSettings();
    const Renderer::DayNightSettings dn = m_renderer.dayNightSettings();
    const Renderer::WeatherSettings wx = m_renderer.weatherSettings();

    auto onOff = [](bool v) -> const char* { return v ? "On" : "Off"; };

    auto weatherModeStr = [](Renderer::WeatherSettings::Mode m) -> const char* {
      using M = Renderer::WeatherSettings::Mode;
      switch (m) {
        case M::Rain: return "Rain";
        case M::Snow: return "Snow";
        default: return "Clear";
      }
    };

    auto gfxThemeStr = [](GfxTheme t) -> const char* {
      switch (t) {
        case GfxTheme::Classic: return "Classic";
        case GfxTheme::Autumn: return "Autumn";
        case GfxTheme::Desert: return "Desert";
        case GfxTheme::Noir: return "Noir";
        case GfxTheme::Neon: return "Neon";
        case GfxTheme::Pastel: return "Pastel";
        case GfxTheme::SpaceColony: return "Space Colony";
        case GfxTheme::Procedural: return "Procedural";
        case GfxTheme::ProceduralMuted: return "Procedural (Muted)";
        case GfxTheme::ProceduralVibrant: return "Procedural (Vibrant)";
        default: return "Classic";
      }
    };

    drawRow(0, "Render cache (banded)", onOff(m_renderer.baseCacheEnabled()));

    drawRow(1, "Layer: Terrain", onOff(m_renderer.layerEnabled(Renderer::RenderLayer::Terrain)));
    drawRow(2, "Layer: Decals", onOff(m_renderer.layerEnabled(Renderer::RenderLayer::Decals)));
    drawRow(3, "Layer: Structures", onOff(m_renderer.layerEnabled(Renderer::RenderLayer::Structures)));
    drawRow(4, "Layer: Overlays", onOff(m_renderer.layerEnabled(Renderer::RenderLayer::Overlays)));

    drawRow(5, "Merged zone buildings", onOff(m_mergedZoneBuildings));

    drawRow(6, "Shadows", onOff(sh.enabled));
    drawRow(7, "Shadow strength", TextFormat("%.0f%%", sh.strength * 100.0f), !sh.enabled);
    drawRow(8, "Shadow softness", TextFormat("%.0f%%", sh.softness * 100.0f), !sh.enabled);
    drawRow(9, "Shadow max len", TextFormat("%.1f tiles", sh.maxLengthTiles), !sh.enabled);
    drawRow(10, "Shadow direction", TextFormat("%.0f°", sh.azimuthDeg), !sh.enabled);

    drawRow(11, "Day/night cycle", onOff(dn.enabled));
    drawRow(12, "Day length", TextFormat("%.0f s", dn.dayLengthSec), !dn.enabled);
    drawRow(13, "Time offset", TextFormat("%.0f s", dn.timeOffsetSec), !dn.enabled);
    drawRow(14, "Night darken", TextFormat("%.0f%%", dn.nightDarken * 100.0f), !dn.enabled);
    drawRow(15, "Dusk tint", TextFormat("%.0f%%", dn.duskTint * 100.0f), !dn.enabled);
    drawRow(16, "Night lights", onOff(dn.drawLights), !dn.enabled);

    drawRow(17, "Weather mode", weatherModeStr(wx.mode));
    drawRow(18, "Intensity", TextFormat("%.0f%%", wx.intensity * 100.0f), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(19, "Wind dir", TextFormat("%.0f°", wx.windAngleDeg), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(20, "Wind speed", TextFormat("%.2fx", wx.windSpeed), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(21, "Overcast", TextFormat("%.0f%%", wx.overcast * 100.0f), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(22, "Fog", TextFormat("%.0f%%", wx.fog * 100.0f), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(23, "Particles", onOff(wx.drawParticles), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(24, "Ground effects", onOff(wx.affectGround), wx.mode == Renderer::WeatherSettings::Mode::Clear);
    drawRow(25, "Reflect lights", onOff(wx.reflectLights),
            (wx.mode != Renderer::WeatherSettings::Mode::Rain) || (wx.mode == Renderer::WeatherSettings::Mode::Clear));

    // Post FX (stylized)
    const char* postFxStatus =
      (!m_postFx.enabled) ? "Off" : (m_postFxPipeline.isReady() ? "On" : "On (fallback)");
    drawRow(26, "Post FX", postFxStatus);
    drawRow(27, "Color bits", TextFormat("%d", m_postFx.colorBits), !m_postFx.enabled);
    drawRow(28, "Dither strength", TextFormat("%.0f%%", m_postFx.ditherStrength * 100.0f), !m_postFx.enabled);
    drawRow(29, "Film grain", TextFormat("%.0f%%", m_postFx.grain * 100.0f), !m_postFx.enabled);
    drawRow(30, "Vignette", TextFormat("%.0f%%", m_postFx.vignette * 100.0f), !m_postFx.enabled);
    drawRow(31, "Chroma aberration", TextFormat("%.0f%%", m_postFx.chroma * 100.0f), !m_postFx.enabled);
    drawRow(32, "Scanlines", TextFormat("%.0f%%", m_postFx.scanlines * 100.0f), !m_postFx.enabled);
    drawRow(33, "FXAA", TextFormat("%.0f%%", m_postFx.fxaa * 100.0f), !m_postFx.enabled);
    drawRow(34, "Sharpen", TextFormat("%.0f%%", m_postFx.sharpen * 100.0f), !m_postFx.enabled);

    const char* taaStatus =
      (!m_postFx.taaEnabled) ? "Off" : (m_postFxPipeline.taaReady() ? "On" : "On (fallback)");
    drawRow(35, "Temporal AA", taaStatus, !m_postFx.enabled);
    drawRow(36, "TAA stability", TextFormat("%.0f%%", m_postFx.taaHistory * 100.0f),
            (!m_postFx.enabled || !m_postFx.taaEnabled));
    drawRow(37, "TAA jitter", TextFormat("%.0f%%", m_postFx.taaJitter * 100.0f),
            (!m_postFx.enabled || !m_postFx.taaEnabled));
    drawRow(38, "TAA response", TextFormat("%.0f%%", m_postFx.taaResponse * 100.0f),
            (!m_postFx.enabled || !m_postFx.taaEnabled));

    drawRow(39, "Include weather", onOff(m_postFx.includeWeather), !m_postFx.enabled);
    drawRow(40, "Bloom", TextFormat("%.0f%%", m_postFx.bloom * 100.0f), !m_postFx.enabled);
    drawRow(41, "Bloom threshold", TextFormat("%.0f%%", m_postFx.bloomThreshold * 100.0f), !m_postFx.enabled);
    drawRow(42, "Bloom radius", TextFormat("%.2fx", m_postFx.bloomRadius), !m_postFx.enabled);
    drawRow(43, "Bloom downsample", TextFormat("%dx", m_postFx.bloomDownsample), !m_postFx.enabled);

    // Filmic tonemap + grade
    drawRow(44, "Filmic tonemap", onOff(m_postFx.tonemapEnabled), !m_postFx.enabled);
    drawRow(45, "Exposure", TextFormat("%.2fx", m_postFx.exposure),
            (!m_postFx.enabled || !m_postFx.tonemapEnabled));
    drawRow(46, "Contrast", TextFormat("%.2fx", m_postFx.contrast),
            (!m_postFx.enabled || !m_postFx.tonemapEnabled));
    drawRow(47, "Saturation", TextFormat("%.2fx", m_postFx.saturation),
            (!m_postFx.enabled || !m_postFx.tonemapEnabled));

    // Screen-space outlines
    drawRow(48, "Outlines", TextFormat("%.0f%%", m_postFx.outline * 100.0f), !m_postFx.enabled);
    drawRow(49, "Outline threshold", TextFormat("%.0f%%", m_postFx.outlineThreshold * 100.0f),
            (!m_postFx.enabled || m_postFx.outline < 0.001f));
    drawRow(50, "Outline thickness", TextFormat("%.2f px", m_postFx.outlineThickness),
            (!m_postFx.enabled || m_postFx.outline < 0.001f));

    // Lens precipitation (rain on lens / wet camera)
    drawRow(51, "Lens rain", TextFormat("%.0f%%", m_postFx.lensWeather * 100.0f), !m_postFx.enabled);
    drawRow(52, "Lens distort", TextFormat("%.0f%%", m_postFx.lensDistort * 100.0f),
            (!m_postFx.enabled || m_postFx.lensWeather < 0.001f));
    drawRow(53, "Lens scale", TextFormat("%.2fx", m_postFx.lensScale),
            (!m_postFx.enabled || m_postFx.lensWeather < 0.001f));
    drawRow(54, "Lens drips", TextFormat("%.0f%%", m_postFx.lensDrips * 100.0f),
            (!m_postFx.enabled || m_postFx.lensWeather < 0.001f));

    drawRow(55, "Reset Post FX", "Click / ]");

    drawRow(56, "Gfx theme", gfxThemeStr(m_renderer.gfxTheme()));
  } else if (m_videoPage == 2) {
    // ----------------------------
    // Atmosphere page
    // ----------------------------
    Renderer::CloudShadowSettings cs = m_renderer.cloudShadowSettings();
    Renderer::VolumetricCloudSettings vc = m_renderer.volumetricCloudSettings();
    bool changed = false;

    auto onOff = [](bool v) -> const char* { return v ? "On" : "Off"; };

    const int xLabel = x;
    const int xValueRight = x0 + panelW - 12;
    const int xSlider = x0 + 270;
    const int sliderW = std::max(80, (xValueRight - 90) - xSlider);
    const int sliderH = 12;

    auto drawSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax,
                             std::string_view valueText, bool enabled = true) {
      const bool selected = (m_videoSelection == idx);
      const Rectangle rr = rowRectFor(rowY);

      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_videoSelection = idx;
        m_videoSelectionAtmos = idx;
      }

      if (selected) {
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
      }

      Color c = enabled ? uiTh.textDim : uiTh.textFaint;
      if (selected) c = enabled ? uiTh.text : uiTh.textFaint;

      ui::Text(xLabel, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      const Rectangle sr{static_cast<float>(xSlider), static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW), static_cast<float>(sliderH)};
      if (ui::SliderFloat(4200 + idx, sr, v, vMin, vMax, mouseUi, uiTime, enabled)) {
        changed = true;
      }

      const int valW = ui::MeasureTextWidth(valueText, 16, /*bold=*/false, 1);
      ui::Text(xValueRight - valW, rowY, 16, valueText, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    auto drawToggleRow = [&](int idx, std::string_view label, bool& v, std::string_view valueText,
                             bool enabled = true) {
      const bool selected = (m_videoSelection == idx);
      const Rectangle rr = rowRectFor(rowY);

      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_videoSelection = idx;
        m_videoSelectionAtmos = idx;
      }

      if (selected) {
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
      }

      Color c = enabled ? uiTh.textDim : uiTh.textFaint;
      if (selected) c = enabled ? uiTh.text : uiTh.textFaint;

      ui::Text(xLabel, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      const Rectangle tr{static_cast<float>(xValueRight - 44), static_cast<float>(rowY + 3), 44.0f,
                         static_cast<float>(rowH - 6)};
      if (ui::Toggle(4000 + idx, tr, v, mouseUi, uiTime, enabled)) {
        changed = true;
      }

      const int valW = ui::MeasureTextWidth(valueText, 16, /*bold=*/false, 1);
      ui::Text(xValueRight - 52 - valW, rowY, 16, valueText, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    // Row indices must match adjustVideoSettings() Atmosphere page cases (0..18).
    drawToggleRow(0, "Cloud shadows", cs.enabled, onOff(cs.enabled));
    drawSliderRow(1, "Shadow strength", cs.strength, 0.0f, 1.0f, TextFormat("%.0f%%", cs.strength * 100.0f), cs.enabled);
    drawSliderRow(2, "Shadow scale", cs.scale, 0.25f, 8.0f, TextFormat("%.2fx", cs.scale), cs.enabled);
    drawSliderRow(3, "Shadow speed", cs.speed, 0.0f, 5.0f, TextFormat("%.2fx", cs.speed), cs.enabled);
    drawSliderRow(4, "Shadow evolve", cs.evolve, 0.0f, 1.0f, TextFormat("%.0f%%", cs.evolve * 100.0f), cs.enabled);
    drawSliderRow(5, "Shadow coverage", cs.coverage, 0.0f, 1.0f, TextFormat("%.0f%%", cs.coverage * 100.0f), cs.enabled);
    drawSliderRow(6, "Shadow softness", cs.softness, 0.0f, 1.0f, TextFormat("%.0f%%", cs.softness * 100.0f), cs.enabled);
    drawSliderRow(7, "Clear sky", cs.clearAmount, 0.0f, 1.0f, TextFormat("%.0f%%", cs.clearAmount * 100.0f), cs.enabled);

    drawToggleRow(8, "Volumetric clouds", vc.enabled, onOff(vc.enabled));
    drawSliderRow(9, "Cloud opacity", vc.opacity, 0.0f, 1.0f, TextFormat("%.0f%%", vc.opacity * 100.0f), vc.enabled);
    drawSliderRow(10, "Cloud coverage", vc.coverage, 0.0f, 1.0f, TextFormat("%.0f%%", vc.coverage * 100.0f), vc.enabled);
    drawSliderRow(11, "Cloud density", vc.density, 0.0f, 2.0f, TextFormat("%.2fx", vc.density), vc.enabled);
    drawSliderRow(12, "Cloud scale", vc.scale, 0.25f, 8.0f, TextFormat("%.2fx", vc.scale), vc.enabled);
    drawSliderRow(13, "Cloud speed", vc.speed, 0.0f, 5.0f, TextFormat("%.2fx", vc.speed), vc.enabled);
    drawSliderRow(14, "Cloud softness", vc.softness, 0.0f, 1.0f, TextFormat("%.0f%%", vc.softness * 100.0f), vc.enabled);
    {
      float stepsF = static_cast<float>(vc.steps);
      drawSliderRow(15, "Cloud steps", stepsF, 8.0f, 64.0f, TextFormat("%d", vc.steps), vc.enabled);
      vc.steps = std::clamp(static_cast<int>(std::lround(stepsF)), 8, 64);
    }
    drawSliderRow(16, "Bottom fade", vc.bottomFade, 0.0f, 1.0f, TextFormat("%.0f%%", vc.bottomFade * 100.0f), vc.enabled);
    drawSliderRow(17, "Clear sky", vc.clearAmount, 0.0f, 1.0f, TextFormat("%.0f%%", vc.clearAmount * 100.0f), vc.enabled);

    // Reset row (clickable).
    {
      const int idx = 18;
      const bool selected = (m_videoSelection == idx);
      const Rectangle rr = rowRectFor(rowY);
      if (selected) {
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
      }

      const bool hovered = CheckCollisionPointRec(mouseUi, rr);
      const bool click = hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
      if (click) {
        cs = Renderer::CloudShadowSettings{};
        vc = Renderer::VolumetricCloudSettings{};
        m_renderer.setCloudShadowSettings(cs);
        m_renderer.setVolumetricCloudSettings(vc);
        changed = false;
        m_videoSelection = idx;
        m_videoSelectionAtmos = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(xLabel, rowY, 16, "Reset atmosphere", c, /*bold=*/false, /*shadow=*/true, 1);
      const char* val = "Click / ]";
      const int valW = ui::MeasureTextWidth(val, 16, /*bold=*/false, 1);
      ui::Text(xValueRight - valW, rowY, 16, val, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    }

    if (changed) {
      m_renderer.setCloudShadowSettings(cs);
      m_renderer.setVolumetricCloudSettings(vc);
    }
  } else {
    // ----------------------------
    // UI Theme page
    // ----------------------------
    ui::Settings s = ui::GetSettings();
    bool changed = false;

    const int xLabel = x;
    const int xValueRight = x0 + panelW - 12;
    const int xSlider = x0 + 260;
    const int sliderW = std::max(80, (xValueRight - 90) - xSlider);
    const int sliderH = 12;

    auto drawSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax, std::string_view valueText,
                             bool enabled = true) {
      const bool selected = (m_videoSelection == idx);
      const Rectangle rr = rowRectFor(rowY);

      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_videoSelection = idx;
        m_videoSelectionUiTheme = idx;
      }

      if (selected) {
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
      }

      Color c = enabled ? uiTh.textDim : uiTh.textFaint;
      if (selected) c = enabled ? uiTh.text : uiTh.textFaint;

      ui::Text(xLabel, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      const Rectangle sr{static_cast<float>(xSlider), static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW), static_cast<float>(sliderH)};
      if (ui::SliderFloat(3100 + idx, sr, v, vMin, vMax, mouseUi, uiTime, enabled)) {
        changed = true;
      }

      const int valW = ui::MeasureTextWidth(valueText, 16, /*bold=*/false, 1);
      ui::Text(xValueRight - valW, rowY, 16, valueText, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    auto drawToggleRow = [&](int idx, std::string_view label, bool& v, std::string_view valueText) {
      const bool selected = (m_videoSelection == idx);
      const Rectangle rr = rowRectFor(rowY);

      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_videoSelection = idx;
        m_videoSelectionUiTheme = idx;
      }

      if (selected) {
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
      }

      Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(xLabel, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      const Rectangle tr{static_cast<float>(xValueRight - 44), static_cast<float>(rowY + 3), 44.0f,
                         static_cast<float>(rowH - 6)};
      if (ui::Toggle(3000 + idx, tr, v, mouseUi, uiTime, /*enabled=*/true)) {
        changed = true;
      }

      const int valW = ui::MeasureTextWidth(valueText, 16, /*bold=*/false, 1);
      ui::Text(xValueRight - 52 - valW, rowY, 16, valueText, c, /*bold=*/false, /*shadow=*/true, 1);

      rowY += rowH;
    };

    // Row indices must match adjustVideoSettings() cases (0..10).
    drawToggleRow(0, "Accent", s.accentFromSeed, s.accentFromSeed ? "Seed" : "Manual");

    {
      std::string hueText = s.accentFromSeed ? std::string("--") : TextFormat("%.0f°", s.accentHueDeg);
      drawSliderRow(1, "Hue", s.accentHueDeg, 0.0f, 360.0f, hueText, !s.accentFromSeed);
    }
    {
      std::string satText = TextFormat("%.0f%%", s.accentSaturation * 100.0f);
      drawSliderRow(2, "Saturation", s.accentSaturation, 0.0f, 1.0f, satText, true);
    }
    {
      std::string valText = TextFormat("%.0f%%", s.accentValue * 100.0f);
      drawSliderRow(3, "Brightness", s.accentValue, 0.0f, 1.0f, valText, true);
    }
    {
      std::string roundText = TextFormat("%.0f%%", s.roundness * 100.0f);
      drawSliderRow(4, "Roundness", s.roundness, 0.0f, 0.45f, roundText, true);
    }
    {
      std::string grainText = TextFormat("%.0f%%", (s.noiseAlpha / 0.25f) * 100.0f);
      float grainNorm = s.noiseAlpha;
      drawSliderRow(5, "Grain", grainNorm, 0.0f, 0.25f, grainText, true);
      s.noiseAlpha = grainNorm;
    }
    {
      std::string scaleText = TextFormat("%.2fx", s.noiseScale);
      drawSliderRow(6, "Grain scale", s.noiseScale, 0.10f, 2.00f, scaleText, true);
    }
    {
      std::string sheenText = TextFormat("%.0f%%", s.headerSheenStrength * 100.0f);
      drawSliderRow(7, "Header sheen", s.headerSheenStrength, 0.0f, 1.0f, sheenText, true);
    }
    {
      float q = static_cast<float>(s.fontAtlasScale);
      std::string qText = TextFormat("%dx", s.fontAtlasScale);
      drawSliderRow(8, "Font quality", q, 2.0f, 6.0f, qText, true);
      s.fontAtlasScale = std::clamp(static_cast<int>(std::lround(q)), 2, 6);
    }
    drawToggleRow(9, "Font style", s.fontFilterPoint, s.fontFilterPoint ? "Pixel" : "Smooth (SDF)");

    // Reset row (clickable).
    {
      const int idx = 10;
      const bool selected = (m_videoSelection == idx);
      const Rectangle rr = rowRectFor(rowY);
      if (selected) {
        ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);
      }

      const bool hovered = CheckCollisionPointRec(mouseUi, rr);
      const bool click = hovered && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
      if (click) {
        ui::ResetSettings();
        s = ui::GetSettings();
        changed = false;
      }

      if (click) {
        m_videoSelection = idx;
        m_videoSelectionUiTheme = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(xLabel, rowY, 16, "Reset UI theme", c, /*bold=*/false, /*shadow=*/true, 1);
      const char* val = "Click / ]";
      const int valW = ui::MeasureTextWidth(val, 16, /*bold=*/false, 1);
      ui::Text(xValueRight - valW, rowY, 16, val, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    }

    if (changed) {
      ui::SetSettings(s);
    }
  }

  // Footer: show current effective world RT size and smoothed FPS.
  const float fps = 1.0f / std::max(0.0001f, m_frameTimeSmoothed);
  const char* rtStr = wantsWorldRenderTarget() ? TextFormat("%dx%d", m_worldRenderRTWidth, m_worldRenderRTHeight) : "native";
  ui::Text(x0 + 12, y0 + panelH - 22, 14, TextFormat("Smoothed FPS: %.1f    World RT: %s", fps, rtStr), uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
}



static void DrawSpaceBackground(int screenW, int screenH, std::uint64_t seed, float t)
{
  // Deep-space gradient base.
  ClearBackground(Color{5, 6, 10, 255});
  DrawRectangleGradientV(0, 0, screenW, screenH, Color{12, 14, 24, 255}, Color{0, 0, 0, 255});

  // A faint planet/moon disk to give depth.
  const int r = static_cast<int>(0.18f * static_cast<float>(std::min(screenW, screenH)));
  const int cx = static_cast<int>(0.82f * static_cast<float>(screenW));
  const int cy = static_cast<int>(0.22f * static_cast<float>(screenH));
  DrawCircleGradient(cx, cy, static_cast<float>(r), Color{80, 86, 96, 255}, Color{30, 34, 40, 255});
  DrawCircle(cx + static_cast<int>(r * 0.22f), cy + static_cast<int>(r * 0.16f), static_cast<float>(r) * 0.98f, Color{0, 0, 0, 115});

  // Procedural stars (deterministic per world seed, subtly twinkling).
  const int target = std::clamp((screenW * screenH) / 9000, 120, 420);
  RNG rng(seed ^ 0x51A7F11Eull);
  for (int i = 0; i < target; ++i) {
    const int x = static_cast<int>(rng.uniform01() * static_cast<float>(screenW));
    const int y = static_cast<int>(rng.uniform01() * static_cast<float>(screenH));

    // Slight tint variety.
    const float tint = rng.uniform01();
    const unsigned char rC = static_cast<unsigned char>(std::clamp(235.0f + 20.0f * tint, 0.0f, 255.0f));
    const unsigned char gC = static_cast<unsigned char>(std::clamp(235.0f + 10.0f * (1.0f - tint), 0.0f, 255.0f));
    const unsigned char bC = static_cast<unsigned char>(std::clamp(240.0f + 25.0f * (1.0f - tint), 0.0f, 255.0f));

    // Twinkle.
    const float freq = 0.35f + 1.75f * rng.uniform01();
    const float phase = rng.uniform01() * 6.2831853f;
    const float tw = 0.55f + 0.45f * std::sin(t * freq + phase);
    const unsigned char a = static_cast<unsigned char>(std::clamp(60.0f + 190.0f * tw, 0.0f, 255.0f));

    const int size = ((rng.nextU32() & 0x7u) == 0u) ? 2 : 1;
    DrawRectangle(x, y, size, size, Color{rC, gC, bC, a});
  }
}

void Game::drawPerfOverlay(int uiW, int uiH, float uiTime)
{
  if (!m_showPerfOverlay) return;

  // The console already provides a lot of diagnostic text; avoid stacking.
  if (m_console.isOpen()) return;

  const ui::Theme& th = ui::GetTheme();

  const int padOuter = 10;
  const int padInner = 10;
  const int fontTitle = 16;
  const int fontText = 13;
  const int lineH = 16;

  // Build lines.
  std::vector<std::string> lines;
  lines.reserve(6);

  const float dtMs = m_loopStats.dtFrame * 1000.0f;
  const float dtRawMs = m_loopStats.dtRaw * 1000.0f;
  const float fps = (m_loopStats.dtFrame > 1.0e-6f) ? (1.0f / m_loopStats.dtFrame) : 0.0f;

  lines.push_back(std::string(TextFormat("dt: %.2fms (raw %.2fms)%s | fps: %.1f",
                                         static_cast<double>(dtMs), static_cast<double>(dtRawMs),
                                         m_loopStats.dtClamped ? " CLAMP" : "", static_cast<double>(fps))));
  lines.push_back(std::string(TextFormat("CPU ms: in %.2f  up %.2f  draw %.2f  tot %.2f",
                                         m_loopStats.cpuInputMs, m_loopStats.cpuUpdateMs, m_loopStats.cpuDrawMs,
                                         m_loopStats.cpuFrameMs)));

  const char* mode = (!m_loopCfg.simTickLimiter) ? "UNLIM" : (m_loopCfg.simTickAuto ? "AUTO" : "FIX");
  lines.push_back(std::string(TextFormat("Sim: dt %.1fms | ticks %d (limit %d, %s)",
                                         static_cast<double>(m_loopStats.dtSim * 1000.0f), m_loopStats.simTicks,
                                         m_loopStats.simTickLimit, mode)));
  lines.push_back(std::string(TextFormat("Backlog: %d ticks (%.2fs)",
                                         m_loopStats.simBacklogTicks, static_cast<double>(m_loopStats.simBacklogSec))));
  lines.push_back(std::string(TextFormat("Sim CPU: %.2fms | tick %.2fms (ema %.2fms)",
                                         m_loopStats.simCpuMs, m_loopStats.simMsPerTick, m_loopStats.simMsPerTickEma)));
  std::string pauseStr = "no";
  if (simPaused()) {
    if (m_simPausedUser && m_simPausedAuto) pauseStr = "user+auto";
    else if (m_simPausedUser) pauseStr = "user";
    else if (m_simPausedAuto) pauseStr = "auto";
    else pauseStr = "yes";
  }

  lines.push_back(std::string(TextFormat("Focus: %s%s%s | fps cap %d | pause %s  (Ctrl+F3)",
                                         m_loopStats.focused ? "yes" : "no",
                                         m_loopStats.minimized ? " minimized" : "",
                                         m_loopStats.focusPaused ? " PAUSED" : "",
                                         m_loopStats.targetFps, pauseStr.c_str())));

  // Compute box size.
  int maxLineW = ui::MeasureTextWidth("PERF", fontTitle, /*bold=*/true, 1);
  for (const std::string& s : lines) {
    maxLineW = std::max(maxLineW, ui::MeasureTextWidth(s, fontText, /*bold=*/false, 1));
  }

  const int boxW = maxLineW + padInner * 2;
  const int boxH = 8 + 20 + static_cast<int>(lines.size()) * lineH + 8;

  int x = padOuter;
  int y = padOuter;

  // Avoid overlapping the POV HUD when active.
  if (m_pov.isActive()) y += 60;

  // Keep within UI bounds.
  if (x + boxW > uiW - padOuter) x = std::max(padOuter, uiW - padOuter - boxW);
  if (y + boxH > uiH - padOuter) y = std::max(padOuter, uiH - padOuter - boxH);

  Rectangle r{static_cast<float>(x), static_cast<float>(y), static_cast<float>(boxW),
              static_cast<float>(boxH)};
  ui::DrawPanel(r, uiTime, /*active=*/true);

  int tx = x + padInner;
  int ty = y + 8;
  ui::Text(tx, ty, fontTitle, "PERF", th.text, /*bold=*/true, /*shadow=*/true, 1);
  ty += 20;

  for (const std::string& s : lines) {
    ui::Text(tx, ty, fontText, s, th.textDim, /*bold=*/false, /*shadow=*/true, 1);
    ty += lineH;
  }
}


void Game::draw()
{
  BeginDrawing();

  const int screenW = GetScreenWidth();
  const int screenH = GetScreenHeight();
  if (m_renderer.gfxTheme() == GfxTheme::SpaceColony) {
    DrawSpaceBackground(screenW, screenH, m_world.seed(), static_cast<float>(GetTime()));
  } else {
    ClearBackground(Color{30, 32, 38, 255});
  }
  const float uiScale = m_uiScale;
  const int uiW = static_cast<int>(std::round(static_cast<float>(screenW) / uiScale));
  const int uiH = static_cast<int>(std::round(static_cast<float>(screenH) / uiScale));

  // World highlights: either the inspect path OR the road-drag preview (if active).
  std::optional<Point> selected = m_inspectSelected;
  const std::vector<Point>* pathPtr = (!m_inspectPath.empty()) ? &m_inspectPath : nullptr;
  int worldBrush = m_brushRadius;

  if (m_roadDragActive) {
    selected = m_roadDragStart;
    pathPtr = (m_roadDragValid && !m_roadDragPath.empty()) ? &m_roadDragPath : nullptr;
    // In drag-path mode, the brush outline is misleading (we're not painting a diamond).
    worldBrush = 0;
  }

  // If neither inspect nor road-drag is active, we can optionally visualize an active navigation
  // route computed via the dev console `route` command.
  if (!m_roadDragActive && pathPtr == nullptr && m_showWayfindingOverlay && m_wayfindingRoute.ok) {
    selected = m_wayfindingRoute.from.roadTile;
    if (m_wayfindingFocusManeuver >= 0 && !m_wayfindingFocusPath.empty()) {
      pathPtr = &m_wayfindingFocusPath;
    } else if (!m_wayfindingRoute.pathTiles.empty()) {
      pathPtr = &m_wayfindingRoute.pathTiles;
    }
  }

  const bool heatmapActive = (m_heatmapOverlay != HeatmapOverlay::Off);

  auto isLandValueHeatmap = [](HeatmapOverlay h) -> bool {
    return (h == HeatmapOverlay::LandValue) ||
           (h == HeatmapOverlay::ParkAmenity) ||
           (h == HeatmapOverlay::WaterAmenity) ||
           (h == HeatmapOverlay::Pollution) ||
           (h == HeatmapOverlay::TrafficSpill);
  };

  const bool landValueHeatmapActive = heatmapActive && isLandValueHeatmap(m_heatmapOverlay);
  const bool servicesHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::ServicesOverall ||
                        m_heatmapOverlay == HeatmapOverlay::ServicesEducation ||
                        m_heatmapOverlay == HeatmapOverlay::ServicesHealth ||
                        m_heatmapOverlay == HeatmapOverlay::ServicesSafety);

  const bool airHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::AirPollution ||
                        m_heatmapOverlay == HeatmapOverlay::AirPollutionEmission);

  const bool skyViewHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::SkyView ||
                        m_heatmapOverlay == HeatmapOverlay::CanyonConfinement);

  const bool solarHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::SolarExposure ||
                        m_heatmapOverlay == HeatmapOverlay::SolarPotential ||
                        m_heatmapOverlay == HeatmapOverlay::EnergyDemand ||
                        m_heatmapOverlay == HeatmapOverlay::EnergySolar ||
                        m_heatmapOverlay == HeatmapOverlay::EnergyBalance ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonEmission ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonSequestration ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonBalance);

  const bool energyHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::EnergyDemand ||
                        m_heatmapOverlay == HeatmapOverlay::EnergySolar ||
                        m_heatmapOverlay == HeatmapOverlay::EnergyBalance ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonEmission ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonSequestration ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonBalance);

  const bool carbonHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::CarbonEmission ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonSequestration ||
                        m_heatmapOverlay == HeatmapOverlay::CarbonBalance);

  const bool crimeHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::CrimeRisk ||
                        m_heatmapOverlay == HeatmapOverlay::PoliceAccess);

  const bool trafficSafetyHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::TrafficCrashRisk ||
                        m_heatmapOverlay == HeatmapOverlay::TrafficCrashExposure ||
                        m_heatmapOverlay == HeatmapOverlay::TrafficCrashPriority);

  const bool needSkyViewResult = skyViewHeatmapActive || trafficSafetyHeatmapActive;

  const bool transitHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::TransitAccess ||
                        m_heatmapOverlay == HeatmapOverlay::TransitModeSharePotential);

  const bool walkHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::Walkability ||
                        m_heatmapOverlay == HeatmapOverlay::WalkabilityPark ||
                        m_heatmapOverlay == HeatmapOverlay::WalkabilityRetail ||
                        m_heatmapOverlay == HeatmapOverlay::WalkabilityEducation ||
                        m_heatmapOverlay == HeatmapOverlay::WalkabilityHealth ||
                        m_heatmapOverlay == HeatmapOverlay::WalkabilitySafety);

  const bool jobsHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::JobAccess ||
                        m_heatmapOverlay == HeatmapOverlay::JobOpportunity);

  const bool jobsNeeded = jobsHeatmapActive || crimeHeatmapActive;

  const bool roadHealthHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::RoadCentrality ||
                        m_heatmapOverlay == HeatmapOverlay::RoadVulnerability ||
                        m_heatmapOverlay == HeatmapOverlay::RoadBypass);

  const bool livabilityHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::Livability ||
                        m_heatmapOverlay == HeatmapOverlay::InterventionPriority);

  const bool hotspotHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::LivabilityHotspot ||
                        m_heatmapOverlay == HeatmapOverlay::InterventionHotspot);

  const bool runoffHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::RunoffPollution ||
                        m_heatmapOverlay == HeatmapOverlay::RunoffPollutionLoad);

  const bool runoffMitigationHeatmapActive =
      heatmapActive && (m_heatmapOverlay == HeatmapOverlay::RunoffMitigationPriority ||
                        m_heatmapOverlay == HeatmapOverlay::RunoffMitigationPlan);

  const bool districtStatsActive = m_showDistrictPanel || (m_showReport && m_reportPage == 4);

  // Many derived systems need the "road component touches map edge" mask.
  // This should be computed regardless of whether the connectivity overlay is *drawn*.
  const bool requireOutside = m_sim.config().requireOutsideConnection;

  // Traffic/goods are used by multiple overlays and analysis layers.
  const bool needTrafficResult =
      (m_showTrafficOverlay || districtStatsActive || landValueHeatmapActive ||
       airHeatmapActive || runoffHeatmapActive || runoffMitigationHeatmapActive ||
       jobsNeeded || crimeHeatmapActive || trafficSafetyHeatmapActive || roadHealthHeatmapActive ||
       livabilityHeatmapActive || transitHeatmapActive || carbonHeatmapActive);

  const bool needGoodsResult = (m_showGoodsOverlay || airHeatmapActive || livabilityHeatmapActive ||
                                transitHeatmapActive || carbonHeatmapActive || crimeHeatmapActive);

  const bool needRoadToEdgeMask =
      requireOutside &&
      (m_showOutsideOverlay || needTrafficResult || needGoodsResult || walkHeatmapActive || districtStatsActive ||
       transitHeatmapActive);

  const std::vector<std::uint8_t>* roadToEdgeMask = nullptr;
  if (needRoadToEdgeMask) {
    ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
    roadToEdgeMask = &m_outsideOverlayRoadToEdge;
  }

  // Only pass the mask to the renderer if the user wants the overlay.
  const std::vector<std::uint8_t>* outsideMask = m_showOutsideOverlay ? roadToEdgeMask : nullptr;

  if (needTrafficResult && m_trafficDirty) {
    const float share = (m_world.stats().population > 0)
                            ? (static_cast<float>(m_world.stats().employed) /
                               static_cast<float>(m_world.stats().population))
                            : 0.0f;

    TrafficConfig tc;
    tc.requireOutsideConnection = requireOutside;
    {
      const TrafficModelSettings& tm = m_sim.trafficModel();
      tc.congestionAwareRouting = tm.congestionAwareRouting;
      tc.congestionIterations = tm.congestionIterations;
      tc.congestionAlpha = tm.congestionAlpha;
      tc.congestionBeta = tm.congestionBeta;
      tc.congestionCapacityScale = tm.congestionCapacityScale;
      tc.congestionRatioClamp = tm.congestionRatioClamp;
      tc.capacityAwareJobs = tm.capacityAwareJobs;
      tc.jobAssignmentIterations = tm.jobAssignmentIterations;
      tc.jobPenaltyBaseMilli = tm.jobPenaltyBaseMilli;
    }

    const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? roadToEdgeMask : nullptr);
    m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
    m_trafficDirty = false;
    m_carbonDirty = true;
    m_crimeDirty = true;
    // Job accessibility depends on congestion costs when available.
    m_jobsDirty = true;
  }

  const std::vector<std::uint16_t>* trafficMask = nullptr;
  int trafficMax = 0;
  if (m_showTrafficOverlay && !m_traffic.roadTraffic.empty()) {
    trafficMask = &m_traffic.roadTraffic;
    trafficMax = m_traffic.maxTraffic;
  }

  const std::vector<std::uint16_t>* goodsTrafficMask = nullptr;
  int goodsMax = 0;
  const std::vector<std::uint8_t>* commercialGoodsFill = nullptr;
  if (needGoodsResult) {
    if (m_goodsDirty) {
      GoodsConfig gc;
      gc.requireOutsideConnection = requireOutside;

      const Stats& st = m_world.stats();
      const TradeModelSettings& ts = m_sim.tradeModel();
      if (ts.enabled) {
        gc.allowImports = ts.allowImports;
        gc.allowExports = ts.allowExports;
        gc.importCapacityPct = std::clamp(st.tradeImportCapacityPct, 0, 100);
        gc.exportCapacityPct = std::clamp(st.tradeExportCapacityPct, 0, 100);
      }

      const std::vector<std::uint8_t>* pre = (gc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_goods = ComputeGoodsFlow(m_world, gc, pre);
      m_goodsDirty = false;
      m_carbonDirty = true;
      m_crimeDirty = true;
    }
  }

  if (m_showGoodsOverlay) {
    goodsTrafficMask = &m_goods.roadGoodsTraffic;
    goodsMax = m_goods.maxRoadGoodsTraffic;
    commercialGoodsFill = &m_goods.commercialFill;
  }

  // --- Land value (heatmap + district stats) ---
  const bool needLandValueResult = landValueHeatmapActive || districtStatsActive;
  if (needLandValueResult) {
    if (m_landValueDirty ||
        m_landValue.value.size() != static_cast<std::size_t>(std::max(0, m_world.width()) * std::max(0, m_world.height()))) {
      LandValueConfig lc;
      lc.requireOutsideConnection = requireOutside;
      const TrafficResult* tptr = needTrafficResult ? &m_traffic : nullptr;
      m_landValue = ComputeLandValue(m_world, lc, tptr, roadToEdgeMask);
      m_landValueDirty = false;
    }
  }

  // --- Sea-level flood heatmap (derived from the heightfield) ---
  const bool needSeaFloodHeatmap = m_showResiliencePanel || (heatmapActive && (m_heatmapOverlay == HeatmapOverlay::FloodDepth));
  if (needSeaFloodHeatmap) {
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));

    if (m_seaFloodDirty || m_seaFloodHeatmap.size() != n) {
      std::vector<float> heights(n, 0.0f);
      if (w > 0 && h > 0) {
        for (int y = 0; y < h; ++y) {
          for (int x = 0; x < w; ++x) {
            heights[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] =
                m_world.at(x, y).height;
          }
        }
      }

      m_seaFlood = ComputeSeaLevelFlood(heights, w, h, m_seaLevel, m_seaFloodCfg);

      m_seaFloodHeatmap.assign(n, 0.0f);
      const float denom = (m_seaFlood.maxDepth > 1e-6f) ? m_seaFlood.maxDepth : 0.0f;
      if (denom > 0.0f) {
        for (std::size_t i = 0; i < n; ++i) {
          const float d = (i < m_seaFlood.depth.size()) ? m_seaFlood.depth[i] : 0.0f;
          m_seaFloodHeatmap[i] = std::clamp(d / denom, 0.0f, 1.0f);
        }
      }

      m_seaFloodDirty = false;
    }
  }

  // --- Ponding / depression-fill heatmap (derived from the heightfield) ---
  const bool needPondingHeatmap = m_showResiliencePanel || (heatmapActive && (m_heatmapOverlay == HeatmapOverlay::PondingDepth));
  if (needPondingHeatmap) {
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));

    if (m_pondingDirty || m_pondingHeatmap.size() != n) {
      std::vector<float> heights(n, 0.0f);
      std::vector<std::uint8_t> drainMask(n, std::uint8_t{0});

      if (w > 0 && h > 0) {
        for (int y = 0; y < h; ++y) {
          for (int x = 0; x < w; ++x) {
            const Tile& t = m_world.at(x, y);
            const std::size_t i = static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
            heights[i] = t.height;

            // Preserve existing water bodies (and rivers) as drains so we measure "new" ponding potential.
            if (t.terrain == Terrain::Water && t.overlay != Overlay::Road) {
              drainMask[i] = 1;
            }
          }
        }
      }

      const DepressionFillResult r = FillDepressionsPriorityFlood(heights, w, h, &drainMask, m_pondingCfg);

      m_pondingFilledCells = r.filledCells;
      m_pondingVolume = r.volume;
      m_pondingMaxDepth = r.maxDepth;

      m_pondingHeatmap.assign(n, 0.0f);
      const float denom = (m_pondingMaxDepth > 1e-6f) ? m_pondingMaxDepth : 0.0f;
      if (denom > 0.0f && r.depth.size() == n) {
        for (std::size_t i = 0; i < n; ++i) {
          m_pondingHeatmap[i] = std::clamp(r.depth[i] / denom, 0.0f, 1.0f);
        }
      }

      m_pondingDirty = false;
    }
  }

  // --- Heatmap overlay (land value + component fields) ---
  const std::vector<float>* heatmap = nullptr;
  Renderer::HeatmapRamp heatmapRamp = Renderer::HeatmapRamp::Good;
  const char* heatmapName = nullptr;

  if (heatmapActive) {
    const int w = m_world.width();
    const int h = m_world.height();
    const std::size_t n = static_cast<std::size_t>(std::max(0, w) * std::max(0, h));

    auto ensureAirPollutionUpToDate = [&]() {
      if (!airHeatmapActive) return;
      if (!m_airPollutionDirty && m_airPollution.w == w && m_airPollution.h == h &&
          m_airPollution.pollution01.size() == n && m_airPollution.emission01.size() == n) {
        return;
      }
      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      const GoodsResult* gr = needGoodsResult ? &m_goods : nullptr;
      m_airPollution = ComputeAirPollution(m_world, m_airPollutionCfg, tr, gr);
      m_airPollutionDirty = false;
    };

    auto ensureSkyViewUpToDate = [&]() {
      if (!needSkyViewResult) return;
      if (!m_skyViewDirty && m_skyView.w == w && m_skyView.h == h &&
          m_skyView.skyView01.size() == n && m_skyView.canyon01.size() == n) {
        return;
      }

      // Defaults tuned for an inexpensive but stable horizon scan.
      m_skyViewCfg.azimuthSamples = 16;
      m_skyViewCfg.maxHorizonRadius = 64;
      m_skyViewCfg.includeBuildings = true;
      m_skyView = ComputeSkyViewFactor(m_world, m_skyViewCfg);
      m_skyViewDirty = false;
    };

    auto ensureSolarUpToDate = [&]() {
      if (!solarHeatmapActive) return;
      if (!m_solarDirty && m_solar.w == w && m_solar.h == h &&
          m_solar.potential01.size() == n && m_solar.exposure01.size() == n) {
        return;
      }
      m_solar = ComputeSolarPotential(m_world, m_solarCfg);
      m_solarDirty = false;
      m_energyDirty = true;
      m_carbonDirty = true;
    };

    auto ensureEnergyUpToDate = [&]() {
      if (!energyHeatmapActive) {
        return;
      }
      if (!m_energyDirty && m_energy.w == w && m_energy.h == h &&
          m_energy.demand01.size() == n && m_energy.solar01.size() == n &&
          m_energy.balance01.size() == n) {
        return;
      }

      // Energy uses solar supply; reuse the cached solar layer and keep this cheap in-game by
      // skipping the heat island cooling adjustment (the dossier export includes it).
      ensureSolarUpToDate();

      EnergyModelConfig ec = m_energyCfg;
      ec.useHeatIslandCooling = false;

      m_energy = ComputeEnergyModel(m_world, ec, &m_solar, nullptr);
      m_energyDirty = false;
      m_carbonDirty = true;
    };



    auto ensureCarbonUpToDate = [&]() {
      if (!carbonHeatmapActive) {
        return;
      }

      ensureEnergyUpToDate();

      const int w = m_world.w;
      const int h = m_world.h;
      const int n = w * h;
      const bool sizeOk = (m_carbon.w == w && m_carbon.h == h &&
                           static_cast<int>(m_carbon.balance01.size()) == n);
      if (!m_carbonDirty && sizeOk) {
        return;
      }

      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      const GoodsResult* gr = needGoodsResult ? &m_goods : nullptr;

      m_carbon = ComputeCarbonModel(m_world, m_carbonCfg, &m_energy, tr, gr);
      m_carbonDirty = false;
    };
    auto ensureTransitAccessUpToDate = [&]() {
      if (!transitHeatmapActive) {
        return;
      }

      // Transit access depends on the planner outputs. If the plan changes, we must recompute.
      const bool planWasDirty = m_transitPlanDirty;
      ensureTransitPlanUpToDate();

      if (!m_transitAccessDirty && !planWasDirty && m_transitAccess.w == w && m_transitAccess.h == h &&
          m_transitAccess.access01.size() == n && m_transitAccess.modeSharePotential01.size() == n) {
        return;
      }

      TransitAccessibilityConfig tc = m_transitAccessCfg;
      tc.requireOutsideConnection = requireOutside;

      const TransitModelSettings& tm = m_sim.transitModel();
      tc.demandMode = tm.demandMode;
      tc.walkRadiusSteps = tm.walkRadiusSteps;
      tc.stopSpacingTiles = tm.stopSpacingTiles;
      tc.serviceLevel = tm.serviceLevel;
      tc.maxModeShare = tm.maxModeShare;
      tc.travelTimeMultiplier = tm.travelTimeMultiplier;
      tc.plannerCfg = tm.plannerCfg;

      TransitAccessibilityInputs in;
      in.traffic = needTrafficResult ? &m_traffic : nullptr;
      in.goods = needGoodsResult ? &m_goods : nullptr;
      in.roadGraph = &m_roadGraph;
      in.plan = &m_transitPlan;
      in.roadToEdgeMask = (tc.requireOutsideConnection ? roadToEdgeMask : nullptr);

      m_transitAccess = ComputeTransitAccessibility(m_world, tc, in);
      m_transitAccessDirty = false;
    };

    auto ensureWalkabilityUpToDate = [&]() {
      if (!walkHeatmapActive) return;
      if (!m_walkabilityDirty && m_walkability.w == w && m_walkability.h == h &&
          m_walkability.overall01.size() == n) {
        return;
      }
      WalkabilityConfig wc = m_walkabilityCfg;
      wc.requireOutsideConnection = requireOutside;
      m_walkability = ComputeWalkability(m_world, wc, roadToEdgeMask, /*precomputedZoneAccess=*/nullptr);
      m_walkabilityDirty = false;
    };

    auto ensureJobsUpToDate = [&]() {
      if (!jobsNeeded) return;
      if (!m_jobsDirty && m_jobs.w == w && m_jobs.h == h &&
          m_jobs.jobAccess01.size() == n && m_jobs.jobOpportunity01.size() == n) {
        return;
      }
      JobOpportunityConfig jc = m_jobsCfg;
      jc.requireOutsideConnection = requireOutside;

      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      const std::vector<std::uint8_t>* pre = (jc.requireOutsideConnection ? roadToEdgeMask : nullptr);

      m_jobs = ComputeJobOpportunity(m_world, jc, tr, pre, /*precomputedZoneAccess=*/nullptr);
      m_jobsDirty = false;
      m_crimeDirty = true;
    };

    auto ensureCrimeUpToDate = [&]() {
      if (!crimeHeatmapActive) return;

      // Crime depends on job accessibility and road connectivity.
      ensureJobsUpToDate();

      if (!m_crimeDirty && m_crime.w == w && m_crime.h == h &&
          m_crime.risk01.size() == n && m_crime.policeAccess01.size() == n &&
          m_crime.policeCostMilli.size() == n) {
        return;
      }

      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      const GoodsResult* gr = needGoodsResult ? &m_goods : nullptr;

      NoiseConfig nc{};
      const NoiseResult noise = ComputeNoisePollution(m_world, nc, tr, gr);

      CrimeModelConfig cc = m_crimeCfg;
      cc.requireOutsideConnection = requireOutside;
      cc.weightMode = IsochroneWeightMode::TravelTime;

      const std::vector<std::uint8_t>* pre = (cc.requireOutsideConnection ? roadToEdgeMask : nullptr);
      m_crime = ComputeCrimeModel(m_world, cc, tr, gr, &m_jobs, &noise, pre, /*precomputedZoneAccess=*/nullptr);
      m_crimeDirty = false;
    };

    auto ensureTrafficSafetyUpToDate = [&]() {
      if (!trafficSafetyHeatmapActive) return;
      if (!m_trafficSafetyDirty && m_trafficSafety.w == w && m_trafficSafety.h == h &&
          m_trafficSafety.risk01.size() == n && m_trafficSafety.exposure01.size() == n &&
          m_trafficSafety.priority01.size() == n) {
        return;
      }

      // Canyon confinement leverages sky view (urban canyons) where available.
      ensureSkyViewUpToDate();

      TrafficSafetyConfig tc = m_trafficSafetyCfg;
      tc.requireOutsideConnection = requireOutside;

      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      const SkyViewResult* sv = needSkyViewResult ? &m_skyView : nullptr;
      const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? roadToEdgeMask : nullptr);

      m_trafficSafety = ComputeTrafficSafety(m_world, tc, tr, sv, pre);
      m_trafficSafetyDirty = false;
    };

    auto ensureRoadHealthUpToDate = [&]() {
      if (!roadHealthHeatmapActive) return;
      if (!m_roadHealthDirty && m_roadHealth.w == w && m_roadHealth.h == h &&
          m_roadHealth.centrality01.size() == n && m_roadHealth.vulnerability01.size() == n) {
        return;
      }
      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      m_roadHealth = ComputeRoadHealth(m_world, m_roadHealthCfg, tr);
      m_roadHealthDirty = false;
    };

    auto ensureLivabilityUpToDate = [&]() {
      if (!(livabilityHeatmapActive || hotspotHeatmapActive)) return;
      if (!m_livabilityDirty && m_livability.w == w && m_livability.h == h &&
          m_livability.livability01.size() == n && m_livability.priority01.size() == n) {
        return;
      }
      LivabilityConfig lc = m_livabilityCfg;
      lc.requireOutsideConnection = requireOutside;
      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      const GoodsResult* gr = needGoodsResult ? &m_goods : nullptr;
      m_livability = ComputeLivability(m_world, lc, tr, gr);
      m_livabilityDirty = false;
      m_hotspotsDirty = true; // hotspots derive from livability fields
    };

    auto ensureHotspotsUpToDate = [&]() {
      if (!hotspotHeatmapActive) return;
      ensureLivabilityUpToDate();
      if (!m_hotspotsDirty && m_livabilityHotspot.w == w && m_livabilityHotspot.h == h &&
          m_livabilityHotspot.z01.size() == n && m_interventionHotspot.z01.size() == n) {
        return;
      }
      m_livabilityHotspot = ComputeHotspotsGiStar(m_world, m_livability.livability01, m_hotspotCfg);
      m_interventionHotspot = ComputeHotspotsGiStar(m_world, m_livability.priority01, m_hotspotCfg);
      m_hotspotsDirty = false;
    };

    auto ensureRunoffUpToDate = [&]() {
      if (!runoffHeatmapActive) return;
      if (!m_runoffDirty && m_runoff.w == w && m_runoff.h == h &&
          m_runoff.pollution01.size() == n && m_runoff.localLoad01.size() == n) {
        return;
      }
      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      m_runoff = ComputeRunoffPollution(m_world, m_runoffCfg, tr);
      m_runoffDirty = false;
    };

    auto ensureRunoffMitigationUpToDate = [&]() {
      if (!runoffMitigationHeatmapActive) return;
      if (!m_runoffMitigationDirty && m_runoffMitigation.w == w && m_runoffMitigation.h == h &&
          m_runoffMitigation.priority01.size() == n && m_runoffMitigation.planMask.size() == n) {
        return;
      }
      const TrafficResult* tr = needTrafficResult ? &m_traffic : nullptr;
      m_runoffMitigation = SuggestRunoffMitigationParks(m_world, m_runoffMitigationCfg, tr);
      m_runoffMitigationDirty = false;
    };

    switch (m_heatmapOverlay) {
    case HeatmapOverlay::LandValue:
      heatmapName = "Land value";
      heatmap = &m_landValue.value;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::ParkAmenity:
      heatmapName = "Park amenity";
      heatmap = &m_landValue.parkAmenity;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WaterAmenity:
      heatmapName = "Water amenity";
      heatmap = &m_landValue.waterAmenity;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::Pollution:
      heatmapName = "Pollution";
      heatmap = &m_landValue.pollution;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::TrafficSpill:
      heatmapName = "Traffic spill";
      heatmap = &m_landValue.traffic;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::ServicesOverall:
      ensureServicesHeatmapsUpToDate();
      heatmapName = "Services";
      heatmap = &m_servicesHeatmaps.overall;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::ServicesEducation:
      ensureServicesHeatmapsUpToDate();
      heatmapName = "Services (edu)";
      heatmap = &m_servicesHeatmaps.education;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::ServicesHealth:
      ensureServicesHeatmapsUpToDate();
      heatmapName = "Services (health)";
      heatmap = &m_servicesHeatmaps.health;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::ServicesSafety:
      ensureServicesHeatmapsUpToDate();
      heatmapName = "Services (safety)";
      heatmap = &m_servicesHeatmaps.safety;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::FireRisk:
      ensureFireRiskUpToDate();
      heatmapName = "Fire risk";
      heatmap = &m_fireRisk.risk01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::FloodDepth:
      heatmapName = "Flood depth";
      heatmap = &m_seaFloodHeatmap;
      heatmapRamp = Renderer::HeatmapRamp::Water;
      break;
    case HeatmapOverlay::PondingDepth:
      heatmapName = "Ponding depth";
      heatmap = &m_pondingHeatmap;
      heatmapRamp = Renderer::HeatmapRamp::Water;
      break;
    case HeatmapOverlay::EvacuationTime:
      ensureEvacuationScenarioUpToDate();
      heatmapName = "Evac time";
      heatmap = &m_evacHeatmaps.evacTime;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::EvacuationUnreachable:
      ensureEvacuationScenarioUpToDate();
      heatmapName = "Evac unreachable";
      heatmap = &m_evacHeatmaps.evacUnreachable;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::EvacuationFlow:
      ensureEvacuationScenarioUpToDate();
      heatmapName = "Evac flow";
      heatmap = &m_evacHeatmaps.evacFlow;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::AirPollution:
      ensureAirPollutionUpToDate();
      heatmapName = "Air pollution";
      heatmap = &m_airPollution.pollution01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::AirPollutionEmission:
      ensureAirPollutionUpToDate();
      heatmapName = "Air emission";
      heatmap = &m_airPollution.emission01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::SkyView:
      ensureSkyViewUpToDate();
      heatmapName = "Sky view";
      heatmap = &m_skyView.skyView01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::CanyonConfinement:
      ensureSkyViewUpToDate();
      heatmapName = "Canyon confinement";
      heatmap = &m_skyView.canyon01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::SolarExposure:
      ensureSolarUpToDate();
      heatmapName = "Solar exposure";
      heatmap = &m_solar.exposure01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::SolarPotential:
      ensureSolarUpToDate();
      heatmapName = "Solar potential";
      heatmap = &m_solar.potential01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::EnergyDemand:
      ensureEnergyUpToDate();
      heatmapName = "Energy demand";
      heatmap = &m_energy.demand01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::EnergySolar:
      ensureEnergyUpToDate();
      heatmapName = "Rooftop solar supply";
      heatmap = &m_energy.solar01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::EnergyBalance:
      ensureEnergyUpToDate();
      heatmapName = "Energy balance";
      heatmap = &m_energy.balance01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::CarbonEmission:
      ensureCarbonUpToDate();
      heatmapName = "Carbon emission";
      heatmap = &m_carbon.emission01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::CarbonSequestration:
      ensureCarbonUpToDate();
      heatmapName = "Carbon sequestration";
      heatmap = &m_carbon.sequestration01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::CarbonBalance:
      ensureCarbonUpToDate();
      heatmapName = "Carbon balance";
      heatmap = &m_carbon.balance01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::CrimeRisk:
      ensureCrimeUpToDate();
      heatmapName = "Crime risk";
      heatmap = &m_crime.risk01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::PoliceAccess:
      ensureCrimeUpToDate();
      heatmapName = "Police access";
      heatmap = &m_crime.policeAccess01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::TrafficCrashRisk:
      ensureTrafficSafetyUpToDate();
      heatmapName = "Traffic crash risk";
      heatmap = &m_trafficSafety.risk01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::TrafficCrashExposure:
      ensureTrafficSafetyUpToDate();
      heatmapName = "Traffic crash exposure";
      heatmap = &m_trafficSafety.exposure01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::TrafficCrashPriority:
      ensureTrafficSafetyUpToDate();
      heatmapName = "Traffic crash priority";
      heatmap = &m_trafficSafety.priority01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;

    case HeatmapOverlay::TransitAccess:
      ensureTransitAccessUpToDate();
      heatmapName = "Transit access";
      heatmap = &m_transitAccess.access01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::TransitModeSharePotential:
      ensureTransitAccessUpToDate();
      heatmapName = "Transit mode share potential";
      heatmap = &m_transitAccess.modeSharePotential01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::Walkability:
      ensureWalkabilityUpToDate();
      heatmapName = "Walkability";
      heatmap = &m_walkability.overall01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WalkabilityPark:
      ensureWalkabilityUpToDate();
      heatmapName = "Walkability (parks)";
      heatmap = &m_walkability.park01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WalkabilityRetail:
      ensureWalkabilityUpToDate();
      heatmapName = "Walkability (retail)";
      heatmap = &m_walkability.retail01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WalkabilityEducation:
      ensureWalkabilityUpToDate();
      heatmapName = "Walkability (education)";
      heatmap = &m_walkability.education01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WalkabilityHealth:
      ensureWalkabilityUpToDate();
      heatmapName = "Walkability (health)";
      heatmap = &m_walkability.health01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::WalkabilitySafety:
      ensureWalkabilityUpToDate();
      heatmapName = "Walkability (safety)";
      heatmap = &m_walkability.safety01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::JobAccess:
      ensureJobsUpToDate();
      heatmapName = "Job access";
      heatmap = &m_jobs.jobAccess01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::JobOpportunity:
      ensureJobsUpToDate();
      heatmapName = "Job opportunity";
      heatmap = &m_jobs.jobOpportunity01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;

    case HeatmapOverlay::RoadCentrality:
      ensureRoadHealthUpToDate();
      heatmapName = "Road centrality";
      heatmap = &m_roadHealth.centrality01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::RoadVulnerability:
      ensureRoadHealthUpToDate();
      heatmapName = "Road vulnerability";
      heatmap = &m_roadHealth.vulnerability01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::RoadBypass: {
      ensureRoadHealthUpToDate();
      heatmapName = "Road bypass plan";
      m_heatmapTmp01.assign(n, 0.0f);
      if (m_roadHealth.bypassMask.size() == n) {
        for (std::size_t i = 0; i < n; ++i) {
          m_heatmapTmp01[i] = (m_roadHealth.bypassMask[i] != 0) ? 1.0f : 0.0f;
        }
      }
      heatmap = &m_heatmapTmp01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    }

    case HeatmapOverlay::Livability:
      ensureLivabilityUpToDate();
      heatmapName = "Livability";
      heatmap = &m_livability.livability01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    case HeatmapOverlay::InterventionPriority:
      ensureLivabilityUpToDate();
      heatmapName = "Intervention priority";
      heatmap = &m_livability.priority01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;

    case HeatmapOverlay::LivabilityHotspot:
      ensureHotspotsUpToDate();
      heatmapName = "Livability hotspot";
      heatmap = &m_livabilityHotspot.z01;
      heatmapRamp = Renderer::HeatmapRamp::Diverging;
      break;
    case HeatmapOverlay::InterventionHotspot:
      ensureHotspotsUpToDate();
      heatmapName = "Priority hotspot";
      heatmap = &m_interventionHotspot.z01;
      heatmapRamp = Renderer::HeatmapRamp::Diverging;
      break;

    case HeatmapOverlay::RunoffPollution:
      ensureRunoffUpToDate();
      heatmapName = "Runoff pollution";
      heatmap = &m_runoff.pollution01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::RunoffPollutionLoad:
      ensureRunoffUpToDate();
      heatmapName = "Runoff load";
      heatmap = &m_runoff.localLoad01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;

    case HeatmapOverlay::RunoffMitigationPriority:
      ensureRunoffMitigationUpToDate();
      heatmapName = "Mitigation priority";
      heatmap = &m_runoffMitigation.priority01;
      heatmapRamp = Renderer::HeatmapRamp::Bad;
      break;
    case HeatmapOverlay::RunoffMitigationPlan: {
      ensureRunoffMitigationUpToDate();
      heatmapName = "Mitigation plan";
      m_heatmapTmp01.assign(n, 0.0f);
      if (m_runoffMitigation.planMask.size() == n) {
        for (std::size_t i = 0; i < n; ++i) {
          m_heatmapTmp01[i] = (m_runoffMitigation.planMask[i] != 0) ? 1.0f : 0.0f;
        }
      }
      heatmap = &m_heatmapTmp01;
      heatmapRamp = Renderer::HeatmapRamp::Good;
      break;
    }
    default: break;
    }
  }

  // District overlay rendering controls.
  const bool showDistrictOverlay = (m_showDistrictOverlay || m_showDistrictPanel || (m_tool == Tool::District));
  const int highlightDistrict = showDistrictOverlay ? std::clamp(m_activeDistrict, 0, kDistrictCount - 1) : -1;
  const bool showDistrictBorders = showDistrictOverlay && m_showDistrictBorders;

  // Screen-space weather (fog/precip). Suppressed in utility overlays for readability.
  // (Vehicles are drawn inside the world pass so they're affected by day/night + wetness grading.)
  const bool allowWeatherFx =
    (outsideMask == nullptr) && (trafficMask == nullptr) && (goodsTrafficMask == nullptr) && (commercialGoodsFill == nullptr) &&
    (heatmap == nullptr);

  // World pass: optionally render to an offscreen target for resolution scaling (and Post FX).
  if (wantsWorldRenderTarget()) {
    ensureWorldRenderTarget(screenW, screenH);
  }

  bool weatherFxDrawnInWorldRT = false;

  auto drawWorldDirect = [&]() {
    std::vector<Renderer::WorldSprite> sprites;
    appendVehicleSprites(m_camera, sprites);

    m_renderer.drawWorld(m_world, m_camera, screenW, screenH, m_timeSec, m_hovered, m_drawGrid, worldBrush, selected,
                         pathPtr, outsideMask,
                         trafficMask, trafficMax,
                         goodsTrafficMask, goodsMax,
                         commercialGoodsFill,
                         heatmap, heatmapRamp,
                         showDistrictOverlay, highlightDistrict, showDistrictBorders,
                         /*mergeZoneBuildings=*/m_mergedZoneBuildings,
                         /*drawBeforeFx=*/{},
                         /*drawAfterFx=*/{},
                         /*sprites=*/sprites.empty() ? nullptr : &sprites);
  };

  if (!wantsWorldRenderTarget() || !m_worldRenderRTValid) {
    drawWorldDirect();
  } else {
    Vector2 taaJitterPx{0.0f, 0.0f};
    bool taaResetHistory = false;

    Camera2D camRT = m_camera;
    camRT.zoom = m_camera.zoom * m_worldRenderScale;
    camRT.offset.x = m_camera.offset.x * m_worldRenderScale;
    camRT.offset.y = m_camera.offset.y * m_worldRenderScale;

    const Camera2D camRTBase = camRT;

    const bool taaActive = m_postFx.enabled && m_postFx.taaEnabled && m_postFxPipeline.taaReady();

    if (taaActive) {
      auto camDifferent = [](const Camera2D& a, const Camera2D& b) -> bool {
        const float epsPos = 1e-3f;
        const float epsZoom = 1e-6f;
        const float epsRot = 1e-3f;
        return std::abs(a.target.x - b.target.x) > epsPos || std::abs(a.target.y - b.target.y) > epsPos ||
               std::abs(a.offset.x - b.offset.x) > epsPos || std::abs(a.offset.y - b.offset.y) > epsPos ||
               std::abs(a.zoom - b.zoom) > epsZoom || std::abs(a.rotation - b.rotation) > epsRot;
      };

      bool forceReset = (!m_taaActiveLastFrame || !m_taaPrevCamValid);
      if (!forceReset && camDifferent(camRTBase, m_taaPrevCam)) forceReset = true;
      if (!forceReset && (m_taaPrevRTW != m_worldRenderRTWidth || m_taaPrevRTH != m_worldRenderRTHeight)) forceReset = true;

      if (forceReset) {
        taaResetHistory = true;
        taaJitterPx = Vector2{0.0f, 0.0f};
        m_taaFrameIndex = 0;
      } else {
        auto halton = [](int index, int base) -> float {
          float f = 1.0f;
          float r = 0.0f;
          int i = index;
          while (i > 0) {
            f /= static_cast<float>(base);
            r += f * static_cast<float>(i % base);
            i /= base;
          }
          return r;
        };

        const float jitterAmp = std::clamp(m_postFx.taaJitter, 0.0f, 1.0f);
        const int idx = m_taaFrameIndex + 1;
        taaJitterPx.x = (halton(idx, 2) - 0.5f) * jitterAmp;
        taaJitterPx.y = (halton(idx, 3) - 0.5f) * jitterAmp;

        m_taaFrameIndex = (m_taaFrameIndex + 1) % 1024;
      }

      // Apply jitter in world-RT pixel units. (PostFxPipeline cancels it in the TAA resolve shader.)
      camRT.offset.x += taaJitterPx.x;
      camRT.offset.y += taaJitterPx.y;

      // Update prev-state using the *unjittered* camera.
      m_taaPrevCam = camRTBase;
      m_taaPrevCamValid = true;
      m_taaPrevWorldRenderScale = m_worldRenderScale;
      m_taaPrevRTW = m_worldRenderRTWidth;
      m_taaPrevRTH = m_worldRenderRTHeight;
      m_taaActiveLastFrame = true;
    } else {
      // If TAA isn't active, clear state so enabling it triggers a reset.
      m_taaActiveLastFrame = false;
      m_taaPrevCamValid = false;
      m_taaFrameIndex = 0;
    }

    BeginTextureMode(m_worldRenderRT);
    ClearBackground(Color{30, 32, 38, 255});

    std::vector<Renderer::WorldSprite> sprites;
    appendVehicleSprites(camRT, sprites);

    m_renderer.drawWorld(m_world, camRT, m_worldRenderRTWidth, m_worldRenderRTHeight, m_timeSec, m_hovered,
                         m_drawGrid, worldBrush, selected,
                         pathPtr, outsideMask,
                         trafficMask, trafficMax,
                         goodsTrafficMask, goodsMax,
                         commercialGoodsFill,
                         heatmap, heatmapRamp,
                         showDistrictOverlay, highlightDistrict, showDistrictBorders,
                         /*mergeZoneBuildings=*/m_mergedZoneBuildings,
                         /*drawBeforeFx=*/{},
                         /*drawAfterFx=*/{},
                         /*sprites=*/sprites.empty() ? nullptr : &sprites);

    // Optional: fold the screen-space weather into the world RT so Post FX affects it.
    if (m_postFx.enabled && m_postFx.includeWeather) {
      m_renderer.drawWeatherScreenFX(m_worldRenderRTWidth, m_worldRenderRTHeight, m_timeSec, allowWeatherFx);
      weatherFxDrawnInWorldRT = true;
    }

    EndTextureMode();

    const Rectangle src = {0.0f, 0.0f, static_cast<float>(m_worldRenderRTWidth),
                           -static_cast<float>(m_worldRenderRTHeight)};
    const Rectangle dst = {0.0f, 0.0f, static_cast<float>(screenW), static_cast<float>(screenH)};

    if (m_postFx.enabled) {
      // Feed weather uniforms to the PostFX shader so lens precipitation (rain on lens)
      // can automatically track current conditions.
      const auto wx = m_renderer.weatherSettings();
      const int wxMode = static_cast<int>(wx.mode);
      const float wxIntensity = wx.intensity;

      // Convert deg -> direction; normalize and bias slightly "down" (y+) so the shader
      // stays stable across all camera orientations.
      const float wxAngRad = wx.windAngleDeg * (3.14159265358979323846f / 180.0f);
      Vector2 wxWindDir{std::cos(wxAngRad), std::sin(wxAngRad)};
      {
        const float l2 = wxWindDir.x * wxWindDir.x + wxWindDir.y * wxWindDir.y;
        if (l2 > 1.0e-6f) {
          const float inv = 1.0f / std::sqrt(l2);
          wxWindDir.x *= inv;
          wxWindDir.y *= inv;
        } else {
          wxWindDir = Vector2{0.0f, 1.0f};
        }
        if (wxWindDir.y < 0.15f) wxWindDir.y = 0.15f;
        const float l3 = wxWindDir.x * wxWindDir.x + wxWindDir.y * wxWindDir.y;
        if (l3 > 1.0e-6f) {
          const float inv = 1.0f / std::sqrt(l3);
          wxWindDir.x *= inv;
          wxWindDir.y *= inv;
        }
      }
      const float wxWindSpeed = wx.windSpeed;

      m_postFxPipeline.drawTexturePro(
          m_worldRenderRT.texture, src, dst, m_postFx, m_timeSec, static_cast<std::uint32_t>(m_world.seed()), WHITE,
          taaJitterPx, taaResetHistory,
          wxMode, wxIntensity,
          wxWindDir, wxWindSpeed);
    } else {
      DrawTexturePro(m_worldRenderRT.texture, src, dst, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
    }
  }

  if (!weatherFxDrawnInWorldRT) {
    m_renderer.drawWeatherScreenFX(screenW, screenH, m_timeSec, allowWeatherFx);
  }

  drawBlueprintOverlay();

  // Road graph overlay (debug): nodes/edges extracted from the current road tiles.
  if (m_showRoadGraphOverlay) {
    ensureRoadGraphUpToDate();

    if (!m_roadGraph.nodes.empty()) {
      BeginMode2D(m_camera);

      const float zoom = std::max(0.25f, m_camera.zoom);
      const float thickness = 2.5f / zoom;
      const float radius = 3.0f / zoom;

      const int w = m_world.width();

      auto edgeIsConnected = [&](const RoadGraphEdge& e) -> bool {
        if (!outsideMask) return true;
        if (w <= 0 || m_world.height() <= 0) return true;
        if (outsideMask->size() != static_cast<std::size_t>(w) * static_cast<std::size_t>(m_world.height())) return true;
        for (const Point& p : e.tiles) {
          const std::size_t idx = static_cast<std::size_t>(p.y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(p.x);
          if (idx >= outsideMask->size()) continue;
          if ((*outsideMask)[idx] == 0) return false;
        }
        return true;
      };

      // Draw edges as polylines along road tile centers.
      for (const RoadGraphEdge& e : m_roadGraph.edges) {
        const bool connected = edgeIsConnected(e);
        const Color c = connected ? Color{0, 220, 255, 140} : Color{255, 80, 80, 170};

        for (std::size_t i = 1; i < e.tiles.size(); ++i) {
          const Point& a = e.tiles[i - 1];
          const Point& b = e.tiles[i];
          const Vector2 wa = TileToWorldCenterElevated(m_world, a.x, a.y, static_cast<float>(m_cfg.tileWidth),
                                                       static_cast<float>(m_cfg.tileHeight), m_elev);
          const Vector2 wb = TileToWorldCenterElevated(m_world, b.x, b.y, static_cast<float>(m_cfg.tileWidth),
                                                       static_cast<float>(m_cfg.tileHeight), m_elev);
          DrawLineEx(wa, wb, thickness, c);
        }
      }

      // Draw nodes as small circles.
      for (const RoadGraphNode& n : m_roadGraph.nodes) {
        Color c = Color{255, 220, 0, 200};
        if (outsideMask && w > 0) {
          const std::size_t idx = static_cast<std::size_t>(n.pos.y) * static_cast<std::size_t>(w) +
                                  static_cast<std::size_t>(n.pos.x);
          if (idx < outsideMask->size() && (*outsideMask)[idx] == 0) c = Color{255, 80, 80, 220};
        }

        const Vector2 wp = TileToWorldCenterElevated(m_world, n.pos.x, n.pos.y, static_cast<float>(m_cfg.tileWidth),
                                                     static_cast<float>(m_cfg.tileHeight), m_elev);
        DrawCircleV(wp, radius, c);
      }

      EndMode2D();
    }
  }

  // Road resilience overlay (debug): bridge edges, articulation nodes, and optional bypass suggestions.
  drawRoadResilienceOverlay();

  // Transit planner overlay (Ctrl+T).
  drawTransitOverlay();

  // Services planner overlay (Ctrl+X).
  drawServicesOverlay();

  // Road upgrade overlay (Ctrl+U).
  drawRoadUpgradeOverlay();

  // Navigator: start/end markers for the currently computed route.
  drawWayfindingOverlay();

  const float simSpeed = kSimSpeeds[static_cast<std::size_t>(std::clamp(m_simSpeedIndex, 0, kSimSpeedCount - 1))];
  const char* inspectInfo = (m_tool == Tool::Inspect && !m_inspectInfo.empty()) ? m_inspectInfo.c_str() : nullptr;

  std::string heatmapInfo;
  if (heatmapActive && heatmapName && heatmap && m_hovered &&
      heatmap->size() == static_cast<std::size_t>(std::max(0, m_world.width()) * std::max(0, m_world.height()))) {
    const std::size_t idx = static_cast<std::size_t>(m_hovered->y) * static_cast<std::size_t>(m_world.width()) +
                            static_cast<std::size_t>(m_hovered->x);
    const float hv = (*heatmap)[idx];
    char buf[256];
    if (m_heatmapOverlay == HeatmapOverlay::FloodDepth) {
      const float depth = (m_seaFlood.maxDepth > 1e-6f) ? (hv * m_seaFlood.maxDepth) : 0.0f;
      std::snprintf(buf, sizeof(buf), "Heatmap: %s (sea %.2f)  depth %.2f", heatmapName,
                    static_cast<double>(m_seaLevel), static_cast<double>(depth));
    } else if (m_heatmapOverlay == HeatmapOverlay::PondingDepth) {
      const float depth = (m_pondingMaxDepth > 1e-6f) ? (hv * m_pondingMaxDepth) : 0.0f;
      std::snprintf(buf, sizeof(buf), "Heatmap: %s  depth %.2f (max %.2f)", heatmapName,
                    static_cast<double>(depth), static_cast<double>(m_pondingMaxDepth));
    } else if (m_heatmapOverlay == HeatmapOverlay::EvacuationTime) {
      ensureEvacuationScenarioUpToDate();
      const EvacuationResult& r = m_evacScenario.evac;
      const int costMilli = (idx < r.resCostMilli.size()) ? r.resCostMilli[idx] : -1;
      const bool hazard = (idx < m_evacScenario.hazardMask.size()) ? (m_evacScenario.hazardMask[idx] != 0) : false;
      const char* unit = m_evacCfg.evac.useTravelTime ? "s" : "steps";
      if (costMilli >= 0) {
        const double t = static_cast<double>(costMilli) / 1000.0;
        std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  tile %.1f%s  p95 %.1f%s  unreachPop %d%s", heatmapName,
                      EvacuationHazardModeName(m_evacCfg.hazardMode), t, unit,
                      static_cast<double>(r.p95EvacTime), unit, r.unreachablePopulation,
                      hazard ? "  (hazard)" : "");
      } else {
        std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  tile UNREACHABLE  unreachPop %d%s", heatmapName,
                      EvacuationHazardModeName(m_evacCfg.hazardMode), r.unreachablePopulation,
                      hazard ? "  (hazard)" : "");
      }
    } else if (m_heatmapOverlay == HeatmapOverlay::EvacuationUnreachable) {
      ensureEvacuationScenarioUpToDate();
      const EvacuationResult& r = m_evacScenario.evac;
      const bool unreach = (idx < r.resCostMilli.size()) ? (r.resCostMilli[idx] < 0) : false;
      const bool hazard = (idx < m_evacScenario.hazardMask.size()) ? (m_evacScenario.hazardMask[idx] != 0) : false;
      std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  tile %s%s  unreachPop %d", heatmapName,
                    EvacuationHazardModeName(m_evacCfg.hazardMode),
                    unreach ? "UNREACHABLE" : "ok", hazard ? " (hazard)" : "",
                    r.unreachablePopulation);
    } else if (m_heatmapOverlay == HeatmapOverlay::EvacuationFlow) {
      ensureEvacuationScenarioUpToDate();
      const EvacuationResult& r = m_evacScenario.evac;
      const unsigned int f = (idx < r.evacRoadFlow.size()) ? r.evacRoadFlow[idx] : 0u;
      std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  tile %u  max %u  cap %d", heatmapName,
                    EvacuationHazardModeName(m_evacCfg.hazardMode), f, r.maxEvacRoadFlow, m_evacCfg.evac.roadTileCapacity);
    } else if (m_heatmapOverlay == HeatmapOverlay::ServicesOverall ||
               m_heatmapOverlay == HeatmapOverlay::ServicesEducation ||
               m_heatmapOverlay == HeatmapOverlay::ServicesHealth ||
               m_heatmapOverlay == HeatmapOverlay::ServicesSafety) {
      ensureServicesHeatmapsUpToDate();
      float city = 0.0f;
      int fac = 0;
      if (m_heatmapOverlay == HeatmapOverlay::ServicesEducation) {
        city = m_servicesHeatmaps.educationSatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[0];
      } else if (m_heatmapOverlay == HeatmapOverlay::ServicesHealth) {
        city = m_servicesHeatmaps.healthSatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[1];
      } else if (m_heatmapOverlay == HeatmapOverlay::ServicesSafety) {
        city = m_servicesHeatmaps.safetySatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[2];
      } else {
        city = m_servicesHeatmaps.overallSatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[0] + m_servicesHeatmaps.activeFacilities[1] +
              m_servicesHeatmaps.activeFacilities[2];
      }
      std::snprintf(buf, sizeof(buf), "Heatmap: %s  tile %.0f%%  city %.0f%%  facilities %d", heatmapName,
                    static_cast<double>(hv * 100.0f), static_cast<double>(city * 100.0f), fac);
    } else if (m_heatmapOverlay == HeatmapOverlay::LivabilityHotspot ||
               m_heatmapOverlay == HeatmapOverlay::InterventionHotspot) {
      ensureHotspotsUpToDate();
      const HotspotResult& hr = (m_heatmapOverlay == HeatmapOverlay::LivabilityHotspot) ? m_livabilityHotspot : m_interventionHotspot;
      const float z = (idx < hr.z.size()) ? hr.z[idx] : 0.0f;
      const std::uint8_t cls = (idx < hr.cls.size()) ? hr.cls[idx] : static_cast<std::uint8_t>(HotspotClass::Neutral);
      std::snprintf(buf, sizeof(buf), "Heatmap: %s  z %.2f  %s", heatmapName, static_cast<double>(z),
                    HotspotClassName(static_cast<HotspotClass>(cls)));
    } else if (m_heatmapOverlay == HeatmapOverlay::RoadBypass ||
               m_heatmapOverlay == HeatmapOverlay::RunoffMitigationPlan) {
      std::snprintf(buf, sizeof(buf), "Heatmap: %s  %s", heatmapName, (hv > 0.5f) ? "YES" : "no");
    } else {
      std::snprintf(buf, sizeof(buf), "Heatmap: %s  %.2f", heatmapName, static_cast<double>(hv));
    }
    heatmapInfo = buf;
  } else if (heatmapActive && heatmapName) {
    if (m_heatmapOverlay == HeatmapOverlay::FloodDepth) {
      char buf[128];
      std::snprintf(buf, sizeof(buf), "Heatmap: %s (sea %.2f)", heatmapName, static_cast<double>(m_seaLevel));
      heatmapInfo = buf;
    } else if (m_heatmapOverlay == HeatmapOverlay::PondingDepth) {
      char buf[128];
      std::snprintf(buf, sizeof(buf), "Heatmap: %s (max %.2f)", heatmapName, static_cast<double>(m_pondingMaxDepth));
      heatmapInfo = buf;
    } else if (m_heatmapOverlay == HeatmapOverlay::EvacuationTime ||
               m_heatmapOverlay == HeatmapOverlay::EvacuationUnreachable ||
               m_heatmapOverlay == HeatmapOverlay::EvacuationFlow) {
      ensureEvacuationScenarioUpToDate();
      const EvacuationResult& r = m_evacScenario.evac;
      const char* unit = m_evacCfg.evac.useTravelTime ? "s" : "steps";
      const double reachPct = (r.population > 0)
                                  ? (100.0 * static_cast<double>(r.reachablePopulation) /
                                     static_cast<double>(r.population))
                                  : 0.0;
      char buf[256];
      if (m_heatmapOverlay == HeatmapOverlay::EvacuationTime) {
        std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  reach %.0f%%  p95 %.1f%s", heatmapName,
                      EvacuationHazardModeName(m_evacCfg.hazardMode), reachPct,
                      static_cast<double>(r.p95EvacTime), unit);
      } else if (m_heatmapOverlay == HeatmapOverlay::EvacuationUnreachable) {
        std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  unreachPop %d  floodedPop %d", heatmapName,
                      EvacuationHazardModeName(m_evacCfg.hazardMode), r.unreachablePopulation,
                      r.floodedPopulation);
      } else {
        std::snprintf(buf, sizeof(buf), "Heatmap: %s (%s)  maxFlow %u", heatmapName,
                      EvacuationHazardModeName(m_evacCfg.hazardMode), r.maxEvacRoadFlow);
      }
      heatmapInfo = buf;
    } else if (m_heatmapOverlay == HeatmapOverlay::ServicesOverall ||
               m_heatmapOverlay == HeatmapOverlay::ServicesEducation ||
               m_heatmapOverlay == HeatmapOverlay::ServicesHealth ||
               m_heatmapOverlay == HeatmapOverlay::ServicesSafety) {
      ensureServicesHeatmapsUpToDate();
      float city = 0.0f;
      int fac = 0;
      if (m_heatmapOverlay == HeatmapOverlay::ServicesEducation) {
        city = m_servicesHeatmaps.educationSatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[0];
      } else if (m_heatmapOverlay == HeatmapOverlay::ServicesHealth) {
        city = m_servicesHeatmaps.healthSatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[1];
      } else if (m_heatmapOverlay == HeatmapOverlay::ServicesSafety) {
        city = m_servicesHeatmaps.safetySatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[2];
      } else {
        city = m_servicesHeatmaps.overallSatisfaction;
        fac = m_servicesHeatmaps.activeFacilities[0] + m_servicesHeatmaps.activeFacilities[1] +
              m_servicesHeatmaps.activeFacilities[2];
      }
      char buf[256];
      std::snprintf(buf, sizeof(buf), "Heatmap: %s  city %.0f%%  facilities %d", heatmapName,
                    static_cast<double>(city * 100.0f), fac);
      heatmapInfo = buf;
    } else {
      heatmapInfo = std::string("Heatmap: ") + heatmapName;
    }
  }

  const char* heatmapInfoC = (!heatmapInfo.empty()) ? heatmapInfo.c_str() : nullptr;

  // ---------------------------------------------------------------------
  // UI (scaled)
  // ---------------------------------------------------------------------
  // The world is rendered at full resolution, but the UI is rendered in a
  // "virtual" coordinate system and scaled up/down. This keeps UI text
  // readable and panels sized consistently across resolutions and DPI.
  Camera2D uiCam{};
  uiCam.zoom = uiScale;
  BeginMode2D(uiCam);

  const float uiTime = static_cast<float>(GetTime());
  const ui::Theme& uiTh = ui::GetTheme();
  const Vector2 mouseUi = mouseUiPosition(uiScale);

  m_renderer.drawHUD(m_world, m_camera, m_tool, m_roadBuildLevel, m_hovered, uiW, uiH, m_showHelp,
                     m_brushRadius, static_cast<int>(m_history.undoSize()), static_cast<int>(m_history.redoSize()),
                     simPaused(), simSpeed, m_saveSlot, m_showMinimap, inspectInfo, heatmapInfoC);

  drawBlueprintPanel(uiW, uiH);

  // Right-side panels share a simple docking layout that wraps to additional columns if vertical
  // space runs out. This prevents panels from falling off-screen when multiple tools are enabled.
  RightPanelDock rightDock(uiW, uiH);

  // Policy / budget panel.
  if (m_showPolicy) {
    SimConfig& cfg = m_sim.config();
    TradeModelSettings& tm = m_sim.tradeModel();
    const Stats& st = m_world.stats();

    // Used for partner names + commodity breakdown.
    ensureTradeMarketUpToDate();

    const int panelW = kPolicyPanelW;
    const int panelH = kPolicyPanelH;
    const Rectangle rect = rightDock.alloc(panelW, panelH);
    const int x0 = static_cast<int>(rect.x);
    const int y0 = static_cast<int>(rect.y);

    const Rectangle panelR{rect.x, rect.y, static_cast<float>(panelW), static_cast<float>(panelH)};
    ui::DrawPanel(panelR, uiTime, /*active=*/true);
    ui::DrawPanelHeader(panelR, "Policy & Budget", uiTime, /*active=*/true, /*titleSizePx=*/22);

    const int x = x0 + 12;
    int y = y0 + 42;

    ui::Text(x, y, 15, "Tab: select  |  [ / ]: adjust/toggle  |  Shift: coarse", uiTh.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    y += 22;

    const int rowH = 22;
    const int rows = 13;
    const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                          static_cast<float>(rows * rowH + 8)};
    ui::DrawPanelInset(listR, uiTime, /*active=*/true);

    bool changed = false;
    int rowY = y + 4;

    auto rowRectFor = [&](int yRow) -> Rectangle {
      return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                       static_cast<float>(rowH)};
    };

    const int xValueRight = x0 + panelW - 14;
    const int sliderH = 12;
    const int sliderW = 150;

    auto drawIntSliderRow = [&](int idx, std::string_view label, int& v, int vMin, int vMax, int step) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_policySelection = idx;
      }
      const bool selected = (m_policySelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[32];
      std::snprintf(buf, sizeof(buf), "%d", v);

      const int valW = ui::MeasureTextWidth(buf, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_policySelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      int vv = v;
      if (ui::SliderInt(2000 + idx, sr, vv, vMin, vMax, step, mouseUi, uiTime, /*enabled=*/true)) {
        v = vv;
        changed = true;
      }

      ui::Text(valX, rowY, 16, buf, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    auto drawToggleRow = [&](int idx, std::string_view label, bool& v) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_policySelection = idx;
      }
      const bool selected = (m_policySelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
      if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_policySelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      if (ui::Toggle(2100 + idx, tr, v, mouseUi, uiTime, /*enabled=*/true)) {
        changed = true;
      }

      rowY += rowH;
    };

    auto drawCycleRow = [&](int idx, std::string_view label, std::string_view value, auto onClick) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_policySelection = idx;
      }
      const bool selected = (m_policySelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      const int bw = std::max(110, ui::MeasureTextWidth(value, 16, /*bold=*/true, 1) + 18);
      const Rectangle br{static_cast<float>(xValueRight - bw),
                         static_cast<float>(rowY + (rowH - 18) / 2),
                         static_cast<float>(bw),
                         18.0f};
      if (ui::Button(2200 + idx, br, value, mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
        onClick();
        changed = true;
      }

      rowY += rowH;
    };

    drawIntSliderRow(0, "Residential tax", cfg.taxResidential, 0, 10, 1);
    drawIntSliderRow(1, "Commercial tax", cfg.taxCommercial, 0, 10, 1);
    drawIntSliderRow(2, "Industrial tax", cfg.taxIndustrial, 0, 10, 1);
    drawIntSliderRow(3, "Road maintenance", cfg.maintenanceRoad, 0, 5, 1);
    drawIntSliderRow(4, "Park maintenance", cfg.maintenancePark, 0, 5, 1);
    drawToggleRow(5, "Outside connection", cfg.requireOutsideConnection);
    drawIntSliderRow(6, "Park radius", cfg.parkInfluenceRadius, 0, 20, 1);

    drawToggleRow(7, "Trade market", tm.enabled);
    drawToggleRow(8, "Allow imports", tm.allowImports);
    drawToggleRow(9, "Allow exports", tm.allowExports);
    drawIntSliderRow(10, "Tariff %", tm.tariffPct, 0, 30, 1);

    const int partnerCount = static_cast<int>(m_tradeMarket.partners.size());
    auto partnerName = [&](int p) -> std::string {
      if (p < 0) return "Auto";
      if (p >= 0 && p < partnerCount) return m_tradeMarket.partners[static_cast<std::size_t>(p)].name;
      return "Auto";
    };

    std::string importLabel = partnerName(tm.importPartner);
    std::string exportLabel = partnerName(tm.exportPartner);

    drawCycleRow(11, "Import partner", importLabel, [&]() {
      const int count = std::max(1, partnerCount);
      int v = tm.importPartner;
      if (v < -1) v = -1;
      v += 1;
      if (v >= count) v = -1;
      tm.importPartner = v;
    });

    drawCycleRow(12, "Export partner", exportLabel, [&]() {
      const int count = std::max(1, partnerCount);
      int v = tm.exportPartner;
      if (v < -1) v = -1;
      v += 1;
      if (v >= count) v = -1;
      tm.exportPartner = v;
    });

    if (changed) {
      // Policies affect derived stats and several overlays.
      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;
      m_servicesHeatmapsDirty = true;
      m_fireRiskDirty = true;
      m_servicesPlanDirty = true;
      m_servicesPlanValid = false;
      m_outsideOverlayRoadToEdge.clear();
    }

    // Footer stats.
    y = rowY + 4;
    DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
    y += 10;

    const int tradeNet = st.exportRevenue - st.importCost;
    const int net = st.income - st.expenses;

    ui::Text(x, y, 18, TextFormat("Net: %+d   Income: %d   Expenses: %d", net, st.income, st.expenses), uiTh.text,
             /*bold=*/false, /*shadow=*/true, 1);
    y += 20;
    {
      std::string breakdown = TextFormat("Tax %d  Maint %d  Upg %d  Trade %+d", st.taxRevenue, st.maintenanceCost,
                                         st.upgradeCost, tradeNet);
      if (st.fireIncidentCost > 0) {
        breakdown += TextFormat("  Fire %d", st.fireIncidentCost);
      }
      ui::Text(x, y, 16, breakdown.c_str(), uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    }
    y += 18;
    ui::Text(x, y, 16,
             TextFormat("Land %.0f%%  Demand R/C/I %.0f%%/%.0f%%/%.0f%%  Tax/cap %.2f",
                        st.avgLandValue * 100.0f,
                        st.demandResidential * 100.0f,
                        st.demandCommercial * 100.0f,
                        st.demandIndustrial * 100.0f,
                        static_cast<double>(st.avgTaxPerCapita)),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

    y += 18;
    // Trade market snapshot (procedural partners + daily pricing).
    const int impIdx = st.tradeImportPartner;
    const int expIdx = st.tradeExportPartner;
    auto partnerByIdx = [&](int idx) -> std::string {
      if (idx < 0) return "None";
      if (idx >= 0 && idx < static_cast<int>(m_tradeMarket.partners.size())) {
        std::string n = m_tradeMarket.partners[static_cast<std::size_t>(idx)].name;
        if (n.size() > 14) {
          n = n.substr(0, 14);
          n += "...";
        }
        return n;
      }
      return "None";
    };

    std::string impName = partnerByIdx(impIdx);
    std::string expName = partnerByIdx(expIdx);
    if (st.tradeImportDisrupted) impName += " !";
    if (st.tradeExportDisrupted) expName += " !";

    ui::Text(x, y, 16,
             TextFormat("Mkt %.0f%%  Cap %d/%d  I:%s  E:%s",
                        static_cast<double>(st.tradeMarketIndex * 100.0f),
                        st.tradeImportCapacityPct, st.tradeExportCapacityPct,
                        impName.c_str(), expName.c_str()),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);


// -------------------------------------------------------------------------
// Policy Advisor: async policy optimization (SimCity-style "budget advisor")
// -------------------------------------------------------------------------
y += 22;
DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
y += 10;

const bool optRunning = (m_policyOptJob != nullptr);
const bool optHasBest = m_policyOptHaveResult && m_policyOptResultOk && !m_policyOptResult.bestByIteration.empty();
const char* presetName = PolicyOptPresetName(m_policyOptPreset);

// Header + action buttons.
ui::Text(x, y, 18, "Policy Advisor", uiTh.text, /*bold=*/false, /*shadow=*/true, 1);

const int btnH = 18;
const int btnW = 92;
const int btnGap = 8;
const int bxRight = x0 + panelW - 14;
const Rectangle presetR{static_cast<float>(bxRight - (btnW * 3 + btnGap * 2)),
                        static_cast<float>(y), static_cast<float>(btnW), static_cast<float>(btnH)};
const Rectangle applyR{static_cast<float>(bxRight - (btnW * 2 + btnGap)),
                       static_cast<float>(y), static_cast<float>(btnW), static_cast<float>(btnH)};
const Rectangle runR{static_cast<float>(bxRight - btnW),
                     static_cast<float>(y), static_cast<float>(btnW), static_cast<float>(btnH)};

if (ui::Button(22600, presetR, presetName, mouseUi, uiTime, /*enabled=*/!optRunning, /*primary=*/false)) {
  m_policyOptPreset = (m_policyOptPreset + 1) % kPolicyOptPresetCount;
}
if (ui::Button(22601, runR, optRunning ? "Running..." : "Optimize", mouseUi, uiTime,
               /*enabled=*/!optRunning, /*primary=*/true)) {
  startPolicyOptimization();
}
if (ui::Button(22602, applyR, "Apply best", mouseUi, uiTime,
               /*enabled=*/optHasBest && !optRunning, /*primary=*/false)) {
  applyPolicyCandidate(m_policyOptResult.best.policy, "Policy advisor: applied best");
}

y += 22;

// Compact settings (mouse-driven; disabled while a job is running).
const int innerW = panelW - 24;
const int labelW = 120;
const int valueW = 44;
const int ctrlX = x0 + 12 + labelW;
const int ctrlW = innerW - labelW - valueW;

auto drawAdvisorSlider = [&](int id, const char* label, int& v, int minV, int maxV, int step) {
  ui::Text(x, y + 2, 14, label, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  const Rectangle sr{static_cast<float>(ctrlX), static_cast<float>(y), static_cast<float>(ctrlW), 18.0f};
  const bool ch = ui::SliderInt(id, sr, v, minV, maxV, step, mouseUi, uiTime, /*enabled=*/!optRunning);
  ui::Text(x0 + 12 + innerW - valueW + 4, y + 2, 14, TextFormat("%d", v), uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
  y += 22;
  return ch;
};

drawAdvisorSlider(22610, "Horizon (days)", m_policyOptCfg.evalDays, 7, 180, 1);
drawAdvisorSlider(22611, "Iterations", m_policyOptCfg.iterations, 1, 40, 1);
drawAdvisorSlider(22614, "Tax radius", m_policyOptTaxRadius, 0, 6, 1);
drawAdvisorSlider(22615, "Maint radius", m_policyOptMaintRadius, 0, 3, 1);


// Status line.
const int itTotal = m_policyOptProgress.iterationsTotal.load();
const int itDone = m_policyOptProgress.iterationsCompleted.load();
const int cand = m_policyOptProgress.candidatesEvaluated.load();
const bool exhaustive = m_policyOptProgress.exhaustive.load();

if (optRunning) {
  ui::Text(x, y, 14,
           TextFormat("Status: running  (%s  it %d/%d  cand %d)",
                      exhaustive ? "exhaustive" : "CEM",
                      itDone, std::max(1, itTotal), cand),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  y += 18;
} else if (m_policyOptHaveResult) {
  if (m_policyOptResultOk) {
    ui::Text(x, y, 14,
             TextFormat("Status: done  (%s, %d cand)  day %d",
                        (m_policyOptResult.methodUsed == PolicyOptMethod::Exhaustive) ? "exhaustive" : "CEM",
                        m_policyOptResult.candidatesEvaluated,
                        m_policyOptResultDay),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  } else {
    std::string err = m_policyOptResultErr;
    if (err.size() > 54) err = err.substr(0, 54) + "...";
    ui::Text(x, y, 14, TextFormat("Status: failed  (%s)", err.c_str()), uiTh.bad,
             /*bold=*/false, /*shadow=*/true, 1);
  }
  y += 18;
} else {
  ui::Text(x, y, 14, "Status: idle  (choose a preset, then Optimize)", uiTh.textDim,
           /*bold=*/false, /*shadow=*/true, 1);
  y += 18;
}

// One-line summary of the best candidate found (if any).
if (optHasBest) {
  const PolicyCandidate& p = m_policyOptResult.best.policy;
  const PolicyEvalResult& r = m_policyOptResult.best.result;

  ui::Text(x, y, 14,
           TextFormat("Best: R%d C%d I%d  RdM%d PkM%d  |  score %.1f  money %+d  pop %d  happy %.0f%%",
                      p.taxResidential, p.taxCommercial, p.taxIndustrial,
                      p.maintenanceRoad, p.maintenancePark,
                      static_cast<double>(m_policyOptResult.best.score),
                      r.moneyDelta, r.popEnd,
                      static_cast<double>(r.happinessEnd * 100.0f)),
           uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
}
  }


  // City News panel (SimCity-style newspaper / advisor feed).
  if (m_showNewsPanel) {
    const Rectangle rect = rightDock.alloc(kNewsPanelW, kNewsPanelH);
    const Rectangle panelR{rect.x, rect.y, static_cast<float>(kNewsPanelW), static_cast<float>(kNewsPanelH)};
    drawNewsPanel(panelR);
  }

  // City Challenges panel (optional "goals" board).
  if (m_showChallengesPanel) {
    const Rectangle rect = rightDock.alloc(kChallengesPanelW, kChallengesPanelH);
    const Rectangle panelR{rect.x, rect.y, static_cast<float>(kChallengesPanelW), static_cast<float>(kChallengesPanelH)};
    drawChallengesPanel(panelR);
  }

  // City Treasury panel (municipal bonds / debt management).
  if (m_showBondsPanel) {
    const Rectangle rect = rightDock.alloc(kBondsPanelW, kBondsPanelH);
    const Rectangle panelR{rect.x, rect.y, static_cast<float>(kBondsPanelW), static_cast<float>(kBondsPanelH)};
    drawBondsPanel(panelR);
  }


  // Traffic model panel (experimental).
  if (m_showTrafficModel) {
    TrafficModelSettings& tm = m_sim.trafficModel();
    const Stats& st = m_world.stats();

    const int panelW = kTrafficPanelW;
    const int panelH = kTrafficPanelH;
    const Rectangle rect = rightDock.alloc(panelW, panelH);
    const int x0 = static_cast<int>(rect.x);
    const int y0 = static_cast<int>(rect.y);

    const Rectangle panelR{rect.x, rect.y, static_cast<float>(panelW), static_cast<float>(panelH)};
    ui::DrawPanel(panelR, uiTime, /*active=*/true);
    ui::DrawPanelHeader(panelR, "Traffic Model", uiTime, /*active=*/true, /*titleSizePx=*/22);

    const int x = x0 + 12;
    int y = y0 + 42;

    ui::Text(x, y, 15, "Tab: select  |  [ / ]: adjust/toggle  |  Shift: coarse", uiTh.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    y += 22;

    const int rowH = 22;
    const int rows = 9;
    const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                          static_cast<float>(rows * rowH + 8)};
    ui::DrawPanelInset(listR, uiTime, /*active=*/true);

    bool changed = false;
    int rowY = y + 4;

    auto rowRectFor = [&](int yRow) -> Rectangle {
      return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                       static_cast<float>(rowH)};
    };

    const int xValueRight = x0 + panelW - 14;
    const int sliderH = 12;
    const int sliderW = 150;

    auto drawToggleRow = [&](int idx, std::string_view label, bool& v) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_trafficModelSelection = idx;
      }
      const bool selected = (m_trafficModelSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
      if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_trafficModelSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      if (ui::Toggle(3000 + idx, tr, v, mouseUi, uiTime, /*enabled=*/true)) {
        changed = true;
      }

      rowY += rowH;
    };

    auto drawIntSliderRow = [&](int idx, std::string_view label, int& v, int vMin, int vMax, int step,
                                std::string_view suffix = {}) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_trafficModelSelection = idx;
      }
      const bool selected = (m_trafficModelSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[64];
      if (suffix.empty()) {
        std::snprintf(buf, sizeof(buf), "%d", v);
      } else {
        std::snprintf(buf, sizeof(buf), "%d %.*s", v, static_cast<int>(suffix.size()), suffix.data());
      }

      const int valW = ui::MeasureTextWidth(buf, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_trafficModelSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      int vv = v;
      if (ui::SliderInt(3100 + idx, sr, vv, vMin, vMax, step, mouseUi, uiTime, /*enabled=*/true)) {
        v = vv;
        changed = true;
      }

      ui::Text(valX, rowY, 16, buf, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    auto drawFloatSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax,
                                  const char* fmt) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_trafficModelSelection = idx;
      }
      const bool selected = (m_trafficModelSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[64];
      std::snprintf(buf, sizeof(buf), fmt, static_cast<double>(v));

      const int valW = ui::MeasureTextWidth(buf, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_trafficModelSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      float vv = v;
      if (ui::SliderFloat(3200 + idx, sr, vv, vMin, vMax, mouseUi, uiTime, /*enabled=*/true)) {
        v = vv;
        changed = true;
      }

      ui::Text(valX, rowY, 16, buf, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    drawToggleRow(0, "Congestion routing", tm.congestionAwareRouting);
    drawIntSliderRow(1, "Passes", tm.congestionIterations, 1, 16, 1);
    drawFloatSliderRow(2, "Alpha", tm.congestionAlpha, 0.0f, 2.0f, "%.2f");
    drawFloatSliderRow(3, "Beta", tm.congestionBeta, 1.0f, 8.0f, "%.1f");
    drawFloatSliderRow(4, "Cap scale", tm.congestionCapacityScale, 0.25f, 4.0f, "%.2f");
    drawFloatSliderRow(5, "Ratio clamp", tm.congestionRatioClamp, 1.0f, 10.0f, "%.1f");
    drawToggleRow(6, "Job capacity assign", tm.capacityAwareJobs);
    drawIntSliderRow(7, "Job iters", tm.jobAssignmentIterations, 1, 32, 1);

    // Job penalty: show as "NNNN (~X.X tiles)".
    {
      const int idx = 8;
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_trafficModelSelection = idx;
      }
      const bool rowSelected = (m_trafficModelSelection == idx);
      if (rowSelected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[64];
      std::snprintf(buf, sizeof(buf), "%d (~%.1f tiles)", tm.jobPenaltyBaseMilli,
                    static_cast<double>(tm.jobPenaltyBaseMilli) / 1000.0);

      const int valW = ui::MeasureTextWidth(buf, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_trafficModelSelection = idx;
      }

      const Color c = rowSelected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, "Job penalty", c, /*bold=*/false, /*shadow=*/true, 1);

      int vv = tm.jobPenaltyBaseMilli;
      if (ui::SliderInt(3100 + idx, sr, vv, 0, 50000, 500, mouseUi, uiTime, /*enabled=*/true)) {
        tm.jobPenaltyBaseMilli = vv;
        changed = true;
      }

      ui::Text(valX, rowY, 16, buf, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    }

    if (changed) {
      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;
    }

    // Footer stats.
    y = rowY + 4;
    DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
    y += 10;
    ui::Text(x, y, 16, TextFormat("Avg commute (time): %.1f   Congestion: %.0f%%",
                                  static_cast<double>(st.avgCommuteTime),
                                  static_cast<double>(st.trafficCongestion * 100.0f)),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  }


  // Resilience panel: flood + ponding + evacuation-to-edge analysis.
  if (m_showResiliencePanel) {
    const int w = m_world.width();
    const int h = m_world.height();

    const int panelW = kResiliencePanelW;
    const int panelH = kResiliencePanelH;
    const Rectangle rect = rightDock.alloc(panelW, panelH);
    const int x0 = static_cast<int>(rect.x);
    const int y0 = static_cast<int>(rect.y);

    const Rectangle panelR{rect.x, rect.y, static_cast<float>(panelW), static_cast<float>(panelH)};
    ui::DrawPanel(panelR, uiTime, /*active=*/true);
    ui::DrawPanelHeader(panelR, "Resilience (Flood + Evac)", uiTime, /*active=*/true, /*titleSizePx=*/22);

    const int x = x0 + 12;
    int y = y0 + 42;

    ui::Text(x, y, 15, "Tab: select  |  [ / ]: adjust/toggle  |  Shift: coarse", uiTh.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    y += 18;
    ui::Text(x, y, 14, "Run recomputes; Export dumps json/geojson/png to captures/.", uiTh.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    y += 22;

    auto heatmapLabel = [](HeatmapOverlay h) -> const char* {
      switch (h) {
      case HeatmapOverlay::Off: return "Off";
      case HeatmapOverlay::LandValue: return "Land value";
      case HeatmapOverlay::ParkAmenity: return "Park amenity";
      case HeatmapOverlay::WaterAmenity: return "Water amenity";
      case HeatmapOverlay::Pollution: return "Pollution";
      case HeatmapOverlay::TrafficSpill: return "Traffic spill";
      case HeatmapOverlay::FloodDepth: return "Flood depth";
      case HeatmapOverlay::PondingDepth: return "Ponding depth";
      case HeatmapOverlay::EvacuationTime: return "Evac time";
      case HeatmapOverlay::EvacuationUnreachable: return "Evac unreachable";
      case HeatmapOverlay::EvacuationFlow: return "Evac flow";
      default: return "?";
      }
    };

    const int rowH = 22;
    const int rows = 14;
    const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                          static_cast<float>(rows * rowH + 8)};
    ui::DrawPanelInset(listR, uiTime, /*active=*/true);

    int rowY = y + 4;
    auto rowRectFor = [&](int yRow) -> Rectangle {
      return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                       static_cast<float>(rowH)};
    };

    const int xValueRight = x0 + panelW - 14;
    const int sliderH = 12;
    const int sliderW = 170;

    bool hydrologyChanged = false;
    bool evacChanged = false;

    auto drawCycleButtonRow = [&](int idx, std::string_view label, std::string_view value) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_resilienceSelection = idx;
      }
      const bool selected = (m_resilienceSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      const int bw = std::max(110, ui::MeasureTextWidth(value, 16, /*bold=*/true, 1) + 18);
      const Rectangle br{static_cast<float>(xValueRight - bw),
                         static_cast<float>(rowY + (rowH - 18) / 2),
                         static_cast<float>(bw),
                         18.0f};
      if (ui::Button(4000 + idx, br, value, mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
        m_resilienceSelection = idx;
        adjustResiliencePanel(+1, /*bigStep=*/false);
      }

      rowY += rowH;
    };

    auto drawToggleRow = [&](int idx, std::string_view label, bool& v, const char* onMsg, const char* offMsg,
                             bool affectsHydrology, bool affectsEvac) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_resilienceSelection = idx;
      }
      const bool selected = (m_resilienceSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
      if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_resilienceSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      bool vv = v;
      if (ui::Toggle(4100 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
        v = vv;
        if (affectsHydrology) hydrologyChanged = true;
        if (affectsEvac) evacChanged = true;
        if (onMsg && offMsg) showToast(v ? onMsg : offMsg, 2.0f);
      }

      rowY += rowH;
    };

    auto drawFloatSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax, const char* fmt,
                                  bool affectsHydrology, bool affectsEvac) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_resilienceSelection = idx;
      }
      const bool selected = (m_resilienceSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[48];
      std::snprintf(buf, sizeof(buf), fmt, static_cast<double>(v));

      const int valW = ui::MeasureTextWidth(buf, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_resilienceSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      float vv = v;
      if (ui::SliderFloat(4200 + idx, sr, vv, vMin, vMax, mouseUi, uiTime, /*enabled=*/true)) {
        v = std::clamp(vv, vMin, vMax);
        if (affectsHydrology) hydrologyChanged = true;
        if (affectsEvac) evacChanged = true;
      }

      ui::Text(valX, rowY, 16, buf, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    auto drawIntSliderRow = [&](int idx, std::string_view label, int& v, int vMin, int vMax, int step,
                                bool affectsHydrology, bool affectsEvac) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_resilienceSelection = idx;
      }
      const bool selected = (m_resilienceSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[48];
      std::snprintf(buf, sizeof(buf), "%d", v);

      const int valW = ui::MeasureTextWidth(buf, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_resilienceSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      int vv = v;
      if (ui::SliderInt(4300 + idx, sr, vv, vMin, vMax, step, mouseUi, uiTime, /*enabled=*/true)) {
        v = std::clamp(vv, vMin, vMax);
        if (affectsHydrology) hydrologyChanged = true;
        if (affectsEvac) evacChanged = true;
      }

      ui::Text(valX, rowY, 16, buf, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    };

    // Rows
    drawCycleButtonRow(0, "Heatmap", heatmapLabel(m_heatmapOverlay));
    drawCycleButtonRow(1, "Hazards", EvacuationHazardModeName(m_evacCfg.hazardMode));
    drawToggleRow(2, "Bridges", m_evacCfg.bridgesPassable, "Evac bridges: passable", "Evac bridges: blocked",
                  /*affectsHydrology=*/false, /*affectsEvac=*/true);
    drawFloatSliderRow(3, "Sea level", m_seaLevel, 0.0f, 1.0f, "%.2f",
                       /*affectsHydrology=*/true, /*affectsEvac=*/false);
    drawToggleRow(4, "Flood coastal", m_seaFloodCfg.requireEdgeConnection,
                  "Flood connectivity: edge-connected", "Flood connectivity: any low cell",
                  /*affectsHydrology=*/true, /*affectsEvac=*/false);
    drawToggleRow(5, "Flood conn", m_seaFloodCfg.eightConnected,
                  "Flood connectivity: 8-neighbor", "Flood connectivity: 4-neighbor",
                  /*affectsHydrology=*/true, /*affectsEvac=*/false);
    drawToggleRow(6, "Pond edges", m_pondingCfg.includeEdges,
                  "Ponding: edges drain", "Ponding: sealed edges",
                  /*affectsHydrology=*/true, /*affectsEvac=*/false);
    drawFloatSliderRow(7, "Pond eps", m_pondingCfg.epsilon, 0.0f, 0.1f, "%.2g",
                       /*affectsHydrology=*/true, /*affectsEvac=*/false);
    drawFloatSliderRow(8, "Evac pond min", m_evacCfg.pondMinDepth, 0.0f, 1.0f, "%.3f",
                       /*affectsHydrology=*/false, /*affectsEvac=*/true);
    drawToggleRow(9, "Evac weight", m_evacCfg.evac.useTravelTime,
                  "Evac weighting: time", "Evac weighting: steps",
                  /*affectsHydrology=*/false, /*affectsEvac=*/true);
    drawIntSliderRow(10, "Walk cost", m_evacCfg.evac.walkCostMilli, 0, 100000, 250,
                     /*affectsHydrology=*/false, /*affectsEvac=*/true);
    drawIntSliderRow(11, "Road cap", m_evacCfg.evac.roadTileCapacity, 1, 100000, 1,
                     /*affectsHydrology=*/false, /*affectsEvac=*/true);

    // Run / Export actions as buttons.
    {
      const int idx = 12;
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_resilienceSelection = idx;
      }
      const bool rowSelected = (m_resilienceSelection == idx);
      if (rowSelected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      ui::Text(x, rowY, 16, "Run", rowSelected ? uiTh.text : uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

      const Rectangle br{static_cast<float>(x0 + panelW - 150),
                         static_cast<float>(rowY + 2),
                         136.0f,
                         18.0f};
      if (ui::Button(4400 + idx, br, "Recompute", mouseUi, uiTime, /*enabled=*/true, /*primary=*/true)) {
        m_resilienceSelection = idx;
        adjustResiliencePanel(+1, /*bigStep=*/false);
      }

      rowY += rowH;
    }
    {
      const int idx = 13;
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_resilienceSelection = idx;
      }
      const bool rowSelected = (m_resilienceSelection == idx);
      if (rowSelected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      ui::Text(x, rowY, 16, "Export", rowSelected ? uiTh.text : uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

      const Rectangle br{static_cast<float>(x0 + panelW - 150),
                         static_cast<float>(rowY + 2),
                         136.0f,
                         18.0f};
      if (ui::Button(4400 + idx, br, "Artifacts", mouseUi, uiTime, /*enabled=*/true, /*primary=*/false)) {
        m_resilienceSelection = idx;
        adjustResiliencePanel(+1, /*bigStep=*/false);
      }

      rowY += rowH;
    }

    // Apply dirtiness.
    if (hydrologyChanged) {
      invalidateHydrology();
      m_landValueDirty = true;
    }
    if (evacChanged) {
      m_evacDirty = true;
    }

    y = rowY + 2;
    DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
    y += 8;

    // Summary (computed lazily when dirty).
    ensureEvacuationScenarioUpToDate();
    const EvacuationResult& r = m_evacScenario.evac;
    const double reachPct =
        (r.population > 0) ? (100.0 * static_cast<double>(r.reachablePopulation) / static_cast<double>(r.population)) : 0.0;
    const char* unit = m_evacCfg.evac.useTravelTime ? "time" : "steps";

    if (w > 0 && h > 0) {
      ui::Text(x, y, 15,
               TextFormat("Sea: flooded %d  maxDepth %.3f", m_seaFlood.floodedCells, static_cast<double>(m_seaFlood.maxDepth)),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
      y += 18;
      ui::Text(x, y, 15,
               TextFormat("Pond: filled %d  maxDepth %.3f  vol %.0f", m_pondingFilledCells,
                          static_cast<double>(m_pondingMaxDepth), m_pondingVolume),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
      y += 18;
    }

    ui::Text(x, y, 15,
             TextFormat("Evac: reach %.0f%%  avg %.1f  p95 %.1f  cong %.0f%%  exits %d  (%s)",
                        reachPct,
                        static_cast<double>(r.avgEvacTime),
                        static_cast<double>(r.p95EvacTime),
                        static_cast<double>(r.congestionFrac * 100.0f),
                        r.exitSources,
                        unit),
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 18;
    ui::Text(x, y, 14, "Tip: with an evac heatmap active, Q+click shows the evacuation route to the nearest exit.",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  }


  // Districts panel (district paint + per-district policy multipliers; saved in v7+).
  if (m_showDistrictPanel) {
    SimConfig& cfg = m_sim.config();

    const int panelW = kDistrictPanelW;
    const int panelH = kDistrictPanelH;
    const Rectangle rect = rightDock.alloc(panelW, panelH);
    const int x0 = static_cast<int>(rect.x);
    const int y0 = static_cast<int>(rect.y);

    const Rectangle panelR{rect.x, rect.y, static_cast<float>(panelW), static_cast<float>(panelH)};
    ui::DrawPanel(panelR, uiTime, /*active=*/true);
    ui::DrawPanelHeader(panelR, "Districts", uiTime, /*active=*/true, /*titleSizePx=*/22);

    const int x = x0 + 12;
    int y = y0 + 42;

    ui::Text(x, y, 15, "Tab: select  |  [ / ]: adjust/toggle  |  Shift: coarse", uiTh.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
    y += 22;

    const int rowH = 22;
    const int rows = 9;
    const Rectangle listR{static_cast<float>(x0 + 12), static_cast<float>(y), static_cast<float>(panelW - 24),
                          static_cast<float>(rows * rowH + 8)};
    ui::DrawPanelInset(listR, uiTime, /*active=*/true);

    int rowY = y + 4;
    auto rowRectFor = [&](int yRow) -> Rectangle {
      return Rectangle{static_cast<float>(x0 + 14), static_cast<float>(yRow - 2), static_cast<float>(panelW - 28),
                       static_cast<float>(rowH)};
    };

    const int xValueRight = x0 + panelW - 14;
    const int sliderH = 12;
    const int sliderW = 150;

    bool policyChanged = false;

    auto drawToggleRow = [&](int idx, std::string_view label, bool& v) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_districtSelection = idx;
      }
      const bool selected = (m_districtSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
      if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_districtSelection = idx;
      }

      const Color c = selected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      bool vv = v;
      if (ui::Toggle(5000 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
        v = vv;
        policyChanged = true;
      }

      rowY += rowH;
    };

    auto drawFloatSliderRow = [&](int idx, std::string_view label, float& v, float vMin, float vMax, const char* fmt,
                                  std::string_view extraValue) {
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_districtSelection = idx;
      }
      const bool selected = (m_districtSelection == idx);
      if (selected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      char buf[96];
      if (!extraValue.empty()) {
        std::snprintf(buf, sizeof(buf), fmt, static_cast<double>(v));
        const std::string valueStr = std::string(buf) + " " + std::string(extraValue);
        const int valW = ui::MeasureTextWidth(valueStr, 16, /*bold=*/false, 1);
        const int valX = xValueRight - valW;

        const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                           static_cast<float>(rowY + (rowH - sliderH) / 2),
                           static_cast<float>(sliderW),
                           static_cast<float>(sliderH)};
        if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
          m_districtSelection = idx;
        }

        const Color c = selected ? uiTh.text : uiTh.textDim;
        ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

        float vv = v;
        if (ui::SliderFloat(5200 + idx, sr, vv, vMin, vMax, mouseUi, uiTime, /*enabled=*/true)) {
          v = std::clamp(vv, vMin, vMax);
          policyChanged = true;
        }

        ui::Text(valX, rowY, 16, valueStr, c, /*bold=*/false, /*shadow=*/true, 1);
      } else {
        std::snprintf(buf, sizeof(buf), fmt, static_cast<double>(v));
        const std::string_view valueStr = buf;

        const int valW = ui::MeasureTextWidth(valueStr, 16, /*bold=*/false, 1);
        const int valX = xValueRight - valW;

        const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                           static_cast<float>(rowY + (rowH - sliderH) / 2),
                           static_cast<float>(sliderW),
                           static_cast<float>(sliderH)};
        if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
          m_districtSelection = idx;
        }

        const Color c = selected ? uiTh.text : uiTh.textDim;
        ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

        float vv = v;
        if (ui::SliderFloat(5200 + idx, sr, vv, vMin, vMax, mouseUi, uiTime, /*enabled=*/true)) {
          v = std::clamp(vv, vMin, vMax);
          policyChanged = true;
        }

        ui::Text(valX, rowY, 16, valueStr, c, /*bold=*/false, /*shadow=*/true, 1);
      }

      rowY += rowH;
    };

    // Rows
    drawToggleRow(0, "Policies enabled", cfg.districtPoliciesEnabled);

    // Active district selector.
    {
      const int idx = 1;
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_districtSelection = idx;
      }
      const bool rowSelected = (m_districtSelection == idx);
      if (rowSelected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      int district = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
      const char* label = "Active district";
      const char* value = (district == 0) ? "0 (Default)" : TextFormat("%d", district);

      const int valW = ui::MeasureTextWidth(value, 16, /*bold=*/false, 1);
      const int valX = xValueRight - valW;

      const Rectangle sr{static_cast<float>(valX - 8 - sliderW),
                         static_cast<float>(rowY + (rowH - sliderH) / 2),
                         static_cast<float>(sliderW),
                         static_cast<float>(sliderH)};
      if (CheckCollisionPointRec(mouseUi, sr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_districtSelection = idx;
      }

      const Color c = rowSelected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, label, c, /*bold=*/false, /*shadow=*/true, 1);

      int vv = district;
      if (ui::SliderInt(5100 + idx, sr, vv, 0, kDistrictCount - 1, 1, mouseUi, uiTime, /*enabled=*/true)) {
        m_activeDistrict = std::clamp(vv, 0, kDistrictCount - 1);
      }

      ui::Text(valX, rowY, 16, value, c, /*bold=*/false, /*shadow=*/true, 1);
      rowY += rowH;
    }

    // Overlay toggle (boolean; tool may auto-enable it).
    {
      const int idx = 2;
      const Rectangle rr = rowRectFor(rowY);
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouseUi, rr)) {
        m_districtSelection = idx;
      }
      const bool rowSelected = (m_districtSelection == idx);
      if (rowSelected) ui::DrawSelectionHighlight(rr, uiTime, /*strong=*/true);

      const Rectangle tr{static_cast<float>(xValueRight - 46), static_cast<float>(rowY + (rowH - 16) / 2), 46.0f, 16.0f};
      if (CheckCollisionPointRec(mouseUi, tr) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        m_districtSelection = idx;
      }

      const Color c = rowSelected ? uiTh.text : uiTh.textDim;
      ui::Text(x, rowY, 16, "Overlay", c, /*bold=*/false, /*shadow=*/true, 1);

      bool vv = m_showDistrictOverlay;
      if (ui::Toggle(5000 + idx, tr, vv, mouseUi, uiTime, /*enabled=*/true)) {
        m_showDistrictOverlay = vv;
      }

      const char* overlayLabel = m_showDistrictOverlay ? "ON" : ((m_tool == Tool::District) ? "AUTO" : "OFF");
      const int overlayW = ui::MeasureTextWidth(overlayLabel, 14, /*bold=*/true, 1);
      ui::Text(xValueRight - 60 - overlayW, rowY + 2, 14, overlayLabel, uiTh.textDim, /*bold=*/true, /*shadow=*/true, 1);

      rowY += rowH;
    }

    drawToggleRow(3, "Borders", m_showDistrictBorders);

    // After the district selector row, compute the active district policy reference.
    const int district = std::clamp(m_activeDistrict, 0, kDistrictCount - 1);
    DistrictPolicy& dp = cfg.districtPolicies[static_cast<std::size_t>(district)];

    const int effResTax = static_cast<int>(std::lround(static_cast<double>(cfg.taxResidential) * dp.taxResidentialMult));
    const int effComTax = static_cast<int>(std::lround(static_cast<double>(cfg.taxCommercial) * dp.taxCommercialMult));
    const int effIndTax = static_cast<int>(std::lround(static_cast<double>(cfg.taxIndustrial) * dp.taxIndustrialMult));
    const int effRoadMaint =
        static_cast<int>(std::lround(static_cast<double>(cfg.maintenanceRoad) * dp.roadMaintenanceMult));
    const int effParkMaint =
        static_cast<int>(std::lround(static_cast<double>(cfg.maintenancePark) * dp.parkMaintenanceMult));

    drawFloatSliderRow(4, "Res tax mult", dp.taxResidentialMult, 0.0f, 3.0f, "x%.2f", TextFormat("(eff %d)", effResTax));
    drawFloatSliderRow(5, "Com tax mult", dp.taxCommercialMult, 0.0f, 3.0f, "x%.2f", TextFormat("(eff %d)", effComTax));
    drawFloatSliderRow(6, "Ind tax mult", dp.taxIndustrialMult, 0.0f, 3.0f, "x%.2f", TextFormat("(eff %d)", effIndTax));
    drawFloatSliderRow(7, "Road maint mult", dp.roadMaintenanceMult, 0.0f, 3.0f, "x%.2f",
                       TextFormat("(eff %d)", effRoadMaint));
    drawFloatSliderRow(8, "Park maint mult", dp.parkMaintenanceMult, 0.0f, 3.0f, "x%.2f",
                       TextFormat("(eff %d)", effParkMaint));

    if (policyChanged) {
      m_sim.refreshDerivedStats(m_world);
      m_trafficDirty = true;
      m_goodsDirty = true;
      m_landValueDirty = true;
      m_vehiclesDirty = true;
      m_transitPlanDirty = true;
      m_transitVizDirty = true;
      m_evacDirty = true;
    }

    y = rowY + 4;
    DrawLine(x, y, x0 + panelW - 12, y, uiTh.panelBorderHi);
    y += 10;

    ui::Text(x, y, 14, "Paint: tool 9.  ,/. change id.  Alt+Click pick.  Shift+Click fill.", uiTh.textDim,
             /*bold=*/false, /*shadow=*/true, 1);

    // Quick live snapshot for the selected district (uses cached land value when available).
    y += 18;
    const int ww = m_world.width();
    const int hh = m_world.height();
    const int n = ww * hh;
    const std::vector<float>* lv =
        (static_cast<int>(m_landValue.value.size()) == n) ? &m_landValue.value : nullptr;
    const DistrictStatsResult ds = ComputeDistrictStats(m_world, cfg, lv, roadToEdgeMask);
    const DistrictSummary& s = ds.districts[static_cast<std::size_t>(district)];
    const double lvPct = static_cast<double>(s.avgLandValue) * 100.0;
    const double accPct =
        (s.zoneTiles > 0) ? (100.0 * static_cast<double>(s.zoneTilesAccessible) / static_cast<double>(s.zoneTiles)) : -1.0;

    if (accPct >= 0.0) {
      ui::Text(x, y, 14, TextFormat("Stats: Pop %d  Emp %d  LV %.0f%%  Net %+d  Acc %.0f%%",
                                    s.population, s.employed, lvPct, s.net, accPct),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    } else {
      ui::Text(x, y, 14, TextFormat("Stats: Pop %d  Emp %d  LV %.0f%%  Net %+d  Acc --",
                                    s.population, s.employed, lvPct, s.net),
               uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    }
  }


  if (m_showServicesPanel) {
    const Rectangle rect = rightDock.alloc(kServicesPanelW, kServicesPanelH);
    drawServicesPanel(static_cast<int>(rect.x), static_cast<int>(rect.y));
  }


  if (m_showTransitPanel) {
    const Rectangle rect = rightDock.alloc(kTransitPanelW, kTransitPanelH);
    drawTransitPanel(static_cast<int>(rect.x), static_cast<int>(rect.y));
  }

  if (m_showWayfindingPanel) {
    const Rectangle rect = rightDock.alloc(kWayfindingPanelW, kWayfindingPanelH);
    drawWayfindingPanel(static_cast<int>(rect.x), static_cast<int>(rect.y));
  }

  if (m_showRoadUpgradePanel) {
    const Rectangle rect = rightDock.alloc(kRoadUpgradePanelW, kRoadUpgradePanelH);
    drawRoadUpgradePanel(static_cast<int>(rect.x), static_cast<int>(rect.y));
  }
  if (m_showAutoBuildPanel) {
    const Rectangle rect = rightDock.alloc(kAutoBuildPanelW, kAutoBuildPanelH);
    drawAutoBuildPanel(static_cast<int>(rect.x), static_cast<int>(rect.y));
  }

  if (m_showHeadlessLab) {
    const Rectangle rect = rightDock.alloc(kHeadlessLabPanelW, kHeadlessLabPanelH);
    drawHeadlessLabPanel(static_cast<int>(rect.x), static_cast<int>(rect.y));
  }

  drawVideoSettingsPanel(uiW, uiH);

  drawReportPanel(uiW, uiH);

  // Save manager panel draws on top of the HUD.
  drawSaveMenuPanel(uiW, uiH);

  // Road-drag overlay: show preview metrics without touching the HUD layout.
  if (m_roadDragActive) {
    const int fontSize = 16;
    const int smallFontSize = 14;
    const int pad = 10;
    const int lineH = fontSize + 4;
    const int smallLineH = smallFontSize + 4;

    const int pathTiles = static_cast<int>(m_roadDragPath.size());
    const int newTiles = m_roadDragBuildCost;
    const int upgradeTiles = m_roadDragUpgradeTiles;
    const int bridgeTiles = m_roadDragBridgeTiles;
    const int cost = m_roadDragMoneyCost;

    const bool valid = m_roadDragValid && (pathTiles > 0);
    const bool affordable = (m_world.stats().money >= cost);

    const char* roadClass = RoadClassName(m_roadBuildLevel);

    const std::string line1 = valid ? TextFormat("Road drag (%s)", roadClass)
                                    : TextFormat("Road drag (%s) - INVALID", roadClass);
    const std::string line2 = TextFormat("Path: %d tiles   New: %d   Upg: %d", pathTiles, newTiles, upgradeTiles);
    const std::string line3 = TextFormat("Bridges: %d   Cost: $%d%s", bridgeTiles, cost,
                                         (valid && !affordable) ? "  (need $)" : "");
    const std::string line4 = "Release: build   Esc/RMB: cancel";

    int maxW = ui::MeasureTextWidth(line1, fontSize, /*bold=*/true, 1);
    maxW = std::max(maxW, ui::MeasureTextWidth(line2, fontSize, /*bold=*/false, 1));
    maxW = std::max(maxW, ui::MeasureTextWidth(line3, fontSize, /*bold=*/false, 1));
    maxW = std::max(maxW, ui::MeasureTextWidth(line4, smallFontSize, /*bold=*/false, 1));

    const int boxW = maxW + pad * 2;
    const int boxH = pad * 2 + lineH * 3 + smallLineH;

    const Rectangle r{12.0f, 12.0f, static_cast<float>(boxW), static_cast<float>(boxH)};
    ui::DrawPanel(r, uiTime, /*active=*/true);

    int x = static_cast<int>(r.x) + pad;
    int y = static_cast<int>(r.y) + pad;

    const Color titleColor = valid ? uiTh.text : uiTh.accentBad;
    ui::Text(x, y, fontSize, line1, titleColor, /*bold=*/true, /*shadow=*/true, 1);
    y += lineH;

    ui::Text(x, y, fontSize, line2, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += lineH;

    const Color costColor = valid ? (affordable ? uiTh.accentOk : uiTh.accentBad) : uiTh.accentBad;
    ui::Text(x, y, fontSize, line3, costColor, /*bold=*/false, /*shadow=*/true, 1);
    y += lineH;

    ui::Text(x, y, smallFontSize, line4, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
  }


  // In-game software 3D preview (Shift+F11). This renders the *actual world mesh*
  // through the CPU renderer (Soft3D) and then uploads it as a texture.
  //
  // Controls:
  //   Shift+F11: toggle panel
  //   Ctrl+Arrows: rotate camera (Shift for bigger steps)
  //   Ctrl+P: toggle projection (iso/persp)
  //   Ctrl+R: reset view
  //   Ctrl+F11: export a high-res 3D render to /captures
  if (m_show3DPreview) {
    if (m_3dPreviewDirty || m_3dPreviewTex.id == 0) {
      Render3DConfig cfg = m_3dPreviewCfg;
      // Keep previews aligned with the current visual mood.
      ApplyWeatherTo3DCfg(cfg, m_renderer.weatherSettings());

      // Preview defaults to the main visual layer.
      const LandValueResult* lv = nullptr;
      const TrafficResult* tr = nullptr;
      const GoodsResult* gr = nullptr;
      PpmImage img = RenderWorld3D(m_world, m_3dPreviewLayer, cfg, lv, tr, gr);
      if (m_3dPreviewApplyGrade) {
        ApplyInGameAtmosphereGradeToPpm(img, m_timeSec, m_renderer.dayNightSettings(), m_renderer.weatherSettings());
      }

      (void)UploadPpmToTexture(m_3dPreviewTex, m_3dPreviewTexW, m_3dPreviewTexH, img);
      m_3dPreviewDirty = false;
    }

    const int panelW = 440;
    const int panelH = 292;
    const int x0 = 12;
    const int y0 = uiH - panelH - 12;

    const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                           static_cast<float>(panelH)};
    ui::DrawPanel(panelR, uiTime, /*active=*/true);
    ui::DrawPanelHeader(panelR, "3D Preview", uiTime, /*active=*/true, /*titleSizePx=*/22);

    int x = x0 + 12;
    int y = y0 + 42;
    ui::Text(x, y, 14, "Shift+F11 toggle  |  Ctrl+F11 export", uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 16;
    ui::Text(x, y, 14, "Ctrl+Arrows rotate (Shift=faster)  |  Ctrl+P proj  |  Ctrl+R reset",
             uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

    const int pad = 10;
    const int imgX = x0 + pad;
    const int imgY = y0 + 64;
    const int imgW = panelW - pad * 2;
    const int imgH = panelH - 74;

    const Rectangle imgR{static_cast<float>(imgX), static_cast<float>(imgY), static_cast<float>(imgW),
                         static_cast<float>(imgH)};
    ui::DrawPanelInset(imgR, uiTime, /*active=*/true);

    const Rectangle inner{imgR.x + 2.0f, imgR.y + 2.0f, imgR.width - 4.0f, imgR.height - 4.0f};

    if (m_3dPreviewTex.id != 0) {
      Rectangle src{0.0f, 0.0f, static_cast<float>(m_3dPreviewTexW), static_cast<float>(m_3dPreviewTexH)};
      DrawTexturePro(m_3dPreviewTex, src, inner, Vector2{0, 0}, 0.0f, WHITE);
    } else {
      ui::Text(static_cast<int>(inner.x) + 12, static_cast<int>(inner.y) + 12, 18, "(rendering...)", uiTh.textDim,
               /*bold=*/false, /*shadow=*/true, 1);
    }
  }

  drawPerfOverlay(uiW, uiH, uiTime);

  // Developer console draws above the HUD/panels but below transient toasts.
  if (m_console.isOpen()) {
    m_console.draw(uiW, uiH);
  }

  // Screenshot capture (queued from input so we can capture the freshly rendered frame)
  if (m_pendingScreenshot) {
    TakeScreenshot(m_pendingScreenshotPath.c_str());
    showToast(std::string("Screenshot saved: ") + m_pendingScreenshotPath, 3.0f);
    m_pendingScreenshot = false;
    m_pendingScreenshotPath.clear();
  }

  // POV HUD (ride-along camera status + stop button).
  if (m_pov.isActive()) {
    const int pad = 10;
    const int fontTitle = 16;
    const int fontSmall = 13;

    const std::string st = m_pov.statusText();
    const int textW = ui::MeasureTextWidth(st, fontSmall, /*bold=*/false, 1);

    const int boxW = std::max(260, textW + pad * 2 + 70);
    const int boxH = 52;

    const int x = pad;
    const int y = pad;
    const Rectangle r{static_cast<float>(x), static_cast<float>(y), static_cast<float>(boxW), static_cast<float>(boxH)};
    ui::DrawPanel(r, uiTime, /*active=*/true);

    ui::Text(x + pad, y + 8, fontTitle, "POV", uiTh.text, /*bold=*/true, /*shadow=*/true, 1);
    ui::Text(x + pad, y + 28, fontSmall, st, uiTh.textDim, /*bold=*/false, /*shadow=*/true, 1);

    const Rectangle stopR{r.x + r.width - 64.0f, r.y + 10.0f, 54.0f, 18.0f};
    if (ui::Button(9050, stopR, "Stop", mouseUi, uiTime, true, false)) {
      stopPov();
    }
  }

  // Toast / status message
  if (m_toastTimer > 0.0f && !m_toast.empty()) {
    const int fontSize = 18;
    const int pad = 10;

    const int textW = ui::MeasureTextWidth(m_toast, fontSize, /*bold=*/false, 1);
    const int boxW = textW + pad * 2;
    const int boxH = fontSize + pad * 2;

    const int x = (uiW - boxW) / 2;
    const int y = uiH - boxH - 18;

    const Rectangle r{static_cast<float>(x), static_cast<float>(y), static_cast<float>(boxW), static_cast<float>(boxH)};
    ui::DrawPanel(r, uiTime, /*active=*/true);
    ui::Text(x + pad, y + pad, fontSize, m_toast, uiTh.text, /*bold=*/false, /*shadow=*/true, 1);
  }

  EndMode2D();

  // Map export (queued from dev console so we can run the renderer with a valid
  // graphics context). Must run *outside* any active BeginMode2D() to avoid
  // nested mode state.
  if (m_pendingMapExport && !m_pendingMapExportPath.empty()) {
    const std::string path = m_pendingMapExportPath;
    const int maxSize = m_pendingMapExportMaxSize;
    m_pendingMapExport = false;
    m_pendingMapExportPath.clear();
    m_pendingMapExportMaxSize = 4096;

    const bool ok = m_renderer.exportWorldOverview(m_world, path.c_str(), maxSize, m_timeSec, /*includeScreenFx=*/true);
    showToast(ok ? (std::string("Map exported: ") + path)
                 : (std::string("Map export failed: ") + path),
             4.0f);
  }
  // Layered map export (terrain/decals/structures/overlays, plus optional weather-only FX layer).
  // Must run *outside* any active BeginMode2D() to avoid nested mode state.
  if (m_pendingMapLayersExport && !m_pendingMapLayersPrefix.empty()) {
    const std::string prefix = m_pendingMapLayersPrefix;
    const int maxSize = m_pendingMapLayersMaxSize;
    m_pendingMapLayersExport = false;
    m_pendingMapLayersPrefix.clear();
    m_pendingMapLayersMaxSize = 4096;

    namespace fs = std::filesystem;

    // Ensure the output directory exists (best effort).
    {
      std::error_code ec;
      const fs::path parent = fs::path(prefix).parent_path();
      if (!parent.empty()) {
        fs::create_directories(parent, ec);
      }
    }

    auto withSuffix = [&](const char* suffix) -> std::string {
      const fs::path p(prefix);
      const fs::path dir = p.parent_path();
      const std::string stem = p.stem().string();
      std::string ext = p.extension().string();
      if (ext.empty()) ext = ".png";
      return (dir / (stem + suffix + ext)).string();
    };

    const std::uint32_t prevMask = m_renderer.layerMask();
    bool ok = true;

    // 1) Composite (current layer mask + full screen FX) so the set contains a "what you see" reference.
    {
      const std::string out = withSuffix("_composite");
      ok = ok && m_renderer.exportWorldOverview(m_world, out.c_str(), maxSize, m_timeSec, /*includeScreenFx=*/true);
    }

    // 2) Per-layer exports (transparent background; screen-space FX disabled).
    auto exportLayer = [&](std::uint32_t mask, const char* suffix) {
      m_renderer.setLayerMask(mask);
      const std::string out = withSuffix(suffix);
      ok = ok && m_renderer.exportWorldOverview(m_world, out.c_str(), maxSize, m_timeSec, /*includeScreenFx=*/false);
    };

    exportLayer(Renderer::kLayerTerrain, "_terrain");
    exportLayer(Renderer::kLayerDecals, "_decals");
    exportLayer(Renderer::kLayerStructures, "_structures");
    exportLayer(Renderer::kLayerOverlays, "_overlays");

    // 3) Weather-only screen FX (drawn over a blank world), useful for compositing.
    {
      m_renderer.setLayerMask(0u);
      const std::string out = withSuffix("_weather_fx");
      ok = ok && m_renderer.exportWorldOverview(m_world, out.c_str(), maxSize, m_timeSec, /*includeScreenFx=*/true);
    }

    // Restore user layer mask.
    m_renderer.setLayerMask(prevMask);

    const fs::path p(prefix);
    const std::string ext = p.extension().empty() ? ".png" : p.extension().string();
    const std::string base = (p.parent_path() / p.stem()).string();

    showToast(ok ? (std::string("Layer exports: ") + base + "_*" + ext)
                 : (std::string("Layer export failed: ") + prefix),
             4.0f);
  }

  // Software 3D render export (queued from dev console or Ctrl+F11).
  if (m_pendingRender3D && !m_pendingRender3DPath.empty()) {
    namespace fs = std::filesystem;

    const std::string path = m_pendingRender3DPath;
    Render3DConfig cfg = m_pendingRender3DCfg;
    const ExportLayer layer = m_pendingRender3DLayer;
    const bool applyGrade = m_pendingRender3DApplyGrade;

    m_pendingRender3D = false;
    m_pendingRender3DPath.clear();

    // Ensure output directory exists (best effort).
    {
      std::error_code ec;
      const fs::path parent = fs::path(path).parent_path();
      if (!parent.empty()) fs::create_directories(parent, ec);
    }

    // Align fog/lighting to current weather (the software renderer supports true depth fog).
    ApplyWeatherTo3DCfg(cfg, m_renderer.weatherSettings());

    // Prepare derived inputs if the selected layer needs them.
    const LandValueResult* lvPtr = nullptr;
    const TrafficResult* trPtr = nullptr;
    const GoodsResult* grPtr = nullptr;

    const bool render3d_requireOutside = m_sim.config().requireOutsideConnection;
    const bool render3d_needRoadToEdgeMask = render3d_requireOutside &&
        (layer == ExportLayer::Traffic || layer == ExportLayer::LandValue || layer == ExportLayer::GoodsTraffic ||
         layer == ExportLayer::GoodsFill || layer == ExportLayer::Noise);

    const std::vector<std::uint8_t>* render3d_roadToEdgeMask = nullptr;
    if (render3d_needRoadToEdgeMask) {
      ComputeRoadsConnectedToEdge(m_world, m_outsideOverlayRoadToEdge);
      render3d_roadToEdgeMask = &m_outsideOverlayRoadToEdge;
    }

    // Traffic (commute)
    if (layer == ExportLayer::Traffic || layer == ExportLayer::LandValue || layer == ExportLayer::Noise) {
      if (m_trafficDirty) {
        const float share = (m_world.stats().population > 0)
                                ? (static_cast<float>(m_world.stats().employed) /
                                   static_cast<float>(m_world.stats().population))
                                : 0.0f;

        TrafficConfig tc;
        tc.requireOutsideConnection = render3d_requireOutside;
        {
          const TrafficModelSettings& tm = m_sim.trafficModel();
          tc.congestionAwareRouting = tm.congestionAwareRouting;
          tc.congestionIterations = tm.congestionIterations;
          tc.congestionAlpha = tm.congestionAlpha;
          tc.congestionBeta = tm.congestionBeta;
          tc.congestionCapacityScale = tm.congestionCapacityScale;
          tc.congestionRatioClamp = tm.congestionRatioClamp;
          tc.capacityAwareJobs = tm.capacityAwareJobs;
          tc.jobAssignmentIterations = tm.jobAssignmentIterations;
          tc.jobPenaltyBaseMilli = tm.jobPenaltyBaseMilli;
        }

        const std::vector<std::uint8_t>* pre = (tc.requireOutsideConnection ? render3d_roadToEdgeMask : nullptr);
        m_traffic = ComputeCommuteTraffic(m_world, tc, share, pre);
        m_trafficDirty = false;
        m_carbonDirty = true;
      }
      trPtr = &m_traffic;
    }

    // Goods
    if (layer == ExportLayer::GoodsTraffic || layer == ExportLayer::GoodsFill || layer == ExportLayer::Noise) {
      if (m_goodsDirty) {
        GoodsConfig gc;
        gc.requireOutsideConnection = render3d_requireOutside;

        const Stats& st = m_world.stats();
        const TradeModelSettings& ts = m_sim.tradeModel();
        if (ts.enabled) {
          gc.allowImports = ts.allowImports;
          gc.allowExports = ts.allowExports;
          gc.importCapacityPct = std::clamp(st.tradeImportCapacityPct, 0, 100);
          gc.exportCapacityPct = std::clamp(st.tradeExportCapacityPct, 0, 100);
        }
        const std::vector<std::uint8_t>* pre = (gc.requireOutsideConnection ? render3d_roadToEdgeMask : nullptr);
        m_goods = ComputeGoodsFlow(m_world, gc, pre);
        m_goodsDirty = false;
        m_carbonDirty = true;
      }
      grPtr = &m_goods;
    }

    // Land value
    if (layer == ExportLayer::LandValue) {
      if (m_landValueDirty ||
          m_landValue.value.size() != static_cast<std::size_t>(std::max(0, m_world.width()) * std::max(0, m_world.height()))) {
        LandValueConfig lc;
        lc.requireOutsideConnection = render3d_requireOutside;
        lvPtr = nullptr;
        m_landValue = ComputeLandValue(m_world, lc, trPtr, render3d_roadToEdgeMask);
        m_landValueDirty = false;
      }
      lvPtr = &m_landValue;
    }

    PpmImage img = RenderWorld3D(m_world, layer, cfg, lvPtr, trPtr, grPtr);
    if (applyGrade) {
      ApplyInGameAtmosphereGradeToPpm(img, m_timeSec, m_renderer.dayNightSettings(), m_renderer.weatherSettings());
    }

    std::string err;
    const bool ok = WriteImageAuto(path, img, err);
    showToast(ok ? (std::string("3D render: ") + path)
                 : (std::string("3D render failed: ") + (err.empty() ? path : err)),
             4.0f);
  }

  // City dossier export runs asynchronously (see updateDossierExportJob).

  EndDrawing();
}


void Game::drawHeadlessLabPanel(int x0, int y0)
{
  const ui::Theme& uiTh = ui::GetTheme();
  const float uiTime = static_cast<float>(GetTime());
  const Vector2 mouseUi = mouseUiPosition(m_uiScale);

  const int panelW = kHeadlessLabPanelW;
  const int panelH = kHeadlessLabPanelH;

  // Keep the mining defaults tracking the current map size.
  m_labMineCfg.w = m_world.width();
  m_labMineCfg.h = m_world.height();

  const Rectangle panelR{static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(panelW),
                         static_cast<float>(panelH)};
  ui::DrawPanel(panelR, uiTime, true);
  ui::DrawPanelHeader(panelR, "City Lab", uiTime, true);

  // Header actions.
  {
    const Rectangle closeR{panelR.x + panelR.width - 72.0f, panelR.y + 10.0f, 60.0f, 18.0f};
    if (ui::Button(9200, closeR, "Close", mouseUi, uiTime, true)) {
      m_showHeadlessLab = false;
      ui::ClearActiveWidget();
      showToast("City Lab: OFF", 1.5f);
      return;
    }
  }

  int y = y0 + 42;
  const int leftPad = 10;
  const int innerPad = 8;
  const int innerX = x0 + leftPad;
  const int innerW = panelW - 2 * leftPad;

  // Tabs.
  {
    const int tabGap = 6;
    const int tabH = 20;
    const int tabW = (innerW - tabGap * 2) / 3;
    const Rectangle tabMineR{static_cast<float>(innerX), static_cast<float>(y), static_cast<float>(tabW),
                             static_cast<float>(tabH)};
    const Rectangle tabDosR{static_cast<float>(innerX + tabW + tabGap), static_cast<float>(y),
                            static_cast<float>(tabW), static_cast<float>(tabH)};
    const Rectangle tabMeshR{static_cast<float>(innerX + (tabW + tabGap) * 2), static_cast<float>(y),
                             static_cast<float>(tabW), static_cast<float>(tabH)};

    if (ui::Button(9201, tabMineR, "Mining", mouseUi, uiTime, true, m_headlessLabTab == 0)) {
      m_headlessLabTab = 0;
      ui::ClearActiveWidget();
    }
    if (ui::Button(9202, tabDosR, "Dossier", mouseUi, uiTime, true, m_headlessLabTab == 1)) {
      m_headlessLabTab = 1;
      ui::ClearActiveWidget();
    }
    if (ui::Button(9203, tabMeshR, "Mesh", mouseUi, uiTime, true, m_headlessLabTab == 2)) {
      m_headlessLabTab = 2;
      ui::ClearActiveWidget();
    }

    y += tabH + 10;
  }

  auto makeDossierCfg = [&](const std::filesystem::path& outDir) {
    CityDossierConfig cfg;
    cfg.outDir = outDir;
    cfg.exportScale = std::clamp(m_labDossierExportScale, 1, 4);
    cfg.exportIso = m_labDossierIso;
    cfg.export3d = m_labDossier3d;
    cfg.format = (m_labDossierFormat == 1) ? "ppm" : "png";

    if (m_labDossierPreset == 1) {
      // Fast preset: fewer layers, smaller footprint.
      cfg.layers2d = {
          ExportLayer2D::Terrain,
          ExportLayer2D::Overlay,
          ExportLayer2D::Height,
          ExportLayer2D::LandValue,
          ExportLayer2D::FloodDepth,
      };
      cfg.export3dPreview = false;
    }

    return cfg;
  };

  // ---------------------------------------------------------------------------
  // Mining tab
  // ---------------------------------------------------------------------------
  if (m_headlessLabTab == 0) {
    const Rectangle cfgR{static_cast<float>(innerX), static_cast<float>(y), static_cast<float>(innerW), 198.0f};
    ui::DrawPanelInset(cfgR, uiTime, true);

    const int labelW = 120;
    const int labelX = innerX + innerPad;
    const int ctrlX = labelX + labelW;
    const int ctrlW = innerW - 2 * innerPad - labelW;

    int cy = y + innerPad;

    // Title / current map.
    {
      ui::Text(labelX, cy, 14, TextFormat("Map: %dx%d  |  Seed: %llu", m_world.width(), m_world.height(),
                                         static_cast<unsigned long long>(m_cfg.seed)),
               uiTh.text);
      cy += 20;
    }

    // Objective (cycle).
    {
      ui::Text(labelX, cy + 2, 14, "Objective", uiTh.textDim);
      const Rectangle btnR{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f};
      if (ui::Button(9210, btnR, MineObjectiveName(m_labMineCfg.objective), mouseUi, uiTime, true)) {
        // Cycle objective.
        switch (m_labMineCfg.objective) {
          case MineObjective::Balanced: m_labMineCfg.objective = MineObjective::Growth; break;
          case MineObjective::Growth: m_labMineCfg.objective = MineObjective::Resilient; break;
          case MineObjective::Resilient: m_labMineCfg.objective = MineObjective::Chaos; break;
          case MineObjective::Chaos: m_labMineCfg.objective = MineObjective::Balanced; break;
        }
        m_mineObjective = m_labMineCfg.objective;
      }
      cy += 22;
    }

    // Samples.
    {
      ui::Text(labelX, cy + 2, 14, "Samples", uiTh.textDim);
      ui::SliderInt(9211, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                    m_labMineCfg.samples, 10, 5000, mouseUi, uiTime);
      cy += 22;
    }

    // Days.
    {
      ui::Text(labelX, cy + 2, 14, "Days", uiTh.textDim);
      ui::SliderInt(9212, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                    m_labMineCfg.days, 0, 365, mouseUi, uiTime);
      cy += 22;
    }

    // Hydrology metrics toggle.
    {
      ui::Text(labelX, cy + 2, 14, "Hydrology", uiTh.textDim);
      ui::Toggle(9213, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f},
                 m_labMineCfg.hydrologyEnabled, mouseUi, uiTime);
      ui::Text(ctrlX + 128, cy + 2, 14, "(flood/pond scoring)", uiTh.textDim);
      cy += 22;
    }

    // Sea level override.
    {
      bool seaOverride = !std::isnan(m_labMineCfg.seaLevelOverride);
      const bool was = seaOverride;
      ui::Text(labelX, cy + 2, 14, "Sea override", uiTh.textDim);
      ui::Toggle(9214, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, seaOverride,
                 mouseUi, uiTime);
      if (seaOverride != was) {
        if (seaOverride)
          m_labMineCfg.seaLevelOverride = m_procCfg.seaLevel;
        else
          m_labMineCfg.seaLevelOverride = std::numeric_limits<float>::quiet_NaN();
      }
      if (seaOverride) {
        float sea = m_labMineCfg.seaLevelOverride;
        ui::SliderFloat(9215,
                        Rectangle{static_cast<float>(ctrlX + 128), static_cast<float>(cy),
                                  static_cast<float>(ctrlW - 128), 18.0f},
                        sea, 0.0f, 1.0f, mouseUi, uiTime);
        m_labMineCfg.seaLevelOverride = sea;
      }
      cy += 22;
    }

    // Seed start controls.
    {
      ui::Text(labelX, cy + 2, 14, "Seed start", uiTh.textDim);
      const std::string seedStr = HexU64(m_labMineCfg.seedStart);

      const int btnW = 46;
      const int btnGap = 4;
      const int totalBtnW = btnW * 3 + btnGap * 2;
      const int seedTextW = std::max(0, ctrlW - totalBtnW - 6);

      ui::Text(ctrlX, cy + 2, 14, seedStr, uiTh.text, false, true, 1);

      const int bx = ctrlX + seedTextW + 6;
      const Rectangle b1{static_cast<float>(bx), static_cast<float>(cy), static_cast<float>(btnW), 18.0f};
      const Rectangle b2{static_cast<float>(bx + btnW + btnGap), static_cast<float>(cy), static_cast<float>(btnW),
                         18.0f};
      const Rectangle b3{static_cast<float>(bx + (btnW + btnGap) * 2), static_cast<float>(cy),
                         static_cast<float>(btnW), 18.0f};

      if (ui::Button(9216, b1, "@1", mouseUi, uiTime, true))
        m_labMineCfg.seedStart = 1;
      if (ui::Button(9217, b2, "@city", mouseUi, uiTime, true))
        m_labMineCfg.seedStart = m_cfg.seed;
      if (ui::Button(9218, b3, "rand", mouseUi, uiTime, true))
        m_labMineCfg.seedStart = TimeSeed();

      cy += 22;
    }

    // Seed step.
    {
      ui::Text(labelX, cy + 2, 14, "Seed step", uiTh.textDim);
      ui::SliderU64(9219, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                     m_labMineCfg.seedStep, 1ULL, 128ULL, mouseUi, uiTime);
      cy += 22;
    }

    // Top seeds filtering.
    {
      ui::Text(labelX, cy + 2, 14, "Top K", uiTh.textDim);
      ui::SliderInt(9220, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                    m_mineTopK, 1, 200, mouseUi, uiTime);
      cy += 22;

      ui::Text(labelX, cy + 2, 14, "Pareto", uiTh.textDim);
      ui::Toggle(9229, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, m_mineTopPareto,
                 mouseUi, uiTime);
      ui::Text(ctrlX + 128, cy + 2, 14, "(multi-objective)", uiTh.textDim);
      cy += 22;

      if (m_mineTopPareto) {
        ui::Text(labelX, cy + 2, 14, "Crowding", uiTh.textDim);
        ui::Toggle(9239, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, m_mineTopParetoCrowding,
                   mouseUi, uiTime);
        ui::Text(ctrlX + 128, cy + 2, 14, "(diversify fronts)", uiTh.textDim);
        cy += 22;
      } else {
        ui::Text(labelX, cy + 2, 14, "Diverse", uiTh.textDim);
        ui::Toggle(9221, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, m_mineTopDiverse,
                   mouseUi, uiTime);
        ui::Text(ctrlX + 128, cy + 2, 14, "(MMR)", uiTh.textDim);
        cy += 22;
      }
    }

    // Auto-run controls.
    {
      ui::Text(labelX, cy + 2, 14, "Auto-run", uiTh.textDim);
      ui::Toggle(9222, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, m_mineAutoRun,
                 mouseUi, uiTime);
      cy += 22;

      ui::Text(labelX, cy + 2, 14, "Steps/frame", uiTh.textDim);
      ui::SliderInt(9223, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                    m_mineAutoStepsPerFrame, 1, 50, mouseUi, uiTime);
      cy += 22;

      ui::Text(labelX, cy + 2, 14, "Budget ms", uiTh.textDim);
      ui::SliderFloat(9224, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                      m_mineAutoBudgetMs, 0.5f, 33.0f, mouseUi, uiTime);
      cy += 22;
    }

    // Session controls.
    {
      const bool hasSession = static_cast<bool>(m_mineSession);
      const int btnH = 18;
      const int btnGap = 6;
      const int btnW = (innerW - innerPad * 2 - btnGap) / 2;
      const int bx = innerX + innerPad;
      const int by = y0 + 42 + 20 + 10 + static_cast<int>(cfgR.height) - innerPad - btnH;

      const Rectangle startR{static_cast<float>(bx), static_cast<float>(by), static_cast<float>(btnW),
                             static_cast<float>(btnH)};
      const Rectangle resetR{static_cast<float>(bx + btnW + btnGap), static_cast<float>(by), static_cast<float>(btnW),
                             static_cast<float>(btnH)};

      if (!hasSession) {
        if (ui::Button(9225, startR, "Start mining", mouseUi, uiTime, true)) {
          endPaintStroke();
          m_mineSession = std::make_unique<MineSession>(m_labMineCfg, m_procCfg, m_sim.config());
          m_labMineTopCacheIndex = -1;
          m_labMineTopIndices.clear();
          m_labMineTopSelection = 0;
          m_labMineTopScroll = 0;
          showToast("Mine session started", 2.0f);
        }
        ui::Button(9226, resetR, "Reset", mouseUi, uiTime, false);
      } else {
        const char* pauseLabel = m_mineAutoRun ? "Pause" : "Resume";
        if (ui::Button(9225, startR, pauseLabel, mouseUi, uiTime, true)) {
          m_mineAutoRun = !m_mineAutoRun;
        }
        if (ui::Button(9226, resetR, "Clear session", mouseUi, uiTime, true)) {
          m_mineSession.reset();
          m_labMineTopCacheIndex = -1;
          m_labMineTopIndices.clear();
          m_labMineTopSelection = 0;
          m_labMineTopScroll = 0;
          showToast("Mine session cleared", 2.0f);
        }
      }
    }

    y += static_cast<int>(cfgR.height) + 10;

    // Status line + manual stepping.
    {
      const bool hasSession = static_cast<bool>(m_mineSession);
      if (!hasSession) {
        ui::Text(innerX + 4, y, 14, "Session: (none)", uiTh.textDim);
        y += 22;
      } else {
        const int idx = m_mineSession->index();
        const int total = m_mineSession->total();
        ui::Text(innerX + 4, y, 14,
                 TextFormat("Session: %d / %d  |  last %.2f ms", idx, total, m_mineLastAutoStepMs), uiTh.text);
        y += 22;

        const int btnH = 18;
        const int btnW = (innerW - 6) / 2;
        const Rectangle step1R{static_cast<float>(innerX), static_cast<float>(y), static_cast<float>(btnW),
                               static_cast<float>(btnH)};
        const Rectangle step10R{static_cast<float>(innerX + btnW + 6), static_cast<float>(y),
                                static_cast<float>(btnW), static_cast<float>(btnH)};

        if (ui::Button(9227, step1R, "Step +1", mouseUi, uiTime, true))
          m_mineManualStepRequests += 1;
        if (ui::Button(9228, step10R, "Step +10", mouseUi, uiTime, true))
          m_mineManualStepRequests += 10;

        y += 24;
      }
    }

    // Top seeds list.
    {
      const Rectangle listR{static_cast<float>(innerX), static_cast<float>(y), static_cast<float>(innerW), 200.0f};
      ui::DrawPanelInset(listR, uiTime, true);

      if (!m_mineSession) {
        ui::Text(innerX + 10, y + 10, 14, "Top seeds will appear here once mining starts.", uiTh.textDim);
        y += static_cast<int>(listR.height) + 10;
      } else {
        const int curIdx = m_mineSession->index();
        if (curIdx != m_labMineTopCacheIndex || m_mineTopK != m_labMineTopCacheTopK ||
            m_mineTopDiverse != m_labMineTopCacheDiverse || m_mineTopPareto != m_labMineTopCachePareto ||
            m_mineTopParetoCrowding != m_labMineTopCacheParetoCrowding) {
          auto& recs = m_mineSession->records();
          if (m_mineTopPareto) {
            std::vector<ParetoObjective> obj;
            obj.push_back({MineMetric::Population, true});
            obj.push_back({MineMetric::Happiness, true});
            obj.push_back({MineMetric::AvgLandValue, true});
            obj.push_back({MineMetric::TrafficCongestion, false});
            if (m_mineSession->config().hydrologyEnabled) obj.push_back({MineMetric::FloodRisk, false});

            const ParetoResult pr = ComputePareto(recs, obj);
            for (std::size_t ii = 0; ii < recs.size(); ++ii) {
              recs[ii].paretoRank = pr.rank[ii];
              recs[ii].paretoCrowding = pr.crowding[ii];
            }
            m_labMineTopIndices = SelectTopParetoIndices(pr, m_mineTopK, m_mineTopParetoCrowding);
          } else {
            m_labMineTopIndices = SelectTopIndices(recs, m_mineTopK, m_mineTopDiverse);
          }

          m_labMineTopCacheIndex = curIdx;
          m_labMineTopCacheTopK = m_mineTopK;
          m_labMineTopCacheDiverse = m_mineTopDiverse;
          m_labMineTopCachePareto = m_mineTopPareto;
          m_labMineTopCacheParetoCrowding = m_mineTopParetoCrowding;
          m_labMineTopSelection = std::clamp(m_labMineTopSelection, 0, std::max(0, (int)m_labMineTopIndices.size() - 1));
          m_labMineTopScroll = std::clamp(m_labMineTopScroll, 0, std::max(0, (int)m_labMineTopIndices.size() - 1));
        }

        const int rowH = 22;
        const int rows = (int)m_labMineTopIndices.size();
        const int viewRows = (int)((listR.height - 16) / rowH);

        m_labMineTopScroll = std::clamp(m_labMineTopScroll, 0, std::max(0, rows - viewRows));

        // Scrollbar.
        const Rectangle barR{listR.x + listR.width - 12.0f, listR.y + 8.0f, 10.0f, listR.height - 16.0f};
        ui::ScrollbarV(9230, barR, rows, viewRows, m_labMineTopScroll, mouseUi, uiTime);

        const Rectangle contentR = ui::ContentRectWithScrollbar(listR);

        const int baseX = (int)contentR.x + 6;
        int ry = (int)contentR.y + 8;

        const int start = m_labMineTopScroll;
        const int end = std::min(rows, start + viewRows);

        for (int i = start; i < end; ++i) {
          const int topIdx = m_labMineTopIndices[i];
          const MineRecord& r = m_mineSession->records().at(topIdx);
          const bool selected = (i == m_labMineTopSelection);

          Rectangle rowR{contentR.x + 2.0f, (float)ry - 2.0f, contentR.width - 6.0f, (float)rowH};
          if (selected)
            DrawRectangleRec(rowR, uiTh.accentHi);

          if (CheckCollisionPointRec(mouseUi, rowR) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            m_labMineTopSelection = i;
          }

          const float floodPct = static_cast<float>(r.seaFloodFrac * 100.0);
          const float pondPct = static_cast<float>(r.pondFrac * 100.0);

          if (m_mineTopPareto) {
            ui::Text(baseX, ry, 14,
                     TextFormat("#%d  %s  pr %d  score %.2f  pop %.0f  hap %.0f  flood %.1f%% pond %.1f%%", i + 1,
                                HexU64(r.seed).c_str(), r.paretoRank, r.score, r.stats.population, r.stats.happiness,
                                floodPct, pondPct),
                     selected ? uiTh.text : uiTh.textDim, false, true, 1);
          } else {
            ui::Text(baseX, ry, 14,
                     TextFormat("#%d  %s  score %.2f  pop %.0f  hap %.0f  flood %.1f%% pond %.1f%%", i + 1,
                                HexU64(r.seed).c_str(), r.score, r.stats.population, r.stats.happiness, floodPct, pondPct),
                     selected ? uiTh.text : uiTh.textDim, false, true, 1);
          }

          ry += rowH;
        }

        y += static_cast<int>(listR.height) + 10;

        // Actions.
        const int btnH = 18;
        const int btnGap = 6;
        const int btnW = (innerW - btnGap * 3) / 4;
        const int bx = innerX;

        const bool hasSelection = !m_labMineTopIndices.empty() && m_labMineTopSelection >= 0 &&
                                  m_labMineTopSelection < (int)m_labMineTopIndices.size();
        std::uint64_t selectedSeed = 0;
        if (hasSelection) {
          const int topIdx = m_labMineTopIndices[m_labMineTopSelection];
          selectedSeed = m_mineSession->records().at(topIdx).seed;
        }

        const Rectangle loadR{static_cast<float>(bx), static_cast<float>(y), static_cast<float>(btnW),
                              static_cast<float>(btnH)};
        const Rectangle csvR{static_cast<float>(bx + (btnW + btnGap)), static_cast<float>(y), static_cast<float>(btnW),
                             static_cast<float>(btnH)};
        const Rectangle dosR{static_cast<float>(bx + (btnW + btnGap) * 2), static_cast<float>(y),
                             static_cast<float>(btnW), static_cast<float>(btnH)};
        const Rectangle batchR{static_cast<float>(bx + (btnW + btnGap) * 3), static_cast<float>(y),
                               static_cast<float>(btnW), static_cast<float>(btnH)};

        if (ui::Button(9231, loadR, "Load", mouseUi, uiTime, hasSelection)) {
          m_pendingMineLoadSeed = true;
          m_pendingMineLoadSeedValue = selectedSeed;
          showToast(std::string("Loading seed ") + HexU64(selectedSeed), 2.0f);
        }

        if (ui::Button(9232, csvR, "CSV", mouseUi, uiTime, true)) {
          const std::string path = TextFormat("captures/mine_%s_%s.csv", MineObjectiveName(m_mineSession->config().objective),
                                              FileTimestamp().c_str());
          std::ofstream f(path);
          if (!f) {
            showToast(std::string("Mine CSV failed: ") + path, 3.0f);
          } else {
            WriteMineCsvHeader(f);
            for (const MineRecord& r : m_mineSession->records()) {
              WriteMineCsvRow(f, r);
            }
            showToast(std::string("Mine CSV: ") + path, 3.0f);
          }
        }

        if (ui::Button(9233, dosR, "Dossier", mouseUi, uiTime, hasSelection)) {
          const std::filesystem::path outDir = std::filesystem::path("captures") /
                                              (std::string("dossier_mined_") + HexU64(selectedSeed) + "_" +
                                               FileTimestamp());
          CityDossierConfig cfg = makeDossierCfg(outDir);
          queueDossierSeeds(std::move(cfg), {selectedSeed}, m_labMineBakeDays, "Queued dossier");
        }

        if (ui::Button(9234, batchR, "Batch", mouseUi, uiTime, true)) {
          const int n = std::clamp(m_labMineBatchExportN, 1, 50);
          const int count = std::min(n, (int)m_labMineTopIndices.size());
          if (count <= 0) {
            showToast("Batch: no top seeds yet", 2.0f);
          } else {
            std::vector<std::uint64_t> seeds;
            seeds.reserve(count);
            for (int i = 0; i < count; ++i) {
              const int topIdx = m_labMineTopIndices[i];
              seeds.push_back(m_mineSession->records().at(topIdx).seed);
            }

            const std::filesystem::path outDir =
                std::filesystem::path("captures") / (std::string("dossiers_mined_") + FileTimestamp());
            CityDossierConfig cfg = makeDossierCfg(outDir);
            queueDossierSeeds(std::move(cfg), std::move(seeds), m_labMineBakeDays, "Queued batch");
          }
        }

        y += 24;

        // Batch controls.
        ui::Text(innerX + 4, y + 2, 14, "Batch: top N", uiTh.textDim);
        ui::SliderInt(9235, Rectangle{static_cast<float>(innerX + 120), static_cast<float>(y), 80.0f, 18.0f},
                      m_labMineBatchExportN, 1, 50, mouseUi, uiTime);
        ui::Text(innerX + 210, y + 2, 14, "Bake days", uiTh.textDim);
        ui::SliderInt(9236, Rectangle{static_cast<float>(innerX + 300), static_cast<float>(y), 120.0f, 18.0f},
                      m_labMineBakeDays, 0, 365, mouseUi, uiTime);

        y += 24;
      }
    }

    return;
  }

  // ---------------------------------------------------------------------------
  // Dossier tab
  // ---------------------------------------------------------------------------
  if (m_headlessLabTab == 1) {
    const Rectangle cfgR{static_cast<float>(innerX), static_cast<float>(y), static_cast<float>(innerW), 210.0f};
    ui::DrawPanelInset(cfgR, uiTime, true);

    const int labelX = innerX + innerPad;
    const int labelW = 120;
    const int ctrlX = labelX + labelW;
    const int ctrlW = innerW - 2 * innerPad - labelW;

    int cy = y + innerPad;

    // Preset.
    {
      ui::Text(labelX, cy + 2, 14, "Preset", uiTh.textDim);
      const Rectangle btnR{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f};
      const char* name = (m_labDossierPreset == 1) ? "Fast" : "Full";
      if (ui::Button(9300, btnR, name, mouseUi, uiTime, true)) {
        m_labDossierPreset = (m_labDossierPreset == 1) ? 0 : 1;
      }
      cy += 22;
    }

    // Export scale.
    {
      ui::Text(labelX, cy + 2, 14, "Scale", uiTh.textDim);
      ui::SliderInt(9301, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                    m_labDossierExportScale, 1, 4, mouseUi, uiTime);
      cy += 22;
    }

    // Iso + 3D toggles.
    {
      ui::Text(labelX, cy + 2, 14, "Iso", uiTh.textDim);
      ui::Toggle(9302, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, m_labDossierIso,
                 mouseUi, uiTime);
      cy += 22;

      ui::Text(labelX, cy + 2, 14, "3D", uiTh.textDim);
      ui::Toggle(9303, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f}, m_labDossier3d,
                 mouseUi, uiTime);
      ui::Text(ctrlX + 128, cy + 2, 14, "(slow)", uiTh.textDim);
      cy += 22;
    }

    // Format.
    {
      ui::Text(labelX, cy + 2, 14, "Format", uiTh.textDim);
      const Rectangle btnR{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f};
      const char* fmtName = (m_labDossierFormat == 1) ? "PPM" : "PNG";
      if (ui::Button(9304, btnR, fmtName, mouseUi, uiTime, true)) {
        m_labDossierFormat = (m_labDossierFormat == 1) ? 0 : 1;
      }
      cy += 22;
    }

    // Export actions.
    {
      const int btnH = 18;
      const int btnGap = 6;
      const int btnW = (innerW - innerPad * 2 - btnGap) / 2;
      const int bx = innerX + innerPad;

      const Rectangle expR{static_cast<float>(bx), static_cast<float>(cy), static_cast<float>(btnW),
                           static_cast<float>(btnH)};
      const Rectangle clearR{static_cast<float>(bx + btnW + btnGap), static_cast<float>(cy), static_cast<float>(btnW),
                             static_cast<float>(btnH)};

      if (ui::Button(9305, expR, "Export current", mouseUi, uiTime, true)) {
        const std::filesystem::path outDir =
            std::filesystem::path("captures") /
            (std::string("dossier_seed") + HexU64(m_cfg.seed) + "_" + FileTimestamp());
        CityDossierConfig cfg = makeDossierCfg(outDir);
        queueDossierCurrentWorld(std::move(cfg), "Queued dossier");
      }

      if (ui::Button(9306, clearR, "Clear queue", mouseUi, uiTime, !m_dossierQueue.empty())) {
        const int n = (int)m_dossierQueue.size();
        m_dossierQueue.clear();
        showToast(TextFormat("Dossier queue cleared (%d)", n), 2.0f);
      }

      cy += 24;
    }

    y += static_cast<int>(cfgR.height) + 10;

    // Job status.
    {
      if (m_dossierJob) {
        std::string stage;
        {
          std::lock_guard<std::mutex> lk(m_dossierJob->mutex);
          stage = m_dossierJob->stage;
        }

        const int si = m_dossierJob->stepIndex.load(std::memory_order_relaxed);
        const int sc = m_dossierJob->stepCount.load(std::memory_order_relaxed);
        const int seedI = m_dossierJob->seedIndex.load(std::memory_order_relaxed);
        const int seedC = m_dossierJob->seedCount.load(std::memory_order_relaxed);

        if (seedC > 0)
          ui::Text(innerX + 4, y, 14, TextFormat("Running: seed %d/%d", seedI, seedC), uiTh.text);
        else
          ui::Text(innerX + 4, y, 14, "Running:", uiTh.text);

        y += 18;
        ui::Text(innerX + 4, y, 14, stage.empty() ? "(working...)" : stage, uiTh.textDim, false, true, 2);
        y += 18;

        if (sc > 0)
          ui::Text(innerX + 4, y, 14, TextFormat("Progress: %d / %d", si, sc), uiTh.textDim);
        else
          ui::Text(innerX + 4, y, 14, TextFormat("Progress: %d", si), uiTh.textDim);

        const Rectangle cancelR{static_cast<float>(innerX + innerW - 90), static_cast<float>(y - 2), 86.0f, 18.0f};
        if (ui::Button(9307, cancelR, "Cancel", mouseUi, uiTime, true)) {
          m_dossierJob->cancel.store(true, std::memory_order_relaxed);
          showToast("Dossier: cancel requested", 2.0f);
        }

        y += 22;
      } else {
        ui::Text(innerX + 4, y, 14, "Status: idle", uiTh.textDim);
        y += 18;

        if (!m_lastDossierMsg.empty()) {
          ui::Text(innerX + 4, y, 14, m_lastDossierMsg, m_lastDossierOk ? uiTh.text : uiTh.bad, false, true, 3);
          y += 18;
        }
      }

      if (!m_dossierQueue.empty()) {
        ui::Text(innerX + 4, y, 14, TextFormat("Queue: %d job(s)", (int)m_dossierQueue.size()), uiTh.textDim);
      }
    }

  }

  // ---------------------------------------------------------------------------
  // Mesh tab
  // ---------------------------------------------------------------------------
  if (m_headlessLabTab == 2) {
    const Rectangle cfgR{static_cast<float>(innerX), static_cast<float>(y), static_cast<float>(innerW), 280.0f};
    ui::DrawPanelInset(cfgR, uiTime, true);

    const int labelX = innerX + innerPad;
    const int labelW = 120;
    const int ctrlX = labelX + labelW;
    const int ctrlW = innerX + innerW - innerPad - ctrlX;

    int cy = y + innerPad;

    ui::Text(innerX + innerPad, cy, 14,
             TextFormat("Export mesh (seed %llu, day %d)", static_cast<unsigned long long>(m_cfg.seed),
                        m_world.stats().day),
             uiTh.text, /*bold=*/true, /*shadow=*/true, 1);
    cy += 20;

    // Export format (cycle).
    {
      ui::Text(labelX, cy + 2, 14, "Format", uiTh.textDim);
      const Rectangle btnR{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f};
      const char* fmtName =
          (m_labMeshFormat == 2) ? "GLB (binary glTF)" : (m_labMeshFormat == 1) ? "glTF (.gltf + .bin)" : "OBJ + MTL";
      if (ui::Button(9400, btnR, fmtName, mouseUi, uiTime, true)) {
        m_labMeshFormat = (m_labMeshFormat + 1) % 3;
      }
      cy += 22;
    }

    // Presets.
    {
      ui::Text(labelX, cy + 2, 14, "Preset", uiTh.textDim);
      const int gap = 4;
      const int bw = (ctrlW - gap * 2) / 3;
      const Rectangle p0{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(bw), 18.0f};
      const Rectangle p1{static_cast<float>(ctrlX + bw + gap), static_cast<float>(cy), static_cast<float>(bw), 18.0f};
      const Rectangle p2{static_cast<float>(ctrlX + (bw + gap) * 2), static_cast<float>(cy), static_cast<float>(bw), 18.0f};

      if (ui::Button(9401, p0, "Preview", mouseUi, uiTime, true)) {
        MeshExportConfig cfg;
        cfg.mergeTopSurfaces = true;
        cfg.heightQuantization = 0.25f;
        cfg.includeCliffs = false;
        cfg.includeBuildings = true;
        cfg.mergeBuildings = true;
        m_labMeshCfg = cfg;
        showToast("Mesh preset: Preview", 1.5f);
      }
      if (ui::Button(9402, p1, "Detail", mouseUi, uiTime, true)) {
        MeshExportConfig cfg;
        cfg.mergeTopSurfaces = false;
        cfg.heightQuantization = 0.0f;
        cfg.includeCliffs = true;
        cfg.includeBuildings = true;
        cfg.mergeBuildings = true;
        m_labMeshCfg = cfg;
        showToast("Mesh preset: Detail", 1.5f);
      }
      if (ui::Button(9403, p2, "Legacy", mouseUi, uiTime, true)) {
        MeshExportConfig cfg;
        ApplyLegacyMeshExportDefaults(cfg);
        m_labMeshCfg = cfg;
        showToast("Mesh preset: Legacy", 1.5f);
      }

      cy += 22;
    }

    // Height scaling.
    {
      ui::Text(labelX, cy + 2, 14, "Height", uiTh.textDim);
      ui::SliderFloat(9404, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                      m_labMeshCfg.heightScale, 1.0f, 32.0f, 0.5f, mouseUi, uiTime);
      cy += 22;
    }

    // Height quantization.
    {
      ui::Text(labelX, cy + 2, 14, "Quantize", uiTh.textDim);
      ui::SliderFloat(9405, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), static_cast<float>(ctrlW), 18.0f},
                      m_labMeshCfg.heightQuantization, 0.0f, 1.0f, 0.05f, mouseUi, uiTime);
      cy += 22;
    }

    // Merge tops.
    {
      ui::Text(labelX, cy + 2, 14, "Merge tops", uiTh.textDim);
      ui::Toggle(9406, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f},
                 m_labMeshCfg.mergeTopSurfaces, mouseUi, uiTime);
      cy += 22;
    }

    // Cliffs.
    {
      ui::Text(labelX, cy + 2, 14, "Cliffs", uiTh.textDim);
      ui::Toggle(9407, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f},
                 m_labMeshCfg.includeCliffs, mouseUi, uiTime);
      ui::SliderFloat(9408,
                      Rectangle{static_cast<float>(ctrlX + 128), static_cast<float>(cy),
                                static_cast<float>(std::max(0, ctrlW - 128)), 18.0f},
                      m_labMeshCfg.cliffThreshold, 0.0f, 0.20f, 0.01f, mouseUi, uiTime, m_labMeshCfg.includeCliffs);
      cy += 22;
    }

    // Buildings.
    {
      ui::Text(labelX, cy + 2, 14, "Buildings", uiTh.textDim);
      ui::Toggle(9409, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f},
                 m_labMeshCfg.includeBuildings, mouseUi, uiTime);
      cy += 22;

      ui::Text(labelX, cy + 2, 14, "Merge bldgs", uiTh.textDim);
      ui::Toggle(9410, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f},
                 m_labMeshCfg.mergeBuildings, mouseUi, uiTime, m_labMeshCfg.includeBuildings);
      cy += 22;
    }

    // Helper: set crop bounds from the current camera viewport.
    auto setCropToViewport = [&]() {
      const int screenW = GetScreenWidth();
      const int screenH = GetScreenHeight();
      const float tileW = static_cast<float>(m_cfg.tileWidth);
      const float tileH = static_cast<float>(m_cfg.tileHeight);

      Vector2 corners[4] = {
          Vector2{0.0f, 0.0f},
          Vector2{static_cast<float>(screenW), 0.0f},
          Vector2{0.0f, static_cast<float>(screenH)},
          Vector2{static_cast<float>(screenW), static_cast<float>(screenH)},
      };

      int minX = m_world.width();
      int minY = m_world.height();
      int maxX = -1;
      int maxY = -1;
      bool any = false;

      for (const Vector2& sc : corners) {
        const Vector2 wpos = GetScreenToWorld2D(sc, m_camera);
        const std::optional<Point> t = WorldToTileElevated(wpos, m_world, tileW, tileH, m_elev);
        if (!t) continue;
        any = true;
        minX = std::min(minX, t->x);
        minY = std::min(minY, t->y);
        maxX = std::max(maxX, t->x);
        maxY = std::max(maxY, t->y);
      }

      if (!any) {
        showToast("Mesh crop: viewport not over map", 2.0f);
        return;
      }

      minX = std::clamp(minX - 1, 0, m_world.width() - 1);
      minY = std::clamp(minY - 1, 0, m_world.height() - 1);
      maxX = std::clamp(maxX + 1, 0, m_world.width() - 1);
      maxY = std::clamp(maxY + 1, 0, m_world.height() - 1);

      m_labMeshCfg.hasCrop = true;
      m_labMeshCfg.cropX = minX;
      m_labMeshCfg.cropY = minY;
      m_labMeshCfg.cropW = (maxX - minX + 1);
      m_labMeshCfg.cropH = (maxY - minY + 1);
      m_labMeshCfg.originAtCrop = true;

      showToast(TextFormat("Mesh crop set: x=%d y=%d w=%d h=%d", m_labMeshCfg.cropX, m_labMeshCfg.cropY,
                           m_labMeshCfg.cropW, m_labMeshCfg.cropH),
                2.0f);
    };

    // Crop.
    {
      ui::Text(labelX, cy + 2, 14, "Crop", uiTh.textDim);
      if (ui::Toggle(9411, Rectangle{static_cast<float>(ctrlX), static_cast<float>(cy), 120.0f, 18.0f},
                     m_labMeshCfg.hasCrop, mouseUi, uiTime)) {
        if (m_labMeshCfg.hasCrop) {
          // Default to full map crop so the config is immediately valid.
          m_labMeshCfg.cropX = 0;
          m_labMeshCfg.cropY = 0;
          m_labMeshCfg.cropW = m_world.width();
          m_labMeshCfg.cropH = m_world.height();
          m_labMeshCfg.originAtCrop = false;
        }
      }

      const Rectangle viewR{static_cast<float>(ctrlX + 128), static_cast<float>(cy),
                            static_cast<float>(std::max(0, ctrlW - 128)), 18.0f};
      if (ui::Button(9412, viewR, "Use viewport", mouseUi, uiTime, true)) {
        setCropToViewport();
      }
      cy += 22;

      if (m_labMeshCfg.hasCrop) {
        ui::Text(ctrlX + 4, cy, 14,
                 TextFormat("x=%d y=%d w=%d h=%d%s", m_labMeshCfg.cropX, m_labMeshCfg.cropY, m_labMeshCfg.cropW,
                            m_labMeshCfg.cropH, m_labMeshCfg.originAtCrop ? " (origin@crop)" : ""),
                 uiTh.textDim);
        cy += 18;
      }
    }

    // Export actions.
    {
      const int btnH = 18;
      const int btnGap = 6;
      const int btnW = (innerW - innerPad * 2 - btnGap) / 2;
      const int bx = innerX + innerPad;
      const bool busy = (m_meshJob != nullptr);

      const Rectangle expR{static_cast<float>(bx), static_cast<float>(cy), static_cast<float>(btnW),
                           static_cast<float>(btnH)};
      const Rectangle resetR{static_cast<float>(bx + btnW + btnGap), static_cast<float>(cy), static_cast<float>(btnW),
                             static_cast<float>(btnH)};

      if (ui::Button(9450, expR, busy ? "Exporting..." : "Export current", mouseUi, uiTime, !busy)) {
        namespace fs = std::filesystem;
        fs::create_directories("captures");

        const int day = m_world.stats().day;
        const std::string base = TextFormat("captures/mesh_seed%llu_day%d_%s", static_cast<unsigned long long>(m_cfg.seed),
                                            day, FileTimestamp().c_str());

        std::string outPath;
        std::string mtlPath;
        MeshJob::Kind kind = MeshJob::Kind::Obj;
        if (m_labMeshFormat == 1) {
          kind = MeshJob::Kind::Gltf;
          outPath = base + ".gltf";
        } else if (m_labMeshFormat == 2) {
          kind = MeshJob::Kind::Glb;
          outPath = base + ".glb";
        } else {
          kind = MeshJob::Kind::Obj;
          outPath = base + ".obj";
          mtlPath = base + ".mtl";
        }

        MeshExportConfig cfg = m_labMeshCfg;
        cfg.objectName = std::string("city_seed") + HexU64(m_cfg.seed);
        if (!mtlPath.empty()) {
          try {
            cfg.mtlFileName = fs::path(mtlPath).filename().string();
          } catch (...) {
          }
        }

        auto job = std::make_unique<MeshJob>();
        job->kind = kind;
        job->cfg = cfg;
        job->world = m_world; // snapshot
        job->outPath = outPath;
        job->mtlPath = mtlPath;

        MeshJob* jp = job.get();
        jp->worker = std::thread([jp]() {
          MeshExportStats stats{};
          std::string err;
          bool ok = false;

          try {
            if (jp->kind == MeshJob::Kind::Obj) {
              ok = ExportWorldObjMtl(jp->outPath, jp->mtlPath, jp->world, jp->cfg, &stats, &err);
            } else if (jp->kind == MeshJob::Kind::Gltf) {
              ok = ExportWorldGltf(jp->outPath, jp->world, jp->cfg, &stats, &err);
            } else {
              ok = ExportWorldGlb(jp->outPath, jp->world, jp->cfg, &stats, &err);
            }
          } catch (const std::exception& e) {
            ok = false;
            err = e.what();
          } catch (...) {
            ok = false;
            err = "Unknown error";
          }

          {
            std::lock_guard<std::mutex> lock(jp->mutex);
            jp->ok = ok;
            jp->stats = stats;
            jp->err = err;
          }
          jp->done.store(true, std::memory_order_relaxed);
        });

        m_meshJob = std::move(job);

        std::string fileLabel = outPath;
        try {
          fileLabel = std::filesystem::path(outPath).filename().string();
        } catch (...) {
        }
        showToast(std::string("Mesh export started: ") + fileLabel, 2.0f);
        m_console.print(std::string("mesh: start ") + outPath);
      }

      if (ui::Button(9451, resetR, "Reset", mouseUi, uiTime, !busy)) {
        m_labMeshCfg = MeshExportConfig{};
        m_labMeshFormat = 0;
        showToast("Mesh export settings reset", 1.5f);
      }

      cy += 24;
    }

    y += static_cast<int>(cfgR.height) + 10;

    // Job status.
    {
      if (m_meshJob) {
        const char* kindName =
            (m_meshJob->kind == MeshJob::Kind::Glb) ? "GLB" : (m_meshJob->kind == MeshJob::Kind::Gltf) ? "glTF" : "OBJ";
        ui::Text(innerX + 4, y, 14, TextFormat("Running: %s", kindName), uiTh.text);
        y += 18;

        std::string outLabel = m_meshJob->outPath;
        try {
          outLabel = std::filesystem::path(m_meshJob->outPath).filename().string();
        } catch (...) {
        }
        ui::Text(innerX + 4, y, 14, outLabel, uiTh.textDim, false, true, 2);
        y += 18;
      } else {
        ui::Text(innerX + 4, y, 14, "Status: idle", uiTh.textDim);
        y += 18;

        if (!m_lastMeshMsg.empty()) {
          ui::Text(innerX + 4, y, 14, m_lastMeshMsg, m_lastMeshOk ? uiTh.text : uiTh.bad, false, true, 3);
          y += 18;
        }
      }
    }
  }
}

} // namespace isocity
