#include "isocity/SaveLoad.hpp"

#include "isocity/Checksum.hpp"
#include "isocity/Compression.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/Sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <limits>
#include <sstream>
#include <type_traits>
#include <vector>

namespace isocity {

namespace {

constexpr char kMagic[8] = {'I', 'S', 'O', 'C', 'I', 'T', 'Y', '\0'};
constexpr std::uint32_t kVersionV1 = 1; // full tiles
constexpr std::uint32_t kVersionV2 = 2; // seed + ProcGenConfig + tile deltas
constexpr std::uint32_t kVersionV3 = 3; // v2 + CRC32 checksum
constexpr std::uint32_t kVersionV4 = 4; // v3 + varint/delta encoding for tile diffs
constexpr std::uint32_t kVersionV5 = 5; // v4 + height deltas (terraforming)
constexpr std::uint32_t kVersionV6 = 6; // v5 + SimConfig (policy/economy settings)
constexpr std::uint32_t kVersionV7 = 7; // v6 + districts + district policy multipliers
constexpr std::uint32_t kVersionV8 = 8; // v7 + compressed delta payload
constexpr std::uint32_t kVersionV9 = 9; // v8 + ProcGen erosion config
constexpr std::uint32_t kVersionV10 = 10; // v9 + ProcGen terrain preset config
constexpr std::uint32_t kVersionV11 = 11; // v10 + ProcGen road hierarchy config
constexpr std::uint32_t kVersionV12 = 12; // v11 + ProcGen districting mode config
constexpr std::uint32_t kVersionCurrent = kVersionV12;

// --- CRC32 ---
// Used by v3+ saves to detect corruption/truncation.
//
// Notes:
// - We compute CRC32 over the entire file contents *excluding* the final CRC field.
// - The CRC32 is stored as a uint32_t appended to the end of the file.


struct Crc32OStreamWriter {
  explicit Crc32OStreamWriter(std::ostream& out) : os(out) {}

  std::ostream& os;
  std::uint32_t crc = 0xFFFFFFFFu;

  bool writeBytes(const void* data, std::size_t size)
  {
    os.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    if (!os.good()) return false;
    crc = Crc32Update(crc, reinterpret_cast<const std::uint8_t*>(data), size);
    return true;
  }

  template <typename T>
  bool write(const T& v)
  {
    static_assert(std::is_trivially_copyable_v<T>);
    return writeBytes(&v, sizeof(T));
  }

  std::uint32_t finalize() const { return crc ^ 0xFFFFFFFFu; }
};

template <typename T>
bool Write(std::ostream& os, const T& v)
{
  static_assert(std::is_trivially_copyable_v<T>);
  os.write(reinterpret_cast<const char*>(&v), static_cast<std::streamsize>(sizeof(T)));
  return os.good();
}

template <typename T>
bool Read(std::istream& is, T& v)
{
  static_assert(std::is_trivially_copyable_v<T>);
  is.read(reinterpret_cast<char*>(&v), static_cast<std::streamsize>(sizeof(T)));
  return is.good();
}

[[maybe_unused]] bool WriteBytes(std::ostream& os, const void* data, std::size_t size)
{
  os.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
  return os.good();
}

bool ReadBytes(std::istream& is, void* data, std::size_t size)
{
  is.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(size));
  return is.good();
}

bool VerifyCrc32File(const std::string& path, bool& outOk, std::string& outError)
{
  outError.clear();
  outOk = true;

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Unable to open file for CRC check: " + path;
    return false;
  }

  f.seekg(0, std::ios::end);
  const std::streamoff sizeOff = f.tellg();
  if (sizeOff < static_cast<std::streamoff>(sizeof(std::uint32_t))) {
    outError = "Save file too small for CRC32";
    return false;
  }
  const std::uint64_t size = static_cast<std::uint64_t>(sizeOff);
  const std::uint64_t payloadSize = size - sizeof(std::uint32_t);
  f.seekg(0, std::ios::beg);

  std::uint32_t crc = 0xFFFFFFFFu;
  std::vector<char> buf(64 * 1024);
  std::uint64_t remaining = payloadSize;

  while (remaining > 0) {
    const std::size_t chunk = static_cast<std::size_t>(
        std::min<std::uint64_t>(remaining, static_cast<std::uint64_t>(buf.size())));
    f.read(buf.data(), static_cast<std::streamsize>(chunk));
    if (!f.good()) {
      outError = "Read failed during CRC32";
      return false;
    }
    crc = Crc32Update(crc, reinterpret_cast<const std::uint8_t*>(buf.data()), chunk);
    remaining -= chunk;
  }

  std::uint32_t expected = 0;
  if (!Read(f, expected)) {
    outError = "Read failed (CRC32 field)";
    return false;
  }

  const std::uint32_t computed = crc ^ 0xFFFFFFFFu;
  outOk = (computed == expected);
  return true;
}

// --- Varint (unsigned LEB128) ---
// Used by v4 to compress monotonically-increasing tile diff indices (delta-encoded)
// and small integers like occupants.
template <typename Writer>
bool WriteVarU32(Writer& w, std::uint32_t v)
{
  std::uint8_t buf[5];
  std::size_t n = 0;

  while (v >= 0x80u) {
    buf[n++] = static_cast<std::uint8_t>((v & 0x7Fu) | 0x80u);
    v >>= 7;
  }
  buf[n++] = static_cast<std::uint8_t>(v & 0x7Fu);

  return w.writeBytes(buf, n);
}

struct VecWriter {
  explicit VecWriter(std::vector<std::uint8_t>& dst) : out(dst) {}

  bool writeBytes(const void* data, std::size_t size)
  {
    const auto* p = reinterpret_cast<const std::uint8_t*>(data);
    out.insert(out.end(), p, p + size);
    return true;
  }

  template <typename T>
  bool writePOD(const T& value)
  {
    static_assert(std::is_trivially_copyable_v<T>, "VecWriter can only write POD/trivially-copyable types");
    return writeBytes(&value, sizeof(value));
  }

  std::vector<std::uint8_t>& out;
};

bool ReadVarU32(std::istream& is, std::uint32_t& out)
{
  out = 0;
  std::uint32_t shift = 0;

  for (int i = 0; i < 5; ++i) {
    std::uint8_t byte = 0;
    if (!Read(is, byte)) return false;

    out |= static_cast<std::uint32_t>(byte & 0x7Fu) << shift;

    if ((byte & 0x80u) == 0) {
      return true;
    }

    shift += 7;
  }

  // Too many bytes for a 32-bit varint (corrupt file).
  return false;
}

// --- Height quantization ---
// We store Tile::height as a quantized unsigned 16-bit value in [0, 65535],
// mapping the game's conceptual [0,1] height range.
//
// This keeps saves deterministic and compact while preserving more than enough
// precision for terraforming.
inline std::uint16_t QuantizeHeight(float h)
{
  const float hc = std::clamp(h, 0.0f, 1.0f);
  const float scaled = hc * 65535.0f;
  const int q = static_cast<int>(std::lround(scaled));
  return static_cast<std::uint16_t>(std::clamp(q, 0, 65535));
}

inline float DequantizeHeight(std::uint16_t q)
{
  return static_cast<float>(q) / 65535.0f;
}

inline Terrain TerrainFromHeight(float h, const ProcGenConfig& cfg)
{
  const float wl = std::clamp(cfg.waterLevel, 0.0f, 1.0f);
  const float sl = std::clamp(cfg.sandLevel, 0.0f, 1.0f);
  if (h < wl) return Terrain::Water;
  if (h < std::max(wl, sl)) return Terrain::Sand;
  return Terrain::Grass;
}

struct StatsBin {
  std::int32_t day = 0;

  std::int32_t population = 0;
  std::int32_t housingCapacity = 0;

  std::int32_t jobsCapacity = 0;
  std::int32_t employed = 0;

  float happiness = 0.5f;

  std::int32_t money = 0;

  std::int32_t roads = 0;
  std::int32_t parks = 0;
};

StatsBin ToBin(const Stats& s)
{
  StatsBin b;
  b.day = static_cast<std::int32_t>(s.day);
  b.population = static_cast<std::int32_t>(s.population);
  b.housingCapacity = static_cast<std::int32_t>(s.housingCapacity);
  b.jobsCapacity = static_cast<std::int32_t>(s.jobsCapacity);
  b.employed = static_cast<std::int32_t>(s.employed);
  b.happiness = s.happiness;
  b.money = static_cast<std::int32_t>(s.money);
  b.roads = static_cast<std::int32_t>(s.roads);
  b.parks = static_cast<std::int32_t>(s.parks);
  return b;
}

void FromBin(Stats& s, const StatsBin& b)
{
  s.day = static_cast<int>(b.day);
  s.population = static_cast<int>(b.population);
  s.housingCapacity = static_cast<int>(b.housingCapacity);
  s.jobsCapacity = static_cast<int>(b.jobsCapacity);
  s.employed = static_cast<int>(b.employed);
  s.happiness = b.happiness;
  s.money = static_cast<int>(b.money);
  s.roads = static_cast<int>(b.roads);
  s.parks = static_cast<int>(b.parks);
}

struct ProcGenConfigBin {
  float terrainScale = 0.08f;
  float waterLevel = 0.35f;
  float sandLevel = 0.42f;
  std::int32_t hubs = 4;
  std::int32_t extraConnections = 2;
  float zoneChance = 0.22f;
  float parkChance = 0.06f;
};

// v10 extends ProcGenConfigBin with macro terrain preset information.
struct ProcGenConfigBinV10 {
  float terrainScale = 0.08f;
  float waterLevel = 0.35f;
  float sandLevel = 0.42f;
  std::int32_t hubs = 4;
  std::int32_t extraConnections = 2;
  float zoneChance = 0.22f;
  float parkChance = 0.06f;

  std::uint8_t terrainPreset = 0; // ProcGenTerrainPreset
  std::uint8_t _pad0 = 0;
  std::uint8_t _pad1 = 0;
  std::uint8_t _pad2 = 0;

  float terrainPresetStrength = 1.0f;
};

// v11 extends ProcGenConfigBinV10 with road hierarchy settings.
struct ProcGenConfigBinV11 {
  float terrainScale = 0.08f;
  float waterLevel = 0.35f;
  float sandLevel = 0.42f;
  std::int32_t hubs = 4;
  std::int32_t extraConnections = 2;
  float zoneChance = 0.22f;
  float parkChance = 0.06f;

  std::uint8_t terrainPreset = 0;        // ProcGenTerrainPreset
  std::uint8_t roadHierarchyEnabled = 0; // bool
  std::uint8_t _pad0 = 0;
  std::uint8_t _pad1 = 0;

  float terrainPresetStrength = 1.0f;
  float roadHierarchyStrength = 0.0f;
};

// v12 extends ProcGenConfigBinV11 with districting mode settings.
struct ProcGenConfigBinV12 {
  float terrainScale = 0.08f;
  float waterLevel = 0.35f;
  float sandLevel = 0.42f;
  std::int32_t hubs = 4;
  std::int32_t extraConnections = 2;
  float zoneChance = 0.22f;
  float parkChance = 0.06f;

  std::uint8_t terrainPreset = 0;        // ProcGenTerrainPreset
  std::uint8_t roadHierarchyEnabled = 0; // bool
  std::uint8_t districtingMode = 0;      // ProcGenDistrictingMode
  std::uint8_t _pad0 = 0;

  float terrainPresetStrength = 1.0f;
  float roadHierarchyStrength = 0.0f;
};

ProcGenConfigBin ToBin(const ProcGenConfig& cfg)
{
  ProcGenConfigBin b;
  b.terrainScale = cfg.terrainScale;
  b.waterLevel = cfg.waterLevel;
  b.sandLevel = cfg.sandLevel;
  b.hubs = static_cast<std::int32_t>(cfg.hubs);
  b.extraConnections = static_cast<std::int32_t>(cfg.extraConnections);
  b.zoneChance = cfg.zoneChance;
  b.parkChance = cfg.parkChance;
  return b;
}

ProcGenConfigBinV10 ToBinV10(const ProcGenConfig& cfg)
{
  ProcGenConfigBinV10 b;
  b.terrainScale = cfg.terrainScale;
  b.waterLevel = cfg.waterLevel;
  b.sandLevel = cfg.sandLevel;
  b.hubs = static_cast<std::int32_t>(cfg.hubs);
  b.extraConnections = static_cast<std::int32_t>(cfg.extraConnections);
  b.zoneChance = cfg.zoneChance;
  b.parkChance = cfg.parkChance;
  b.terrainPreset = static_cast<std::uint8_t>(cfg.terrainPreset);
  b.terrainPresetStrength = cfg.terrainPresetStrength;
  return b;
}

ProcGenConfigBinV11 ToBinV11(const ProcGenConfig& cfg)
{
  ProcGenConfigBinV11 b;
  b.terrainScale = cfg.terrainScale;
  b.waterLevel = cfg.waterLevel;
  b.sandLevel = cfg.sandLevel;
  b.hubs = static_cast<std::int32_t>(cfg.hubs);
  b.extraConnections = static_cast<std::int32_t>(cfg.extraConnections);
  b.zoneChance = cfg.zoneChance;
  b.parkChance = cfg.parkChance;

  b.terrainPreset = static_cast<std::uint8_t>(cfg.terrainPreset);
  b.roadHierarchyEnabled = static_cast<std::uint8_t>(cfg.roadHierarchyEnabled ? 1u : 0u);
  b.terrainPresetStrength = cfg.terrainPresetStrength;
  b.roadHierarchyStrength = cfg.roadHierarchyStrength;
  return b;
}

ProcGenConfigBinV12 ToBinV12(const ProcGenConfig& cfg)
{
  ProcGenConfigBinV12 b;
  b.terrainScale = cfg.terrainScale;
  b.waterLevel = cfg.waterLevel;
  b.sandLevel = cfg.sandLevel;
  b.hubs = static_cast<std::int32_t>(cfg.hubs);
  b.extraConnections = static_cast<std::int32_t>(cfg.extraConnections);
  b.zoneChance = cfg.zoneChance;
  b.parkChance = cfg.parkChance;

  b.terrainPreset = static_cast<std::uint8_t>(cfg.terrainPreset);
  b.roadHierarchyEnabled = static_cast<std::uint8_t>(cfg.roadHierarchyEnabled ? 1u : 0u);
  b.districtingMode = static_cast<std::uint8_t>(cfg.districtingMode);
  b.terrainPresetStrength = cfg.terrainPresetStrength;
  b.roadHierarchyStrength = cfg.roadHierarchyStrength;
  return b;
}

void FromBin(ProcGenConfig& cfg, const ProcGenConfigBin& b)
{
  cfg.terrainScale = b.terrainScale;
  cfg.waterLevel = b.waterLevel;
  cfg.sandLevel = b.sandLevel;
  cfg.hubs = static_cast<int>(b.hubs);
  cfg.extraConnections = static_cast<int>(b.extraConnections);
  cfg.zoneChance = b.zoneChance;
  cfg.parkChance = b.parkChance;

  // v9 and older: no terrain preset or road hierarchy settings were stored.
  cfg.terrainPreset = ProcGenTerrainPreset::Classic;
  cfg.terrainPresetStrength = 1.0f;
  cfg.roadHierarchyEnabled = false;
  cfg.roadHierarchyStrength = 0.0f;

  // v11 and older did not persist districting mode; default to legacy Voronoi to preserve determinism.
  cfg.districtingMode = ProcGenDistrictingMode::Voronoi;

  // Older save versions did not persist erosion settings. Default to disabled and let newer versions override.
  cfg.erosion = ErosionConfig{};
  cfg.erosion.enabled = false;
}

void FromBinV10(ProcGenConfig& cfg, const ProcGenConfigBinV10& b)
{
  cfg.terrainScale = b.terrainScale;
  cfg.waterLevel = b.waterLevel;
  cfg.sandLevel = b.sandLevel;
  cfg.hubs = static_cast<int>(b.hubs);
  cfg.extraConnections = static_cast<int>(b.extraConnections);
  cfg.zoneChance = b.zoneChance;
  cfg.parkChance = b.parkChance;

  // Defensive enum validation (avoid UB on corrupted saves).
  std::uint8_t presetU8 = b.terrainPreset;
  if (presetU8 > static_cast<std::uint8_t>(ProcGenTerrainPreset::MountainRing)) {
    presetU8 = static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
  }
  cfg.terrainPreset = static_cast<ProcGenTerrainPreset>(presetU8);
  cfg.terrainPresetStrength = std::clamp(b.terrainPresetStrength, 0.0f, 5.0f);

  // v10 did not include the road hierarchy pass; default it to disabled to preserve determinism.
  cfg.roadHierarchyEnabled = false;
  cfg.roadHierarchyStrength = 0.0f;

  // v11 and older did not persist districting mode; default to legacy Voronoi to preserve determinism.
  cfg.districtingMode = ProcGenDistrictingMode::Voronoi;

  // Erosion settings are persisted separately starting in v9; loader will override.
  cfg.erosion = ErosionConfig{};
}

void FromBinV11(ProcGenConfig& cfg, const ProcGenConfigBinV11& b)
{
  cfg.terrainScale = b.terrainScale;
  cfg.waterLevel = b.waterLevel;
  cfg.sandLevel = b.sandLevel;
  cfg.hubs = static_cast<int>(b.hubs);
  cfg.extraConnections = static_cast<int>(b.extraConnections);
  cfg.zoneChance = b.zoneChance;
  cfg.parkChance = b.parkChance;

  std::uint8_t presetU8 = b.terrainPreset;
  if (presetU8 > static_cast<std::uint8_t>(ProcGenTerrainPreset::MountainRing)) {
    presetU8 = static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
  }
  cfg.terrainPreset = static_cast<ProcGenTerrainPreset>(presetU8);
  cfg.terrainPresetStrength = std::clamp(b.terrainPresetStrength, 0.0f, 5.0f);

  cfg.roadHierarchyEnabled = (b.roadHierarchyEnabled != 0);
  cfg.roadHierarchyStrength = std::clamp(b.roadHierarchyStrength, 0.0f, 3.0f);

  // v11 did not persist districting mode; default to legacy Voronoi to preserve determinism.
  cfg.districtingMode = ProcGenDistrictingMode::Voronoi;

  // Erosion settings are persisted separately starting in v9; loader will override.
  cfg.erosion = ErosionConfig{};
}

void FromBinV12(ProcGenConfig& cfg, const ProcGenConfigBinV12& b)
{
  cfg.terrainScale = b.terrainScale;
  cfg.waterLevel = b.waterLevel;
  cfg.sandLevel = b.sandLevel;
  cfg.hubs = static_cast<int>(b.hubs);
  cfg.extraConnections = static_cast<int>(b.extraConnections);
  cfg.zoneChance = b.zoneChance;
  cfg.parkChance = b.parkChance;

  std::uint8_t presetU8 = b.terrainPreset;
  if (presetU8 > static_cast<std::uint8_t>(ProcGenTerrainPreset::MountainRing)) {
    presetU8 = static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
  }
  cfg.terrainPreset = static_cast<ProcGenTerrainPreset>(presetU8);
  cfg.terrainPresetStrength = std::clamp(b.terrainPresetStrength, 0.0f, 5.0f);

  cfg.roadHierarchyEnabled = (b.roadHierarchyEnabled != 0);
  cfg.roadHierarchyStrength = std::clamp(b.roadHierarchyStrength, 0.0f, 3.0f);

  std::uint8_t modeU8 = b.districtingMode;
  if (modeU8 > static_cast<std::uint8_t>(ProcGenDistrictingMode::BlockGraph)) {
    modeU8 = static_cast<std::uint8_t>(ProcGenDistrictingMode::Voronoi);
  }
  cfg.districtingMode = static_cast<ProcGenDistrictingMode>(modeU8);

  // Erosion settings are persisted separately starting in v9; loader will override.
  cfg.erosion = ErosionConfig{};
}


struct ErosionConfigBin {
  std::uint8_t enabled = 1;
  std::uint8_t riversEnabled = 1;
  std::uint8_t _pad0 = 0;
  std::uint8_t _pad1 = 0;

  std::int32_t thermalIterations = 20;
  float thermalTalus = 0.02f;
  float thermalRate = 0.50f;

  std::int32_t riverMinAccum = 0;
  float riverCarve = 0.055f;
  float riverCarvePower = 0.60f;

  std::int32_t smoothIterations = 1;
  float smoothRate = 0.25f;

  std::int32_t quantizeScale = 4096;
};

ErosionConfigBin ToBin(const ErosionConfig& cfg)
{
  ErosionConfigBin b;
  b.enabled = static_cast<std::uint8_t>(cfg.enabled ? 1 : 0);
  b.riversEnabled = static_cast<std::uint8_t>(cfg.riversEnabled ? 1 : 0);

  b.thermalIterations = static_cast<std::int32_t>(cfg.thermalIterations);
  b.thermalTalus = cfg.thermalTalus;
  b.thermalRate = cfg.thermalRate;

  b.riverMinAccum = static_cast<std::int32_t>(cfg.riverMinAccum);
  b.riverCarve = cfg.riverCarve;
  b.riverCarvePower = cfg.riverCarvePower;

  b.smoothIterations = static_cast<std::int32_t>(cfg.smoothIterations);
  b.smoothRate = cfg.smoothRate;

  b.quantizeScale = static_cast<std::int32_t>(cfg.quantizeScale);
  return b;
}

void FromBin(ErosionConfig& cfg, const ErosionConfigBin& b)
{
  cfg.enabled = (b.enabled != 0);
  cfg.riversEnabled = (b.riversEnabled != 0);

  cfg.thermalIterations = static_cast<int>(b.thermalIterations);
  cfg.thermalTalus = b.thermalTalus;
  cfg.thermalRate = b.thermalRate;

  cfg.riverMinAccum = static_cast<int>(b.riverMinAccum);
  cfg.riverCarve = b.riverCarve;
  cfg.riverCarvePower = b.riverCarvePower;

  cfg.smoothIterations = static_cast<int>(b.smoothIterations);
  cfg.smoothRate = b.smoothRate;

  cfg.quantizeScale = static_cast<int>(b.quantizeScale);
}

struct SimConfigBin {
  float tickSeconds = 0.5f;

  std::int32_t parkInfluenceRadius = 6;
  std::uint8_t requireOutsideConnection = 1;

  std::int32_t taxResidential = 1;
  std::int32_t taxCommercial = 2;
  std::int32_t taxIndustrial = 2;

  std::int32_t maintenanceRoad = 1;
  std::int32_t maintenancePark = 1;

  float taxHappinessPerCapita = 0.02f;

  float residentialDesirabilityWeight = 0.70f;
  float commercialDesirabilityWeight = 0.80f;
  float industrialDesirabilityWeight = 0.80f;
};

struct DistrictPolicyBin {
  float taxResidentialMult = 1.0f;
  float taxCommercialMult = 1.0f;
  float taxIndustrialMult = 1.0f;

  float roadMaintenanceMult = 1.0f;
  float parkMaintenanceMult = 1.0f;
};

DistrictPolicyBin ToBin(const DistrictPolicy& p)
{
  DistrictPolicyBin b;
  b.taxResidentialMult = p.taxResidentialMult;
  b.taxCommercialMult = p.taxCommercialMult;
  b.taxIndustrialMult = p.taxIndustrialMult;
  b.roadMaintenanceMult = p.roadMaintenanceMult;
  b.parkMaintenanceMult = p.parkMaintenanceMult;
  return b;
}

void FromBin(DistrictPolicy& p, const DistrictPolicyBin& b)
{
  auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };
  p.taxResidentialMult = clampF(b.taxResidentialMult, 0.0f, 10.0f);
  p.taxCommercialMult = clampF(b.taxCommercialMult, 0.0f, 10.0f);
  p.taxIndustrialMult = clampF(b.taxIndustrialMult, 0.0f, 10.0f);
  p.roadMaintenanceMult = clampF(b.roadMaintenanceMult, 0.0f, 10.0f);
  p.parkMaintenanceMult = clampF(b.parkMaintenanceMult, 0.0f, 10.0f);
}

SimConfigBin ToBin(const SimConfig& cfg)
{
  SimConfigBin b;
  b.tickSeconds = cfg.tickSeconds;

  b.parkInfluenceRadius = static_cast<std::int32_t>(cfg.parkInfluenceRadius);
  b.requireOutsideConnection = cfg.requireOutsideConnection ? 1u : 0u;

  b.taxResidential = static_cast<std::int32_t>(cfg.taxResidential);
  b.taxCommercial = static_cast<std::int32_t>(cfg.taxCommercial);
  b.taxIndustrial = static_cast<std::int32_t>(cfg.taxIndustrial);

  b.maintenanceRoad = static_cast<std::int32_t>(cfg.maintenanceRoad);
  b.maintenancePark = static_cast<std::int32_t>(cfg.maintenancePark);

  b.taxHappinessPerCapita = cfg.taxHappinessPerCapita;

  b.residentialDesirabilityWeight = cfg.residentialDesirabilityWeight;
  b.commercialDesirabilityWeight = cfg.commercialDesirabilityWeight;
  b.industrialDesirabilityWeight = cfg.industrialDesirabilityWeight;
  return b;
}

void FromBin(SimConfig& cfg, const SimConfigBin& b)
{
  auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };
  auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

  // v7 adds district-policy multipliers; v6 and older saves should reset them.
  cfg.districtPoliciesEnabled = false;
  cfg.districtPolicies = {};

  cfg.tickSeconds = clampF(b.tickSeconds, 0.01f, 60.0f);

  cfg.parkInfluenceRadius = clampI(static_cast<int>(b.parkInfluenceRadius), 0, 64);
  cfg.requireOutsideConnection = (b.requireOutsideConnection != 0);

  cfg.taxResidential = clampI(static_cast<int>(b.taxResidential), 0, 100);
  cfg.taxCommercial = clampI(static_cast<int>(b.taxCommercial), 0, 100);
  cfg.taxIndustrial = clampI(static_cast<int>(b.taxIndustrial), 0, 100);

  cfg.maintenanceRoad = clampI(static_cast<int>(b.maintenanceRoad), 0, 100);
  cfg.maintenancePark = clampI(static_cast<int>(b.maintenancePark), 0, 100);

  cfg.taxHappinessPerCapita = clampF(b.taxHappinessPerCapita, 0.0f, 1.0f);

  cfg.residentialDesirabilityWeight = clampF(b.residentialDesirabilityWeight, 0.0f, 4.0f);
  cfg.commercialDesirabilityWeight = clampF(b.commercialDesirabilityWeight, 0.0f, 4.0f);
  cfg.industrialDesirabilityWeight = clampF(b.industrialDesirabilityWeight, 0.0f, 4.0f);
}


// --- v1 tile serialization (full tiles) ---

bool ReadTileV1(std::istream& is, Tile& t)
{
  std::uint8_t terrain = 0;
  std::uint8_t overlay = 0;

  if (!Read(is, terrain)) return false;
  if (!Read(is, overlay)) return false;

  t.terrain = static_cast<Terrain>(terrain);
  t.overlay = static_cast<Overlay>(overlay);

  if (!Read(is, t.height)) return false;
  if (!Read(is, t.variation)) return false;
  if (!Read(is, t.level)) return false;
  if (!Read(is, t.occupants)) return false;

  // v7+ adds districts; older formats default to district 0.
  t.district = 0;

  return true;
}

// --- v2 tile delta serialization (only mutable fields) ---

[[maybe_unused]] bool WriteTileDeltaV2(std::ostream& os, std::uint8_t overlay, std::uint8_t level, std::uint16_t occ)
{
  if (!Write(os, overlay)) return false;
  if (!Write(os, level)) return false;
  if (!Write(os, occ)) return false;
  return true;
}

bool ReadTileDeltaV2(std::istream& is, std::uint8_t& outOverlay, std::uint8_t& outLevel, std::uint16_t& outOcc)
{
  if (!Read(is, outOverlay)) return false;
  if (!Read(is, outLevel)) return false;
  if (!Read(is, outOcc)) return false;
  return true;
}

bool ReadAndValidateHeader(std::istream& is, std::uint32_t& outVersion, std::uint32_t& outW, std::uint32_t& outH,
                           std::uint64_t& outSeed, std::string& outError)
{
  outError.clear();

  char magic[8] = {};
  if (!ReadBytes(is, magic, sizeof(magic))) {
    outError = "Read failed (magic)";
    return false;
  }
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "Not a ProcIsoCity save file (bad magic)";
    return false;
  }

  if (!Read(is, outVersion)) {
    outError = "Read failed (version)";
    return false;
  }

  if (!Read(is, outW) || !Read(is, outH) || !Read(is, outSeed)) {
    outError = "Read failed (header fields)";
    return false;
  }

  constexpr std::uint32_t kMaxDim = 4096;
  if (outW == 0 || outH == 0 || outW > kMaxDim || outH > kMaxDim) {
    outError = "Invalid map dimensions in save file";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(outW) * static_cast<std::uint64_t>(outH);
  if (tileCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    outError = "Save file map dimensions overflow size_t";
    return false;
  }

  return true;
}

// --- Version-specific loaders ---

bool LoadBodyV1(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, std::string& outError)
{
  // v1: full tiles, no procgen config stored.
  outProcCfg = ProcGenConfig{};

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  World loaded(static_cast<int>(w), static_cast<int>(h), seed);
  FromBin(loaded.stats(), sb);

  for (int y = 0; y < loaded.height(); ++y) {
    for (int x = 0; x < loaded.width(); ++x) {
      Tile t;
      if (!ReadTileV1(is, t)) {
        outError = "Read failed (tiles)";
        return false;
      }
      loaded.at(x, y) = t;
    }
  }

  // Older saves (and bulk edits like undo/redo) may have stale road connectivity bits.
  loaded.recomputeRoadMasks();

  outWorld = std::move(loaded);
  return true;
}

// Apply v7-style delta streams (overlay/level/district/occupants + height edits) onto a baseline world.
// Used by both v7 (raw deltas) and v8 (compressed delta payload).
bool ApplyDeltasV7(std::istream& is, std::uint32_t w, std::uint32_t h, World& loaded, const ProcGenConfig& procCfg,
                   std::string& outError)
{
  // --- Overlay diffs ---
  std::uint32_t diffCount = 0;
  if (!ReadVarU32(is, diffCount)) {
    outError = "Read failed (diff count)";
    return false;
  }

  const std::uint64_t maxTiles = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (static_cast<std::uint64_t>(diffCount) > maxTiles) {
    outError = "Invalid diff count in save file";
    return false;
  }

  std::uint32_t prevIdx = 0;
  for (std::uint32_t i = 0; i < diffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (diff idx delta)";
      return false;
    }
    if (i > 0 && delta == 0) {
      outError = "Invalid diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxTiles || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint8_t overlayU8 = 0;
    std::uint8_t level = 1;
    std::uint8_t districtU8 = 0;
    if (!Read(is, overlayU8) || !Read(is, level) || !Read(is, districtU8)) {
      outError = "Read failed (diff tile header)";
      return false;
    }
    if (districtU8 >= static_cast<std::uint8_t>(kDistrictCount)) {
      outError = "Invalid district value in save file";
      return false;
    }

    std::uint32_t occ32 = 0;
    if (!ReadVarU32(is, occ32)) {
      outError = "Read failed (diff occupants)";
      return false;
    }
    if (occ32 > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) {
      outError = "Invalid occupants value in save file";
      return false;
    }
    const std::uint16_t occ = static_cast<std::uint16_t>(occ32);

    // Basic enum validation (avoid UB on corrupted saves).
    if (overlayU8 > static_cast<std::uint8_t>(Overlay::Park)) {
      outError = "Invalid overlay value in save file";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);

    Tile& t = loaded.at(x, y);
    t.overlay = static_cast<Overlay>(overlayU8);
    t.district = districtU8;

    const bool isZone =
      (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
    const bool isRoad = (t.overlay == Overlay::Road);

    if (isZone) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = occ;
    } else if (isRoad) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = 0;
    } else {
      t.level = 1;
      t.occupants = 0;
    }
  }

  // --- Height diffs ---
  std::uint32_t heightDiffCount = 0;
  if (!ReadVarU32(is, heightDiffCount)) {
    outError = "Read failed (height diff count)";
    return false;
  }
  if (static_cast<std::uint64_t>(heightDiffCount) > maxTiles) {
    outError = "Invalid height diff count in save file";
    return false;
  }

  prevIdx = 0;
  for (std::uint32_t i = 0; i < heightDiffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (height diff idx delta)";
      return false;
    }
    if (i > 0 && delta == 0) {
      outError = "Invalid height diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxTiles || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in height diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint16_t hq = 0;
    if (!Read(is, hq)) {
      outError = "Read failed (height diff value)";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);
    Tile& t = loaded.at(x, y);

    t.height = DequantizeHeight(hq);
    t.terrain = TerrainFromHeight(t.height, procCfg);

    if (t.terrain == Terrain::Water) {
      // If terraforming made this tile water, clear most overlays to keep invariants.
      // Roads on water are bridges and are allowed.
      if (t.overlay != Overlay::Road) {
        t.overlay = Overlay::None;
        t.level = 1;
        t.occupants = 0;
      } else {
        t.occupants = 0;
        t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(t.level), 1, 3));
      }
    }
  }

  // Road auto-tiling uses per-tile masks stored in Tile::variation low bits.
  // Deltas do not store those; we recompute after all overlays are applied.
  loaded.recomputeRoadMasks();

  return true;
}

bool LoadBodyV2(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, std::string& outError)
{
  // v2: seed + procgen config + tile deltas
  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  std::uint32_t diffCount = 0;
  if (!Read(is, diffCount)) {
    outError = "Read failed (diff count)";
    return false;
  }

  const std::uint64_t maxDiffs = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (static_cast<std::uint64_t>(diffCount) > maxDiffs) {
    outError = "Invalid diff count in save file";
    return false;
  }

  for (std::uint32_t i = 0; i < diffCount; ++i) {
    std::uint32_t idx = 0;
    if (!Read(is, idx)) {
      outError = "Read failed (diff idx)";
      return false;
    }

    if (static_cast<std::uint64_t>(idx) >= maxDiffs) {
      outError = "Invalid tile index in diff list";
      return false;
    }

    std::uint8_t overlayU8 = 0;
    std::uint8_t level = 1;
    std::uint16_t occ = 0;
    if (!ReadTileDeltaV2(is, overlayU8, level, occ)) {
      outError = "Read failed (diff tile)";
      return false;
    }

    // Basic enum validation (avoid UB on corrupted saves).
    if (overlayU8 > static_cast<std::uint8_t>(Overlay::Park)) {
      outError = "Invalid overlay value in save file";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);

    Tile& t = loaded.at(x, y);
    t.overlay = static_cast<Overlay>(overlayU8);

    const bool isZone =
      (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
    const bool isRoad = (t.overlay == Overlay::Road);

    if (isZone) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = occ;
    } else if (isRoad) {
      // Roads also use the level field (Street/Avenue/Highway) in newer versions.
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = 0;
    } else {
      // Keep non-zone tiles sane even if the file had garbage values.
      t.level = 1;
      t.occupants = 0;
    }
  }

  // Road auto-tiling uses per-tile masks stored in Tile::variation low bits.
  // Deltas do not store those; we recompute after all overlays are applied.
  loaded.recomputeRoadMasks();

  FromBin(loaded.stats(), sb);

  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV4(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, std::string& outError)
{
  // v4: same semantic payload as v2/v3 (seed + ProcGenConfig + tile diffs), but
  // the diff list is stored more compactly:
  //   varint(diffCount)
  //   repeated diffCount times:
  //     varint(idxDelta), u8 overlay, u8 level, varint(occupants)
  // Where idxDelta is delta-encoded from the previous idx (monotonically increasing).

  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  std::uint32_t diffCount = 0;
  if (!ReadVarU32(is, diffCount)) {
    outError = "Read failed (diff count)";
    return false;
  }

  const std::uint64_t maxDiffs = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (static_cast<std::uint64_t>(diffCount) > maxDiffs) {
    outError = "Invalid diff count in save file";
    return false;
  }

  std::uint32_t prevIdx = 0;
  for (std::uint32_t i = 0; i < diffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (diff idx delta)";
      return false;
    }

    if (i > 0 && delta == 0) {
      outError = "Invalid diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxDiffs || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint8_t overlayU8 = 0;
    std::uint8_t level = 1;
    if (!Read(is, overlayU8) || !Read(is, level)) {
      outError = "Read failed (diff tile header)";
      return false;
    }

    std::uint32_t occ32 = 0;
    if (!ReadVarU32(is, occ32)) {
      outError = "Read failed (diff occupants)";
      return false;
    }
    if (occ32 > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) {
      outError = "Invalid occupants value in save file";
      return false;
    }
    const std::uint16_t occ = static_cast<std::uint16_t>(occ32);

    // Basic enum validation (avoid UB on corrupted saves).
    if (overlayU8 > static_cast<std::uint8_t>(Overlay::Park)) {
      outError = "Invalid overlay value in save file";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);

    Tile& t = loaded.at(x, y);
    t.overlay = static_cast<Overlay>(overlayU8);

    const bool isZone =
      (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
    const bool isRoad = (t.overlay == Overlay::Road);

    if (isZone) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = occ;
    } else if (isRoad) {
      // Roads also use the level field (Street/Avenue/Highway) in newer versions.
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = 0;
    } else {
      // Keep non-zone tiles sane even if the file had garbage values.
      t.level = 1;
      t.occupants = 0;
    }
  }

  // Road auto-tiling uses per-tile masks stored in Tile::variation low bits.
  // Deltas do not store those; we recompute after all overlays are applied.
  loaded.recomputeRoadMasks();

  FromBin(loaded.stats(), sb);

  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV5(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, std::string& outError)
{
  // v5: v4 + height deltas (terraforming).
  //
  // Payload:
  //   ProcGenConfigBin
  //   StatsBin
  //   varint(overlayDiffCount)
  //   repeated overlayDiffCount times:
  //     varint(idxDelta), u8 overlay, u8 level, varint(occupants)
  //   varint(heightDiffCount)
  //   repeated heightDiffCount times:
  //     varint(idxDelta), u16 heightQ

  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // --- Overlay diffs (same as v4) ---
  std::uint32_t diffCount = 0;
  if (!ReadVarU32(is, diffCount)) {
    outError = "Read failed (diff count)";
    return false;
  }

  const std::uint64_t maxTiles = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (static_cast<std::uint64_t>(diffCount) > maxTiles) {
    outError = "Invalid diff count in save file";
    return false;
  }

  std::uint32_t prevIdx = 0;
  for (std::uint32_t i = 0; i < diffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (diff idx delta)";
      return false;
    }
    if (i > 0 && delta == 0) {
      outError = "Invalid diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxTiles || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint8_t overlayU8 = 0;
    std::uint8_t level = 1;
    if (!Read(is, overlayU8) || !Read(is, level)) {
      outError = "Read failed (diff tile header)";
      return false;
    }

    std::uint32_t occ32 = 0;
    if (!ReadVarU32(is, occ32)) {
      outError = "Read failed (diff occupants)";
      return false;
    }
    if (occ32 > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) {
      outError = "Invalid occupants value in save file";
      return false;
    }
    const std::uint16_t occ = static_cast<std::uint16_t>(occ32);

    // Basic enum validation (avoid UB on corrupted saves).
    if (overlayU8 > static_cast<std::uint8_t>(Overlay::Park)) {
      outError = "Invalid overlay value in save file";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);

    Tile& t = loaded.at(x, y);
    t.overlay = static_cast<Overlay>(overlayU8);

    const bool isZone =
      (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
    const bool isRoad = (t.overlay == Overlay::Road);

    if (isZone) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = occ;
    } else if (isRoad) {
      // Roads also use the level field (Street/Avenue/Highway) in newer versions.
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = 0;
    } else {
      // Keep non-zone tiles sane even if the file had garbage values.
      t.level = 1;
      t.occupants = 0;
    }
  }

  // --- Height diffs ---
  std::uint32_t heightDiffCount = 0;
  if (!ReadVarU32(is, heightDiffCount)) {
    outError = "Read failed (height diff count)";
    return false;
  }
  if (static_cast<std::uint64_t>(heightDiffCount) > maxTiles) {
    outError = "Invalid height diff count in save file";
    return false;
  }

  prevIdx = 0;
  for (std::uint32_t i = 0; i < heightDiffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (height diff idx delta)";
      return false;
    }
    if (i > 0 && delta == 0) {
      outError = "Invalid height diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxTiles || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in height diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint16_t hq = 0;
    if (!Read(is, hq)) {
      outError = "Read failed (height diff value)";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);
    Tile& t = loaded.at(x, y);

    t.height = DequantizeHeight(hq);
    t.terrain = TerrainFromHeight(t.height, outProcCfg);

    if (t.terrain == Terrain::Water) {
      // If terraforming made this tile water, clear most overlays to keep invariants.
      // Roads on water are bridges and are allowed.
      if (t.overlay != Overlay::Road) {
        t.overlay = Overlay::None;
        t.level = 1;
        t.occupants = 0;
      } else {
        t.occupants = 0;
        t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(t.level), 1, 3));
      }
    }
  }

  // Road auto-tiling uses per-tile masks stored in Tile::variation low bits.
  // Deltas do not store those; we recompute after all overlays are applied.
  loaded.recomputeRoadMasks();

  FromBin(loaded.stats(), sb);

  outWorld = std::move(loaded);
  return true;
}


bool LoadBodyV6(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v6: v5 + SimConfig (policy/economy settings).
  //
  // Payload:
  //   ProcGenConfigBin
  //   StatsBin
  //   SimConfigBin
  //   varint(overlayDiffCount)
  //   repeated overlayDiffCount times:
  //     varint(idxDelta), u8 overlay, u8 level, varint(occupants)
  //   varint(heightDiffCount)
  //   repeated heightDiffCount times:
  //     varint(idxDelta), u16 heightQ

  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // --- Overlay diffs (same as v4/v5) ---
  std::uint32_t diffCount = 0;
  if (!ReadVarU32(is, diffCount)) {
    outError = "Read failed (diff count)";
    return false;
  }

  const std::uint64_t maxTiles = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (static_cast<std::uint64_t>(diffCount) > maxTiles) {
    outError = "Invalid diff count in save file";
    return false;
  }

  std::uint32_t prevIdx = 0;
  for (std::uint32_t i = 0; i < diffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (diff idx delta)";
      return false;
    }
    if (i > 0 && delta == 0) {
      outError = "Invalid diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxTiles || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint8_t overlayU8 = 0;
    std::uint8_t level = 1;
    if (!Read(is, overlayU8) || !Read(is, level)) {
      outError = "Read failed (diff tile header)";
      return false;
    }

    std::uint32_t occ32 = 0;
    if (!ReadVarU32(is, occ32)) {
      outError = "Read failed (diff occupants)";
      return false;
    }
    if (occ32 > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) {
      outError = "Invalid occupants value in save file";
      return false;
    }
    const std::uint16_t occ = static_cast<std::uint16_t>(occ32);

    // Basic enum validation (avoid UB on corrupted saves).
    if (overlayU8 > static_cast<std::uint8_t>(Overlay::Park)) {
      outError = "Invalid overlay value in save file";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);

    Tile& t = loaded.at(x, y);
    t.overlay = static_cast<Overlay>(overlayU8);

    const bool isZone =
      (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);
    const bool isRoad = (t.overlay == Overlay::Road);

    if (isZone) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = occ;
    } else if (isRoad) {
      // Roads also use the level field (Street/Avenue/Highway) in newer versions.
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = 0;
    } else {
      // Keep non-zone tiles sane even if the file had garbage values.
      t.level = 1;
      t.occupants = 0;
    }
  }

  // --- Height diffs (same as v5) ---
  std::uint32_t heightDiffCount = 0;
  if (!ReadVarU32(is, heightDiffCount)) {
    outError = "Read failed (height diff count)";
    return false;
  }
  if (static_cast<std::uint64_t>(heightDiffCount) > maxTiles) {
    outError = "Invalid height diff count in save file";
    return false;
  }

  prevIdx = 0;
  for (std::uint32_t i = 0; i < heightDiffCount; ++i) {
    std::uint32_t delta = 0;
    if (!ReadVarU32(is, delta)) {
      outError = "Read failed (height diff idx delta)";
      return false;
    }
    if (i > 0 && delta == 0) {
      outError = "Invalid height diff idx delta (non-increasing)";
      return false;
    }

    const std::uint64_t idx64 = static_cast<std::uint64_t>(prevIdx) + static_cast<std::uint64_t>(delta);
    if (idx64 >= maxTiles || idx64 > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Invalid tile index in height diff list";
      return false;
    }
    const std::uint32_t idx = static_cast<std::uint32_t>(idx64);
    prevIdx = idx;

    std::uint16_t hq = 0;
    if (!Read(is, hq)) {
      outError = "Read failed (height diff value)";
      return false;
    }

    const int x = static_cast<int>(idx % w);
    const int y = static_cast<int>(idx / w);
    Tile& t = loaded.at(x, y);

    t.height = DequantizeHeight(hq);
    t.terrain = TerrainFromHeight(t.height, outProcCfg);

    if (t.terrain == Terrain::Water) {
      // If terraforming made this tile water, clear most overlays to keep invariants.
      // Roads on water are bridges and are allowed.
      if (t.overlay != Overlay::Road) {
        t.overlay = Overlay::None;
        t.level = 1;
        t.occupants = 0;
      } else {
        t.occupants = 0;
        t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(t.level), 1, 3));
      }
    }
  }

  // Road auto-tiling uses per-tile masks stored in Tile::variation low bits.
  // Deltas do not store those; we recompute after all overlays are applied.
  loaded.recomputeRoadMasks();

  FromBin(loaded.stats(), sb);

  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV7(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v7: v6 + per-tile districts + optional district policy multipliers.
  //
  // Payload:
  //   ProcGenConfigBin
  //   StatsBin
  //   SimConfigBin
  //   u8 districtPoliciesEnabled
  //   DistrictPolicyBin[kDistrictCount]
  //   varint(overlayDiffCount)
  //   repeated overlayDiffCount times:
  //     varint(idxDelta), u8 overlay, u8 level, u8 district, varint(occupants)
  //   varint(heightDiffCount)
  //   repeated heightDiffCount times:
  //     varint(idxDelta), u16 heightQ

  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  // District policy chunk (v7+).
  std::uint8_t dpEnabled = 0;
  if (!Read(is, dpEnabled)) {
    outError = "Read failed (district policy enabled)";
    return false;
  }
  outSimCfg.districtPoliciesEnabled = (dpEnabled != 0);

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBin dpb{};
    if (!Read(is, dpb)) {
      outError = "Read failed (district policy)";
      return false;
    }
    FromBin(outSimCfg.districtPolicies[static_cast<std::size_t>(d)], dpb);
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  if (!ApplyDeltasV7(is, w, h, loaded, outProcCfg, outError)) {
    return false;
  }
  FromBin(loaded.stats(), sb);

  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV8(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v8: v7 + compressed delta payload.
  //
  // Payload:
  //   ProcGenConfigBin
  //   StatsBin
  //   SimConfigBin
  //   u8 districtPoliciesEnabled
  //   DistrictPolicyBin[kDistrictCount]
  //   u8 compressionMethod (0=None, 1=SLLZ)
  //   varint(uncompressedSize)
  //   varint(storedSize)
  //   stored bytes
  //
  // The uncompressed bytes are a v7-style delta stream (see ApplyDeltasV7).

  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  // District policy chunk (v7+).
  std::uint8_t dpEnabled = 0;
  if (!Read(is, dpEnabled)) {
    outError = "Read failed (district policy enabled)";
    return false;
  }
  outSimCfg.districtPoliciesEnabled = (dpEnabled != 0);

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBin dpb{};
    if (!Read(is, dpb)) {
      outError = "Read failed (district policy)";
      return false;
    }
    FromBin(outSimCfg.districtPolicies[static_cast<std::size_t>(d)], dpb);
  }

  // Compressed delta payload.
  std::uint8_t methodU8 = 0;
  if (!Read(is, methodU8)) {
    outError = "Read failed (compression method)";
    return false;
  }

  std::uint32_t uncompressedSize = 0;
  std::uint32_t storedSize = 0;
  if (!ReadVarU32(is, uncompressedSize) || !ReadVarU32(is, storedSize)) {
    outError = "Read failed (compressed payload sizes)";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  const std::uint64_t maxReasonable = tileCount * 32ull + 1024ull;
  if (static_cast<std::uint64_t>(uncompressedSize) > maxReasonable || static_cast<std::uint64_t>(storedSize) > maxReasonable) {
    outError = "Invalid compressed payload size";
    return false;
  }

  std::vector<std::uint8_t> stored(storedSize);
  if (storedSize > 0) {
    is.read(reinterpret_cast<char*>(stored.data()), static_cast<std::streamsize>(storedSize));
    if (!is.good()) {
      outError = "Read failed (compressed payload bytes)";
      return false;
    }
  }

  std::vector<std::uint8_t> delta;
  if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::None)) {
    if (storedSize != uncompressedSize) {
      outError = "Invalid payload sizes for uncompressed delta stream";
      return false;
    }
    delta = std::move(stored);
  } else if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::SLLZ)) {
    std::string decErr;
    if (!DecompressSLLZ(stored.data(), stored.size(), static_cast<std::size_t>(uncompressedSize), delta, decErr)) {
      outError = "Delta payload decompression failed: " + decErr;
      return false;
    }
  } else {
    outError = "Unknown compression method in save file";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // Parse delta stream from an in-memory buffer.
  std::string deltaStr(reinterpret_cast<const char*>(delta.data()), delta.size());
  std::istringstream ds(deltaStr, std::ios::binary);
  if (!ApplyDeltasV7(ds, w, h, loaded, outProcCfg, outError)) {
    return false;
  }

  FromBin(loaded.stats(), sb);
  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV9(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v9: v8 + ProcGen erosion config.
  //
  // Payload:
  //   ProcGenConfigBin
  //   ErosionConfigBin
  //   StatsBin
  //   SimConfigBin
  //   u8 districtPoliciesEnabled
  //   DistrictPolicyBin[kDistrictCount]
  //   u8 compressionMethod (0=None, 1=SLLZ)
  //   varint(uncompressedSize)
  //   varint(storedSize)
  //   stored bytes
  //
  // The uncompressed bytes are a v7-style delta stream (see ApplyDeltasV7).

  ProcGenConfigBin pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBin(outProcCfg, pcb);

  ErosionConfigBin ecb{};
  if (!Read(is, ecb)) {
    outError = "Read failed (erosion config)";
    return false;
  }
  FromBin(outProcCfg.erosion, ecb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  // District policy chunk (v7+).
  std::uint8_t dpEnabled = 0;
  if (!Read(is, dpEnabled)) {
    outError = "Read failed (district policy enabled)";
    return false;
  }
  outSimCfg.districtPoliciesEnabled = (dpEnabled != 0);

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBin dpb{};
    if (!Read(is, dpb)) {
      outError = "Read failed (district policy)";
      return false;
    }
    FromBin(outSimCfg.districtPolicies[static_cast<std::size_t>(d)], dpb);
  }

  // Compressed delta payload.
  std::uint8_t methodU8 = 0;
  if (!Read(is, methodU8)) {
    outError = "Read failed (compression method)";
    return false;
  }

  std::uint32_t uncompressedSize = 0;
  std::uint32_t storedSize = 0;
  if (!ReadVarU32(is, uncompressedSize) || !ReadVarU32(is, storedSize)) {
    outError = "Read failed (compressed payload sizes)";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  const std::uint64_t maxReasonable = tileCount * 32ull + 1024ull;
  if (static_cast<std::uint64_t>(uncompressedSize) > maxReasonable || static_cast<std::uint64_t>(storedSize) > maxReasonable) {
    outError = "Invalid compressed payload size";
    return false;
  }

  std::vector<std::uint8_t> stored(storedSize);
  if (storedSize > 0) {
    is.read(reinterpret_cast<char*>(stored.data()), static_cast<std::streamsize>(storedSize));
    if (!is.good()) {
      outError = "Read failed (compressed payload bytes)";
      return false;
    }
  }

  std::vector<std::uint8_t> delta;
  if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::None)) {
    if (storedSize != uncompressedSize) {
      outError = "Invalid payload sizes for uncompressed delta stream";
      return false;
    }
    delta = std::move(stored);
  } else if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::SLLZ)) {
    std::string decErr;
    if (!DecompressSLLZ(stored.data(), stored.size(), static_cast<std::size_t>(uncompressedSize), delta, decErr)) {
      outError = "Delta payload decompression failed: " + decErr;
      return false;
    }
  } else {
    outError = "Unknown compression method in save file";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // Parse delta stream from an in-memory buffer.
  std::string deltaStr(reinterpret_cast<const char*>(delta.data()), delta.size());
  std::istringstream ds(deltaStr, std::ios::binary);
  if (!ApplyDeltasV7(ds, w, h, loaded, outProcCfg, outError)) {
    return false;
  }

  FromBin(loaded.stats(), sb);
  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV10(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                 ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v10: v9 + ProcGen terrain preset config.
  //
  // Payload:
  //   ProcGenConfigBinV10
  //   ErosionConfigBin
  //   StatsBin
  //   SimConfigBin
  //   u8 districtPoliciesEnabled
  //   DistrictPolicyBin[kDistrictCount]
  //   u8 compressionMethod (0=None, 1=SLLZ)
  //   varint(uncompressedSize)
  //   varint(storedSize)
  //   stored bytes
  //
  // The uncompressed bytes are a v7-style delta stream (see ApplyDeltasV7).

  ProcGenConfigBinV10 pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBinV10(outProcCfg, pcb);

  ErosionConfigBin ecb{};
  if (!Read(is, ecb)) {
    outError = "Read failed (erosion config)";
    return false;
  }
  FromBin(outProcCfg.erosion, ecb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  // District policy chunk (v7+).
  std::uint8_t dpEnabled = 0;
  if (!Read(is, dpEnabled)) {
    outError = "Read failed (district policy enabled)";
    return false;
  }
  outSimCfg.districtPoliciesEnabled = (dpEnabled != 0);

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBin dpb{};
    if (!Read(is, dpb)) {
      outError = "Read failed (district policy)";
      return false;
    }
    FromBin(outSimCfg.districtPolicies[static_cast<std::size_t>(d)], dpb);
  }

  // Compressed delta payload.
  std::uint8_t methodU8 = 0;
  if (!Read(is, methodU8)) {
    outError = "Read failed (compression method)";
    return false;
  }

  std::uint32_t uncompressedSize = 0;
  std::uint32_t storedSize = 0;
  if (!ReadVarU32(is, uncompressedSize) || !ReadVarU32(is, storedSize)) {
    outError = "Read failed (compressed payload sizes)";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  const std::uint64_t maxReasonable = tileCount * 32ull + 1024ull;
  if (static_cast<std::uint64_t>(uncompressedSize) > maxReasonable ||
      static_cast<std::uint64_t>(storedSize) > maxReasonable) {
    outError = "Invalid compressed payload size";
    return false;
  }

  std::vector<std::uint8_t> stored(storedSize);
  if (storedSize > 0) {
    is.read(reinterpret_cast<char*>(stored.data()), static_cast<std::streamsize>(storedSize));
    if (!is.good()) {
      outError = "Read failed (compressed payload bytes)";
      return false;
    }
  }

  std::vector<std::uint8_t> delta;
  if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::None)) {
    if (storedSize != uncompressedSize) {
      outError = "Invalid payload sizes for uncompressed delta stream";
      return false;
    }
    delta = std::move(stored);
  } else if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::SLLZ)) {
    std::string decErr;
    if (!DecompressSLLZ(stored.data(), stored.size(), static_cast<std::size_t>(uncompressedSize), delta, decErr)) {
      outError = "Delta payload decompression failed: " + decErr;
      return false;
    }
  } else {
    outError = "Unknown compression method in save file";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // Parse delta stream from an in-memory buffer.
  std::string deltaStr(reinterpret_cast<const char*>(delta.data()), delta.size());
  std::istringstream ds(deltaStr, std::ios::binary);
  if (!ApplyDeltasV7(ds, w, h, loaded, outProcCfg, outError)) {
    return false;
  }

  FromBin(loaded.stats(), sb);
  outWorld = std::move(loaded);
  return true;
}

bool LoadBodyV11(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                 ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v11: v10 + ProcGen road hierarchy config.
  //
  // Payload:
  //   ProcGenConfigBinV11
  //   ErosionConfigBin
  //   StatsBin
  //   SimConfigBin
  //   u8 districtPoliciesEnabled
  //   DistrictPolicyBin[kDistrictCount]
  //   u8 compressionMethod (0=None, 1=SLLZ)
  //   varint(uncompressedSize)
  //   varint(storedSize)
  //   stored bytes
  //
  // The uncompressed bytes are a v7-style delta stream (see ApplyDeltasV7).

  ProcGenConfigBinV11 pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBinV11(outProcCfg, pcb);

  ErosionConfigBin ecb{};
  if (!Read(is, ecb)) {
    outError = "Read failed (erosion config)";
    return false;
  }
  FromBin(outProcCfg.erosion, ecb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  // District policy chunk (v7+).
  std::uint8_t dpEnabled = 0;
  if (!Read(is, dpEnabled)) {
    outError = "Read failed (district policy enabled)";
    return false;
  }
  outSimCfg.districtPoliciesEnabled = (dpEnabled != 0);

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBin dpb{};
    if (!Read(is, dpb)) {
      outError = "Read failed (district policy)";
      return false;
    }
    FromBin(outSimCfg.districtPolicies[static_cast<std::size_t>(d)], dpb);
  }

  // Compressed delta payload.
  std::uint8_t methodU8 = 0;
  if (!Read(is, methodU8)) {
    outError = "Read failed (compression method)";
    return false;
  }

  std::uint32_t uncompressedSize = 0;
  std::uint32_t storedSize = 0;
  if (!ReadVarU32(is, uncompressedSize) || !ReadVarU32(is, storedSize)) {
    outError = "Read failed (compressed payload sizes)";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  const std::uint64_t maxReasonable = tileCount * 32ull + 1024ull;
  if (static_cast<std::uint64_t>(uncompressedSize) > maxReasonable ||
      static_cast<std::uint64_t>(storedSize) > maxReasonable) {
    outError = "Invalid compressed payload size";
    return false;
  }

  std::vector<std::uint8_t> stored(storedSize);
  if (storedSize > 0) {
    is.read(reinterpret_cast<char*>(stored.data()), static_cast<std::streamsize>(storedSize));
    if (!is.good()) {
      outError = "Read failed (compressed payload bytes)";
      return false;
    }
  }

  std::vector<std::uint8_t> delta;
  if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::None)) {
    if (storedSize != uncompressedSize) {
      outError = "Invalid payload sizes for uncompressed delta stream";
      return false;
    }
    delta = std::move(stored);
  } else if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::SLLZ)) {
    std::string decErr;
    if (!DecompressSLLZ(stored.data(), stored.size(), static_cast<std::size_t>(uncompressedSize), delta, decErr)) {
      outError = "Delta payload decompression failed: " + decErr;
      return false;
    }
  } else {
    outError = "Unknown compression method in save file";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // Parse delta stream from an in-memory buffer.
  std::string deltaStr(reinterpret_cast<const char*>(delta.data()), delta.size());
  std::istringstream ds(deltaStr, std::ios::binary);
  if (!ApplyDeltasV7(ds, w, h, loaded, outProcCfg, outError)) {
    return false;
  }

  FromBin(loaded.stats(), sb);
  outWorld = std::move(loaded);
  return true;
}



} // namespace

bool LoadBodyV12(std::istream& is, std::uint32_t w, std::uint32_t h, std::uint64_t seed, World& outWorld,
                 ProcGenConfig& outProcCfg, SimConfig& outSimCfg, std::string& outError)
{
  // v12: v11 + ProcGen districting mode config.
  //
  // Payload:
  //   ProcGenConfigBinV12
  //   ErosionConfigBin
  //   StatsBin
  //   SimConfigBin
  //   u8 districtPoliciesEnabled
  //   DistrictPolicyBin[kDistrictCount]
  //   u8 compressionMethod (0=None, 1=SLLZ)
  //   varint(uncompressedSize)
  //   varint(storedSize)
  //   stored bytes
  //
  // The uncompressed bytes are a v7-style delta stream (see ApplyDeltasV7).

  ProcGenConfigBinV12 pcb{};
  if (!Read(is, pcb)) {
    outError = "Read failed (procgen config)";
    return false;
  }
  FromBinV12(outProcCfg, pcb);

  ErosionConfigBin ecb{};
  if (!Read(is, ecb)) {
    outError = "Read failed (erosion config)";
    return false;
  }
  FromBin(outProcCfg.erosion, ecb);

  StatsBin sb{};
  if (!Read(is, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  SimConfigBin scb{};
  if (!Read(is, scb)) {
    outError = "Read failed (sim config)";
    return false;
  }
  FromBin(outSimCfg, scb);

  // District policy chunk (v7+).
  std::uint8_t dpEnabled = 0;
  if (!Read(is, dpEnabled)) {
    outError = "Read failed (district policy enabled)";
    return false;
  }
  outSimCfg.districtPoliciesEnabled = (dpEnabled != 0);

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictPolicyBin dpb{};
    if (!Read(is, dpb)) {
      outError = "Read failed (district policy)";
      return false;
    }
    FromBin(outSimCfg.districtPolicies[static_cast<std::size_t>(d)], dpb);
  }

  // Compressed delta payload.
  std::uint8_t methodU8 = 0;
  if (!Read(is, methodU8)) {
    outError = "Read failed (compression method)";
    return false;
  }

  std::uint32_t uncompressedSize = 0;
  std::uint32_t storedSize = 0;
  if (!ReadVarU32(is, uncompressedSize) || !ReadVarU32(is, storedSize)) {
    outError = "Read failed (compressed payload sizes)";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  const std::uint64_t maxReasonable = tileCount * 32ull + 1024ull;
  if (static_cast<std::uint64_t>(uncompressedSize) > maxReasonable ||
      static_cast<std::uint64_t>(storedSize) > maxReasonable) {
    outError = "Invalid compressed payload size";
    return false;
  }

  std::vector<std::uint8_t> stored(storedSize);
  if (storedSize > 0) {
    is.read(reinterpret_cast<char*>(stored.data()), static_cast<std::streamsize>(storedSize));
    if (!is.good()) {
      outError = "Read failed (compressed payload bytes)";
      return false;
    }
  }

  std::vector<std::uint8_t> delta;
  if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::None)) {
    if (storedSize != uncompressedSize) {
      outError = "Invalid payload sizes for uncompressed delta stream";
      return false;
    }
    delta = std::move(stored);
  } else if (methodU8 == static_cast<std::uint8_t>(CompressionMethod::SLLZ)) {
    std::string decErr;
    if (!DecompressSLLZ(stored.data(), stored.size(), static_cast<std::size_t>(uncompressedSize), delta, decErr)) {
      outError = "Delta payload decompression failed: " + decErr;
      return false;
    }
  } else {
    outError = "Unknown compression method in save file";
    return false;
  }

  World loaded = GenerateWorld(static_cast<int>(w), static_cast<int>(h), seed, outProcCfg);

  // Parse delta stream from an in-memory buffer.
  std::string deltaStr(reinterpret_cast<const char*>(delta.data()), delta.size());
  std::istringstream ds(deltaStr, std::ios::binary);
  if (!ApplyDeltasV7(ds, w, h, loaded, outProcCfg, outError)) {
    return false;
  }

  FromBin(loaded.stats(), sb);
  outWorld = std::move(loaded);
  return true;
}

bool ReadSaveSummary(const std::string& path, SaveSummary& outSummary, std::string& outError, bool verifyCrc)
{
  outError.clear();
  outSummary = SaveSummary{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Unable to open file for reading: " + path;
    return false;
  }

  std::uint32_t version = 0;
  std::uint32_t w = 0;
  std::uint32_t h = 0;
  std::uint64_t seed = 0;
  if (!ReadAndValidateHeader(f, version, w, h, seed, outError)) {
    return false;
  }

  outSummary.version = version;
  outSummary.width = static_cast<int>(w);
  outSummary.height = static_cast<int>(h);
  outSummary.seed = seed;

  // Local copies of the bin structs so we can parse without loading tiles.
  struct StatsBinLocal {
    std::int32_t day = 0;
    std::int32_t population = 0;
    std::int32_t housingCapacity = 0;
    std::int32_t jobsCapacity = 0;
    std::int32_t employed = 0;
    float happiness = 0.5f;
    std::int32_t money = 0;
    std::int32_t roads = 0;
    std::int32_t parks = 0;
  };

  auto fromBinStats = [](Stats& s, const StatsBinLocal& b) {
    s.day = static_cast<int>(b.day);
    s.population = static_cast<int>(b.population);
    s.housingCapacity = static_cast<int>(b.housingCapacity);
    s.jobsCapacity = static_cast<int>(b.jobsCapacity);
    s.employed = static_cast<int>(b.employed);
    s.happiness = b.happiness;
    s.money = static_cast<int>(b.money);
    s.roads = static_cast<int>(b.roads);
    s.parks = static_cast<int>(b.parks);
  };

  auto readBytes = [&](void* dst, std::size_t n) -> bool {
    f.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
    return f.good();
  };

  auto readPOD = [&](auto& dst) -> bool {
    return readBytes(&dst, sizeof(dst));
  };

  if (version == 1) {
    // v1: header + StatsBin, then full tiles.
    StatsBinLocal sb{};
    if (!readPOD(sb)) {
      outError = "Read failed (stats)";
      return false;
    }
    fromBinStats(outSummary.stats, sb);
    outSummary.hasStats = true;
    outSummary.hasProcCfg = false;
    outSummary.hasSimCfg = false;
  } else {
    if (version < 2 || version > kVersionCurrent) {
      outError = "Unsupported save version";
      return false;
    }
    if (version >= kVersionV12) {
      struct ProcGenConfigBinLocalV12 {
        float terrainScale = 0.08f;
        float waterLevel = 0.35f;
        float sandLevel = 0.42f;
        std::int32_t hubs = 4;
        std::int32_t extraConnections = 2;
        float zoneChance = 0.22f;
        float parkChance = 0.06f;

        std::uint8_t terrainPreset = 0;
        std::uint8_t roadHierarchyEnabled = 0;
        std::uint8_t districtingMode = 0;
        std::uint8_t _pad0 = 0;

        float terrainPresetStrength = 1.0f;
        float roadHierarchyStrength = 0.0f;
      };

      ProcGenConfigBinLocalV12 pcb{};
      if (!readPOD(pcb)) {
        outError = "Read failed (procgen config)";
        return false;
      }

      outSummary.procCfg.terrainScale = pcb.terrainScale;
      outSummary.procCfg.waterLevel = pcb.waterLevel;
      outSummary.procCfg.sandLevel = pcb.sandLevel;
      outSummary.procCfg.hubs = static_cast<int>(pcb.hubs);
      outSummary.procCfg.extraConnections = static_cast<int>(pcb.extraConnections);
      outSummary.procCfg.zoneChance = pcb.zoneChance;
      outSummary.procCfg.parkChance = pcb.parkChance;

      std::uint8_t presetU8 = pcb.terrainPreset;
      if (presetU8 > static_cast<std::uint8_t>(ProcGenTerrainPreset::MountainRing)) {
        presetU8 = static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
      }
      outSummary.procCfg.terrainPreset = static_cast<ProcGenTerrainPreset>(presetU8);
      outSummary.procCfg.terrainPresetStrength = std::clamp(pcb.terrainPresetStrength, 0.0f, 5.0f);

      outSummary.procCfg.roadHierarchyEnabled = (pcb.roadHierarchyEnabled != 0);
      outSummary.procCfg.roadHierarchyStrength = std::clamp(pcb.roadHierarchyStrength, 0.0f, 3.0f);

      std::uint8_t dmU8 = pcb.districtingMode;
      if (dmU8 > static_cast<std::uint8_t>(ProcGenDistrictingMode::BlockGraph)) {
        dmU8 = static_cast<std::uint8_t>(ProcGenDistrictingMode::Voronoi);
      }
      outSummary.procCfg.districtingMode = static_cast<ProcGenDistrictingMode>(dmU8);
    } else if (version >= kVersionV11) {
      struct ProcGenConfigBinLocalV11 {
        float terrainScale = 0.08f;
        float waterLevel = 0.35f;
        float sandLevel = 0.42f;
        std::int32_t hubs = 4;
        std::int32_t extraConnections = 2;
        float zoneChance = 0.22f;
        float parkChance = 0.06f;

        std::uint8_t terrainPreset = 0;
        std::uint8_t roadHierarchyEnabled = 0;
        std::uint8_t _pad0 = 0;
        std::uint8_t _pad1 = 0;

        float terrainPresetStrength = 1.0f;
        float roadHierarchyStrength = 0.0f;
      };

      ProcGenConfigBinLocalV11 pcb{};
      if (!readPOD(pcb)) {
        outError = "Read failed (procgen config)";
        return false;
      }

      outSummary.procCfg.terrainScale = pcb.terrainScale;
      outSummary.procCfg.waterLevel = pcb.waterLevel;
      outSummary.procCfg.sandLevel = pcb.sandLevel;
      outSummary.procCfg.hubs = static_cast<int>(pcb.hubs);
      outSummary.procCfg.extraConnections = static_cast<int>(pcb.extraConnections);
      outSummary.procCfg.zoneChance = pcb.zoneChance;
      outSummary.procCfg.parkChance = pcb.parkChance;

      std::uint8_t presetU8 = pcb.terrainPreset;
      if (presetU8 > static_cast<std::uint8_t>(ProcGenTerrainPreset::MountainRing)) {
        presetU8 = static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
      }
      outSummary.procCfg.terrainPreset = static_cast<ProcGenTerrainPreset>(presetU8);
      outSummary.procCfg.terrainPresetStrength = std::clamp(pcb.terrainPresetStrength, 0.0f, 5.0f);

      outSummary.procCfg.roadHierarchyEnabled = (pcb.roadHierarchyEnabled != 0);
      outSummary.procCfg.roadHierarchyStrength = std::clamp(pcb.roadHierarchyStrength, 0.0f, 3.0f);

      // v11 did not include districting mode settings.
      outSummary.procCfg.districtingMode = ProcGenDistrictingMode::Voronoi;
    } else if (version >= kVersionV10) {
      struct ProcGenConfigBinLocalV10 {
        float terrainScale = 0.08f;
        float waterLevel = 0.35f;
        float sandLevel = 0.42f;
        std::int32_t hubs = 4;
        std::int32_t extraConnections = 2;
        float zoneChance = 0.22f;
        float parkChance = 0.06f;

        std::uint8_t terrainPreset = 0;
        std::uint8_t _pad0 = 0;
        std::uint8_t _pad1 = 0;
        std::uint8_t _pad2 = 0;

        float terrainPresetStrength = 1.0f;
      };

      ProcGenConfigBinLocalV10 pcb{};
      if (!readPOD(pcb)) {
        outError = "Read failed (procgen config)";
        return false;
      }

      outSummary.procCfg.terrainScale = pcb.terrainScale;
      outSummary.procCfg.waterLevel = pcb.waterLevel;
      outSummary.procCfg.sandLevel = pcb.sandLevel;
      outSummary.procCfg.hubs = static_cast<int>(pcb.hubs);
      outSummary.procCfg.extraConnections = static_cast<int>(pcb.extraConnections);
      outSummary.procCfg.zoneChance = pcb.zoneChance;
      outSummary.procCfg.parkChance = pcb.parkChance;

      std::uint8_t presetU8 = pcb.terrainPreset;
      if (presetU8 > static_cast<std::uint8_t>(ProcGenTerrainPreset::MountainRing)) {
        presetU8 = static_cast<std::uint8_t>(ProcGenTerrainPreset::Classic);
      }
      outSummary.procCfg.terrainPreset = static_cast<ProcGenTerrainPreset>(presetU8);
      outSummary.procCfg.terrainPresetStrength = std::clamp(pcb.terrainPresetStrength, 0.0f, 5.0f);

      // v10 did not include road hierarchy settings.
      outSummary.procCfg.roadHierarchyEnabled = false;
      outSummary.procCfg.roadHierarchyStrength = 0.0f;

      // v11 and older did not include districting mode settings.
      outSummary.procCfg.districtingMode = ProcGenDistrictingMode::Voronoi;
    } else {
      struct ProcGenConfigBinLocal {
        float terrainScale = 0.08f;
        float waterLevel = 0.35f;
        float sandLevel = 0.42f;
        std::int32_t hubs = 4;
        std::int32_t extraConnections = 2;
        float zoneChance = 0.22f;
        float parkChance = 0.06f;
      };

      ProcGenConfigBinLocal pcb{};
      if (!readPOD(pcb)) {
        outError = "Read failed (procgen config)";
        return false;
      }
      outSummary.procCfg.terrainScale = pcb.terrainScale;
      outSummary.procCfg.waterLevel = pcb.waterLevel;
      outSummary.procCfg.sandLevel = pcb.sandLevel;
      outSummary.procCfg.hubs = static_cast<int>(pcb.hubs);
      outSummary.procCfg.extraConnections = static_cast<int>(pcb.extraConnections);
      outSummary.procCfg.zoneChance = pcb.zoneChance;
      outSummary.procCfg.parkChance = pcb.parkChance;

      // Older versions did not store macro presets or road hierarchy.
      outSummary.procCfg.terrainPreset = ProcGenTerrainPreset::Classic;
      outSummary.procCfg.terrainPresetStrength = 1.0f;
      outSummary.procCfg.roadHierarchyEnabled = false;
      outSummary.procCfg.roadHierarchyStrength = 0.0f;

      // v11 and older did not include districting mode settings.
      outSummary.procCfg.districtingMode = ProcGenDistrictingMode::Voronoi;
    }

    // Erosion config (v9+). Older versions did not persist it.
    if (version >= kVersionV9) {
      struct ErosionConfigBinLocal {
        std::uint8_t enabled = 1;
        std::uint8_t riversEnabled = 1;
        std::uint8_t _pad0 = 0;
        std::uint8_t _pad1 = 0;

        std::int32_t thermalIterations = 20;
        float thermalTalus = 0.02f;
        float thermalRate = 0.50f;

        std::int32_t riverMinAccum = 0;
        float riverCarve = 0.055f;
        float riverCarvePower = 0.60f;

        std::int32_t smoothIterations = 1;
        float smoothRate = 0.25f;

        std::int32_t quantizeScale = 4096;
      };

      ErosionConfigBinLocal ecb{};
      if (!readPOD(ecb)) {
        outError = "Read failed (erosion config)";
        return false;
      }

      outSummary.procCfg.erosion.enabled = (ecb.enabled != 0);
      outSummary.procCfg.erosion.riversEnabled = (ecb.riversEnabled != 0);
      outSummary.procCfg.erosion.thermalIterations = static_cast<int>(ecb.thermalIterations);
      outSummary.procCfg.erosion.thermalTalus = ecb.thermalTalus;
      outSummary.procCfg.erosion.thermalRate = ecb.thermalRate;
      outSummary.procCfg.erosion.riverMinAccum = static_cast<int>(ecb.riverMinAccum);
      outSummary.procCfg.erosion.riverCarve = ecb.riverCarve;
      outSummary.procCfg.erosion.riverCarvePower = ecb.riverCarvePower;
      outSummary.procCfg.erosion.smoothIterations = static_cast<int>(ecb.smoothIterations);
      outSummary.procCfg.erosion.smoothRate = ecb.smoothRate;
      outSummary.procCfg.erosion.quantizeScale = static_cast<int>(ecb.quantizeScale);
    } else {
      outSummary.procCfg.erosion = ErosionConfig{};
      outSummary.procCfg.erosion.enabled = false;
    }

outSummary.hasProcCfg = true;

    StatsBinLocal sb{};
    if (!readPOD(sb)) {
      outError = "Read failed (stats)";
      return false;
    }
    fromBinStats(outSummary.stats, sb);
    outSummary.hasStats = true;

    if (version >= 6) {
      struct SimConfigBinLocal {
        float tickSeconds = 0.5f;
        std::int32_t parkInfluenceRadius = 6;
        std::uint8_t requireOutsideConnection = 1;
        std::int32_t taxResidential = 1;
        std::int32_t taxCommercial = 2;
        std::int32_t taxIndustrial = 2;
        std::int32_t maintenanceRoad = 1;
        std::int32_t maintenancePark = 1;
        float taxHappinessPerCapita = 0.02f;
        float residentialDesirabilityWeight = 0.70f;
        float commercialDesirabilityWeight = 0.80f;
        float industrialDesirabilityWeight = 0.80f;
      };

      SimConfigBinLocal scb{};
      if (!readPOD(scb)) {
        outError = "Read failed (sim config)";
        return false;
      }

      auto clampF = [](float v, float lo, float hi) -> float { return std::clamp(v, lo, hi); };
      auto clampI = [](int v, int lo, int hi) -> int { return std::clamp(v, lo, hi); };

      outSummary.simCfg.tickSeconds = clampF(scb.tickSeconds, 0.01f, 60.0f);
      outSummary.simCfg.parkInfluenceRadius = clampI(static_cast<int>(scb.parkInfluenceRadius), 0, 64);
      outSummary.simCfg.requireOutsideConnection = (scb.requireOutsideConnection != 0);
      outSummary.simCfg.taxResidential = clampI(static_cast<int>(scb.taxResidential), 0, 100);
      outSummary.simCfg.taxCommercial = clampI(static_cast<int>(scb.taxCommercial), 0, 100);
      outSummary.simCfg.taxIndustrial = clampI(static_cast<int>(scb.taxIndustrial), 0, 100);
      outSummary.simCfg.maintenanceRoad = clampI(static_cast<int>(scb.maintenanceRoad), 0, 100);
      outSummary.simCfg.maintenancePark = clampI(static_cast<int>(scb.maintenancePark), 0, 100);
      outSummary.simCfg.taxHappinessPerCapita = clampF(scb.taxHappinessPerCapita, 0.0f, 1.0f);
      outSummary.simCfg.residentialDesirabilityWeight = clampF(scb.residentialDesirabilityWeight, 0.0f, 4.0f);
      outSummary.simCfg.commercialDesirabilityWeight = clampF(scb.commercialDesirabilityWeight, 0.0f, 4.0f);
      outSummary.simCfg.industrialDesirabilityWeight = clampF(scb.industrialDesirabilityWeight, 0.0f, 4.0f);

      // v7 adds optional district policy multipliers.
      outSummary.simCfg.districtPoliciesEnabled = false;
      outSummary.simCfg.districtPolicies = {};

      if (version >= 7) {
        std::uint8_t dpEnabled = 0;
        if (!readPOD(dpEnabled)) {
          outError = "Read failed (district policy enabled)";
          return false;
        }
        outSummary.simCfg.districtPoliciesEnabled = (dpEnabled != 0);

        struct DistrictPolicyBinLocal {
          float taxResidentialMult = 1.0f;
          float taxCommercialMult = 1.0f;
          float taxIndustrialMult = 1.0f;
          float roadMaintenanceMult = 1.0f;
          float parkMaintenanceMult = 1.0f;
        };

        for (int d = 0; d < kDistrictCount; ++d) {
          DistrictPolicyBinLocal dpb{};
          if (!readPOD(dpb)) {
            outError = "Read failed (district policy)";
            return false;
          }

          DistrictPolicy& p = outSummary.simCfg.districtPolicies[static_cast<std::size_t>(d)];
          p.taxResidentialMult = clampF(dpb.taxResidentialMult, 0.0f, 10.0f);
          p.taxCommercialMult = clampF(dpb.taxCommercialMult, 0.0f, 10.0f);
          p.taxIndustrialMult = clampF(dpb.taxIndustrialMult, 0.0f, 10.0f);
          p.roadMaintenanceMult = clampF(dpb.roadMaintenanceMult, 0.0f, 10.0f);
          p.parkMaintenanceMult = clampF(dpb.parkMaintenanceMult, 0.0f, 10.0f);
        }
      }

      outSummary.hasSimCfg = true;
    }
  }

  if (verifyCrc && version >= 3) {
    bool ok = true;
    std::string err;
    if (!VerifyCrc32File(path, ok, err)) {
      outError = err;
      return false;
    }
    outSummary.crcChecked = true;
    outSummary.crcOk = ok;
  }

  

  return true;
}


namespace {

struct Crc32VecWriter {
  explicit Crc32VecWriter(std::vector<std::uint8_t>& out_) : out(out_) {}

  std::vector<std::uint8_t>& out;
  std::uint32_t crc = 0xFFFFFFFFu;

  bool writeBytes(const void* data, std::size_t size)
  {
    const auto* p = reinterpret_cast<const std::uint8_t*>(data);
    out.insert(out.end(), p, p + size);
    crc = Crc32Update(crc, p, size);
    return true;
  }

  template <typename T>
  bool write(const T& v)
  {
    static_assert(std::is_trivially_copyable_v<T>);
    return writeBytes(&v, sizeof(T));
  }

  std::uint32_t finalize() const { return crc ^ 0xFFFFFFFFu; }
};

// Append a trivially-copyable POD to a byte vector without any endianness conversion.
// This matches the on-disk save format (native little-endian on supported platforms).
template <typename T>
void AppendPOD(std::vector<std::uint8_t>& out, const T& v)
{
  static_assert(std::is_trivially_copyable_v<T>);
  const auto* p = reinterpret_cast<const std::uint8_t*>(&v);
  out.insert(out.end(), p, p + sizeof(T));
}

template <typename Writer>
bool WriteWorldBinaryPayload(Writer& cw, const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg,
                             std::string& outError)
{
  // Header
  if (!cw.writeBytes(kMagic, sizeof(kMagic))) {
    outError = "Write failed (magic)";
    return false;
  }

  if (!cw.write(kVersionCurrent)) {
    outError = "Write failed (version)";
    return false;
  }

  const std::uint32_t w = static_cast<std::uint32_t>(world.width());
  const std::uint32_t h = static_cast<std::uint32_t>(world.height());
  const std::uint64_t seed = world.seed();

  if (!cw.write(w) || !cw.write(h) || !cw.write(seed)) {
    outError = "Write failed (header fields)";
    return false;
  }

  // Procgen config (needed to regenerate the baseline for delta saves).
  const ProcGenConfigBinV12 pcb = ToBinV12(procCfg);
  if (!cw.write(pcb)) {
    outError = "Write failed (procgen config)";
    return false;
  }

  // Erosion config (v9+).
  const ErosionConfigBin ecb = ToBin(procCfg.erosion);
  if (!cw.write(ecb)) {
    outError = "Write failed (erosion config)";
    return false;
  }

  // Stats
  const StatsBin sb = ToBin(world.stats());
  if (!cw.write(sb)) {
    outError = "Write failed (stats)";
    return false;
  }

  // Sim config (policy/economy settings)
  const SimConfigBin scb = ToBin(simCfg);
  if (!cw.write(scb)) {
    outError = "Write failed (sim config)";
    return false;
  }

  // District policy multipliers (v7+).
  const std::uint8_t dpEnabled = simCfg.districtPoliciesEnabled ? 1u : 0u;
  if (!cw.write(dpEnabled)) {
    outError = "Write failed (district policy enabled)";
    return false;
  }
  for (int d = 0; d < kDistrictCount; ++d) {
    const DistrictPolicyBin dpb = ToBin(simCfg.districtPolicies[static_cast<std::size_t>(d)]);
    if (!cw.write(dpb)) {
      outError = "Write failed (district policy)";
      return false;
    }
  }

  // --- Tile deltas ---
  // We store only the tiles whose mutable fields differ from a regenerated baseline.
  // This keeps save files small while still being deterministic.
  const World baseline = GenerateWorld(world.width(), world.height(), seed, procCfg);

  struct Diff {
    std::uint32_t idx = 0;
    std::uint8_t overlay = 0;
    std::uint8_t level = 1;
    std::uint8_t district = 0;
    std::uint16_t occupants = 0;
  };

  std::vector<Diff> diffs;
  diffs.reserve(static_cast<std::size_t>(world.width() * world.height() / 8));

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& cur = world.at(x, y);
      const Tile& base = baseline.at(x, y);

      // Compare only the fields that can change during gameplay (player edits + sim).
      // Road masks (tile.variation low bits) are recomputed on load.
      if (cur.overlay != base.overlay || cur.level != base.level || cur.district != base.district ||
          cur.occupants != base.occupants) {
        const std::uint32_t idx = static_cast<std::uint32_t>(y * world.width() + x);
        const std::uint8_t d = static_cast<std::uint8_t>(
            std::clamp<int>(static_cast<int>(cur.district), 0, kDistrictCount - 1));
        diffs.push_back(Diff{idx, static_cast<std::uint8_t>(cur.overlay), cur.level, d, cur.occupants});
      }
    }
  }

  // --- Height deltas (terraforming) ---
  // v5 extends the delta format by persisting Tile::height changes.
  //
  // We store only tiles whose *quantized* height differs from the regenerated baseline.
  // Heights are stored as u16 in [0,65535] representing [0,1].
  struct HeightDiff {
    std::uint32_t idx = 0;
    std::uint16_t heightQ = 0;
  };

  std::vector<HeightDiff> heightDiffs;
  heightDiffs.reserve(static_cast<std::size_t>(world.width() * world.height() / 8));

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& cur = world.at(x, y);
      const Tile& base = baseline.at(x, y);

      const std::uint16_t qc = QuantizeHeight(cur.height);
      const std::uint16_t qb = QuantizeHeight(base.height);
      if (qc != qb) {
        const std::uint32_t idx = static_cast<std::uint32_t>(y * world.width() + x);
        heightDiffs.push_back(HeightDiff{idx, qc});
      }
    }
  }

  // --- Delta payload (v8) ---
  // v8 compresses the delta streams as a single blob:
  //   u8 compressionMethod (0=None, 1=SLLZ)
  //   varint(uncompressedSize)
  //   varint(storedSize)
  //   stored bytes (either raw or SLLZ compressed)
  //
  // The *uncompressed* delta payload is compatible with v7's encoding:
  //   varint(diffCount)
  //   repeated diffCount times:
  //     varint(idxDelta), u8 overlay, u8 level, u8 district, varint(occupants)
  //   varint(heightDiffCount)
  //   repeated heightDiffCount times:
  //     varint(idxDelta), u16 heightQ

  std::vector<std::uint8_t> deltaPayload;
  deltaPayload.reserve(diffs.size() * 8 + heightDiffs.size() * 4 + 32);
  VecWriter vw(deltaPayload);

  // Overlay/zone/road diffs.
  const std::uint32_t diffCount = static_cast<std::uint32_t>(diffs.size());
  if (!WriteVarU32(vw, diffCount)) {
    outError = "Write failed (diff count)";
    return false;
  }

  std::uint32_t prevIdx = 0;
  for (const Diff& d : diffs) {
    if (d.idx < prevIdx) {
      outError = "Internal error: diff list not sorted";
      return false;
    }

    const std::uint32_t delta = d.idx - prevIdx;
    prevIdx = d.idx;

    if (!WriteVarU32(vw, delta)) {
      outError = "Write failed (diff idx delta)";
      return false;
    }

    if (!vw.writePOD(d.overlay) || !vw.writePOD(d.level) || !vw.writePOD(d.district)) {
      outError = "Write failed (diff tile header)";
      return false;
    }

    if (!WriteVarU32(vw, static_cast<std::uint32_t>(d.occupants))) {
      outError = "Write failed (diff occupants)";
      return false;
    }
  }

  // Height diffs.
  const std::uint32_t heightDiffCount = static_cast<std::uint32_t>(heightDiffs.size());
  if (!WriteVarU32(vw, heightDiffCount)) {
    outError = "Write failed (height diff count)";
    return false;
  }

  prevIdx = 0;
  for (const HeightDiff& d : heightDiffs) {
    if (d.idx < prevIdx) {
      outError = "Internal error: height diff list not sorted";
      return false;
    }

    const std::uint32_t delta = d.idx - prevIdx;
    prevIdx = d.idx;

    if (!WriteVarU32(vw, delta)) {
      outError = "Write failed (height diff idx delta)";
      return false;
    }

    if (!vw.writePOD(d.heightQ)) {
      outError = "Write failed (height diff value)";
      return false;
    }
  }

  // Compress (SLLZ) when it helps.
  CompressionMethod method = CompressionMethod::None;
  std::vector<std::uint8_t> compressed;
  CompressSLLZ(deltaPayload.data(), deltaPayload.size(), compressed);
  const std::vector<std::uint8_t>* stored = &deltaPayload;
  if (!compressed.empty() && compressed.size() < deltaPayload.size()) {
    method = CompressionMethod::SLLZ;
    stored = &compressed;
  }

  if (deltaPayload.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) ||
      stored->size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "Delta payload too large";
    return false;
  }

  const std::uint8_t methodU8 = static_cast<std::uint8_t>(method);
  const std::uint32_t uncompressedSizeU32 = static_cast<std::uint32_t>(deltaPayload.size());
  const std::uint32_t storedSizeU32 = static_cast<std::uint32_t>(stored->size());

  if (!cw.write(methodU8)) {
    outError = "Write failed (compression method)";
    return false;
  }
  if (!WriteVarU32(cw, uncompressedSizeU32) || !WriteVarU32(cw, storedSizeU32)) {
    outError = "Write failed (compressed payload sizes)";
    return false;
  }
  if (storedSizeU32 > 0 && !cw.writeBytes(stored->data(), stored->size())) {
    outError = "Write failed (compressed payload bytes)";
    return false;
  }

  return true;
}

bool VerifyCrc32Bytes(const std::uint8_t* data, std::size_t size, bool& outOk, std::string& outError)
{
  outError.clear();
  outOk = true;

  if (!data || size < sizeof(std::uint32_t)) {
    outError = "Save buffer too small for CRC32";
    return false;
  }

  const std::size_t payloadSize = size - sizeof(std::uint32_t);
  std::uint32_t crc = 0xFFFFFFFFu;
  crc = Crc32Update(crc, data, payloadSize);

  std::uint32_t expected = 0;
  std::memcpy(&expected, data + payloadSize, sizeof(expected));

  const std::uint32_t computed = crc ^ 0xFFFFFFFFu;
  outOk = (computed == expected);
  return true;
}

class SpanStreamBuf : public std::streambuf {
public:
  SpanStreamBuf(const std::uint8_t* data, std::size_t size)
  {
    char* p = const_cast<char*>(reinterpret_cast<const char*>(data));
    setg(p, p, p + static_cast<std::ptrdiff_t>(size));
  }
};

} // namespace

bool SaveWorldBinary(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg, const std::string& path, std::string& outError)
{
  outError.clear();

  namespace fs = std::filesystem;

  if (path.empty()) {
    outError = "Save path is empty";
    return false;
  }

  const fs::path outPath(path);
  fs::path tmpPath = outPath;
  tmpPath += ".tmp";
  fs::path bakPath = outPath;
  bakPath += ".bak";

  std::error_code ec;

  // Ensure the parent directory exists (if specified).
  const fs::path parent = outPath.parent_path();
  if (!parent.empty()) {
    fs::create_directories(parent, ec);
    if (ec) {
      outError = "Unable to create save directory: " + parent.string() + " (" + ec.message() + ")";
      return false;
    }
  }

  // Remove a stale temp file from a prior failed/crashed save.
  fs::remove(tmpPath, ec);

  std::ofstream f(tmpPath, std::ios::binary | std::ios::trunc);
  if (!f) {
    outError = "Unable to open file for writing: " + tmpPath.string();
    return false;
  }

  // v3+ computes a CRC32 over the whole file (excluding the final CRC field).
  Crc32OStreamWriter cw(f);

  if (!WriteWorldBinaryPayload(cw, world, procCfg, simCfg, outError)) {
    return false;
  }

  // CRC32 (v3) - appended at the end and NOT included in the CRC itself.
  const std::uint32_t crc = cw.finalize();
  if (!Write(f, crc)) {
    outError = "Write failed (crc)";
    return false;
  }

  // Make sure all bytes hit disk before we swap the temp file into place.
  f.flush();
  if (!f.good()) {
    outError = "Write failed (flush)";
    return false;
  }
  f.close();

  // Atomically replace the destination:
  //  - move existing -> .bak
  //  - move tmp -> destination
  //  - cleanup .bak on success
  fs::remove(bakPath, ec);

  if (fs::exists(outPath, ec)) {
    fs::rename(outPath, bakPath, ec);
    if (ec) {
      outError = "Unable to backup existing save: " + outPath.string() + " (" + ec.message() + ")";
      return false;
    }
  }

  fs::rename(tmpPath, outPath, ec);
  if (ec) {
    // Best-effort rollback: restore the previous save if it exists.
    std::error_code ec2;
    if (fs::exists(bakPath, ec2)) {
      fs::rename(bakPath, outPath, ec2);
    }
    outError = "Unable to move temp save into place: " + outPath.string() + " (" + ec.message() + ")";
    return false;
  }

  // Best-effort cleanup of the backup file.
  fs::remove(bakPath, ec);

  return true;
}



bool SaveWorldBinary(const World& world, const ProcGenConfig& procCfg, const std::string& path, std::string& outError)
{
  // Back-compat helper: use the default sim config (v6+ stores SimConfig).
  return SaveWorldBinary(world, procCfg, SimConfig{}, path, outError);
}

bool SaveWorldBinary(const World& world, const std::string& path, std::string& outError)
{
  // Back-compat helper: use the default procgen + sim configs.
  return SaveWorldBinary(world, ProcGenConfig{}, SimConfig{}, path, outError);
}

bool SaveWorldBinaryToBytes(const World& world, const ProcGenConfig& procCfg, const SimConfig& simCfg,
                            std::vector<std::uint8_t>& outBytes, std::string& outError)
{
  outError.clear();
  outBytes.clear();

  Crc32VecWriter cw(outBytes);
  if (!WriteWorldBinaryPayload(cw, world, procCfg, simCfg, outError)) {
    outBytes.clear();
    return false;
  }

  const std::uint32_t crc = cw.finalize();
  AppendPOD(outBytes, crc);
  return true;
}

bool SaveWorldBinaryToBytes(const World& world, const ProcGenConfig& procCfg, std::vector<std::uint8_t>& outBytes,
                            std::string& outError)
{
  return SaveWorldBinaryToBytes(world, procCfg, SimConfig{}, outBytes, outError);
}

bool SaveWorldBinaryToBytes(const World& world, std::vector<std::uint8_t>& outBytes, std::string& outError)
{
  return SaveWorldBinaryToBytes(world, ProcGenConfig{}, SimConfig{}, outBytes, outError);
}

bool LoadWorldBinary(World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg, const std::string& path, std::string& outError)
{
  outError.clear();
  outSimCfg = SimConfig{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Unable to open file for reading: " + path;
    return false;
  }

  std::uint32_t version = 0;
  std::uint32_t w = 0;
  std::uint32_t h = 0;
  std::uint64_t seed = 0;

  if (!ReadAndValidateHeader(f, version, w, h, seed, outError)) {
    return false;
  }

  if (version == kVersionV1) {
    outSimCfg = SimConfig{};
    return LoadBodyV1(f, w, h, seed, outWorld, outProcCfg, outError);
  }

  if (version == kVersionV2) {
    outSimCfg = SimConfig{};
    return LoadBodyV2(f, w, h, seed, outWorld, outProcCfg, outError);
  }

  if (version == kVersionV3 || version == kVersionV4 || version == kVersionV5 || version == kVersionV6 ||
      version == kVersionV7 || version == kVersionV8 || version == kVersionV9 || version == kVersionV10 ||
      version == kVersionV11 || version == kVersionV12) {
    // v3+ saves append a CRC32 at the end of the file.
    //
    // We validate the CRC before parsing to detect corruption/truncation.
    //
    // Implementation note:
    //   We do a streaming CRC pass over the on-disk file (no full file buffering),
    //   then parse the body from the already-open ifstream. This is memory-friendly
    //   for large saves at the cost of an extra read pass.
    bool crcOk = true;
    std::string crcErr;
    if (!VerifyCrc32File(path, crcOk, crcErr)) {
      outError = crcErr;
      return false;
    }
    if (!crcOk) {
      outError = "Save file CRC mismatch (file is corrupted or incomplete)";
      return false;
    }

    if (version == kVersionV3) {
      outSimCfg = SimConfig{};
      return LoadBodyV2(f, w, h, seed, outWorld, outProcCfg, outError);
    }
    if (version == kVersionV4) {
      outSimCfg = SimConfig{};
      return LoadBodyV4(f, w, h, seed, outWorld, outProcCfg, outError);
    }
    if (version == kVersionV5) {
      // v5 saves did not include SimConfig.
      outSimCfg = SimConfig{};
      return LoadBodyV5(f, w, h, seed, outWorld, outProcCfg, outError);
    }
    if (version == kVersionV6) {
      return LoadBodyV6(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }
    if (version == kVersionV7) {
      // v7: includes per-tile districts + district policy multipliers.
      return LoadBodyV7(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV8) {
      // v8: same as v7 but with a compressed delta payload.
      return LoadBodyV8(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV9) {
      // v9: v8 + ProcGen erosion config.
      return LoadBodyV9(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV10) {
      // v10: v9 + ProcGen terrain preset config.
      return LoadBodyV10(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV11) {
      // v11: v10 + ProcGen road hierarchy config.
      return LoadBodyV11(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    // v12: v11 + ProcGen districting mode config.
    return LoadBodyV12(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
  }

  std::ostringstream oss;
  oss << "Unsupported save version: " << version << " (supported: " << kVersionV1 << ", " << kVersionV2 << ", "
      << kVersionV3 << ", " << kVersionV4 << ", " << kVersionV5 << ", " << kVersionV6 << ", "
      << kVersionV7 << ", " << kVersionV8 << ", " << kVersionV9 << ", " << kVersionV10 << ", "
      << kVersionV11 << ", " << kVersionV12 << ")";
  outError = oss.str();
  return false;
}

bool LoadWorldBinary(World& outWorld, ProcGenConfig& outProcCfg, const std::string& path, std::string& outError)
{
  SimConfig cfg;
  return LoadWorldBinary(outWorld, outProcCfg, cfg, path, outError);
}

bool LoadWorldBinary(World& outWorld, const std::string& path, std::string& outError)
{
  ProcGenConfig cfg;
  SimConfig simCfg;
  return LoadWorldBinary(outWorld, cfg, simCfg, path, outError);
}


bool LoadWorldBinaryFromBytes(World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg, const std::uint8_t* data,
                              std::size_t size, std::string& outError)
{
  outError.clear();
  outSimCfg = SimConfig{};

  if (!data || size == 0) {
    outError = "Save buffer is empty";
    return false;
  }

  SpanStreamBuf buf(data, size);
  std::istream f(&buf);

  std::uint32_t version = 0;
  std::uint32_t w = 0;
  std::uint32_t h = 0;
  std::uint64_t seed = 0;

  if (!ReadAndValidateHeader(f, version, w, h, seed, outError)) {
    return false;
  }

  if (version == kVersionV1) {
    outSimCfg = SimConfig{};
    return LoadBodyV1(f, w, h, seed, outWorld, outProcCfg, outError);
  }

  if (version == kVersionV2) {
    outSimCfg = SimConfig{};
    return LoadBodyV2(f, w, h, seed, outWorld, outProcCfg, outError);
  }

  if (version == kVersionV3 || version == kVersionV4 || version == kVersionV5 || version == kVersionV6 ||
      version == kVersionV7 || version == kVersionV8 || version == kVersionV9 || version == kVersionV10 ||
      version == kVersionV11 || version == kVersionV12) {
    // v3+ saves append a CRC32 at the end of the file.
    //
    // For in-memory loads we validate the CRC over the buffer (excluding the final CRC field).
    bool crcOk = true;
    std::string crcErr;
    if (!VerifyCrc32Bytes(data, size, crcOk, crcErr)) {
      outError = crcErr;
      return false;
    }
    if (!crcOk) {
      outError = "Save file CRC mismatch (buffer is corrupted or incomplete)";
      return false;
    }

    if (version == kVersionV3) {
      outSimCfg = SimConfig{};
      return LoadBodyV2(f, w, h, seed, outWorld, outProcCfg, outError);
    }
    if (version == kVersionV4) {
      outSimCfg = SimConfig{};
      return LoadBodyV4(f, w, h, seed, outWorld, outProcCfg, outError);
    }
    if (version == kVersionV5) {
      // v5 saves did not include SimConfig.
      outSimCfg = SimConfig{};
      return LoadBodyV5(f, w, h, seed, outWorld, outProcCfg, outError);
    }
    if (version == kVersionV6) {
      return LoadBodyV6(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }
    if (version == kVersionV7) {
      // v7: includes per-tile districts + district policy multipliers.
      return LoadBodyV7(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV8) {
      // v8: same as v7 but with a compressed delta payload.
      return LoadBodyV8(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV9) {
      // v9: v8 + ProcGen erosion config.
      return LoadBodyV9(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV10) {
      // v10: v9 + ProcGen terrain preset config.
      return LoadBodyV10(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    if (version == kVersionV11) {
      // v11: v10 + ProcGen road hierarchy config.
      return LoadBodyV11(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
    }

    // v12: v11 + ProcGen districting mode config.
    return LoadBodyV12(f, w, h, seed, outWorld, outProcCfg, outSimCfg, outError);
  }

  std::ostringstream oss;
  oss << "Unsupported save version: " << version << " (supported: " << kVersionV1 << ", " << kVersionV2 << ", "
      << kVersionV3 << ", " << kVersionV4 << ", " << kVersionV5 << ", " << kVersionV6 << ", "
      << kVersionV7 << ", " << kVersionV8 << ", " << kVersionV9 << ", " << kVersionV10 << ", "
      << kVersionV11 << ", " << kVersionV12 << ")";
  outError = oss.str();
  return false;
}

} // namespace isocity
