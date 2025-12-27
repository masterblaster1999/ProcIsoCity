#include "isocity/SaveLoad.hpp"

#include "isocity/ProcGen.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
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
constexpr std::uint32_t kVersionCurrent = kVersionV3;

// --- CRC32 ---
// Used by v3 saves to detect corruption/truncation.
//
// Notes:
// - We compute CRC32 over the entire file contents *excluding* the final CRC field.
// - The CRC32 is stored as a uint32_t appended to the end of the file.
const std::uint32_t* Crc32Table()
{
  static std::uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int bit = 0; bit < 8; ++bit) {
        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      }
      table[i] = c;
    }
    init = true;
  }
  return table;
}

std::uint32_t Crc32Update(std::uint32_t crc, const std::uint8_t* data, std::size_t size)
{
  const std::uint32_t* table = Crc32Table();
  for (std::size_t i = 0; i < size; ++i) {
    crc = table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
  }
  return crc;
}

std::uint32_t Crc32(const std::uint8_t* data, std::size_t size)
{
  std::uint32_t crc = 0xFFFFFFFFu;
  crc = Crc32Update(crc, data, size);
  return crc ^ 0xFFFFFFFFu;
}

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

void FromBin(ProcGenConfig& cfg, const ProcGenConfigBin& b)
{
  cfg.terrainScale = b.terrainScale;
  cfg.waterLevel = b.waterLevel;
  cfg.sandLevel = b.sandLevel;
  cfg.hubs = static_cast<int>(b.hubs);
  cfg.extraConnections = static_cast<int>(b.extraConnections);
  cfg.zoneChance = b.zoneChance;
  cfg.parkChance = b.parkChance;
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

    const bool isZone = (t.overlay == Overlay::Residential || t.overlay == Overlay::Commercial || t.overlay == Overlay::Industrial);

    if (isZone) {
      t.level = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(level), 1, 3));
      t.occupants = occ;
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

} // namespace

bool SaveWorldBinary(const World& world, const ProcGenConfig& procCfg, const std::string& path, std::string& outError)
{
  outError.clear();

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Unable to open file for writing: " + path;
    return false;
  }

  // v3+ computes a CRC32 over the whole file (excluding the final CRC field).
  Crc32OStreamWriter cw(f);

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
  const ProcGenConfigBin pcb = ToBin(procCfg);
  if (!cw.write(pcb)) {
    outError = "Write failed (procgen config)";
    return false;
  }

  // Stats
  const StatsBin sb = ToBin(world.stats());
  if (!cw.write(sb)) {
    outError = "Write failed (stats)";
    return false;
  }

  // --- Tile deltas ---
  // We store only the tiles whose mutable fields differ from a regenerated baseline.
  // This keeps save files small while still being deterministic.
  const World baseline = GenerateWorld(world.width(), world.height(), seed, procCfg);

  struct Diff {
    std::uint32_t idx = 0;
    std::uint8_t overlay = 0;
    std::uint8_t level = 1;
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
      if (cur.overlay != base.overlay || cur.level != base.level || cur.occupants != base.occupants) {
        const std::uint32_t idx = static_cast<std::uint32_t>(y * world.width() + x);
        diffs.push_back(Diff{idx, static_cast<std::uint8_t>(cur.overlay), cur.level, cur.occupants});
      }
    }
  }

  const std::uint32_t diffCount = static_cast<std::uint32_t>(diffs.size());
  if (!cw.write(diffCount)) {
    outError = "Write failed (diff count)";
    return false;
  }

  for (const Diff& d : diffs) {
    if (!cw.write(d.idx)) {
      outError = "Write failed (diff idx)";
      return false;
    }

    // Write in the same order we read.
    if (!cw.write(d.overlay) || !cw.write(d.level) || !cw.write(d.occupants)) {
      outError = "Write failed (diff tile)";
      return false;
    }
  }

  // CRC32 (v3) - appended at the end and NOT included in the CRC itself.
  const std::uint32_t crc = cw.finalize();
  if (!Write(f, crc)) {
    outError = "Write failed (crc)";
    return false;
  }

  return true;
}

bool SaveWorldBinary(const World& world, const std::string& path, std::string& outError)
{
  // Back-compat helper: use the default procgen config.
  return SaveWorldBinary(world, ProcGenConfig{}, path, outError);
}

bool LoadWorldBinary(World& outWorld, ProcGenConfig& outProcCfg, const std::string& path, std::string& outError)
{
  outError.clear();

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
    return LoadBodyV1(f, w, h, seed, outWorld, outProcCfg, outError);
  }

  if (version == kVersionV2) {
    return LoadBodyV2(f, w, h, seed, outWorld, outProcCfg, outError);
  }

  if (version == kVersionV3) {
    // v3: same payload as v2, but with a CRC32 appended at the end of the file.
    // We validate the CRC before parsing to detect corruption/truncation.

    // Read the whole file into memory to verify CRC.
    f.clear();
    f.seekg(0, std::ios::end);
    const std::streamoff fileSize = f.tellg();
    if (fileSize <= 0) {
      outError = "Read failed (file size)";
      return false;
    }
    if (fileSize < static_cast<std::streamoff>(sizeof(std::uint32_t))) {
      outError = "Read failed (crc)";
      return false;
    }
    if (static_cast<std::uint64_t>(fileSize) > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
      outError = "Save file too large";
      return false;
    }

    const std::size_t sizeU = static_cast<std::size_t>(fileSize);
    std::vector<std::uint8_t> bytes(sizeU);

    f.seekg(0, std::ios::beg);
    f.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(sizeU));
    if (!f.good()) {
      outError = "Read failed (file)";
      return false;
    }

    std::uint32_t storedCrc = 0;
    std::memcpy(&storedCrc, bytes.data() + (sizeU - sizeof(std::uint32_t)), sizeof(std::uint32_t));

    const std::uint32_t calcCrc = Crc32(bytes.data(), sizeU - sizeof(std::uint32_t));
    if (storedCrc != calcCrc) {
      outError = "Save file CRC mismatch (file is corrupted or incomplete)";
      return false;
    }

    // Parse the CRC-verified payload (excluding the final CRC field).
    const std::size_t payloadSize = sizeU - sizeof(std::uint32_t);
    std::string payload(reinterpret_cast<const char*>(bytes.data()), payloadSize);
    std::istringstream is(payload, std::ios::binary);

    std::uint32_t v2 = 0;
    std::uint32_t w2 = 0;
    std::uint32_t h2 = 0;
    std::uint64_t seed2 = 0;
    if (!ReadAndValidateHeader(is, v2, w2, h2, seed2, outError)) {
      return false;
    }
    if (v2 != kVersionV3) {
      outError = "Save file version mismatch after CRC validation";
      return false;
    }

    return LoadBodyV2(is, w2, h2, seed2, outWorld, outProcCfg, outError);
  }

  std::ostringstream oss;
  oss << "Unsupported save version: " << version << " (supported: " << kVersionV1 << ", " << kVersionV2 << ", "
      << kVersionV3 << ")";
  outError = oss.str();
  return false;
}

bool LoadWorldBinary(World& outWorld, const std::string& path, std::string& outError)
{
  ProcGenConfig cfg;
  return LoadWorldBinary(outWorld, cfg, path, outError);
}

} // namespace isocity
