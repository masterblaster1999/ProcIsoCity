#include "isocity/WorldPatch.hpp"

#include "isocity/Compression.hpp"
#include "isocity/Hash.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>

namespace isocity {

namespace {

// --- Tiny binary IO helpers (little-endian, float-as-bits) ---

struct ByteWriter {
  std::vector<std::uint8_t> out;

  bool writeBytes(const void* data, std::size_t n)
  {
    if (n == 0) return true;
    const auto* p = reinterpret_cast<const std::uint8_t*>(data);
    out.insert(out.end(), p, p + n);
    return true;
  }

  bool writeU8(std::uint8_t v) { return writeBytes(&v, 1); }

  bool writeU16(std::uint16_t v)
  {
    const std::uint8_t b[2] = {
        static_cast<std::uint8_t>((v >> 0) & 0xFFu),
        static_cast<std::uint8_t>((v >> 8) & 0xFFu),
    };
    return writeBytes(b, 2);
  }

  bool writeU32(std::uint32_t v)
  {
    const std::uint8_t b[4] = {
        static_cast<std::uint8_t>((v >> 0) & 0xFFu),
        static_cast<std::uint8_t>((v >> 8) & 0xFFu),
        static_cast<std::uint8_t>((v >> 16) & 0xFFu),
        static_cast<std::uint8_t>((v >> 24) & 0xFFu),
    };
    return writeBytes(b, 4);
  }

  bool writeI32(std::int32_t v)
  {
    std::uint32_t uv = 0;
    static_assert(sizeof(uv) == sizeof(v), "i32/u32 size mismatch");
    std::memcpy(&uv, &v, sizeof(uv));
    return writeU32(uv);
  }

  bool writeU64(std::uint64_t v)
  {
    const std::uint8_t b[8] = {
        static_cast<std::uint8_t>((v >> 0) & 0xFFull),
        static_cast<std::uint8_t>((v >> 8) & 0xFFull),
        static_cast<std::uint8_t>((v >> 16) & 0xFFull),
        static_cast<std::uint8_t>((v >> 24) & 0xFFull),
        static_cast<std::uint8_t>((v >> 32) & 0xFFull),
        static_cast<std::uint8_t>((v >> 40) & 0xFFull),
        static_cast<std::uint8_t>((v >> 48) & 0xFFull),
        static_cast<std::uint8_t>((v >> 56) & 0xFFull),
    };
    return writeBytes(b, 8);
  }

  bool writeF32(float v)
  {
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    return writeU32(bits);
  }

  // Unsigned LEB128 varint.
  bool writeVarU32(std::uint32_t v)
  {
    std::uint8_t buf[5];
    std::size_t n = 0;
    while (v >= 0x80u) {
      buf[n++] = static_cast<std::uint8_t>((v & 0x7Fu) | 0x80u);
      v >>= 7;
    }
    buf[n++] = static_cast<std::uint8_t>(v & 0x7Fu);
    return writeBytes(buf, n);
  }
};

struct ByteReader {
  const std::uint8_t* data = nullptr;
  std::size_t size = 0;
  std::size_t pos = 0;

  bool readBytes(void* out, std::size_t n)
  {
    if (n == 0) return true;
    if (!out) return false;
    if (pos + n > size) return false;
    std::memcpy(out, data + pos, n);
    pos += n;
    return true;
  }

  bool readU8(std::uint8_t& out) { return readBytes(&out, 1); }

  bool readU16(std::uint16_t& out)
  {
    std::uint8_t b[2];
    if (!readBytes(b, 2)) return false;
    out = static_cast<std::uint16_t>(static_cast<std::uint16_t>(b[0]) | (static_cast<std::uint16_t>(b[1]) << 8));
    return true;
  }

  bool readU32(std::uint32_t& out)
  {
    std::uint8_t b[4];
    if (!readBytes(b, 4)) return false;
    out = (static_cast<std::uint32_t>(b[0]) << 0) |
          (static_cast<std::uint32_t>(b[1]) << 8) |
          (static_cast<std::uint32_t>(b[2]) << 16) |
          (static_cast<std::uint32_t>(b[3]) << 24);
    return true;
  }

  bool readI32(std::int32_t& out)
  {
    std::uint32_t uv = 0;
    if (!readU32(uv)) return false;
    static_assert(sizeof(uv) == sizeof(out), "i32/u32 size mismatch");
    std::memcpy(&out, &uv, sizeof(out));
    return true;
  }

  bool readU64(std::uint64_t& out)
  {
    std::uint8_t b[8];
    if (!readBytes(b, 8)) return false;
    out = (static_cast<std::uint64_t>(b[0]) << 0) |
          (static_cast<std::uint64_t>(b[1]) << 8) |
          (static_cast<std::uint64_t>(b[2]) << 16) |
          (static_cast<std::uint64_t>(b[3]) << 24) |
          (static_cast<std::uint64_t>(b[4]) << 32) |
          (static_cast<std::uint64_t>(b[5]) << 40) |
          (static_cast<std::uint64_t>(b[6]) << 48) |
          (static_cast<std::uint64_t>(b[7]) << 56);
    return true;
  }

  bool readF32(float& out)
  {
    std::uint32_t bits = 0;
    if (!readU32(bits)) return false;
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::memcpy(&out, &bits, sizeof(out));
    return true;
  }

  bool readVarU32(std::uint32_t& out)
  {
    out = 0;
    std::uint32_t shift = 0;
    for (int i = 0; i < 5; ++i) {
      std::uint8_t byte = 0;
      if (!readU8(byte)) return false;
      out |= static_cast<std::uint32_t>(byte & 0x7Fu) << shift;
      if ((byte & 0x80u) == 0) return true;
      shift += 7;
    }
    return false;
  }
};

inline bool MaskHas(std::uint8_t mask, TileFieldMask bit)
{
  return (mask & static_cast<std::uint8_t>(bit)) != 0;
}

bool WriteProcCfg(ByteWriter& w, const ProcGenConfig& cfg, std::uint32_t patchVersion)
{
  if (!(w.writeF32(cfg.terrainScale) &&
        w.writeF32(cfg.waterLevel) &&
        w.writeF32(cfg.sandLevel) &&
        w.writeI32(static_cast<std::int32_t>(cfg.hubs)) &&
        w.writeI32(static_cast<std::int32_t>(cfg.extraConnections)) &&
        w.writeF32(cfg.zoneChance) &&
        w.writeF32(cfg.parkChance))) {
    return false;
  }

  // v2+: include erosion settings so delta saves/patches can deterministically regenerate the same terrain.
  if (patchVersion >= 2) {
    if (!w.writeU8(static_cast<std::uint8_t>(cfg.erosion.enabled ? 1 : 0))) return false;
    if (!w.writeU8(static_cast<std::uint8_t>(cfg.erosion.riversEnabled ? 1 : 0))) return false;

    if (!w.writeI32(static_cast<std::int32_t>(cfg.erosion.thermalIterations))) return false;
    if (!w.writeF32(cfg.erosion.thermalTalus)) return false;
    if (!w.writeF32(cfg.erosion.thermalRate)) return false;

    if (!w.writeI32(static_cast<std::int32_t>(cfg.erosion.riverMinAccum))) return false;
    if (!w.writeF32(cfg.erosion.riverCarve)) return false;
    if (!w.writeF32(cfg.erosion.riverCarvePower)) return false;

    if (!w.writeI32(static_cast<std::int32_t>(cfg.erosion.smoothIterations))) return false;
    if (!w.writeF32(cfg.erosion.smoothRate)) return false;

    if (!w.writeI32(static_cast<std::int32_t>(cfg.erosion.quantizeScale))) return false;
  }

  return true;
}

bool ReadProcCfg(ByteReader& r, ProcGenConfig& cfg, std::uint32_t patchVersion)
{
  std::int32_t hubs = 0;
  std::int32_t extra = 0;
  if (!r.readF32(cfg.terrainScale)) return false;
  if (!r.readF32(cfg.waterLevel)) return false;
  if (!r.readF32(cfg.sandLevel)) return false;
  if (!r.readI32(hubs)) return false;
  if (!r.readI32(extra)) return false;
  cfg.hubs = static_cast<int>(hubs);
  cfg.extraConnections = static_cast<int>(extra);
  if (!r.readF32(cfg.zoneChance)) return false;
  if (!r.readF32(cfg.parkChance)) return false;

  if (patchVersion >= 2) {
    std::uint8_t erosionEnabled = 0;
    std::uint8_t riversEnabled = 0;
    if (!r.readU8(erosionEnabled)) return false;
    if (!r.readU8(riversEnabled)) return false;
    cfg.erosion.enabled = (erosionEnabled != 0);
    cfg.erosion.riversEnabled = (riversEnabled != 0);

    std::int32_t ti = 0;
    std::int32_t rma = 0;
    std::int32_t si = 0;
    std::int32_t qs = 0;
    if (!r.readI32(ti)) return false;
    cfg.erosion.thermalIterations = static_cast<int>(ti);
    if (!r.readF32(cfg.erosion.thermalTalus)) return false;
    if (!r.readF32(cfg.erosion.thermalRate)) return false;

    if (!r.readI32(rma)) return false;
    cfg.erosion.riverMinAccum = static_cast<int>(rma);
    if (!r.readF32(cfg.erosion.riverCarve)) return false;
    if (!r.readF32(cfg.erosion.riverCarvePower)) return false;

    if (!r.readI32(si)) return false;
    cfg.erosion.smoothIterations = static_cast<int>(si);
    if (!r.readF32(cfg.erosion.smoothRate)) return false;

    if (!r.readI32(qs)) return false;
    cfg.erosion.quantizeScale = static_cast<int>(qs);
  } else {
    // v1 patches predate erosion; preserve old behavior.
    cfg.erosion = ErosionConfig{};
    cfg.erosion.enabled = false;
  }

  return true;
}

bool WriteSimCfg(ByteWriter& w, const SimConfig& cfg)
{
  if (!w.writeF32(cfg.tickSeconds)) return false;
  if (!w.writeI32(static_cast<std::int32_t>(cfg.parkInfluenceRadius))) return false;
  if (!w.writeU8(static_cast<std::uint8_t>(cfg.requireOutsideConnection ? 1 : 0))) return false;

  if (!w.writeI32(static_cast<std::int32_t>(cfg.taxResidential))) return false;
  if (!w.writeI32(static_cast<std::int32_t>(cfg.taxCommercial))) return false;
  if (!w.writeI32(static_cast<std::int32_t>(cfg.taxIndustrial))) return false;
  if (!w.writeI32(static_cast<std::int32_t>(cfg.maintenanceRoad))) return false;
  if (!w.writeI32(static_cast<std::int32_t>(cfg.maintenancePark))) return false;
  if (!w.writeF32(cfg.taxHappinessPerCapita)) return false;

  if (!w.writeF32(cfg.residentialDesirabilityWeight)) return false;
  if (!w.writeF32(cfg.commercialDesirabilityWeight)) return false;
  if (!w.writeF32(cfg.industrialDesirabilityWeight)) return false;

  if (!w.writeU8(static_cast<std::uint8_t>(cfg.districtPoliciesEnabled ? 1 : 0))) return false;

  for (int i = 0; i < kDistrictCount; ++i) {
    const DistrictPolicy& p = cfg.districtPolicies[static_cast<std::size_t>(i)];
    if (!w.writeF32(p.taxResidentialMult)) return false;
    if (!w.writeF32(p.taxCommercialMult)) return false;
    if (!w.writeF32(p.taxIndustrialMult)) return false;
    if (!w.writeF32(p.roadMaintenanceMult)) return false;
    if (!w.writeF32(p.parkMaintenanceMult)) return false;
  }
  return true;
}

bool ReadSimCfg(ByteReader& r, SimConfig& cfg)
{
  std::uint8_t b = 0;
  std::int32_t i32 = 0;

  if (!r.readF32(cfg.tickSeconds)) return false;
  if (!r.readI32(i32)) return false;
  cfg.parkInfluenceRadius = static_cast<int>(i32);
  if (!r.readU8(b)) return false;
  cfg.requireOutsideConnection = (b != 0);

  if (!r.readI32(i32)) return false;
  cfg.taxResidential = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  cfg.taxCommercial = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  cfg.taxIndustrial = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  cfg.maintenanceRoad = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  cfg.maintenancePark = static_cast<int>(i32);
  if (!r.readF32(cfg.taxHappinessPerCapita)) return false;

  if (!r.readF32(cfg.residentialDesirabilityWeight)) return false;
  if (!r.readF32(cfg.commercialDesirabilityWeight)) return false;
  if (!r.readF32(cfg.industrialDesirabilityWeight)) return false;

  if (!r.readU8(b)) return false;
  cfg.districtPoliciesEnabled = (b != 0);

  for (int i = 0; i < kDistrictCount; ++i) {
    DistrictPolicy& p = cfg.districtPolicies[static_cast<std::size_t>(i)];
    if (!r.readF32(p.taxResidentialMult)) return false;
    if (!r.readF32(p.taxCommercialMult)) return false;
    if (!r.readF32(p.taxIndustrialMult)) return false;
    if (!r.readF32(p.roadMaintenanceMult)) return false;
    if (!r.readF32(p.parkMaintenanceMult)) return false;
  }
  return true;
}

bool WriteStats(ByteWriter& w, const Stats& s)
{
  // Keep in sync with ReadStats.
  return w.writeI32(static_cast<std::int32_t>(s.day)) &&
         w.writeI32(static_cast<std::int32_t>(s.population)) &&
         w.writeI32(static_cast<std::int32_t>(s.housingCapacity)) &&
         w.writeI32(static_cast<std::int32_t>(s.jobsCapacity)) &&
         w.writeI32(static_cast<std::int32_t>(s.jobsCapacityAccessible)) &&
         w.writeI32(static_cast<std::int32_t>(s.employed)) &&
         w.writeF32(s.happiness) &&
         w.writeI32(static_cast<std::int32_t>(s.money)) &&
         w.writeI32(static_cast<std::int32_t>(s.roads)) &&
         w.writeI32(static_cast<std::int32_t>(s.parks)) &&
         // Traffic
         w.writeI32(static_cast<std::int32_t>(s.commuters)) &&
         w.writeI32(static_cast<std::int32_t>(s.commutersUnreachable)) &&
         w.writeF32(s.avgCommute) &&
         w.writeF32(s.p95Commute) &&
         w.writeF32(s.avgCommuteTime) &&
         w.writeF32(s.p95CommuteTime) &&
         w.writeF32(s.trafficCongestion) &&
         w.writeI32(static_cast<std::int32_t>(s.congestedRoadTiles)) &&
         w.writeI32(static_cast<std::int32_t>(s.maxRoadTraffic)) &&
         // Goods
         w.writeI32(static_cast<std::int32_t>(s.goodsProduced)) &&
         w.writeI32(static_cast<std::int32_t>(s.goodsDemand)) &&
         w.writeI32(static_cast<std::int32_t>(s.goodsDelivered)) &&
         w.writeI32(static_cast<std::int32_t>(s.goodsImported)) &&
         w.writeI32(static_cast<std::int32_t>(s.goodsExported)) &&
         w.writeI32(static_cast<std::int32_t>(s.goodsUnreachableDemand)) &&
         w.writeF32(s.goodsSatisfaction) &&
         w.writeI32(static_cast<std::int32_t>(s.maxRoadGoodsTraffic)) &&
         // Economy snapshot
         w.writeI32(static_cast<std::int32_t>(s.income)) &&
         w.writeI32(static_cast<std::int32_t>(s.expenses)) &&
         w.writeI32(static_cast<std::int32_t>(s.taxRevenue)) &&
         w.writeI32(static_cast<std::int32_t>(s.maintenanceCost)) &&
         w.writeI32(static_cast<std::int32_t>(s.upgradeCost)) &&
         w.writeI32(static_cast<std::int32_t>(s.importCost)) &&
         w.writeI32(static_cast<std::int32_t>(s.exportRevenue)) &&
         w.writeF32(s.avgTaxPerCapita) &&
         // Demand/valuation
         w.writeF32(s.demandResidential) &&
         w.writeF32(s.avgLandValue);
}

bool ReadStats(ByteReader& r, Stats& s)
{
  // Keep in sync with WriteStats.
  std::int32_t i32 = 0;
  if (!r.readI32(i32)) return false;
  s.day = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.population = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.housingCapacity = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.jobsCapacity = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.jobsCapacityAccessible = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.employed = static_cast<int>(i32);
  if (!r.readF32(s.happiness)) return false;
  if (!r.readI32(i32)) return false;
  s.money = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.roads = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.parks = static_cast<int>(i32);

  if (!r.readI32(i32)) return false;
  s.commuters = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.commutersUnreachable = static_cast<int>(i32);
  if (!r.readF32(s.avgCommute)) return false;
  if (!r.readF32(s.p95Commute)) return false;
  if (!r.readF32(s.avgCommuteTime)) return false;
  if (!r.readF32(s.p95CommuteTime)) return false;
  if (!r.readF32(s.trafficCongestion)) return false;
  if (!r.readI32(i32)) return false;
  s.congestedRoadTiles = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.maxRoadTraffic = static_cast<int>(i32);

  if (!r.readI32(i32)) return false;
  s.goodsProduced = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.goodsDemand = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.goodsDelivered = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.goodsImported = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.goodsExported = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.goodsUnreachableDemand = static_cast<int>(i32);
  if (!r.readF32(s.goodsSatisfaction)) return false;
  if (!r.readI32(i32)) return false;
  s.maxRoadGoodsTraffic = static_cast<int>(i32);

  if (!r.readI32(i32)) return false;
  s.income = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.expenses = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.taxRevenue = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.maintenanceCost = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.upgradeCost = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.importCost = static_cast<int>(i32);
  if (!r.readI32(i32)) return false;
  s.exportRevenue = static_cast<int>(i32);
  if (!r.readF32(s.avgTaxPerCapita)) return false;

  if (!r.readF32(s.demandResidential)) return false;
  if (!r.readF32(s.avgLandValue)) return false;
  return true;
}

// Patch header (binary file)
//
// Layout (little-endian):
//   magic[8] = "ISOPATCH"
//   u32 version
//   u32 flags
//   i32 width
//   i32 height
//   u64 baseHash
//   u64 targetHash
//   u8  compressionMethod (WorldPatchCompression)
//   u32 payloadSize (uncompressed)
//   u32 payloadSizeCompressed
inline constexpr std::uint32_t kPatchVersion = 2;
inline constexpr std::uint8_t kMagic[8] = { 'I','S','O','P','A','T','C','H' };

enum PatchFlags : std::uint32_t {
  kFlagProcCfg = 1u << 0,
  kFlagSimCfg  = 1u << 1,
  kFlagStats   = 1u << 2,
};

bool WriteHeader(std::ofstream& f, const WorldPatch& p, WorldPatchCompression comp,
                 std::uint32_t payloadSize, std::uint32_t payloadSizeCompressed)
{
  ByteWriter w;
  w.writeBytes(kMagic, sizeof(kMagic));
  w.writeU32(kPatchVersion);

  std::uint32_t flags = 0;
  if (p.includeProcCfg) flags |= kFlagProcCfg;
  if (p.includeSimCfg)  flags |= kFlagSimCfg;
  if (p.includeStats)   flags |= kFlagStats;
  w.writeU32(flags);

  w.writeI32(static_cast<std::int32_t>(p.width));
  w.writeI32(static_cast<std::int32_t>(p.height));
  w.writeU64(p.baseHash);
  w.writeU64(p.targetHash);
  w.writeU8(static_cast<std::uint8_t>(comp));
  w.writeU32(payloadSize);
  w.writeU32(payloadSizeCompressed);

  f.write(reinterpret_cast<const char*>(w.out.data()), static_cast<std::streamsize>(w.out.size()));
  return static_cast<bool>(f);
}

bool ReadHeader(std::ifstream& f, WorldPatch& outPatch, WorldPatchCompression& outComp,
                std::uint32_t& outPayloadSize, std::uint32_t& outPayloadSizeCompressed,
                std::string& outError)
{
  // Fixed header size.
  constexpr std::size_t kHeaderSize = 8 + 4 + 4 + 4 + 4 + 8 + 8 + 1 + 4 + 4;
  std::uint8_t buf[kHeaderSize];
  f.read(reinterpret_cast<char*>(buf), static_cast<std::streamsize>(kHeaderSize));
  if (f.gcount() != static_cast<std::streamsize>(kHeaderSize)) {
    outError = "Failed to read patch header";
    return false;
  }

  ByteReader r;
  r.data = buf;
  r.size = kHeaderSize;
  r.pos = 0;

  std::uint8_t magic[8];
  if (!r.readBytes(magic, sizeof(magic))) {
    outError = "Corrupt patch header (magic)";
    return false;
  }
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "Bad patch magic (not an ISOPATCH file)";
    return false;
  }

  std::uint32_t version = 0;
  if (!r.readU32(version)) {
    outError = "Corrupt patch header (version)";
    return false;
  }
  if (version < 1 || version > kPatchVersion) {
    outError = "Unsupported patch version";
    return false;
  }
  outPatch.version = version;

  std::uint32_t flags = 0;
  if (!r.readU32(flags)) {
    outError = "Corrupt patch header (flags)";
    return false;
  }

  std::int32_t w = 0;
  std::int32_t h = 0;
  if (!r.readI32(w) || !r.readI32(h)) {
    outError = "Corrupt patch header (size)";
    return false;
  }
  outPatch.width = static_cast<int>(w);
  outPatch.height = static_cast<int>(h);

  if (!r.readU64(outPatch.baseHash) || !r.readU64(outPatch.targetHash)) {
    outError = "Corrupt patch header (hashes)";
    return false;
  }

  std::uint8_t compByte = 0;
  if (!r.readU8(compByte)) {
    outError = "Corrupt patch header (compression)";
    return false;
  }
  outComp = static_cast<WorldPatchCompression>(compByte);
  if (outComp != WorldPatchCompression::None && outComp != WorldPatchCompression::SLLZ) {
    outError = "Unsupported patch compression";
    return false;
  }

  if (!r.readU32(outPayloadSize) || !r.readU32(outPayloadSizeCompressed)) {
    outError = "Corrupt patch header (payload sizes)";
    return false;
  }

  outPatch.includeProcCfg = (flags & kFlagProcCfg) != 0;
  outPatch.includeSimCfg = (flags & kFlagSimCfg) != 0;
  outPatch.includeStats = (flags & kFlagStats) != 0;

  // Basic sanity checks.
  if (outPatch.width <= 0 || outPatch.height <= 0) {
    outError = "Invalid patch dimensions";
    return false;
  }
  if (outPayloadSize > (1u << 30)) {
    outError = "Patch payload too large";
    return false;
  }
  if (outPayloadSizeCompressed > (1u << 30)) {
    outError = "Patch payload too large (compressed)";
    return false;
  }

  return true;
}

bool WriteTileDeltas(ByteWriter& w, const std::vector<WorldPatchTileDelta>& tiles)
{
  if (!w.writeVarU32(static_cast<std::uint32_t>(tiles.size()))) return false;

  std::uint32_t prev = 0;
  for (std::size_t i = 0; i < tiles.size(); ++i) {
    const WorldPatchTileDelta& d = tiles[i];
    if (d.index < prev) return false; // must be sorted
    const std::uint32_t delta = d.index - prev;
    prev = d.index;

    if (!w.writeVarU32(delta)) return false;
    if (!w.writeU8(d.mask)) return false;

    const Tile& t = d.value;
    if (MaskHas(d.mask, TileFieldMask::Terrain)) {
      if (!w.writeU8(static_cast<std::uint8_t>(t.terrain))) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Overlay)) {
      if (!w.writeU8(static_cast<std::uint8_t>(t.overlay))) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Height)) {
      if (!w.writeF32(t.height)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Variation)) {
      if (!w.writeU8(t.variation)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Level)) {
      if (!w.writeU8(t.level)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Occupants)) {
      if (!w.writeU16(t.occupants)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::District)) {
      if (!w.writeU8(t.district)) return false;
    }
  }

  return true;
}

bool ReadTileDeltas(ByteReader& r, std::vector<WorldPatchTileDelta>& out)
{
  std::uint32_t count = 0;
  if (!r.readVarU32(count)) return false;

  out.clear();
  out.reserve(count);

  std::uint32_t index = 0;
  for (std::uint32_t i = 0; i < count; ++i) {
    std::uint32_t delta = 0;
    if (!r.readVarU32(delta)) return false;
    if (delta > std::numeric_limits<std::uint32_t>::max() - index) return false;
    index += delta;

    WorldPatchTileDelta d;
    d.index = index;
    if (!r.readU8(d.mask)) return false;

    Tile t{};
    if (MaskHas(d.mask, TileFieldMask::Terrain)) {
      std::uint8_t v = 0;
      if (!r.readU8(v)) return false;
      t.terrain = static_cast<Terrain>(v);
    }
    if (MaskHas(d.mask, TileFieldMask::Overlay)) {
      std::uint8_t v = 0;
      if (!r.readU8(v)) return false;
      t.overlay = static_cast<Overlay>(v);
    }
    if (MaskHas(d.mask, TileFieldMask::Height)) {
      if (!r.readF32(t.height)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Variation)) {
      if (!r.readU8(t.variation)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Level)) {
      if (!r.readU8(t.level)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Occupants)) {
      if (!r.readU16(t.occupants)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::District)) {
      if (!r.readU8(t.district)) return false;
    }
    d.value = t;
    out.push_back(d);
  }

  return true;
}

} // namespace

bool MakeWorldPatch(const World& baseWorld, const ProcGenConfig& /*baseProcCfg*/, const SimConfig& /*baseSimCfg*/,
                    const World& targetWorld, const ProcGenConfig& targetProcCfg, const SimConfig& targetSimCfg,
                    WorldPatch& outPatch, std::string& outError,
                    bool includeProcCfg, bool includeSimCfg, bool includeStats)
{
  outError.clear();
  outPatch = {};

  if (baseWorld.width() != targetWorld.width() || baseWorld.height() != targetWorld.height()) {
    outError = "WorldPatch requires identical dimensions";
    return false;
  }
  if (baseWorld.seed() != targetWorld.seed()) {
    outError = "WorldPatch requires identical seeds";
    return false;
  }

  outPatch.width = baseWorld.width();
  outPatch.height = baseWorld.height();
  outPatch.includeProcCfg = includeProcCfg;
  outPatch.includeSimCfg = includeSimCfg;
  outPatch.includeStats = includeStats;

  if (includeProcCfg) outPatch.procCfg = targetProcCfg;
  if (includeSimCfg) outPatch.simCfg = targetSimCfg;
  if (includeStats) outPatch.stats = targetWorld.stats();

  outPatch.baseHash = HashWorld(baseWorld, includeStats);
  outPatch.targetHash = HashWorld(targetWorld, includeStats);

  // Compute per-tile field masks.
  outPatch.tiles.clear();
  outPatch.tiles.reserve(256);

  const int w = baseWorld.width();
  const int h = baseWorld.height();
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& a = baseWorld.at(x, y);
      const Tile& b = targetWorld.at(x, y);

      std::uint8_t mask = 0;
      if (a.terrain != b.terrain) mask |= static_cast<std::uint8_t>(TileFieldMask::Terrain);
      if (a.overlay != b.overlay) mask |= static_cast<std::uint8_t>(TileFieldMask::Overlay);
      if (a.height != b.height) mask |= static_cast<std::uint8_t>(TileFieldMask::Height);
      if (a.variation != b.variation) mask |= static_cast<std::uint8_t>(TileFieldMask::Variation);
      if (a.level != b.level) mask |= static_cast<std::uint8_t>(TileFieldMask::Level);
      if (a.occupants != b.occupants) mask |= static_cast<std::uint8_t>(TileFieldMask::Occupants);
      if (a.district != b.district) mask |= static_cast<std::uint8_t>(TileFieldMask::District);

      if (mask == 0) continue;

      WorldPatchTileDelta d;
      d.index = static_cast<std::uint32_t>(y * w + x);
      d.mask = mask;
      d.value = b;
      outPatch.tiles.push_back(d);
    }
  }

  // Ensure deterministic ordering.
  std::sort(outPatch.tiles.begin(), outPatch.tiles.end(), [](const WorldPatchTileDelta& a, const WorldPatchTileDelta& b) {
    return a.index < b.index;
  });

  return true;
}

bool ApplyWorldPatch(World& inOutWorld, ProcGenConfig& inOutProcCfg, SimConfig& inOutSimCfg, const WorldPatch& patch,
                     std::string& outError, bool force)
{
  outError.clear();

  if (inOutWorld.width() != patch.width || inOutWorld.height() != patch.height) {
    outError = "Patch dimension mismatch";
    return false;
  }

  const std::uint64_t baseHash = HashWorld(inOutWorld, patch.includeStats);
  if (!force && baseHash != patch.baseHash) {
    outError = "Base hash mismatch (refusing to apply patch without --force)";
    return false;
  }

  if (patch.includeProcCfg) inOutProcCfg = patch.procCfg;
  if (patch.includeSimCfg) inOutSimCfg = patch.simCfg;
  if (patch.includeStats) inOutWorld.stats() = patch.stats;

  const int w = inOutWorld.width();

  for (const WorldPatchTileDelta& d : patch.tiles) {
    const std::uint32_t idx = d.index;
    const int x = static_cast<int>(idx % static_cast<std::uint32_t>(w));
    const int y = static_cast<int>(idx / static_cast<std::uint32_t>(w));
    if (!inOutWorld.inBounds(x, y)) {
      outError = "Patch contains out-of-bounds tile index";
      return false;
    }

    Tile& t = inOutWorld.at(x, y);
    const Tile& v = d.value;

    if (MaskHas(d.mask, TileFieldMask::Terrain)) t.terrain = v.terrain;
    if (MaskHas(d.mask, TileFieldMask::Overlay)) t.overlay = v.overlay;
    if (MaskHas(d.mask, TileFieldMask::Height)) t.height = v.height;
    if (MaskHas(d.mask, TileFieldMask::Variation)) t.variation = v.variation;
    if (MaskHas(d.mask, TileFieldMask::Level)) t.level = v.level;
    if (MaskHas(d.mask, TileFieldMask::Occupants)) t.occupants = v.occupants;
    if (MaskHas(d.mask, TileFieldMask::District)) t.district = v.district;
  }

  const std::uint64_t finalHash = HashWorld(inOutWorld, patch.includeStats);
  if (finalHash != patch.targetHash) {
    outError = "Patched world hash mismatch (patch corrupt or incompatible build)";
    return false;
  }

  return true;
}

bool SaveWorldPatchBinary(const WorldPatch& patch, const std::string& path, std::string& outError,
                          WorldPatchCompression compression)
{
  outError.clear();

  if (patch.width <= 0 || patch.height <= 0) {
    outError = "Invalid patch dimensions";
    return false;
  }
  if (path.empty()) {
    outError = "Empty output path";
    return false;
  }

  // Build payload.
  ByteWriter payload;
  if (patch.includeProcCfg) {
    if (!WriteProcCfg(payload, patch.procCfg, kPatchVersion)) {
      outError = "Failed to write ProcGenConfig";
      return false;
    }
  }
  if (patch.includeSimCfg) {
    if (!WriteSimCfg(payload, patch.simCfg)) {
      outError = "Failed to write SimConfig";
      return false;
    }
  }
  if (patch.includeStats) {
    if (!WriteStats(payload, patch.stats)) {
      outError = "Failed to write Stats";
      return false;
    }
  }
  if (!WriteTileDeltas(payload, patch.tiles)) {
    outError = "Failed to write tile deltas";
    return false;
  }

  const std::uint32_t payloadSize = static_cast<std::uint32_t>(payload.out.size());

  std::vector<std::uint8_t> payloadCompressed;
  if (compression == WorldPatchCompression::SLLZ) {
    if (!CompressSLLZ(payload.out.data(), payload.out.size(), payloadCompressed)) {
      outError = "Compression failed";
      return false;
    }
  } else {
    payloadCompressed = payload.out;
  }

  const std::uint32_t payloadSizeCompressed = static_cast<std::uint32_t>(payloadCompressed.size());

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open output patch file";
    return false;
  }

  if (!WriteHeader(f, patch, compression, payloadSize, payloadSizeCompressed)) {
    outError = "Failed to write patch header";
    return false;
  }

  f.write(reinterpret_cast<const char*>(payloadCompressed.data()), static_cast<std::streamsize>(payloadCompressed.size()));
  if (!f) {
    outError = "Failed to write patch payload";
    return false;
  }

  return true;
}

bool LoadWorldPatchBinary(WorldPatch& outPatch, const std::string& path, std::string& outError)
{
  outError.clear();
  outPatch = {};

  if (path.empty()) {
    outError = "Empty patch path";
    return false;
  }

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open patch file";
    return false;
  }

  WorldPatchCompression comp = WorldPatchCompression::None;
  std::uint32_t payloadSize = 0;
  std::uint32_t payloadSizeCompressed = 0;
  if (!ReadHeader(f, outPatch, comp, payloadSize, payloadSizeCompressed, outError)) {
    return false;
  }

  std::vector<std::uint8_t> payloadCompressed(payloadSizeCompressed);
  f.read(reinterpret_cast<char*>(payloadCompressed.data()), static_cast<std::streamsize>(payloadCompressed.size()));
  if (f.gcount() != static_cast<std::streamsize>(payloadCompressed.size())) {
    outError = "Failed to read patch payload";
    return false;
  }

  std::vector<std::uint8_t> payload;
  if (comp == WorldPatchCompression::SLLZ) {
    if (!DecompressSLLZ(payloadCompressed.data(), payloadCompressed.size(), payloadSize, payload, outError)) {
      if (outError.empty()) outError = "Patch decompression failed";
      return false;
    }
  } else {
    payload = std::move(payloadCompressed);
    if (payload.size() != payloadSize) {
      outError = "Patch payload size mismatch";
      return false;
    }
  }

  ByteReader r;
  r.data = payload.data();
  r.size = payload.size();
  r.pos = 0;

  if (outPatch.includeProcCfg) {
    if (!ReadProcCfg(r, outPatch.procCfg, outPatch.version)) {
      outError = "Failed to parse ProcGenConfig";
      return false;
    }
  }
  if (outPatch.includeSimCfg) {
    if (!ReadSimCfg(r, outPatch.simCfg)) {
      outError = "Failed to parse SimConfig";
      return false;
    }
  }
  if (outPatch.includeStats) {
    if (!ReadStats(r, outPatch.stats)) {
      outError = "Failed to parse Stats";
      return false;
    }
  }

  if (!ReadTileDeltas(r, outPatch.tiles)) {
    outError = "Failed to parse tile deltas";
    return false;
  }

  // Ensure we've consumed the whole payload (helps catch corrupt files).
  if (r.pos != r.size) {
    outError = "Extra bytes at end of patch payload";
    return false;
  }

  return true;
}

bool SerializeWorldPatchBinary(const WorldPatch& patch, std::vector<std::uint8_t>& outBytes, std::string& outError,
                               WorldPatchCompression compression)
{
  outError.clear();
  outBytes.clear();

  if (patch.width <= 0 || patch.height <= 0) {
    outError = "Invalid patch dimensions";
    return false;
  }

  // Build payload.
  ByteWriter payload;
  if (patch.includeProcCfg) {
    if (!WriteProcCfg(payload, patch.procCfg, kPatchVersion)) {
      outError = "Failed to write ProcGenConfig";
      return false;
    }
  }
  if (patch.includeSimCfg) {
    if (!WriteSimCfg(payload, patch.simCfg)) {
      outError = "Failed to write SimConfig";
      return false;
    }
  }
  if (patch.includeStats) {
    if (!WriteStats(payload, patch.stats)) {
      outError = "Failed to write Stats";
      return false;
    }
  }
  if (!WriteTileDeltas(payload, patch.tiles)) {
    outError = "Failed to write tile deltas";
    return false;
  }

  const std::uint32_t payloadSize = static_cast<std::uint32_t>(payload.out.size());

  std::vector<std::uint8_t> payloadCompressed;
  if (compression == WorldPatchCompression::SLLZ) {
    if (!CompressSLLZ(payload.out.data(), payload.out.size(), payloadCompressed)) {
      outError = "Compression failed";
      return false;
    }
  } else {
    payloadCompressed = payload.out;
  }

  const std::uint32_t payloadSizeCompressed = static_cast<std::uint32_t>(payloadCompressed.size());

  // Header
  ByteWriter header;
  header.writeBytes(kMagic, sizeof(kMagic));
  header.writeU32(kPatchVersion);

  std::uint32_t flags = 0;
  if (patch.includeProcCfg) flags |= kFlagProcCfg;
  if (patch.includeSimCfg) flags |= kFlagSimCfg;
  if (patch.includeStats) flags |= kFlagStats;
  header.writeU32(flags);

  header.writeI32(static_cast<std::int32_t>(patch.width));
  header.writeI32(static_cast<std::int32_t>(patch.height));
  header.writeU64(patch.baseHash);
  header.writeU64(patch.targetHash);
  header.writeU8(static_cast<std::uint8_t>(compression));
  header.writeU32(payloadSize);
  header.writeU32(payloadSizeCompressed);

  outBytes.reserve(header.out.size() + payloadCompressed.size());
  outBytes.insert(outBytes.end(), header.out.begin(), header.out.end());
  outBytes.insert(outBytes.end(), payloadCompressed.begin(), payloadCompressed.end());
  return true;
}

bool DeserializeWorldPatchBinary(WorldPatch& outPatch, const std::uint8_t* data, std::size_t size,
                                 std::string& outError)
{
  outError.clear();
  outPatch = {};

  if (!data || size == 0) {
    outError = "Empty patch buffer";
    return false;
  }

  constexpr std::size_t kHeaderSize = 8 + 4 + 4 + 4 + 4 + 8 + 8 + 1 + 4 + 4;
  if (size < kHeaderSize) {
    outError = "Truncated patch buffer";
    return false;
  }

  ByteReader r;
  r.data = data;
  r.size = size;
  r.pos = 0;

  std::uint8_t magic[8];
  if (!r.readBytes(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "Bad patch magic (not an ISOPATCH file)";
    return false;
  }

  std::uint32_t version = 0;
  if (!r.readU32(version)) {
    outError = "Corrupt patch header (version)";
    return false;
  }
  if (version < 1 || version > kPatchVersion) {
    outError = "Unsupported patch version";
    return false;
  }
  outPatch.version = version;

  std::uint32_t flags = 0;
  if (!r.readU32(flags)) {
    outError = "Corrupt patch header (flags)";
    return false;
  }

  std::int32_t w = 0;
  std::int32_t h = 0;
  if (!r.readI32(w) || !r.readI32(h)) {
    outError = "Corrupt patch header (size)";
    return false;
  }
  outPatch.width = static_cast<int>(w);
  outPatch.height = static_cast<int>(h);

  if (!r.readU64(outPatch.baseHash) || !r.readU64(outPatch.targetHash)) {
    outError = "Corrupt patch header (hashes)";
    return false;
  }

  std::uint8_t compByte = 0;
  if (!r.readU8(compByte)) {
    outError = "Corrupt patch header (compression)";
    return false;
  }

  WorldPatchCompression comp = static_cast<WorldPatchCompression>(compByte);
  if (comp != WorldPatchCompression::None && comp != WorldPatchCompression::SLLZ) {
    outError = "Unsupported patch compression";
    return false;
  }

  std::uint32_t payloadSize = 0;
  std::uint32_t payloadSizeCompressed = 0;
  if (!r.readU32(payloadSize) || !r.readU32(payloadSizeCompressed)) {
    outError = "Corrupt patch header (payload sizes)";
    return false;
  }

  outPatch.includeProcCfg = (flags & kFlagProcCfg) != 0;
  outPatch.includeSimCfg = (flags & kFlagSimCfg) != 0;
  outPatch.includeStats = (flags & kFlagStats) != 0;

  // Basic sanity checks.
  if (outPatch.width <= 0 || outPatch.height <= 0) {
    outError = "Invalid patch dimensions";
    return false;
  }
  if (payloadSize > (1u << 30) || payloadSizeCompressed > (1u << 30)) {
    outError = "Patch payload too large";
    return false;
  }
  if (r.pos + payloadSizeCompressed > size) {
    outError = "Truncated patch payload";
    return false;
  }

  // Extract payload bytes.
  std::vector<std::uint8_t> payloadCompressed(data + r.pos, data + r.pos + payloadSizeCompressed);
  r.pos += payloadSizeCompressed;

  // Decompress if needed.
  std::vector<std::uint8_t> payload;
  if (comp == WorldPatchCompression::SLLZ) {
    if (!DecompressSLLZ(payloadCompressed.data(), payloadCompressed.size(), payloadSize, payload, outError)) {
      if (outError.empty()) outError = "Patch decompression failed";
      return false;
    }
  } else {
    payload = std::move(payloadCompressed);
    if (payload.size() != payloadSize) {
      outError = "Patch payload size mismatch";
      return false;
    }
  }

  ByteReader pr;
  pr.data = payload.data();
  pr.size = payload.size();
  pr.pos = 0;

  if (outPatch.includeProcCfg) {
    if (!ReadProcCfg(pr, outPatch.procCfg, outPatch.version)) {
      outError = "Failed to parse ProcGenConfig";
      return false;
    }
  }
  if (outPatch.includeSimCfg) {
    if (!ReadSimCfg(pr, outPatch.simCfg)) {
      outError = "Failed to parse SimConfig";
      return false;
    }
  }
  if (outPatch.includeStats) {
    if (!ReadStats(pr, outPatch.stats)) {
      outError = "Failed to parse Stats";
      return false;
    }
  }
  if (!ReadTileDeltas(pr, outPatch.tiles)) {
    outError = "Failed to parse tile deltas";
    return false;
  }
  if (pr.pos != pr.size) {
    outError = "Extra bytes at end of patch payload";
    return false;
  }

  // Ensure no trailing bytes beyond the payload in the container.
  if (r.pos != r.size) {
    outError = "Extra bytes at end of patch file";
    return false;
  }

  return true;
}


bool InvertWorldPatch(const World& baseWorld, const ProcGenConfig& baseProcCfg, const SimConfig& baseSimCfg,
                      const WorldPatch& forwardPatch, WorldPatch& outInverse, std::string& outError, bool force)
{
  outError.clear();
  outInverse = {};

  if (baseWorld.width() != forwardPatch.width || baseWorld.height() != forwardPatch.height) {
    outError = "InvertWorldPatch: dimension mismatch";
    return false;
  }

  const std::uint64_t baseHash = HashWorld(baseWorld, forwardPatch.includeStats);
  if (!force && baseHash != forwardPatch.baseHash) {
    outError = "InvertWorldPatch: base hash mismatch (refusing without --force)";
    return false;
  }

  outInverse.width = forwardPatch.width;
  outInverse.height = forwardPatch.height;

  outInverse.includeProcCfg = forwardPatch.includeProcCfg;
  outInverse.includeSimCfg = forwardPatch.includeSimCfg;
  outInverse.includeStats = forwardPatch.includeStats;

  // Swap hashes: inverse expects to be applied on the forward target state.
  outInverse.baseHash = forwardPatch.targetHash;
  outInverse.targetHash = forwardPatch.baseHash;

  if (outInverse.includeProcCfg) outInverse.procCfg = baseProcCfg;
  if (outInverse.includeSimCfg) outInverse.simCfg = baseSimCfg;
  if (outInverse.includeStats) outInverse.stats = baseWorld.stats();

  outInverse.tiles.clear();
  outInverse.tiles.reserve(forwardPatch.tiles.size());

  const int w = baseWorld.width();

  for (const WorldPatchTileDelta& d : forwardPatch.tiles) {
    const std::uint32_t idx = d.index;
    const int x = static_cast<int>(idx % static_cast<std::uint32_t>(w));
    const int y = static_cast<int>(idx / static_cast<std::uint32_t>(w));
    if (!baseWorld.inBounds(x, y)) {
      outError = "InvertWorldPatch: forward patch contains out-of-bounds tile index";
      return false;
    }

    WorldPatchTileDelta inv;
    inv.index = d.index;
    inv.mask = d.mask;
    inv.value = baseWorld.at(x, y);
    outInverse.tiles.push_back(inv);
  }

  // Ensure deterministic ordering.
  std::sort(outInverse.tiles.begin(), outInverse.tiles.end(),
            [](const WorldPatchTileDelta& a, const WorldPatchTileDelta& b) { return a.index < b.index; });

  return true;
}

bool ComposeWorldPatches(const World& baseWorld, const ProcGenConfig& baseProcCfg, const SimConfig& baseSimCfg,
                         const std::vector<WorldPatch>& patches, WorldPatch& outPatch, std::string& outError,
                         bool includeProcCfg, bool includeSimCfg, bool includeStats, bool force)
{
  outError.clear();
  outPatch = {};

  if (patches.empty()) {
    outError = "ComposeWorldPatches: requires at least one patch";
    return false;
  }

  World w = baseWorld;
  ProcGenConfig proc = baseProcCfg;
  SimConfig sim = baseSimCfg;

  for (std::size_t i = 0; i < patches.size(); ++i) {
    std::string err;
    if (!ApplyWorldPatch(w, proc, sim, patches[i], err, force)) {
      outError = "ComposeWorldPatches: apply failed at patch[" + std::to_string(i) + "]: " + err;
      return false;
    }
  }

  if (!MakeWorldPatch(baseWorld, baseProcCfg, baseSimCfg, w, proc, sim, outPatch, outError,
                      includeProcCfg, includeSimCfg, includeStats)) {
    if (outError.empty()) outError = "ComposeWorldPatches: MakeWorldPatch failed";
    return false;
  }

  return true;
}


} // namespace isocity
