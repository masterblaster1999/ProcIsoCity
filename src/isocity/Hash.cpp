#include "isocity/Hash.hpp"

#include "isocity/World.hpp"

#include <cstring>

namespace isocity {

namespace {

// 64-bit FNV-1a
constexpr std::uint64_t kFNVOffset = 1469598103934665603ull;
constexpr std::uint64_t kFNVPrime = 1099511628211ull;

inline void HashByte(std::uint64_t& h, std::uint8_t b)
{
  h ^= static_cast<std::uint64_t>(b);
  h *= kFNVPrime;
}

inline void HashU16(std::uint64_t& h, std::uint16_t v)
{
  HashByte(h, static_cast<std::uint8_t>((v >> 0) & 0xFFu));
  HashByte(h, static_cast<std::uint8_t>((v >> 8) & 0xFFu));
}

inline void HashU32(std::uint64_t& h, std::uint32_t v)
{
  HashByte(h, static_cast<std::uint8_t>((v >> 0) & 0xFFu));
  HashByte(h, static_cast<std::uint8_t>((v >> 8) & 0xFFu));
  HashByte(h, static_cast<std::uint8_t>((v >> 16) & 0xFFu));
  HashByte(h, static_cast<std::uint8_t>((v >> 24) & 0xFFu));
}

inline void HashU64(std::uint64_t& h, std::uint64_t v)
{
  HashByte(h, static_cast<std::uint8_t>((v >> 0) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 8) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 16) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 24) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 32) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 40) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 48) & 0xFFull));
  HashByte(h, static_cast<std::uint8_t>((v >> 56) & 0xFFull));
}

inline void HashI32(std::uint64_t& h, int v)
{
  // Hash ints as signed 32-bit values in little-endian.
  const std::int32_t sv = static_cast<std::int32_t>(v);
  std::uint32_t uv = 0;
  std::memcpy(&uv, &sv, sizeof(uv));
  HashU32(h, uv);
}

inline void HashF32(std::uint64_t& h, float v)
{
  static_assert(sizeof(float) == 4, "float must be 32-bit");
  std::uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  HashU32(h, bits);
}

inline void HashTile(std::uint64_t& h, const Tile& t)
{
  HashByte(h, static_cast<std::uint8_t>(t.terrain));
  HashByte(h, static_cast<std::uint8_t>(t.overlay));
  HashF32(h, t.height);
  HashByte(h, t.variation);
  HashByte(h, t.level);
  HashU16(h, t.occupants);
  HashByte(h, t.district);
}

} // namespace

std::uint64_t HashStats(const Stats& s)
{
  std::uint64_t h = kFNVOffset;

  // Core time/state.
  HashI32(h, s.day);

  // Population + economy.
  HashI32(h, s.population);
  HashI32(h, s.housingCapacity);
  HashI32(h, s.jobsCapacity);
  HashI32(h, s.jobsCapacityAccessible);
  HashI32(h, s.employed);
  HashF32(h, s.happiness);
  HashI32(h, s.money);
  HashI32(h, s.roads);
  HashI32(h, s.parks);

  // Commute/traffic.
  HashI32(h, s.commuters);
  HashI32(h, s.commutersUnreachable);
  HashF32(h, s.avgCommute);
  HashF32(h, s.p95Commute);
  HashF32(h, s.avgCommuteTime);
  HashF32(h, s.p95CommuteTime);
  HashF32(h, s.trafficCongestion);
  HashI32(h, s.congestedRoadTiles);
  HashI32(h, s.maxRoadTraffic);

  // Transit.
  HashI32(h, s.transitLines);
  HashI32(h, s.transitStops);
  HashI32(h, s.transitRiders);
  HashF32(h, s.transitModeShare);
  HashF32(h, s.transitCommuteCoverage);

  // Goods/logistics.
  HashI32(h, s.goodsProduced);
  HashI32(h, s.goodsDemand);
  HashI32(h, s.goodsDelivered);
  HashI32(h, s.goodsImported);
  HashI32(h, s.goodsExported);
  HashI32(h, s.goodsUnreachableDemand);
  HashF32(h, s.goodsSatisfaction);
  HashI32(h, s.maxRoadGoodsTraffic);

  // Trade/market.
  HashI32(h, s.tradeImportPartner);
  HashI32(h, s.tradeExportPartner);
  HashI32(h, s.tradeImportCapacityPct);
  HashI32(h, s.tradeExportCapacityPct);
  HashByte(h, static_cast<std::uint8_t>(s.tradeImportDisrupted ? 1 : 0));
  HashByte(h, static_cast<std::uint8_t>(s.tradeExportDisrupted ? 1 : 0));
  HashF32(h, s.tradeMarketIndex);

  // Macro economy.
  HashF32(h, s.economyIndex);
  HashF32(h, s.economyInflation);
  HashI32(h, s.economyEventKind);
  HashI32(h, s.economyEventDaysLeft);
  HashF32(h, s.economyCityWealth);

  // Economy snapshot.
  HashI32(h, s.income);
  HashI32(h, s.expenses);
  HashI32(h, s.taxRevenue);
  HashI32(h, s.maintenanceCost);
  HashI32(h, s.upgradeCost);
  HashI32(h, s.importCost);
  HashI32(h, s.exportRevenue);
  HashI32(h, s.transitCost);
  HashF32(h, s.avgTaxPerCapita);

  // Demand/valuation.
  HashF32(h, s.demandResidential);
  HashF32(h, s.demandCommercial);
  HashF32(h, s.demandIndustrial);
  HashF32(h, s.avgLandValue);

  return h;
}

std::uint64_t HashWorld(const World& world, bool includeStats)
{
  std::uint64_t h = kFNVOffset;

  HashI32(h, world.width());
  HashI32(h, world.height());
  HashU64(h, world.seed());

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      HashTile(h, world.at(x, y));
    }
  }

  if (includeStats) {
    const std::uint64_t sh = HashStats(world.stats());
    HashU64(h, sh);
  }

  return h;
}

} // namespace isocity
