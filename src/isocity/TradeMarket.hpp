#pragma once

#include "isocity/Goods.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

// Non-persistent runtime tuning for the procedural trade market.
//
// The market is deterministic per world seed + day; settings are *not* saved in
// the world save file to avoid save-version churn.
struct TradeModelSettings {
  // Master enable for the trade market.
  // When disabled, the simulator falls back to the legacy fixed exchange rates.
  bool enabled = true;

  // Whether the city is allowed to import/export via the map edge.
  // Disabling imports will reduce commercial goods satisfaction.
  bool allowImports = true;
  bool allowExports = true;

  // Tariff applied to imported goods (percentage). This increases import cost.
  int tariffPct = 0; // 0..30

  // Partner selection: -1 = Auto, otherwise index in [0, partnerCount).
  int importPartner = -1;
  int exportPartner = -1;

  bool operator==(const TradeModelSettings& o) const
  {
    return enabled == o.enabled && allowImports == o.allowImports && allowExports == o.allowExports &&
           tariffPct == o.tariffPct && importPartner == o.importPartner && exportPartner == o.exportPartner;
  }

  bool operator!=(const TradeModelSettings& o) const { return !(*this == o); }
};

// Procedurally generated commodity.
struct TradeCommodity {
  std::uint8_t id = 0;
  std::uint8_t category = 0; // 0=bulk, 1=consumer, 2=luxury
  int basePricePerCrate = 1; // dollars per crate (before market/partner multipliers)
  float volatility = 0.10f;  // 0..1-ish
  std::string name;
};

// Procedurally generated trade partner.
struct TradePartner {
  std::string name;

  // Higher values mean more expensive shipping (imports) / lower net revenue (exports).
  float shippingMult = 1.0f;

  // 0..1; lower means more likely to suffer a disruption on a given day.
  float reliability = 0.9f;

  // Commodity IDs this partner pays extra for (we export).
  std::array<std::uint8_t, 3> favoredBuys{0, 0, 0};
  // Commodity IDs this partner sells at a discount (we import).
  std::array<std::uint8_t, 3> favoredSells{0, 0, 0};
};

// Trade plan chosen for the current day.
struct TradeMarketSummary {
  int day = 0;
  int chosenImportPartner = -1;
  int chosenExportPartner = -1;

  // Import/export "capacity" used to throttle the goods model when trade is disrupted.
  // These are in [0,100].
  int importCapacityPct = 100;
  int exportCapacityPct = 100;

  bool importDisrupted = false;
  bool exportDisrupted = false;

  // Average commodity market factor today (roughly 1.0 = baseline).
  float marketIndex = 1.0f;
};

// Full trade computation result (for UI/debug).
struct TradeMarketResult {
  TradeMarketSummary summary;

  std::vector<TradeCommodity> commodities;
  std::vector<TradePartner> partners;

  // Citywide commodity totals (goods units).
  std::vector<int> supply;
  std::vector<int> demand;
  std::vector<int> deficit;
  std::vector<int> surplus;

  // Allocation of the goods model's imports/exports across commodities (goods units).
  std::vector<int> imported;
  std::vector<int> exported;

  // Per-commodity market multiplier for today.
  std::vector<float> marketFactor;

  // Per-commodity prices for the chosen partners (dollars per crate).
  std::vector<int> importPricePerCrate;
  std::vector<int> exportPricePerCrate;

  int importCost = 0;    // dollars
  int exportRevenue = 0; // dollars
};

// Default catalog sizes.
constexpr int kDefaultTradeCommodityCount = 8;
constexpr int kDefaultTradePartnerCount = 3;

// Compute today's partner selection + capacity throttles (independent of goods routing).
TradeMarketSummary PlanTradeMarket(const World& world, int day, const TradeModelSettings& settings,
                                  int commodityCount = kDefaultTradeCommodityCount,
                                  int partnerCount = kDefaultTradePartnerCount);

// Compute full trade breakdown + budget impacts.
//
// `goods` should be the GoodsResult produced by the simulator for this tick/day.
TradeMarketResult ComputeTradeMarket(const World& world, int day, const TradeModelSettings& settings,
                                     const GoodsResult& goods, const TradeMarketSummary& plan,
                                     int commodityCount = kDefaultTradeCommodityCount,
                                     int partnerCount = kDefaultTradePartnerCount);

} // namespace isocity
