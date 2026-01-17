#include "isocity/TradeMarket.hpp"

#include "isocity/Random.hpp"
#include "isocity/DeterministicMath.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <unordered_set>

namespace isocity {

namespace {

constexpr int kCrateSize = 10; // goods units per tradable crate (keeps $ scales tame)

inline float Frac01(std::uint32_t u) { return static_cast<float>(u) / 4294967295.0f; }

inline int ClampI(int v, int lo, int hi) { return std::clamp(v, lo, hi); }

inline int BaseIndustrialSupply(int level) { return 12 * std::clamp(level, 0, 3); }
inline int BaseCommercialDemand(int level) { return 8 * std::clamp(level, 0, 3); }

static std::uint32_t DaySeed32(std::uint64_t worldSeed, int day, std::uint32_t salt)
{
  const std::uint32_t s0 = static_cast<std::uint32_t>(worldSeed & 0xFFFFFFFFu);
  std::uint32_t v = s0 ^ (static_cast<std::uint32_t>(day) * 0x9E3779B1u) ^ (salt * 0x85EBCA6Bu);
  v ^= v >> 16;
  v *= 0x7FEB352Du;
  v ^= v >> 15;
  v *= 0x846CA68Bu;
  v ^= v >> 16;
  return v;
}

static bool Contains3(const std::array<std::uint8_t, 3>& a, std::uint8_t id)
{
  return (a[0] == id || a[1] == id || a[2] == id);
}

static std::vector<TradeCommodity> GenerateCommodities(std::uint32_t seed32, int commodityCount)
{
  static constexpr const char* kAdj[] = {
    "Iron", "Copper", "Amber", "Verdant", "Cobalt", "Ivory", "Saffron", "Frost", "Umber", "Azure",
    "Silk", "Granite", "Gilded", "Brass", "Silver", "Crimson", "Dawn", "Moss", "Smoke", "Sun",
  };
  static constexpr const char* kNoun[] = {
    "Grain", "Timber", "Textiles", "Machinery", "Tools", "Glass", "Ceramics", "Spices", "Tea", "Salt",
    "Alloys", "Microchips", "Medicine", "Luxuries", "Fuel", "Paper", "Cement", "Plastics", "Dyes", "Gadgets",
  };

  const int adjN = static_cast<int>(sizeof(kAdj) / sizeof(kAdj[0]));
  const int nounN = static_cast<int>(sizeof(kNoun) / sizeof(kNoun[0]));

  std::vector<TradeCommodity> out;
  out.reserve(static_cast<std::size_t>(std::max(0, commodityCount)));

  std::unordered_set<std::string> used;
  used.reserve(64);

  for (int i = 0; i < commodityCount; ++i) {
    const std::uint32_t h = HashCoords32(i * 97 + 13, i * 31 + 7, seed32 ^ 0xC011A11Du);

    TradeCommodity c;
    c.id = static_cast<std::uint8_t>(i);

    // Categories: deliberately skew so bulk is common.
    const float t = Frac01(h ^ 0xA5A5A5A5u);
    if (t < 0.45f) c.category = 0;
    else if (t < 0.80f) c.category = 1;
    else c.category = 2;

    // Base price per crate: bulk cheaper, luxury expensive.
    const int base = (c.category == 0) ? 1 : (c.category == 1) ? 2 : 3;
    c.basePricePerCrate = base + static_cast<int>((h >> 12) % 4u); // 1..6-ish

    // Volatility: luxury tends to be spikier.
    const float v0 = 0.06f + 0.14f * Frac01(h ^ 0x5EEDBEEFu);
    c.volatility = v0 * ((c.category == 2) ? 1.35f : (c.category == 1) ? 1.10f : 0.95f);

    // Name: ensure uniqueness via a tiny collision loop.
    std::string name;
    std::uint32_t hn = h;
    for (int tries = 0; tries < 8; ++tries) {
      const int ai = static_cast<int>((hn) % static_cast<std::uint32_t>(adjN));
      const int ni = static_cast<int>(((hn >> 8) ^ (hn >> 16)) % static_cast<std::uint32_t>(nounN));
      name = std::string(kAdj[ai]) + " " + kNoun[ni];
      if (used.insert(name).second) break;
      hn = HashCoords32(static_cast<int>(hn), i + tries * 17, seed32 ^ 0xBADC0DEu);
    }
    c.name = name;

    out.push_back(c);
  }

  return out;
}

static std::vector<TradePartner> GeneratePartners(std::uint32_t seed32, int partnerCount, int commodityCount)
{
  static constexpr const char* kDir[] = {"North", "South", "East", "West", "High", "Low", "Outer", "Inner"};
  static constexpr const char* kNoun[] = {"League", "Consortium", "Freeport", "Guild", "Marches", "Archipelago", "Federation", "Combine"};

  const int dirN = static_cast<int>(sizeof(kDir) / sizeof(kDir[0]));
  const int nounN = static_cast<int>(sizeof(kNoun) / sizeof(kNoun[0]));

  std::vector<TradePartner> out;
  out.reserve(static_cast<std::size_t>(std::max(0, partnerCount)));

  for (int p = 0; p < partnerCount; ++p) {
    const std::uint32_t h = HashCoords32(p * 19 + 3, p * 53 + 11, seed32 ^ 0x7A11F00Du);
    TradePartner tp;

    const int di = static_cast<int>(h % static_cast<std::uint32_t>(dirN));
    const int ni = static_cast<int>(((h >> 8) ^ (h >> 17)) % static_cast<std::uint32_t>(nounN));
    tp.name = std::string(kDir[di]) + " " + kNoun[ni];

    // Shipping multiplier: 1.05..1.45 roughly.
    const float dist = 40.0f + 140.0f * Frac01(h ^ 0x13579BDFu);
    tp.shippingMult = 1.0f + dist / 400.0f;

    // Reliability: 0.72..0.97.
    tp.reliability = 0.72f + 0.25f * Frac01(h ^ 0x2468ACE0u);

    // Pick favored commodities.
    auto pickId = [&](std::uint32_t salt) -> std::uint8_t {
      if (commodityCount <= 0) return 0;
      const std::uint32_t hv = HashCoords32(p, static_cast<int>(salt), seed32 ^ salt);
      return static_cast<std::uint8_t>(hv % static_cast<std::uint32_t>(commodityCount));
    };

    tp.favoredBuys = {pickId(0xB001u), pickId(0xB002u), pickId(0xB003u)};
    tp.favoredSells = {pickId(0x5E110u), pickId(0x5E111u), pickId(0x5E112u)};

    out.push_back(tp);
  }

  return out;
}

static std::vector<float> ComputeDailyMarketFactor(std::uint32_t seed32, int day,
                                                   const std::vector<TradeCommodity>& comm)
{
  std::vector<float> out;
  out.resize(comm.size(), 1.0f);

  const std::uint32_t ds = DaySeed32(static_cast<std::uint64_t>(seed32), day, 0x4D4B545Fu); // "MKT_"

  for (std::size_t i = 0; i < comm.size(); ++i) {
    const std::uint32_t h0 = HashCoords32(static_cast<int>(i), 17, seed32 ^ 0xA11CEu);
    const std::uint32_t h1 = HashCoords32(static_cast<int>(i), 91, seed32 ^ 0xC0FFEEu);

    // Deterministic pseudo-cycle (avoid std::sin to reduce cross-platform drift).
    const int minPeriod = 63;
    const int maxPeriod = 210;
    const int span = std::max(1, maxPeriod - minPeriod);
    const int periodDays = minPeriod + static_cast<int>(h1 % static_cast<std::uint32_t>(span));
    const int phaseDays = static_cast<int>(h0 % static_cast<std::uint32_t>(std::max(2, periodDays)));

    const float cyc = Q16ToFloat(PseudoSineWaveQ16(day, periodDays, phaseDays));
    const float cycAmp = 0.12f + 0.10f * Frac01(h1 ^ 0x9E3779B9u);
    const std::uint32_t hn = HashCoords32(day, static_cast<int>(i) * 13 + 7, ds ^ 0xD00Du);
    const float noise = (Frac01(hn) - 0.5f) * comm[i].volatility;

    float f = 1.0f + cyc * cycAmp + noise;
    f = std::clamp(f, 0.65f, 1.45f);
    out[i] = f;
  }

  return out;
}

static float PartnerMood(std::uint32_t seed32, int day, int p)
{
  const std::uint32_t ds = DaySeed32(static_cast<std::uint64_t>(seed32), day, 0x50415254u); // "PART"
  const std::uint32_t h = HashCoords32(day, p, ds ^ 0x1234567u);
  // Small +/- 4% wobble.
  return 0.98f + 0.08f * Frac01(h);
}

static bool PartnerDisrupted(std::uint32_t seed32, int day, int p, float reliability)
{
  const std::uint32_t ds = DaySeed32(static_cast<std::uint64_t>(seed32), day, 0x53484F4Bu); // "SHOK"
  const std::uint32_t h = HashCoords32(p, day, ds ^ 0xF00Du);
  const float r = Frac01(h);
  return r > std::clamp(reliability, 0.0f, 1.0f);
}

static int PriceBuyPerCrate(const TradeCommodity& c, float market, const TradePartner& p,
                            float mood, bool disrupted)
{
  float mult = 1.0f;
  if (Contains3(p.favoredBuys, c.id)) mult *= 1.22f;
  if (disrupted) mult *= 0.92f;

  float price = static_cast<float>(std::max(1, c.basePricePerCrate));
  price *= market;
  price *= mood;
  price *= mult;
  price /= std::max(0.75f, p.shippingMult);

  const int out = static_cast<int>(std::lround(std::clamp(price, 1.0f, 99.0f)));
  return std::max(1, out);
}

static int PriceSellPerCrate(const TradeCommodity& c, float market, const TradePartner& p,
                             float mood, bool disrupted)
{
  float mult = 1.0f;
  if (Contains3(p.favoredSells, c.id)) mult *= 0.84f;
  if (disrupted) mult *= 1.10f;

  float price = static_cast<float>(std::max(1, c.basePricePerCrate));
  price *= market;
  price *= mood;
  price *= mult;
  price *= std::max(0.75f, p.shippingMult);

  const int out = static_cast<int>(std::lround(std::clamp(price, 1.0f, 99.0f)));
  return std::max(1, out);
}

struct SupplyDemand {
  std::vector<int> supply;
  std::vector<int> demand;
};

static SupplyDemand ComputeSupplyDemandByCommodity(const World& world,
                                                  const std::vector<TradeCommodity>& commodities)
{
  SupplyDemand out;
  const int nComm = static_cast<int>(commodities.size());
  out.supply.assign(static_cast<std::size_t>(nComm), 0);
  out.demand.assign(static_cast<std::size_t>(nComm), 0);
  if (nComm <= 0) return out;

  // Precompute commodity IDs per category.
  std::vector<int> byCat[3];
  for (int i = 0; i < nComm; ++i) {
    const int cat = std::clamp(static_cast<int>(commodities[i].category), 0, 2);
    byCat[cat].push_back(i);
  }

  const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu);

  auto pickFromCat = [&](int cat, int x, int y, std::uint32_t salt) -> int {
    cat = std::clamp(cat, 0, 2);
    const std::uint32_t h = HashCoords32(x, y, seed32 ^ salt);
    if (!byCat[cat].empty()) {
      return byCat[cat][static_cast<std::size_t>(h % static_cast<std::uint32_t>(byCat[cat].size()))];
    }
    return static_cast<int>(h % static_cast<std::uint32_t>(nComm));
  };

  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      const Tile& t = world.at(x, y);

      if (t.overlay == Overlay::Industrial && t.level > 0) {
        const int level = static_cast<int>(t.level);
        const int amount = BaseIndustrialSupply(level);

        const int cat = std::clamp(level - 1, 0, 2);
        const int cid = pickFromCat(cat, x, y, 0x01AD011u);
        out.supply[static_cast<std::size_t>(cid)] += amount;
      }

      if (t.overlay == Overlay::Commercial && t.level > 0) {
        const int level = static_cast<int>(t.level);
        const int base = BaseCommercialDemand(level);
        if (base <= 0) continue;

        // Category split: higher-level commercial consumes more luxury goods.
        float wBulk = (level == 1) ? 0.55f : (level == 2) ? 0.40f : 0.25f;
        float wCons = (level == 1) ? 0.35f : (level == 2) ? 0.40f : 0.45f;
        float wLux  = (level == 1) ? 0.10f : (level == 2) ? 0.20f : 0.30f;

        // District bias: nudges tastes so districts diverge.
        const int d = static_cast<int>(t.district);
        const float b0 = (Frac01(HashCoords32(d, 0, seed32 ^ 0xD15A71Cu)) - 0.5f) * 0.10f;
        const float b1 = (Frac01(HashCoords32(d, 1, seed32 ^ 0xD15A71Cu)) - 0.5f) * 0.10f;
        const float b2 = (Frac01(HashCoords32(d, 2, seed32 ^ 0xD15A71Cu)) - 0.5f) * 0.10f;

        wBulk = std::clamp(wBulk + b0, 0.10f, 0.80f);
        wCons = std::clamp(wCons + b1, 0.10f, 0.80f);
        wLux  = std::clamp(wLux  + b2, 0.05f, 0.80f);

        const float sum = std::max(0.001f, wBulk + wCons + wLux);
        wBulk /= sum;
        wCons /= sum;
        wLux /= sum;

        int aBulk = static_cast<int>(std::lround(static_cast<float>(base) * wBulk));
        int aCons = static_cast<int>(std::lround(static_cast<float>(base) * wCons));
        int aLux  = std::max(0, base - aBulk - aCons);
        if (aLux < 0) aLux = 0;

        const int c0 = pickFromCat(0, x + 11, y + 17, 0xC0AABB0u);
        const int c1 = pickFromCat(1, x + 37, y + 41, 0xC0AABB1u);
        const int c2 = pickFromCat(2, x + 59, y + 73, 0xC0AABB2u);

        out.demand[static_cast<std::size_t>(c0)] += aBulk;
        out.demand[static_cast<std::size_t>(c1)] += aCons;
        out.demand[static_cast<std::size_t>(c2)] += aLux;
      }
    }
  }

  return out;
}

static int ChooseBestPartnerForImports(const std::vector<TradeCommodity>& comm,
                                       const std::vector<TradePartner>& partners,
                                       const std::vector<float>& market,
                                       const std::vector<int>& deficit,
                                       const std::vector<std::uint8_t>& disrupted,
                                       std::uint32_t seed32,
                                       int day)
{
  if (partners.empty() || comm.empty()) return -1;

  std::vector<std::size_t> cand;
  cand.reserve(partners.size());
  for (std::size_t i = 0; i < partners.size(); ++i) {
    if (i < disrupted.size() && disrupted[i] == 0) cand.push_back(i);
  }
  if (cand.empty()) {
    for (std::size_t i = 0; i < partners.size(); ++i) cand.push_back(i);
  }

  int best = -1;
  double bestAvg = 0.0;
  bool bestSet = false;

  for (std::size_t pi : cand) {
    const TradePartner& p = partners[pi];
    const bool dis = (pi < disrupted.size() && disrupted[pi] != 0);
    const float mood = PartnerMood(seed32, day, static_cast<int>(pi));

    std::int64_t totalCrates = 0;
    std::int64_t totalCost = 0;

    for (std::size_t ci = 0; ci < comm.size() && ci < market.size() && ci < deficit.size(); ++ci) {
      const int d = deficit[ci];
      if (d <= 0) continue;
      const int crates = (d + kCrateSize - 1) / kCrateSize;
      const int price = PriceSellPerCrate(comm[ci], market[ci], p, mood, dis);
      totalCrates += crates;
      totalCost += static_cast<std::int64_t>(crates) * static_cast<std::int64_t>(price);
    }

    const double avg = (totalCrates > 0) ? (static_cast<double>(totalCost) / static_cast<double>(totalCrates)) : 0.0;

    if (!bestSet || avg < bestAvg) {
      bestSet = true;
      bestAvg = avg;
      best = static_cast<int>(pi);
    } else if (bestSet && avg == bestAvg) {
      // Deterministic tie-break.
      const std::uint32_t hBest = HashCoords32(best, day, seed32 ^ 0x1F00A11u);
      const std::uint32_t hThis = HashCoords32(static_cast<int>(pi), day, seed32 ^ 0x1F00A11u);
      if (hThis < hBest) best = static_cast<int>(pi);
    }
  }

  return best;
}

static int ChooseBestPartnerForExports(const std::vector<TradeCommodity>& comm,
                                       const std::vector<TradePartner>& partners,
                                       const std::vector<float>& market,
                                       const std::vector<int>& surplus,
                                       const std::vector<std::uint8_t>& disrupted,
                                       std::uint32_t seed32,
                                       int day)
{
  if (partners.empty() || comm.empty()) return -1;

  std::vector<std::size_t> cand;
  cand.reserve(partners.size());
  for (std::size_t i = 0; i < partners.size(); ++i) {
    if (i < disrupted.size() && disrupted[i] == 0) cand.push_back(i);
  }
  if (cand.empty()) {
    for (std::size_t i = 0; i < partners.size(); ++i) cand.push_back(i);
  }

  int best = -1;
  double bestAvg = 0.0;
  bool bestSet = false;

  for (std::size_t pi : cand) {
    const TradePartner& p = partners[pi];
    const bool dis = (pi < disrupted.size() && disrupted[pi] != 0);
    const float mood = PartnerMood(seed32, day, static_cast<int>(pi));

    std::int64_t totalCrates = 0;
    std::int64_t totalRev = 0;

    for (std::size_t ci = 0; ci < comm.size() && ci < market.size() && ci < surplus.size(); ++ci) {
      const int s = surplus[ci];
      if (s <= 0) continue;
      const int crates = (s + kCrateSize - 1) / kCrateSize;
      const int price = PriceBuyPerCrate(comm[ci], market[ci], p, mood, dis);
      totalCrates += crates;
      totalRev += static_cast<std::int64_t>(crates) * static_cast<std::int64_t>(price);
    }

    const double avg = (totalCrates > 0) ? (static_cast<double>(totalRev) / static_cast<double>(totalCrates)) : 0.0;

    if (!bestSet || avg > bestAvg) {
      bestSet = true;
      bestAvg = avg;
      best = static_cast<int>(pi);
    } else if (bestSet && avg == bestAvg) {
      const std::uint32_t hBest = HashCoords32(best, day, seed32 ^ 0x3F00B22u);
      const std::uint32_t hThis = HashCoords32(static_cast<int>(pi), day, seed32 ^ 0x3F00B22u);
      if (hThis < hBest) best = static_cast<int>(pi);
    }
  }

  return best;
}

static int SeverityCapacityPct(std::uint32_t seed32, int day, int partnerIdx, bool disrupted)
{
  if (!disrupted) return 100;
  const std::uint32_t ds = DaySeed32(static_cast<std::uint64_t>(seed32), day, 0x53455652u); // "SEVR"
  const std::uint32_t h = HashCoords32(day, partnerIdx, ds ^ 0xCAFEu);
  const float t = Frac01(h);
  const int cap = 50 + static_cast<int>(std::lround(t * 35.0f));
  return std::clamp(cap, 30, 90);
}

static void ComputeDeficitSurplus(const std::vector<int>& supply, const std::vector<int>& demand,
                                 std::vector<int>& deficit, std::vector<int>& surplus)
{
  const std::size_t n = std::min(supply.size(), demand.size());
  deficit.assign(n, 0);
  surplus.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    const int s = supply[i];
    const int d = demand[i];
    if (d > s) deficit[i] = d - s;
    else if (s > d) surplus[i] = s - d;
  }
}

static void AllocateByWeights(int total, const std::vector<int>& w, std::vector<int>& out, std::uint32_t seed32, int day, std::uint32_t salt)
{
  out.assign(w.size(), 0);
  if (total <= 0) return;

  std::int64_t sumW = 0;
  for (int v : w) sumW += std::max(0, v);
  if (sumW <= 0) {
    // No weights: put everything in bucket 0.
    if (!out.empty()) out[0] = total;
    return;
  }

  std::int64_t assigned = 0;
  std::vector<std::uint32_t> frac(w.size(), 0u);

  for (std::size_t i = 0; i < w.size(); ++i) {
    const std::int64_t wi = std::max(0, w[i]);
    const std::int64_t num = static_cast<std::int64_t>(total) * wi;
    const int base = static_cast<int>(num / sumW);
    out[i] = base;
    assigned += base;
    frac[i] = static_cast<std::uint32_t>(num % sumW);
  }

  int rem = total - static_cast<int>(assigned);
  if (rem <= 0) return;

  std::vector<std::size_t> idx(out.size());
  std::iota(idx.begin(), idx.end(), 0u);

  std::sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
    if (frac[a] != frac[b]) return frac[a] > frac[b];
    const std::uint32_t ha = HashCoords32(static_cast<int>(a), day, seed32 ^ salt);
    const std::uint32_t hb = HashCoords32(static_cast<int>(b), day, seed32 ^ salt);
    return ha < hb;
  });

  for (std::size_t k = 0; k < idx.size() && rem > 0; ++k) {
    out[idx[k]] += 1;
    rem -= 1;
  }

  // Any still-remaining remainder (due to empty vectors) goes to first bucket.
  if (rem > 0 && !out.empty()) out[0] += rem;
}

} // namespace

TradeMarketSummary PlanTradeMarket(const World& world, int day, const TradeModelSettings& settings,
                                  int commodityCount, int partnerCount)
{
  TradeMarketSummary sum;
  sum.day = day;

  const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu);
  commodityCount = std::clamp(commodityCount, 1, 32);
  partnerCount = std::clamp(partnerCount, 1, 8);

  if (!settings.enabled) {
    // Legacy mode: treat trade as always available (goods model decides connectivity).
    sum.chosenImportPartner = -1;
    sum.chosenExportPartner = -1;
    sum.importCapacityPct = settings.allowImports ? 100 : 0;
    sum.exportCapacityPct = settings.allowExports ? 100 : 0;
    sum.marketIndex = 1.0f;
    return sum;
  }

  const std::vector<TradeCommodity> commodities = GenerateCommodities(seed32, commodityCount);
  const std::vector<TradePartner> partners = GeneratePartners(seed32, partnerCount, commodityCount);
  const std::vector<float> market = ComputeDailyMarketFactor(seed32, day, commodities);

  // Supply/demand snapshot used only to pick partners (independent of route feasibility).
  const SupplyDemand sd = ComputeSupplyDemandByCommodity(world, commodities);
  std::vector<int> deficit, surplus;
  ComputeDeficitSurplus(sd.supply, sd.demand, deficit, surplus);

  // Disruption status per partner.
  std::vector<std::uint8_t> disrupted(partners.size(), 0u);
  for (std::size_t i = 0; i < partners.size(); ++i) {
    disrupted[i] = PartnerDisrupted(seed32, day, static_cast<int>(i), partners[i].reliability) ? 1u : 0u;
  }

  int importP = -1;
  int exportP = -1;

  if (settings.allowImports) {
    if (settings.importPartner >= 0 && settings.importPartner < static_cast<int>(partners.size())) {
      importP = settings.importPartner;
    } else {
      importP = ChooseBestPartnerForImports(commodities, partners, market, deficit, disrupted, seed32, day);
    }
  }

  if (settings.allowExports) {
    if (settings.exportPartner >= 0 && settings.exportPartner < static_cast<int>(partners.size())) {
      exportP = settings.exportPartner;
    } else {
      exportP = ChooseBestPartnerForExports(commodities, partners, market, surplus, disrupted, seed32, day);
    }
  }

  sum.chosenImportPartner = importP;
  sum.chosenExportPartner = exportP;

  sum.importDisrupted = (importP >= 0 && importP < static_cast<int>(disrupted.size()) && disrupted[importP] != 0);
  sum.exportDisrupted = (exportP >= 0 && exportP < static_cast<int>(disrupted.size()) && disrupted[exportP] != 0);

  sum.importCapacityPct = settings.allowImports ? SeverityCapacityPct(seed32, day, importP, sum.importDisrupted) : 0;
  sum.exportCapacityPct = settings.allowExports ? SeverityCapacityPct(seed32, day, exportP, sum.exportDisrupted) : 0;

  // Market index: unweighted mean.
  double mi = 0.0;
  for (float f : market) mi += static_cast<double>(f);
  mi = (!market.empty()) ? (mi / static_cast<double>(market.size())) : 1.0;
  sum.marketIndex = static_cast<float>(std::clamp(mi, 0.65, 1.45));

  return sum;
}

TradeMarketResult ComputeTradeMarket(const World& world, int day, const TradeModelSettings& settings,
                                     const GoodsResult& goods, const TradeMarketSummary& plan,
                                     int commodityCount, int partnerCount)
{
  TradeMarketResult out;
  out.summary = plan;

  const std::uint32_t seed32 = static_cast<std::uint32_t>(world.seed() & 0xFFFFFFFFu);
  commodityCount = std::clamp(commodityCount, 1, 32);
  partnerCount = std::clamp(partnerCount, 1, 8);

  out.commodities = GenerateCommodities(seed32, commodityCount);
  out.partners = GeneratePartners(seed32, partnerCount, commodityCount);
  out.marketFactor = ComputeDailyMarketFactor(seed32, day, out.commodities);

  const SupplyDemand sd = ComputeSupplyDemandByCommodity(world, out.commodities);
  out.supply = sd.supply;
  out.demand = sd.demand;
  ComputeDeficitSurplus(out.supply, out.demand, out.deficit, out.surplus);

  // Allocate goods-model imports/exports across commodities.
  AllocateByWeights(goods.goodsImported, out.deficit, out.imported, seed32, day, 0x1F00A11u);
  AllocateByWeights(goods.goodsExported, out.surplus, out.exported, seed32, day, 0x3F00B22u);

  out.importPricePerCrate.assign(out.commodities.size(), 1);
  out.exportPricePerCrate.assign(out.commodities.size(), 1);

  // Legacy mode: fixed exchange rates.
  if (!settings.enabled) {
    out.importCost = goods.goodsImported / 20;
    out.exportRevenue = goods.goodsExported / 25;
    return out;
  }

  // Choose partners (defensive fallbacks).
  int importP = plan.chosenImportPartner;
  int exportP = plan.chosenExportPartner;
  if (importP < 0 || importP >= static_cast<int>(out.partners.size())) importP = (settings.allowImports ? 0 : -1);
  if (exportP < 0 || exportP >= static_cast<int>(out.partners.size())) exportP = (settings.allowExports ? 0 : -1);

  const bool importDis = plan.importDisrupted;
  const bool exportDis = plan.exportDisrupted;

  // Compute per-commodity prices for the chosen partners.
  for (std::size_t i = 0; i < out.commodities.size(); ++i) {
    const TradeCommodity& c = out.commodities[i];
    const float m = (i < out.marketFactor.size()) ? out.marketFactor[i] : 1.0f;

    if (importP >= 0 && importP < static_cast<int>(out.partners.size())) {
      const TradePartner& p = out.partners[static_cast<std::size_t>(importP)];
      const float mood = PartnerMood(seed32, day, importP);
      out.importPricePerCrate[i] = PriceSellPerCrate(c, m, p, mood, importDis);
    }

    if (exportP >= 0 && exportP < static_cast<int>(out.partners.size())) {
      const TradePartner& p = out.partners[static_cast<std::size_t>(exportP)];
      const float mood = PartnerMood(seed32, day, exportP);
      out.exportPricePerCrate[i] = PriceBuyPerCrate(c, m, p, mood, exportDis);
    }
  }

  // Budget impacts.
  std::int64_t importCost = 0;
  std::int64_t exportRev = 0;

  const int tariff = std::clamp(settings.tariffPct, 0, 30);

  for (std::size_t i = 0; i < out.commodities.size(); ++i) {
    const int imp = (i < out.imported.size()) ? out.imported[i] : 0;
    const int exp = (i < out.exported.size()) ? out.exported[i] : 0;

    if (imp > 0 && importP >= 0) {
      const int crates = (imp + kCrateSize - 1) / kCrateSize;
      const int p = (i < out.importPricePerCrate.size()) ? out.importPricePerCrate[i] : 1;
      std::int64_t c = static_cast<std::int64_t>(crates) * static_cast<std::int64_t>(p);
      // Tariff increases cost.
      c += (c * tariff) / 100;
      importCost += c;
    }

    if (exp > 0 && exportP >= 0) {
      const int crates = (exp + kCrateSize - 1) / kCrateSize;
      const int p = (i < out.exportPricePerCrate.size()) ? out.exportPricePerCrate[i] : 1;
      exportRev += static_cast<std::int64_t>(crates) * static_cast<std::int64_t>(p);
    }
  }

  out.importCost = static_cast<int>(std::clamp<std::int64_t>(importCost, 0, 1'000'000'000ll));
  out.exportRevenue = static_cast<int>(std::clamp<std::int64_t>(exportRev, 0, 1'000'000'000ll));

  return out;
}

} // namespace isocity
