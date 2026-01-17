#include "isocity/Economy.hpp"

#include "isocity/Random.hpp"
#include "isocity/DeterministicMath.hpp"
#include "isocity/ZoneMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

namespace {


inline float Clamp01(float v)
{
  return std::clamp(v, 0.0f, 1.0f);
}

inline int ClampInt(int v, int lo, int hi)
{
  return std::clamp(v, lo, hi);
}

inline float SafeClamp(float v, float lo, float hi)
{
  if (!std::isfinite(v)) return lo;
  return std::clamp(v, lo, hi);
}

std::uint64_t EconomyBaseSeed(const World& world, const EconomyModelSettings& settings)
{
  // Mix world seed with a fixed odd constant and an optional caller-provided salt.
  //
  // We avoid storing state; all derived values are recomputed deterministically.
  const std::uint64_t s0 = world.seed() ^ 0xD1B54A32D192ED03ULL;
  const std::uint64_t s1 = settings.seedSalt * 0x9E3779B97F4A7C15ULL;
  return s0 ^ s1;
}

EconomySectorKind PickSectorKind(RNG& rng)
{
  // A stable distribution with mild bias toward classic city-builder archetypes.
  const EconomySectorKind kinds[] = {
      EconomySectorKind::Manufacturing,
      EconomySectorKind::Logistics,
      EconomySectorKind::Tech,
      EconomySectorKind::Tourism,
      EconomySectorKind::Energy,
      EconomySectorKind::Finance,
      EconomySectorKind::Construction,
      EconomySectorKind::Agriculture,
  };
  const int n = static_cast<int>(sizeof(kinds) / sizeof(kinds[0]));
  const int i = rng.rangeInt(0, std::max(0, n - 1));
  return kinds[i];
}

void SectorPreset(EconomySectorKind kind, float& outInd, float& outCom, float& outVol)
{
  // Base affinities reflect typical city economy roles.
  // Values are later nudged by RNG.
  switch (kind) {
    case EconomySectorKind::Agriculture:
      outInd = 0.60f;
      outCom = 0.35f;
      outVol = 0.35f;
      break;
    case EconomySectorKind::Manufacturing:
      outInd = 0.82f;
      outCom = 0.30f;
      outVol = 0.45f;
      break;
    case EconomySectorKind::Logistics:
      outInd = 0.68f;
      outCom = 0.55f;
      outVol = 0.55f;
      break;
    case EconomySectorKind::Energy:
      outInd = 0.74f;
      outCom = 0.32f;
      outVol = 0.60f;
      break;
    case EconomySectorKind::Tech:
      outInd = 0.55f;
      outCom = 0.78f;
      outVol = 0.65f;
      break;
    case EconomySectorKind::Tourism:
      outInd = 0.25f;
      outCom = 0.88f;
      outVol = 0.60f;
      break;
    case EconomySectorKind::Finance:
      outInd = 0.30f;
      outCom = 0.82f;
      outVol = 0.75f;
      break;
    case EconomySectorKind::Construction:
      outInd = 0.72f;
      outCom = 0.45f;
      outVol = 0.50f;
      break;
  }
}

std::string BuildSectorName(EconomySectorKind kind, RNG& rng)
{
  const char* prefixes[] = {
      "North",  "New",   "Port",  "Grand", "Stone", "Silver", "Bright",
      "Union",  "Ever",  "Iron",  "Oak",   "Sun",   "Aurora", "Cedar",
      "Crown",  "Harbor", "Summit", "Metro", "Vista", "River",
  };

  const char* suffixes[] = {
      "Works",      "Holdings", "Guild",     "Collective", "Co",     "Industries",
      "Group",      "Dynamics", "Exchange",  "Labs",       "Lines",  "Ventures",
      "Studios",    "Resorts",  "Foundry",   "Energy",     "Logistics", "Fabrics",
      "Markets",    "Systems",
  };

  const char* kindNouns[] = {
      "Farms",       // Agriculture
      "Foundries",   // Manufacturing
      "Freight",     // Logistics
      "Power",       // Energy
      "Compute",     // Tech
      "Leisure",     // Tourism
      "Capital",     // Finance
      "Build",       // Construction
  };

  const int pN = static_cast<int>(sizeof(prefixes) / sizeof(prefixes[0]));
  const int sN = static_cast<int>(sizeof(suffixes) / sizeof(suffixes[0]));

  const std::string pre = prefixes[rng.rangeInt(0, std::max(0, pN - 1))];
  const std::string suf = suffixes[rng.rangeInt(0, std::max(0, sN - 1))];

  const int ki = ClampInt(static_cast<int>(kind), 0, static_cast<int>(sizeof(kindNouns) / sizeof(kindNouns[0])) - 1);
  const std::string noun = kindNouns[ki];

  // Deterministic but varied naming: sometimes use a noun, sometimes a suffix.
  if (rng.chance(0.55f)) {
    return pre + " " + noun;
  }
  return pre + " " + suf;
}

bool EventStartForDay(std::uint64_t baseSeed, int day, std::uint32_t& outTag)
{
  // A deterministic pseudo-random predicate.
  // About ~1% chance per day => a handful of events per in-game year.
  const std::uint64_t s = baseSeed ^ (static_cast<std::uint64_t>(day) * 0x9E3779B97F4A7C15ULL) ^ 0xF00DF00DF00DF00DULL;
  RNG rng(s);
  const std::uint32_t tag = rng.nextU32();
  outTag = tag;

  // Avoid the very earliest days to reduce early-game chaos.
  if (day < 5) return false;

  return (tag % 103u) == 0u;
}

EconomyEvent EventForStartTag(std::uint32_t tag, int startDay, const EconomyModelSettings& settings)
{
  EconomyEvent e;
  e.startDay = startDay;

  const int minDur = std::max(1, settings.minEventDurationDays);
  const int maxDur = std::max(minDur, settings.maxEventDurationDays);
  const int span = maxDur - minDur + 1;
  e.durationDays = minDur + static_cast<int>(tag % static_cast<std::uint32_t>(span));

  const std::uint32_t sev8 = (tag >> 8) & 0xFFu;
  const float sev01 = static_cast<float>(sev8) / 255.0f;
  e.severity = SafeClamp(0.25f + 0.75f * sev01, 0.0f, 1.0f);

  // Select a non-none event kind.
  const int kKinds = 6; // number of non-none kinds in EconomyEventKind
  const int pick = static_cast<int>((tag >> 16) % static_cast<std::uint32_t>(kKinds));
  switch (pick) {
    case 0: e.kind = EconomyEventKind::Recession; break;
    case 1: e.kind = EconomyEventKind::FuelSpike; break;
    case 2: e.kind = EconomyEventKind::ImportShock; break;
    case 3: e.kind = EconomyEventKind::ExportBoom; break;
    case 4: e.kind = EconomyEventKind::TechBoom; break;
    case 5: e.kind = EconomyEventKind::TourismSurge; break;
    default: e.kind = EconomyEventKind::Recession; break;
  }

  return e;
}

struct EventAdjust {
  float supply = 1.0f;
  float demand = 1.0f;
  float tax = 1.0f;
  float macro = 1.0f;
  float inflationAdd = 0.0f;
};

EventAdjust AdjustForEvent(const EconomyEvent& e, const EconomySector& sector)
{
  EventAdjust a;
  if (e.kind == EconomyEventKind::None || e.durationDays <= 0 || e.severity <= 0.0f) return a;

  const float sev = SafeClamp(e.severity, 0.0f, 1.0f);
  const float vol = SafeClamp(sector.volatility, 0.0f, 1.0f);
  const float swing = sev * (0.55f + 0.65f * vol);

  switch (e.kind) {
    case EconomyEventKind::Recession:
      a.macro = 1.0f - 0.22f * swing;
      a.demand = 1.0f - 0.28f * swing;
      a.supply = 1.0f - 0.12f * swing;
      a.tax = 1.0f - 0.20f * swing;
      a.inflationAdd = 0.015f * sev;
      break;
    case EconomyEventKind::FuelSpike:
      a.macro = 1.0f - 0.10f * swing;
      a.demand = 1.0f - 0.10f * swing;
      a.supply = 1.0f - 0.22f * swing;
      a.tax = 1.0f - 0.08f * swing;
      a.inflationAdd = 0.045f * sev;
      break;
    case EconomyEventKind::ImportShock:
      a.macro = 1.0f - 0.12f * swing;
      a.demand = 1.0f - 0.16f * swing;
      a.supply = 1.0f - 0.10f * swing;
      a.tax = 1.0f - 0.10f * swing;
      a.inflationAdd = 0.030f * sev;
      break;
    case EconomyEventKind::ExportBoom:
      a.macro = 1.0f + 0.18f * swing;
      a.demand = 1.0f + 0.05f * swing;
      a.supply = 1.0f + 0.22f * swing;
      a.tax = 1.0f + 0.12f * swing;
      a.inflationAdd = -0.008f * sev;
      break;
    case EconomyEventKind::TechBoom: {
      const bool techFavored = (sector.kind == EconomySectorKind::Tech || sector.kind == EconomySectorKind::Finance);
      const float k = techFavored ? 1.0f : 0.55f;
      a.macro = 1.0f + (0.12f * k) * swing;
      a.demand = 1.0f + (0.18f * k) * swing;
      a.supply = 1.0f + (0.06f * k) * swing;
      a.tax = 1.0f + (0.14f * k) * swing;
      a.inflationAdd = 0.010f * sev;
      break;
    }
    case EconomyEventKind::TourismSurge: {
      const bool tourismFavored = (sector.kind == EconomySectorKind::Tourism);
      const float k = tourismFavored ? 1.0f : 0.50f;
      a.macro = 1.0f + (0.10f * k) * swing;
      a.demand = 1.0f + (0.22f * k) * swing;
      a.supply = 1.0f + (0.03f * k) * swing;
      a.tax = 1.0f + (0.10f * k) * swing;
      a.inflationAdd = 0.012f * sev;
      break;
    }
    case EconomyEventKind::None:
      break;
  }

  // Guard.
  a.supply = SafeClamp(a.supply, 0.0f, 4.0f);
  a.demand = SafeClamp(a.demand, 0.0f, 4.0f);
  a.tax = SafeClamp(a.tax, 0.0f, 4.0f);
  a.macro = SafeClamp(a.macro, 0.0f, 4.0f);
  return a;
}

struct DistrictScan {
  int landTiles = 0;

  int resTiles = 0;
  int comTiles = 0;
  int indTiles = 0;

  int resCap = 0;
  int comCap = 0;
  int indCap = 0;

  int resOcc = 0;
  int comOcc = 0;
  int indOcc = 0;

  float resLevelSum = 0.0f;
  float comLevelSum = 0.0f;
  float indLevelSum = 0.0f;
};

} // namespace

EconomySnapshot ComputeEconomySnapshot(const World& world, int day, const EconomyModelSettings& settings)
{
  EconomySnapshot out;
  out.day = std::max(0, day);

  // --- Parameter hygiene ---
  const int sectorCount = std::clamp(settings.sectorCount, 1, 16);
  const float period = std::max(4.0f, settings.macroPeriodDays);
  const int scanback = std::clamp(settings.eventScanbackDays, 0, 64);

  const std::uint64_t baseSeed = EconomyBaseSeed(world, settings);

  // --- Procedural sector generation (seeded, deterministic) ---
  out.sectors.clear();
  out.sectors.reserve(static_cast<std::size_t>(sectorCount));

  RNG srng(baseSeed ^ 0xC3A5C85C97CB3127ULL);
  for (int i = 0; i < sectorCount; ++i) {
    EconomySector s;
    s.kind = PickSectorKind(srng);

    float ind = 0.5f;
    float com = 0.5f;
    float vol = 0.5f;
    SectorPreset(s.kind, ind, com, vol);

    // RNG nudges (small).
    ind = SafeClamp(ind + srng.rangeFloat(-0.08f, 0.08f), 0.05f, 0.95f);
    com = SafeClamp(com + srng.rangeFloat(-0.08f, 0.08f), 0.05f, 0.95f);
    vol = SafeClamp(vol + srng.rangeFloat(-0.10f, 0.10f), 0.05f, 0.95f);

    s.industrialAffinity = ind;
    s.commercialAffinity = com;
    s.volatility = vol;

    s.name = BuildSectorName(s.kind, srng);

    out.sectors.push_back(std::move(s));
  }

  // --- Deterministic event detection (scanback for an active event) ---
  out.activeEvent = EconomyEvent{};
  out.activeEventDaysLeft = 0;

  for (int off = 0; off <= scanback; ++off) {
    const int startDay = out.day - off;
    if (startDay < 0) break;

    std::uint32_t tag = 0;
    if (!EventStartForDay(baseSeed, startDay, tag)) continue;

    const EconomyEvent e = EventForStartTag(tag, startDay, settings);
    if (e.durationDays <= 0) continue;

    const int endDay = e.startDay + e.durationDays;
    if (out.day < endDay) {
      out.activeEvent = e;
      out.activeEventDaysLeft = endDay - out.day;
      break; // most recent active event wins (scan from today backwards)
    }
  }

  // --- Macro cycle ---
  // Deterministic two-phase pseudo-sine + deterministic day noise.
  //
  // We avoid std::sin here to reduce cross-platform drift in simulations/hashes.
  RNG prng(baseSeed ^ 0x9E3779B97F4A7C15ULL);

  const int periodI = std::max(4, static_cast<int>(std::lround(period)));
  const int period2I = std::max(2, (periodI * 55 + 50) / 100); // ~55% of primary

  const int phase1 = (periodI > 0) ? prng.rangeInt(0, periodI - 1) : 0;
  const int phase2 = (period2I > 0) ? prng.rangeInt(0, period2I - 1) : 0;

  const float cyc1 = Q16ToFloat(PseudoSineWaveQ16(out.day, periodI, phase1));
  const float cyc2 = Q16ToFloat(PseudoSineWaveQ16(out.day, period2I, phase2));

  // Small deterministic noise.
  const std::uint64_t noiseSeed = baseSeed ^ (static_cast<std::uint64_t>(out.day) * 0xBF58476D1CE4E5B9ULL);
  RNG nrng(noiseSeed);
  const float noise = Q16ToFloat(NoiseQ16FromU32(nrng.nextU32())); // [-1,1]

  float macro = 1.0f + 0.12f * cyc1 + 0.05f * cyc2 + 0.03f * noise;
  macro = SafeClamp(macro, 0.55f, 1.50f);

  // Inflation / volatility proxy.
  float infl = 0.012f + 0.022f * std::abs(cyc2) + 0.010f * std::abs(noise);

  // Apply event-level macro adjustments (citywide). We use a generic sector to compute swing.
  // The per-district multipliers use sector-aware adjustments below.
  if (out.activeEvent.kind != EconomyEventKind::None) {
    EconomySector dummy;
    dummy.kind = EconomySectorKind::Logistics;
    dummy.volatility = 0.6f;
    const EventAdjust ea = AdjustForEvent(out.activeEvent, dummy);
    macro *= ea.macro;
    infl += ea.inflationAdd;
  }

  out.economyIndex = SafeClamp(macro, 0.55f, 1.75f);
  out.inflation = SafeClamp(infl, 0.0f, 0.25f);

  // --- District scan (current built form influences wealth/productivity) ---
  std::array<DistrictScan, static_cast<std::size_t>(kDistrictCount)> scan{};

  const int w = world.width();
  const int h = world.height();

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const Tile& t0 = world.at(x, y);
      const int d = ClampInt(static_cast<int>(t0.district), 0, kDistrictCount - 1);
      DistrictScan& ds = scan[static_cast<std::size_t>(d)];

      if (t0.terrain != Terrain::Water) ds.landTiles++;

      if (t0.overlay == Overlay::Residential) {
        ds.resTiles++;
        ds.resCap += HousingForLevel(static_cast<int>(t0.level));
        ds.resOcc += static_cast<int>(t0.occupants);
        ds.resLevelSum += static_cast<float>(ClampZoneLevel(t0.level));
      } else if (t0.overlay == Overlay::Commercial) {
        ds.comTiles++;
        ds.comCap += JobsCommercialForLevel(static_cast<int>(t0.level));
        ds.comOcc += static_cast<int>(t0.occupants);
        ds.comLevelSum += static_cast<float>(ClampZoneLevel(t0.level));
      } else if (t0.overlay == Overlay::Industrial) {
        ds.indTiles++;
        ds.indCap += JobsIndustrialForLevel(static_cast<int>(t0.level));
        ds.indOcc += static_cast<int>(t0.occupants);
        ds.indLevelSum += static_cast<float>(ClampZoneLevel(t0.level));
      }
    }
  }

  // --- District profiling ---
  float wealthSum = 0.0f;
  float wealthW = 0.0f;

  for (int d = 0; d < kDistrictCount; ++d) {
    DistrictEconomyProfile p;

    const DistrictScan& ds = scan[static_cast<std::size_t>(d)];

    // District latent endowment (seeded, independent of current construction).
    const std::uint64_t dSeed = baseSeed ^ (static_cast<std::uint64_t>(d) * 0xA24BAED4963EE407ULL) ^ 0x632BE59BD9B4E019ULL;
    RNG drng(dSeed);

    const float baseWealth = 0.35f + 0.30f * drng.nextF01();
    const float baseProd = 0.35f + 0.30f * drng.nextF01();

    const float biasWealth = (drng.nextF01() - 0.5f) * 0.20f;
    const float biasProd = (drng.nextF01() - 0.5f) * 0.20f;

    const int secIdx = (out.sectors.empty()) ? -1 : drng.rangeInt(0, static_cast<int>(out.sectors.size()) - 1);
    p.dominantSector = secIdx;

    const EconomySector sec = (secIdx >= 0 && static_cast<std::size_t>(secIdx) < out.sectors.size())
                                  ? out.sectors[static_cast<std::size_t>(secIdx)]
                                  : EconomySector{};

    const float occRes = (ds.resCap > 0) ? SafeClamp(static_cast<float>(ds.resOcc) / static_cast<float>(ds.resCap), 0.0f, 1.25f)
                                        : 0.0f;
    const float occCom = (ds.comCap > 0) ? SafeClamp(static_cast<float>(ds.comOcc) / static_cast<float>(ds.comCap), 0.0f, 1.25f)
                                        : 0.0f;
    const float occInd = (ds.indCap > 0) ? SafeClamp(static_cast<float>(ds.indOcc) / static_cast<float>(ds.indCap), 0.0f, 1.25f)
                                        : 0.0f;

    const float lvlRes = (ds.resTiles > 0) ? SafeClamp(ds.resLevelSum / static_cast<float>(ds.resTiles), 1.0f, 3.0f)
                                          : 1.0f;
    const float lvlCom = (ds.comTiles > 0) ? SafeClamp(ds.comLevelSum / static_cast<float>(ds.comTiles), 1.0f, 3.0f)
                                          : 1.0f;
    const float lvlInd = (ds.indTiles > 0) ? SafeClamp(ds.indLevelSum / static_cast<float>(ds.indTiles), 1.0f, 3.0f)
                                          : 1.0f;

    const float lvlResN = (lvlRes - 1.0f) * 0.5f;
    const float lvlComN = (lvlCom - 1.0f) * 0.5f;
    const float lvlIndN = (lvlInd - 1.0f) * 0.5f;

    // Development signals.
    const float devWealth = Clamp01(0.55f * occRes + 0.35f * occCom + 0.10f * (0.5f * (lvlResN + lvlComN)));
    const float devProd = Clamp01(0.65f * occInd + 0.20f * occCom + 0.15f * lvlIndN);

    // Sector biases: commercial-heavy sectors skew wealth, industrial-heavy skew productivity.
    const float secWealthBias = (sec.commercialAffinity - 0.5f) * 0.20f;
    const float secProdBias = (sec.industrialAffinity - 0.5f) * 0.20f;

    p.wealth = Clamp01(baseWealth + biasWealth + 0.45f * devWealth + secWealthBias);
    p.productivity = Clamp01(baseProd + biasProd + 0.45f * devProd + secProdBias);

    // Baseline multipliers (before shocks).
    const float macroFactor = SafeClamp(out.economyIndex, 0.70f, 1.30f);

    float taxBase = (0.65f + 0.85f * p.wealth + 0.25f * p.productivity) * macroFactor;
    float supply = (0.50f + 1.10f * p.productivity) * macroFactor;
    float demand = (0.50f + 1.15f * p.wealth) * macroFactor;

    // Sector affinity nudges.
    supply *= SafeClamp(0.88f + 0.28f * sec.industrialAffinity, 0.50f, 1.35f);
    demand *= SafeClamp(0.88f + 0.28f * sec.commercialAffinity, 0.50f, 1.35f);

    // Event adjustments (sector-aware).
    const EventAdjust ea = AdjustForEvent(out.activeEvent, sec);
    supply *= ea.supply;
    demand *= ea.demand;
    taxBase *= ea.tax;

    // Clamp to conservative bounds; downstream systems may clamp again.
    p.taxBaseMult = SafeClamp(taxBase, 0.25f, 2.50f);
    p.industrialSupplyMult = SafeClamp(supply, 0.0f, 4.0f);
    p.commercialDemandMult = SafeClamp(demand, 0.0f, 4.0f);

    out.districts[static_cast<std::size_t>(d)] = p;

    const float wgt = (ds.landTiles > 0) ? static_cast<float>(ds.landTiles) : 1.0f;
    wealthSum += p.wealth * wgt;
    wealthW += wgt;
  }

  out.cityWealth = (wealthW > 0.0f) ? SafeClamp(wealthSum / wealthW, 0.0f, 1.0f) : 0.5f;

  return out;
}

} // namespace isocity
