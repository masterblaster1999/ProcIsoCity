#include "isocity/Chronicle.hpp"

#include "isocity/Cartography.hpp"
#include "isocity/Economy.hpp"
#include "isocity/Json.hpp"
#include "isocity/Random.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace isocity;

std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

std::string FormatCommaInt(int v)
{
  const bool neg = v < 0;
  std::uint64_t x = static_cast<std::uint64_t>(neg ? -static_cast<std::int64_t>(v) : v);
  std::string s = std::to_string(x);
  for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3) {
    s.insert(static_cast<std::size_t>(i), ",");
  }
  if (neg) s.insert(s.begin(), '-');
  return s;
}

std::string ReplaceAll(std::string s, const std::string& from, const std::string& to)
{
  if (from.empty()) return s;
  std::size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
  return s;
}

std::string ApplyTemplate(std::string t, const std::vector<std::pair<std::string, std::string>>& kv)
{
  for (const auto& p : kv) {
    t = ReplaceAll(std::move(t), p.first, p.second);
  }
  return t;
}

float Clamp01(float v)
{
  if (!std::isfinite(v)) return 0.0f;
  return std::clamp(v, 0.0f, 1.0f);
}

std::string DistrictLabel(int districtId, const std::vector<std::string>& names)
{
  if (districtId >= 0 && districtId < static_cast<int>(names.size())) {
    const std::string& n = names[static_cast<std::size_t>(districtId)];
    if (!n.empty()) return n;
  }
  if (districtId >= 0) return "District " + std::to_string(districtId);
  return "the city";
}

void AddUniqueTag(std::vector<std::string>& tags, const std::string& tag)
{
  if (tag.empty()) return;
  for (const auto& t : tags) {
    if (t == tag) return;
  }
  tags.push_back(tag);
}

float ComputeInstantMayorRating(const Stats& s)
{
  // Heuristic 0..100 rating built from multiple citywide indicators.
  const float happy = Clamp01(s.happiness);
  const float goods = Clamp01(s.goodsSatisfaction);
  const float services = Clamp01(s.servicesOverallSatisfaction);
  const float congestion = Clamp01(s.trafficCongestion);

  // Money factor: maps roughly [-100..500] into [0..1].
  const float money01 = std::clamp((static_cast<float>(s.money) + 100.0f) / 600.0f, 0.0f, 1.0f);

  const float score01 =
      0.38f * happy +
      0.18f * money01 +
      0.18f * goods +
      0.16f * services +
      0.10f * (1.0f - congestion);

  return std::clamp(score01 * 100.0f, 0.0f, 100.0f);
}

EconomyEventKind SafeEconomyKind(int v)
{
  if (v <= 0) return EconomyEventKind::None;
  if (v > static_cast<int>(EconomyEventKind::TourismSurge)) {
    return EconomyEventKind::None;
  }
  return static_cast<EconomyEventKind>(v);
}

std::string PrettyEconomyEvent(EconomyEventKind k)
{
  switch (k) {
    case EconomyEventKind::None: return "None";
    case EconomyEventKind::Recession: return "Recession";
    case EconomyEventKind::FuelSpike: return "Fuel Spike";
    case EconomyEventKind::ImportShock: return "Import Shock";
    case EconomyEventKind::ExportBoom: return "Export Boom";
    case EconomyEventKind::TechBoom: return "Tech Boom";
    case EconomyEventKind::TourismSurge: return "Tourism Surge";
    default: return "Economy Event";
  }
}

ChronicleTone ToneForEconomyEvent(EconomyEventKind k)
{
  switch (k) {
    case EconomyEventKind::ExportBoom:
    case EconomyEventKind::TechBoom:
    case EconomyEventKind::TourismSurge: return ChronicleTone::Good;
    case EconomyEventKind::Recession:
    case EconomyEventKind::FuelSpike:
    case EconomyEventKind::ImportShock: return ChronicleTone::Bad;
    case EconomyEventKind::None: return ChronicleTone::Neutral;
    default: return ChronicleTone::Neutral;
  }
}

struct Candidate {
  float score = 0.0f;
  ChronicleTone tone = ChronicleTone::Neutral;
  std::vector<std::string> tags;
  std::string headline;
  std::string body;
  std::string tip;
};

void AppendDailyNumbers(std::string& body, const Stats& cur, const Stats* prev)
{
  // Keep this compact and deterministic.
  const int dPop = prev ? (cur.population - prev->population) : 0;
  const int dMoney = prev ? (cur.money - prev->money) : 0;

  std::ostringstream oss;
  oss << "\n\n";
  oss << "Numbers: pop " << FormatCommaInt(cur.population);
  if (prev) oss << " (" << (dPop >= 0 ? "+" : "") << FormatCommaInt(dPop) << ")";
  oss << ", money " << FormatCommaInt(cur.money);
  if (prev) oss << " (" << (dMoney >= 0 ? "+" : "") << FormatCommaInt(dMoney) << ")";
  oss << ", happy " << std::fixed << std::setprecision(2) << static_cast<double>(cur.happiness);
  oss << ", congestion " << std::fixed << std::setprecision(3) << static_cast<double>(cur.trafficCongestion);
  oss << ", goods " << std::fixed << std::setprecision(3) << static_cast<double>(cur.goodsSatisfaction);
  oss << ", services " << std::fixed << std::setprecision(3) << static_cast<double>(cur.servicesOverallSatisfaction);
  body += oss.str();
}

void PushCandidate(std::vector<Candidate>& out, Candidate c)
{
  if (c.headline.empty()) return;
  // Ensure all tags are unique.
  std::vector<std::string> dedup;
  dedup.reserve(c.tags.size());
  for (const std::string& t : c.tags) AddUniqueTag(dedup, t);
  c.tags = std::move(dedup);
  out.push_back(std::move(c));
}

int FindPopulationMilestone(int prevPop, int curPop)
{
  const int milestones[] = {25, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000};
  for (int m : milestones) {
    if (prevPop < m && curPop >= m) return m;
  }
  return -1;
}

} // namespace

namespace isocity {

const char* ChronicleToneName(ChronicleTone t)
{
  switch (t) {
    case ChronicleTone::Good: return "good";
    case ChronicleTone::Neutral: return "neutral";
    case ChronicleTone::Bad: return "bad";
    case ChronicleTone::Alert: return "alert";
    default: return "neutral";
  }
}

Chronicle GenerateCityChronicle(const World& world, const std::vector<Stats>& ticks)
{
  Chronicle out;
  out.seed = world.seed();
  out.cityName = GenerateCityName(world.seed());
  out.districtNames = GenerateDistrictNames(world);

  // Use provided ticks, but gracefully handle an empty vector.
  std::vector<Stats> localTicks;
  if (ticks.empty()) {
    localTicks.push_back(world.stats());
  }
  const std::vector<Stats>& rows = ticks.empty() ? localTicks : ticks;

  float ratingEma = 50.0f;

  for (std::size_t i = 0; i < rows.size(); ++i) {
    const Stats& cur = rows[i];
    const Stats* prev = (i > 0) ? &rows[i - 1] : nullptr;

    const float ratingInstant = ComputeInstantMayorRating(cur);
    if (i == 0) {
      ratingEma = ratingInstant;
    } else {
      // Slow EMA to keep the number stable in UI.
      ratingEma = ratingEma * 0.92f + ratingInstant * 0.08f;
    }

    RNG rng(world.seed() ^ (static_cast<std::uint64_t>(cur.day) * 0x9E3779B97F4A7C15ULL) ^ 0xC1A7BEEF1234ULL);

    std::vector<Candidate> cands;
    cands.reserve(16);

    // Founding day.
    if (!prev) {
      Candidate c;
      c.score = 200.0f;
      c.tone = ChronicleTone::Neutral;
      c.headline = out.cityName + " breaks ground";
      c.body = "Survey crews mark the first districts and road corridors. Early decisions will shape the city's future.";
      c.tip = "Try connecting key neighborhoods with a reliable road spine, then zone Residential near parks and low traffic.";
      AddUniqueTag(c.tags, "founding");
      AddUniqueTag(c.tags, "planning");
      PushCandidate(cands, std::move(c));
    }

    // Incidents.
    if (cur.fireIncidentDestroyed > 0 || cur.fireIncidentDamaged > 0) {
      const std::string district = DistrictLabel(cur.fireIncidentDistrict, out.districtNames);
      Candidate c;
      c.tone = (cur.fireIncidentDestroyed > 0) ? ChronicleTone::Alert : ChronicleTone::Bad;
      c.score = 1000.0f + 70.0f * static_cast<float>(cur.fireIncidentDestroyed) +
                25.0f * static_cast<float>(cur.fireIncidentDamaged) +
                0.15f * static_cast<float>(std::max(0, cur.fireIncidentDisplaced));

      const int v = static_cast<int>(rng.rangeU32(3));
      if (v == 0) {
        c.headline = "Fire damages " + FormatCommaInt(cur.fireIncidentDamaged) + " buildings in " + district;
      } else if (v == 1) {
        c.headline = "Blaze sweeps " + district + "; " + FormatCommaInt(cur.fireIncidentDestroyed) + " destroyed";
      } else {
        c.headline = "Fire crews respond in " + district;
      }

      std::ostringstream body;
      body << "A fire incident occurred in " << district << ". ";
      if (cur.fireIncidentDestroyed > 0) {
        body << FormatCommaInt(cur.fireIncidentDestroyed) << " buildings were destroyed";
        if (cur.fireIncidentDamaged > 0) body << " and " << FormatCommaInt(cur.fireIncidentDamaged) << " were damaged";
        body << ". ";
      } else {
        body << FormatCommaInt(cur.fireIncidentDamaged) << " buildings were damaged. ";
      }
      if (cur.fireIncidentDisplaced > 0) body << "Displaced residents: " << FormatCommaInt(cur.fireIncidentDisplaced) << ". ";
      if (cur.fireIncidentJobsLostCap > 0) body << "Jobs capacity lost: ~" << FormatCommaInt(cur.fireIncidentJobsLostCap) << ". ";
      if (cur.fireIncidentOriginX >= 0 && cur.fireIncidentOriginY >= 0) {
        body << "Origin: (" << cur.fireIncidentOriginX << "," << cur.fireIncidentOriginY << "). ";
      }
      if (cur.fireIncidentCost > 0) body << "Response cost: " << FormatCommaInt(cur.fireIncidentCost) << ".";

      c.body = body.str();
      c.tip = "Reduce fire risk by spacing dense blocks, adding parks as firebreaks, and ensuring fire stations can reach hotspots quickly.";
      AddUniqueTag(c.tags, "fire");
      AddUniqueTag(c.tags, "incident");
      AddUniqueTag(c.tags, "safety");
      PushCandidate(cands, std::move(c));
    }

    if (cur.trafficIncidentInjuries > 0) {
      const std::string district = DistrictLabel(cur.trafficIncidentDistrict, out.districtNames);
      Candidate c;
      c.tone = (cur.trafficIncidentInjuries >= 10) ? ChronicleTone::Alert : ChronicleTone::Bad;
      c.score = 800.0f + 8.0f * static_cast<float>(cur.trafficIncidentInjuries) +
                0.02f * static_cast<float>(std::max(0, cur.trafficIncidentCost));

      const int v = static_cast<int>(rng.rangeU32(3));
      if (v == 0) {
        c.headline = "Crash injures " + FormatCommaInt(cur.trafficIncidentInjuries) + " in " + district;
      } else if (v == 1) {
        c.headline = "Traffic incident disrupts " + district;
      } else {
        c.headline = "Road safety concerns rise after incident";
      }

      std::ostringstream body;
      body << "A traffic incident was recorded in " << district << ". ";
      body << "Estimated injuries: " << FormatCommaInt(cur.trafficIncidentInjuries) << ". ";
      if (cur.trafficIncidentOriginX >= 0 && cur.trafficIncidentOriginY >= 0) {
        body << "Origin: (" << cur.trafficIncidentOriginX << "," << cur.trafficIncidentOriginY << "). ";
      }
      if (cur.trafficIncidentCost > 0) body << "Response cost: " << FormatCommaInt(cur.trafficIncidentCost) << ".";

      c.body = body.str();
      c.tip = "Mitigate crash risk by reducing congestion, upgrading critical intersections, and expanding transit to lower traffic volumes.";
      AddUniqueTag(c.tags, "traffic");
      AddUniqueTag(c.tags, "safety");
      AddUniqueTag(c.tags, "incident");
      PushCandidate(cands, std::move(c));
    }

    // Trade disruptions.
    if (cur.tradeImportDisrupted || cur.tradeExportDisrupted) {
      Candidate c;
      c.tone = ChronicleTone::Bad;
      c.score = 520.0f;
      if (cur.tradeImportDisrupted && cur.tradeExportDisrupted) {
        c.headline = "Trade routes disrupted: imports and exports hit";
      } else if (cur.tradeImportDisrupted) {
        c.headline = "Import disruption strains supply chains";
      } else {
        c.headline = "Export disruption squeezes local industry";
      }
      std::ostringstream body;
      body << "The trade market reported disruptions today. ";
      body << "Import cap " << cur.tradeImportCapacityPct << "%, export cap " << cur.tradeExportCapacityPct << "%. ";
      body << "Market index: " << std::fixed << std::setprecision(3) << static_cast<double>(cur.tradeMarketIndex) << ".";
      c.body = body.str();
      c.tip = "Outside connection reliability matters: add alternate road routes to the map edge and maintain a balanced R/C/I economy.";
      AddUniqueTag(c.tags, "trade");
      AddUniqueTag(c.tags, "economy");
      PushCandidate(cands, std::move(c));
    }

    // Economy events (start/end).
    {
      const EconomyEventKind ek = SafeEconomyKind(cur.economyEventKind);
      const EconomyEventKind pk = prev ? SafeEconomyKind(prev->economyEventKind) : EconomyEventKind::None;

      if (ek != pk) {
        Candidate c;
        if (ek != EconomyEventKind::None) {
          c.tone = ToneForEconomyEvent(ek);
          c.score = 460.0f;
          c.headline = PrettyEconomyEvent(ek) + " reaches " + out.cityName;
          c.body = "A new economy event is active: " + std::string(EconomyEventKindName(ek)) +
                   ". Remaining days: " + std::to_string(std::max(0, cur.economyEventDaysLeft)) +
                   ". Economy index: " + std::to_string(cur.economyIndex) + ".";
          c.tip = "During volatility, keep an eye on goods flow and budget stability; diversify zoning to reduce shocks.";
          AddUniqueTag(c.tags, "economy");
          AddUniqueTag(c.tags, "macro");
          PushCandidate(cands, std::move(c));
        } else if (pk != EconomyEventKind::None) {
          c.tone = (ToneForEconomyEvent(pk) == ChronicleTone::Bad) ? ChronicleTone::Good : ChronicleTone::Neutral;
          c.score = 280.0f;
          c.headline = PrettyEconomyEvent(pk) + " fades";
          c.body = "The active economy event has ended. Economy index: " + std::to_string(cur.economyIndex) + ".";
          c.tip = "Consider reassessing taxes and maintenance now that conditions have shifted.";
          AddUniqueTag(c.tags, "economy");
          AddUniqueTag(c.tags, "recovery");
          PushCandidate(cands, std::move(c));
        }
      }
    }

    // Population milestone or big change.
    if (prev) {
      const int milestone = FindPopulationMilestone(prev->population, cur.population);
      if (milestone > 0) {
        Candidate c;
        c.tone = ChronicleTone::Good;
        c.score = 260.0f + static_cast<float>(milestone) * 0.01f;
        c.headline = out.cityName + " reaches " + FormatCommaInt(milestone) + " residents";
        c.body = "A population milestone marks a new chapter for the city. Growth brings opportunity—and stress on services and roads.";
        c.tip = "Balance growth with infrastructure: watch commute times, goods satisfaction, and service coverage as density rises.";
        AddUniqueTag(c.tags, "growth");
        AddUniqueTag(c.tags, "milestone");
        PushCandidate(cands, std::move(c));
      } else {
        const int dPop = cur.population - prev->population;
        if (std::abs(dPop) >= std::max(25, prev->population / 10)) {
          Candidate c;
          c.tone = (dPop >= 0) ? ChronicleTone::Good : ChronicleTone::Bad;
          c.score = 240.0f + 0.5f * static_cast<float>(std::abs(dPop));
          c.headline = (dPop >= 0) ? ("Population surges by " + FormatCommaInt(dPop))
                                   : ("Population falls by " + FormatCommaInt(-dPop));
          c.body = "Rapid change can expose weak links in housing, jobs access, or quality of life.";
          c.tip = "If growth stalls, check congestion, goods delivery, and job accessibility; if booming, expand services and parks.";
          AddUniqueTag(c.tags, "growth");
          PushCandidate(cands, std::move(c));
        }
      }
    }

    // Infrastructure openings.
    if (prev) {
      const int dParks = cur.parks - prev->parks;
      if (dParks > 0) {
        Candidate c;
        c.tone = ChronicleTone::Good;
        c.score = 220.0f + 12.0f * static_cast<float>(dParks);
        const int v = static_cast<int>(rng.rangeU32(3));
        if (v == 0) c.headline = "New parks open across " + out.cityName;
        else if (v == 1) c.headline = "Green space expands: +" + FormatCommaInt(dParks) + " parks";
        else c.headline = "Residents celebrate new parkland";
        c.body = "Parks improve livability and can act as buffers for heat, noise, and fire spread.";
        c.tip = "Place parks near dense Residential or high-noise corridors for the biggest quality-of-life gains.";
        AddUniqueTag(c.tags, "parks");
        AddUniqueTag(c.tags, "livability");
        PushCandidate(cands, std::move(c));
      }

      const int dLines = cur.transitLines - prev->transitLines;
      if (dLines > 0) {
        Candidate c;
        c.tone = ChronicleTone::Good;
        c.score = 230.0f + 40.0f * static_cast<float>(dLines);
        c.headline = "Transit network grows: +" + FormatCommaInt(dLines) + " line" + (dLines == 1 ? "" : "s");
        c.body = "Expanded transit can reduce congestion and improve access to jobs.";
        c.tip = "Target high-demand corridors and ensure stops are spaced to cover dense residential blocks.";
        AddUniqueTag(c.tags, "transit");
        AddUniqueTag(c.tags, "mobility");
        PushCandidate(cands, std::move(c));
      }

      const int dRoads = cur.roads - prev->roads;
      if (dRoads > 0) {
        Candidate c;
        c.tone = ChronicleTone::Neutral;
        c.score = 170.0f + 1.2f * static_cast<float>(dRoads);
        c.headline = "Roadworks continue: +" + FormatCommaInt(dRoads) + " road tiles";
        c.body = "New connections can reduce travel times, but added capacity may attract more traffic.";
        c.tip = "Prefer resilient networks: multiple routes to the map edge and fewer single-point bottlenecks.";
        AddUniqueTag(c.tags, "roads");
        AddUniqueTag(c.tags, "infrastructure");
        PushCandidate(cands, std::move(c));
      }
    }

    // Goods/service shortfalls.
    if (cur.goodsDemand > 0) {
      const float goodsSat = Clamp01(cur.goodsSatisfaction);
      if (goodsSat < 0.80f || cur.goodsUnreachableDemand > 0) {
        Candidate c;
        c.tone = ChronicleTone::Bad;
        c.score = 380.0f + 500.0f * (1.0f - goodsSat) + 0.05f * static_cast<float>(cur.goodsUnreachableDemand);
        c.headline = "Shops report shortages as deliveries slip";
        std::ostringstream body;
        body << "Goods satisfaction fell to " << std::fixed << std::setprecision(3) << static_cast<double>(goodsSat)
             << " (delivered " << FormatCommaInt(cur.goodsDelivered) << "/" << FormatCommaInt(cur.goodsDemand) << ").";
        if (cur.goodsUnreachableDemand > 0) {
          body << " Unreachable demand: " << FormatCommaInt(cur.goodsUnreachableDemand) << ".";
        }
        c.body = body.str();
        c.tip = "Improve freight flow with clearer road access to Commercial zones and enough Industrial production capacity.";
        AddUniqueTag(c.tags, "goods");
        AddUniqueTag(c.tags, "logistics");
        PushCandidate(cands, std::move(c));
      }
    }

    const float svcSat = Clamp01(cur.servicesOverallSatisfaction);
    if (svcSat < 0.60f && cur.population > 0) {
      Candidate c;
      c.tone = ChronicleTone::Bad;
      c.score = 320.0f + 450.0f * (1.0f - svcSat);
      c.headline = "Service coverage strained as demand grows";
      std::ostringstream body;
      body << "Overall services satisfaction is " << std::fixed << std::setprecision(3) << static_cast<double>(svcSat)
           << " (edu " << cur.servicesEducationFacilities << ", health " << cur.servicesHealthFacilities
           << ", safety " << cur.servicesSafetyFacilities << ").";
      c.body = body.str();
      c.tip = "Add or upgrade facilities and ensure road access; service effectiveness depends on reachable coverage.";
      AddUniqueTag(c.tags, "services");
      AddUniqueTag(c.tags, "civic");
      PushCandidate(cands, std::move(c));
    }

    // Budget warning.
    if (cur.money < 0 || (prev && cur.expenses > cur.income && cur.population > 0 && cur.day > 0)) {
      Candidate c;
      c.tone = ChronicleTone::Bad;
      c.score = 260.0f + std::clamp(static_cast<float>(-cur.money) * 0.2f, 0.0f, 240.0f);
      c.headline = (cur.money < 0) ? "City budget in the red" : "Expenses outpace income";
      std::ostringstream body;
      body << "Income " << FormatCommaInt(cur.income) << ", expenses " << FormatCommaInt(cur.expenses)
           << ", maintenance " << FormatCommaInt(cur.maintenanceCost) << ".";
      c.body = body.str();
      c.tip = "Tune taxes, avoid overbuilding maintenance-heavy facilities, and keep goods/trade healthy to stabilize revenue.";
      AddUniqueTag(c.tags, "budget");
      AddUniqueTag(c.tags, "economy");
      PushCandidate(cands, std::move(c));
    }

    // Congestion and environmental alerts.
    if (cur.trafficCongestion > 0.35f) {
      Candidate c;
      c.tone = (cur.trafficCongestion > 0.65f) ? ChronicleTone::Bad : ChronicleTone::Neutral;
      c.score = 240.0f + 500.0f * (cur.trafficCongestion - 0.35f);
      c.headline = "Congestion rises on key corridors";
      std::ostringstream body;
      body << "Citywide congestion is " << std::fixed << std::setprecision(3) << static_cast<double>(cur.trafficCongestion)
           << " with " << FormatCommaInt(cur.congestedRoadTiles) << " congested tiles.";
      c.body = body.str();
      c.tip = "Try multiple routes between hubs, upgrade intersections, and increase transit coverage to shift mode share.";
      AddUniqueTag(c.tags, "traffic");
      AddUniqueTag(c.tags, "congestion");
      PushCandidate(cands, std::move(c));
    }

    if (cur.airPollutionResidentAvg01 > 0.60f && cur.airPollutionResidentPopulation > 0) {
      Candidate c;
      c.tone = ChronicleTone::Bad;
      c.score = 220.0f + 400.0f * (cur.airPollutionResidentAvg01 - 0.60f);
      c.headline = "Air quality worsens in residential areas";
      std::ostringstream body;
      body << "Resident-weighted exposure is " << std::fixed << std::setprecision(3)
           << static_cast<double>(cur.airPollutionResidentAvg01)
           << " (high exposure frac " << static_cast<double>(cur.airPollutionResidentHighExposureFrac) << ").";
      c.body = body.str();
      c.tip = "Reduce emissions by easing congestion, separating Industry from Residential, and adding green buffers.";
      AddUniqueTag(c.tags, "environment");
      AddUniqueTag(c.tags, "air");
      PushCandidate(cands, std::move(c));
    }

    // If nothing triggered, create a neutral daily brief.
    if (cands.empty()) {
      Candidate c;
      c.tone = ChronicleTone::Neutral;
      c.score = 50.0f;
      const int v = static_cast<int>(rng.rangeU32(3));
      if (v == 0) c.headline = "A quiet day in " + out.cityName;
      else if (v == 1) c.headline = "City Hall issues routine update";
      else c.headline = "Streets stay calm as plans continue";
      c.body = "No major incidents were recorded today. The city continues its steady march toward the next milestone.";
      c.tip = "Use calm periods to plan resilience: add alternate connections, improve goods flow, and invest in services.";
      AddUniqueTag(c.tags, "brief");
      PushCandidate(cands, std::move(c));
    }

    // Sort by score (desc), then headline for determinism.
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
      if (a.score != b.score) return a.score > b.score;
      return a.headline < b.headline;
    });

    // Pick the top story. Optionally include a secondary story if it is also salient.
    const int maxStories = 2;
    for (int si = 0; si < maxStories && si < static_cast<int>(cands.size()); ++si) {
      const Candidate& c = cands[static_cast<std::size_t>(si)];

      if (si == 1) {
        const float top = cands[0].score;
        if (!(c.score >= 220.0f && c.score >= top * 0.65f)) break;

        // Avoid duplicating the exact same tag-set as the lead story.
        if (c.tags == cands[0].tags) break;
      }

      ChronicleEntry e;
      e.day = cur.day;
      e.tone = c.tone;
      e.mayorRating = ratingEma;
      e.score = c.score;
      e.tags = c.tags;
      e.headline = c.headline;
      e.body = c.body;
      e.tip = c.tip;

      AppendDailyNumbers(e.body, cur, prev);

      out.entries.push_back(std::move(e));
    }
  }

  return out;
}

bool WriteCityChronicleJson(const std::string& path, const Chronicle& chronicle, std::string& outError)
{
  if (path.empty()) {
    outError = "Empty output path";
    return false;
  }

  try {
    const std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
  } catch (...) {
    outError = "Failed to create output directory";
    return false;
  }

  auto add = [](JsonValue& obj, const std::string& key, JsonValue v) {
    obj.objectValue.emplace_back(key, std::move(v));
  };

  JsonValue root = JsonValue::MakeObject();
  add(root, "version", JsonValue::MakeNumber(static_cast<double>(chronicle.version)));
  add(root, "seed", JsonValue::MakeNumber(static_cast<double>(chronicle.seed)));
  add(root, "seedHex", JsonValue::MakeString(HexU64(chronicle.seed)));
  add(root, "cityName", JsonValue::MakeString(chronicle.cityName));

  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(chronicle.districtNames.size());
    for (const std::string& s : chronicle.districtNames) {
      arr.arrayValue.push_back(JsonValue::MakeString(s));
    }
    add(root, "districtNames", std::move(arr));
  }

  {
    JsonValue arr = JsonValue::MakeArray();
    arr.arrayValue.reserve(chronicle.entries.size());

    for (const ChronicleEntry& e : chronicle.entries) {
      JsonValue o = JsonValue::MakeObject();
      add(o, "day", JsonValue::MakeNumber(static_cast<double>(e.day)));
      add(o, "tone", JsonValue::MakeString(ChronicleToneName(e.tone)));
      add(o, "mayorRating", JsonValue::MakeNumber(static_cast<double>(e.mayorRating)));
      add(o, "score", JsonValue::MakeNumber(static_cast<double>(e.score)));

      JsonValue tags = JsonValue::MakeArray();
      tags.arrayValue.reserve(e.tags.size());
      for (const std::string& t : e.tags) tags.arrayValue.push_back(JsonValue::MakeString(t));
      add(o, "tags", std::move(tags));

      add(o, "headline", JsonValue::MakeString(e.headline));
      add(o, "body", JsonValue::MakeString(e.body));
      add(o, "tip", JsonValue::MakeString(e.tip));

      arr.arrayValue.push_back(std::move(o));
    }

    add(root, "entries", std::move(arr));
  }

  JsonWriteOptions opt;
  opt.pretty = true;
  opt.indent = 2;
  opt.sortKeys = false;

  return WriteJsonFile(path, root, outError, opt);
}

bool WriteCityChronicleMarkdown(const std::string& path, const Chronicle& chronicle, std::string& outError)
{
  if (path.empty()) {
    outError = "Empty output path";
    return false;
  }

  try {
    const std::filesystem::path p(path);
    const std::filesystem::path parent = p.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
  } catch (...) {
    outError = "Failed to create output directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Failed to open for writing";
    return false;
  }

  f << "# City Chronicle: " << chronicle.cityName << "\n\n";
  f << "Seed: `" << HexU64(chronicle.seed) << "`\n\n";

  int curDay = std::numeric_limits<int>::min();
  for (const ChronicleEntry& e : chronicle.entries) {
    if (e.day != curDay) {
      curDay = e.day;
      f << "---\n\n";
      f << "## Day " << e.day << "\n\n";
    }

    f << "### " << e.headline << "\n\n";
    f << "- Tone: `" << ChronicleToneName(e.tone) << "`\n";
    f << "- Mayor rating: " << std::fixed << std::setprecision(1) << static_cast<double>(e.mayorRating) << "\n";
    if (!e.tags.empty()) {
      f << "- Tags: ";
      for (std::size_t i = 0; i < e.tags.size(); ++i) {
        f << '`' << e.tags[i] << '`';
        if (i + 1 < e.tags.size()) f << ", ";
      }
      f << "\n";
    }
    f << "\n";

    if (!e.body.empty()) {
      f << e.body << "\n\n";
    }
    if (!e.tip.empty()) {
      f << "**Tip:** " << e.tip << "\n\n";
    }
  }

  if (!f) {
    outError = "Failed while writing markdown";
    return false;
  }

  outError.clear();
  return true;
}

} // namespace isocity
