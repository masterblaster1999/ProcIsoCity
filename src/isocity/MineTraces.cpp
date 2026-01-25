#include "isocity/MineTraces.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace isocity {

namespace {

static std::string NormalizeKey(const std::string& s)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    unsigned char uc = static_cast<unsigned char>(c);
    char out = static_cast<char>(std::tolower(uc));
    if (out == '-' || out == ' ' || out == '.') out = '_';
    t.push_back(out);
  }
  return t;
}

static double FiniteOrZero(double v)
{
  return std::isfinite(v) ? v : 0.0;
}

} // namespace

const char* MineTraceMetricName(MineTraceMetric m)
{
  switch (m) {
  case MineTraceMetric::Population: return "population";
  case MineTraceMetric::Happiness: return "happiness";
  case MineTraceMetric::Money: return "money";
  case MineTraceMetric::AvgLandValue: return "avg_land_value";
  case MineTraceMetric::TrafficCongestion: return "traffic_congestion";
  case MineTraceMetric::GoodsSatisfaction: return "goods_satisfaction";
  case MineTraceMetric::ServicesOverallSatisfaction: return "services_overall_satisfaction";
  case MineTraceMetric::TransitModeShare: return "transit_mode_share";
  case MineTraceMetric::AvgCommuteTime: return "avg_commute_time";
  case MineTraceMetric::EconomyIndex: return "economy_index";
  case MineTraceMetric::TradeMarketIndex: return "trade_market_index";
  }
  return "population";
}

bool ParseMineTraceMetric(const std::string& s, MineTraceMetric& out)
{
  const std::string k = NormalizeKey(s);

  if (k == "population" || k == "pop") {
    out = MineTraceMetric::Population;
    return true;
  }
  if (k == "happiness" || k == "happy") {
    out = MineTraceMetric::Happiness;
    return true;
  }
  if (k == "money" || k == "cash") {
    out = MineTraceMetric::Money;
    return true;
  }
  if (k == "avg_land_value" || k == "land_value" || k == "landvalue" || k == "lv") {
    out = MineTraceMetric::AvgLandValue;
    return true;
  }
  if (k == "traffic_congestion" || k == "congestion" || k == "cong" || k == "traffic") {
    out = MineTraceMetric::TrafficCongestion;
    return true;
  }
  if (k == "goods_satisfaction" || k == "goods" || k == "goods_sat") {
    out = MineTraceMetric::GoodsSatisfaction;
    return true;
  }
  if (k == "services_overall_satisfaction" || k == "services" || k == "service" || k == "services_sat") {
    out = MineTraceMetric::ServicesOverallSatisfaction;
    return true;
  }
  if (k == "transit_mode_share" || k == "transit" || k == "mode_share" || k == "transit_share") {
    out = MineTraceMetric::TransitModeShare;
    return true;
  }
  if (k == "avg_commute_time" || k == "commute_time" || k == "commute") {
    out = MineTraceMetric::AvgCommuteTime;
    return true;
  }
  if (k == "economy_index" || k == "economy") {
    out = MineTraceMetric::EconomyIndex;
    return true;
  }
  if (k == "trade_market_index" || k == "trade" || k == "market") {
    out = MineTraceMetric::TradeMarketIndex;
    return true;
  }

  return false;
}

bool ParseMineTraceMetricList(const std::string& csv,
                             std::vector<MineTraceMetric>& outMetrics,
                             std::string* outError)
{
  outMetrics.clear();
  if (outError) outError->clear();
  if (csv.empty()) return true;

  std::vector<MineTraceMetric> tmp;
  std::size_t start = 0;
  while (start < csv.size()) {
    std::size_t end = csv.find(',', start);
    if (end == std::string::npos) end = csv.size();

    std::string token = csv.substr(start, end - start);
    // Trim.
    while (!token.empty() && std::isspace(static_cast<unsigned char>(token.front()))) token.erase(token.begin());
    while (!token.empty() && std::isspace(static_cast<unsigned char>(token.back()))) token.pop_back();

    if (!token.empty()) {
      MineTraceMetric m;
      if (!ParseMineTraceMetric(token, m)) {
        if (outError) {
          *outError = "Unknown trace metric: '" + token + "'";
        }
        outMetrics.clear();
        return false;
      }
      tmp.push_back(m);
    }

    start = end + 1;
  }

  // Deduplicate while preserving order.
  for (MineTraceMetric m : tmp) {
    if (std::find(outMetrics.begin(), outMetrics.end(), m) == outMetrics.end()) {
      outMetrics.push_back(m);
    }
  }
  return true;
}

std::vector<MineTraceMetric> DefaultMineTraceMetrics()
{
  return {
      MineTraceMetric::Population,
      MineTraceMetric::Happiness,
      MineTraceMetric::TrafficCongestion,
      MineTraceMetric::Money,
  };
}

double MineTraceMetricValue(const Stats& s, MineTraceMetric m)
{
  switch (m) {
  case MineTraceMetric::Population: return static_cast<double>(s.population);
  case MineTraceMetric::Happiness: return FiniteOrZero(static_cast<double>(s.happiness));
  case MineTraceMetric::Money: return static_cast<double>(s.money);
  case MineTraceMetric::AvgLandValue: return FiniteOrZero(static_cast<double>(s.avgLandValue));
  case MineTraceMetric::TrafficCongestion: return FiniteOrZero(static_cast<double>(s.trafficCongestion));
  case MineTraceMetric::GoodsSatisfaction: return FiniteOrZero(static_cast<double>(s.goodsSatisfaction));
  case MineTraceMetric::ServicesOverallSatisfaction: return FiniteOrZero(static_cast<double>(s.servicesOverallSatisfaction));
  case MineTraceMetric::TransitModeShare: return FiniteOrZero(static_cast<double>(s.transitModeShare));
  case MineTraceMetric::AvgCommuteTime: return FiniteOrZero(static_cast<double>(s.avgCommuteTime));
  case MineTraceMetric::EconomyIndex: return FiniteOrZero(static_cast<double>(s.economyIndex));
  case MineTraceMetric::TradeMarketIndex: return FiniteOrZero(static_cast<double>(s.tradeMarketIndex));
  }

  return 0.0;
}

} // namespace isocity
