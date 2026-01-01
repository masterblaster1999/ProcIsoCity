#include "isocity/PolicyOptimizerExport.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ostream>

namespace isocity {

namespace fs = std::filesystem;

namespace {

bool EnsureParentDir(const std::string& path)
{
  fs::path p(path);
  fs::path parent = p.parent_path();
  if (parent.empty()) return true;
  std::error_code ec;
  fs::create_directories(parent, ec);
  return !ec;
}

const char* MethodName(PolicyOptMethod m)
{
  switch (m) {
    case PolicyOptMethod::Exhaustive: return "exhaustive";
    case PolicyOptMethod::CEM: return "cem";
    default: return "unknown";
  }
}

void WritePolicyJson(std::ostream& os, const PolicyCandidate& p)
{
  os << "{"
     << "\"taxResidential\": " << p.taxResidential << ", "
     << "\"taxCommercial\": " << p.taxCommercial << ", "
     << "\"taxIndustrial\": " << p.taxIndustrial << ", "
     << "\"maintenanceRoad\": " << p.maintenanceRoad << ", "
     << "\"maintenancePark\": " << p.maintenancePark
     << "}";
}

void WriteMetricsJson(std::ostream& os, const PolicyEvalMetrics& m)
{
  os << "{";
  os << "\"daysSimulated\": " << m.daysSimulated << ", ";
  os << "\"moneyStart\": " << m.moneyStart << ", ";
  os << "\"moneyEnd\": " << m.moneyEnd << ", ";
  os << "\"moneyDelta\": " << m.moneyDelta << ", ";
  os << "\"populationEnd\": " << m.populationEnd << ", ";
  os << "\"employedEnd\": " << m.employedEnd << ", ";
  os << "\"jobsCapacityAccessibleEnd\": " << m.jobsCapacityAccessibleEnd << ", ";
  os << "\"happinessEnd\": " << std::fixed << std::setprecision(6) << m.happinessEnd << ", ";
  os << "\"avgHappiness\": " << std::fixed << std::setprecision(6) << m.avgHappiness << ", ";
  os << "\"demandResidentialEnd\": " << std::fixed << std::setprecision(6) << m.demandResidentialEnd << ", ";
  os << "\"avgLandValueEnd\": " << std::fixed << std::setprecision(6) << m.avgLandValueEnd << ", ";
  os << "\"avgCommuteTimeEnd\": " << std::fixed << std::setprecision(6) << m.avgCommuteTimeEnd << ", ";
  os << "\"trafficCongestionEnd\": " << std::fixed << std::setprecision(6) << m.trafficCongestionEnd << ", ";
  os << "\"avgNetPerDay\": " << std::fixed << std::setprecision(6) << m.avgNetPerDay;
  os << "}";
}

void WriteEvalJson(std::ostream& os, const PolicyEvalResult& r)
{
  os << "{";
  os << "\"policy\": ";
  WritePolicyJson(os, r.policy);
  os << ", ";
  os << "\"metrics\": ";
  WriteMetricsJson(os, r.metrics);
  os << ", ";
  os << "\"score\": " << std::fixed << std::setprecision(9) << r.score;
  os << "}";
}

void WriteSpaceJson(std::ostream& os, const PolicySearchSpace& s)
{
  os << "{";
  os << "\"taxResidential\": [" << s.taxResMin << ", " << s.taxResMax << "], ";
  os << "\"taxCommercial\": [" << s.taxComMin << ", " << s.taxComMax << "], ";
  os << "\"taxIndustrial\": [" << s.taxIndMin << ", " << s.taxIndMax << "], ";
  os << "\"maintenanceRoad\": [" << s.maintRoadMin << ", " << s.maintRoadMax << "], ";
  os << "\"maintenancePark\": [" << s.maintParkMin << ", " << s.maintParkMax << "]";
  os << "}";
}

} // namespace

bool WritePolicyOptimizationJson(std::ostream& os, const PolicyOptimizationResult& r, const PolicyOptimizerConfig& cfg,
                                const PolicySearchSpace& space, const PolicyEvalResult* baseline, bool includeTop,
                                std::string* outError)
{
  if (outError) outError->clear();

  os << "{\n";
  os << "  \"method\": \"" << MethodName(r.methodUsed) << "\",\n";
  os << "  \"evalDays\": " << cfg.evalDays << ",\n";
  os << "  \"candidatesEvaluated\": " << r.candidatesEvaluated << ",\n";
  os << "  \"iterationsCompleted\": " << r.iterationsCompleted << ",\n";

  os << "  \"space\": ";
  WriteSpaceJson(os, space);
  os << ",\n";

  os << "  \"objective\": {";
  os << "\"wMoneyDelta\": " << std::fixed << std::setprecision(6) << cfg.objective.wMoneyDelta << ", ";
  os << "\"wPopulation\": " << std::fixed << std::setprecision(6) << cfg.objective.wPopulation << ", ";
  os << "\"wHappyPop\": " << std::fixed << std::setprecision(6) << cfg.objective.wHappyPop << ", ";
  os << "\"wUnemployed\": " << std::fixed << std::setprecision(6) << cfg.objective.wUnemployed << ", ";
  os << "\"wCongestionPop\": " << std::fixed << std::setprecision(6) << cfg.objective.wCongestionPop << ", ";
  os << "\"minHappiness\": " << std::fixed << std::setprecision(6) << cfg.objective.minHappiness << ", ";
  os << "\"minMoneyEnd\": " << cfg.objective.minMoneyEnd;
  os << "},\n";

  if (baseline) {
    os << "  \"baseline\": ";
    WriteEvalJson(os, *baseline);
    os << ",\n";
  }

  os << "  \"best\": ";
  WriteEvalJson(os, r.best);
  os << ",\n";

  if (includeTop) {
    os << "  \"top\": [\n";
    for (std::size_t i = 0; i < r.top.size(); ++i) {
      os << "    ";
      WriteEvalJson(os, r.top[i]);
      if (i + 1 != r.top.size()) os << ",";
      os << "\n";
    }
    os << "  ],\n";
  }

  // Trace (best-by-iter + dist).
  os << "  \"trace\": {\n";
  os << "    \"bestByIteration\": [\n";
  for (std::size_t i = 0; i < r.bestByIteration.size(); ++i) {
    os << "      ";
    WriteEvalJson(os, r.bestByIteration[i]);
    if (i + 1 != r.bestByIteration.size()) os << ",";
    os << "\n";
  }
  os << "    ],\n";

  os << "    \"distByIteration\": [\n";
  for (std::size_t i = 0; i < r.distByIteration.size(); ++i) {
    const PolicyDistribution& d = r.distByIteration[i];
    os << "      {";
    os << "\"meanTaxResidential\": " << std::fixed << std::setprecision(6) << d.meanTaxResidential << ", "
       << "\"stdTaxResidential\": " << std::fixed << std::setprecision(6) << d.stdTaxResidential << ", "
       << "\"meanTaxCommercial\": " << std::fixed << std::setprecision(6) << d.meanTaxCommercial << ", "
       << "\"stdTaxCommercial\": " << std::fixed << std::setprecision(6) << d.stdTaxCommercial << ", "
       << "\"meanTaxIndustrial\": " << std::fixed << std::setprecision(6) << d.meanTaxIndustrial << ", "
       << "\"stdTaxIndustrial\": " << std::fixed << std::setprecision(6) << d.stdTaxIndustrial << ", "
       << "\"meanMaintRoad\": " << std::fixed << std::setprecision(6) << d.meanMaintRoad << ", "
       << "\"stdMaintRoad\": " << std::fixed << std::setprecision(6) << d.stdMaintRoad << ", "
       << "\"meanMaintPark\": " << std::fixed << std::setprecision(6) << d.meanMaintPark << ", "
       << "\"stdMaintPark\": " << std::fixed << std::setprecision(6) << d.stdMaintPark;
    os << "}";
    if (i + 1 != r.distByIteration.size()) os << ",";
    os << "\n";
  }
  os << "    ]\n";
  os << "  }\n";

  os << "}\n";

  return true;
}

bool ExportPolicyOptimizationJson(const std::string& path, const PolicyOptimizationResult& r, const PolicyOptimizerConfig& cfg,
                                 const PolicySearchSpace& space, const PolicyEvalResult* baseline, bool includeTop,
                                 std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }

  return WritePolicyOptimizationJson(f, r, cfg, space, baseline, includeTop, outError);
}

bool ExportPolicyOptimizationTopCsv(const std::string& path, const PolicyOptimizationResult& r, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }

  f << "rank,score,daysSimulated,moneyStart,moneyEnd,moneyDelta,avgNetPerDay,"
    << "populationEnd,employedEnd,jobsCapacityAccessibleEnd,"
    << "happinessEnd,avgHappiness,demandResidentialEnd,avgLandValueEnd,avgCommuteTimeEnd,trafficCongestionEnd,"
    << "taxResidential,taxCommercial,taxIndustrial,maintenanceRoad,maintenancePark\n";

  for (std::size_t i = 0; i < r.top.size(); ++i) {
    const PolicyEvalResult& e = r.top[i];
    const PolicyEvalMetrics& m = e.metrics;
    f << (i + 1) << ","
      << std::fixed << std::setprecision(9) << e.score << ","
      << m.daysSimulated << ","
      << m.moneyStart << ","
      << m.moneyEnd << ","
      << m.moneyDelta << ","
      << std::fixed << std::setprecision(6) << m.avgNetPerDay << ","
      << m.populationEnd << ","
      << m.employedEnd << ","
      << m.jobsCapacityAccessibleEnd << ","
      << std::fixed << std::setprecision(6) << m.happinessEnd << ","
      << std::fixed << std::setprecision(6) << m.avgHappiness << ","
      << std::fixed << std::setprecision(6) << m.demandResidentialEnd << ","
      << std::fixed << std::setprecision(6) << m.avgLandValueEnd << ","
      << std::fixed << std::setprecision(6) << m.avgCommuteTimeEnd << ","
      << std::fixed << std::setprecision(6) << m.trafficCongestionEnd << ","
      << e.policy.taxResidential << ","
      << e.policy.taxCommercial << ","
      << e.policy.taxIndustrial << ","
      << e.policy.maintenanceRoad << ","
      << e.policy.maintenancePark
      << "\n";
  }

  return true;
}

bool ExportPolicyOptimizationTraceCsv(const std::string& path, const PolicyOptimizationResult& r, std::string* outError)
{
  if (outError) outError->clear();
  if (!EnsureParentDir(path)) {
    if (outError) *outError = "failed to create parent directory";
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    if (outError) *outError = "failed to open for writing: " + path;
    return false;
  }

  f << "iter,bestScore,bestMoneyDelta,bestPopulation,bestAvgHappiness,"
    << "meanTaxResidential,stdTaxResidential,meanTaxCommercial,stdTaxCommercial,meanTaxIndustrial,stdTaxIndustrial,"
    << "meanMaintRoad,stdMaintRoad,meanMaintPark,stdMaintPark\n";

  const std::size_t n = std::max(r.bestByIteration.size(), r.distByIteration.size());
  for (std::size_t i = 0; i < n; ++i) {
    const PolicyEvalResult* best = (i < r.bestByIteration.size()) ? &r.bestByIteration[i] : nullptr;
    const PolicyDistribution* dist = (i < r.distByIteration.size()) ? &r.distByIteration[i] : nullptr;

    f << (i + 1) << ",";
    if (best) {
      f << std::fixed << std::setprecision(9) << best->score << ","
        << best->metrics.moneyDelta << ","
        << best->metrics.populationEnd << ","
        << std::fixed << std::setprecision(6) << best->metrics.avgHappiness;
    } else {
      f << ",,,";
    }

    f << ",";

    if (dist) {
      f << std::fixed << std::setprecision(6) << dist->meanTaxResidential << ","
        << std::fixed << std::setprecision(6) << dist->stdTaxResidential << ","
        << std::fixed << std::setprecision(6) << dist->meanTaxCommercial << ","
        << std::fixed << std::setprecision(6) << dist->stdTaxCommercial << ","
        << std::fixed << std::setprecision(6) << dist->meanTaxIndustrial << ","
        << std::fixed << std::setprecision(6) << dist->stdTaxIndustrial << ","
        << std::fixed << std::setprecision(6) << dist->meanMaintRoad << ","
        << std::fixed << std::setprecision(6) << dist->stdMaintRoad << ","
        << std::fixed << std::setprecision(6) << dist->meanMaintPark << ","
        << std::fixed << std::setprecision(6) << dist->stdMaintPark;
    } else {
      f << ",,,,,,,,,,";
    }

    f << "\n";
  }

  return true;
}

} // namespace isocity
