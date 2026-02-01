#pragma once

#include "isocity/World.hpp"

#include <ostream>
#include <string_view>

namespace isocity {

// A shared CSV schema for writing per-tick Stats time series.
//
// Notes:
// - The first columns preserve the historical ordering used by multiple CLI tools.
// - Additional derived stats are appended so older consumers that read only the first
//   N columns remain compatible.
inline constexpr std::string_view kStatsCsvHeader =
    "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,"
    "avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,"
    "demandResidential,demandCommercial,demandIndustrial,"
    "commuters,commutersUnreachable,avgCommute,p95Commute,p95CommuteTime,congestedRoadTiles,maxRoadTraffic,"
    "trafficSafetyRoadTilesConsidered,trafficSafetyResidentPopulation,trafficSafetyResidentMeanExposure,"
    "trafficSafetyResidentMeanPriority,trafficSafetyHappinessPenalty,trafficSafetyHotspotX,trafficSafetyHotspotY,"
    "trafficSafetyHotspotDistrict,trafficSafetyHotspotRisk01,"
    "transitLines,transitStops,transitRiders,transitModeShare,transitCommuteCoverage,"
    "servicesEducationFacilities,servicesHealthFacilities,servicesSafetyFacilities,servicesEducationSatisfaction,"
    "servicesHealthSatisfaction,servicesSafetySatisfaction,servicesOverallSatisfaction,servicesMaintenanceCost,"
    "goodsProduced,goodsImported,goodsExported,goodsUnreachableDemand,maxRoadGoodsTraffic,"
    "tradeImportPartner,tradeExportPartner,tradeImportCapacityPct,tradeExportCapacityPct,tradeImportDisrupted,"
    "tradeExportDisrupted,tradeMarketIndex,"
    "economyIndex,economyInflation,economyEventKind,economyEventDaysLeft,economyCityWealth,"
    "income,expenses,taxRevenue,maintenanceCost,upgradeCost,importCost,exportRevenue,avgTaxPerCapita,transitCost,"
    "airPollutionResidentPopulation,airPollutionResidentAvg01,airPollutionResidentHighExposureFrac,"
    "airPollutionHappinessPenalty,"
    "fireIncidentDamaged,fireIncidentDestroyed,fireIncidentDisplaced,fireIncidentJobsLostCap,fireIncidentCost,"
    "fireIncidentHappinessPenalty,fireIncidentOriginX,fireIncidentOriginY,fireIncidentDistrict,"
    "trafficIncidentInjuries,trafficIncidentCost,trafficIncidentHappinessPenalty,trafficIncidentOriginX,"
    "trafficIncidentOriginY,trafficIncidentDistrict";

inline bool WriteStatsCsvHeader(std::ostream& os)
{
  os << kStatsCsvHeader << '\n';
  return static_cast<bool>(os);
}

inline bool WriteStatsCsvRow(std::ostream& os, const Stats& s)
{
  os << s.day << ','
     << s.population << ','
     << s.money << ','
     << s.housingCapacity << ','
     << s.jobsCapacity << ','
     << s.jobsCapacityAccessible << ','
     << s.employed << ','
     << s.happiness << ','
     << s.roads << ','
     << s.parks << ','
     << s.avgCommuteTime << ','
     << s.trafficCongestion << ','
     << s.goodsDemand << ','
     << s.goodsDelivered << ','
     << s.goodsSatisfaction << ','
     << s.avgLandValue << ','
     << s.demandResidential << ','
     << s.demandCommercial << ','
     << s.demandIndustrial << ','
     << s.commuters << ','
     << s.commutersUnreachable << ','
     << s.avgCommute << ','
     << s.p95Commute << ','
     << s.p95CommuteTime << ','
     << s.congestedRoadTiles << ','
     << s.maxRoadTraffic << ','
     << s.trafficSafetyRoadTilesConsidered << ','
     << s.trafficSafetyResidentPopulation << ','
     << s.trafficSafetyResidentMeanExposure << ','
     << s.trafficSafetyResidentMeanPriority << ','
     << s.trafficSafetyHappinessPenalty << ','
     << s.trafficSafetyHotspotX << ','
     << s.trafficSafetyHotspotY << ','
     << s.trafficSafetyHotspotDistrict << ','
     << s.trafficSafetyHotspotRisk01 << ','
     << s.transitLines << ','
     << s.transitStops << ','
     << s.transitRiders << ','
     << s.transitModeShare << ','
     << s.transitCommuteCoverage << ','
     << s.servicesEducationFacilities << ','
     << s.servicesHealthFacilities << ','
     << s.servicesSafetyFacilities << ','
     << s.servicesEducationSatisfaction << ','
     << s.servicesHealthSatisfaction << ','
     << s.servicesSafetySatisfaction << ','
     << s.servicesOverallSatisfaction << ','
     << s.servicesMaintenanceCost << ','
     << s.goodsProduced << ','
     << s.goodsImported << ','
     << s.goodsExported << ','
     << s.goodsUnreachableDemand << ','
     << s.maxRoadGoodsTraffic << ','
     << s.tradeImportPartner << ','
     << s.tradeExportPartner << ','
     << s.tradeImportCapacityPct << ','
     << s.tradeExportCapacityPct << ','
     << s.tradeImportDisrupted << ','
     << s.tradeExportDisrupted << ','
     << s.tradeMarketIndex << ','
     << s.economyIndex << ','
     << s.economyInflation << ','
     << s.economyEventKind << ','
     << s.economyEventDaysLeft << ','
     << s.economyCityWealth << ','
     << s.income << ','
     << s.expenses << ','
     << s.taxRevenue << ','
     << s.maintenanceCost << ','
     << s.upgradeCost << ','
     << s.importCost << ','
     << s.exportRevenue << ','
     << s.avgTaxPerCapita << ','
     << s.transitCost << ','
     << s.airPollutionResidentPopulation << ','
     << s.airPollutionResidentAvg01 << ','
     << s.airPollutionResidentHighExposureFrac << ','
     << s.airPollutionHappinessPenalty << ','
     << s.fireIncidentDamaged << ','
     << s.fireIncidentDestroyed << ','
     << s.fireIncidentDisplaced << ','
     << s.fireIncidentJobsLostCap << ','
     << s.fireIncidentCost << ','
     << s.fireIncidentHappinessPenalty << ','
     << s.fireIncidentOriginX << ','
     << s.fireIncidentOriginY << ','
     << s.fireIncidentDistrict << ','
     << s.trafficIncidentInjuries << ','
     << s.trafficIncidentCost << ','
     << s.trafficIncidentHappinessPenalty << ','
     << s.trafficIncidentOriginX << ','
     << s.trafficIncidentOriginY << ','
     << s.trafficIncidentDistrict
     << '\n';
  return static_cast<bool>(os);
}

} // namespace isocity
