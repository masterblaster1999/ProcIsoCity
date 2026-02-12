#include "isocity/ProcGen.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace isocity {

static std::string LowerCopy(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

const char* ToString(ProcGenTerrainPreset p)
{
  switch (p) {
  case ProcGenTerrainPreset::Classic: return "classic";
  case ProcGenTerrainPreset::Island: return "island";
  case ProcGenTerrainPreset::Archipelago: return "archipelago";
  case ProcGenTerrainPreset::InlandSea: return "inland_sea";
  case ProcGenTerrainPreset::RiverValley: return "river_valley";
  case ProcGenTerrainPreset::MountainRing: return "mountain_ring";
  case ProcGenTerrainPreset::Fjords: return "fjords";
  case ProcGenTerrainPreset::Canyon: return "canyon";
  case ProcGenTerrainPreset::Volcano: return "volcano";
  case ProcGenTerrainPreset::Delta: return "delta";
  case ProcGenTerrainPreset::Tectonic: return "tectonic";
  case ProcGenTerrainPreset::Atoll: return "atoll";
  case ProcGenTerrainPreset::Peninsula: return "peninsula";
  default: return "classic";
  }
}

bool ParseProcGenTerrainPreset(const std::string& s, ProcGenTerrainPreset& out)
{
  const std::string t = LowerCopy(s);
  if (t.empty()) return false;

  auto eq = [&](const char* a) { return t == a; };

  if (eq("classic") || eq("default") || eq("continent") || eq("continental")) {
    out = ProcGenTerrainPreset::Classic;
    return true;
  }
  if (eq("island") || eq("islands")) {
    out = ProcGenTerrainPreset::Island;
    return true;
  }
  if (eq("archipelago") || eq("arch") || eq("isle") || eq("isles")) {
    out = ProcGenTerrainPreset::Archipelago;
    return true;
  }
  if (eq("inlandsea") || eq("inland_sea") || eq("sea") || eq("lake") || eq("inlandse") || eq("inland")) {
    out = ProcGenTerrainPreset::InlandSea;
    return true;
  }
  if (eq("river") || eq("rivervalley") || eq("river_valley") || eq("valley") || eq("river-valley")) {
    out = ProcGenTerrainPreset::RiverValley;
    return true;
  }
  if (eq("mountain") || eq("mountains") || eq("ring") || eq("mountainring") || eq("mountain_ring") ||
      eq("crater")) {
    out = ProcGenTerrainPreset::MountainRing;
    return true;
  }

  if (eq("fjord") || eq("fjords") || eq("glacier") || eq("glacial") || eq("inlet") || eq("inlets")) {
    out = ProcGenTerrainPreset::Fjords;
    return true;
  }

  if (eq("canyon") || eq("canyons") || eq("gorge") || eq("gorges") || eq("grandcanyon") ||
      eq("grand_canyon") || eq("grand-canyon")) {
    out = ProcGenTerrainPreset::Canyon;
    return true;
  }

  if (eq("volcano") || eq("volcanic") || eq("caldera") || eq("crater_lake") || eq("craterlake") ||
      eq("crater-lake")) {
    out = ProcGenTerrainPreset::Volcano;
    return true;
  }

  if (eq("delta") || eq("riverdelta") || eq("river_delta") || eq("river-delta") || eq("floodplain") ||
      eq("wetlands") || eq("marsh") || eq("marshes")) {
    out = ProcGenTerrainPreset::Delta;
    return true;
  }

  if (eq("tectonic") || eq("plates") || eq("plate") || eq("plate_tectonics") || eq("plate-tectonics") ||
      eq("tectonic_plate") || eq("tectonic_plates") || eq("ranges") || eq("mountain_ranges")) {
    out = ProcGenTerrainPreset::Tectonic;
    return true;
  }

  if (eq("peninsula") || eq("cape") || eq("promontory") || eq("headland") || eq("spit") ||
      eq("finger") || eq("landfinger") || eq("land_finger") || eq("land-finger") ||
      eq("strait") || eq("straits")) {
    out = ProcGenTerrainPreset::Peninsula;
    return true;
  }

  if (eq("atoll") || eq("reef") || eq("ring_island") || eq("ring-island") || eq("lagoon") ||
      eq("coral") || eq("coral_reef") || eq("coral-reef")) {
    out = ProcGenTerrainPreset::Atoll;
    return true;
  }

  return false;
}

const char* ToString(ProcGenDistrictingMode m)
{
  switch (m) {
  case ProcGenDistrictingMode::Voronoi: return "voronoi";
  case ProcGenDistrictingMode::RoadFlow: return "road_flow";
  case ProcGenDistrictingMode::BlockGraph: return "block_graph";
  case ProcGenDistrictingMode::Watershed: return "watershed";
  default: return "voronoi";
  }
}

bool ParseProcGenDistrictingMode(const std::string& s, ProcGenDistrictingMode& out)
{
  const std::string t = LowerCopy(s);
  if (t.empty()) return false;

  auto eq = [&](const char* a) { return t == a; };

  if (eq("voronoi") || eq("legacy") || eq("tile") || eq("tiles") || eq("tile_voronoi") || eq("tile-voronoi")) {
    out = ProcGenDistrictingMode::Voronoi;
    return true;
  }

  if (eq("road") || eq("roads") || eq("roadflow") || eq("road_flow") || eq("road-flow") || eq("flow") ||
      eq("auto") || eq("travel") || eq("traveltime") || eq("travel_time") || eq("travel-time")) {
    out = ProcGenDistrictingMode::RoadFlow;
    return true;
  }

  if (eq("block") || eq("blocks") || eq("blockgraph") || eq("block_graph") || eq("block-graph") ||
      eq("neighborhood") || eq("neighbourhood") || eq("neighborhoods") || eq("neighbourhoods")) {
    out = ProcGenDistrictingMode::BlockGraph;
    return true;
  }

  if (eq("watershed") || eq("watersheds") || eq("basin") || eq("basins") || eq("hydrology") ||
      eq("drainage") || eq("drainage_basin") || eq("drainage-basin") || eq("catchment") || eq("catchments") ||
      eq("river_basin") || eq("river-basin")) {
    out = ProcGenDistrictingMode::Watershed;
    return true;
  }

  return false;
}

const char* ToString(ProcGenRoadLayout m)
{
  switch (m) {
  case ProcGenRoadLayout::Organic: return "organic";
  case ProcGenRoadLayout::Grid: return "grid";
  case ProcGenRoadLayout::Radial: return "radial";
  case ProcGenRoadLayout::SpaceColonization: return "space_colonization";
  case ProcGenRoadLayout::VoronoiCells: return "voronoi_cells";
  case ProcGenRoadLayout::Physarum: return "physarum";
  case ProcGenRoadLayout::MedialAxis: return "medial_axis";
  case ProcGenRoadLayout::TensorField: return "tensor_field";
  default: return "organic";
  }
}

bool ParseProcGenRoadLayout(const std::string& s, ProcGenRoadLayout& out)
{
  const std::string t = LowerCopy(s);
  if (t.empty()) return false;

  auto eq = [&](const char* a) { return t == a; };

  if (eq("organic") || eq("org") || eq("classic") || eq("legacy") || eq("mst") || eq("default")) {
    out = ProcGenRoadLayout::Organic;
    return true;
  }

  if (eq("grid") || eq("manhattan") || eq("orthogonal") || eq("rect") || eq("rectilinear")) {
    out = ProcGenRoadLayout::Grid;
    return true;
  }

  if (eq("radial") || eq("ring") || eq("spoke") || eq("spokes") || eq("hubspoke") || eq("hub_spoke") ||
      eq("hub-and-spoke") || eq("hub_and_spoke")) {
    out = ProcGenRoadLayout::Radial;
    return true;
  }

  if (eq("physarum") || eq("slime") || eq("slimemold") || eq("slime_mold") || eq("slime-mold") ||
      eq("mold") || eq("mould") || eq("amoeba") || eq("ant") || eq("ants") || eq("ant_colony") ||
      eq("ant-colony") || eq("aco") || eq("pheromone") || eq("pheromones")) {
    out = ProcGenRoadLayout::Physarum;
    return true;
  }

  if (eq("medial_axis") || eq("medial-axis") || eq("medialaxis") || eq("skeleton") || eq("skel") ||
      eq("spine") || eq("backbone") || eq("centerline") || eq("centreline") || eq("midline") || eq("mid-line")) {
    out = ProcGenRoadLayout::MedialAxis;
    return true;
  }

  if (eq("tensor_field") || eq("tensor-field") || eq("tensorfield") || eq("tensor") ||
      eq("field") || eq("flow") || eq("flow_field") || eq("flow-field") || eq("flowfield") ||
      eq("orientation") || eq("orientation_field") || eq("orientation-field") || eq("orient") ||
      eq("parish") || eq("muller") || eq("parish_muller") || eq("parish-muller")) {
    out = ProcGenRoadLayout::TensorField;
    return true;
  }

  if (eq("voronoi_cells") || eq("voronoi-cells") || eq("voronoicells") || eq("cells") || eq("cellular") || eq("voronoi") ||
      eq("voronoi_mesh") || eq("voronoi-mesh") || eq("mesh") || eq("superblocks") || eq("superblock")) {
    out = ProcGenRoadLayout::VoronoiCells;
    return true;
  }

  if (eq("space_colonization") || eq("space-colonization") || eq("spacecolonization") || eq("space") ||
      eq("colonization") || eq("colonisation") || eq("sca") || eq("sc")) {
    out = ProcGenRoadLayout::SpaceColonization;
    return true;
  }

  return false;
}

} // namespace isocity
