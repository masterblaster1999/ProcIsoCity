#include "isocity/GeoJsonExport.hpp"

#include <ostream>

namespace isocity {

void WriteGeoJsonRing(std::ostream& os, const std::vector<IPoint>& ring)
{
  os << "[";
  for (std::size_t i = 0; i < ring.size(); ++i) {
    if (i) os << ",";
    os << "[" << ring[i].x << "," << ring[i].y << "]";
  }
  os << "]";
}

void WriteGeoJsonPolygonCoords(std::ostream& os, const VectorPolygon& poly)
{
  os << "[";
  WriteGeoJsonRing(os, poly.outer);
  for (const auto& hole : poly.holes) {
    os << ",";
    WriteGeoJsonRing(os, hole);
  }
  os << "]";
}

void WriteGeoJsonMultiPolygonCoords(std::ostream& os, const VectorMultiPolygon& mp)
{
  os << "[";
  for (std::size_t pi = 0; pi < mp.polygons.size(); ++pi) {
    if (pi) os << ",";
    WriteGeoJsonPolygonCoords(os, mp.polygons[pi]);
  }
  os << "]";
}

void WriteGeoJsonGeometry(std::ostream& os, const VectorMultiPolygon& mp)
{
  if (mp.polygons.empty()) {
    os << "{\"type\":\"GeometryCollection\",\"geometries\":[]}";
    return;
  }

  if (mp.polygons.size() == 1) {
    os << "{\"type\":\"Polygon\",\"coordinates\":";
    WriteGeoJsonPolygonCoords(os, mp.polygons[0]);
    os << "}";
    return;
  }

  os << "{\"type\":\"MultiPolygon\",\"coordinates\":";
  WriteGeoJsonMultiPolygonCoords(os, mp);
  os << "}";
}

} // namespace isocity
