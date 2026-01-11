#include "isocity/GeoJsonExport.hpp"

#include "isocity/Json.hpp"

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



// Streaming (JsonWriter) overloads ---------------------------------------------------------------

void WriteGeoJsonRing(JsonWriter& w, const std::vector<IPoint>& ring)
{
  w.beginArray();
  for (const IPoint& p : ring) {
    w.beginArray();
    w.intValue(p.x);
    w.intValue(p.y);
    w.endArray();
  }
  w.endArray();
}

void WriteGeoJsonPolygonCoords(JsonWriter& w, const VectorPolygon& poly)
{
  w.beginArray();
  WriteGeoJsonRing(w, poly.outer);
  for (const auto& hole : poly.holes) {
    WriteGeoJsonRing(w, hole);
  }
  w.endArray();
}

void WriteGeoJsonMultiPolygonCoords(JsonWriter& w, const VectorMultiPolygon& mp)
{
  w.beginArray();
  for (const auto& poly : mp.polygons) {
    WriteGeoJsonPolygonCoords(w, poly);
  }
  w.endArray();
}

void WriteGeoJsonGeometry(JsonWriter& w, const VectorMultiPolygon& mp)
{
  if (mp.polygons.empty()) {
    w.beginObject();
    w.key("type");
    w.stringValue("GeometryCollection");
    w.key("geometries");
    w.beginArray();
    w.endArray();
    w.endObject();
    return;
  }

  if (mp.polygons.size() == 1) {
    w.beginObject();
    w.key("type");
    w.stringValue("Polygon");
    w.key("coordinates");
    WriteGeoJsonPolygonCoords(w, mp.polygons[0]);
    w.endObject();
    return;
  }

  w.beginObject();
  w.key("type");
  w.stringValue("MultiPolygon");
  w.key("coordinates");
  WriteGeoJsonMultiPolygonCoords(w, mp);
  w.endObject();
}


} // namespace isocity
