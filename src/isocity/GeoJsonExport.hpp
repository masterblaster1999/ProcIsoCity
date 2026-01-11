#pragma once

#include "isocity/Vectorize.hpp"

#include <iosfwd>

namespace isocity {

class JsonWriter;

// Minimal helpers for writing GeoJSON geometries from Vectorize output.
//
// These functions intentionally avoid a full JSON library so they can be used
// by headless CLI tools while keeping deterministic, dependency-free output.
//
// Notes:
//  - The caller is responsible for writing surrounding JSON object structure
//    (FeatureCollection, Feature properties, etc.).
//  - Rings are expected to be *closed* (ring.front() == ring.back()).
//  - Coordinates are emitted as [x,y] arrays in tile-corner space.

// Write a GeoJSON linear ring coordinate array:
//   [[x0,y0],[x1,y1],...]
void WriteGeoJsonRing(std::ostream& os, const std::vector<IPoint>& ring);

// Write a GeoJSON Polygon "coordinates" value:
//   [ outerRing, holeRing1, holeRing2, ... ]
void WriteGeoJsonPolygonCoords(std::ostream& os, const VectorPolygon& poly);

// Write a GeoJSON MultiPolygon "coordinates" value:
//   [ polygon0Coords, polygon1Coords, ... ]
void WriteGeoJsonMultiPolygonCoords(std::ostream& os, const VectorMultiPolygon& mp);

// Write a GeoJSON geometry object for a VectorMultiPolygon.
//
// Emits either:
//   {"type":"Polygon","coordinates": ...}
// or
//   {"type":"MultiPolygon","coordinates": ...}
//
// If mp.polygons is empty, emits a null geometry:
//   {"type":"GeometryCollection","geometries":[]}
void WriteGeoJsonGeometry(std::ostream& os, const VectorMultiPolygon& mp);

// Overloads that write directly to a JsonWriter (streaming).
void WriteGeoJsonRing(JsonWriter& w, const std::vector<IPoint>& ring);
void WriteGeoJsonPolygonCoords(JsonWriter& w, const VectorPolygon& poly);
void WriteGeoJsonMultiPolygonCoords(JsonWriter& w, const VectorMultiPolygon& mp);
void WriteGeoJsonGeometry(JsonWriter& w, const VectorMultiPolygon& mp);

} // namespace isocity
