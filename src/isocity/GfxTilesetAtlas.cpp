#include "isocity/GfxTilesetAtlas.hpp"

#include "isocity/Json.hpp"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <sstream>

namespace isocity {

namespace {

bool ReadFileText(const std::string& path, std::string& out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream oss;
  oss << f.rdbuf();
  out = oss.str();
  return true;
}

const JsonValue* FindObjMember(const JsonValue& obj, const char* key)
{
  if (!key) return nullptr;
  return FindJsonMember(obj, std::string(key));
}

bool ReadI32(const JsonValue& obj, const char* key, int& out, std::string& err)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v) {
    err = std::string("missing key: ") + key;
    return false;
  }
  if (!v->isNumber()) {
    err = std::string("expected number for key: ") + key;
    return false;
  }
  out = static_cast<int>(v->numberValue);
  return true;
}

bool ReadBool(const JsonValue& obj, const char* key, bool& out, std::string& err)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v) {
    err = std::string("missing key: ") + key;
    return false;
  }
  if (!v->isBool()) {
    err = std::string("expected bool for key: ") + key;
    return false;
  }
  out = v->boolValue;
  return true;
}

bool ReadString(const JsonValue& obj, const char* key, std::string& out, std::string& err)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v) {
    err = std::string("missing key: ") + key;
    return false;
  }
  if (!v->isString()) {
    err = std::string("expected string for key: ") + key;
    return false;
  }
  out = v->stringValue;
  return true;
}

bool StartsWith(const std::string& s, const char* prefix)
{
  if (!prefix) return false;
  const std::size_t n = std::char_traits<char>::length(prefix);
  if (s.size() < n) return false;
  return s.compare(0, n, prefix) == 0;
}

int ParseTrailingIntAfter(const std::string& s, const char* needle)
{
  const std::string n = needle ? std::string(needle) : std::string();
  const std::size_t pos = s.rfind(n);
  if (pos == std::string::npos) return -1;
  std::size_t i = pos + n.size();
  if (i >= s.size()) return -1;
  int v = 0;
  bool any = false;
  for (; i < s.size(); ++i) {
    const char c = s[i];
    if (c < '0' || c > '9') break;
    any = true;
    v = v * 10 + (c - '0');
  }
  return any ? v : -1;
}

int KindIndexFromName(const std::string& name)
{
  if (StartsWith(name, "building_res_")) return 0;
  if (StartsWith(name, "building_com_")) return 1;
  if (StartsWith(name, "building_ind_")) return 2;
  return -1;
}

int LevelFromBuildingName(const std::string& name)
{
  // building_<kind>_L{lvl}_v{var}
  const std::size_t lpos = name.find("_L");
  if (lpos == std::string::npos) return -1;
  int lvl = 0;
  std::size_t i = lpos + 2;
  if (i >= name.size()) return -1;
  while (i < name.size()) {
    const char c = name[i];
    if (c < '0' || c > '9') break;
    lvl = lvl * 10 + (c - '0');
    ++i;
  }
  return (lvl >= 1 && lvl <= 3) ? lvl : -1;
}

} // namespace

bool LoadGfxTilesetAtlas(const std::string& atlasPngPath, const std::string& metaJsonPath, GfxTilesetAtlas& out, std::string& outError)
{
  outError.clear();
  out = GfxTilesetAtlas{};

  // Load atlas PNG (RGBA).
  {
    std::string err;
    if (!ReadPngRGBA(atlasPngPath, out.atlas, err)) {
      outError = "failed reading atlas png: " + err;
      return false;
    }
  }

  // Load JSON metadata.
  std::string text;
  if (!ReadFileText(metaJsonPath, text)) {
    outError = "failed to read meta json: " + metaJsonPath;
    return false;
  }

  JsonValue root;
  if (!ParseJson(text, root, outError)) return false;
  if (!root.isObject()) {
    outError = "tileset meta json must be an object";
    return false;
  }

  int atlasW = 0, atlasH = 0;
  if (!ReadI32(root, "atlasW", atlasW, outError) || !ReadI32(root, "atlasH", atlasH, outError)) return false;
  if (atlasW != out.atlas.width || atlasH != out.atlas.height) {
    outError = "atlas dimension mismatch between meta json and png";
    return false;
  }

  // Optional: logical tile size used to generate diamond tiles (independent of trimming/packing).
  const JsonValue* tw = FindObjMember(root, "tileW");
  const JsonValue* th = FindObjMember(root, "tileH");
  if (tw && tw->isNumber()) out.tileW = static_cast<int>(tw->numberValue);
  if (th && th->isNumber()) out.tileH = static_cast<int>(th->numberValue);

  bool hasEm = false;
  if (!ReadBool(root, "hasEmissive", hasEm, outError)) return false;
  out.hasEmissive = hasEm;

  const JsonValue* sprites = FindObjMember(root, "sprites");
  if (!sprites || !sprites->isArray()) {
    outError = "tileset meta json missing sprites array";
    return false;
  }

  out.entries.reserve(sprites->arrayValue.size());
  for (const JsonValue& it : sprites->arrayValue) {
    if (!it.isObject()) {
      outError = "tileset sprite entry must be an object";
      return false;
    }
    GfxAtlasEntry e;
    if (!ReadString(it, "name", e.name, outError)) return false;
    if (!ReadI32(it, "x", e.x, outError)) return false;
    if (!ReadI32(it, "y", e.y, outError)) return false;
    if (!ReadI32(it, "w", e.w, outError)) return false;
    if (!ReadI32(it, "h", e.h, outError)) return false;

    // Pivot is optional in older metadata; default to center.
    const JsonValue* px = FindObjMember(it, "pivotX");
    const JsonValue* py = FindObjMember(it, "pivotY");
    e.pivotX = (px && px->isNumber()) ? static_cast<int>(px->numberValue) : (e.w / 2);
    e.pivotY = (py && py->isNumber()) ? static_cast<int>(py->numberValue) : (e.h / 2);

    // Optional trimming metadata.
    const JsonValue* sw = FindObjMember(it, "srcW");
    const JsonValue* sh = FindObjMember(it, "srcH");
    const JsonValue* tx = FindObjMember(it, "trimX");
    const JsonValue* ty = FindObjMember(it, "trimY");
    e.srcW = (sw && sw->isNumber()) ? static_cast<int>(sw->numberValue) : e.w;
    e.srcH = (sh && sh->isNumber()) ? static_cast<int>(sh->numberValue) : e.h;
    e.trimX = (tx && tx->isNumber()) ? static_cast<int>(tx->numberValue) : 0;
    e.trimY = (ty && ty->isNumber()) ? static_cast<int>(ty->numberValue) : 0;

    out.entries.push_back(std::move(e));
  }

  // Sort by name for deterministic lookup.
  std::sort(out.entries.begin(), out.entries.end(), [](const GfxAtlasEntry& a, const GfxAtlasEntry& b) {
    return a.name < b.name;
  });

  // Infer convenience counts.
  for (const GfxAtlasEntry& e : out.entries) {
    // Tile size from terrain sprites.
    if (out.tileW == 0 && StartsWith(e.name, "terrain_water_v")) {
      out.tileW = e.w;
      out.tileH = e.h;
    }

    const int v = ParseTrailingIntAfter(e.name, "_v");
    if (v < 0) continue;

    if (StartsWith(e.name, "terrain_water_v") || StartsWith(e.name, "terrain_sand_v") || StartsWith(e.name, "terrain_grass_v")) {
      out.terrainVariants = std::max(out.terrainVariants, v + 1);
    }
    if (StartsWith(e.name, "road_L")) {
      out.roadVariants = std::max(out.roadVariants, v + 1);
    }
    if (StartsWith(e.name, "bridge_L")) {
      out.bridgeVariants = std::max(out.bridgeVariants, v + 1);
    }
    if (StartsWith(e.name, "terrain_shore_ws_")) {
      out.transitionVariantsWS = std::max(out.transitionVariantsWS, v + 1);
    }
    if (StartsWith(e.name, "terrain_shore_sg_")) {
      out.transitionVariantsSG = std::max(out.transitionVariantsSG, v + 1);
    }

    // Optional props / vehicles.
    if (StartsWith(e.name, "prop_tree_deciduous_v")) {
      out.propTreeDeciduousVariants = std::max(out.propTreeDeciduousVariants, v + 1);
    }
    if (StartsWith(e.name, "prop_tree_conifer_v")) {
      out.propTreeConiferVariants = std::max(out.propTreeConiferVariants, v + 1);
    }
    if (StartsWith(e.name, "prop_streetlight_v")) {
      out.propStreetlightVariants = std::max(out.propStreetlightVariants, v + 1);
    }
    if (StartsWith(e.name, "prop_car_v")) {
      out.propCarVariants = std::max(out.propCarVariants, v + 1);
    }
    if (StartsWith(e.name, "prop_truck_v")) {
      out.propTruckVariants = std::max(out.propTruckVariants, v + 1);
    }

    const int kind = KindIndexFromName(e.name);
    if (kind >= 0) {
      const int lvl = LevelFromBuildingName(e.name);
      if (lvl >= 1 && lvl <= 3) {
        out.buildingVariants[kind][lvl - 1] = std::max(out.buildingVariants[kind][lvl - 1], v + 1);
      }
    }
  }

  return out.valid();
}

bool LoadGfxTilesetAtlasEmissive(const std::string& emissivePngPath, GfxTilesetAtlas& io, std::string& outError)
{
  outError.clear();
  if (!io.valid()) {
    outError = "tileset atlas must be loaded before emissive";
    return false;
  }

  RgbaImage img;
  std::string err;
  if (!ReadPngRGBA(emissivePngPath, img, err)) {
    outError = "failed reading emissive png: " + err;
    return false;
  }
  if (img.width != io.atlas.width || img.height != io.atlas.height) {
    outError = "emissive atlas dimensions must match base atlas";
    return false;
  }
  io.emissiveAtlas = std::move(img);
  io.hasEmissive = true;
  return true;
}

namespace {

bool LoadAuxAtlas(const std::string& pngPath, const char* label, GfxTilesetAtlas& io, RgbaImage& outImg, bool& outFlag,
                 std::string& outError)
{
  outError.clear();
  if (!io.valid()) {
    outError = "tileset atlas must be loaded before ";
    outError += (label ? label : "aux");
    return false;
  }

  RgbaImage img;
  std::string err;
  if (!ReadPngRGBA(pngPath, img, err)) {
    outError = "failed reading ";
    outError += (label ? label : "aux");
    outError += " png: ";
    outError += err;
    return false;
  }
  if (img.width != io.atlas.width || img.height != io.atlas.height) {
    outError = std::string(label ? label : "aux") + " atlas dimensions must match base atlas";
    return false;
  }

  outImg = std::move(img);
  outFlag = true;
  return true;
}

} // namespace

bool LoadGfxTilesetAtlasNormals(const std::string& normalPngPath, GfxTilesetAtlas& io, std::string& outError)
{
  return LoadAuxAtlas(normalPngPath, "normal", io, io.normalAtlas, io.hasNormals, outError);
}

bool LoadGfxTilesetAtlasShadows(const std::string& shadowPngPath, GfxTilesetAtlas& io, std::string& outError)
{
  return LoadAuxAtlas(shadowPngPath, "shadow", io, io.shadowAtlas, io.hasShadows, outError);
}

const GfxAtlasEntry* FindGfxAtlasEntry(const GfxTilesetAtlas& ts, const std::string& name)
{
  if (ts.entries.empty()) return nullptr;
  auto it = std::lower_bound(ts.entries.begin(), ts.entries.end(), name,
                             [](const GfxAtlasEntry& a, const std::string& b) { return a.name < b; });
  if (it == ts.entries.end() || it->name != name) return nullptr;
  return &(*it);
}

} // namespace isocity
