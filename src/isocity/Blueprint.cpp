#include "isocity/Blueprint.hpp"

#include "isocity/Compression.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <vector>

namespace isocity {

namespace {

inline constexpr std::uint8_t kMagic[8] = {'I', 'S', 'O', 'B', 'L', 'U', 'E', 'P'};
inline constexpr std::uint32_t kCurrentVersion = 1;

inline constexpr std::uint8_t kKnownTileMask =
    static_cast<std::uint8_t>(TileFieldMask::Terrain) |
    static_cast<std::uint8_t>(TileFieldMask::Overlay) |
    static_cast<std::uint8_t>(TileFieldMask::Height) |
    static_cast<std::uint8_t>(TileFieldMask::Variation) |
    static_cast<std::uint8_t>(TileFieldMask::Level) |
    static_cast<std::uint8_t>(TileFieldMask::Occupants) |
    static_cast<std::uint8_t>(TileFieldMask::District);

inline bool MaskHas(std::uint8_t mask, TileFieldMask bit)
{
  return (mask & static_cast<std::uint8_t>(bit)) != 0;
}

// --- Tiny binary IO helpers (little-endian, float-as-bits) ---

struct ByteWriter {
  std::vector<std::uint8_t> out;

  bool writeBytes(const void* data, std::size_t n)
  {
    if (n == 0) return true;
    const auto* p = reinterpret_cast<const std::uint8_t*>(data);
    out.insert(out.end(), p, p + n);
    return true;
  }

  bool writeU8(std::uint8_t v) { return writeBytes(&v, 1); }

  bool writeU16(std::uint16_t v)
  {
    const std::uint8_t b[2] = {
        static_cast<std::uint8_t>((v >> 0) & 0xFFu),
        static_cast<std::uint8_t>((v >> 8) & 0xFFu),
    };
    return writeBytes(b, 2);
  }

  bool writeU32(std::uint32_t v)
  {
    const std::uint8_t b[4] = {
        static_cast<std::uint8_t>((v >> 0) & 0xFFu),
        static_cast<std::uint8_t>((v >> 8) & 0xFFu),
        static_cast<std::uint8_t>((v >> 16) & 0xFFu),
        static_cast<std::uint8_t>((v >> 24) & 0xFFu),
    };
    return writeBytes(b, 4);
  }

  bool writeI32(std::int32_t v)
  {
    std::uint32_t uv = 0;
    static_assert(sizeof(uv) == sizeof(v), "i32/u32 size mismatch");
    std::memcpy(&uv, &v, sizeof(uv));
    return writeU32(uv);
  }

  bool writeF32(float v)
  {
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    return writeU32(bits);
  }

  // Unsigned LEB128 varint.
  bool writeVarU32(std::uint32_t v)
  {
    std::uint8_t buf[5];
    std::size_t n = 0;
    while (v >= 0x80u) {
      buf[n++] = static_cast<std::uint8_t>((v & 0x7Fu) | 0x80u);
      v >>= 7;
    }
    buf[n++] = static_cast<std::uint8_t>(v & 0x7Fu);
    return writeBytes(buf, n);
  }
};

struct ByteReader {
  const std::uint8_t* data = nullptr;
  std::size_t size = 0;
  std::size_t pos = 0;

  bool readBytes(void* out, std::size_t n)
  {
    if (n == 0) return true;
    if (!out) return false;
    if (pos + n > size) return false;
    std::memcpy(out, data + pos, n);
    pos += n;
    return true;
  }

  bool readU8(std::uint8_t& out) { return readBytes(&out, 1); }

  bool readU16(std::uint16_t& out)
  {
    std::uint8_t b[2];
    if (!readBytes(b, 2)) return false;
    out = static_cast<std::uint16_t>(static_cast<std::uint16_t>(b[0]) | (static_cast<std::uint16_t>(b[1]) << 8));
    return true;
  }

  bool readU32(std::uint32_t& out)
  {
    std::uint8_t b[4];
    if (!readBytes(b, 4)) return false;
    out = static_cast<std::uint32_t>(static_cast<std::uint32_t>(b[0]) |
                                     (static_cast<std::uint32_t>(b[1]) << 8) |
                                     (static_cast<std::uint32_t>(b[2]) << 16) |
                                     (static_cast<std::uint32_t>(b[3]) << 24));
    return true;
  }

  bool readI32(std::int32_t& out)
  {
    std::uint32_t uv = 0;
    if (!readU32(uv)) return false;
    static_assert(sizeof(uv) == sizeof(out), "i32/u32 size mismatch");
    std::memcpy(&out, &uv, sizeof(out));
    return true;
  }

  bool readF32(float& out)
  {
    std::uint32_t bits = 0;
    if (!readU32(bits)) return false;
    static_assert(sizeof(float) == sizeof(bits), "float must be 32-bit");
    std::memcpy(&out, &bits, sizeof(out));
    return true;
  }

  bool readVarU32(std::uint32_t& out)
  {
    out = 0;
    std::uint32_t shift = 0;
    for (int i = 0; i < 5; ++i) {
      std::uint8_t b = 0;
      if (!readU8(b)) return false;
      out |= static_cast<std::uint32_t>(b & 0x7Fu) << shift;
      if ((b & 0x80u) == 0) return true;
      shift += 7;
    }
    return false;
  }
};

static bool WriteTileDeltas(ByteWriter& w, const std::vector<WorldPatchTileDelta>& tiles)
{
  if (!w.writeVarU32(static_cast<std::uint32_t>(tiles.size()))) return false;

  std::uint32_t prev = 0;
  bool first = true;

  for (const auto& d : tiles) {
    const std::uint32_t idx = d.index;
    const std::uint32_t delta = first ? idx : (idx - prev);

    if (!w.writeVarU32(delta)) return false;
    if (!w.writeU8(d.mask)) return false;

    const Tile& v = d.value;
    if (MaskHas(d.mask, TileFieldMask::Terrain)) {
      if (!w.writeU8(static_cast<std::uint8_t>(v.terrain))) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Overlay)) {
      if (!w.writeU8(static_cast<std::uint8_t>(v.overlay))) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Height)) {
      if (!w.writeF32(v.height)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Variation)) {
      if (!w.writeU8(v.variation)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Level)) {
      if (!w.writeU8(v.level)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::Occupants)) {
      if (!w.writeU16(v.occupants)) return false;
    }
    if (MaskHas(d.mask, TileFieldMask::District)) {
      if (!w.writeU8(v.district)) return false;
    }

    prev = idx;
    first = false;
  }

  return true;
}

static bool ReadTileDeltas(ByteReader& r, std::vector<WorldPatchTileDelta>& outTiles,
                           std::size_t maxTiles, std::string& outError)
{
  outTiles.clear();

  std::uint32_t count = 0;
  if (!r.readVarU32(count)) {
    outError = "failed to read tile delta count";
    return false;
  }
  if (count > static_cast<std::uint32_t>(maxTiles)) {
    outError = "tile delta count exceeds maxTiles";
    return false;
  }

  outTiles.reserve(count);

  std::uint32_t idx = 0;
  bool first = true;
  for (std::uint32_t i = 0; i < count; ++i) {
    std::uint32_t delta = 0;
    if (!r.readVarU32(delta)) {
      outError = "failed to read tile index delta";
      return false;
    }
    if (first) {
      idx = delta;
    } else {
      if (std::numeric_limits<std::uint32_t>::max() - idx < delta) {
        outError = "tile index overflow";
        return false;
      }
      idx += delta;
    }

    std::uint8_t mask = 0;
    if (!r.readU8(mask)) {
      outError = "failed to read tile mask";
      return false;
    }
    if ((mask & ~kKnownTileMask) != 0) {
      outError = "tile delta contains unknown mask bits";
      return false;
    }

    WorldPatchTileDelta d;
    d.index = idx;
    d.mask = mask;
    d.value = Tile{};

    if (MaskHas(mask, TileFieldMask::Terrain)) {
      std::uint8_t tv = 0;
      if (!r.readU8(tv)) {
        outError = "failed to read terrain";
        return false;
      }
      if (tv > 2u) {
        outError = "invalid terrain value";
        return false;
      }
      d.value.terrain = static_cast<Terrain>(tv);
    }

    if (MaskHas(mask, TileFieldMask::Overlay)) {
      std::uint8_t ov = 0;
      if (!r.readU8(ov)) {
        outError = "failed to read overlay";
        return false;
      }
      if (ov > 5u) {
        outError = "invalid overlay value";
        return false;
      }
      d.value.overlay = static_cast<Overlay>(ov);
    }

    if (MaskHas(mask, TileFieldMask::Height)) {
      if (!r.readF32(d.value.height)) {
        outError = "failed to read height";
        return false;
      }
    }

    if (MaskHas(mask, TileFieldMask::Variation)) {
      if (!r.readU8(d.value.variation)) {
        outError = "failed to read variation";
        return false;
      }
    }

    if (MaskHas(mask, TileFieldMask::Level)) {
      if (!r.readU8(d.value.level)) {
        outError = "failed to read level";
        return false;
      }
    }

    if (MaskHas(mask, TileFieldMask::Occupants)) {
      if (!r.readU16(d.value.occupants)) {
        outError = "failed to read occupants";
        return false;
      }
    }

    if (MaskHas(mask, TileFieldMask::District)) {
      if (!r.readU8(d.value.district)) {
        outError = "failed to read district";
        return false;
      }
    }

    outTiles.push_back(d);
    first = false;
  }

  return true;
}

static bool ValidateRotation(int rotateDeg, std::string& outError)
{
  if (rotateDeg == 0 || rotateDeg == 90 || rotateDeg == 180 || rotateDeg == 270) {
    return true;
  }
  outError = "invalid rotation (allowed: 0,90,180,270)";
  return false;
}

static void TransformCoord(int x, int y, int w, int h, const BlueprintTransform& tr,
                           int& outX, int& outY, int& outW, int& outH)
{
  // Compute rotated coord.
  int rx = 0;
  int ry = 0;
  int tw = w;
  int th = h;

  const int r = tr.rotateDeg;
  if (r == 0) {
    rx = x;
    ry = y;
    tw = w;
    th = h;
  } else if (r == 90) {
    tw = h;
    th = w;
    rx = (h - 1) - y;
    ry = x;
  } else if (r == 180) {
    tw = w;
    th = h;
    rx = (w - 1) - x;
    ry = (h - 1) - y;
  } else {
    // 270
    tw = h;
    th = w;
    rx = y;
    ry = (w - 1) - x;
  }

  // Mirror after rotation.
  if (tr.mirrorX) {
    rx = (tw - 1) - rx;
  }
  if (tr.mirrorY) {
    ry = (th - 1) - ry;
  }

  outX = rx;
  outY = ry;
  outW = tw;
  outH = th;
}

} // namespace

const char* BlueprintCompressionName(BlueprintCompression c)
{
  switch (c) {
  case BlueprintCompression::None: return "None";
  case BlueprintCompression::SLLZ: return "SLLZ";
  default: return "Unknown";
  }
}

bool CaptureBlueprintRect(const World& world, int x0, int y0, int w, int h,
                          Blueprint& outBlueprint, std::string& outError,
                          const BlueprintCaptureOptions& opt)
{
  outError.clear();
  outBlueprint = Blueprint{};

  if (w <= 0 || h <= 0) {
    outError = "invalid blueprint size";
    return false;
  }
  if (!world.inBounds(x0, y0) || !world.inBounds(x0 + w - 1, y0 + h - 1)) {
    outError = "capture rect out of bounds";
    return false;
  }

  outBlueprint.width = w;
  outBlueprint.height = h;
  outBlueprint.version = kCurrentVersion;

  const bool canSparse = opt.sparseByOverlay && MaskHas(opt.fieldMask, TileFieldMask::Overlay);

  outBlueprint.tiles.clear();
  outBlueprint.tiles.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));

  for (int ry = 0; ry < h; ++ry) {
    for (int rx = 0; rx < w; ++rx) {
      const Tile& src = world.at(x0 + rx, y0 + ry);
      if (canSparse && src.overlay == Overlay::None) {
        continue;
      }

      WorldPatchTileDelta d;
      d.index = static_cast<std::uint32_t>(ry * w + rx);
      d.mask = opt.fieldMask;
      d.value = src;

      if (opt.zeroOccupants && MaskHas(d.mask, TileFieldMask::Occupants)) {
        d.value.occupants = 0;
      }

      outBlueprint.tiles.push_back(d);
    }
  }

  // Ensure deterministic ordering.
  std::sort(outBlueprint.tiles.begin(), outBlueprint.tiles.end(),
            [](const WorldPatchTileDelta& a, const WorldPatchTileDelta& b) { return a.index < b.index; });

  return true;
}

bool ApplyBlueprint(World& world, const Blueprint& bp, int dstX, int dstY,
                    const BlueprintApplyOptions& opt, std::string& outError)
{
  outError.clear();

  if (bp.width <= 0 || bp.height <= 0) {
    outError = "invalid blueprint";
    return false;
  }

  std::string rotErr;
  if (!ValidateRotation(opt.transform.rotateDeg, rotErr)) {
    outError = rotErr;
    return false;
  }

  const std::uint64_t area = static_cast<std::uint64_t>(bp.width) * static_cast<std::uint64_t>(bp.height);
  if (area > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "blueprint area too large";
    return false;
  }

  bool touchedRoad = false;

  for (const auto& d : bp.tiles) {
    if (d.index >= area) {
      outError = "blueprint contains out-of-range tile index";
      return false;
    }

    std::uint8_t mask = static_cast<std::uint8_t>(d.mask & opt.fieldMask);
    if (mask == 0) {
      continue;
    }

    if (opt.mode == BlueprintApplyMode::Stamp && MaskHas(mask, TileFieldMask::Overlay) && d.value.overlay == Overlay::None) {
      continue;
    }

    const int localX = static_cast<int>(d.index % static_cast<std::uint32_t>(bp.width));
    const int localY = static_cast<int>(d.index / static_cast<std::uint32_t>(bp.width));

    int tx = 0;
    int ty = 0;
    int tw = 0;
    int th = 0;
    TransformCoord(localX, localY, bp.width, bp.height, opt.transform, tx, ty, tw, th);

    const int x = dstX + tx;
    const int y = dstY + ty;

    if (!world.inBounds(x, y)) {
      if (opt.allowOutOfBounds) {
        continue;
      }
      outError = "apply out of bounds";
      return false;
    }

    // Apply fields in an order that keeps invariants reasonable.
    Tile& t = world.at(x, y);

    if (MaskHas(mask, TileFieldMask::Terrain)) {
      t.terrain = d.value.terrain;
    }

    // If we are about to place a zone/park on water and not forcing, treat it as an error.
    if (!opt.force && MaskHas(mask, TileFieldMask::Overlay)) {
      const Overlay o = d.value.overlay;
      if (t.terrain == Terrain::Water && o != Overlay::None && o != Overlay::Road) {
        outError = "cannot place non-road overlay on water (force=0)";
        return false;
      }
    }

    if (MaskHas(mask, TileFieldMask::Height)) {
      t.height = d.value.height;
    }

    if (MaskHas(mask, TileFieldMask::Overlay)) {
      const Overlay before = t.overlay;
      world.setOverlay(d.value.overlay, x, y);
      const Overlay after = world.at(x, y).overlay;
      if (before != after) {
        touchedRoad = touchedRoad || (before == Overlay::Road) || (after == Overlay::Road);
      }
    }

    Tile& ta = world.at(x, y);

    if (MaskHas(mask, TileFieldMask::Variation)) {
      if (ta.overlay == Overlay::Road) {
        // Preserve the auto-tiling road mask in the low nibble.
        const std::uint8_t low = static_cast<std::uint8_t>(ta.variation & 0x0Fu);
        ta.variation = static_cast<std::uint8_t>((d.value.variation & 0xF0u) | low);
      } else {
        ta.variation = d.value.variation;
      }
    }

    if (MaskHas(mask, TileFieldMask::Level)) {
      const int lvl = std::clamp<int>(static_cast<int>(d.value.level), 1, 3);
      if (ta.overlay == Overlay::Road || ta.overlay == Overlay::Residential || ta.overlay == Overlay::Commercial ||
          ta.overlay == Overlay::Industrial) {
        ta.level = static_cast<std::uint8_t>(lvl);
      } else {
        ta.level = 1;
      }
    }

    if (MaskHas(mask, TileFieldMask::Occupants)) {
      if (ta.overlay == Overlay::Residential || ta.overlay == Overlay::Commercial || ta.overlay == Overlay::Industrial) {
        ta.occupants = d.value.occupants;
      } else {
        ta.occupants = 0;
      }
    }

    if (MaskHas(mask, TileFieldMask::District)) {
      ta.district = static_cast<std::uint8_t>(std::clamp<int>(static_cast<int>(d.value.district), 0, kDistrictCount - 1));
    }
  }

  if (opt.recomputeRoadMasks && touchedRoad) {
    world.recomputeRoadMasks();
  }

  return true;
}

bool SerializeBlueprintBinary(const Blueprint& bp, std::vector<std::uint8_t>& outBytes,
                              std::string& outError, BlueprintCompression compression)
{
  outError.clear();
  outBytes.clear();

  if (bp.width <= 0 || bp.height <= 0) {
    outError = "invalid blueprint size";
    return false;
  }

  const std::uint64_t area = static_cast<std::uint64_t>(bp.width) * static_cast<std::uint64_t>(bp.height);
  if (area > static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "blueprint area too large";
    return false;
  }

  // Sort tile deltas for stable encoding.
  std::vector<WorldPatchTileDelta> tiles = bp.tiles;
  std::sort(tiles.begin(), tiles.end(),
            [](const WorldPatchTileDelta& a, const WorldPatchTileDelta& b) { return a.index < b.index; });

  // Validate indices + masks.
  for (const auto& d : tiles) {
    if (static_cast<std::uint64_t>(d.index) >= area) {
      outError = "tile delta index out of range";
      return false;
    }
    if ((d.mask & ~kKnownTileMask) != 0) {
      outError = "tile delta contains unknown mask bits";
      return false;
    }
  }

  // Build payload.
  ByteWriter payloadW;
  if (!WriteTileDeltas(payloadW, tiles)) {
    outError = "failed to serialize tile deltas";
    return false;
  }

  const std::vector<std::uint8_t>& payload = payloadW.out;
  if (payload.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "payload too large";
    return false;
  }

  std::vector<std::uint8_t> payloadComp;
  if (compression == BlueprintCompression::SLLZ) {
    if (!CompressSLLZ(payload.data(), payload.size(), payloadComp)) {
      outError = "compression failed";
      return false;
    }
  } else {
    payloadComp = payload;
  }

  if (payloadComp.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "compressed payload too large";
    return false;
  }

  // Build header.
  ByteWriter w;
  if (!w.writeBytes(kMagic, sizeof(kMagic))) return false;
  if (!w.writeU32(kCurrentVersion)) return false;
  if (!w.writeI32(static_cast<std::int32_t>(bp.width))) return false;
  if (!w.writeI32(static_cast<std::int32_t>(bp.height))) return false;
  if (!w.writeU8(static_cast<std::uint8_t>(compression))) return false;
  if (!w.writeU32(static_cast<std::uint32_t>(payload.size()))) return false;
  if (!w.writeU32(static_cast<std::uint32_t>(payloadComp.size()))) return false;
  if (!w.writeBytes(payloadComp.data(), payloadComp.size())) return false;

  outBytes = std::move(w.out);
  return true;
}

bool DeserializeBlueprintBinary(const std::vector<std::uint8_t>& bytes, Blueprint& outBlueprint,
                                std::string& outError)
{
  outError.clear();
  outBlueprint = Blueprint{};

  ByteReader r;
  r.data = bytes.data();
  r.size = bytes.size();
  r.pos = 0;

  std::uint8_t magic[8];
  if (!r.readBytes(magic, sizeof(magic))) {
    outError = "file too small";
    return false;
  }
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "bad magic";
    return false;
  }

  std::uint32_t version = 0;
  if (!r.readU32(version)) {
    outError = "failed to read version";
    return false;
  }
  if (version != kCurrentVersion) {
    outError = "unsupported blueprint version";
    return false;
  }

  std::int32_t w = 0;
  std::int32_t h = 0;
  if (!r.readI32(w) || !r.readI32(h)) {
    outError = "failed to read dimensions";
    return false;
  }
  if (w <= 0 || h <= 0) {
    outError = "invalid dimensions";
    return false;
  }

  std::uint8_t compU8 = 0;
  if (!r.readU8(compU8)) {
    outError = "failed to read compression";
    return false;
  }
  const BlueprintCompression comp = static_cast<BlueprintCompression>(compU8);

  std::uint32_t payloadSize = 0;
  std::uint32_t payloadSizeComp = 0;
  if (!r.readU32(payloadSize) || !r.readU32(payloadSizeComp)) {
    outError = "failed to read payload sizes";
    return false;
  }

  if (r.pos + payloadSizeComp > r.size) {
    outError = "truncated payload";
    return false;
  }

  const std::uint8_t* payloadData = r.data + r.pos;
  const std::size_t payloadDataSize = payloadSizeComp;
  r.pos += payloadSizeComp;

  std::vector<std::uint8_t> payload;
  if (comp == BlueprintCompression::None) {
    if (payloadSizeComp != payloadSize) {
      outError = "payload size mismatch";
      return false;
    }
    payload.assign(payloadData, payloadData + payloadDataSize);
  } else if (comp == BlueprintCompression::SLLZ) {
    if (!DecompressSLLZ(payloadData, payloadDataSize, static_cast<std::size_t>(payloadSize), payload, outError)) {
      if (outError.empty()) outError = "decompression failed";
      return false;
    }
  } else {
    outError = "unknown compression";
    return false;
  }

  // Parse payload.
  ByteReader pr;
  pr.data = payload.data();
  pr.size = payload.size();
  pr.pos = 0;

  std::vector<WorldPatchTileDelta> tiles;
  const std::uint64_t area = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (!ReadTileDeltas(pr, tiles, static_cast<std::size_t>(area), outError)) {
    return false;
  }

  // Validate indices.
  for (const auto& d : tiles) {
    if (static_cast<std::uint64_t>(d.index) >= area) {
      outError = "tile delta index out of range";
      return false;
    }
  }

  outBlueprint.width = static_cast<int>(w);
  outBlueprint.height = static_cast<int>(h);
  outBlueprint.version = version;
  outBlueprint.tiles = std::move(tiles);

  // Keep deterministic ordering.
  std::sort(outBlueprint.tiles.begin(), outBlueprint.tiles.end(),
            [](const WorldPatchTileDelta& a, const WorldPatchTileDelta& b) { return a.index < b.index; });

  return true;
}

bool SaveBlueprintBinary(const Blueprint& bp, const std::string& path,
                         std::string& outError, BlueprintCompression compression)
{
  outError.clear();

  std::vector<std::uint8_t> bytes;
  if (!SerializeBlueprintBinary(bp, bytes, outError, compression)) {
    return false;
  }

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open for writing";
    return false;
  }

  f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!f) {
    outError = "write failed";
    return false;
  }

  return true;
}

bool LoadBlueprintBinary(Blueprint& outBlueprint, const std::string& path, std::string& outError)
{
  outError.clear();
  outBlueprint = Blueprint{};

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open";
    return false;
  }

  std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (!f && !f.eof()) {
    outError = "read failed";
    return false;
  }

  return DeserializeBlueprintBinary(bytes, outBlueprint, outError);
}

} // namespace isocity
