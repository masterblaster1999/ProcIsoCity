#include "isocity/SaveLoad.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <type_traits>

namespace isocity {

namespace {

constexpr char kMagic[8] = {'I', 'S', 'O', 'C', 'I', 'T', 'Y', '\0'};
constexpr std::uint32_t kVersion = 1;

template <typename T>
bool Write(std::ostream& os, const T& v)
{
  static_assert(std::is_trivially_copyable_v<T>);
  os.write(reinterpret_cast<const char*>(&v), static_cast<std::streamsize>(sizeof(T)));
  return os.good();
}

template <typename T>
bool Read(std::istream& is, T& v)
{
  static_assert(std::is_trivially_copyable_v<T>);
  is.read(reinterpret_cast<char*>(&v), static_cast<std::streamsize>(sizeof(T)));
  return is.good();
}

bool WriteBytes(std::ostream& os, const void* data, std::size_t size)
{
  os.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
  return os.good();
}

bool ReadBytes(std::istream& is, void* data, std::size_t size)
{
  is.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(size));
  return is.good();
}

struct StatsBin {
  std::int32_t day = 0;

  std::int32_t population = 0;
  std::int32_t housingCapacity = 0;

  std::int32_t jobsCapacity = 0;
  std::int32_t employed = 0;

  float happiness = 0.5f;

  std::int32_t money = 0;

  std::int32_t roads = 0;
  std::int32_t parks = 0;
};

StatsBin ToBin(const Stats& s)
{
  StatsBin b;
  b.day = static_cast<std::int32_t>(s.day);
  b.population = static_cast<std::int32_t>(s.population);
  b.housingCapacity = static_cast<std::int32_t>(s.housingCapacity);
  b.jobsCapacity = static_cast<std::int32_t>(s.jobsCapacity);
  b.employed = static_cast<std::int32_t>(s.employed);
  b.happiness = s.happiness;
  b.money = static_cast<std::int32_t>(s.money);
  b.roads = static_cast<std::int32_t>(s.roads);
  b.parks = static_cast<std::int32_t>(s.parks);
  return b;
}

void FromBin(Stats& s, const StatsBin& b)
{
  s.day = static_cast<int>(b.day);
  s.population = static_cast<int>(b.population);
  s.housingCapacity = static_cast<int>(b.housingCapacity);
  s.jobsCapacity = static_cast<int>(b.jobsCapacity);
  s.employed = static_cast<int>(b.employed);
  s.happiness = b.happiness;
  s.money = static_cast<int>(b.money);
  s.roads = static_cast<int>(b.roads);
  s.parks = static_cast<int>(b.parks);
}

bool WriteTile(std::ostream& os, const Tile& t)
{
  const std::uint8_t terrain = static_cast<std::uint8_t>(t.terrain);
  const std::uint8_t overlay = static_cast<std::uint8_t>(t.overlay);
  if (!Write(os, terrain)) return false;
  if (!Write(os, overlay)) return false;
  if (!Write(os, t.height)) return false;
  if (!Write(os, t.variation)) return false;
  if (!Write(os, t.level)) return false;
  if (!Write(os, t.occupants)) return false;
  return true;
}

bool ReadTile(std::istream& is, Tile& t)
{
  std::uint8_t terrain = 0;
  std::uint8_t overlay = 0;

  if (!Read(is, terrain)) return false;
  if (!Read(is, overlay)) return false;

  t.terrain = static_cast<Terrain>(terrain);
  t.overlay = static_cast<Overlay>(overlay);

  if (!Read(is, t.height)) return false;
  if (!Read(is, t.variation)) return false;
  if (!Read(is, t.level)) return false;
  if (!Read(is, t.occupants)) return false;

  return true;
}

} // namespace

bool SaveWorldBinary(const World& world, const std::string& path, std::string& outError)
{
  outError.clear();

  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "Unable to open file for writing: " + path;
    return false;
  }

  // Header
  if (!WriteBytes(f, kMagic, sizeof(kMagic))) {
    outError = "Write failed (magic)";
    return false;
  }

  if (!Write(f, kVersion)) {
    outError = "Write failed (version)";
    return false;
  }

  const std::uint32_t w = static_cast<std::uint32_t>(world.width());
  const std::uint32_t h = static_cast<std::uint32_t>(world.height());
  const std::uint64_t seed = world.seed();

  if (!Write(f, w) || !Write(f, h) || !Write(f, seed)) {
    outError = "Write failed (header fields)";
    return false;
  }

  // Stats
  const StatsBin sb = ToBin(world.stats());
  if (!Write(f, sb)) {
    outError = "Write failed (stats)";
    return false;
  }

  // Tiles
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      if (!WriteTile(f, world.at(x, y))) {
        outError = "Write failed (tiles)";
        return false;
      }
    }
  }

  return true;
}

bool LoadWorldBinary(World& outWorld, const std::string& path, std::string& outError)
{
  outError.clear();

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outError = "Unable to open file for reading: " + path;
    return false;
  }

  // Header
  char magic[8] = {};
  if (!ReadBytes(f, magic, sizeof(magic))) {
    outError = "Read failed (magic)";
    return false;
  }
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "Not a ProcIsoCity save file (bad magic)";
    return false;
  }

  std::uint32_t version = 0;
  if (!Read(f, version)) {
    outError = "Read failed (version)";
    return false;
  }
  if (version != kVersion) {
    std::ostringstream oss;
    oss << "Unsupported save version: " << version << " (expected " << kVersion << ")";
    outError = oss.str();
    return false;
  }

  std::uint32_t w = 0, h = 0;
  std::uint64_t seed = 0;
  if (!Read(f, w) || !Read(f, h) || !Read(f, seed)) {
    outError = "Read failed (header fields)";
    return false;
  }

  constexpr std::uint32_t kMaxDim = 4096;
  if (w == 0 || h == 0 || w > kMaxDim || h > kMaxDim) {
    outError = "Invalid map dimensions in save file";
    return false;
  }

  const std::uint64_t tileCount = static_cast<std::uint64_t>(w) * static_cast<std::uint64_t>(h);
  if (tileCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    outError = "Save file map dimensions overflow size_t";
    return false;
  }

  StatsBin sb{};
  if (!Read(f, sb)) {
    outError = "Read failed (stats)";
    return false;
  }

  World loaded(static_cast<int>(w), static_cast<int>(h), seed);
  FromBin(loaded.stats(), sb);

  for (int y = 0; y < loaded.height(); ++y) {
    for (int x = 0; x < loaded.width(); ++x) {
      Tile t;
      if (!ReadTile(f, t)) {
        outError = "Read failed (tiles)";
        return false;
      }
      loaded.at(x, y) = t;
    }
  }

  // Older saves (and bulk edits like undo/redo) may have stale road connectivity bits.
  // Recompute ensures road auto-tiling stays consistent.
  loaded.recomputeRoadMasks();

  outWorld = std::move(loaded);
  return true;
}

} // namespace isocity
