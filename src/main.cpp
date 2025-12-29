#include "isocity/Game.hpp"
#include "isocity/Random.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

static bool StartsWith(const std::string& s, const std::string& prefix)
{
  return s.rfind(prefix, 0) == 0;
}

static std::uint64_t ParseU64(const std::string& s)
{
  // Supports: 1234, 0x1234
  std::size_t idx = 0;
  int base = 10;
  if (StartsWith(s, "0x") || StartsWith(s, "0X")) {
    base = 16;
    idx = 2;
  }
  std::uint64_t value = 0;
  for (; idx < s.size(); ++idx) {
    const char c = s[idx];
    int digit = -1;
    if (c >= '0' && c <= '9') digit = (c - '0');
    else if (base == 16 && c >= 'a' && c <= 'f') digit = 10 + (c - 'a');
    else if (base == 16 && c >= 'A' && c <= 'F') digit = 10 + (c - 'A');
    else break;
    value = value * static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(digit);
  }
  return value;
}

static bool ParseWxH(const std::string& s, int& outW, int& outH)
{
  auto pos = s.find('x');
  if (pos == std::string::npos) pos = s.find('X');
  if (pos == std::string::npos) return false;

  try {
    outW = std::stoi(s.substr(0, pos));
    outH = std::stoi(s.substr(pos + 1));
    return outW > 0 && outH > 0;
  } catch (...) {
    return false;
  }
}

static float ParseFloat(const std::string& s, float fallback)
{
  try {
    // std::stof accepts leading/trailing whitespace and stops at first invalid char.
    return std::stof(s);
  } catch (...) {
    return fallback;
  }
}

static int ParseInt(const std::string& s, int fallback)
{
  try {
    return std::stoi(s);
  } catch (...) {
    return fallback;
  }
}

int main(int argc, char** argv)
{
  isocity::Config cfg;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      std::cout << "ProcIsoCity\n"
                << "  --seed <u64|0xHEX>\n"
                << "  --size <W>x<H>      (map size)\n"
                << "  --window <W>x<H>    (window size)\n"
                << "  --elev <scale>      (elevation scale multiplier, 0=flat; default 0.75)\n"
                << "  --elevsteps <N>     (0=smooth, otherwise quantize to N steps; default 16)\n"
                << "  --flat              (shortcut for --elev 0)\n"
                << "  --novsync\n";
      return 0;
    }

    if (arg == "--seed" && i + 1 < argc) {
      cfg.seed = ParseU64(argv[++i]);
      continue;
    }

    if (arg == "--size" && i + 1 < argc) {
      int w = 0, h = 0;
      if (ParseWxH(argv[++i], w, h)) {
        cfg.mapWidth = w;
        cfg.mapHeight = h;
      }
      continue;
    }

    if (arg == "--window" && i + 1 < argc) {
      int w = 0, h = 0;
      if (ParseWxH(argv[++i], w, h)) {
        cfg.windowWidth = w;
        cfg.windowHeight = h;
      }
      continue;
    }

    if (arg == "--novsync") {
      cfg.vsync = false;
      continue;
    }

    if (arg == "--elev" && i + 1 < argc) {
      cfg.elevationScale = ParseFloat(argv[++i], cfg.elevationScale);
      if (cfg.elevationScale < 0.0f) cfg.elevationScale = 0.0f;
      continue;
    }

    if (arg == "--elevsteps" && i + 1 < argc) {
      cfg.elevationSteps = std::max(0, ParseInt(argv[++i], cfg.elevationSteps));
      continue;
    }

    if (arg == "--flat") {
      cfg.elevationScale = 0.0f;
      continue;
    }
  }

  if (cfg.seed == 0) cfg.seed = isocity::TimeSeed();

  try {
    isocity::Game game(cfg);
    game.run();
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Fatal error: unknown exception\n";
    return 1;
  }

  return 0;
}
