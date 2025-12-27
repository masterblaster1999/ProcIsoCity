#pragma once

#include "isocity/Iso.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <optional>

#include "raylib.h"

namespace isocity {

class Renderer {
public:
  Renderer(int tileW, int tileH, std::uint64_t seed);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  void rebuildTextures(std::uint64_t seed);

  void drawWorld(const World& world, const Camera2D& camera, float timeSec, std::optional<Point> hovered,
                 bool drawGrid);

  void drawHUD(const World& world, Tool tool, std::optional<Point> hovered, int screenW, int screenH, bool showHelp);

private:
  int m_tileW = 64;
  int m_tileH = 32;

  std::array<Texture2D, 3> m_terrainTex{};
  std::array<Texture2D, 6> m_overlayTex{};

  void unloadTextures();

  Texture2D& terrain(Terrain t);
  Texture2D& overlay(Overlay o);

  static Color BrightnessTint(float b);
};

} // namespace isocity
