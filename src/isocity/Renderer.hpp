#pragma once

#include "isocity/Iso.hpp"
#include "isocity/World.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

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
                 bool drawGrid, int brushRadius, std::optional<Point> selected, const std::vector<Point>* highlightPath,
                 const std::vector<std::uint8_t>* roadToEdgeMask = nullptr,
                 const std::vector<std::uint16_t>* roadTraffic = nullptr, int trafficMax = 0,
                 const std::vector<std::uint16_t>* roadGoodsTraffic = nullptr, int goodsMax = 0,
                 const std::vector<std::uint8_t>* commercialGoodsFill = nullptr);

  void drawHUD(const World& world, Tool tool, std::optional<Point> hovered, int screenW, int screenH, bool showHelp,
               int brushRadius, int undoCount, int redoCount, bool simPaused, float simSpeed, int saveSlot,
               const char* inspectInfo);

private:
  int m_tileW = 64;
  int m_tileH = 32;

  std::array<Texture2D, 3> m_terrainTex{};
  std::array<Texture2D, 6> m_overlayTex{};
  std::array<Texture2D, 16> m_roadTex{}; // auto-tiling variants (connection mask 0..15)

  void unloadTextures();

  Texture2D& terrain(Terrain t);
  Texture2D& overlay(Overlay o);
  Texture2D& road(std::uint8_t mask);

  static Color BrightnessTint(float b);
};

} // namespace isocity
