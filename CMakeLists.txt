cmake_minimum_required(VERSION 3.22)

project(ProcIsoCity
  VERSION 0.2.0
  DESCRIPTION "Procedurally-generated, open-source isometric city-building simulation (no external art assets)."
  LANGUAGES C CXX
)

# ---- Options ----
option(PROCISOCITY_BUILD_APP "Build the interactive raylib application" ON)
option(PROCISOCITY_BUILD_TESTS "Build headless tests (no raylib required)" OFF)
option(PROCISOCITY_BUILD_CLI "Build headless command-line tools" ON)

# Only relevant when PROCISOCITY_BUILD_APP=ON
option(PROCISOCITY_USE_SYSTEM_RAYLIB "Use system-installed raylib instead of FetchContent" OFF)

# ---- Language ----
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- Warnings / compiler options ----
# We keep warning levels *per target* to avoid drowning in warnings from third-party
# dependencies (raylib and its bundled single-file libs).
if (MSVC)
  # CMake injects a default warning level (/W3) for MSVC. If we also set /W4 we
  # end up with D9025 ("overriding '/W4' with '/W3'"). Strip any /W* flags from
  # the global flags so each target can choose its own warning level.
  foreach(flag_var
          CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE CMAKE_C_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS_MINSIZEREL
          CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_CXX_FLAGS_MINSIZEREL)
    if(DEFINED ${flag_var})
      string(REGEX REPLACE "/W[0-4]" "" ${flag_var} "${${flag_var}}")
    endif()
  endforeach()
endif()

function(procisocity_set_strict_warnings target)
  if (MSVC)
    target_compile_options(${target} PRIVATE /W4 $<$<COMPILE_LANGUAGE:CXX>:/permissive->)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# ---- Core library (no raylib dependency) ----
add_library(isocity_core
  src/isocity/Checksum.cpp
  src/isocity/Compression.cpp
  src/isocity/GltfExport.cpp
  src/isocity/GeoJsonExport.cpp
  src/isocity/Suite.cpp
  src/isocity/Blueprint.cpp
  src/isocity/MeshExport.cpp
  src/isocity/WorldMeshBuilder.cpp
  src/isocity/RoadGraphExport.cpp
  src/isocity/RoadGraphTrafficExport.cpp
  src/isocity/Districting.cpp
  src/isocity/FloodFill.cpp
  src/isocity/CityBlocks.cpp
  src/isocity/Vectorize.cpp
  src/isocity/CityBlockGraph.cpp
  src/isocity/BlockDistricting.cpp
  src/isocity/Script.cpp
  src/isocity/AutoBuild.cpp
  src/isocity/Replay.cpp
  src/isocity/WorldPatch.cpp
  src/isocity/World.cpp
  src/isocity/WorldDiff.cpp
  src/isocity/WorldTransform.cpp
  src/isocity/ProcGen.cpp
  src/isocity/Erosion.cpp
  src/isocity/DepressionFill.cpp
  src/isocity/FloodRisk.cpp
  src/isocity/Evacuation.cpp
  src/isocity/Hydrology.cpp
  src/isocity/Contours.cpp
  src/isocity/Isochrone.cpp
  src/isocity/TransitPlanner.cpp
  src/isocity/TransitPlannerExport.cpp
  src/isocity/Sim.cpp
  src/isocity/FlowField.cpp
  src/isocity/Traffic.cpp
  src/isocity/Goods.cpp
  src/isocity/ParkOptimizer.cpp
  src/isocity/LandValue.cpp
  src/isocity/DistrictStats.cpp
  src/isocity/ZoneAccess.cpp
  src/isocity/SaveLoad.cpp
  src/isocity/Export.cpp
  src/isocity/Export3D.cpp
  src/isocity/ImageIO.cpp
  src/isocity/Soft3D.cpp
  src/isocity/GfxQuantize.cpp
  src/isocity/GfxAtlasFx.cpp
  src/isocity/GfxMipmaps.cpp
  src/isocity/GfxOutlines.cpp
  src/isocity/GfxPacker.cpp
  src/isocity/GfxTilesetAtlas.cpp
  src/isocity/GfxPalette.cpp
  src/isocity/GfxBuildings.cpp
  src/isocity/GfxProps.cpp
  src/isocity/GfxTileset.cpp
  src/isocity/Heightmap.cpp
  src/isocity/Hash.cpp
  src/isocity/EditHistory.cpp
  src/isocity/Pathfinding.cpp
  src/isocity/RoadGraph.cpp
  src/isocity/RoadGraphRouting.cpp
  src/isocity/RoadGraphTraffic.cpp
  src/isocity/RoadGraphResilience.cpp
  src/isocity/RoadGraphCentrality.cpp
  src/isocity/RoadGraphCentralityExport.cpp
  src/isocity/RoadUpgradePlanner.cpp
  src/isocity/PolicyOptimizer.cpp
  src/isocity/PolicyOptimizerExport.cpp
  src/isocity/ZoneParcels.cpp
  src/isocity/Json.cpp
  src/isocity/ConfigIO.cpp
)

target_include_directories(isocity_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

procisocity_set_strict_warnings(isocity_core)

# Some platforms still need libm explicitly for <cmath>.
if (UNIX AND NOT APPLE)
  target_link_libraries(isocity_core PUBLIC m)
endif()

# ---- App (raylib) ----
if (PROCISOCITY_BUILD_APP)
  # Dependencies (raylib)
  if (PROCISOCITY_USE_SYSTEM_RAYLIB)
    find_package(raylib REQUIRED)
  else()
    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)

    # raylib release tag used by this template (raylib 5.5).
    # You can update this later once you’re ready.
    FetchContent_Declare(
      raylib
      GIT_REPOSITORY https://github.com/raysan5/raylib.git
      GIT_TAG 5.5
      GIT_SHALLOW TRUE
    )

    # Keep raylib lean for an app/game.
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(raylib)

    # raylib (and its bundled single-file dependencies) generate a lot of warnings on MSVC.
    # Suppress warnings for the third-party target so the build output focuses on our code.
    if (TARGET raylib)
      get_target_property(_raylib_imported raylib IMPORTED)
      if (NOT _raylib_imported)
        if (MSVC)
          target_compile_options(raylib PRIVATE /W0)
        else()
          target_compile_options(raylib PRIVATE -w)
        endif()
      endif()
    endif()
  endif()

  add_executable(proc_isocity
    src/main.cpp

    src/isocity/Config.hpp
    src/isocity/Hash.hpp
    src/isocity/Console.hpp
    src/isocity/Console.cpp
    src/isocity/Game.hpp
    src/isocity/Game.cpp

    src/isocity/Iso.hpp
    src/isocity/Noise.hpp
    src/isocity/Random.hpp

    src/isocity/World.hpp
    src/isocity/ProcGen.hpp
    src/isocity/Sim.hpp
    src/isocity/SaveLoad.hpp
    src/isocity/EditHistory.hpp

    src/isocity/Renderer.hpp
    src/isocity/VisualPrefs.hpp
    src/isocity/Renderer.cpp
    src/isocity/VisualPrefs.cpp
  )

  target_include_directories(proc_isocity PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity PRIVATE isocity_core raylib)

  # raylib pulls in Windows headers; prevent `min`/`max` macro pollution that
  # breaks std::min/std::max in our sources.
  if (WIN32)
    target_compile_definitions(proc_isocity PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
  endif()

  procisocity_set_strict_warnings(proc_isocity)

  # Convenience: copy DLLs on Windows when using MSVC + shared builds.
  if (WIN32 AND TARGET raylib)
    add_custom_command(TARGET proc_isocity POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:raylib>
        $<TARGET_FILE_DIR:proc_isocity>
      COMMENT "Copying raylib runtime next to the executable (if needed)"
    )
  endif()
endif()

# ---- CLI (headless) ----
if (PROCISOCITY_BUILD_CLI)
  add_executable(proc_isocity_cli
    src/cli/main.cpp

    src/isocity/Hash.hpp
  )

  target_include_directories(proc_isocity_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_cli PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_cli)

  # Save diff tool (headless): compare two save files and emit a summary + optional diff artifacts.
  add_executable(proc_isocity_diff
    src/cli/diff.cpp
  )

  target_include_directories(proc_isocity_diff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_diff PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_diff)

  # Save inspector (headless): print a fast header/metadata summary without loading the full world.
  add_executable(proc_isocity_inspect
    src/cli/inspect.cpp
  )

  target_include_directories(proc_isocity_inspect PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_inspect PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_inspect)

  # Config tool (headless): dump/apply ProcGenConfig and SimConfig embedded in a save.
  add_executable(proc_isocity_config
    src/cli/config.cpp
  )

  target_include_directories(proc_isocity_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_config PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_config)


  # Script runner (headless): apply deterministic editing/simulation scripts for regression testing.
  add_executable(proc_isocity_script
    src/cli/script.cpp
  )

  target_include_directories(proc_isocity_script PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_script PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_script)

  # Auto-build tool (headless): deterministic "city bot" growth for rapid scenario generation.
  add_executable(proc_isocity_autobuild
    src/cli/autobuild.cpp
  )

  target_include_directories(proc_isocity_autobuild PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_autobuild PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_autobuild)

  # Binary patch tool (headless): create/apply compact diffs between saves.
  add_executable(proc_isocity_patch
    src/cli/patch.cpp
  )

  target_include_directories(proc_isocity_patch PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_patch PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_patch)

  # Replay tool (headless): pack/inspect/play deterministic replay journals.
  add_executable(proc_isocity_replay
    src/cli/replay.cpp
  )

  target_include_directories(proc_isocity_replay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_replay PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_replay)

  # Blueprint tool (headless): capture/apply rectangular stamps of a save.
  add_executable(proc_isocity_blueprint
    src/cli/blueprint.cpp
  )

  target_include_directories(proc_isocity_blueprint PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_blueprint PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_blueprint)

  # Scenario suite runner (headless): run many scripts/replays and emit CI-friendly reports.
  add_executable(proc_isocity_suite
    src/cli/suite.cpp
  )

  target_include_directories(proc_isocity_suite PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_suite PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_suite)

  # Timelapse tool (headless): run the simulator forward and export an isometric frame sequence.
  add_executable(proc_isocity_timelapse
    src/cli/timelapse.cpp
  )

  target_include_directories(proc_isocity_timelapse PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_timelapse PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_timelapse)


  # World transform tool (headless): rotate/mirror/crop entire saves.
  add_executable(proc_isocity_transform
    src/cli/transform.cpp
  )

  target_include_directories(proc_isocity_transform PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_transform PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_transform)


  # Heightmap tool (headless): import/export raster heightmaps.
  add_executable(proc_isocity_heightmap
    src/cli/heightmap.cpp
  )

  target_include_directories(proc_isocity_heightmap PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_heightmap PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_heightmap)

  # Hydrology tool (headless): analyze terrain flow accumulation / rivers / basins.
  add_executable(proc_isocity_hydrology
    src/cli/hydrology.cpp
  )

  target_include_directories(proc_isocity_hydrology PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_hydrology PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_hydrology)

  # Contours tool (headless): extract topographic contour lines and terrain analysis rasters.
  add_executable(proc_isocity_contours
    src/cli/contours.cpp
  )

  target_include_directories(proc_isocity_contours PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_contours PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_contours)


  # Isochrone tool (headless): accessibility heatmaps + isochrone polygons.
  add_executable(proc_isocity_isochrone
    src/cli/isochrone.cpp
  )

  target_include_directories(proc_isocity_isochrone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_isochrone PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_isochrone)




  # Road graph export tool (headless): export compressed road network graphs to DOT/JSON/CSV.
  add_executable(proc_isocity_roadgraph
    src/cli/roadgraph.cpp
  )

  target_include_directories(proc_isocity_roadgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_roadgraph PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_roadgraph)

  # Pathfinding tool (headless): query paths on the road/land grid and road-build planner.
  add_executable(proc_isocity_path
    src/cli/path.cpp
  )

  target_include_directories(proc_isocity_path PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_path PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_path)

  # Road centrality tool (headless): betweenness centrality on the compressed road graph.
  add_executable(proc_isocity_roadcentrality
    src/cli/roadcentrality.cpp
  )

  target_include_directories(proc_isocity_roadcentrality PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_roadcentrality PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_roadcentrality)

  # Traffic graph export tool (headless): compute commute traffic and aggregate onto the road graph.
  add_executable(proc_isocity_trafficgraph
    src/cli/trafficgraph.cpp
  )

  target_include_directories(proc_isocity_trafficgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_trafficgraph PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_trafficgraph)

  # Goods graph export tool (headless): compute goods flow, aggregate onto road graph, and export OD.
  add_executable(proc_isocity_goodsgraph
    src/cli/goodsgraph.cpp
  )

  target_include_directories(proc_isocity_goodsgraph PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_goodsgraph PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_goodsgraph)

  # Transit line planning tool (headless): plan bus-line style corridors from simulated demand.
  add_executable(proc_isocity_transitplan
    src/cli/transitplan.cpp
  )

  target_include_directories(proc_isocity_transitplan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_transitplan PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_transitplan)


  # Park placement tool (headless): suggest new parks based on underserved demand.
  add_executable(proc_isocity_parkopt
    src/cli/parkopt.cpp
  )

  target_include_directories(proc_isocity_parkopt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_parkopt PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_parkopt)


  # Flood/ponding tool (headless): sea-level inundation + Priority-Flood depression depth.
  add_executable(proc_isocity_floodrisk
    src/cli/floodrisk.cpp
  )

  target_include_directories(proc_isocity_floodrisk PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_floodrisk PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_floodrisk)

  add_executable(proc_isocity_evac
    src/cli/evacuation.cpp
  )
  target_include_directories(proc_isocity_evac PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_evac PRIVATE isocity_core)
  procisocity_set_strict_warnings(proc_isocity_evac)

  # Road upgrades tool (headless): propose road-level upgrades under a budget based on combined flows.
  add_executable(proc_isocity_roadupgrades
    src/cli/roadupgrades.cpp
  )

  target_include_directories(proc_isocity_roadupgrades PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_roadupgrades PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_roadupgrades)

  # Policy optimizer tool (headless): search tax/maintenance policy space under an objective.
  add_executable(proc_isocity_policyopt
    src/cli/policyopt.cpp
  )

  target_include_directories(proc_isocity_policyopt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_policyopt PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_policyopt)




  # Road resilience tool (headless): identify bridge edges/articulation nodes and suggest bypass roads.
  add_executable(proc_isocity_roadresilience
    src/cli/roadresilience.cpp
  )

  target_include_directories(proc_isocity_roadresilience PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_roadresilience PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_roadresilience)



  # City blocks tool (headless): extract road-separated land blocks and export metrics.
  add_executable(proc_isocity_blocks
    src/cli/blocks.cpp
  )

  target_include_directories(proc_isocity_blocks PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_blocks PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_blocks)

  # Block-based district assignment tool.
  add_executable(proc_isocity_blockdistricts
    src/cli/blockdistricts.cpp
  )

  target_include_directories(proc_isocity_blockdistricts PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_blockdistricts PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_blockdistricts)



  # Mesh exporter (headless): export a save/world to Wavefront OBJ+MTL for debugging/interop.
  add_executable(proc_isocity_mesh
    src/cli/mesh.cpp
  )

  target_include_directories(proc_isocity_mesh PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_mesh PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_mesh)

  # PPM diff tool (headless): compare two PPM images (P6) and optionally write a diff visualization.
  add_executable(proc_isocity_imagediff
    src/cli/imagediff.cpp
  )

  target_include_directories(proc_isocity_imagediff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_imagediff PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_imagediff)

  # District report tool (headless): per-district stats + GeoJSON/SVG vector exports.
  add_executable(proc_isocity_districtreport
    src/cli/districtreport.cpp
  )

  target_include_directories(proc_isocity_districtreport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_districtreport PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_districtreport)

  # Map export tool (headless): export a world as a single GeoJSON FeatureCollection.
  add_executable(proc_isocity_mapexport
    src/cli/mapexport.cpp
  )

  target_include_directories(proc_isocity_mapexport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_mapexport PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_mapexport)

  # Tileset generator tool (headless): export procedural tile textures as a sprite atlas PNG (+ optional JSON metadata).
  add_executable(proc_isocity_tileset
    src/cli/tileset.cpp
  )

  target_include_directories(proc_isocity_tileset PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_tileset PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_tileset)

endif()

# ---- Tests (headless) ----
if (PROCISOCITY_BUILD_TESTS)
  enable_testing()

  add_executable(proc_isocity_tests
    tests/ProcIsoCityTests.cpp
  )
  target_link_libraries(proc_isocity_tests PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_tests)

  add_test(NAME proc_isocity_tests COMMAND proc_isocity_tests)
endif()
