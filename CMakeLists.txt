cmake_minimum_required(VERSION 3.22)

project(ProcIsoCity
  VERSION 0.2.0
  DESCRIPTION "Procedurally-generated, open-source isometric city-building simulation (no external art assets)."
  LANGUAGES C CXX
)

# ---- Version / build metadata ----
# Expose the project version (and optionally the git commit SHA) as compile-time macros.
#
# - The version comes from the CMake project() VERSION field.
# - The git SHA is best-effort: it is "unknown" when building from a source archive
#   without a .git directory, or when git is not available.
set(PROCISOCITY_GIT_SHA "unknown")
find_package(Git QUIET)
if (GIT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE _procisocity_git_sha
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _procisocity_git_sha_rv
    ERROR_QUIET
  )
  if (_procisocity_git_sha_rv EQUAL 0 AND NOT "${_procisocity_git_sha}" STREQUAL "")
    set(PROCISOCITY_GIT_SHA "${_procisocity_git_sha}")
  endif()
endif()

# ---- Options ----
# NOTE: Default OFF to keep the default build fully offline-capable.
# The interactive app requires raylib; enable explicitly when you have raylib available.
option(PROCISOCITY_BUILD_APP "Build the interactive raylib application (requires raylib)" OFF)
option(PROCISOCITY_BUILD_TESTS "Build headless tests (no raylib required)" OFF)
option(PROCISOCITY_BUILD_LITE_TESTS "Build lightweight unit tests (fast, minimal deps)" OFF)
option(PROCISOCITY_BUILD_CLI "Build headless command-line tools" ON)
option(PROCISOCITY_ENABLE_GLTF_EXPORT "Enable glTF/glb export (can increase build time)" ON)


# Only relevant when PROCISOCITY_BUILD_APP=ON
option(PROCISOCITY_USE_SYSTEM_RAYLIB "Use system-installed raylib instead of FetchContent" OFF)

# ---- Core library toggle ----
# The isocity_core library is large. For "lite" unit tests and some CI scenarios,
# it can be useful to configure a build that does *not* compile the full library.
#
# Default behavior:
# - ON when building the app, CLI, or full test suite.
# - OFF when only building lite tests.
set(_procisocity_default_build_core OFF)
if (PROCISOCITY_BUILD_APP OR PROCISOCITY_BUILD_CLI OR PROCISOCITY_BUILD_TESTS)
  set(_procisocity_default_build_core ON)
endif()
option(PROCISOCITY_BUILD_CORE "Build the core headless library (isocity_core)" ${_procisocity_default_build_core})

if ((PROCISOCITY_BUILD_APP OR PROCISOCITY_BUILD_CLI OR PROCISOCITY_BUILD_TESTS) AND NOT PROCISOCITY_BUILD_CORE)
  message(FATAL_ERROR
    "PROCISOCITY_BUILD_CORE=OFF, but PROCISOCITY_BUILD_APP/CLI/TESTS is ON. "
    "Enable PROCISOCITY_BUILD_CORE or disable those options.")
endif()


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

if (PROCISOCITY_BUILD_CORE)

# ---- Core library (no raylib dependency) ----
add_library(isocity_core
  src/isocity/AppPaths.cpp
  src/isocity/CrashHandler.cpp
  src/isocity/LogTee.cpp
  src/isocity/SupportBundle.cpp
  src/isocity/ZipWriter.cpp
  src/isocity/SessionLock.cpp
  src/isocity/FileSync.cpp
  src/isocity/HealthCheck.cpp
  src/isocity/StackTrace.cpp
  src/isocity/Checksum.cpp
  src/isocity/Compression.cpp
  src/isocity/GeoJsonExport.cpp
  src/isocity/OsmImport.cpp
  src/isocity/Suite.cpp
  src/isocity/Blueprint.cpp
  src/isocity/WorldTiling.cpp
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
  src/isocity/ReplayCapture.cpp
  src/isocity/WorldPatch.cpp
  src/isocity/WorldPatchJson.cpp
  src/isocity/World.cpp
  src/isocity/WorldDiff.cpp
  src/isocity/WorldDiffViz.cpp
  src/isocity/WorldTransform.cpp
  src/isocity/ProcGen.cpp
  src/isocity/ProcGenEnums.cpp
  src/isocity/Erosion.cpp
  src/isocity/DepressionFill.cpp
  src/isocity/FloodRisk.cpp
  src/isocity/Evacuation.cpp
  src/isocity/EvacuationScenario.cpp
  src/isocity/Hydrology.cpp
  src/isocity/Contours.cpp
  src/isocity/Isochrone.cpp
  src/isocity/TransitPlanner.cpp
  src/isocity/TransitAccessibility.cpp
  src/isocity/TransitPlannerExport.cpp
  src/isocity/Sim.cpp
  src/isocity/Economy.cpp
  src/isocity/Services.cpp
  src/isocity/ServiceOptimizer.cpp
  src/isocity/FlowField.cpp
  src/isocity/Traffic.cpp
  src/isocity/NoisePollution.cpp
  src/isocity/HeatIsland.cpp
  src/isocity/AirPollution.cpp
  src/isocity/RunoffPollution.cpp
  src/isocity/RunoffMitigation.cpp
  src/isocity/SolarPotential.cpp
  src/isocity/SkyView.cpp
  src/isocity/EnergyModel.cpp
  src/isocity/CarbonModel.cpp
  src/isocity/CrimeModel.cpp
  src/isocity/TrafficSafety.cpp
  src/isocity/FireRisk.cpp
  src/isocity/Walkability.cpp
  src/isocity/Livability.cpp
  src/isocity/JobOpportunity.cpp
  src/isocity/HotspotAnalysis.cpp
  src/isocity/Goods.cpp
  src/isocity/TradeMarket.cpp
  src/isocity/ParkOptimizer.cpp
  src/isocity/LandValue.cpp
  src/isocity/LandUseMix.cpp
  src/isocity/DistrictStats.cpp
  src/isocity/ZoneAccess.cpp
  src/isocity/SaveLoad.cpp
  src/isocity/SaveDiscovery.cpp
  src/isocity/CityMeta.cpp
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
  src/isocity/GfxFacilities.cpp
  src/isocity/GfxProps.cpp
  src/isocity/GfxSigils.cpp
  src/isocity/GfxPatterns.cpp
  src/isocity/GfxFrames.cpp
  src/isocity/GfxTileset.cpp
  src/isocity/Heightmap.cpp
  src/isocity/Hash.cpp
  src/isocity/FileHash.cpp
  src/isocity/CliManifest.cpp
  src/isocity/EditHistory.cpp
  src/isocity/Pathfinding.cpp
  src/isocity/RoadGraph.cpp
  src/isocity/RoadGraphRouting.cpp
  src/isocity/RoadGraphTraffic.cpp
  src/isocity/RoadGraphResilience.cpp
  src/isocity/RoadResilienceBypass.cpp
  src/isocity/RoadGraphCentrality.cpp
  src/isocity/RoadGraphCentralityExport.cpp
  src/isocity/RoadHealth.cpp
  src/isocity/RoadUpgradePlanner.cpp
  src/isocity/RoadUpgradePlannerExport.cpp
  src/isocity/PolicyOptimizer.cpp
  src/isocity/PolicyOptimizerExport.cpp
  src/isocity/ZoneParcels.cpp
  src/isocity/StreetNames.cpp
  src/isocity/Cartography.cpp
  src/isocity/TourPlanner.cpp
  src/isocity/Wayfinding.cpp
  src/isocity/Json.cpp
  src/isocity/ConfigIO.cpp
  src/isocity/SeedMiner.cpp
  src/isocity/MineExpr.cpp
  src/isocity/MineCheckpoint.cpp
  src/isocity/MineCheckpointSh.cpp
  src/isocity/MineGallery.cpp
  src/isocity/MineClustering.cpp
  src/isocity/MineEmbedding.cpp
  src/isocity/MineNeighbors.cpp
  src/isocity/MineTraces.cpp
  src/isocity/Chronicle.cpp
  src/isocity/Dossier.cpp
)

# Optional: allow disabling glTF export to reduce build times / memory usage.
# When disabled, a small stub is compiled so the API remains link-compatible.
if (PROCISOCITY_ENABLE_GLTF_EXPORT)
  target_sources(isocity_core PRIVATE src/isocity/GltfExport.cpp)
  target_compile_definitions(isocity_core PUBLIC PROCISOCITY_ENABLE_GLTF_EXPORT=1)
else()
  target_sources(isocity_core PRIVATE src/isocity/GltfExportStub.cpp)
  target_compile_definitions(isocity_core PUBLIC PROCISOCITY_ENABLE_GLTF_EXPORT=0)
endif()


target_include_directories(isocity_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Provide version/build metadata to all targets that link against isocity_core.
target_compile_definitions(isocity_core PUBLIC
  PROCISOCITY_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
  PROCISOCITY_VERSION_MINOR=${PROJECT_VERSION_MINOR}
  PROCISOCITY_VERSION_PATCH=${PROJECT_VERSION_PATCH}
  PROCISOCITY_VERSION_STRING=\"${PROJECT_VERSION}\"
  PROCISOCITY_GIT_SHA=\"${PROCISOCITY_GIT_SHA}\"
)

procisocity_set_strict_warnings(isocity_core)

# Some platforms still need libm explicitly for <cmath>.
if (UNIX AND NOT APPLE)
  target_link_libraries(isocity_core PUBLIC m)
endif()

# Stack trace symbolization on Windows uses DbgHelp.
if (WIN32)
  target_link_libraries(isocity_core PUBLIC dbghelp)
endif()


endif()

# ---- Shared CLI library (no raylib dependency) ----
#
# Build the CLI implementation as a reusable library so the interactive app can
# optionally dispatch into the headless toolchain:
#
#   proc_isocity cli --help
#
# This also avoids compiling the (large) CLI translation unit twice.
if (PROCISOCITY_BUILD_CLI OR PROCISOCITY_BUILD_APP)
  add_library(isocity_cli_lib STATIC
    src/cli/main.cpp
    src/cli/CliMain.hpp
  )

  target_include_directories(isocity_cli_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(isocity_cli_lib PUBLIC isocity_core)

  procisocity_set_strict_warnings(isocity_cli_lib)
endif()

# ---- App (raylib) ----
if (PROCISOCITY_BUILD_APP)
  # Dependencies (raylib)
  if (PROCISOCITY_USE_SYSTEM_RAYLIB)
    # Prefer an upstream CMake config package if it exists (some installations ship one).
    find_package(raylib CONFIG QUIET)
    if (NOT raylib_FOUND)
      list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
      find_package(raylib REQUIRED)
    endif()
  else()
    # Fetch raylib unless the user has requested a fully-disconnected configure and
    # the raylib sources are not already available locally.
    set(_procisocity_need_fetch_raylib TRUE)

    if (FETCHCONTENT_FULLY_DISCONNECTED)
      # If the user points FetchContent at a local checkout, it can still be used in
      # fully disconnected mode.
      if (DEFINED FETCHCONTENT_SOURCE_DIR_RAYLIB AND NOT "${FETCHCONTENT_SOURCE_DIR_RAYLIB}" STREQUAL "")
        set(_procisocity_need_fetch_raylib TRUE)
      else()
        # If raylib was populated previously, it will usually live under:
        #   ${FETCHCONTENT_BASE_DIR}/raylib-src
        # (default: ${CMAKE_BINARY_DIR}/_deps/raylib-src).
        set(_fc_base_dir "${FETCHCONTENT_BASE_DIR}")
        if (NOT _fc_base_dir)
          set(_fc_base_dir "${CMAKE_BINARY_DIR}/_deps")
        endif()
        set(_raylib_populated_dir "${_fc_base_dir}/raylib-src")

        if (NOT EXISTS "${_raylib_populated_dir}/CMakeLists.txt")
          set(_procisocity_need_fetch_raylib FALSE)
        endif()
      endif()

      if (NOT _procisocity_need_fetch_raylib)
        message(STATUS "FetchContent is fully disconnected and raylib sources are not available locally; trying system raylib.")
        find_package(raylib CONFIG QUIET)
        if (NOT raylib_FOUND)
          list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
          find_package(raylib QUIET)
        endif()
        if (NOT raylib_FOUND)
          message(FATAL_ERROR
            "raylib was not found, but FetchContent downloads are disabled (FETCHCONTENT_FULLY_DISCONNECTED=ON).\n"
            "Options:\n"
            "  - Install raylib (and optionally set raylib_ROOT/RAYLIB_ROOT)\n"
            "  - Provide a local raylib checkout: -DFETCHCONTENT_SOURCE_DIR_RAYLIB=<path>\n"
            "  - Disable the app build: -DPROCISOCITY_BUILD_APP=OFF\n"
            "  - Or configure once with network access so FetchContent can populate raylib\n")
        endif()
      endif()
    endif()

    if (_procisocity_need_fetch_raylib)
      include(FetchContent)

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
  endif()

  # Normalize target naming across package variants.
  if (TARGET raylib::raylib AND NOT TARGET raylib)
    add_library(raylib ALIAS raylib::raylib)
  endif()

  add_executable(proc_isocity
    src/main.cpp

    src/isocity/Config.hpp
    src/isocity/Hash.hpp
    src/isocity/Console.hpp
    src/isocity/Console.cpp
    src/isocity/RaylibLog.hpp
    src/isocity/RaylibLog.cpp
    src/isocity/RaylibTrace.hpp
    src/isocity/RaylibTrace.cpp
    src/isocity/Game.hpp
    src/isocity/Game.cpp

    src/isocity/Iso.hpp
    src/isocity/Noise.hpp
    src/isocity/Random.hpp

    src/isocity/World.hpp
    src/isocity/ProcGen.hpp
    src/isocity/Sim.hpp
    src/isocity/TradeMarket.hpp
    src/isocity/SaveLoad.hpp
    src/isocity/EditHistory.hpp

    src/isocity/Renderer.hpp
    src/isocity/GpuGeom.hpp
    src/isocity/VisualPrefs.hpp
    src/isocity/OrganicMaterial.hpp
    src/isocity/PovController.hpp
    src/isocity/PovProcedural.hpp
    src/isocity/Renderer.cpp
    src/isocity/GpuGeom.cpp
    src/isocity/OrganicMaterial.cpp
    src/isocity/PovController.cpp
    src/isocity/PovProcedural.cpp
    src/isocity/Ui.cpp
    src/isocity/VisualPrefs.cpp
    src/isocity/PostFxSettings.hpp
    src/isocity/PostFx.hpp
    src/isocity/PostFx.cpp

    src/isocity/RenderPipeline.hpp
    src/isocity/RenderPipeline.cpp

    src/isocity/ShaderUtil.hpp
    src/isocity/ShaderUtil.cpp
  )

  target_include_directories(proc_isocity PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity PRIVATE isocity_core raylib)

  if (TARGET isocity_cli_lib)
    target_link_libraries(proc_isocity PRIVATE isocity_cli_lib)
    target_compile_definitions(proc_isocity PRIVATE PROCISOCITY_HAS_EMBEDDED_CLI=1)
  else()
    target_compile_definitions(proc_isocity PRIVATE PROCISOCITY_HAS_EMBEDDED_CLI=0)
  endif()

  # raylib pulls in Windows headers; prevent `min`/`max` macro pollution that
  # breaks std::min/std::max in our sources.
  if (WIN32)
    target_compile_definitions(proc_isocity PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
  endif()

  procisocity_set_strict_warnings(proc_isocity)

  # Convenience: copy the on-disk shader override directory next to the executable.
  #
  # The app embeds safe shader fallbacks, so this is not required to *run*.
  # But having `./shaders/` available enables hot-reload (shader_reload) and makes
  # packaged builds easier to mod.
  if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
    add_custom_command(TARGET proc_isocity POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:proc_isocity>/shaders
      COMMENT "Copying shader override directory next to the executable"
    )
  endif()

  # Convenience: copy DLLs on Windows when using MSVC + shared builds.
  #
  # Note: some system raylib configurations provide an INTERFACE target (e.g. via pkg-config).
  # INTERFACE libraries don't have a runtime artifact, so $<TARGET_FILE:...> would be invalid.
  if (WIN32 AND TARGET raylib)
    get_target_property(_raylib_type raylib TYPE)
    if (NOT _raylib_type STREQUAL "INTERFACE_LIBRARY")
      add_custom_command(TARGET proc_isocity POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_FILE:raylib>
          $<TARGET_FILE_DIR:proc_isocity>
        COMMENT "Copying raylib runtime next to the executable (if needed)"
      )
    endif()
  endif()
endif()

# ---- CLI (headless) ----
if (PROCISOCITY_BUILD_CLI)
  add_executable(proc_isocity_cli
    src/cli/cli_entry.cpp
  )

  target_include_directories(proc_isocity_cli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_cli PRIVATE isocity_cli_lib)

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

  # Evolution tool (headless): search the procedural parameter space for "great" cities.
  # Uses a deterministic cross-entropy style sampler + seed mutation.
  add_executable(proc_isocity_evolve
    src/cli/evolve.cpp
  )

  target_include_directories(proc_isocity_evolve PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_evolve PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_evolve)

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

  # Street naming + parcel addressing (headless): deterministic street names and addresses.
  add_executable(proc_isocity_streetnames
    src/cli/streetnames.cpp
  )

  target_include_directories(proc_isocity_streetnames PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_streetnames PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_streetnames)

  # Cartography poster exporter: labeled isometric overview (streets + districts) -> PNG/JSON.
  add_executable(proc_isocity_cartography
    src/cli/cartography.cpp
  )

  target_include_directories(proc_isocity_cartography PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_cartography PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_cartography)

  # Wayfinding tool (headless): geocode addresses/intersections and emit turn-by-turn routes.
  add_executable(proc_isocity_wayfind
    src/cli/wayfind.cpp
  )

  target_include_directories(proc_isocity_wayfind PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_wayfind PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_wayfind)

  # Tour guide tool (headless): synthesize POIs and generate a walkable itinerary + poster.
  add_executable(proc_isocity_tour
    src/cli/tour.cpp
  )

  target_include_directories(proc_isocity_tour PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_tour PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_tour)


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

  # Road health tool (headless): centrality + vulnerability + suggested bypass paths.
  add_executable(proc_isocity_roadhealth
    src/cli/roadhealth.cpp
  )

  target_include_directories(proc_isocity_roadhealth PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_roadhealth PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_roadhealth)

  add_executable(proc_isocity_livability
    src/cli/livability.cpp
  )

  target_include_directories(proc_isocity_livability PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_livability PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_livability)



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

  # Macro economy snapshot tool (headless): compute a deterministic daily economy state.
  add_executable(proc_isocity_economy
    src/cli/economy.cpp
  )

  target_include_directories(proc_isocity_economy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_economy PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_economy)

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

  # Civic services placement tool (headless): suggest education/health/safety facilities.
  add_executable(proc_isocity_servicesopt
    src/cli/servicesopt.cpp
  )

  target_include_directories(proc_isocity_servicesopt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_servicesopt PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_servicesopt)


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

  # World diff tool (headless): compare two worlds (saves or generated seeds) and optionally write diff images.
  add_executable(proc_isocity_worlddiff
    src/cli/worlddiff.cpp
  )

  target_include_directories(proc_isocity_worlddiff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_worlddiff PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_worlddiff)


  # District report tool (headless): per-district stats + GeoJSON/SVG vector exports.
  add_executable(proc_isocity_districtreport
    src/cli/districtreport.cpp
  )

  target_include_directories(proc_isocity_districtreport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_districtreport PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_districtreport)

  # Walkability tool (headless): amenity accessibility / "15-minute city" summary report.
  add_executable(proc_isocity_walkability
    src/cli/walkability.cpp
  )

  target_include_directories(proc_isocity_walkability PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_walkability PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_walkability)

  # Solar potential tool (headless): solar exposure + rooftop PV potential summary report.
  add_executable(proc_isocity_solar
    src/cli/solar.cpp
  )

  target_include_directories(proc_isocity_solar PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_solar PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_solar)

  # Runoff pollution tool (headless): route stormwater pollutant loads downhill.
  add_executable(proc_isocity_runoff
    src/cli/runoff.cpp
  )

  target_include_directories(proc_isocity_runoff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_runoff PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_runoff)

  # Dossier tool (headless): one-command export of images + CSV/JSON + HTML.
  add_executable(proc_isocity_dossier
    src/cli/dossier.cpp
  )

  target_include_directories(proc_isocity_dossier PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_dossier PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_dossier)

  # Chronicle tool (headless): generate a deterministic city newspaper feed.
  add_executable(proc_isocity_chronicle
    src/cli/chronicle.cpp
  )

  target_include_directories(proc_isocity_chronicle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_chronicle PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_chronicle)

  # Seed miner tool (headless): batch-generate/simulate seeds, compute KPIs, and rank by objective.
  add_executable(proc_isocity_mine
    src/cli/mine.cpp
  )

  target_include_directories(proc_isocity_mine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_mine PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_mine)


  # Map export tool (headless): export a world as a single GeoJSON FeatureCollection.
  add_executable(proc_isocity_mapexport
    src/cli/mapexport.cpp
  )

  target_include_directories(proc_isocity_mapexport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_mapexport PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_mapexport)

  # OSM import tool (headless): import an OpenStreetMap (OSM XML) extract into a new save.
  add_executable(proc_isocity_osmimport
    src/cli/osmimport.cpp
  )

  target_include_directories(proc_isocity_osmimport PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_osmimport PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_osmimport)


  # Tileset generator tool (headless): export procedural tile textures as a sprite atlas PNG (+ optional JSON metadata).
  add_executable(proc_isocity_tileset
    src/cli/tileset.cpp
  )

  target_include_directories(proc_isocity_tileset PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_tileset PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_tileset)

  # Custom graphics generator tool (headless): export procedural "sigil" badge icons as a PNG sheet.
  add_executable(proc_isocity_customgfx
    src/cli/customgfx.cpp
  )

  target_include_directories(proc_isocity_customgfx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(proc_isocity_customgfx PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_customgfx)

endif()

# ---- Tests (headless) ----
if (PROCISOCITY_BUILD_TESTS OR PROCISOCITY_BUILD_LITE_TESTS)
  enable_testing()
endif()

# Full headless test suite (large; links isocity_core)
if (PROCISOCITY_BUILD_TESTS)
  add_executable(proc_isocity_tests
    tests/ProcIsoCityTests.cpp
  )
  target_link_libraries(proc_isocity_tests PRIVATE isocity_core)

  procisocity_set_strict_warnings(proc_isocity_tests)

  add_test(NAME proc_isocity_tests COMMAND proc_isocity_tests)
endif()

# Lightweight unit tests (fast; minimal sources, no isocity_core link)
if (PROCISOCITY_BUILD_LITE_TESTS)
  add_executable(proc_isocity_zip_tests
    tests/ZipWriterLiteTests.cpp
    src/isocity/ZipWriter.cpp
    src/isocity/Checksum.cpp
  )

  target_include_directories(proc_isocity_zip_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  procisocity_set_strict_warnings(proc_isocity_zip_tests)

  add_test(NAME proc_isocity_zip_tests COMMAND proc_isocity_zip_tests)


  add_executable(proc_isocity_procgen_enum_tests
    tests/ProcGenEnumLiteTests.cpp
    src/isocity/ProcGenEnums.cpp
  )

  target_include_directories(proc_isocity_procgen_enum_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  procisocity_set_strict_warnings(proc_isocity_procgen_enum_tests)

  add_test(NAME proc_isocity_procgen_enum_tests COMMAND proc_isocity_procgen_enum_tests)


  add_executable(proc_isocity_cli_parse_tests
    tests/CliParseLiteTests.cpp
  )

  target_include_directories(proc_isocity_cli_parse_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  procisocity_set_strict_warnings(proc_isocity_cli_parse_tests)

  add_test(NAME proc_isocity_cli_parse_tests COMMAND proc_isocity_cli_parse_tests)
endif()
