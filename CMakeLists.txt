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
  src/isocity/Compression.cpp
  src/isocity/World.cpp
  src/isocity/ProcGen.cpp
  src/isocity/Sim.cpp
  src/isocity/FlowField.cpp
  src/isocity/Traffic.cpp
  src/isocity/Goods.cpp
  src/isocity/LandValue.cpp
  src/isocity/DistrictStats.cpp
  src/isocity/SaveLoad.cpp
  src/isocity/Export.cpp
  src/isocity/Hash.cpp
  src/isocity/EditHistory.cpp
  src/isocity/Pathfinding.cpp
  src/isocity/RoadGraph.cpp
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
    src/isocity/Renderer.cpp
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
