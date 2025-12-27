cmake_minimum_required(VERSION 3.22)

project(ProcIsoCity
  VERSION 0.2.0
  DESCRIPTION "Procedurally-generated, open-source isometric city-building simulation (no external art assets)."
  LANGUAGES C CXX
)

# ---- Options ----
option(PROCISOCITY_BUILD_APP "Build the interactive raylib application" ON)
option(PROCISOCITY_BUILD_TESTS "Build headless tests (no raylib required)" OFF)

# Only relevant when PROCISOCITY_BUILD_APP=ON
option(PROCISOCITY_USE_SYSTEM_RAYLIB "Use system-installed raylib instead of FetchContent" OFF)

# ---- Language / warnings ----
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Core library (no raylib dependency) ----
add_library(isocity_core
  src/isocity/World.cpp
  src/isocity/ProcGen.cpp
  src/isocity/Sim.cpp
  src/isocity/SaveLoad.cpp
  src/isocity/EditHistory.cpp
)

target_include_directories(isocity_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

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
  endif()

  add_executable(proc_isocity
    src/main.cpp

    src/isocity/Config.hpp
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

# ---- Tests (headless) ----
if (PROCISOCITY_BUILD_TESTS)
  enable_testing()

  add_executable(proc_isocity_tests
    tests/ProcIsoCityTests.cpp
  )
  target_link_libraries(proc_isocity_tests PRIVATE isocity_core)

  add_test(NAME proc_isocity_tests COMMAND proc_isocity_tests)
endif()
