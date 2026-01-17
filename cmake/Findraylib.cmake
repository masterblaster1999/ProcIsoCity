# Findraylib.cmake
# -----------------
# Fallback find-module for locating a system-installed raylib.
#
# ProcIsoCity normally uses FetchContent to obtain raylib, but many
# environments provide raylib via system packages (and some build
# environments are offline). This module makes `find_package(raylib)` work
# even when raylib does not ship a CMake config package.
#
# Strategy:
#   1) Prefer pkg-config (raylib.pc) when available; this is the most reliable
#      way to pick up transitive link flags, especially for static builds.
#   2) Fall back to a minimal manual search for raylib.h and the raylib library.
#
# Targets:
#   raylib          - stable target name expected by ProcIsoCity
#   raylib::raylib  - alias (compat with some upstream configs)
#
# Variables:
#   raylib_FOUND
#   raylib_INCLUDE_DIRS
#   raylib_LIBRARIES
#
# Optional hints:
#   raylib_ROOT / RAYLIB_ROOT (CMake variable or environment variable)
#
# Notes:
#   - When raylib is provided as a static library on Linux, extra system
#     libraries may be required (OpenGL, X11, pthread, dl, m, etc). The
#     pkg-config path is recommended in that case.

include(FindPackageHandleStandardArgs)

# Allow users to hint an installation prefix.
set(_raylib_root_hints "")
foreach(_v raylib_ROOT RAYLIB_ROOT)
  if (DEFINED ${_v})
    list(APPEND _raylib_root_hints "${${_v}}")
  endif()
endforeach()
foreach(_ev raylib_ROOT RAYLIB_ROOT)
  if (DEFINED ENV{${_ev}})
    list(APPEND _raylib_root_hints "$ENV{${_ev}}")
  endif()
endforeach()

unset(raylib_INCLUDE_DIRS)
unset(raylib_LIBRARIES)

# --- pkg-config path ---
find_package(PkgConfig QUIET)
if (PkgConfig_FOUND)
  pkg_check_modules(RAYLIB_PC QUIET IMPORTED_TARGET raylib)
  if (RAYLIB_PC_FOUND)
    set(raylib_INCLUDE_DIRS "${RAYLIB_PC_INCLUDE_DIRS}")
    set(raylib_LIBRARIES "${RAYLIB_PC_LIBRARIES}")

    if (NOT TARGET raylib)
      add_library(raylib INTERFACE IMPORTED)
      target_link_libraries(raylib INTERFACE PkgConfig::RAYLIB_PC)
    endif()

    if (NOT TARGET raylib::raylib)
      add_library(raylib::raylib ALIAS raylib)
    endif()
  endif()
endif()

# --- manual fallback ---
if (NOT raylib_INCLUDE_DIRS OR NOT raylib_LIBRARIES)
  find_path(raylib_INCLUDE_DIR
    NAMES raylib.h
    HINTS ${_raylib_root_hints}
    PATH_SUFFIXES include Include
  )

  find_library(raylib_LIBRARY
    NAMES raylib
    HINTS ${_raylib_root_hints}
    PATH_SUFFIXES lib Lib
  )

  if (raylib_INCLUDE_DIR AND raylib_LIBRARY)
    set(raylib_INCLUDE_DIRS "${raylib_INCLUDE_DIR}")
    set(raylib_LIBRARIES "${raylib_LIBRARY}")

    if (NOT TARGET raylib)
      add_library(raylib UNKNOWN IMPORTED)
      set_target_properties(raylib PROPERTIES
        IMPORTED_LOCATION "${raylib_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${raylib_INCLUDE_DIR}"
      )
    endif()

    if (NOT TARGET raylib::raylib)
      add_library(raylib::raylib ALIAS raylib)
    endif()
  endif()
endif()

find_package_handle_standard_args(raylib
  DEFAULT_MSG
  raylib_INCLUDE_DIRS
  raylib_LIBRARIES
)

mark_as_advanced(raylib_INCLUDE_DIR raylib_LIBRARY)
