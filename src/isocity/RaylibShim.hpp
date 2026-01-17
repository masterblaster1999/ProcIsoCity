#pragma once

// Raylib include shim.
//
// Some C++ toolchains can error if <raylib.h> is included before <cstdio>/<stdio.h>
// (see upstream raylib issue #3747). To avoid fragile include-order problems, this
// shim always includes <cstdio> and <cstdarg> first, then includes raylib.
//
// Use this header instead of including raylib.h directly in project sources.

#include <cstdarg>
#include <cstdio>

// raylib.h already provides extern "C" guards for C++.
#include <raylib.h>
