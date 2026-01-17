# Custom shaders

ProcIsoCity embeds its GLSL shaders in C++ so the game is completely asset-free.
However, for iteration and modding it's useful to override those shaders on disk.

This repository now supports **optional** on-disk shader overrides:

* `postfx.vs.glsl` / `postfx.fs.glsl` - screen-space post-processing applied when drawing the
  world render target to the window.
* `volcloud.vs.glsl` / `volcloud.fs.glsl` - volumetric cloud layer shader.

The engine searches for a `shaders/` directory starting at the current working directory and
walking up a few parent directories (so running from a `build/` folder works).

## Includes

GLSL does not reliably support `#include` across drivers. ProcIsoCity implements a tiny
CPU-side preprocessor that expands:

```glsl
#include "common.glsl"
```

Includes are resolved relative to the including file.

## Built-in uniforms

The built-in shaders expect raylib's default 2D batch uniforms (`texture0`, `colDiffuse`, and
`mvp`). The PostFX shader also receives:

* `u_time` (float)
* `u_seed` (float in [0,1))
* `u_bits` (int)
* `u_dither` (float)
* `u_grain` (float)
* `u_vignette` (float)
* `u_chroma` (float)
* `u_scanlines` (float)

Optional convenience uniforms (set if present in your custom shader):

* `u_resolution` (vec2) - screen size in pixels
* `u_texelSize` (vec2) - 1/texture size for `texture0`

## Reloading

In the interactive app, open the dev console (F4) and run:

```
shader_reload
```

This forces recompilation of PostFX and volumetric cloud shaders from disk.
