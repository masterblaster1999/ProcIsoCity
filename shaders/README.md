# Custom shaders

ProcIsoCity embeds its GLSL shaders in C++ so the game is completely asset-free.
However, for iteration and modding it's useful to override those shaders on disk.

This repository now supports **optional** on-disk shader overrides:

* `postfx.vs.glsl` / `postfx.fs.glsl` - screen-space post-processing applied when drawing the
  world render target to the window.
* `bloom_extract.vs.glsl` / `bloom_extract.fs.glsl` - bright-pass extraction for the optional
  bloom/glow pass.
* `bloom_blur.vs.glsl` / `bloom_blur.fs.glsl` - separable blur pass for bloom (run twice: horizontal
  + vertical).
* `volcloud.vs.glsl` / `volcloud.fs.glsl` - volumetric cloud layer shader.
* `cloudmask.vs.glsl` / `cloudmask.fs.glsl` - GPU-generated tileable cloud-shadow mask (used for
  ground cloud shadows).
* `weatherfx.vs.glsl` / `weatherfx.fs.glsl` - screen-space precipitation particles (rain/snow) drawn as an overlay.
* `materialfx.vs.glsl` / `materialfx.fs.glsl` - world-space procedural material animation applied
  when drawing cached terrain bands (water ripples, vegetation flutter).

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
`mvp`).

### PostFX uniforms

The PostFX shader also receives:

* `u_time` (float)
* `u_seed` (float in [0,1))
* `u_bits` (int)
* `u_dither` (float)
* `u_grain` (float)
* `u_vignette` (float)
* `u_chroma` (float)
* `u_scanlines` (float)
* `u_fxaa` (float)
* `u_sharpen` (float)

Filmic tonemap + grade (optional, gated by `u_tonemapEnabled`):

* `u_tonemapEnabled` (float; 0 or 1)
* `u_exposure` (float)
* `u_contrast` (float)
* `u_saturation` (float)

Screen-space outlines (optional, gated by `u_outline`):

* `u_outline` (float)
* `u_outlineThreshold` (float)
* `u_outlineThickness` (float)

Weather-driven lens precipitation (optional, gated by `u_lensWeather` and `u_weatherMode`):

* `u_weatherMode` (int) - 0=clear, 1=rain, 2=snow
* `u_weatherIntensity` (float) - 0..1
* `u_windDir` (vec2) - y-down
* `u_windSpeed` (float)
* `u_lensWeather` (float) - 0..1 (strength)
* `u_lensDistort` (float) - 0..1 (refraction amount)
* `u_lensScale` (float) - 0.5..2 (droplet field scale)
* `u_lensDrips` (float) - 0..1 (drip/trail strength)

Optional convenience uniforms (set if present in your custom shader):

* `u_resolution` (vec2) - screen size in pixels
* `u_texelSize` (vec2) - 1/texture size for `texture0`

### Bloom uniforms

The Bloom shaders receive:

* `bloom_extract`: `u_threshold` (float), `u_knee` (float)
* `bloom_blur`: `u_texelSize` (vec2), `u_direction` (vec2), `u_radius` (float)

### WeatherFX uniforms

The WeatherFX shader (if present) receives:

* `u_resolution` (vec2) - screen size in pixels
* `u_time` (float)
* `u_seed` (float in [0,1))
* `u_mode` (int) - 0=clear, 1=rain, 2=snow
* `u_intensity` (float) - 0..1
* `u_windDir` (vec2) - y-down
* `u_windSpeed` (float)
* `u_day` (float) - 0..1 (1=day)

### MaterialFX uniforms

The MaterialFX shader receives:

* `u_time` (float)
* `u_seed` (float in [0,1))
* `u_windDir` (vec2)
* `u_windSpeed` (float)
* `u_texelSize` (vec2) - 1/texture size for `texture0`
* `u_freq` (float) - world-space noise frequency

Water controls:

* `u_waterStrength` (float)
* `u_waterDistortPx` (float)
* `u_waterSparkle` (float)
* `u_foamStrength` (float) - shoreline foam intensity
* `u_foamWidthPx` (float) - foam sampling radius in pixels
* `u_causticsStrength` (float) - animated caustics intensity

Land controls:

* `u_vegStrength` (float)
* `u_wetSandStrength` (float) - wet shoreline sand intensity
* `u_wetSandWidthPx` (float) - wet-sand sampling radius in pixels

And an additional sampler:

* `u_materialMask` (sampler2D) - per-pixel material mask:
  * R = water
  * G = vegetation (grass)
  * B = shoreline sand (sand tiles adjacent to water; may be left 0 elsewhere)

### CloudMask uniforms

The CloudMask shader (if present) receives:

* `u_time` (float)
* `u_seed` (float in [0,1))
* `u_coverage` (float) - 0..1 cloud coverage threshold
* `u_softness` (float) - 0..1 edge softness
* `u_evolve` (float) - 0..1 internal morph speed

## Reloading

In the interactive app, open the dev console (F4) and run:

```
shader_reload
```

This forces recompilation of PostFX, Bloom, volumetric cloud, WeatherFX, MaterialFX, and CloudMask shaders from disk.
