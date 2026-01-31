# Round 7

- Added **air quality gameplay**: simulator now computes resident-weighted air pollution exposure and applies a mild happiness penalty (headless) based on average exposure + high-exposure fraction.
  - Code: `src/isocity/Sim.cpp`, settings: `src/isocity/Sim.hpp`, stats fields: `src/isocity/World.hpp`
- Exposed runtime tuning via `Simulator::airPollutionModel()` (enable/disable + config + penalty scales).
- Fixed traffic safety stats wiring to match `TrafficSafetyResult` and existing `Stats` fields.
- Fixed traffic incident severity logic to match `TrafficIncidentSettings` (no missing fields).
- Fixed test helper that used a non-existent `World::applyTool({...})` stroke overload.

Verify:
- Run the app and zone heavy industrial upwind of residential; happiness should drop vs the same layout with buffers/less industry.
- Build tests (`-DPROCISOCITY_BUILD_APP=OFF -DPROCISOCITY_BUILD_TESTS=ON`) and run `proc_isocity_tests`.
