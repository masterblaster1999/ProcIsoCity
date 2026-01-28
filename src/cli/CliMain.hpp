#pragma once

// Shared entrypoint for the headless ProcIsoCity CLI.
//
// This allows the CLI to be:
//  - built as a standalone executable (`proc_isocity_cli`)
//  - optionally embedded inside the interactive app (`proc_isocity`) so one
//    binary can act as both the game and the toolchain.
//
// Implementation: src/cli/main.cpp

namespace isocity {

int ProcIsoCityCliMain(int argc, char** argv);

} // namespace isocity
