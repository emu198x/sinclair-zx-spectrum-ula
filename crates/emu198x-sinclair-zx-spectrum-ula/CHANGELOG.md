# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0](https://github.com/emu198x/sinclair-zx-spectrum-ula/compare/emu198x-sinclair-zx-spectrum-ula-v0.2.1...emu198x-sinclair-zx-spectrum-ula-v0.3.0) - 2026-11-03

### Fixed

- **Breaking:** snow hits one fetch pair, decided where the refresh begins: one beginning at fetch phase 3 gives the first pair (an odd column) the refresh's row; one beginning at phase 7 makes the second pair repeat the first. Three fields are added to the serialized engine, so 0.2 saves must be rejected
- snow corrupts only the video fetch's DRAM row (A0–A6, from `R[6:0]`), keeping the ULA's column, instead of reading the whole refresh address

## [0.2.1](https://github.com/emu198x/sinclair-zx-spectrum-ula/compare/emu198x-sinclair-zx-spectrum-ula-v0.2.0...emu198x-sinclair-zx-spectrum-ula-v0.2.1) - 2026-11-03

### Fixed

- Render Timex mode 6 as complete character columns

## 0.1.0

### Added

- The shared Spectrum-family ULA engine, `UlaEngine`, with its per-variant
  `UlaConfig`s, contention and delay tables, `snow_address`,
  `floating_bus_byte`, `FRAME_ROUTING_VERSION`, and the `attr_to_indices` and
  `attr_flash` attribute decoders. They moved here unchanged from the
  `ula_engine` and `palette` modules of `common-sinclair-zx-spectrum`, the
  family's glue, which re-exports them under those paths.
