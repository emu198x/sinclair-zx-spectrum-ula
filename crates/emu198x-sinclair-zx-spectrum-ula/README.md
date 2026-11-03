# emu198x-sinclair-zx-spectrum-ula

The video engine every ZX Spectrum-family ULA shares. It covers the screen and
attribute fetch, the two-stage pixel shifter, the border latch, FLASH, the
interrupt window, the floating bus and snow. Each variant configures it with
its own timing and builds its ULA around it. The variants are the Ferranti
6C001E (48K), Sinclair 7K010E (128K), Amstrad 40077 (+2A/+3), Timex SCLD,
Pentagon and Scorpion. Contention and port decoding differ between them, so
they stay in each variant's own crate.

The engine renders into a palette-indexed `u8` framebuffer. It reads the
screen through `MemoryBus` from `emu198x-bus-sinclair-zx-spectrum`, which also
defines the `Ula` trait the variants implement.

## Provenance

Part of [Emu198x](https://github.com/emu198x/emu198x), a family of cycle-accurate
retro-computing emulator cores. This crate is published so siblings and outside
projects can use it on its own; it is versioned independently of the
Emu198x suite and bumps only when it changes.

Licensed GPL-2.0-or-later.

## Save-state compatibility

Version 0.2 preserves the video engine’s pending fetches, border output latch, VidEN and falling-edge phase/address in serialized state. Its positional binary layout differs from 0.1. Callers must version their enclosing save format and reject older payloads before decoding; missing pipeline state cannot be recovered exactly. The 0.2.0 transition kept the stepping API and hardware algorithms unchanged. The owner must still reattach its immutable timing configuration after restore.

## Timex 64-column output

Documented SCLD mode 6 emits all eight pixels of a primary-file byte followed
by all eight pixels of the corresponding secondary-file byte. Port `$FF`
bits 3–5 select ink; paper is its three-bit complement and also fixes the
border. Flash and bright are off, including ink 0 (black on white).

This follows the Timex Computer Corporation *TS2068 Technical Manual*,
Table 3.2.2-1 and §5.2.3, held in the umbrella reference library. Fuse agrees
on byte-column order and complementary colours but sets bright in its
conversion table. The explicit manufacturer bright=0 statement is used here;
this does not claim analogue RGB calibration. Reserved modes 3/4/5/7 retain
legacy, unqualified output and are not four additional documented modes.

The correction retains the positional state layout and the normal Spectrum
rendering paths, but changes Timex mode-6 output and frame routing version
from 5 to 6. Consumers that promise exact continued output must version their
enclosing Timex saves accordingly. The owning Emu198x runtime uses v18.
`UlaEngine::config()` is an additive read-only accessor for the immutable
timing reference; existing callers need no API migration and must still
reattach that reference after deserialization.
