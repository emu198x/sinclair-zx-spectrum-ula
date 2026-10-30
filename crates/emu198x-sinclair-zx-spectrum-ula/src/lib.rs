//! Shared Spectrum-family ULA rendering engine.
//!
//! Every Spectrum-family ULA crate — the Ferranti 6C001E, the Sinclair
//! 7K010E, the Amstrad 40077, the Timex SCLD and the Pentagon and
//! Scorpion ULAs — wraps this engine and adds its own contention and I/O.
//! The machine drives a ULA through the `Ula` trait of
//! `emu198x-bus-sinclair-zx-spectrum`, and the engine reads the screen
//! through that crate's `MemoryBus`.
//!
//! Source references:
//! - `knowledge/systems/spectrum/overview.md`
//! - `knowledge/systems/spectrum/contention.md`
//! - Adapted from `../Emu198x-Older/crates/common-sinclair-zx-spectrum/src/ula_engine.rs`
//!
//! Shared ULA rendering engine — the common display logic across all
/// Spectrum ULA variants (Ferranti, Sinclair 7K, Amstrad 40077, etc.).
///
/// Each variant configures this with its timing constants. The engine
/// handles video fetch, pixel output, interrupt timing, and flash. The
/// variant-specific ULA wrapper adds contention and I/O handling.
use emu198x_bus_sinclair_zx_spectrum::MemoryBus;
use emu198x_bus_sinclair_zx_spectrum::timing::{SCREEN_HEIGHT, SCREEN_WIDTH};

/// Frame routing version. Bumped when the rendering path through this
/// engine (fetch ordering, shifter pipeline depth, palette mapping,
/// border timing) changes in a way that invalidates previously-captured
/// frame hashes in the catalogue. The catalogue manifest carries the
/// version each hash was captured against; a mismatch fails loud with
/// a re-capture instruction.
///
/// **Version 1** (2026-05-19): single-latch shifter model with fetches
/// at pixels 8/10/12/14 and `fetch_start: 8`. Palette derived from
/// BT.601-style values in `palette.rs`. Border updates immediately on
/// `write_fe`. The pre-Seam-1 rendering described in
/// `knowledge/decisions/spectrum-architecture-review.md`.
///
/// **Version 2** (2026-05-19): two-stage shifter (Seam 1)
/// landed alongside the Smith Chapter 16 palette refinement.
/// `MEM_TABLE` and `IDLE_TABLE` shifted four entries left so fetches
/// happen at phases 4/6/8/10 instead of 8/10/12/14; `fetch_start: 4`
/// in all configs; pipeline depth between first fetch and first visible
/// pixel is now 8 pixels (4 T-states) per Smith Chapter 12 Figure 12-2.
/// Float48K strict-mode target: T=14338 for 48K, T=14364 for 128K.
/// Border still updates immediately on `write_fe`.
///
/// **Version 3** (2026-05-19): AOLatch border timing
/// (Smith Chapter 14 p. 134, `/AOLatch = /(/C0 + C1 + /C2)`).
/// `write_fe` still updates `border` (BorderLatch) immediately, but
/// the rendered border colour now reads from `border_aolatch`, which
/// samples `border` every 8 pixels at `(p & 0x07) == 4` un-gated by
/// VidEN. This is the silicon basis for the 8-pixel border-write
/// granularity that border-effect demos exploit. Frame hashes that
/// cover the border region during multi-colour effects will change.
///
/// **Version 4** (2026-07-22): 128K-family rendering uses two
/// ULA edges per five master-clock ticks instead of one edge per two ticks.
/// The version remains ahead of the Spectrum catalogue manifest until
/// affected entries have been reviewed and re-captured.
///
/// **Version 5**: Sinclair fetches use counter phases 8/10/12/14.
/// The memory latch feeds SLoad at 12/20; VidEN delays the border gate,
/// not the fetched byte by another character. IRQ and fetch signals are
/// evaluated before advancing the counter. See z80-irq-deadline-validation.
///
/// Bumps planned: further versions per substantive rendering change.
/// See the architecture review's Seam 4 for the re-capture discipline
/// this constant enforces.
pub const FRAME_ROUTING_VERSION: u32 = 5;

/// Timing and layout constants for a specific ULA variant.
#[derive(Clone, Debug)]
pub struct UlaConfig {
    /// ULA clocks per scanline (448 for 48K, 456 for 128K).
    pub pixels_per_line: u16,
    /// Scanlines per frame (312 for 48K, 311 for 128K).
    pub lines_per_frame: u16,

    /// Pixel position where video fetch begins (8 for Sinclair ULAs).
    pub fetch_start: u16,
    /// Exclusive end of the video-fetch window.
    pub fetch_end: u16,
    /// Pixel position where screen data starts rendering (always 12 — pipeline delay).
    pub screen_start: u16,
    /// Pixel position where screen data ends (screen_start + 256).
    pub screen_end: u16,

    /// Scanline where INT is first asserted.
    pub int_scan: u16,
    /// Pixel within int_scan where INT asserts.
    pub int_start_pixel: u16,
    /// Pixel within int_scan where INT deasserts.
    pub int_end_pixel: u16,

    /// Framebuffer X offset: fb_x = pixel + fb_x_offset (for pixels < fb_x_cutoff).
    pub fb_x_offset: i32,
    /// Pixel value above which fb_x wraps: fb_x = pixel - fb_x_wrap_offset.
    pub fb_x_wrap_start: u16,
    /// Offset for wrapped pixels.
    pub fb_x_wrap_offset: i32,
    /// Pixel value above which fb_x = -1 (HBlank).
    pub fb_x_hblank_start: u16,

    /// Bottom border starts at this scan line.
    pub bottom_border_start: u16,
    /// VSync starts at this scan line (fb_y = -1).
    pub vsync_start: u16,
    /// VSync ends at this scan line (top border resumes).
    pub vsync_end: u16,
}

/// 48K timing (Ferranti 6C001E): 448 pixels/line, 312 lines.
pub const CONFIG_48K: UlaConfig = UlaConfig {
    pixels_per_line: 448,
    lines_per_frame: 312,
    fetch_start: 8,
    fetch_end: 264,
    screen_start: 12,
    screen_end: 12 + 256,
    int_scan: 248,
    int_start_pixel: 1,
    int_end_pixel: 65,
    fb_x_offset: 36,
    fb_x_wrap_start: 412,
    fb_x_wrap_offset: 412,
    fb_x_hblank_start: 316,
    bottom_border_start: 192,
    vsync_start: 248,
    vsync_end: 264,
};

/// Early Toastrack timing (Sinclair 7K010E): 456 pixels/line, 311 lines.
pub const CONFIG_128K: UlaConfig = UlaConfig {
    pixels_per_line: 456,
    lines_per_frame: 311,
    fetch_start: 8,
    fetch_end: 264,
    screen_start: 12,
    screen_end: 12 + 256,
    int_scan: 248,
    int_start_pixel: 5,
    int_end_pixel: 77,
    fb_x_offset: 36,
    fb_x_wrap_start: 420, // 456 - 36
    fb_x_wrap_offset: 420,
    fb_x_hblank_start: 320, // 456 - 136 (wider HBlank)
    bottom_border_start: 192,
    vsync_start: 248,
    vsync_end: 263,
};

/// Grey +2 (Amstrad 40056): late interrupt phase, with a 36T pulse.
/// Both edges precede the early Toastrack by one T-state (two ULA ticks).
/// See `test-data/spectrum-grey-plus2-timing-validation.md` for hardware
/// reports and the pinned SpecIde signal-level comparison.
pub const CONFIG_PLUS2: UlaConfig = UlaConfig {
    int_start_pixel: 3,
    int_end_pixel: 75,
    ..CONFIG_128K
};

/// +2A / +2B / +3 timing (Amstrad 40077).
///
/// Display geometry matches the Sinclair 128K, but the Amstrad ASIC's
/// interrupt pulse is 32 T-states rather than 36.
pub const CONFIG_PLUS2A: UlaConfig = UlaConfig {
    fetch_start: 4,
    fetch_end: 260,
    int_start_pixel: 1,
    int_end_pixel: 65,
    ..CONFIG_128K
};

/// PAL Timex timing, retaining the SCLD fetch and interrupt ordering.
/// Sinclair ULA phase measurements do not establish SCLD timing.
pub const CONFIG_TIMEX_PAL: UlaConfig = UlaConfig {
    fetch_start: 4,
    fetch_end: 260,
    ..CONFIG_48K
};

/// TS2068 NTSC timing: 448 pixels/line, 262 lines, 14.112 MHz crystal.
///
/// NTSC values are from Timex documentation, NOT Smith Chapter 11.
/// Smith covers the unreleased Sinclair 6C011 NTSC ULA (264 lines,
/// 63.5 μs/line) which Timex did not use; the TS2068 uses Timex's own
/// video controller with these constants. See `knowledge/decisions/
/// spectrum-architecture-review.md` verified-non-issue entry.
pub const CONFIG_TS2068: UlaConfig = UlaConfig {
    pixels_per_line: 448,
    lines_per_frame: 262,
    fetch_start: 4,
    fetch_end: 260,
    screen_start: 12,
    screen_end: 12 + 256,
    int_scan: 224, // NTSC: fewer lines, INT earlier
    int_start_pixel: 1,
    int_end_pixel: 65,
    fb_x_offset: 36,
    fb_x_wrap_start: 412,
    fb_x_wrap_offset: 412,
    fb_x_hblank_start: 316,
    bottom_border_start: 192,
    vsync_start: 224,
    vsync_end: 230,
};

/// Pentagon timing: 448 pixels/line (same as 48K), 320 lines (extra VBlank).
/// No contention — the ULA never withholds the CPU clock.
pub const CONFIG_PENTAGON: UlaConfig = UlaConfig {
    pixels_per_line: 448,
    lines_per_frame: 320,
    fetch_start: 4,
    fetch_end: 260,
    screen_start: 12,
    screen_end: 12 + 256,
    int_scan: 256, // Pentagon INT is later than Sinclair (line 256, not 248)
    int_start_pixel: 1,
    int_end_pixel: 65,
    fb_x_offset: 36,
    fb_x_wrap_start: 412,
    fb_x_wrap_offset: 412,
    fb_x_hblank_start: 316,
    bottom_border_start: 192,
    vsync_start: 256,
    vsync_end: 272,
};

/// The rendering and timing state shared by all ULA variants.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct UlaEngine {
    /// Current pixel position within the scanline.
    pub pixel: u16,
    /// Current scanline.
    pub scan: u16,
    /// Border colour (0-7) — the BorderLatch / OutB. Holds the value
    /// most recently written by `write_fe` and reflects CPU writes
    /// immediately. The rendered border colour is `border_aolatch`,
    /// which samples this through the AOLatch every 8 pixels (Smith
    /// Chapter 14 `/AOLatch = /(/C0 + C1 + /C2)`, un-gated by VidEN).
    pub border: u8,
    /// Beeper state (bit 4 of port 0xFE).
    pub beeper: bool,
    /// MIC state (bit 3 of port 0xFE).
    pub mic: bool,
    /// Flash counter: toggles every 16 frames (counts 0..31).
    pub flash_counter: u8,
    /// Flash state: true when ink/paper should be swapped.
    pub flash_active: bool,
    /// CPU clock output: true = CPU may tick.
    pub cpu_clock: bool,
    /// Z80 internal clock phase. Toggles when CPU ticks.
    ///
    /// Read it through [`UlaEngine::gate_arms_this_halfcycle`] rather than
    /// directly: the raw level names the *previous* half-cycle's, and
    /// which of the two a contention gate wants is not a matter of taste.
    pub z80_clock_high: bool,
    /// Interrupt signal.
    pub int_active: bool,
    /// Current byte on the ULA data bus (floating bus).
    pub bus_data: u8,
    /// Is the ULA idle (not reading VRAM)?
    pub idle: bool,
    // Video fetch state.
    //
    // Smith Chapter 12 Figure 12-2 documents a two-stage double buffer
    // per stream: memory → DataLatch → ShiftRegister, clocked by two
    // distinct signals (`DataLatch` and `SLoad`). The `*_pending` fields
    // are the memory-side stage; `data_latch`/`attr_latch` are the
    // DataLatch stage; `data_reg`/`attr_reg` are the ShiftRegister.
    // Transfer points are described in `tick_rendering`.
    //
    // Pending fetches are real pipeline state. They must survive a save
    // taken between a memory fetch and its transfer to DataLatch; otherwise
    // the first resumed pixels differ from uninterrupted execution.
    pub data_reg: u8,
    pub attr_reg: u8,
    pub data_latch: u8,
    pub attr_latch: u8,
    pub data_latch_pending: u8,
    pub attr_latch_pending: u8,
    pub data_addr: u16,
    pub attr_addr: u16,
    /// Active video (scan < 192 and within fetch range).
    pub video: bool,
    /// Border area flag.
    pub border_active: bool,
    /// AOLatch — the latched border colour driving the output mux.
    /// Smith Chapter 14 p. 134: `/AOLatch = /(/C0 + C1 + /C2)`, fires
    /// every 8 CLK7 cycles across the full scanline including border
    /// (un-gated by VidEN). The CPU writes to `border` (BorderLatch /
    /// OutB) instantly via `write_fe`, but the *displayed* colour only
    /// catches up at the next AOLatch trigger. This is the silicon
    /// basis for the well-known 8-pixel border-write granularity that
    /// border-effect demos rely on. Preserve the latched value separately
    /// from BorderLatch so a resume retains the in-flight colour delay.
    pub border_aolatch: u8,

    /// VidEN: `/Border` delayed by one character cell (8 CLK7 = 4 T-states).
    /// Per Smith Chapter 12 p. 134, `SLoad` is gated on `/VidEN` rather
    /// than `/Border`. In the current implementation we gate the SLoad
    /// transfer on `scan < 192` (the rendering scanline window) rather
    /// than tracking VidEN as a separate signal — equivalent for the
    /// 48K timing model because the SLoad/DataLatch transfers happen
    /// uniformly on every active scanline, and the visual offset from
    /// VidEN's one-character delay is already accounted for by
    /// `fetch_start` / `fetch_end`. Field reserved for future fidelity
    /// work (e.g. AOLatch border timing); it remains explicit saved state.
    pub vid_en: bool,
    // Contention tracking
    /// `MREQT23` — Smith Chapter 18, pp. 192-193 and Figure 18-15.
    ///
    /// A **gated D-latch**, not a delay line. `/MREQ` falls halfway
    /// through `T1`; delaying it until the clock goes high at the end of
    /// that T-state produces a signal that is high for `T2` and `T3` and
    /// low for `T1`, which is what disables contention checking for the
    /// rest of an M-cycle once its access has committed. Smith draws the
    /// latch inside the contention handler itself, fed from `/MREQD` —
    /// it is generated within the ULA rather than supplied by the CPU,
    /// so it needs no extra pin on `Ula::tick`.
    ///
    /// The distinction from a delay matters: `z80_mreq_prev` decays after
    /// one half-cycle and lets the gate re-arm in `T3` while the
    /// contended address is still on the bus, charging a second full
    /// rotation. A latch holds across `T2` and `T3` and releases in time
    /// for the next `T1`.
    pub mreq_t23: bool,

    /// `IOREQTW3` — the second gated D-latch of Smith's Figure 18-15, and
    /// the counterpart to [`Self::mreq_t23`]. Where `MREQT23` arms
    /// contention at `T1`, this one *cancels* it once a legitimate I/O
    /// cycle has been recognised.
    ///
    /// Source: `zx_ula`, `fpga_version/rtl/ula.v` — `ioreqtw3 <= ioreq_n`
    /// on `posedge CPUClk`, with `ioreq_n = a[0] | iorq_n`. Stored
    /// active-high, so this field is the HDL register inverted.
    pub ioreq_tw3: bool,

    pub z80_mreq_prev: bool,
    pub z80_iorq_prev: bool,
    pub z80_iorq_prev2: bool,

    /// How many strobe-free falling half-cycles the current M-cycle has
    /// offered so far; 0 when this is not one of them.
    ///
    /// FUSE's `C:n` is *one contention lookup then n T-states*, not a
    /// duration, so an I/O M-cycle's lookup count differs by port class —
    /// see `knowledge/decisions/io-contention-is-a-count-not-a-level.md`.
    /// Expressing a count needs a position inside the M-cycle, and two
    /// half-cycles of `/IORQ` history cannot supply one.
    ///
    /// The falling half-cycles are the ones that matter: they are where the
    /// Sinclair gate arms ([`Self::gate_arms_this_halfcycle`]), so they are
    /// where FUSE's per-T-state lookups land. An M-cycle presents its
    /// address with every strobe released, then drops `/MREQ` half a
    /// T-state later — so a memory cycle offers exactly one strobe-free
    /// falling edge and an I/O cycle, whose `/MREQ` never drops at all,
    /// historically offered two before the old late `/IORQ` edge. With
    /// corrected IORQ timing only the first is strobe-free; the answered
    /// port's offset-1 lookup uses the first visible IORQ edge instead.
    /// This counter remains for diagnostic traces, not contention gating.
    ///
    /// A new address restarts the count, which is what keeps an `Internal`
    /// M-cycle — strobe-free throughout, and carrying the previous cycle's
    /// stale address — from ever reaching 2.
    ///
    /// Saved with the previous falling address so a restore during an
    /// M-cycle preserves the diagnostic phase before the next address change.
    pub mcycle_fall: u8,
    /// The address seen on the previous falling half-cycle, for
    /// [`Self::mcycle_fall`]'s new-address test.
    pub prev_fall_addr: u16,

    /// Framebuffer width (352 for standard, 704 for Timex hi-res capable).
    pub fb_width: usize,

    /// SCLD video mode (port $FF bits 0-2, Timex only). 0 = standard.
    /// Bit 0: dual-screen, Bit 1: hi-colour (8×1 attrs), Bit 2: hi-res (512×192).
    pub scld_mode: u8,
    /// SCLD hi-res ink colour (port $FF bits 3-5, Timex only).
    pub scld_hires_ink: u8,
    /// Second data register for hi-res mode (odd columns from $6000).
    pub data_reg2: u8,
    pub data_latch2: u8,
    /// Hi-res pending-latch slot — mirrors `data_latch_pending` for the
    /// $6000-screen path used by Timex SCLD hi-res mode. Routed through
    /// the same two-stage pipeline so the two streams emerge in lock-step.
    pub data_latch_pending2: u8,

    /// Timing configuration for this variant.
    /// Skipped during serialization — the owning machine must re-set this
    /// after deserialization via `set_config()`.
    #[serde(skip, default = "default_config")]
    config: &'static UlaConfig,
}

/// Sinclair VRAM reads at counter phases 8, 10, 12 and 14.
/// Smith chapter 14 p.132 gates DataLatch with C3; SpecIde's ULA and
/// zx_ula's address multiplexer independently place the same four fetches.
/// Configurations retaining the older phase translate their table index.
pub const MEM_TABLE: [bool; 16] = [
    true, true, true, true, true, true, true, true, false, true, false, true, false, true, false,
    true,
];

/// Sinclair display-bus exposure: C3 high, bitmap/attribute pairs in
/// phases 8..15. The bus is pulled high outside the fetch window.
pub const IDLE_TABLE: [bool; 16] = [
    true, true, true, true, true, true, true, true, false, false, false, false, false, false,
    false, false,
];

/// Scan lines that carry contention — the display lines.
pub const CONTENDED_LINES: u16 = 192;

/// Pixels at the start of each display line over which the ULA may
/// withhold the CPU clock.
///
/// The HDL's `Border_n` is `!hc[8]`: 256 `clk7` cycles, which is sixteen
/// whole 16-pixel fetch cycles and 128 T-states. It begins at a fetch
/// cycle boundary — `hc` 0, eight pixels ahead of that cycle's VRAM
/// access at `hc` 8 — and so does ours, at pixel 0, eight pixels ahead of
/// the access at pixel 8.
///
/// The window is therefore **not** `UlaEngine::video`, which opens at
/// `fetch_start` because it is the fetch window. The two differ by those
/// eight pixels at both ends of all 192 display lines.
pub const CONTENDED_PIXELS_PER_LINE: u16 = 256;

/// The physical fetch counter now agrees with the HDL's counter.
pub const HDL_HC_LEAD_PIXELS: usize = 0;

/// Raw `C2 | C3` wait window (Smith chapter 18, p.192).
/// The Sinclair wrappers evaluate the upcoming control phase, one pixel
/// ahead of the fetch phase: this reproduces SpecIde's 3..14 delay mask.
/// Do not rotate the raw counter to make an instruction-level timestamp fit.
pub const DELAY_TABLE_48K: [bool; 16] = clkwait_from_counter_bits();

/// `C3 + C2` over one 16-pixel cycle, read on our counter's origin.
const fn clkwait_from_counter_bits() -> [bool; 16] {
    let mut table = [false; 16];
    let mut pixel = 0usize;
    while pixel < 16 {
        let hc = (pixel + HDL_HC_LEAD_PIXELS) & 0x0F;
        table[pixel] = (hc >> 2) & 1 == 1 || (hc >> 3) & 1 == 1;
        pixel += 1;
    }
    table
}

/// +2A/+3 contention delay table, indexed by `pixel & 0x0F`.
/// `true` = the gate array may withhold the CPU clock this half-cycle.
///
/// Eleven contiguous asserted half-cycles from pixel 4, free over
/// pixels 15 and 0–3.
///
/// **This mask is measured, not derived.** Unlike `DELAY_TABLE_48K`,
/// which `clkwait_from_counter_bits` builds from `C3 + C2` and a stated
/// counter relationship, no primary description of the Amstrad gate
/// array's clock-withholding window is in hand. What fixes it here is
/// `machine-sinclair-zx-spectrum-plus2a`'s arrival-resolved differential
/// against FUSE: sweeping run length 2–14 against every one of the
/// sixteen start positions puts a single sharp minimum at length 11,
/// start 4 — **5,166 of 375,407** samples disagreeing, against
/// 85,962–92,489 at every adjacent start and 88,485 at length 10.
/// A seventeen-fold gap to its neighbours is what makes it a
/// measurement rather than a fit.
///
/// The previous literal asserted three half-cycles and scored 149,185 of
/// 442,666. It undercharged everywhere — a single-M-cycle instruction
/// was never contended at any arrival T-state, `NOP` costing 4.00
/// against FUSE's mean 9.00.
///
/// Note what this mask is *not*. FUSE contends the +3 with
/// `contention_pattern_76543210` (`{5,4,3,2,1,0,7,6}` read at offset 4,
/// so `1,0,7,6,5,4,3,2`), which spends seven of every eight T-states
/// contended and one free. Reproducing that ratio directly — fourteen
/// asserted half-cycles of sixteen — overcharges by roughly double,
/// `NOP` reaching 14.67 against a wanted 7.44, and scores 45,707. The
/// gate array's window does not map onto FUSE's per-T-state table by a
/// simple count, and `cpu_divisor` 5 puts 2.5 pixels in a T-state, so
/// this table's domain and FUSE's are not the same grid. Deriving the
/// window from the hardware remains open; see #856 for why fitting to a
/// frame *maximum* rather than an arrival-resolved differential is the
/// trap here.
pub const DELAY_TABLE_PLUS2A: [bool; 16] = [
    false, false, false, false, true, true, true, true, true, true, true, true, true, true, true,
    false,
];

/// Default config for serde deserialization. The owning machine must call
/// `set_config()` immediately after deserializing to install the correct config.
fn default_config() -> &'static UlaConfig {
    &CONFIG_48K
}

/// Compute the snow-effect fetch override for one ULA tick.
///
/// Returns `Some(refresh_addr)` when the CPU is presenting a refresh
/// address in screen-RAM range — `cpu_rfsh` is asserted (T3/T4 of an M1,
/// `I:R` on the bus) and the I register is in `$40..=$7F`, i.e. the
/// address falls in `$4000..=$7FFF`. In that case a colliding CPU `/RAS`
/// corrupts the in-flight video fetch and the ULA reads from the refresh
/// address instead. `None` otherwise. ULAs that exhibit snow (Ferranti
/// 48K, Sinclair 128K) feed the result to `tick_rendering`; the Amstrad
/// gate array and the clone ULAs pass `None`.
///
/// Mechanism: Chris Smith, *The ZX Spectrum ULA*, pp. 246–248. See
/// `knowledge/systems/spectrum/snow-effect.md`.
#[must_use]
pub fn snow_address(cpu_rfsh: bool, cpu_addr: u16) -> Option<u16> {
    if cpu_rfsh && (cpu_addr & 0xC000) == 0x4000 {
        Some(cpu_addr)
    } else {
        None
    }
}

/// Spectron/FUSE floating-bus model byte at frame T-state `frame_tstate`
/// (FUSE convention). Idle (`0xFF`) outside the four fetch slots of each
/// 8-T group and outside the 192-line display; bitmap/attr otherwise.
/// This is an instruction-level reference helper, not live pin timing.
#[must_use]
pub fn floating_bus_byte(
    frame_tstate: u32,
    float_start: u32,
    tstates_per_line: u32,
    memory: &dyn MemoryBus,
) -> u8 {
    // `float_start` is the T-state of the *first* data byte, not the last
    // idle one — FUSE returns `display_line_start[0]` at 14338 itself.
    // Treating it as exclusive put every slot one T-state late across the
    // whole frame; see `fuse_floating_bus_differential`.
    if frame_tstate < float_start {
        return 0xFF;
    }
    let rel = frame_tstate - float_start;
    let line = rel / tstates_per_line;
    if line >= 192 {
        return 0xFF;
    }
    let col = rel % tstates_per_line;
    if col >= 128 {
        return 0xFF;
    }
    let goff = (col & 0x07) as u16;
    if goff >= 4 {
        return 0xFF;
    }
    let group = (col / 8) as u16;
    let cc = 2 * group + (goff >> 1);
    let y = line as u16;
    let addr = if goff & 0x01 == 0 {
        0x4000 | UlaEngine::compute_data_addr(y).wrapping_add(cc)
    } else {
        0x4000 | UlaEngine::compute_attr_addr(y).wrapping_add(cc)
    };
    memory.read_screen(addr)
}

impl UlaEngine {
    /// Re-install the timing config after deserialization.
    pub fn set_config(&mut self, config: &'static UlaConfig) {
        self.config = config;
    }

    /// Create with a wider framebuffer for Timex hi-res support.
    pub fn new_hires(config: &'static UlaConfig) -> Self {
        let mut e = Self::new(config);
        e.fb_width = emu198x_bus_sinclair_zx_spectrum::timing::SCREEN_WIDTH_HIRES;
        e
    }

    pub fn new(config: &'static UlaConfig) -> Self {
        Self {
            pixel: 0,
            scan: 0,
            border: 7,
            beeper: false,
            mic: false,
            flash_counter: 0,
            flash_active: false,
            cpu_clock: true,
            z80_clock_high: true,
            int_active: false,
            bus_data: 0xFF,
            idle: true,
            data_reg: 0,
            attr_reg: 0,
            data_latch: 0,
            attr_latch: 0,
            data_latch_pending: 0,
            attr_latch_pending: 0,
            data_addr: 0,
            attr_addr: 0,
            video: false,
            border_active: true,
            border_aolatch: 7,
            vid_en: false,
            mreq_t23: false,
            ioreq_tw3: false,
            z80_mreq_prev: false,
            z80_iorq_prev: false,
            z80_iorq_prev2: false,
            mcycle_fall: 0,
            prev_fall_addr: 0,
            fb_width: SCREEN_WIDTH,
            scld_mode: 0,
            scld_hires_ink: 0,
            data_reg2: 0,
            data_latch2: 0,
            data_latch_pending2: 0,
            config,
        }
    }

    /// Compute bitmap VRAM address for the given scanline.
    #[inline]
    pub fn compute_data_addr(scan: u16) -> u16 {
        ((scan & 0x38) << 2) | ((scan & 0x07) << 8) | ((scan & 0xC0) << 5)
    }

    /// Compute attribute VRAM address for the given scanline.
    #[inline]
    pub fn compute_attr_addr(scan: u16) -> u16 {
        0x1800 | ((scan & 0xF8) << 2)
    }

    /// Run the rendering portion of a ULA tick: video fetch, pixel output,
    /// counter advance, and interrupt timing. Does NOT handle contention —
    /// that's the variant wrapper's job.
    /// `snow`: when `Some(refresh_addr)`, the CPU is driving a refresh
    /// address in screen-RAM range (`I` ∈ `$40..=$7F`) that collides with
    /// this cycle's video fetch — the ULA reads from `refresh_addr`
    /// instead of its intended screen address, producing the "snow"
    /// artefact. See `knowledge/systems/spectrum/snow-effect.md`. `None`
    /// for the overwhelming majority of cycles (and for ULAs that don't
    /// model snow).
    pub fn tick_rendering(
        &mut self,
        memory: &dyn MemoryBus,
        framebuffer: &mut [u8],
        snow: Option<u16>,
    ) {
        let p = self.pixel as usize;
        let phase = p & 0x0F;
        let cfg = self.config;

        // === Checkpoint events ===
        match self.pixel {
            0 => {
                self.border_active = self.scan >= 192;
            }
            x if x == cfg.fetch_start => {
                self.video = self.scan < 192;
                if self.video {
                    self.data_addr = Self::compute_data_addr(self.scan);
                    self.attr_addr = Self::compute_attr_addr(self.scan);
                }
                self.border_active = !self.video;
            }
            256 => {
                self.border_active = true;
            }
            x if x == cfg.fetch_end => {
                self.video = false;
            }
            _ => {}
        }

        // Sinclair: the first pair is fetched at 8/10 and consumed by
        // SLoad at 12, before the next bitmap fetch overwrites its latch.
        // Smith p.132 delays /Border until C3; it does not add an entire
        // character of delay after DataLatch. Other variants retain their
        // existing transfer phase until independently validated.
        if self.scan < 192 {
            if (p & 0x07) == usize::from((cfg.fetch_start + 4) & 7) {
                self.data_latch = self.data_latch_pending;
                self.data_latch2 = self.data_latch_pending2;
                self.attr_latch = self.attr_latch_pending;
            }
            if (p & 0x07) == 4 {
                self.data_reg = self.data_latch;
                self.data_reg2 = self.data_latch2;
                self.attr_reg = self.attr_latch;
            }
        }

        // === AOLatch transfer (Smith Ch 14 p. 134) ===
        // `/AOLatch = /(/C0 + C1 + /C2)` fires every 8 CLK7 cycles, un-gated
        // by VidEN — across the full scanline including border. The latched
        // colour is what the screen output mux passes to the video DAC. In
        // our 16-pixel-cell indexing, AOLatch fires at `(p & 0x07) == 4`,
        // immediately following SLoad in each cycle (Smith: "AOLatch goes
        // low while SLoad is low. This presents the attribute byte to the
        // colour output multiplexer at the exact moment the display byte
        // is loaded into the shift register."). The result is the
        // well-known 8-pixel border-write granularity that border-effect
        // demos exploit. Unlike SLoad, this trigger is unconditional:
        // border colour can change anywhere on the scanline.
        if (p & 0x07) == 4 {
            self.border_aolatch = self.border;
        }

        // === Video fetch ===
        if self.video {
            let fetch_phase = (phase + 8 - usize::from(cfg.fetch_start)) & 0x0f;
            self.idle = IDLE_TABLE[fetch_phase];
            let hires = self.scld_mode & 0x04 != 0;
            let hicolour = self.scld_mode & 0x02 != 0;
            let dual = self.scld_mode & 0x01 != 0;

            // VRAM reads at the configured phases (two RAS-CAS fetch pairs
            // per 16-pixel cycle). bus_data is set at the moment of
            // each CAS strobe — this is the value `IN A,($FF)` samples
            // via the floating-bus path on the ULA.
            if !MEM_TABLE[fetch_phase] {
                if phase & 0x02 == 0 {
                    // Bitmap fetch (CAS-A or CAS-C falling)
                    let a = self.data_addr;
                    self.data_addr = self.data_addr.wrapping_add(1);
                    let base = if dual && !hires { 0x6000u16 } else { 0x4000u16 };
                    // Snow: a colliding refresh /RAS corrupts the fetched
                    // row address, so the ULA reads from the refresh
                    // address instead. The fetch counter still advances.
                    let addr = snow.unwrap_or(base | (a & 0x1FFF));
                    self.bus_data = memory.read_screen(addr);
                    self.data_latch_pending = self.bus_data;

                    // Hi-res: also fetch from the other screen for odd columns
                    if hires {
                        self.data_latch_pending2 = memory.read_screen(0x6000 | (a & 0x1FFF));
                    }
                } else {
                    // Attribute fetch (CAS-B or CAS-D falling)
                    let a = self.attr_addr;
                    self.attr_addr = self.attr_addr.wrapping_add(1);

                    if hires {
                        // Hi-res: no attributes — colour from port $FF bits 3-5
                        self.bus_data = 0xFF;
                        self.attr_latch_pending = 0xFF;
                    } else if hicolour {
                        // Hi-colour: attribute from $6000 + bitmap_offset
                        // (each pixel row has its own attribute, not 8×8 cells)
                        let bitmap_offset = Self::compute_data_addr(self.scan);
                        let attr_addr = 0x6000 | (bitmap_offset & 0x1FFF) | (a & 0x1F); // column
                        self.bus_data = memory.read_screen(attr_addr);
                        self.attr_latch_pending = self.bus_data;
                    } else {
                        let base = if dual { 0x7800u16 } else { 0x5800u16 };
                        // For dual-screen without hi-colour: attrs from second screen area
                        let intended = if dual {
                            base | (a & 0x02FF)
                        } else {
                            0x4000 | (a & 0x1FFF)
                        };
                        // Snow corrupts the attribute fetch the same way.
                        let addr = snow.unwrap_or(intended);
                        self.bus_data = memory.read_screen(addr);
                        self.attr_latch_pending = self.bus_data;
                    }
                }
            }
        } else {
            self.idle = true;
            self.bus_data = 0xFF;
        }

        // === Pixel output ===
        // The scale factor: 2 for hi-res framebuffers (704 wide), 1 for standard (352).
        let hscale = if self.fb_width > SCREEN_WIDTH { 2 } else { 1 };
        let hires = self.scld_mode & 0x04 != 0;

        let fb_x: i32 = if self.pixel >= cfg.fb_x_wrap_start {
            ((p as i32) - cfg.fb_x_wrap_offset) * hscale
        } else if self.pixel < cfg.fb_x_hblank_start {
            ((p as i32) + cfg.fb_x_offset) * hscale
        } else {
            -1
        };

        let fb_y: i32 = if self.scan < cfg.vsync_start {
            self.scan as i32 + 48
        } else if self.scan < cfg.vsync_end {
            -1
        } else {
            (self.scan as i32) - cfg.vsync_end as i32
        };

        let fw = self.fb_width as i32;
        if fb_x >= 0 && fb_x < fw && fb_y >= 0 && fb_y < SCREEN_HEIGHT as i32 {
            let in_screen =
                self.scan < 192 && self.pixel >= cfg.screen_start && self.pixel < cfg.screen_end;

            if in_screen && hires && hscale == 2 {
                // Hi-res on wide framebuffer: 2 pixels per ULA clock
                let bit1 = (self.data_reg >> 7) & 1;
                let bit2 = (self.data_reg2 >> 7) & 1;
                self.data_reg <<= 1;
                self.data_reg2 <<= 1;
                let ink = self.scld_hires_ink;
                let px_even = if bit1 != 0 { ink } else { 0 };
                let px_odd = if bit2 != 0 { ink } else { 0 };
                let off = fb_y as usize * self.fb_width + fb_x as usize;
                if off + 1 < framebuffer.len() {
                    framebuffer[off] = px_even;
                    framebuffer[off + 1] = px_odd;
                }
            } else if in_screen {
                // Standard / hi-colour: attribute-based rendering
                let attr = self.attr_reg;
                let (ink, paper) = attr_to_indices(attr);
                let flash = attr_flash(attr) && self.flash_active;
                let bit = (self.data_reg >> 7) & 1;
                self.data_reg <<= 1;
                let pixel_idx = if (bit != 0) ^ flash { ink } else { paper };
                let off = fb_y as usize * self.fb_width + fb_x as usize;
                if off < framebuffer.len() {
                    framebuffer[off] = pixel_idx;
                    // On wide framebuffer, double each standard pixel
                    if hscale == 2 && off + 1 < framebuffer.len() {
                        framebuffer[off + 1] = pixel_idx;
                    }
                }
            } else {
                // Border — driven by the AOLatch, not the BorderLatch.
                // 8-pixel granularity per Smith Ch 14 (see AOLatch
                // transfer comment above).
                let off = fb_y as usize * self.fb_width + fb_x as usize;
                if off < framebuffer.len() {
                    framebuffer[off] = self.border_aolatch;
                    if hscale == 2 && off + 1 < framebuffer.len() {
                        framebuffer[off + 1] = self.border_aolatch;
                    }
                }
            }
        } else if self.scan < 192 && self.pixel >= cfg.screen_start && self.pixel < cfg.screen_end {
            self.data_reg <<= 1;
            if hires {
                self.data_reg2 <<= 1;
            }
        }

        if cfg.fetch_start == 8 {
            self.update_interrupt();
        }

        // === Advance pixel counter ===
        self.pixel += 1;
        if self.pixel >= cfg.pixels_per_line {
            self.pixel = 0;
            self.scan += 1;
            if self.scan >= cfg.lines_per_frame {
                self.scan = 0;
            }
        }

        if cfg.fetch_start != 8 {
            self.update_interrupt();
        }
    }

    fn update_interrupt(&mut self) {
        let cfg = self.config;
        if self.scan == cfg.int_scan {
            if self.pixel == cfg.int_start_pixel {
                self.int_active = true;
            } else if self.pixel == cfg.int_end_pixel {
                self.int_active = false;
            }
        }
    }

    /// Is this the half-cycle whose CPU clock edge would begin a bus access?
    ///
    /// The Sinclair ULAs stall the CPU *before* it commits to a contended
    /// access: they see a contended address with `/MREQ` still inactive and
    /// withhold the clock edge that would drop it. That edge is `T1`↓ — a
    /// `Fall` phase — so this is the only half-cycle per M-cycle on which a
    /// memory access can be contended, now that `/MREQ` runs `T1b`–`T3b`
    /// and covers the other five.
    ///
    /// `z80_clock_high` is sampled before `track_z80_clock` advances it, so
    /// it names the level the CPU clock held during the previous
    /// half-cycle: true when the CPU is about to run a `Rise` phase, false
    /// when it is about to run a `Fall` phase. That is measured, not
    /// assumed — see `machine-sinclair-zx-spectrum-48k`'s
    /// `contention_arming`, which records the real machine from inside
    /// `tick` and reports which half-cycles offer the gate an arming
    /// opportunity and what this term reads on each.
    ///
    /// So the gate arms on the complement. While the CPU is stalled
    /// `track_z80_clock` does not run and this holds, which is what keeps
    /// the stall asserted until the delay table's window closes — the
    /// stall self-limits on the raster, not on a counter.
    #[must_use]
    pub fn gate_arms_this_halfcycle(&self) -> bool {
        !self.z80_clock_high
    }

    /// Advance [`Self::mcycle_fall`] — called *before* the contention
    /// decision, unlike every other latch here.
    ///
    /// The ordering is forced, and it is why this is not folded into
    /// [`Self::track_z80_clock`] as
    /// `knowledge/decisions/io-contention-is-a-count-not-a-level.md`
    /// sketched. `track_z80_clock` runs after the decision and is handed the
    /// pins the decision was made on; a counter updated there names the
    /// half-cycle that has just been decided, so a gate reading it would be
    /// one falling edge behind and would charge FUSE's offset 1 at offset 2.
    /// This runs first and names the half-cycle being decided now.
    ///
    /// A stalled CPU has not moved, so neither has its M-cycle: `cpu_clock`
    /// here is the previous half-cycle's decision, and while it is false the
    /// pins and the clock phase are both frozen and the count must freeze
    /// with them. Counting through a stall would inflate the position by the
    /// length of the stall the gate itself caused.
    pub fn track_mcycle_fall(&mut self, cpu_addr: u16, cpu_mreq: bool, cpu_iorq: bool) {
        if !self.cpu_clock || !self.gate_arms_this_halfcycle() {
            return;
        }
        self.mcycle_fall = if cpu_mreq || cpu_iorq {
            0
        } else if cpu_addr != self.prev_fall_addr {
            1
        } else if self.mcycle_fall > 0 {
            self.mcycle_fall.saturating_add(1)
        } else {
            // A strobe-free falling edge on a *stale* address, with no run
            // in progress: an `Internal` M-cycle sitting on the refresh
            // address. It offers the memory gate its usual arming
            // opportunity but it is not an M-cycle position, so the count
            // stays shut rather than walking up to 2.
            0
        };
        self.prev_fall_addr = cpu_addr;
    }

    /// Track Z80 clock phase (called after contention decision).
    ///
    /// `ula_io` is the variant's own decode of "the ULA answers this port
    /// and `IORQ` is asserted" — `a[0] | iorq_n` in the HDL, inverted.
    pub fn track_z80_clock(&mut self, cpu_iorq: bool, cpu_mreq: bool, ula_io: bool) {
        if self.cpu_clock {
            self.z80_iorq_prev2 = self.z80_iorq_prev;
            self.z80_iorq_prev = cpu_iorq;
            self.z80_mreq_prev = cpu_mreq;
            self.z80_clock_high = !self.z80_clock_high;
            // Latch `/MREQ` on the rising clock edge. While the CPU is
            // stalled this method does not run, so the latch holds — which
            // is correct: a stalled CPU sits in `T1` with `MREQ` inactive
            // and must keep contending.
            // Both latches share the edge in the HDL (`posedge CPUClk`).
            if self.z80_clock_high {
                self.mreq_t23 = cpu_mreq;
                self.ioreq_tw3 = ula_io;
            }
        }
    }

    /// End-of-frame housekeeping.
    pub fn end_frame(&mut self) {
        self.flash_counter = (self.flash_counter + 1) & 0x1F;
        self.flash_active = self.flash_counter >= 16;
        self.pixel = 0;
        self.scan = 0;
    }

    /// Write port 0xFE.
    pub fn write_fe(&mut self, val: u8) {
        self.border = val & 0x07;
        self.mic = val & 0x08 != 0;
        self.beeper = val & 0x10 != 0;
    }

    /// Read port 0xFE (keyboard + EAR).
    pub fn read_fe(&self, port: u16, keyboard: &[u8; 8]) -> u8 {
        let mut result = 0xBF;
        let high = (port >> 8) as u8;
        let mut keys = 0x1F;
        for (row, row_state) in keyboard.iter().enumerate().take(8) {
            if high & (1 << row) == 0 {
                keys &= *row_state;
            }
        }
        result = (result & 0xE0) | (keys & 0x1F);
        // EAR bit high. Each ULA wrapper sets its own idle level over
        // this; only the Timex SCLD keeps it.
        result |= 0x40;
        result
    }
}

/// Convert a Spectrum attribute byte to ink and paper palette indices.
///
/// Attribute format: FBPPPIII
///   F = flash, B = bright, PPP = paper colour (0-7), III = ink colour (0-7)
///
/// Returns (ink_index, paper_index) into the 16-colour palette.
#[inline]
pub fn attr_to_indices(attr: u8) -> (u8, u8) {
    let bright = if attr & 0x40 != 0 { 8 } else { 0 };
    let ink = (attr & 0x07) | bright;
    let paper = ((attr >> 3) & 0x07) | bright;
    (ink, paper)
}

/// Check if an attribute has the FLASH bit set.
#[inline]
pub fn attr_flash(attr: u8) -> bool {
    attr & 0x80 != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use emu198x_bus_sinclair_zx_spectrum::MemoryBus;

    /// Minimal MemoryBus stub returning 0 for every read — sufficient
    /// for border-rendering tests that don't depend on bitmap or
    /// attribute content.
    struct ZeroMemory;
    impl MemoryBus for ZeroMemory {
        fn read(&self, _addr: u16) -> u8 {
            0
        }
        fn write(&mut self, _addr: u16, _value: u8) {}
        fn is_contended(&self, _addr: u16) -> bool {
            false
        }
    }

    /// AOLatch behaviour: `write_fe` updates `border` (BorderLatch)
    /// immediately, but `border_aolatch` (the rendered colour) only
    /// catches up at the next `(p & 0x07) == 4` boundary. This is the
    /// silicon basis for the well-known 8-pixel border-write
    /// granularity (Smith Ch 14 p. 134, `/AOLatch = /(/C0 + C1 + /C2)`,
    /// un-gated by VidEN).
    #[test]
    fn aolatch_defers_border_writes_to_next_8_pixel_boundary() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        let mem = ZeroMemory;
        let mut fb = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT];

        // Position the engine just past an AOLatch trigger. AOLatch
        // samples on a tick that *starts* with pixel ≡ 4 (mod 8), so
        // starting at pixel 5 the next trigger fires on the tick where
        // pixel = 12 at entry. The counter increments at the end of
        // each tick.
        e.pixel = 5;
        e.scan = 0;
        e.border_aolatch = 7;
        e.border = 7;

        // CPU writes border = 2 (red). BorderLatch updates instantly,
        // AOLatch does not.
        e.write_fe(0x02);
        assert_eq!(e.border, 2, "BorderLatch reflects CPU write instantly");
        assert_eq!(
            e.border_aolatch, 7,
            "AOLatch still holds the previous colour"
        );

        // Tick 7 times: pixel 5→12 at end, none of the entry-pixels
        // (5..=11) match p&7==4.
        for _ in 0..7 {
            e.tick_rendering(&mem, &mut fb, None);
        }
        assert_eq!(e.pixel, 12);
        assert_eq!(e.border_aolatch, 7, "AOLatch unchanged before the boundary");

        // Eighth tick: entry pixel = 12 (matches p&0x07==4). AOLatch
        // samples BorderLatch. Pixel increments to 13.
        e.tick_rendering(&mem, &mut fb, None);
        assert_eq!(e.pixel, 13);
        assert_eq!(
            e.border_aolatch, 2,
            "AOLatch sampled BorderLatch on its trigger"
        );
    }

    /// AOLatch fires regardless of scan position — Smith Ch 14
    /// confirms `/AOLatch` has no VidEN gate, so the trigger is
    /// equally active during border scanlines (scan >= 192) and
    /// active video.
    #[test]
    fn aolatch_fires_during_border_scanlines() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        let mem = ZeroMemory;
        let mut fb = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT];

        // Place the engine in a bottom-border scanline at pixel 4 —
        // entry pixel matches p&7==4, so the next tick fires AOLatch.
        e.pixel = 4;
        e.scan = 250; // well into bottom border
        e.border_aolatch = 7;
        e.write_fe(0x04); // border = green

        e.tick_rendering(&mem, &mut fb, None);
        assert_eq!(e.pixel, 5);
        assert_eq!(
            e.border_aolatch, 4,
            "AOLatch fired during border-only scanline"
        );
    }

    /// Snow gate (issue #12): the effect arms only when the CPU is in a
    /// refresh half-cycle (`rfsh`) AND the refresh address (`I:R`) lands
    /// in screen-RAM range — `I` ∈ `$40..=$7F`, i.e. `$4000..=$7FFF`.
    #[test]
    fn snow_address_gates_on_refresh_and_i_register_range() {
        // Armed: refresh active, I in $40..=$7F.
        assert_eq!(snow_address(true, 0x4000), Some(0x4000));
        assert_eq!(snow_address(true, 0x5B42), Some(0x5B42));
        assert_eq!(snow_address(true, 0x7FFF), Some(0x7FFF));
        // Not a refresh cycle → never snows.
        assert_eq!(snow_address(false, 0x4000), None);
        // I below the screen bank (ROM / IM1 vector area, e.g. I=$3F).
        assert_eq!(snow_address(true, 0x3FFF), None);
        // I at/above $80 (the usual IM2 table region) → no collision.
        assert_eq!(snow_address(true, 0x8000), None);
        assert_eq!(snow_address(true, 0xBE00), None);
    }

    /// A memory whose `read` returns the low byte of the address, so a
    /// fetched value reveals which address the ULA actually read.
    struct AddrLowMemory;
    impl MemoryBus for AddrLowMemory {
        fn read(&self, addr: u16) -> u8 {
            (addr & 0xFF) as u8
        }
        fn write(&mut self, _addr: u16, _value: u8) {}
        fn is_contended(&self, _addr: u16) -> bool {
            false
        }
    }

    /// When snow is armed, the bitmap fetch reads from the refresh
    /// address instead of the intended screen address — the visible
    /// corruption. The fetch counter still advances either way.
    #[test]
    fn snow_corrupts_the_bitmap_fetch_address() {
        let mem = AddrLowMemory;
        let mut fb = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT];

        // Pixel 12 is a bitmap-fetch phase (MEM_TABLE[12] == false,
        // phase & 0x02 == 0) that is NOT `fetch_start` (8), so it doesn't
        // recompute `data_addr` — letting us pin a known fetch address. A
        // visible scanline (scan < 192) keeps `video` set.
        let setup = |e: &mut UlaEngine| {
            e.video = true;
            e.scan = 100;
            e.pixel = 12;
            e.data_addr = 0x0142;
            e.scld_mode = 0;
        };

        // No snow: reads the intended address $4000 | (0x142 & 0x1FFF)
        // = $4142 → low byte $42.
        let mut clean = UlaEngine::new(&CONFIG_48K);
        setup(&mut clean);
        clean.tick_rendering(&mem, &mut fb, None);
        assert_eq!(clean.bus_data, 0x42, "clean fetch reads the screen address");

        // Snow armed at refresh address $4099: the ULA reads $4099
        // instead → low byte $99.
        let mut snowy = UlaEngine::new(&CONFIG_48K);
        setup(&mut snowy);
        snowy.tick_rendering(&mem, &mut fb, Some(0x4099));
        assert_eq!(
            snowy.bus_data, 0x99,
            "snow redirects the fetch to the refresh address"
        );
        assert_eq!(
            snowy.data_addr, 0x0143,
            "the fetch counter still advances under snow"
        );
    }

    /// `floating_bus_byte` reproduces the Spectron/FUSE model: idle
    /// (`0xFF`) outside the four data slots of each 8-T group and outside
    /// the 192-line display; bitmap / attribute at the data slots.
    /// Column-encoded screen so each fetched byte reveals its column.
    struct ColumnMemory;
    impl MemoryBus for ColumnMemory {
        fn read(&self, addr: u16) -> u8 {
            if (0x5800..0x5B00).contains(&addr) {
                0x80 | (addr & 0x1F) as u8
            } else {
                (addr & 0x1F) as u8
            }
        }
        fn write(&mut self, _addr: u16, _value: u8) {}
        fn is_contended(&self, _addr: u16) -> bool {
            false
        }
    }

    #[test]
    fn floating_bus_byte_matches_the_spectron_model() {
        let mem = ColumnMemory;
        const FS: u32 = 14_338; // 48K FloatingBusStartTicks
        let b = |t: u32| floating_bus_byte(t, FS, 224, &mem);

        // `float_start` is the T-state of the *first data byte*, not the
        // last idle one. This test previously indexed from `FS + 1`, which
        // encoded the one-T-state lag corrected on 2026-08-10 — see
        // `fuse_floating_bus_differential`, which checks the whole frame
        // against FUSE's `spectrum_unattached_port` rather than four slots.
        //
        // First 8-T group of display line 0: bitmap col0, attr col0,
        // bitmap col1, attr col1, then four idle slots.
        assert_eq!(b(FS), 0x00, "rel 0 → bitmap column 0");
        assert_eq!(b(FS + 1), 0x80, "rel 1 → attribute column 0");
        assert_eq!(b(FS + 2), 0x01, "rel 2 → bitmap column 1");
        assert_eq!(b(FS + 3), 0x81, "rel 3 → attribute column 1");
        for rel in 4..8 {
            assert_eq!(b(FS + rel), 0xFF, "rel {rel} idle");
        }
        // Before the first data T-state, the right border (col >= 128), and
        // below the 192-line display all read idle.
        assert_eq!(b(FS - 1), 0xFF, "before float_start");
        assert_eq!(b(FS + 128), 0xFF, "column 128 past the active region");
        assert_eq!(b(FS + 192 * 224), 0xFF, "line 192 past the display");
    }
}

#[cfg(test)]
mod fuse_floating_bus_differential {
    use super::*;
    use emu198x_bus_sinclair_zx_spectrum::MemoryBus;

    /// Screen memory whose byte identifies its own address, and which
    /// never returns `0xFF` — so `0xFF` unambiguously means "idle bus"
    /// and any other mismatch means the two models addressed different
    /// bytes.
    struct AddressCodedScreen;
    impl MemoryBus for AddressCodedScreen {
        fn read(&self, addr: u16) -> u8 {
            (addr % 251) as u8
        }
        fn write(&mut self, _addr: u16, _value: u8) {}
        fn is_contended(&self, _addr: u16) -> bool {
            true
        }
    }

    /// FUSE's `spectrum_unattached_port`, transcribed.
    ///
    /// Constants from `libspectrum` `timings_frame_ferranti_5c_6c`
    /// (left_border 24, horizontal_screen 128, 224 T/line, top_left_pixel
    /// 14336) and FUSE's `machine.c`, where
    /// `line_times[0] = top_left_pixel - 24*224 - 16 = 8944`, so the first
    /// screen line starts at `line_times[24] = 14320`.
    ///
    /// This is the reference implementation, not a restatement of ours —
    /// the point is to disagree where we are wrong.
    fn fuse_unattached_port(t: u32, memory: &dyn MemoryBus) -> u8 {
        const FIRST_SCREEN_LINE_T: u32 = 14_320;
        const PER_LINE: u32 = 224;
        const LEFT_BORDER: u32 = 24;
        const HORIZONTAL_SCREEN: u32 = 128;
        // `left_border - DISPLAY_BORDER_WIDTH_COLS * 4` = 24 - 16.
        const THROUGH_LINE_ADJUST: u32 = 8;

        if t < FIRST_SCREEN_LINE_T {
            return 0xFF;
        }
        let line = (t - FIRST_SCREEN_LINE_T) / PER_LINE;
        if line >= 192 {
            return 0xFF;
        }
        let through_line = t - (FIRST_SCREEN_LINE_T + line * PER_LINE) + THROUGH_LINE_ADJUST;
        if !(LEFT_BORDER..LEFT_BORDER + HORIZONTAL_SCREEN).contains(&through_line) {
            return 0xFF;
        }
        let mut column = ((through_line - LEFT_BORDER) / 8) * 2;
        let y = line as u16;
        match through_line % 8 {
            5 => {
                column += 1;
                memory.read_screen(
                    0x4000 | UlaEngine::compute_attr_addr(y).wrapping_add(column as u16),
                )
            }
            3 => memory
                .read_screen(0x4000 | UlaEngine::compute_attr_addr(y).wrapping_add(column as u16)),
            4 => {
                column += 1;
                memory.read_screen(
                    0x4000 | UlaEngine::compute_data_addr(y).wrapping_add(column as u16),
                )
            }
            2 => memory
                .read_screen(0x4000 | UlaEngine::compute_data_addr(y).wrapping_add(column as u16)),
            _ => 0xFF,
        }
    }

    /// Our floating-bus model must agree with FUSE's at every T-state of
    /// the frame.
    ///
    /// This compares *models* rather than outcomes, which is the point:
    /// every floating-bus experiment routed through a test program also
    /// routed through that program's conventions — Ramsoft's first byte
    /// at 14347 against FUSE's 14338 is a nine-T-state gap sitting
    /// between the emulator and the answer. A direct diff removes it, and
    /// cannot be satisfied by tuning a constant.
    #[test]
    fn matches_fuse_across_the_whole_frame() {
        let mem = AddressCodedScreen;
        let mut mismatches = Vec::new();
        for t in 0..69_888u32 {
            let ours = floating_bus_byte(t, 14_338, 224, &mem);
            let theirs = fuse_unattached_port(t, &mem);
            if ours != theirs {
                mismatches.push((t, ours, theirs));
            }
        }
        if !mismatches.is_empty() {
            let total = mismatches.len();
            let sample: Vec<String> = mismatches
                .iter()
                .take(12)
                .map(|(t, o, f)| format!("t={t} ours={o:#04x} fuse={f:#04x}"))
                .collect();
            panic!(
                "floating-bus model disagrees with FUSE at {total} of 69888 T-states\n  {}",
                sample.join("\n  ")
            );
        }
    }
}

#[cfg(test)]
mod engine_paths {
    use super::*;
    use emu198x_bus_sinclair_zx_spectrum::MemoryBus;
    use emu198x_bus_sinclair_zx_spectrum::timing::SCREEN_WIDTH_HIRES;

    /// A flat 64K address space, so a test can place known bytes at the
    /// exact addresses the engine is expected to fetch.
    struct Ram(Vec<u8>);
    impl Ram {
        fn with(bytes: &[(u16, u8)]) -> Self {
            let mut ram = Ram(vec![0; 0x1_0000]);
            for &(addr, value) in bytes {
                ram.write(addr, value);
            }
            ram
        }
    }
    impl MemoryBus for Ram {
        fn read(&self, addr: u16) -> u8 {
            self.0[usize::from(addr)]
        }
        fn write(&mut self, addr: u16, value: u8) {
            self.0[usize::from(addr)] = value;
        }
        fn is_contended(&self, _addr: u16) -> bool {
            false
        }
    }

    /// A value the framebuffer starts filled with, so a test can tell
    /// which cells the engine wrote.
    const UNTOUCHED: u8 = 0x55;

    fn standard_fb() -> Vec<u8> {
        vec![UNTOUCHED; SCREEN_WIDTH * SCREEN_HEIGHT]
    }

    fn wide_fb() -> Vec<u8> {
        vec![UNTOUCHED; SCREEN_WIDTH_HIRES * SCREEN_HEIGHT]
    }

    /// `UlaConfig` constants are inlined at each use, so compare timing by
    /// value rather than by address.
    fn same_timing(a: &UlaConfig, b: &UlaConfig) -> bool {
        format!("{a:?}") == format!("{b:?}")
    }

    /// Tick until the engine's pixel counter reads `pixel`.
    fn tick_until(e: &mut UlaEngine, mem: &dyn MemoryBus, fb: &mut [u8], pixel: u16) {
        while e.pixel != pixel {
            e.tick_rendering(mem, fb, None);
        }
    }

    /// The table builder is a `const fn`, so the constant is computed at
    /// compile time. Running it here checks the stated rule directly:
    /// `C2 + C3` over a 16-pixel cycle is low for counter values 0..=3
    /// and high for 4..=15.
    #[test]
    fn delay_table_48k_is_c2_or_c3_of_the_fetch_counter() {
        let built = clkwait_from_counter_bits();
        assert_eq!(built, DELAY_TABLE_48K);
        for (pixel, &wait) in built.iter().enumerate() {
            assert_eq!(wait, pixel >= 4, "pixel {pixel}");
        }
    }

    /// The +2A table is the measured window its doc comment describes:
    /// eleven asserted half-cycles from pixel 4, free over 15 and 0-3.
    #[test]
    fn delay_table_plus2a_is_eleven_half_cycles_from_pixel_4() {
        let asserted: Vec<usize> = (0..16).filter(|&p| DELAY_TABLE_PLUS2A[p]).collect();
        assert_eq!(asserted, (4..15).collect::<Vec<_>>());
    }

    /// One serialisable field value, for a deserializer that needs no
    /// format crate.
    enum Field {
        U(u64),
        B(bool),
    }
    impl<'de> serde::de::IntoDeserializer<'de, serde::de::value::Error> for Field {
        type Deserializer = Self;
        fn into_deserializer(self) -> Self {
            self
        }
    }
    impl<'de> serde::Deserializer<'de> for Field {
        type Error = serde::de::value::Error;
        fn deserialize_any<V: serde::de::Visitor<'de>>(
            self,
            visitor: V,
        ) -> Result<V::Value, Self::Error> {
            match self {
                Field::U(n) => visitor.visit_u64(n),
                Field::B(b) => visitor.visit_bool(b),
            }
        }
        serde::forward_to_deserialize_any! {
            bool i8 i16 i32 i64 i128 u8 u16 u32 u64 u128 f32 f64 char str string
            bytes byte_buf option unit unit_struct newtype_struct seq tuple
            tuple_struct map struct enum identifier ignored_any
        }
    }

    /// A restored engine has no timing config of its own: the field is
    /// skipped, so it falls back to 48K timing until the machine calls
    /// `set_config`. Mutable pipeline fields retain their saved values.
    #[test]
    fn deserialized_engine_uses_48k_timing_until_set_config() {
        use Field::{B, U};
        // Every serialised field, in declaration order.
        let fields = vec![
            U(447),  // pixel
            U(10),   // scan
            U(2),    // border
            B(true), // beeper
            B(false),
            U(3), // flash_counter
            B(false),
            B(true), // cpu_clock
            B(true), // z80_clock_high
            B(false),
            U(0xFF), // bus_data
            B(true), // idle
            U(0),
            U(0),
            U(0),
            U(0),
            U(0xa5), // data_latch_pending
            U(0x47), // attr_latch_pending
            U(0),    // data_addr
            U(0),
            B(false), // video
            B(true),  // border_active
            U(2),     // border_aolatch
            B(true),  // vid_en
            B(false),
            B(false),
            B(false),
            B(false),
            B(false),
            U(2),                   // mcycle_fall
            U(0xc123),              // prev_fall_addr
            U(SCREEN_WIDTH as u64), // fb_width
            U(0),
            U(0),
            U(0),
            U(0),
            U(0x3c), // data_latch_pending2
        ];
        let de = serde::de::value::SeqDeserializer::<_, serde::de::value::Error>::new(
            fields.into_iter(),
        );
        let mut e: UlaEngine = serde::Deserialize::deserialize(de).expect("engine deserializes");

        assert!(same_timing(e.config, &CONFIG_48K));
        assert_eq!(
            (e.pixel, e.scan, e.border, e.flash_counter),
            (447, 10, 2, 3)
        );
        assert_eq!(
            (
                e.data_latch_pending,
                e.attr_latch_pending,
                e.data_latch_pending2
            ),
            (0xa5, 0x47, 0x3c)
        );
        assert_eq!(e.border_aolatch, 2);
        assert!(e.vid_en);
        assert_eq!((e.mcycle_fall, e.prev_fall_addr), (2, 0xc123));

        // A 48K line has 448 pixels, so pixel 447 wraps to the next line.
        let mem = Ram::with(&[]);
        let mut fb = standard_fb();
        let mut wrapped = e.clone();
        wrapped.tick_rendering(&mem, &mut fb, None);
        assert_eq!((wrapped.pixel, wrapped.scan), (0, 11));

        // Under 128K timing the same pixel is not the end of the line.
        e.set_config(&CONFIG_128K);
        assert!(same_timing(e.config, &CONFIG_128K));
        e.tick_rendering(&mem, &mut fb, None);
        assert_eq!((e.pixel, e.scan), (448, 10));
    }

    #[test]
    fn new_hires_differs_from_new_only_in_framebuffer_width() {
        let standard = UlaEngine::new(&CONFIG_TIMEX_PAL);
        let hires = UlaEngine::new_hires(&CONFIG_TIMEX_PAL);
        assert_eq!(standard.fb_width, SCREEN_WIDTH);
        assert_eq!(hires.fb_width, SCREEN_WIDTH_HIRES);
        assert!(same_timing(hires.config, &CONFIG_TIMEX_PAL));
        assert_eq!(hires.border, standard.border);
        assert_eq!(hires.scld_mode, 0);
        assert!(hires.cpu_clock && hires.border_active && !hires.video);
    }

    /// One whole 48K display line: checkpoint flags, fetch addresses,
    /// the two-stage pipeline's output and the border either side.
    #[test]
    fn a_48k_display_line_renders_its_first_two_cells_and_border() {
        let mem = Ram::with(&[
            (0x4000, 0xA0), // bitmap column 0: 1010_0000
            (0x5800, 0x0A), // paper 1, ink 2
            (0x4001, 0x0F), // bitmap column 1: 0000_1111
            (0x5801, 0x47), // bright, paper 0, ink 7
        ]);
        let mut fb = standard_fb();
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.write_fe(3);

        // Pixel 0 of a display line clears the border flag.
        e.tick_rendering(&mem, &mut fb, None);
        assert!(!e.border_active);
        assert!(!e.video, "the fetch window has not opened yet");

        // `fetch_start` opens the window and loads the line's addresses;
        // the first bitmap fetch happens on the same tick.
        tick_until(&mut e, &mem, &mut fb, 8);
        e.tick_rendering(&mem, &mut fb, None);
        assert!(e.video);
        assert!(!e.idle);
        assert_eq!(e.bus_data, 0xA0, "bitmap fetch at phase 8");
        assert_eq!(e.data_addr, 0x0001);
        assert_eq!(e.attr_addr, 0x1800);
        tick_until(&mut e, &mem, &mut fb, 11);
        assert_eq!(e.bus_data, 0x0A, "attribute fetch at phase 10");

        // Pixel 256 raises the border flag; fetching carries on to 264.
        tick_until(&mut e, &mem, &mut fb, 257);
        assert!(e.border_active);
        assert!(e.video);
        tick_until(&mut e, &mem, &mut fb, 265);
        assert!(!e.video);
        assert!(e.idle);
        assert_eq!(e.bus_data, 0xFF, "the bus floats high outside the fetch");
        assert_eq!(e.data_addr, 0x0020, "32 bitmap fetches per line");
        assert_eq!(e.attr_addr, 0x1820, "32 attribute fetches per line");

        tick_until(&mut e, &mem, &mut fb, 0);
        assert_eq!(e.scan, 1);

        let row = &fb[48 * SCREEN_WIDTH..49 * SCREEN_WIDTH];
        // Pixels 0-3 precede the first AOLatch trigger, so they show the
        // power-on colour; from pixel 4 the written border shows.
        assert_eq!(&row[36..40], &[7, 7, 7, 7]);
        assert_eq!(&row[40..48], &[3; 8]);
        // Screen starts at pixel 12 (x = 48).
        assert_eq!(&row[48..56], &[2, 1, 2, 1, 1, 1, 1, 1]);
        assert_eq!(&row[56..64], &[8, 8, 8, 8, 15, 15, 15, 15]);
        // Pixel 268 is right border; 315 is the last before HBlank.
        assert_eq!(row[304], 3);
        assert_eq!(row[351], 3);
        // Pixels 412..447 wrap to the left edge of the same row.
        assert_eq!(&row[0..36], &[3; 36]);
    }

    /// On the bottom-border lines the fetch window never opens.
    #[test]
    fn border_lines_never_open_the_fetch_window() {
        let mem = Ram::with(&[(0x4000, 0xA0)]);
        let mut fb = standard_fb();
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.scan = 192;
        e.border_active = false;
        e.tick_rendering(&mem, &mut fb, None);
        assert!(e.border_active, "pixel 0 sets the border flag at scan 192");
        tick_until(&mut e, &mem, &mut fb, 9);
        assert!(!e.video);
        assert!(e.border_active);
        assert_eq!(e.bus_data, 0xFF);
        assert_eq!(e.data_addr, 0);
    }

    /// The framebuffer row follows the scan: display and bottom border
    /// sit 48 rows down, VSync lines are not drawn, and the lines after
    /// VSync are the top border at row 0.
    #[test]
    fn framebuffer_rows_and_columns_follow_the_48k_geometry() {
        let mem = Ram::with(&[]);
        let draw = |scan: u16, pixel: u16| {
            let mut fb = standard_fb();
            let mut e = UlaEngine::new(&CONFIG_48K);
            e.scan = scan;
            e.pixel = pixel;
            e.border_aolatch = 5;
            e.border = 5;
            e.tick_rendering(&mem, &mut fb, None);
            fb.iter()
                .position(|&v| v != UNTOUCHED)
                .map(|i| (i % SCREEN_WIDTH, i / SCREEN_WIDTH))
        };
        assert_eq!(draw(200, 0), Some((36, 248)), "bottom border");
        assert_eq!(draw(247, 0), Some((36, 295)), "last row before VSync");
        assert_eq!(draw(248, 0), None, "VSync starts");
        assert_eq!(draw(263, 0), None, "VSync's last line");
        assert_eq!(draw(264, 0), Some((36, 0)), "top border starts");
        assert_eq!(draw(300, 0), Some((36, 36)));
        assert_eq!(draw(300, 315), Some((351, 36)), "last pixel before HBlank");
        assert_eq!(draw(300, 316), None, "HBlank");
        assert_eq!(draw(300, 411), None, "HBlank");
        assert_eq!(draw(300, 412), Some((0, 36)), "wrapped left border");
        assert_eq!(draw(300, 447), Some((35, 36)));
    }

    /// The shift register keeps clocking through screen pixels that land
    /// outside the framebuffer, so it stays in step with the raster.
    #[test]
    fn shifter_clocks_through_screen_pixels_outside_the_framebuffer() {
        static HIDDEN_FROM_14: UlaConfig = UlaConfig {
            fb_x_hblank_start: 14,
            ..CONFIG_48K
        };
        let mem = Ram::with(&[]);
        let mut fb = standard_fb();
        let mut e = UlaEngine::new(&HIDDEN_FROM_14);
        e.pixel = 12;
        e.data_latch_pending = 0xB4; // 1011_0100
        e.attr_latch_pending = 0x38; // paper 7, ink 0

        // Pixels 12 and 13 are drawn from the top two bits.
        e.tick_rendering(&mem, &mut fb, None);
        e.tick_rendering(&mem, &mut fb, None);
        let row = 48 * SCREEN_WIDTH;
        assert_eq!(&fb[row + 48..row + 50], &[0, 7]);

        // Pixels 14 and 15 are hidden, but still shift.
        e.tick_rendering(&mem, &mut fb, None);
        e.tick_rendering(&mem, &mut fb, None);
        assert_eq!(e.data_reg, 0xB4 << 4);
        assert_eq!(fb[row + 50], UNTOUCHED);

        // In hi-res mode the second register shifts with the first.
        e.scld_mode = 0x04;
        e.data_reg = 0x80;
        e.data_reg2 = 0xC0;
        e.tick_rendering(&mem, &mut fb, None);
        assert_eq!((e.data_reg, e.data_reg2), (0x00, 0x80));
    }

    /// Each SCLD mode reads its bitmap and attributes from its own area.
    /// Scan 9's first column has bitmap offset $0120 and attribute
    /// offset $1820.
    #[test]
    fn scld_modes_fetch_from_their_own_screen_areas() {
        let mem = Ram::with(&[
            (0x4120, 0x11),
            (0x5820, 0x22),
            (0x6120, 0x33),
            (0x7820, 0x44),
        ]);
        // Timex PAL fetches bitmap at pixel 4 and attribute at pixel 6.
        let fetch = |scld_mode: u8| {
            let mut fb = standard_fb();
            let mut e = UlaEngine::new(&CONFIG_TIMEX_PAL);
            e.scan = 9;
            e.pixel = 4;
            e.scld_mode = scld_mode;
            e.tick_rendering(&mem, &mut fb, None);
            let bitmap = (e.data_latch_pending, e.data_latch_pending2);
            tick_until(&mut e, &mem, &mut fb, 7);
            (bitmap, e.attr_latch_pending, e.bus_data)
        };
        assert_eq!(fetch(0x00), ((0x11, 0), 0x22, 0x22), "standard");
        assert_eq!(fetch(0x01), ((0x33, 0), 0x44, 0x44), "dual screen");
        assert_eq!(
            fetch(0x02),
            ((0x11, 0), 0x33, 0x33),
            "hi-colour: one attribute per pixel row, from $6000"
        );
        assert_eq!(
            fetch(0x04),
            ((0x11, 0x33), 0xFF, 0xFF),
            "hi-res: both screens' bitmaps, no attribute"
        );
    }

    /// Hi-res on a wide framebuffer draws two pixels per ULA clock: the
    /// even one from the $4000 screen and the odd one from $6000, both in
    /// the port $FF ink.
    #[test]
    fn hires_interleaves_both_screens_on_a_wide_framebuffer() {
        let mem = Ram::with(&[(0x4000, 0xF0), (0x6000, 0x3C)]);
        let mut fb = wide_fb();
        let mut e = UlaEngine::new_hires(&CONFIG_TIMEX_PAL);
        e.scld_mode = 0x04;
        e.scld_hires_ink = 5;
        tick_until(&mut e, &mem, &mut fb, 20);

        let row = 48 * SCREEN_WIDTH_HIRES;
        // Border pixels are doubled: pixel 0 covers x = 72 and 73.
        assert_eq!(&fb[row + 72..row + 74], &[7, 7]);
        // Screen pixel 12 starts at x = (12 + 36) * 2 = 96.
        let even = [5, 5, 5, 5, 0, 0, 0, 0];
        let odd = [0, 0, 5, 5, 5, 5, 0, 0];
        for i in 0..8 {
            assert_eq!(fb[row + 96 + 2 * i], even[i], "even pixel {i}");
            assert_eq!(fb[row + 97 + 2 * i], odd[i], "odd pixel {i}");
        }
    }

    /// Standard rendering on a wide framebuffer doubles every pixel.
    #[test]
    fn standard_pixels_are_doubled_on_a_wide_framebuffer() {
        let mem = Ram::with(&[(0x4000, 0xA0), (0x5800, 0x0A)]);
        let mut fb = wide_fb();
        let mut e = UlaEngine::new_hires(&CONFIG_48K);
        tick_until(&mut e, &mem, &mut fb, 20);

        let row = 48 * SCREEN_WIDTH_HIRES;
        assert_eq!(
            &fb[row + 96..row + 112],
            &[2, 2, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        );
    }

    /// A framebuffer shorter than the geometry implies is never written
    /// past its end: border and screen pixels that fall beyond it are
    /// dropped instead of panicking.
    #[test]
    fn a_short_framebuffer_is_never_written_past_its_end() {
        let mem = Ram::with(&[(0x4000, 0xFF), (0x5800, 0x07)]);
        for mut e in [
            UlaEngine::new(&CONFIG_48K),
            UlaEngine::new_hires(&CONFIG_48K),
        ] {
            let mut fb = vec![UNTOUCHED; 16];
            for _ in 0..448 {
                e.tick_rendering(&mem, &mut fb, None);
            }
            assert_eq!((e.scan, e.pixel), (1, 0));
            assert_eq!(fb, [UNTOUCHED; 16], "row 48 lies beyond the buffer");
        }
    }

    /// With the flash phase active, a FLASH attribute swaps ink and paper.
    #[test]
    fn flash_swaps_ink_and_paper_when_active() {
        let mem = Ram::with(&[(0x4000, 0xF0), (0x5800, 0x80 | 0x0A)]);
        let render = |flash_active: bool| {
            let mut fb = standard_fb();
            let mut e = UlaEngine::new(&CONFIG_48K);
            e.flash_active = flash_active;
            tick_until(&mut e, &mem, &mut fb, 20);
            fb[48 * SCREEN_WIDTH + 48..48 * SCREEN_WIDTH + 56].to_vec()
        };
        assert_eq!(render(false), [2, 2, 2, 2, 1, 1, 1, 1]);
        assert_eq!(render(true), [1, 1, 1, 1, 2, 2, 2, 2]);
    }

    /// Run from pixel 0 of the interrupt line and report the pixel
    /// counter after the tick that raised INT, and how many ticks it
    /// stayed raised.
    fn interrupt_pulse(config: &'static UlaConfig) -> (u16, usize) {
        let mem = Ram::with(&[]);
        let mut fb = standard_fb();
        let mut e = UlaEngine::new(config);
        e.scan = config.int_scan;
        let mut first = None;
        let mut ticks = 0;
        for _ in 0..200 {
            e.tick_rendering(&mem, &mut fb, None);
            if e.int_active {
                first.get_or_insert(e.pixel);
                ticks += 1;
            }
        }
        assert!(!e.int_active, "INT has fallen again");
        (first.expect("INT was raised"), ticks)
    }

    /// Configurations with `fetch_start` 8 evaluate INT before the counter
    /// advances, the others after. Pulse lengths are two ULA pixels per
    /// T-state: 32 T-states for 48K, 36 for the 128K and grey +2, 32 for
    /// the +2A.
    #[test]
    fn interrupt_pulses_match_each_configuration() {
        assert_eq!(interrupt_pulse(&CONFIG_48K), (2, 64));
        assert_eq!(interrupt_pulse(&CONFIG_128K), (6, 72));
        assert_eq!(interrupt_pulse(&CONFIG_PLUS2), (4, 72));
        assert_eq!(interrupt_pulse(&CONFIG_PLUS2A), (1, 64));
        assert_eq!(interrupt_pulse(&CONFIG_PENTAGON), (1, 64));
        assert_eq!(interrupt_pulse(&CONFIG_TS2068), (1, 64));
    }

    #[test]
    fn interrupt_is_not_raised_on_other_lines() {
        let mem = Ram::with(&[]);
        let mut fb = standard_fb();
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.scan = 247;
        for _ in 0..448 {
            e.tick_rendering(&mem, &mut fb, None);
            assert!(!e.int_active);
        }
        assert_eq!(e.scan, 248);
    }

    #[test]
    fn the_counter_wraps_at_the_end_of_each_line_and_frame() {
        let mem = Ram::with(&[]);
        let mut fb = standard_fb();
        let after = |config: &'static UlaConfig, scan: u16, pixel: u16| {
            let mut fb2 = standard_fb();
            let mut e = UlaEngine::new(config);
            e.scan = scan;
            e.pixel = pixel;
            e.tick_rendering(&mem, &mut fb2, None);
            (e.scan, e.pixel)
        };
        assert_eq!(after(&CONFIG_48K, 311, 447), (0, 0));
        assert_eq!(after(&CONFIG_128K, 310, 455), (0, 0));
        assert_eq!(after(&CONFIG_128K, 311 - 1, 447), (310, 448));
        assert_eq!(after(&CONFIG_PENTAGON, 319, 447), (0, 0));
        assert_eq!(after(&CONFIG_TS2068, 261, 447), (0, 0));
        // A plain mid-frame tick only advances the pixel.
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.tick_rendering(&mem, &mut fb, None);
        assert_eq!((e.scan, e.pixel), (0, 1));
    }

    #[test]
    fn end_frame_toggles_flash_every_16_frames_and_rewinds_the_raster() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.pixel = 100;
        e.scan = 200;
        e.end_frame();
        assert_eq!((e.pixel, e.scan), (0, 0));
        assert_eq!(e.flash_counter, 1);
        for _ in 1..15 {
            e.end_frame();
        }
        assert_eq!(e.flash_counter, 15);
        assert!(!e.flash_active);
        e.end_frame();
        assert_eq!(e.flash_counter, 16);
        assert!(e.flash_active);
        for _ in 16..31 {
            e.end_frame();
        }
        assert!(e.flash_active, "still inverted at frame 31");
        e.end_frame();
        assert_eq!(e.flash_counter, 0, "the counter wraps at 32");
        assert!(!e.flash_active);
    }

    #[test]
    fn read_fe_ands_the_selected_keyboard_rows() {
        let e = UlaEngine::new(&CONFIG_48K);
        let mut keys = [0xFF; 8];
        assert_eq!(e.read_fe(0x00FE, &keys), 0xFF, "no key pressed");

        keys[0] = 0xFE; // row 0, bit 0
        keys[3] = 0xFD; // row 3, bit 1
        assert_eq!(e.read_fe(0xFEFE, &keys), 0xFE, "row 0 selected");
        assert_eq!(e.read_fe(0xF7FE, &keys), 0xFD, "row 3 selected");
        assert_eq!(e.read_fe(0x7FFE, &keys), 0xFF, "row 7 only");
        assert_eq!(e.read_fe(0x00FE, &keys), 0xFC, "all rows");

        // Only bits 0-4 come from the keyboard; bits 5-7 always read high.
        keys[0] = 0x1E;
        assert_eq!(e.read_fe(0xFEFE, &keys), 0xFE);
    }

    #[test]
    fn track_z80_clock_latches_strobes_on_the_rising_edge_only() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        assert!(e.z80_clock_high);
        assert!(!e.gate_arms_this_halfcycle());

        // Falling edge: history moves, the T2/T3 latches do not.
        e.track_z80_clock(false, true, true);
        assert!(!e.z80_clock_high);
        assert!(e.gate_arms_this_halfcycle());
        assert!(e.z80_mreq_prev);
        assert!(!e.mreq_t23 && !e.ioreq_tw3);

        // Rising edge: both latches take the pins.
        e.track_z80_clock(true, true, true);
        assert!(e.z80_clock_high);
        assert!(e.mreq_t23 && e.ioreq_tw3);
        assert!(e.z80_iorq_prev && !e.z80_iorq_prev2);

        // Next falling edge: latches hold, IORQ history shifts.
        e.track_z80_clock(false, false, false);
        assert!(e.mreq_t23 && e.ioreq_tw3);
        assert!(!e.z80_iorq_prev && e.z80_iorq_prev2);
        assert!(!e.z80_mreq_prev);

        // A stalled CPU freezes everything.
        e.cpu_clock = false;
        e.track_z80_clock(true, true, false);
        assert!(!e.z80_clock_high);
        assert!(!e.z80_iorq_prev && e.z80_iorq_prev2 && !e.z80_mreq_prev);

        // Released, the next rising edge drops the latches.
        e.cpu_clock = true;
        e.track_z80_clock(false, false, false);
        assert!(e.z80_clock_high);
        assert!(!e.mreq_t23 && !e.ioreq_tw3);
    }

    /// One half-cycle as a machine drives it: count first, then advance
    /// the clock. Returns the count the gate would read.
    fn half_cycle(e: &mut UlaEngine, addr: u16, mreq: bool, iorq: bool) -> u8 {
        e.track_mcycle_fall(addr, mreq, iorq);
        let count = e.mcycle_fall;
        e.track_z80_clock(iorq, mreq, false);
        count
    }

    #[test]
    fn mcycle_fall_counts_strobe_free_falling_edges_of_one_address() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        // The first half-cycle is a rising one: nothing is counted.
        assert_eq!(half_cycle(&mut e, 0x8000, false, false), 0);
        assert_eq!(e.prev_fall_addr, 0);

        // Memory cycle: one strobe-free falling edge, then /MREQ.
        assert_eq!(half_cycle(&mut e, 0x4000, false, false), 1);
        assert_eq!(e.prev_fall_addr, 0x4000);
        half_cycle(&mut e, 0x4000, true, false);
        assert_eq!(half_cycle(&mut e, 0x4000, true, false), 0);

        // I/O cycle on a new address: two strobe-free falls, then /IORQ.
        half_cycle(&mut e, 0x40FE, false, false);
        assert_eq!(half_cycle(&mut e, 0x40FE, false, false), 1);
        half_cycle(&mut e, 0x40FE, false, false);
        assert_eq!(half_cycle(&mut e, 0x40FE, false, false), 2);
        half_cycle(&mut e, 0x40FE, false, true);
        assert_eq!(half_cycle(&mut e, 0x40FE, false, true), 0);

        // An Internal M-cycle on the stale address never starts a run.
        half_cycle(&mut e, 0x40FE, false, false);
        assert_eq!(half_cycle(&mut e, 0x40FE, false, false), 0);
        half_cycle(&mut e, 0x40FE, false, false);
        assert_eq!(half_cycle(&mut e, 0x40FE, false, false), 0);
    }

    #[test]
    fn mcycle_fall_freezes_while_the_cpu_is_stalled() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.z80_clock_high = false;
        e.track_mcycle_fall(0x4000, false, false);
        assert_eq!(e.mcycle_fall, 1);

        e.cpu_clock = false;
        e.track_mcycle_fall(0x4000, false, false);
        e.track_mcycle_fall(0x5000, false, false);
        assert_eq!(e.mcycle_fall, 1);
        assert_eq!(e.prev_fall_addr, 0x4000);

        // On a rising half-cycle the count is left alone too.
        e.cpu_clock = true;
        e.z80_clock_high = true;
        e.track_mcycle_fall(0x5000, true, false);
        assert_eq!(e.mcycle_fall, 1);
    }

    #[test]
    fn mcycle_fall_saturates_rather_than_wrapping() {
        let mut e = UlaEngine::new(&CONFIG_48K);
        e.z80_clock_high = false;
        e.prev_fall_addr = 0x4000;
        e.mcycle_fall = u8::MAX;
        e.track_mcycle_fall(0x4000, false, false);
        assert_eq!(e.mcycle_fall, u8::MAX);
    }
}
