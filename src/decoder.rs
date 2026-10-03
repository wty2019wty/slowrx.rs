//! [`SstvDecoder`] — public state machine driving the decode pipeline.
//!
//! Two-pass per-image flow: VIS detection (via [`crate::vis`]) →
//! buffer audio in `Decoding` state until ~one image's worth → run
//! `crate::sync::find_sync` once to recover the slant-corrected
//! rate + line-zero `Skip` → burst-decode every row (via
//! `crate::demod::decode_one_channel_into` in the per-mode glue
//! for PD/Robot/Scottie/Martin) → emit `LineDecoded` events and a
//! final `ImageComplete`. Multi-image streaming is supported in one
//! `process()` call (issue #90).
//!
//! Translated in spirit from slowrx's `slowrx.c::Listen()` loop +
//! `vis.c::GetVIS()` + `video.c::GetVideo()`. ISC License — see
//! `NOTICE.md`. Inline `// slowrx <file>.c:NNN` references throughout
//! point at the gitignored local reference clone under `original/slowrx/`
//! (see `clone-slowrx.sh`); verified at audit #94 (2026-05-15).

use crate::error::Result;
use crate::image::SstvImage;
use crate::modespec::SstvMode;
use crate::resample::Resampler;
use crate::sync::{find_sync, periodic_train_start, SyncTracker, SYNC_PROBE_STRIDE};

/// One observable event emitted by [`SstvDecoder::process`].
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum SstvEvent {
    /// VIS header parsed and a known mode dispatched.
    VisDetected {
        /// Mode identified by the VIS bits.
        mode: SstvMode,
        /// Working-rate (11025 Hz) sample offset where the VIS stop bit ended.
        /// Useful for callers that want to align audio captures with decoder events.
        sample_offset: u64,
        /// Radio mistuning offset in Hz: `observed_leader_hz - 1900`. The
        /// decoder applies this offset internally to per-pixel demod so the
        /// downstream pixel band shifts with the radio's tuning. Surfaced
        /// here purely for caller diagnostics; consumers do not need to do
        /// anything with it. Translated from slowrx's `CurrentPic.HedrShift`
        /// (`vis.c` line 106 → `video.c` line 406).
        hedr_shift_hz: f64,
    },
    /// A VIS header parsed and passed parity, but its 7-bit code maps to no
    /// SSTV mode this build can decode (reserved / undefined, or a mode not
    /// yet implemented). The decoder discards the burst and resumes scanning
    /// for the next VIS — equivalent to slowrx's `printf("Unknown VIS %d")`
    /// plus its retry-`GetVIS()` loop.
    UnknownVis {
        /// The 7-bit VIS code that did not resolve.
        code: u8,
        /// Radio mistuning offset in Hz: `observed_leader_hz - 1900` (the
        /// same quantity as [`SstvEvent::VisDetected`]'s `hedr_shift_hz`).
        /// Surfaced for diagnostics; the burst is dropped, so it does not
        /// feed any decode.
        hedr_shift_hz: f64,
        /// Working-rate (11025 Hz) sample offset where the VIS stop bit ended.
        sample_offset: u64,
    },
    /// One scan line completed (callers may render incrementally).
    ///
    /// For PD and Robot 72: `pixels` is fully composed at emission time
    /// (own Y/U/V or Y(odd)/Cr/Cb/Y(even) for the row).
    ///
    /// For Robot 36 / Robot 24: `pixels` reflects the image buffer state
    /// at emission time, which has a transient cross-row dependency due
    /// to chroma alternation. Each radio line writes its own Cr-or-Cb
    /// AND duplicates that chroma to the NEXT image row. So row N is
    /// emitted with: own Y, own Cr-or-Cb (per row parity), and the
    /// OTHER chroma channel duplicated from the previous radio line.
    /// Row 0 is the exception — its `Cb` channel is zero-init at
    /// emission time (no row -1 to duplicate from), giving a transient
    /// color cast on the very top row. Faithful to slowrx C, which
    /// `calloc`'s its image buffer and never writes row 0's Cb.
    /// `ImageComplete` carries the final populated state for all rows
    /// `1..image_lines` and the same row-0 Cb-zero artifact.
    LineDecoded {
        /// Mode currently being decoded.
        mode: SstvMode,
        /// 0-based row index for this line.
        line_index: u32,
        /// `line_pixels` RGB triplets for this row, taken from the
        /// in-progress image at emission time.
        pixels: Vec<[u8; 3]>,
    },
    /// Image complete (`LineDecoded` for the final line was just emitted).
    /// `partial` is reserved for future mid-image VIS handling — V1 always
    /// emits `partial: false`. `reset()` discards in-flight images silently
    /// without emitting any event.
    ImageComplete {
        /// Final pixel buffer.
        image: SstvImage,
        /// Reserved for future mid-image VIS handling. V1 always sets this
        /// to `false`. See the deferred mid-image VIS TODO in
        /// [`SstvDecoder::process`] for details.
        partial: bool,
    },
}

/// Internal state of the decoder.
enum State {
    AwaitingVis,
    /// Boxed because [`DecodingState`] contains the working FFT plans +
    /// audio buffer and dwarfs the unit `AwaitingVis` variant; clippy
    /// warns about size disparity otherwise.
    Decoding(Box<DecodingState>),
}

/// Sub-phase of a forced-mode [`DecodingState`] (issue #113 follow-up).
///
/// Forced decoding has no VIS header to mark where an image begins, so it
/// cannot chop the stream into fixed windows: a window boundary that lands in
/// the middle of a transmission splits one image into two partial decodes, and
/// a noisy window can fabricate an image from nothing. Instead the forced path
/// first *searches* for a periodic sync train (a run of line-spaced 1200 Hz
/// pulses) and only then collects a full window anchored on that train's line
/// 0. `Searching` never emits; `Collecting` behaves like the VIS path.
///
/// The VIS path always starts in `Collecting` (the VIS stop bit already marks
/// line 0), so this phase is only consulted when [`SstvDecoder::forced_mode`]
/// is set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ForcedPhase {
    /// Scan for a periodic sync train; discard audio that can no longer begin
    /// a train so an arbitrary-length gap stays bounded in memory.
    Searching,
    /// A train was found and the window is anchored on its line 0; accumulate
    /// one full image plus margin, then decode.
    Collecting,
}

impl std::fmt::Debug for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::AwaitingVis => write!(f, "AwaitingVis"),
            Self::Decoding(d) => f
                .debug_struct("Decoding")
                .field("mode", &d.mode)
                .field("audio_samples", &d.audio.len())
                .field("has_sync_probes", &d.has_sync.len())
                .field("next_probe_sample", &d.next_probe_sample)
                .field("target_audio_samples", &d.target_audio_samples)
                .field("hedr_shift_hz", &d.hedr_shift_hz)
                .finish_non_exhaustive(),
        }
    }
}

/// Two-pass decoding state.
///
/// While `audio.len() < target_audio_samples`, the decoder accumulates
/// audio and probes the 1200 Hz sync band into `has_sync`. When the
/// buffer is full, [`find_sync`] runs once to recover the
/// slant-corrected rate + line-zero `Skip`; per-pair decode then runs in
/// a single fast burst, emitting [`SstvEvent::LineDecoded`] for every
/// row.
struct DecodingState {
    mode: SstvMode,
    spec: crate::modespec::ModeSpec,
    image: SstvImage,
    /// Working-rate audio captured from VIS-stop-bit forward.
    audio: Vec<f32>,
    /// Per-stride boolean track from [`SyncTracker::has_sync_at`]. One
    /// entry per [`SYNC_PROBE_STRIDE`] working-rate samples.
    has_sync: Vec<bool>,
    /// Next sample index in `audio` to probe. Always a multiple of
    /// [`SYNC_PROBE_STRIDE`].
    next_probe_sample: usize,
    /// Sync-band tracker. Constructed when `Decoding` is entered so the
    /// hedr-shift bin offsets match the detected mistuning.
    sync_tracker: SyncTracker,
    /// Radio mistuning offset in Hz extracted at VIS time. Plumbed to
    /// per-pixel demod so the pixel band shifts with radio tuning.
    hedr_shift_hz: f64,
    /// Total audio samples we must accumulate before running
    /// [`find_sync`] and per-pair decode. Computed at state-entry as
    /// `image_lines / 2 × line_seconds × FINDSYNC_AUDIO_HEADROOM × work_rate`.
    target_audio_samples: usize,
    /// Per-mode chroma planes side buffer.
    ///
    /// `Some([cr_plane, cb_plane])` for `SstvMode::Robot24` and
    /// `SstvMode::Robot36`. Each plane is `image_lines * line_pixels`
    /// bytes, populated as radio lines are decoded: each radio line N
    /// writes its own chroma + duplicates to the next row's chroma slot
    /// (slowrx `video.c:421-425`); RGB composition for row N reads the
    /// duplicated-from-N-1 chroma channel that the line N-1 decode
    /// wrote earlier.
    ///
    /// `None` for every other mode. PD composes RGB in-place per pair
    /// (see `mode_pd::decode_pd_line_pair`); Robot 72 and Scottie 1/2/DX
    /// also compose RGB in-place per radio line (see
    /// `mode_robot::decode_r72_line` and `mode_scottie::decode_line`).
    /// `SstvMode` is `#[non_exhaustive]`; any future mode that does
    /// need cross-radio-line chroma state will need to extend the
    /// constructor's match in `process` to opt in.
    chroma_planes: Option<[Vec<u8>; 2]>,
    /// Forced-mode acquisition phase. Always `Collecting` for the VIS path.
    /// See [`ForcedPhase`].
    phase: ForcedPhase,
}

/// Headroom factor on the buffered audio length before [`find_sync`]
/// runs. 1.00 = exactly the nominal image length. The Hough transform
/// re-anchors the rate against whatever sync pulses are present, so
/// trailing audio beyond the last line is not strictly required. We
/// keep this knob in case future modes (Scottie pre-line skip) want to
/// pad the buffer to absorb additional offset.
const FINDSYNC_AUDIO_HEADROOM: f64 = 1.00;

/// Extra trailing audio (seconds) buffered beyond one image in *forced mode*,
/// so the sync locator and the last line's pixel windows always have audio
/// ahead of them. Applies only when a mode was forced via
/// [`SstvDecoder::with_mode`] / [`SstvDecoder::set_forced_mode`].
const FORCED_MODE_TRAILING_MARGIN_SECONDS: f64 = 1.0;

/// Minimum number of line-spaced sync pulses required before forced-mode
/// decoding accepts a window as a real transmission (issue #113 follow-up).
///
/// A real image emits one sync pulse per radio line (>= 24 for every supported
/// mode), so requiring four in a row on the line clock keeps the existing
/// one-pulse Hough peak from turning noise, hum, or a lone tone into a
/// fabricated image. See [`crate::sync::periodic_train_start`].
const FORCED_MIN_SYNC_PULSES: usize = 4;

/// Number of radio lines on the wire in one image of `spec`. PD packs two
/// image rows per line; Robot and Scottie/Martin pack one.
fn radio_frames_per_image(spec: crate::modespec::ModeSpec) -> u32 {
    match spec.channel_layout {
        crate::modespec::ChannelLayout::PdYcbcr => spec.image_lines / 2,
        crate::modespec::ChannelLayout::RobotYuv
        | crate::modespec::ChannelLayout::RgbSequential => spec.image_lines,
    }
}

/// Nominal airtime of one image, in seconds (un-rounded).
fn nominal_image_seconds(spec: crate::modespec::ModeSpec) -> f64 {
    f64::from(radio_frames_per_image(spec)) * spec.line_seconds
}

/// How many `ModeSpec` line-times (`spec.line_seconds` — the per-radio-line
/// duration; for PD, where a radio frame carries two image rows, that's
/// twice as many *image* scan lines) of the *just-decoded* image audio to
/// keep when re-arming the VIS detector after `ImageComplete` (issue #90 D4).
/// A back-to-back transmission's VIS leader starts right after the image's
/// last line; carrying back this much audio absorbs a fast transmitter clock
/// (4 line-times ≈ 1.5–2 % of any mode's airtime — far more than any real
/// clock error, <0.1 %) so the leader is always inside the carry-forward
/// window. For a single transmission this is just the image's last few
/// lines plus trailing silence; the fresh detector finds nothing and waits
/// for more audio.
const MULTI_IMAGE_CARRYBACK_LINES: u32 = 4;

/// `|c| crate::modespec::lookup(c).is_some()` as an `fn` pointer — the
/// "is this VIS code one we can decode?" predicate handed to every
/// [`crate::vis::VisDetector`] (issue #89 A3). The closure captures nothing,
/// so it coerces to `fn(u8) -> bool` in const context.
const IS_KNOWN_VIS: fn(u8) -> bool = |c| crate::modespec::lookup(c).is_some();

/// Streaming SSTV decoder. Push audio buffers in via
/// [`Self::process`]; consume the returned events.
pub struct SstvDecoder {
    resampler: Resampler,
    vis: crate::vis::VisDetector,
    channel_demod: crate::demod::ChannelDemod,
    /// SNR estimator. Owns its own FFT plan (separate from `channel_demod`)
    /// so the per-pixel demod's scratch buffer is never aliased. SNR
    /// is re-estimated periodically inside
    /// [`crate::mode_pd::decode_pd_line_pair`] (every
    /// [`crate::demod::SNR_REESTIMATE_STRIDE`] samples).
    snr_est: crate::snr::SnrEstimator,
    /// Scratch buffers for `find_sync` (`sync_img` / `lines` / `x_acc`).
    /// Hoisted here so they're reused across decode passes instead of
    /// being allocated fresh per call. (Audit #93 D6.)
    find_sync_scratch: crate::sync::FindSyncScratch,
    state: State,
    samples_processed: u64,
    /// Cumulative working-rate samples emitted by the resampler.
    /// Used as the unit for `SstvEvent::VisDetected.sample_offset` so
    /// that value is consistent regardless of caller's input rate.
    ///
    /// **Informational only** — this counter counts samples the resampler
    /// has produced and does NOT get decremented when
    /// [`crate::vis::VisDetector::take_residual_buffer`] transfers post-stop-bit
    /// audio back to the decoder's `Decoding` state. Those residual samples
    /// were already counted here when the resampler emitted them; the
    /// residual transfer is a borrow, not a retraction. Consequently the
    /// counter may be slightly ahead of what the image decoder has consumed.
    ///
    /// The counter is used to anchor the VIS detector each time a fresh one
    /// is constructed (initial, post-image, post-unknown-VIS). Note: for the
    /// *first* detection on a freshly-built decoder, `DetectedVis::end_sample`
    /// (→ `SstvEvent::VisDetected.sample_offset` / `UnknownVis.sample_offset`)
    /// is an absolute working-rate index from sample 0. After a *restart*
    /// (post-image — see `restart_vis_detection` — or post-unknown-VIS) the
    /// fresh detector counts hops from 0, so a later detection's `sample_offset`
    /// is relative to where the carry-forward audio began, not absolute. That
    /// is acceptable — `sample_offset` is informational only — and tracked in
    /// issue #99 (fixing it needs a `VisDetector` API change). The counter
    /// here does not gate any decode logic.
    ///
    /// If mid-image VIS detection is ever re-activated (see the TODO in
    /// `process`), and a single `SstvDecoder` is reused across detections,
    /// the slight inflation is harmless: each new detection uses the then-
    /// current resampler-output count as its anchor, and the residual buffer
    /// is handed to a fresh `VisDetector::new()`.
    ///
    /// Closes #29 and #34 (both are the same observation from different angles).
    working_samples_emitted: u64,
    /// When `Some`, VIS detection is bypassed: every full image window is
    /// decoded as this mode (issue #113). Set at construction via
    /// [`SstvDecoder::with_mode`] or at runtime via
    /// [`SstvDecoder::set_forced_mode`]. `None` restores automatic VIS
    /// detection.
    forced_mode: Option<SstvMode>,
}

impl SstvDecoder {
    /// Construct a decoder consuming audio at `input_sample_rate_hz`.
    ///
    /// # Errors
    /// Returns [`crate::Error::InvalidSampleRate`] if the rate is 0 or
    /// > [`crate::resample::MAX_INPUT_SAMPLE_RATE_HZ`].
    pub fn new(input_sample_rate_hz: u32) -> Result<Self> {
        Ok(Self {
            resampler: Resampler::new(input_sample_rate_hz)?,
            vis: crate::vis::VisDetector::new(IS_KNOWN_VIS),
            channel_demod: crate::demod::ChannelDemod::new(),
            snr_est: crate::snr::SnrEstimator::new(),
            find_sync_scratch: crate::sync::FindSyncScratch::new(),
            state: State::AwaitingVis,
            samples_processed: 0,
            working_samples_emitted: 0,
            forced_mode: None,
        })
    }

    /// Construct a decoder that decodes every image as `mode`, bypassing
    /// automatic VIS header detection (issue #113).
    ///
    /// Use this when the VIS header is missing, damaged, or misdetected, or
    /// when batch-processing recordings of a known mode.
    ///
    /// Decoding does not begin blindly at the start of the audio: forced mode
    /// first *acquires* a transmission by scanning for a run of several
    /// line-spaced 1200 Hz sync pulses, then anchors the decode window on
    /// line 0 and decodes one full image. Audio
    /// before the train (lead-in silence, a stray VIS header, an inter-image
    /// gap) is skipped, and audio that can no longer begin a train is discarded
    /// so arbitrarily long gaps stay bounded in memory. This means a recording
    /// holding several transmissions separated by silence yields one aligned
    /// image per transmission, and noise or a lone tone does not fabricate an
    /// image.
    ///
    /// Because no VIS is parsed, no [`SstvEvent::VisDetected`] event is emitted
    /// and the radio-mistuning offset (`hedr_shift_hz`) is treated as zero.
    /// A transmission that starts before the recording does (the capture
    /// catches only its tail) is decoded from the first acquired line.
    ///
    /// # Errors
    /// Returns [`crate::Error::InvalidSampleRate`] if the rate is 0 or
    /// > [`crate::resample::MAX_INPUT_SAMPLE_RATE_HZ`].
    pub fn with_mode(input_sample_rate_hz: u32, mode: SstvMode) -> Result<Self> {
        let mut decoder = Self::new(input_sample_rate_hz)?;
        decoder.forced_mode = Some(mode);
        Ok(decoder)
    }

    /// Set or clear the forced-decoding mode.
    ///
    /// Passing `Some(mode)` bypasses VIS detection and decodes subsequent
    /// images as `mode`; `None` restores automatic VIS detection. Any
    /// in-flight image is discarded and the change takes effect on the next
    /// [`Self::process`] call. Sample counters and the resampler state are
    /// preserved (use [`Self::reset`] to clear those too).
    pub fn set_forced_mode(&mut self, mode: Option<SstvMode>) {
        self.forced_mode = mode;
        // Discard any in-flight image and restart detection with a fresh
        // detector (honors the `#40` re-anchor contract).
        self.state = State::AwaitingVis;
        self.vis = crate::vis::VisDetector::new(IS_KNOWN_VIS);
    }

    /// The mode forced for decoding, if any. `None` means automatic VIS
    /// detection.
    #[must_use]
    pub fn forced_mode(&self) -> Option<SstvMode> {
        self.forced_mode
    }

    /// Process a chunk of mono `f32` audio samples in caller's rate.
    ///
    /// Returns events produced during this call's processing window.
    // `too_many_lines`: `process` is the decoder's state-machine loop; splitting
    // it (e.g. extracting `DecodingState::new`) is tracked in the code-review
    // audit (B14, epic #97).
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::too_many_lines
    )]
    pub fn process(&mut self, audio: &[f32]) -> Vec<SstvEvent> {
        let working = self.resampler.process(audio);
        self.samples_processed = self.samples_processed.saturating_add(audio.len() as u64);
        self.working_samples_emitted = self
            .working_samples_emitted
            .saturating_add(working.len() as u64);

        let mut out = Vec::new();
        let mut remaining: &[f32] = working.as_slice();
        loop {
            match &mut self.state {
                State::AwaitingVis => {
                    // Forced mode (issue #113) bypasses VIS detection
                    // entirely. It begins in the `Searching` phase: no image
                    // window is committed until a periodic sync train is
                    // found, so gaps between transmissions (and lead-in
                    // silence/headers) neither split an image across window
                    // boundaries nor fabricate one from noise. Mistuning is
                    // unknown without a VIS, so it is treated as zero.
                    if let Some(mode) = self.forced_mode {
                        let spec = crate::modespec::for_mode(mode);
                        self.state =
                            Self::start_decoding(spec, 0.0, Vec::new(), 0, ForcedPhase::Searching);
                        continue;
                    }
                    self.vis.process(remaining, self.working_samples_emitted);
                    remaining = &[];
                    if let Some(detected) = self.vis.take_detected() {
                        if let Some(spec) = crate::modespec::lookup(detected.code) {
                            out.push(SstvEvent::VisDetected {
                                mode: spec.mode,
                                sample_offset: detected.end_sample,
                                hedr_shift_hz: detected.hedr_shift_hz,
                            });
                            // Recover any post-stop-bit audio that the VIS
                            // detector buffered but did not consume — it is
                            // the leading edge of the image data.
                            let residual = self.vis.take_residual_buffer();
                            let work_rate = f64::from(crate::resample::WORKING_SAMPLE_RATE_HZ);
                            let nominal_samples =
                                (nominal_image_seconds(spec) * work_rate) as usize;
                            let target =
                                ((nominal_samples as f64) * FINDSYNC_AUDIO_HEADROOM) as usize;
                            self.state = Self::start_decoding(
                                spec,
                                detected.hedr_shift_hz,
                                residual,
                                target,
                                ForcedPhase::Collecting,
                            );
                            continue; // re-enter loop to process leftover audio
                        }
                        // Unknown VIS code: surface it so stream-monitoring
                        // callers know a burst arrived, then reseed the
                        // detector (the `#40` re-anchor contract) on the
                        // post-stop-bit residue and re-enter the loop — a
                        // back-to-back VIS in the residue then surfaces in
                        // this same `process` call. Mirrors the known-code
                        // branch's `continue`.
                        out.push(SstvEvent::UnknownVis {
                            code: detected.code,
                            hedr_shift_hz: detected.hedr_shift_hz,
                            sample_offset: detected.end_sample,
                        });
                        let residual = self.vis.take_residual_buffer();
                        Self::restart_vis_detection(
                            &mut self.vis,
                            self.working_samples_emitted,
                            &residual,
                        );
                        continue;
                    }
                    break;
                }
                State::Decoding(d) => {
                    let forced = self.forced_mode.is_some();
                    // TODO(future): mid-image VIS detection. When a new VIS
                    // burst arrives during decoding the spec calls for flushing
                    // the in-flight image as `partial: true` and restarting.
                    // The straightforward approach — running `self.vis` against
                    // `audio` each call — fails because the decoding buffer is
                    // not aligned to 30 ms window boundaries: the residual from
                    // the previous VIS detection starts at an arbitrary sample
                    // offset, so the first classifier window is a mix of silence
                    // and leader tone and does not reliably pass the 5× dominance
                    // threshold. A correct implementation would re-align the VIS
                    // window scan to the next 30 ms boundary, or run a separate
                    // correlator tuned to the 1900 Hz leader. Deferred to PR-3.

                    d.audio.extend_from_slice(remaining);

                    // Probe sync-band for every newly available stride
                    // window. The probe needs SYNC_FFT_WINDOW_SAMPLES/2
                    // trailing samples beyond the center; rather than
                    // depend on that constant, we conservatively wait
                    // until the audio extends `SYNC_PROBE_STRIDE * 2`
                    // beyond the next probe center.
                    while d.next_probe_sample + SYNC_PROBE_STRIDE * 2 <= d.audio.len() {
                        let center = d.next_probe_sample + SYNC_PROBE_STRIDE / 2;
                        let has = d.sync_tracker.has_sync_at(&d.audio, center);
                        d.has_sync.push(has);
                        d.next_probe_sample += SYNC_PROBE_STRIDE;
                    }

                    // Forced-mode acquisition (issue #113 follow-up). Before
                    // committing to a decode window, require a periodic sync
                    // train so a fixed window boundary cannot split one image
                    // in two (or fabricate an image from noise). Once a train
                    // is found, trim the buffer so it begins on line 0's *start*
                    // and switch to collecting a full image.
                    if forced && d.phase == ForcedPhase::Searching {
                        let work_rate = f64::from(crate::resample::WORKING_SAMPLE_RATE_HZ);
                        // Probes to back up from a pulse's leading edge to
                        // line 0's start (mid-line for Scottie), plus one probe
                        // of leading margin so the first pixel window has
                        // context. Whole probes keep `has_sync` aligned with
                        // `audio`.
                        let lead_probes = ((d.spec.sync_lead_offset_seconds() * work_rate)
                            / (SYNC_PROBE_STRIDE as f64))
                            .ceil() as usize;
                        if let Some(anchor_probe) = periodic_train_start(
                            &d.has_sync,
                            work_rate,
                            d.spec,
                            FORCED_MIN_SYNC_PULSES,
                        ) {
                            // Anchor on line 0's start: back up from the first
                            // pulse past the sync's within-line position, drop
                            // the leading silence/header, then size the window
                            // to one image plus a small trailing margin.
                            let drain_probes =
                                anchor_probe.saturating_sub(lead_probes).saturating_sub(1);
                            let anchor = drain_probes * SYNC_PROBE_STRIDE;
                            d.audio.drain(0..anchor);
                            d.has_sync.drain(0..drain_probes);
                            d.next_probe_sample = d.next_probe_sample.saturating_sub(anchor);
                            let nominal_samples =
                                (nominal_image_seconds(d.spec) * work_rate) as usize;
                            let margin_samples =
                                (FORCED_MODE_TRAILING_MARGIN_SECONDS * work_rate) as usize;
                            d.target_audio_samples = nominal_samples + margin_samples;
                            d.phase = ForcedPhase::Collecting;
                        } else {
                            // No train yet. Drop the prefix that can no longer
                            // begin one (a start at probe `i` is decided once
                            // probes through `i + (min_pulses-1)*period + tol`
                            // exist and it did not qualify), keeping the
                            // retained tail long enough to catch a train that
                            // straddles the drop boundary. `lead_probes` extra
                            // is retained so a future train's pre-sync line-0
                            // content (Scottie) is not discarded. This bounds
                            // memory over arbitrarily long gaps.
                            let period = (d.spec.line_seconds * work_rate
                                / (SYNC_PROBE_STRIDE as f64))
                                .round() as usize;
                            if period > 0 {
                                let decided_after = (FORCED_MIN_SYNC_PULSES - 1) * period + 2;
                                let drop_probes = d
                                    .has_sync
                                    .len()
                                    .saturating_sub(decided_after)
                                    .saturating_sub(lead_probes);
                                if drop_probes > 0 {
                                    let drop_samples = drop_probes * SYNC_PROBE_STRIDE;
                                    d.audio.drain(0..drop_samples);
                                    d.has_sync.drain(0..drop_probes);
                                    d.next_probe_sample =
                                        d.next_probe_sample.saturating_sub(drop_samples);
                                }
                            }
                            break; // need more audio to find a train
                        }
                    }

                    if d.audio.len() < d.target_audio_samples {
                        break;
                    }

                    // Buffer is full → run FindSync once, then decode every
                    // line. For VIS mode we capture the carryback audio before
                    // moving `d` into `run_findsync_and_decode` (which consumes
                    // it by value so `d.image` can move directly into the
                    // `ImageComplete` event without a fresh black-image
                    // realloc — Audit #93 D6.2). Forced mode instead re-seeds
                    // on the exact audio after the decoded image, returned by
                    // `run_findsync_and_decode`.
                    let carry_audio: Vec<f32> = if forced {
                        Vec::new()
                    } else {
                        let work_rate = f64::from(crate::resample::WORKING_SAMPLE_RATE_HZ);
                        let carryback = (f64::from(MULTI_IMAGE_CARRYBACK_LINES)
                            * d.spec.line_seconds
                            * work_rate) as usize;
                        let carry_from = d.target_audio_samples.saturating_sub(carryback);
                        d.audio[carry_from..].to_vec()
                    };
                    // Now extract the Box<DecodingState> by value. The
                    // `mem::replace` swap leaves `self.state` as `AwaitingVis`
                    // so the next loop iteration re-enters the AwaitingVis arm
                    // (no explicit assignment needed below).
                    let State::Decoding(d_box) =
                        std::mem::replace(&mut self.state, State::AwaitingVis)
                    else {
                        unreachable!("outer match arm is Decoding");
                    };
                    let forced_tail = Self::run_findsync_and_decode(
                        *d_box,
                        &mut self.channel_demod,
                        &mut self.snr_est,
                        &mut self.find_sync_scratch,
                        &mut out,
                        forced,
                    );

                    if forced {
                        // Resume searching for the next transmission. The tail
                        // after this image (empty if the caller feeds more in a
                        // later `process` call) is re-seeded into a fresh
                        // `Searching` window, which re-probes it and waits for
                        // the next periodic sync train.
                        if let Some(mode) = self.forced_mode {
                            let spec = crate::modespec::for_mode(mode);
                            self.state = Self::start_decoding(
                                spec,
                                0.0,
                                forced_tail.unwrap_or_default(),
                                0,
                                ForcedPhase::Searching,
                            );
                        }
                    } else {
                        // Image complete. Re-arm VIS detection in place (no
                        // `break`! — the loop re-iterates into `AwaitingVis`) — a
                        // back-to-back transmission's VIS leader starts right after
                        // this image's last line, so feed the fresh detector only
                        // the tail of the image audio (a few lines, to absorb a
                        // fast TX clock) plus everything past it; the rest is
                        // decoded video tones a VIS burst can't hide in. Falling
                        // through (vs the old `break`) means that next VIS — and
                        // the image after it — surface in this same `process()`
                        // call, mirroring the known/unknown-code branches above.
                        // Closes #31; #90 (A2 + D4). (`sample_offset` on detections
                        // after the first is relative to the carry-forward start,
                        // not absolute — #99.)
                        Self::restart_vis_detection(
                            &mut self.vis,
                            self.working_samples_emitted,
                            &carry_audio,
                        );
                    }
                    remaining = &[]; // already folded into d.audio → now inside the fresh detector
                }
            }
        }
        out
    }

    /// Discard `vis` and start a fresh detector on `leftover_audio`
    /// (post-stop-bit residue, or trailing image audio). Honors the `#40`
    /// re-anchor contract documented on
    /// [`crate::vis::VisDetector::take_residual_buffer`] — a spent detector's
    /// `hops_completed` / `history` state is never reset, so it must be
    /// replaced rather than re-used. `working_samples_emitted` is the
    /// decoder's running working-rate output count (used to anchor the fresh
    /// detector).
    fn restart_vis_detection(
        vis: &mut crate::vis::VisDetector,
        working_samples_emitted: u64,
        leftover_audio: &[f32],
    ) {
        *vis = crate::vis::VisDetector::new(IS_KNOWN_VIS);
        vis.process(leftover_audio, working_samples_emitted);
    }

    /// Build the [`State::Decoding`] for `spec` with `residual` already
    /// buffered and a total `target_audio_samples` window. Shared by the VIS
    /// path (residual = post-stop-bit audio, phase = `Collecting`) and the
    /// forced-mode path (residual = empty or the previous image's tail, phase =
    /// `Searching` or `Collecting`). `target_audio_samples` is ignored while
    /// `phase` is `Searching` (the window is sized when a train is found).
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn start_decoding(
        spec: crate::modespec::ModeSpec,
        hedr_shift_hz: f64,
        residual: Vec<f32>,
        target_audio_samples: usize,
        phase: ForcedPhase,
    ) -> State {
        let image = SstvImage::new(spec.mode, spec.line_pixels, spec.image_lines);
        State::Decoding(Box::new(DecodingState {
            mode: spec.mode,
            spec,
            image,
            audio: {
                // D6.1: keep the residual move + pre-reserve the remaining
                // capacity. Avoids copying residual bytes AND avoids Vec
                // growth reallocs over the burst.
                let mut v = residual;
                v.reserve(target_audio_samples.saturating_sub(v.len()));
                v
            },
            has_sync: Vec::with_capacity(target_audio_samples / SYNC_PROBE_STRIDE),
            next_probe_sample: 0,
            sync_tracker: SyncTracker::new(hedr_shift_hz),
            hedr_shift_hz,
            target_audio_samples,
            phase,
            chroma_planes: match spec.mode {
                SstvMode::Robot24 | SstvMode::Robot36 => {
                    let n = (spec.image_lines as usize) * (spec.line_pixels as usize);
                    Some([vec![0_u8; n], vec![0_u8; n]])
                }
                // PD modes compose RGB per-pair in place. Robot 72 and Scottie
                // 1/2/DX also compose RGB in-place per radio line (see
                // mode_robot::decode_r72_line, mode_scottie::decode_line);
                // only the R36/R24 chroma-alternation + duplication path
                // requires the side buffer. SstvMode is #[non_exhaustive], so
                // the wildcard arm is required; future modes with cross-line
                // chroma state would need to opt in here.
                _ => None,
            },
        }))
    }

    /// Run [`find_sync`] over the buffered sync track, then decode every
    /// PD line pair against the corrected `(rate, skip)`. Pushes
    /// [`SstvEvent::LineDecoded`] for every row + a final
    /// [`SstvEvent::ImageComplete`] into `out`.
    ///
    /// **Lookahead note (#33):** Each call to
    /// [`crate::mode_pd::decode_pd_line_pair`] receives `&d.audio` — the
    /// entire image audio buffer, not a slice ending at the pair's nominal
    /// end sample. This means the FFT window for the last pixel of the last
    /// channel of each line pair can freely extend rightward into subsequent
    /// pair audio (or zero if the buffer ends). The lookahead is therefore
    /// *implicit*: the full-buffer pass-through provides the context that a
    /// naive `&audio[..pair_end]` slice would lose. No explicit `lookahead`
    /// variable is required, and none should be added. (Issue #33 noted
    /// a now-deleted `lookahead` variable that was dead code; Phase 3's
    /// rewrite eliminated it by design.)
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        clippy::cast_possible_wrap,
        clippy::too_many_lines
    )]
    fn run_findsync_and_decode(
        mut d: DecodingState,
        channel_demod: &mut crate::demod::ChannelDemod,
        snr_est: &mut crate::snr::SnrEstimator,
        find_sync_scratch: &mut crate::sync::FindSyncScratch,
        out: &mut Vec<SstvEvent>,
        forced: bool,
    ) -> Option<Vec<f32>> {
        let work_rate = f64::from(crate::resample::WORKING_SAMPLE_RATE_HZ);
        // `find_sync` walks `image_lines` line periods, which for PD modes
        // spans *two* images (PD packs two image rows per radio line). When the
        // buffer carries audio past the current image (a large single `process`
        // call, or a gap before the next transmission) that second-image sync
        // train — at a different phase — would corrupt the falling-edge search.
        // Restrict forced mode to the collected window's sync track; the decode
        // still uses the full `d.audio` for its trailing lookahead.
        let sync_track: &[bool] = if forced {
            let window_probes = (d.target_audio_samples / SYNC_PROBE_STRIDE).min(d.has_sync.len());
            &d.has_sync[..window_probes]
        } else {
            &d.has_sync
        };
        let result = find_sync(sync_track, work_rate, d.spec, find_sync_scratch);
        let rate = result.adjusted_rate_hz;
        // A Hough peak means at least some sync pulses registered. In VIS mode
        // this is informational; in forced mode it gates emission so a window
        // of silence does not produce a black "image".
        let sync_found = result.slant_deg.is_some();

        if forced && !sync_found {
            // Forced mode on a sync-less window (silence, noise, or trailing
            // padding): emit nothing and advance by one full window so the
            // loop makes progress instead of re-scanning the same audio.
            let start = d.target_audio_samples.min(d.audio.len());
            return Some(d.audio[start..].to_vec());
        }

        // Forced mode starts at the stream origin, so a leading VIS header or
        // lead-in silence aliases `find_sync`'s line-relative skip by whole
        // lines; recover the absolute line-0 offset from the first periodic
        // sync pulse. VIS mode already starts at the stop bit, so it needs no
        // correction.
        let skip = if forced {
            crate::sync::absolute_skip_from_first_sync(
                result.skip_samples,
                sync_track,
                rate,
                d.spec,
            )
        } else {
            result.skip_samples
        };

        // Image-complete burst: image_lines LineDecoded events + 1 ImageComplete.
        // Pre-reserve to avoid Vec growth reallocs. (Audit #93 D5.)
        out.reserve(d.spec.image_lines as usize + 1);

        let line_pixels = d.spec.line_pixels as usize;
        match d.spec.channel_layout {
            crate::modespec::ChannelLayout::PdYcbcr => {
                let pair_count = d.spec.image_lines / 2;
                for pair in 0..pair_count {
                    // slowrx `video.c:140-142` computes pixel time as
                    // `Skip + round(Rate * (y/2 * LineTime + ChanStart +
                    // PixelTime * (x + 0.5)))`. Compute `pair_seconds = y/2 *
                    // LineTime` here (un-rounded) and let
                    // [`crate::mode_pd::decode_pd_line_pair`] fold it into its
                    // own `round()`, so per-pair rounding error never
                    // accumulates.
                    let pair_seconds = f64::from(pair) * d.spec.line_seconds;
                    crate::mode_pd::decode_pd_line_pair(
                        d.spec,
                        pair,
                        &d.audio,
                        skip,
                        pair_seconds,
                        rate,
                        &mut d.image,
                        channel_demod,
                        snr_est,
                        d.hedr_shift_hz,
                    );
                    let row0 = pair * 2;
                    let row1 = row0 + 1;
                    for r in [row0, row1] {
                        let start = (r as usize) * line_pixels;
                        let end = start + line_pixels;
                        out.push(SstvEvent::LineDecoded {
                            mode: d.mode,
                            line_index: r,
                            pixels: d.image.pixels[start..end].to_vec(),
                        });
                    }
                }
            }
            crate::modespec::ChannelLayout::RobotYuv => {
                // Robot is per-line (no PD line-pairing). For R36/R24 the
                // chroma-duplication writes to the next image row; that's
                // handled inside mode_robot::decode_line. LineDecoded for image
                // row N is emitted after radio-line N's decode — for R36/R24
                // row 0 the Cb channel is at zero-init at this point (slowrx
                // C does the same; final ImageComplete carries the populated
                // state).
                for line in 0..d.spec.image_lines {
                    let line_seconds_offset = f64::from(line) * d.spec.line_seconds;
                    crate::mode_robot::decode_line(
                        d.spec,
                        d.mode,
                        line,
                        &d.audio,
                        skip,
                        line_seconds_offset,
                        rate,
                        &mut d.image,
                        d.chroma_planes.as_mut(),
                        channel_demod,
                        snr_est,
                        d.hedr_shift_hz,
                    );
                    let start = (line as usize) * line_pixels;
                    let end = start + line_pixels;
                    out.push(SstvEvent::LineDecoded {
                        mode: d.mode,
                        line_index: line,
                        pixels: d.image.pixels[start..end].to_vec(),
                    });
                }
            }
            crate::modespec::ChannelLayout::RgbSequential => {
                // Scottie family. Mid-line sync handling lives inside
                // mode_scottie::decode_line. No chroma_planes — RGB is composed
                // in-place per line (no deferred chroma like R36/R24).
                for line in 0..d.spec.image_lines {
                    let line_seconds_offset = f64::from(line) * d.spec.line_seconds;
                    crate::mode_scottie::decode_line(
                        d.spec,
                        line,
                        &d.audio,
                        skip,
                        line_seconds_offset,
                        rate,
                        &mut d.image,
                        channel_demod,
                        snr_est,
                        d.hedr_shift_hz,
                    );
                    let start = (line as usize) * line_pixels;
                    let end = start + line_pixels;
                    out.push(SstvEvent::LineDecoded {
                        mode: d.mode,
                        line_index: line,
                        pixels: d.image.pixels[start..end].to_vec(),
                    });
                }
            }
        }

        // Move the now-populated image into the ImageComplete event. The
        // by-value `d` (audit #93 D6.2) lets `d.image` move directly
        // into the event without a fresh black-image realloc.
        out.push(SstvEvent::ImageComplete {
            image: d.image,
            partial: false,
        });

        // Forced mode: return the audio after this image so the caller can
        // seed the next window. VIS mode re-arms via its carryback and
        // returns `None`.
        if forced {
            let image_end = skip + (nominal_image_seconds(d.spec) * rate).round() as i64;
            let start = image_end.clamp(0, d.audio.len() as i64) as usize;
            Some(d.audio[start..].to_vec())
        } else {
            None
        }
    }

    /// Reset to `AwaitingVis`; discard any in-flight image. The forced-decoding
    /// mode (if one is set via [`Self::with_mode`] / [`Self::set_forced_mode`])
    /// is preserved.
    pub fn reset(&mut self) {
        self.state = State::AwaitingVis;
        self.samples_processed = 0;
        self.working_samples_emitted = 0;
        self.vis = crate::vis::VisDetector::new(IS_KNOWN_VIS);
        self.resampler.reset_state();
        self.channel_demod = crate::demod::ChannelDemod::new();
        self.snr_est = crate::snr::SnrEstimator::new();
    }

    /// Total samples processed since construction (or last `reset`).
    #[must_use]
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed
    }
}

impl std::fmt::Debug for SstvDecoder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SstvDecoder")
            .field("rate", &self.resampler.input_rate())
            .field("state", &self.state)
            .field("samples_processed", &self.samples_processed)
            .field("working_samples_emitted", &self.working_samples_emitted)
            .field("forced_mode", &self.forced_mode)
            .finish_non_exhaustive()
    }
}

/// Estimate the dominant tone frequency in `window` (working-rate samples).
/// Returns the estimated frequency in Hz, biased toward 1500-2300 Hz
/// (the SSTV video band).
///
/// Algorithm: Goertzel-bank evaluated at 25-Hz steps from 1450 to 2350 Hz,
/// then quadratic peak interpolation around the maximum bin.
#[must_use]
#[allow(clippy::cast_precision_loss, dead_code)]
pub(crate) fn estimate_freq(window: &[f32]) -> f64 {
    const STEP_HZ: f64 = 25.0;
    const FIRST_HZ: f64 = 1450.0;
    const N_BINS: usize = 37; // 1450..2350 inclusive at 25 Hz steps

    let mut powers = [0.0_f64; N_BINS];
    for (i, p) in powers.iter_mut().enumerate() {
        let f = FIRST_HZ + (i as f64) * STEP_HZ;
        *p = crate::dsp::goertzel_power(window, f);
    }
    let (mut max_i, mut max_p) = (0_usize, powers[0]);
    for (i, &p) in powers.iter().enumerate().skip(1) {
        if p > max_p {
            max_p = p;
            max_i = i;
        }
    }
    let center_hz = FIRST_HZ + (max_i as f64) * STEP_HZ;
    // Quadratic interpolation if we have both neighbours.
    if max_i > 0 && max_i < N_BINS - 1 && max_p > 0.0 {
        let a = powers[max_i - 1];
        let b = max_p;
        let c = powers[max_i + 1];
        let denom = a - 2.0 * b + c;
        if denom.abs() > 1e-12 {
            let delta = 0.5 * (a - c) / denom;
            return center_hz + delta * STEP_HZ;
        }
    }
    center_hz
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
mod tests {
    use super::*;
    use crate::error::Error;
    use crate::resample::{MAX_INPUT_SAMPLE_RATE_HZ, WORKING_SAMPLE_RATE_HZ};

    #[test]
    fn rejects_invalid_sample_rates() {
        assert!(matches!(
            SstvDecoder::new(0),
            Err(Error::InvalidSampleRate { got: 0 })
        ));
        assert!(matches!(
            SstvDecoder::new(MAX_INPUT_SAMPLE_RATE_HZ + 1),
            Err(Error::InvalidSampleRate { .. })
        ));
    }

    #[test]
    fn accepts_common_rates() {
        assert!(SstvDecoder::new(11_025).is_ok());
        assert!(SstvDecoder::new(44_100).is_ok());
        assert!(SstvDecoder::new(48_000).is_ok());
    }

    #[test]
    fn process_advances_sample_counter() {
        let mut d = SstvDecoder::new(11_025).expect("decoder");
        assert_eq!(d.samples_processed(), 0);
        let _ = d.process(&[0.0_f32; 1024]);
        assert_eq!(d.samples_processed(), 1024);
        let _ = d.process(&[0.0_f32; 256]);
        assert_eq!(d.samples_processed(), 1280);
    }

    #[test]
    fn process_returns_no_events_for_silence() {
        let mut d = SstvDecoder::new(11_025).expect("decoder");
        // Silence produces no VIS match.
        let events = d.process(&[0.5_f32; 512]);
        assert!(events.is_empty());
    }

    #[test]
    fn process_emits_vis_detected_for_pd120_burst() {
        use crate::vis::tests::synth_vis;
        let mut d = SstvDecoder::new(WORKING_SAMPLE_RATE_HZ).expect("decoder");
        // Pad with trailing silence so the polyphase FIR's ~64-sample group
        // delay still yields a full set of stop-bit windows (PR-2 T2.1).
        let mut burst = synth_vis(0x5F, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        let hedr = events
            .iter()
            .find_map(|e| match e {
                SstvEvent::VisDetected {
                    mode: SstvMode::Pd120,
                    hedr_shift_hz,
                    ..
                } => Some(*hedr_shift_hz),
                _ => None,
            })
            .expect("expected VisDetected for PD120");
        assert!(
            hedr.abs() < 10.0,
            "synthetic burst should report ~0 Hz shift, got {hedr}"
        );
    }

    #[test]
    fn process_emits_vis_detected_for_pd180_burst() {
        use crate::vis::tests::synth_vis;
        let mut d = SstvDecoder::new(WORKING_SAMPLE_RATE_HZ).expect("decoder");
        let mut burst = synth_vis(0x60, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        let hedr = events
            .iter()
            .find_map(|e| match e {
                SstvEvent::VisDetected {
                    mode: SstvMode::Pd180,
                    hedr_shift_hz,
                    ..
                } => Some(*hedr_shift_hz),
                _ => None,
            })
            .expect("expected VisDetected for PD180");
        assert!(hedr.abs() < 10.0);
    }

    #[test]
    fn process_emits_vis_detected_for_pd240_burst() {
        use crate::vis::tests::synth_vis;
        let mut d = SstvDecoder::new(WORKING_SAMPLE_RATE_HZ).expect("decoder");
        let mut burst = synth_vis(0x61, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        let hedr = events
            .iter()
            .find_map(|e| match e {
                SstvEvent::VisDetected {
                    mode: SstvMode::Pd240,
                    hedr_shift_hz,
                    ..
                } => Some(*hedr_shift_hz),
                _ => None,
            })
            .expect("expected VisDetected for PD240");
        assert!(hedr.abs() < 10.0);
    }

    #[test]
    fn process_emits_vis_detected_for_robot24_burst() {
        use crate::vis::tests::synth_vis;
        let mut d = SstvDecoder::new(WORKING_SAMPLE_RATE_HZ).expect("decoder");
        let mut burst = synth_vis(0x04, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        let hedr = events
            .iter()
            .find_map(|e| match e {
                SstvEvent::VisDetected {
                    mode: SstvMode::Robot24,
                    hedr_shift_hz,
                    ..
                } => Some(*hedr_shift_hz),
                _ => None,
            })
            .expect("expected VisDetected for Robot24");
        assert!(hedr.abs() < 10.0);
    }

    #[test]
    fn process_emits_vis_detected_for_robot36_burst() {
        use crate::vis::tests::synth_vis;
        let mut d = SstvDecoder::new(WORKING_SAMPLE_RATE_HZ).expect("decoder");
        let mut burst = synth_vis(0x08, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        let hedr = events
            .iter()
            .find_map(|e| match e {
                SstvEvent::VisDetected {
                    mode: SstvMode::Robot36,
                    hedr_shift_hz,
                    ..
                } => Some(*hedr_shift_hz),
                _ => None,
            })
            .expect("expected VisDetected for Robot36");
        assert!(hedr.abs() < 10.0);
    }

    #[test]
    fn process_emits_vis_detected_for_robot72_burst() {
        use crate::vis::tests::synth_vis;
        let mut d = SstvDecoder::new(WORKING_SAMPLE_RATE_HZ).expect("decoder");
        let mut burst = synth_vis(0x0C, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        let hedr = events
            .iter()
            .find_map(|e| match e {
                SstvEvent::VisDetected {
                    mode: SstvMode::Robot72,
                    hedr_shift_hz,
                    ..
                } => Some(*hedr_shift_hz),
                _ => None,
            })
            .expect("expected VisDetected for Robot72");
        assert!(hedr.abs() < 10.0);
    }

    #[test]
    fn reset_clears_sample_counter() {
        let mut d = SstvDecoder::new(11_025).expect("decoder");
        let _ = d.process(&[0.0_f32; 1024]);
        d.reset();
        assert_eq!(d.samples_processed(), 0);
    }

    // 40 ms tones make every 25-Hz bank bin map to a unique Goertzel k
    // (11025/441 = 25.0). Production windows are ~5 ms; ~50 Hz suffices.
    fn synth_tone_at_working(freq_hz: f64, secs: f64) -> Vec<f32> {
        let sr = f64::from(WORKING_SAMPLE_RATE_HZ);
        let n = (secs * sr).round() as usize;
        (0..n)
            .map(|i| (2.0 * std::f64::consts::PI * freq_hz * (i as f64) / sr).sin() as f32)
            .collect()
    }

    #[test]
    fn estimate_freq_recovers_known_tone() {
        for &f in &[1500.0_f64, 1700.0, 1900.0, 2100.0, 2300.0] {
            let window = synth_tone_at_working(f, 0.040);
            let est = estimate_freq(&window);
            assert!((est - f).abs() < 30.0, "freq={f} estimate={est}");
        }
    }

    #[test]
    fn estimate_freq_no_interp_at_left_boundary() {
        // Tone at 1450 Hz lands on bin 0; no left neighbour → no interp.
        let window = synth_tone_at_working(1450.0, 0.040);
        let est = estimate_freq(&window);
        assert!((est - 1450.0).abs() < 30.0, "expected ≈1450, got {est}");
    }

    #[test]
    fn reset_during_decoding_emits_partial_via_subsequent_process() {
        let mut d = SstvDecoder::new(crate::resample::WORKING_SAMPLE_RATE_HZ).unwrap();
        // Push a VIS so the decoder transitions to Decoding. Trailing zeros
        // accommodate the FIR group delay so the burst actually triggers
        // detection (without the padding the test would mask Finding 1
        // by never entering Decoding).
        let mut burst = crate::vis::tests::synth_vis(0x5F, 0.0);
        burst.extend(std::iter::repeat_n(0.0_f32, 512));
        let events = d.process(&burst);
        assert!(
            events
                .iter()
                .any(|e| matches!(e, SstvEvent::VisDetected { .. })),
            "expected VIS detection before reset, got {events:?}"
        );
        // We're now in Decoding state.
        d.reset();
        // After reset, the decoder is back in AwaitingVis with FIR resampler
        // and ChannelDemod state cleared. The next process call with quiet audio
        // yields no events.
        let events = d.process(&[0.0_f32; 100]);
        assert!(
            events.is_empty(),
            "reset should clear in-flight; got {events:?}"
        );
    }

    // TODO(future/PR-3): mid_image_vis_emits_partial_then_new_vis
    //
    // When a new VIS burst arrives during Decoding the spec calls for
    // emitting `ImageComplete { partial: true }` for the in-flight image,
    // then transitioning to AwaitingVis.
    //
    // The naive approach (running `self.vis` against the decoding buffer
    // each call) fails because the residual buffer from a previous VIS
    // detection is not aligned to 30 ms window boundaries: the first
    // classifier window is a mix of silence and leader tone and does not
    // reliably pass the 5× dominance threshold. A correct implementation
    // would re-align the scan to the next 30 ms boundary or run a separate
    // 1900 Hz energy detector. Deferred to PR-3 (cross-validation).
}
