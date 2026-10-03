//! Forced-mode decoding (issue #113): manually specify an SSTV mode and
//! bypass automatic VIS header detection.
//!
//! These use the same synthetic PD120 encoder as `tests/roundtrip.rs`, but
//! feed the audio **without** a VIS header (except where the test explicitly
//! checks that a leading header is absorbed by the forced-mode preroll).

#![cfg(feature = "test-support")]
#![allow(
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use slowrx::{SstvDecoder, SstvEvent, SstvMode, WORKING_SAMPLE_RATE_HZ};

/// PD120 == VIS code 0x5F.
const PD120_CODE: u8 = 0x5F;

/// Luma gradient + smooth chroma stripes, matching `tests/roundtrip.rs`'s
/// shape so the synthetic encoder's adjacent-row chroma averaging is
/// reproducible.
fn pd120_test_image() -> Vec<[u8; 3]> {
    let spec = slowrx::for_mode(SstvMode::Pd120);
    let w = spec.line_pixels;
    let h = spec.image_lines;
    let mut ycrcb = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let lum = ((f64::from(x)) / (f64::from(w)) * 255.0) as u8;
            let cr = if y % 4 < 2 { 200 } else { 56 };
            let cb = if (y / 2) % 2 == 0 { 200 } else { 56 };
            ycrcb.push([lum, cr, cb]);
        }
    }
    ycrcb
}

fn pd120_audio() -> Vec<f32> {
    slowrx::__test_support::mode_pd::encode_pd(SstvMode::Pd120, &pd120_test_image())
}

/// Padding that clears the resampler group delay *and* the one-second
/// forced-mode preroll, so a single image can complete.
fn pad(audio: &mut Vec<f32>) {
    audio.extend(std::iter::repeat_n(
        0.0_f32,
        WORKING_SAMPLE_RATE_HZ as usize + 4096,
    ));
}

fn count_complete(events: &[SstvEvent]) -> usize {
    events
        .iter()
        .filter(|e| matches!(e, SstvEvent::ImageComplete { partial: false, .. }))
        .count()
}

/// Mean absolute per-channel error of a decoded PD120 image against the
/// synthetic source.
fn pd120_mean_err(image: &slowrx::SstvImage) -> f64 {
    let src = pd120_test_image();
    let mut sum_diff: u64 = 0;
    let mut n: u64 = 0;
    for (i, s) in src.iter().enumerate() {
        let src_rgb = slowrx::__test_support::mode_pd::ycbcr_to_rgb(s[0], s[1], s[2]);
        let dec = image.pixels[i];
        for (src_ch, dec_ch) in src_rgb.iter().zip(dec.iter()) {
            sum_diff += u64::from((i32::from(*src_ch) - i32::from(*dec_ch)).unsigned_abs());
            n += 1;
        }
    }
    sum_diff as f64 / n as f64
}

/// Assert the decoded PD120 image matches the encoded source closely (same
/// mean-absolute-error budget as `tests/roundtrip.rs`).
fn assert_pd120_close(image: &slowrx::SstvImage) {
    let spec = slowrx::for_mode(SstvMode::Pd120);
    assert_eq!(image.mode, SstvMode::Pd120);
    assert_eq!(image.width, spec.line_pixels);
    assert_eq!(image.height, spec.image_lines);
    let mean = pd120_mean_err(image);
    assert!(mean < 5.0, "mean={mean:.2}");
}

#[test]
fn forced_mode_decodes_vis_less_pd120() {
    let mut audio = pd120_audio();
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");

    let events = decoder.process(&audio);

    // No VIS was present, so no VisDetected may be emitted.
    assert!(
        !events
            .iter()
            .any(|e| matches!(e, SstvEvent::VisDetected { .. })),
        "forced mode must not emit VisDetected; got {events:?}"
    );
    assert_eq!(count_complete(&events), 1, "expected exactly one image");
    let image = events
        .iter()
        .find_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .expect("ImageComplete");
    assert_pd120_close(image);
}

#[test]
fn forced_mode_absorbs_leading_vis_header() {
    // A file that still carries a (valid) VIS header should decode correctly
    // too: the one-second preroll absorbs the header, and `find_sync` locates
    // line 0 after it.
    let mut audio = slowrx::__test_support::vis::synth_vis(PD120_CODE, 0.0);
    audio.extend(pd120_audio());
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");
    let events = decoder.process(&audio);

    // Forced mode ignores the header, so no VisDetected event.
    assert!(!events
        .iter()
        .any(|e| matches!(e, SstvEvent::VisDetected { .. })));
    assert_eq!(count_complete(&events), 1, "expected exactly one image");
    let image = events
        .iter()
        .find_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .expect("ImageComplete");
    assert_pd120_close(image);
}

#[test]
fn forced_mode_recovers_from_leading_silence() {
    // 0.7 s of lead-in silence exceeds one PD120 line (0.508 s), so this
    // exercises the whole-line alias recovery (the first periodic sync is not
    // line 0's *first* line-clock slot). Stays within the one-second preroll
    // so the streaming path would also buffer the whole image.
    let mut audio = vec![0.0_f32; (f64::from(WORKING_SAMPLE_RATE_HZ) * 0.7) as usize];
    audio.extend(pd120_audio());
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");
    let events = decoder.process(&audio);

    assert_eq!(count_complete(&events), 1, "expected exactly one image");
    let image = events
        .iter()
        .find_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .expect("ImageComplete");
    assert_pd120_close(image);
}

#[test]
fn forced_mode_decodes_back_to_back_images() {
    let image = pd120_audio();
    let mut audio = image.clone();
    audio.extend(image);
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");
    let events = decoder.process(&audio); // ONE call

    assert_eq!(
        count_complete(&events),
        2,
        "expected two back-to-back images; got {} events",
        events.len()
    );
}

#[test]
fn forced_mode_on_silence_emits_nothing() {
    // Robot 24 has the shortest synthetic-friendly image length (~36 s); feed
    // more than one forced window of silence and assert no black image is
    // fabricated. (The forced path suppresses windows with no sync pulses.)
    let mut audio = vec![0.0_f32; WORKING_SAMPLE_RATE_HZ as usize * 45];
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Robot24).expect("decoder");
    let events = decoder.process(&audio);

    assert_eq!(
        count_complete(&events),
        0,
        "forced mode must not fabricate an image from silence"
    );
}

/// A recording with three full transmissions separated by multi-second gaps
/// must yield exactly three aligned images — not a split of each one across
/// window boundaries (issue #113 follow-up).
#[test]
fn forced_mode_decodes_three_images_separated_by_gaps() {
    let gap = |secs: f64| {
        std::iter::repeat_n(0.0_f32, (f64::from(WORKING_SAMPLE_RATE_HZ) * secs) as usize)
    };
    let mut audio = pd120_audio();
    audio.extend(gap(2.0));
    audio.extend(pd120_audio());
    audio.extend(gap(3.0));
    audio.extend(pd120_audio());
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");
    let events = decoder.process(&audio);

    assert_eq!(
        count_complete(&events),
        3,
        "expected three images; got {} complete events",
        count_complete(&events)
    );
    for (idx, image) in events
        .iter()
        .filter_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .enumerate()
    {
        let mean = pd120_mean_err(image);
        assert!(mean < 5.0, "image {idx} mean={mean:.2}");
    }
}

/// Same scenario as above, but fed in small chunks so the `Searching` phase's
/// memory-bounding discard runs across many `process` calls.
#[test]
fn forced_mode_streaming_decodes_three_images_separated_by_gaps() {
    let gap = |secs: f64| {
        std::iter::repeat_n(0.0_f32, (f64::from(WORKING_SAMPLE_RATE_HZ) * secs) as usize)
    };
    let mut audio = pd120_audio();
    audio.extend(gap(2.0));
    audio.extend(pd120_audio());
    audio.extend(gap(3.0));
    audio.extend(pd120_audio());
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");
    let mut events = Vec::new();
    for chunk in audio.chunks(4096) {
        events.extend(decoder.process(chunk));
    }

    assert_eq!(
        count_complete(&events),
        3,
        "expected three images from a chunked stream; got {}",
        count_complete(&events)
    );
    for (idx, image) in events
        .iter()
        .filter_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .enumerate()
    {
        let mean = pd120_mean_err(image);
        assert!(mean < 5.0, "image {idx} mean={mean:.2}");
    }
}

/// Forced acquisition must also work for a mid-line-sync mode (Scottie), where
/// `find_sync`'s skip carries a mode-specific correction and two transmissions
/// are separated by a gap.
#[test]
#[allow(clippy::many_single_char_names)]
fn forced_mode_decodes_scottie_with_midline_sync() {
    let spec = slowrx::for_mode(SstvMode::Scottie1);
    let (w, h) = (spec.line_pixels, spec.image_lines);
    let mut rgb = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let r = ((f64::from(x)) / (f64::from(w)) * 255.0) as u8;
            let g = if y % 8 < 4 { 200 } else { 56 };
            let b = if (y + 2) % 8 < 4 { 200 } else { 56 };
            rgb.push([r, g, b]);
        }
    }
    let gap = |secs: f64| {
        std::iter::repeat_n(0.0_f32, (f64::from(WORKING_SAMPLE_RATE_HZ) * secs) as usize)
    };
    let image = slowrx::__test_support::mode_scottie::encode_scottie(SstvMode::Scottie1, &rgb);

    let mut audio: Vec<f32> = gap(0.5).collect();
    audio.extend(image.clone());
    audio.extend(gap(2.0));
    audio.extend(image);
    pad(&mut audio);

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Scottie1).expect("decoder");
    let events = decoder.process(&audio);

    let images: Vec<_> = events
        .iter()
        .filter_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .collect();
    assert_eq!(images.len(), 2, "expected two Scottie images");
    for (idx, image) in images.iter().enumerate() {
        assert_eq!(image.mode, SstvMode::Scottie1);
        let mut sum_diff: u64 = 0;
        let mut n: u64 = 0;
        for (i, src) in rgb.iter().enumerate() {
            for (src_ch, dec_ch) in src.iter().zip(image.pixels[i].iter()) {
                sum_diff += u64::from((i32::from(*src_ch) - i32::from(*dec_ch)).unsigned_abs());
                n += 1;
            }
        }
        let mean = sum_diff as f64 / n as f64;
        assert!(mean < 5.0, "image {idx} mean={mean:.2}");
    }
}

/// Isolated 1200 Hz bursts that do not repeat on the mode's line clock must not
/// be mistaken for an image (the pulse-count gate, issue #113 follow-up).
#[test]
fn forced_mode_rejects_non_periodic_sync_bursts() {
    let sr = f64::from(WORKING_SAMPLE_RATE_HZ);
    let total = (sr * 20.0) as usize;
    let mut audio = vec![0.0_f32; total];
    let burst = (sr * 0.015) as usize;
    let spacing = (sr * 0.2) as usize;
    let mut i = 0;
    while i + burst <= total {
        for (n, slot) in audio.iter_mut().enumerate().skip(i).take(burst) {
            *slot = (2.0 * std::f64::consts::PI * 1200.0 * (n as f64) / sr).sin() as f32;
        }
        i += spacing;
    }

    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Robot24).expect("decoder");
    let events = decoder.process(&audio);

    assert_eq!(
        count_complete(&events),
        0,
        "non-periodic bursts must not fabricate an image"
    );
}

#[test]
fn set_forced_mode_none_restores_vis_detection() {
    let mut decoder =
        SstvDecoder::with_mode(WORKING_SAMPLE_RATE_HZ, SstvMode::Pd120).expect("decoder");
    assert_eq!(decoder.forced_mode(), Some(SstvMode::Pd120));

    decoder.set_forced_mode(None);
    assert_eq!(decoder.forced_mode(), None);

    let mut burst = slowrx::__test_support::vis::synth_vis(PD120_CODE, 0.0);
    burst.extend(std::iter::repeat_n(0.0_f32, 512));
    let events = decoder.process(&burst);
    assert!(
        events
            .iter()
            .any(|e| matches!(e, SstvEvent::VisDetected { .. })),
        "VIS detection should be active after clearing the forced mode; got {events:?}"
    );
}
