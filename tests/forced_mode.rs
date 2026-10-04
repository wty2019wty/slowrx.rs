//! Forced-mode decoding (issues #113/#114): specify an SSTV mode **and** a
//! decode window, bypassing automatic VIS header detection. A forced mode
//! always carries a window, so these tests all anchor explicitly.
//!
//! These use the same synthetic PD120 encoder as `tests/roundtrip.rs`, but
//! feed the audio **without** a VIS header (except where a test explicitly
//! checks that a leading header is absorbed).

#![cfg(feature = "test-support")]
#![allow(
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use slowrx::{DecodeWindow, SstvDecoder, SstvEvent, SstvMode, WORKING_SAMPLE_RATE_HZ};

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

/// Padding that clears the resampler group delay *and* the forced-mode trailing
/// margin, so a single image can complete.
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

// ---------------------------------------------------------------------------
// Manual decode window (issue #114)
// ---------------------------------------------------------------------------

/// Nominal image airtime (VIS not counted), in seconds.
fn image_secs(mode: SstvMode) -> f64 {
    let spec = slowrx::for_mode(mode);
    let radio_frames = match spec.channel_layout {
        slowrx::ChannelLayout::PdYcbcr => spec.image_lines / 2,
        _ => spec.image_lines,
    };
    f64::from(radio_frames) * spec.line_seconds
}

/// A PD120 source image with every luma sample shifted by `shift` (wrapping).
/// Lets the tests tell two otherwise-identical transmissions apart.
fn pd120_source_luma_shift(shift: u8) -> Vec<[u8; 3]> {
    let spec = slowrx::for_mode(SstvMode::Pd120);
    let w = spec.line_pixels;
    let h = spec.image_lines;
    let mut ycrcb = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let lum = (((f64::from(x)) / (f64::from(w)) * 255.0) as u8).wrapping_add(shift);
            let cr = if y % 4 < 2 { 200 } else { 56 };
            let cb = if (y / 2) % 2 == 0 { 200 } else { 56 };
            ycrcb.push([lum, cr, cb]);
        }
    }
    ycrcb
}

fn pd120_audio_luma_shift(shift: u8) -> Vec<f32> {
    slowrx::__test_support::mode_pd::encode_pd(SstvMode::Pd120, &pd120_source_luma_shift(shift))
}

fn mean_err_against(image: &slowrx::SstvImage, src: &[[u8; 3]]) -> f64 {
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

fn only_complete(events: &[SstvEvent]) -> &slowrx::SstvImage {
    assert_eq!(count_complete(events), 1, "expected exactly one image");
    events
        .iter()
        .find_map(|e| match e {
            SstvEvent::ImageComplete { image, .. } => Some(image),
            _ => None,
        })
        .expect("ImageComplete")
}

/// `--start` pointing at the image's first line decodes the whole image.
#[test]
fn manual_start_at_image_start_decodes_pd120() {
    let mut audio = pd120_audio();
    pad(&mut audio);
    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Pd120,
        DecodeWindow::starting_at(0.0),
    )
    .expect("decoder");
    let events = decoder.process(&audio);
    assert_pd120_close(only_complete(&events));
}

/// `--start` pointing at the transmission start (VIS header present) must
/// absorb the header so the last ~0.6–0.9 s of image is not cut off
/// (requirement 4). The mode is forced, so no `VisDetected` is emitted.
#[test]
fn manual_start_at_transmission_start_absorbs_vis() {
    let mut audio = slowrx::__test_support::vis::synth_vis(PD120_CODE, 0.0);
    audio.extend(pd120_audio());
    pad(&mut audio);

    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Pd120,
        DecodeWindow::starting_at(0.0),
    )
    .expect("decoder");
    let events = decoder.process(&audio);

    assert!(
        !events
            .iter()
            .any(|e| matches!(e, SstvEvent::VisDetected { .. })),
        "manual forced decode must not emit VisDetected"
    );
    assert_pd120_close(only_complete(&events));
}

/// `--end` is the image-data end; the start is one nominal image back and the
/// image still decodes in full even with a preceding VIS header.
#[test]
fn manual_end_decodes_pd120() {
    let vis = slowrx::__test_support::vis::synth_vis(PD120_CODE, 0.0);
    let vis_secs = vis.len() as f64 / f64::from(WORKING_SAMPLE_RATE_HZ);
    let mut audio = vis;
    audio.extend(pd120_audio());
    pad(&mut audio);

    let end = vis_secs + image_secs(SstvMode::Pd120);
    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Pd120,
        DecodeWindow::ending_at(end),
    )
    .expect("decoder");
    let events = decoder.process(&audio);
    assert_pd120_close(only_complete(&events));
}

/// A window anchored on the *second* of two transmissions decodes that image
/// and nothing else (not the first, not a third).
#[test]
fn manual_window_selects_the_anchored_image() {
    let gap = |secs: f64| {
        std::iter::repeat_n(0.0_f32, (f64::from(WORKING_SAMPLE_RATE_HZ) * secs) as usize)
    };
    let first_secs = image_secs(SstvMode::Pd120);
    let second_start = first_secs + 2.0;
    let mut audio = pd120_audio_luma_shift(0);
    audio.extend(gap(2.0));
    audio.extend(pd120_audio_luma_shift(100));
    pad(&mut audio);

    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Pd120,
        DecodeWindow::starting_at(second_start),
    )
    .expect("decoder");
    let events = decoder.process(&audio);

    let image = only_complete(&events);
    let second = pd120_source_luma_shift(100);
    let first = pd120_source_luma_shift(0);
    assert!(
        mean_err_against(image, &second) < 5.0,
        "decoded image should match the anchored second transmission"
    );
    assert!(
        mean_err_against(image, &first) > 20.0,
        "decoded image must not be the first transmission"
    );
}

/// After the manual window's one image the decoder stops, even if more audio
/// (a second back-to-back image) follows.
#[test]
fn manual_window_stops_after_one_image() {
    let image = pd120_audio();
    let mut audio = image.clone();
    audio.extend(image);
    pad(&mut audio);

    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Pd120,
        DecodeWindow::starting_at(0.0),
    )
    .expect("decoder");
    let events = decoder.process(&audio);
    assert_eq!(
        count_complete(&events),
        1,
        "manual window decodes one image"
    );
}

/// Robot 36 is the canonical "36 s image, 36.9 s with VIS" case: anchoring at
/// the transmission start must still recover all 240 lines.
#[test]
fn manual_start_robot36_absorbs_vis_to_full_36s() {
    let spec = slowrx::for_mode(SstvMode::Robot36);
    let (w, h) = (spec.line_pixels, spec.image_lines);
    let mut ycrcb = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let lum = ((f64::from(x)) / (f64::from(w)) * 255.0) as u8;
            let cr = if y % 4 < 2 { 200 } else { 56 };
            let cb = if (y + 1) % 4 < 2 { 200 } else { 56 };
            ycrcb.push([lum, cr, cb]);
        }
    }
    let mut audio = slowrx::__test_support::vis::synth_vis(0x08, 0.0);
    audio.extend(slowrx::__test_support::mode_robot::encode_robot(
        SstvMode::Robot36,
        &ycrcb,
    ));
    pad(&mut audio);

    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Robot36,
        DecodeWindow::starting_at(0.0),
    )
    .expect("decoder");
    let events = decoder.process(&audio);

    let image = only_complete(&events);
    assert_eq!(image.width, w);
    assert_eq!(image.height, h);
    let mean = mean_err_against(image, &ycrcb);
    assert!(mean < 5.0, "Robot 36 mean={mean:.2}");
}

/// Mid-line-sync mode (Scottie), VIS header present: the header is absorbed and
/// `find_sync`'s mode-specific skip correction still lands line 0.
#[test]
#[allow(clippy::many_single_char_names)]
fn manual_window_decodes_scottie_absorbing_vis() {
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
    let mut audio = slowrx::__test_support::vis::synth_vis(0x3C, 0.0);
    audio.extend(slowrx::__test_support::mode_scottie::encode_scottie(
        SstvMode::Scottie1,
        &rgb,
    ));
    pad(&mut audio);

    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Scottie1,
        DecodeWindow::starting_at(0.0),
    )
    .expect("decoder");
    let events = decoder.process(&audio);

    let image = only_complete(&events);
    assert_eq!(image.mode, SstvMode::Scottie1);
    // Scottie sources are RGB, so compare channels directly.
    let mut sum_diff: u64 = 0;
    let mut n: u64 = 0;
    for (i, src) in rgb.iter().enumerate() {
        for (src_ch, dec_ch) in src.iter().zip(image.pixels[i].iter()) {
            sum_diff += u64::from((i32::from(*src_ch) - i32::from(*dec_ch)).unsigned_abs());
            n += 1;
        }
    }
    let mean = sum_diff as f64 / n as f64;
    assert!(mean < 5.0, "Scottie mean={mean:.2}");
}

/// A window anchored on silence fabricates nothing (sync gate).
#[test]
fn manual_window_on_silence_emits_nothing() {
    let mut audio = vec![0.0_f32; WORKING_SAMPLE_RATE_HZ as usize * 45];
    pad(&mut audio);
    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Robot24,
        DecodeWindow::starting_at(0.0),
    )
    .expect("decoder");
    let events = decoder.process(&audio);
    assert_eq!(
        count_complete(&events),
        0,
        "manual window on silence must not fabricate an image"
    );
}

#[test]
fn clear_forced_mode_restores_vis_detection() {
    let mut decoder = SstvDecoder::with_mode(
        WORKING_SAMPLE_RATE_HZ,
        SstvMode::Pd120,
        DecodeWindow::starting_at(1.0),
    )
    .expect("decoder");
    assert_eq!(decoder.forced_mode(), Some(SstvMode::Pd120));
    assert!(decoder.decode_window().is_some());

    decoder.clear_forced_mode();
    assert_eq!(decoder.forced_mode(), None);
    assert!(decoder.decode_window().is_none());

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
