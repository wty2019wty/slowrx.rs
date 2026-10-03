//! Integration test for the `slowrx-cli` binary.
//!
//! Skips silently if no ARISS fixture is available locally — `docs/wav_files/`
//! is gitignored. CI does not run this; it's for local validation that the
//! installed binary works end-to-end against real audio.

#![cfg(feature = "cli")]
#![allow(clippy::expect_used, clippy::unwrap_used)]

use std::fs;
use std::path::PathBuf;

use assert_cmd::Command;

fn first_aris_fixture() -> Option<PathBuf> {
    // CARGO_MANIFEST_DIR is the slowrx.rs root.
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let dir = root.join("docs/wav_files/201712-ISS_SSTV");
    if !dir.is_dir() {
        return None;
    }
    fs::read_dir(&dir)
        .ok()?
        .filter_map(Result::ok)
        .map(|e| e.path())
        .find(|p| p.extension().is_some_and(|x| x == "wav"))
}

#[test]
fn cli_decodes_aris_fixture_to_png() {
    let Some(wav) = first_aris_fixture() else {
        eprintln!("skipping: no ARISS fixture in docs/wav_files/201712-ISS_SSTV/");
        return;
    };

    let out_dir = std::env::temp_dir().join(format!("slowrx-cli-test-{}", std::process::id()));
    let _ = fs::remove_dir_all(&out_dir);

    Command::cargo_bin("slowrx-cli")
        .expect("binary built")
        .arg("--input")
        .arg(&wav)
        .arg("--output")
        .arg(&out_dir)
        .arg("--quiet")
        .assert()
        .success();

    let pngs: Vec<_> = fs::read_dir(&out_dir)
        .expect("output dir exists")
        .filter_map(Result::ok)
        .filter(|e| e.path().extension().is_some_and(|x| x == "png"))
        .collect();
    assert!(
        !pngs.is_empty(),
        "expected at least one PNG written to {}",
        out_dir.display()
    );

    let _ = fs::remove_dir_all(&out_dir);
}

#[cfg(feature = "test-support")]
#[test]
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
fn cli_list_modes_prints_table() {
    let output = Command::cargo_bin("slowrx-cli")
        .expect("binary built")
        .arg("--list-modes")
        .output()
        .expect("run --list-modes");
    assert!(output.status.success(), "exit status: {}", output.status);
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("pd120"), "stdout: {stdout}");
    assert!(stdout.contains("robot36"), "stdout: {stdout}");
    assert!(stdout.contains("PD-120"), "stdout: {stdout}");
}

/// End-to-end: force a mode on a VIS-less synthetic PD120 WAV and check the
/// PNG comes out under the forced mode's filename (issue #113).
#[cfg(feature = "test-support")]
#[test]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]
fn cli_mode_flag_decodes_vis_less_wav() {
    use slowrx::{SstvMode, WORKING_SAMPLE_RATE_HZ};

    // Synthetic PD120 image (same shape as other tests).
    let spec = slowrx::for_mode(SstvMode::Pd120);
    let (w, h) = (spec.line_pixels, spec.image_lines);
    let mut ycrcb = Vec::with_capacity((w * h) as usize);
    for y in 0..h {
        for x in 0..w {
            let lum = ((f64::from(x)) / (f64::from(w)) * 255.0) as u8;
            let cr = if y % 4 < 2 { 200 } else { 56 };
            let cb = if (y / 2) % 2 == 0 { 200 } else { 56 };
            ycrcb.push([lum, cr, cb]);
        }
    }
    // No VIS header: exactly what forced mode exists for.
    let mut audio = slowrx::__test_support::mode_pd::encode_pd(SstvMode::Pd120, &ycrcb);
    // Clear the resampler group delay + the one-second forced-mode preroll.
    audio.extend(std::iter::repeat_n(
        0.0_f32,
        WORKING_SAMPLE_RATE_HZ as usize + 4096,
    ));

    let dir = std::env::temp_dir().join(format!("slowrx-cli-mode-{}", std::process::id()));
    let _ = fs::remove_dir_all(&dir);
    fs::create_dir_all(&dir).expect("create temp dir");
    let wav = dir.join("input.wav");
    {
        let wav_spec = hound::WavSpec {
            channels: 1,
            sample_rate: WORKING_SAMPLE_RATE_HZ,
            bits_per_sample: 16,
            sample_format: hound::SampleFormat::Int,
        };
        let mut writer = hound::WavWriter::create(&wav, wav_spec).expect("create WAV");
        for &s in &audio {
            writer
                .write_sample((s.clamp(-1.0, 1.0) * f32::from(i16::MAX)) as i16)
                .expect("write sample");
        }
        writer.finalize().expect("finalize WAV");
    }

    let out_dir = dir.join("out");
    Command::cargo_bin("slowrx-cli")
        .expect("binary built")
        .arg("--input")
        .arg(&wav)
        .arg("--output")
        .arg(&out_dir)
        .arg("--mode")
        .arg("PD-120")
        .arg("--quiet")
        .assert()
        .success();

    assert!(
        out_dir.join("img-001-pd120.png").is_file(),
        "expected img-001-pd120.png in {}",
        out_dir.display()
    );

    let _ = fs::remove_dir_all(&dir);
}
