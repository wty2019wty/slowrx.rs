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

/// End-to-end: `--start` and `--end` restrict forced decoding to a single
/// image (issue #114). Uses a short Robot 36 image so the test stays quick.
#[cfg(feature = "test-support")]
#[test]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss
)]
fn cli_window_flags_decode_robot36() {
    use slowrx::{SstvMode, WORKING_SAMPLE_RATE_HZ};

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
    let mut audio = slowrx::__test_support::mode_robot::encode_robot(SstvMode::Robot36, &ycrcb);
    audio.extend(std::iter::repeat_n(
        0.0_f32,
        WORKING_SAMPLE_RATE_HZ as usize + 4096,
    ));

    let dir = std::env::temp_dir().join(format!("slowrx-cli-window-{}", std::process::id()));
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

    let nominal = f64::from(h) * spec.line_seconds;
    let end_value = format!("{nominal}");
    for (label, flag, value) in [
        ("start", "--start", "0.0"),
        ("end", "--end", end_value.as_str()),
    ] {
        let out_dir = dir.join(format!("out-{label}"));
        Command::cargo_bin("slowrx-cli")
            .expect("binary built")
            .arg("--input")
            .arg(&wav)
            .arg("--output")
            .arg(&out_dir)
            .arg("--mode")
            .arg("robot36")
            .arg(flag)
            .arg(value)
            .arg("--quiet")
            .assert()
            .success();
        assert!(
            out_dir.join("img-001-robot36.png").is_file(),
            "expected img-001-robot36.png for {flag} in {}",
            out_dir.display()
        );
    }

    let _ = fs::remove_dir_all(&dir);
}

/// `--mode` without `--start`/`--end` must be rejected: a forced mode always
/// requires a time anchor (issue #114 follow-up).
#[test]
fn cli_mode_requires_window_flag() {
    Command::cargo_bin("slowrx-cli")
        .expect("binary built")
        .arg("--input")
        .arg("missing.wav")
        .arg("--output")
        .arg("out")
        .arg("--mode")
        .arg("pd120")
        .assert()
        .failure();
}
