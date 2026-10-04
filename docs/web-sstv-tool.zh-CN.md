# Web 端 SSTV 解码工具 — 方案设计文档

- 状态：草案（Draft）
- 日期：2026-10-04
- 适用范围：在浏览器中运行的 SSTV 解码工具，解码核心复用 `slowrx.rs`（编译为 WASM）
- 说明：本文档中的性能数据为**实测结果**，环境与方法见附录；浏览器（V8）端数字仍待确认

---

## 1. 背景与目标

在 Web 浏览器中构建一个 SSTV 解码工具：

- 解码核心使用 `slowrx`（本仓库），编译为 WebAssembly 在浏览器内运行；
- 支持从**文件**或**麦克风**获取音频；
- 支持**选取部分音频**送入解码，而非只能整段解码；
- 提供类似 Adobe Audition 的**音频频谱图**，用于可视化与时间选区交互；
- 解码结果在 Canvas 上渲染，可下载；
- **纯前端**，音频数据不上传服务器。

选择 Rust→WASM 的核心动机：相比纯 JS 方案，`slowrx` 在 PD / Robot / Scottie / Martin 等模式上的时序对齐与抗噪解码更可靠。

---

## 2. 需求梳理

| 编号 | 需求 | 优先级 |
|---|---|---|
| R1 | `slowrx` 编译为 WASM，流式解码 | 必须 |
| R2 | 支持 PD / Robot / Scottie / Martin 模式族 | 必须 |
| R3 | 文件 / 麦克风音频输入 | 必须 |
| R4 | 音频转单声道 `f32` PCM | 必须 |
| R5 | 不强制依赖 ffmpeg（优先 Web Audio API） | 必须 |
| R6 | 选取任意时间范围音频送入解码 | 必须 |
| R7 | Audition 风格频谱图（频率轴、颜色映射、可缩放/平移） | 必须 |
| R8 | 频谱图上时间选区的创建/调整/移动 | 必须 |
| R9 | 频谱图与选区视觉同步 | 必须 |
| R10 | 解码结果 Canvas 渲染 + 下载 | 必须 |
| R11 | 接收/解码中的状态反馈 | 应该 |
| R12 | 纯前端、不上传音频 | 必须 |
| R13 | 控制依赖体积与初始化开销 | 应该 |

---

## 3. 可行性结论（已实测验证）

**总体结论：可行。核心解码（Rust→WASM）无障碍；真正的约束是性能，且已量化、可控。**

### 3.1 编译验证

- `rustfft v6.4.1` 及其依赖链（`strength_reduce` / `transpose` / `primal-check` / `num-complex`）在 `wasm32-unknown-unknown` 下编译通过；
- `slowrx v0.5.3` 本体（默认 feature，不含 `cli`）同样通过；
- `wasm32-wasip1` 也可编译，用于离线性能实测。

> 结论：**编译不是风险点**。此前担心的 `rustfft` 在 wasm 上的 SIMD/`std::arch` 问题不存在。

### 3.2 性能实测

同机、同源码、warmup + 取中位数：

| 指标 | native | wasm 标量 | wasm SIMD128 | SIMD vs native |
|---|---|---|---|---|
| 256 点 FFT | 0.192 µs | 1.147 µs | 0.491 µs | 2.6× 慢 |
| PD120 整图 | 2.35 s | 14.4 s | 5.77 s | 2.5× 慢 |
| PD180 整图 | ~2.9 s | 18.1 s | 7.53 s | 2.6× 慢 |
| PD240 整图 | ~3.5 s | 21.7 s | 9.25 s | 2.6× 慢 |

结论：

- 标量 wasm 约为 native 的 **6 倍**耗时；
- 开启 `wasm_simd` 后约 **2.4 倍提速**，差距缩到 **2.5–2.6 倍**；
- 文件体积：wasi 产物约 **443 KB**（未 `wasm-opt`，含基准 main）。

**FFT 数量精确核算**（`src/demod.rs`：`FFT_LEN = 1024`、`PIXEL_FFT_STRIDE = 1`，即每个工作采样点一次 1024 点 FFT；PD 族 640×496、248 行对、每对 4 通道）：

| 模式 | 每图 FFT 数 | native 每 FFT | wasm SIMD 每 FFT |
|---|---|---|---|
| PD120 | ~2.35 M | ~1.00 µs | ~2.46 µs |
| PD180 | ~3.02 M | ~0.96 µs | ~2.50 µs |
| PD240 | ~3.69 M | ~0.96 µs | ~2.51 µs |

三种模式与裸 FFT 的 native/wasm 比值高度一致（≈6×），说明**整条解码管线几乎 100% 是 FFT 时间**，其余环节（重采样、SNR、幅度提取）可忽略。

---

## 4. 关键发现与设计约束

以下约束直接来自 `slowrx` 的真实实现，是架构设计的硬前提。

### 4.1 解码是“两遍式”，没有逐行实时输出

`SstvDecoder::process`（`src/decoder.rs:537`）在 `Decoding` 状态下先累积**约一整张图**的音频（`target_audio_samples`），缓冲满后才一次性跑 `find_sync` + 爆发式解码所有行，`SstvEvent::LineDecoded` 与 `ImageComplete` 在同一批事件里返回；`partial` 恒为 `false`。

**影响：**

- 不能做“边收边画”的实时预览；
- “接收中…”只能是**定时器进度**，不是真实逐行进度；
- 单次解码是计算突发，**必须放 Web Worker**，否则主线程卡死。

### 4.2 强制模式必须携带解码窗口

`SstvDecoder::with_mode`（`src/decoder.rs:435`）要求 `mode` + `DecodeWindow`（`starting_at` / `ending_at`）。窗口时间为**相对“喂入流的第一帧”**的秒数。

- 无 VIS 头、只框住图像体的选区 → 必须强制模式 + 锚点；
- 选区含 VIS 头 → 可用自动识模 `SstvDecoder::new`。

### 4.3 无 flush/收尾接口

库没有公开的“结束输入”API。喂完选区若差几帧凑不满 `target_audio_samples`，可能不触发解码。

**对策：** 每次送入解码的切片末尾**补 0.5–1 s 静音**再喂给解码器。

### 4.4 内部自带重采样

`Resampler`（`src/resample.rs`）接受 0–192 kHz 任意输入率，内部重采样到 11 025 Hz。因此**不必强制转 44.1 kHz**，直接把 `AudioContext.sampleRate` 传给解码器即可，省一次重采样。

### 4.5 性能开关

- 必须启用 rustfft 的 `wasm_simd` feature，并以 `-C target-feature=+simd128` 编译；
- `PIXEL_FFT_STRIDE = 1`（`src/demod.rs:441`）是本移植项目为提高精确度而定的策略（原 slowrx 用稀疏 FFT）。若性能仍不足，可评估调大 stride 换取线性提速，但会影响解码保真度，仅作最后手段。

---

## 5. 总体架构

```
┌──────────────────────────── 主线程 (UI) ────────────────────────────┐
│  文件/麦克风音频输入 → 解码为单声道 f32 (Web Audio API)             │
│  频谱图渲染 + 时间选区交互 (Canvas)                                 │
│  解码结果渲染 (Canvas) / 下载                                       │
└───────────────┬─────────────────────────────────────────────────────┘
                │ postMessage(transferable Float32Array)
                ▼
┌──────────────────────────── Web Worker ─────────────────────────────┐
│  slowrx-wasm (wasm-bindgen)：SstvDecoder 流式喂入                    │
│  末尾补静音 → 收集 LineDecoded / ImageComplete                       │
│  (可选) STFT 计算，供频谱图使用（复用 rustfft）                       │
└─────────────────────────────────────────────────────────────────────┘
```

设计要点：

- **Worker 负责吃 CPU**：解码与（可选）STFT；
- **不引入 SharedArrayBuffer / 线程**：用 `postMessage` + `transfer` 传递 `Float32Array` 即可，避免 COOP/COEP 部署复杂度；
- **不依赖 ffmpeg.wasm**：常见格式走 Web Audio API。

---

## 6. 模块设计

### 6.1 `slowrx-wasm` 包装 crate

- **不修改**已发布的 `slowrx` 纯库接口；新建独立 crate，`crate-type = ["cdylib", "rlib"]`，依赖 `slowrx`；
- 启用 SIMD：在该 crate 的依赖中启用 `rustfft` 的 `wasm_simd` feature（cargo feature 统一），并配 `RUSTFLAGS = -C target-feature=+simd128`；
- 通过 `#[wasm_bindgen]` 暴露：
  - 构造解码器（输入采样率）；
  - `push_audio(&[f32]) -> 事件`（或分块推入 + 取事件）；
  - 图片输出为 RGBA `Uint8Array`（`SstvImage` 是 `Vec<[u8;3]>`，在 Rust 侧转 RGBA 更省事）；
- 建议给 `slowrx` 增补一个 `wasm` feature 转发到 `rustfft/wasm_simd`，避免业务侧手动协调 feature。

### 6.2 音频输入

- 文件：`decodeAudioData` 支持 mp3/aac/ogg/flac/wav；
- 麦克风：`getUserMedia`（需 HTTPS/localhost）→ `AudioWorklet` 抓 `f32` 块；
- 单声道化：对各 `AudioBuffer.getChannelData` 求平均；
- 采样率：直接把浏览器采样率传给 `SstvDecoder::new`（见 4.4）；
- 兜底：仅在浏览器不支持的格式/复杂转换时才考虑 ffmpeg.wasm（默认不引入）。

### 6.3 频谱图

- 在 Worker 内计算 STFT（**复用 wasm 内 rustfft**，与解码器同源，避免重复 FFT 实现与额外 JS 依赖）；
- 主线程只渲染**可视时间窗口**，分块缓存结果，避免大文件全量渲染；
- 频率轴刻度、颜色映射（灰度/彩色）、时间-频率强度用 Canvas 2D 足够；无需 WebGL；
- 不优先采用 `wavesurfer.js + Spectrogram` 插件：其缩放/平移/虚拟化受限，且 FFT 实现与 wasm 侧重复。

### 6.4 选区交互

- 在 overlay Canvas 上实现时间范围选区的拖拽创建、移动、两端缩放；
- 频谱图与选区共用同一时间轴 scale，保证同步；
- 需求只要求时间范围精确，不要求 Audition 的频域框选，复杂度可控。

### 6.5 选区 → 解码的语义

| 选区情形 | 解码方式 |
|---|---|
| 含 VIS 头 | `SstvDecoder::new(rate)` 自动识模 |
| 只含图像体 | 用户指定模式 + `DecodeWindow::starting_at(0)` |
| 多张图连续 | 流式喂入，多图事件依次产出 |

统一步骤：截取选区 PCM → 末尾补静音（见 4.3）→ 新建解码器（时间轴以选区起点为 0）→ 喂入 → 收集事件。

### 6.6 结果渲染

- `ImageComplete.image` → RGBA → `ImageData` → Canvas；
- 下载：`canvas.toBlob()`；
- 状态反馈：解码期间显示“解码中…”（进度为估算）。

---

## 7. 关键接口（草案）

Rust 侧（`slowrx-wasm`）：

```rust
#[wasm_bindgen]
pub struct WasmDecoder { /* SstvDecoder + rate */ }

#[wasm_bindgen]
impl WasmDecoder {
    #[wasm_bindgen(constructor)]
    pub fn new(sample_rate_hz: u32) -> Result<WasmDecoder, JsValue>;

    /// 推入一段单声道 f32，返回本批产生的事件（JSON 或结构化对象）。
    pub fn push_audio(&mut self, samples: &[f32]) -> JsValue;

    /// 以指定模式 + 窗口强制解码（选区不含 VIS 头时）。
    pub fn force_mode(&mut self, mode: &str, start_secs: Option<f64>, end_secs: Option<f64>);
}
```

---

## 8. 风险与对策

| 风险 | 等级 | 对策 |
|---|---|---|
| wasm 解码耗时长（PD180/240 达 7–9 s） | 高 | 必开 SIMD；放 Worker；进度反馈；预期管理 |
| 强制模式/窗口语义误用（选区不含 VIS） | 中 | UI 明确“自动识模 / 指定模式”两种路径 |
| 末尾缺静音导致不触发解码 | 中 | 统一补 0.5–1 s 静音 |
| “实时逐行预览”无法实现 | 中 | 产品层改为“收完显图”，进度为估算 |
| 工具链/目标不一致（默认 stable 缺 wasip1/wasm std） | 低 | 在工程内固定 `rust-toolchain.toml`（1.95.0）并显式 `+1.95.0` |
| 大文件频谱图内存/性能 | 中 | 可视窗口渲染 + 分块缓存 |
| V8 与 Cranelift 性能差异 | 中 | 阶段 2 浏览器实测确认 |

---

## 9. 实施里程碑

| 里程碑 | 内容 | 验证标准 |
|---|---|---|
| M0（已完成） | 编译验证 | `rustfft` + `slowrx` 在 wasm 编译通过 |
| M1（已完成） | 性能基线实测 | 得到 native / wasm 标量 / wasm SIMD 数据 |
| M2 | `slowrx-wasm` 包装 crate + Worker | Worker 内解码一张合成分片出图 |
| M3 | 音频输入 + 选区 + 补静音 | 选区含 VIS 自动识模出图 |
| M4 | 强制模式路径 | 选区不含 VIS 也能出图 |
| M5 | 频谱图 + 选区交互 | 频谱可视 + 选区同步 + 触发解码 |
| M6 | 结果渲染/下载/状态 | 端到端可用 |
| M7（可选） | 浏览器实测 + 性能收尾 | V8 数字确认，必要时调 stride |

---

## 10. 未决问题 / 待验证

1. **浏览器 V8 实测**：当前数据来自 wasmtime(Cranelift)，需阶段 2 在 Edge/Chrome 确认；
2. **`wasm_simd` 兼容性**：目标浏览器需支持 fixed-width SIMD（现代浏览器普遍支持）；
3. **STFT 是否复用 wasm 内 rustfft**：需评估额外 API 暴露与内存成本；
4. **`PIXEL_FFT_STRIDE` 是否调整**：仅在性能仍不足时评估，涉及保真度；
5. **短模式（Robot36 等）实测耗时**：预计约 2–3 s，待实测确认。

---

## 附录 A：基准复现方法

- 基准 crate（仓库外，不污染主库）：临时目录下的独立 crate，依赖 `slowrx`（`features = ["test-support"]`，用合成编码器造 PD 音频）与 `rustfft`；
- native：`cargo +1.95.0 run --release`；
- wasm 标量：`cargo +1.95.0 build --release --target wasm32-wasip1` + `wasmtime run <wasm> 5 1`；
- wasm SIMD：`rustfft` 启用 `wasm_simd`，`RUSTFLAGS='-C target-feature=+simd128'` 后同上；
- 方法：warmup + 多轮取中位数；构建配置 `lto = true`、`opt-level = 3`、`codegen-units = 1`。

## 附录 B：环境记录

- rustc / cargo：1.95.0（`x86_64-pc-windows-gnu`）；
- 目标：`wasm32-unknown-unknown`、`wasm32-wasip1`；
- 运行时：wasmtime 49.0.2（Cranelift）；
- `rustfft` 6.4.1（features：`avx`/`sse`/`neon` 默认；`wasm_simd` 需显式启用）；
- 注意：仓库根有 `rust-toolchain.toml` 固定 1.95.0；在无该文件的新工程中，默认 stable 不含 wasm std，需 `+1.95.0`。
