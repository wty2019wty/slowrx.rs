# 手动解码窗口（强制模式，issue #113 / #114）

本文档说明 slowrx.rs 在**强制指定 SSTV 模式**解码时，如何用调用方给定的时刻精确
锁定解码窗口。文档为中文说明，代码标识符、API 名称与命令行参数保持英文。

相关实现：`src/decoder.rs`、`src/bin/slowrx_cli.rs`；对应 issue #113、#114。

---

## 1. 背景与现状

issue #113 最初让强制模式（forced mode）通过"搜索周期性同步脉冲串"自动捕获图像：
扫描一串按行距排列的 1200 Hz 同步脉冲，找到后把窗口锚定到该脉冲串的第 0 行。

这种方式无法表达"图就在这里"，在以下场景并不合适：

- 一段长录音中有多张图，只想解其中某一张；
- 已知图像大致位置，不希望解码器在整段音频里自行搜索；
- 同步脉冲较弱或缺失，但调用方已经从别的途径知道图像位置。

issue #114 引入**手动解码窗口**（manual decode window），并把它确立为强制模式的
唯一形态：**指定模式时必须同时给出 `--start` 或 `--end` 之一**。库层不再提供
"只指定模式、全流自动扫描"的入口。

锚点即权威：解码器**不会**再去搜索图像从哪里开始。锚点处若有 VIS 头则吸收其
时长；**找不到 VIS 就直接按锚点解码**（把锚点当作第 0 行）。同步脉冲串搜索相关
的自动定位逻辑已移除。

---

## 2. 核心语义

### 2.1 时刻基准与单位

- 时刻单位为**秒**，基准是**喂给解码器的第一个采样点**（即输入录音时间线）。
- 内部会换算成工作采样率（11025 Hz）样本数。重采样器的群延迟（约几十个输入
  采样，毫秒级）由 `find_sync` 在解码时吸收，不影响结果。

### 2.2 只需一个端点

提供开始或结束其中之一即可（CLI 用 ArgGroup 限制最多一个）：

- 给了开始：窗口起点 = 开始时刻，长度 = 模式的**标称图像时长**；
- 给了结束：窗口终点 = 结束时刻，起点 = 结束时刻 − 模式的**标称图像时长**。

调用方不需要自己计算另一个端点。

### 2.3 `--start` 的指向口径

`--start` 允许指向两种位置，二者都能解出完整图像：

1. **传输起点**（含 VIS 头）：解码器会在锚点处探测 VIS 头，命中后**吸收**该头部
   （跳过其时长），把图像第 0 行钉在头部之后；
2. **图像第一行起点**：锚点本身即第 0 行。

### 2.4 `--end` 的指向口径

`--end` 指向**图像数据末尾**（不含尾部间隙）。起点由标称时长反推。

### 2.5 长度口径：VIS 不计入图像长度

以 Robot 36 为例：

- **36.0 s** 是图像数据本身；
- **约 36.9 s** 才是"图像 + VIS 头"。

因此：**锚点若指向传输起点（含 VIS），必须把 VIS 头吸收掉，而解码长度仍按图像
数据的标称时长计算**，否则 Robot 36 最后约 0.9 秒的图像会被切掉。手动窗口正是
这样处理的（见 4.2 的 `ManualProbe`）。

### 2.6 优先级与强制约束

- **强制模式必须带窗口**：`--mode` 必须搭配 `--start` 或 `--end`；
- 窗口内仍会运行一次 `find_sync` 做精确的第 0 行 / 斜率校正（这是解码本身必需的
  对齐步骤，不是"重新猜位置"）；
- 未设置强制模式时，走自动 VIS 检测路径。

---

## 3. 库层 API

### 3.1 `DecodeWindow`

```rust
use slowrx::DecodeWindow;

// 锚定图像第一行（或 VIS 头之前的传输起点）。
let w = DecodeWindow::starting_at(12.5);

// 锚定图像数据末尾；起点 = 12.5 - 标称图像时长。
let w = DecodeWindow::ending_at(48.5);
```

`DecodeWindow` 为 `#[non_exhaustive]`，公开字段：

| 字段 | 含义 |
|---|---|
| `start_secs: Option<f64>` | 图像 / 传输起点（秒） |
| `end_secs: Option<f64>` | 图像数据末尾（秒） |

### 3.2 解码器构造与设置

```rust
use slowrx::{DecodeWindow, SstvDecoder, SstvMode};

// 构造：模式 + 窗口一起给出。
let mut decoder = SstvDecoder::with_mode(
    44_100,
    SstvMode::Robot36,
    DecodeWindow::starting_at(12.5),
)?;

// 运行时重新指定模式 + 窗口。
decoder.set_forced_mode(SstvMode::Robot36, DecodeWindow::ending_at(48.5));

// 清除强制模式，恢复自动 VIS 检测。
decoder.clear_forced_mode();

// 读回状态。
assert_eq!(decoder.forced_mode(), None);
assert!(decoder.decode_window().is_none());
```

模式与窗口始终成对出现：没有"只有模式、没有窗口"的状态。设置时会丢弃在途图像，
并以新的状态重新开始。

### 3.3 停止与重置

- 强制窗口**只解一张图**，随后停止；即使后面还有音频也不会继续解。
- 想再解一张，需要重新 `set_forced_mode(...)`，或调用 `reset()`。

---

## 4. 解码流程

### 4.1 输入裁剪

`process()` 在最前端按窗口裁剪输入音频：

1. **跳过锚点之前**的采样（`manual_skip_input`）；
2. **限制锚点之后的喂入总量**为一个硬上限
   （`manual_feed_budget` = `MANUAL_VIS_PROBE_SECONDS` + 标称图像时长 + 尾部余量）。

第 2 步用于给窗口封顶：若锚点给错，解码器也不会一路扫到后面另一张图上。

### 4.2 状态机

窗口进入以下阶段：

```
ManualProbe ──(探测到 VIS -> 吸收头部)──> Collecting ──> find_sync + 逐行解码
     │
     └──(无 VIS)──────────────────────> Collecting ──> find_sync + 逐行解码
```

- **`ManualProbe`**：对锚点起的约 1.1 s 前缀运行一次 VIS 检测。
  - **命中**：吸收 VIS 头（按整探针数丢弃，保持 `has_sync` 与 `audio` 对齐），
    之后第 0 行即头部末端。
  - **未命中**：**直接把锚点当作第 0 行**，不做任何搜索。
- **`Collecting`**：累积"一张图 + 尾部余量"后运行 `find_sync` 并逐行解码。

### 4.3 第 0 行对齐

VIS 路径从停止位开始、强制窗口从锚点（吸收可选 VIS 头之后）开始，两者都已固定
line 0 的相位，因此直接使用 `find_sync` 返回的行内 `skip`（兼容 Scottie 的
**行中同步**），不再做绝对整行吸附。

### 4.4 同步门限

窗口内**完全检测不到同步脉冲**时（`find_sync` 返回 `slant_deg == None`）不出图，
避免把静音 / 噪声渲染成黑图。

### 4.5 事件

- 不发出 `VisDetected`（模式由强制参数决定，VIS 码不可信）。
- 依旧发出 `LineDecoded` 与 `ImageComplete`。
- 强制模式下 `hedr_shift_hz` 视为 0（未从 VIS 提取频率偏移）。

---

## 5. CLI 用法

```text
--mode  <MODE>     指定模式；必须搭配 --start 或 --end
--start <SECONDS>  解出开始时刻（秒）处的单张图像
--end   <SECONDS>  解出图像数据在此时刻（秒）结束的单张图像
```

```bash
# 解 12.5 s 处的一张 Robot 36（若此处有 VIS 头会被吸收）。
slowrx-cli --input recording.wav --output ./out --mode robot36 --start 12.5

# 解图像数据在 48.5 s 结束的一张 Robot 36（长度取标称 36 s）。
slowrx-cli --input recording.wav --output ./out --mode robot36 --end 48.5
```

约束：

- `--mode` 由 clap 强制要求 `--start` 或 `--end`；只给模式会报错；
- `--start` 与 `--end` 互斥（ArgGroup），最多给一个；
- `--start` / `--end` 由 clap 强制要求 `--mode`；
- 时刻必须为非负数；
- 仍需保证录音在窗口末端之后有足够音频（窗口需要"一张图 + 余量"才能完成解码）。

---

## 6. 边界与注意事项

| 情形 | 行为 |
|---|---|
| VIS 头存在且可解析 | 吸收头部，长度按图像数据标称时长 |
| VIS 头缺失 / 校验损坏 | 直接把锚点当作图像起点解码（不搜索） |
| 窗口内无同步脉冲 | 不出图（保留同步门限） |
| 锚点给错、窗口内无可识别图像 | 受 `manual_feed_budget` 约束，不会扫到后续图像 |
| 一次解多张 | 不支持；窗口只解一张，之后停止 |
| 发射机时钟漂移 | `--end` 依赖标称时长反推起点；若时钟误差超过约一行，顶部若干行可能偏移（`--start` 锚在图像首行时不受此影响） |
| 锚点在录音中已播放过的位置之后才设置 | 无法回退，通常解不出图；应在喂入该时刻之前设置窗口 |
| 窗口后音频不足 | 不会输出图像；请确保录音含足够尾部余量 |

---

## 7. 测试覆盖

`tests/forced_mode.rs`：

- `manual_start_at_image_start_decodes_pd120`：图像首行锚点；
- `manual_start_at_transmission_start_absorbs_vis`：传输起点吸收 VIS，不切尾；
- `manual_end_decodes_pd120`：`--end` 口径；
- `manual_window_selects_the_anchored_image`：两张图中精确选中第二张；
- `manual_window_stops_after_one_image`：只解一张；
- `manual_start_robot36_absorbs_vis_to_full_36s`：Robot 36 完整 36 s；
- `manual_window_decodes_scottie_absorbing_vis`：Scottie 行中同步 + VIS 吸收；
- `manual_window_on_silence_emits_nothing`：静音不出图；
- `clear_forced_mode_restores_vis_detection`：清除强制模式后恢复 VIS 检测。

`tests/cli.rs`：

- `cli_window_flags_decode_robot36`：`--start` 与 `--end` 端到端；
- `cli_mode_requires_window_flag`：只给 `--mode` 会被拒绝。

---

## 8. 兼容性

issue #113 / #114 均在 `[Unreleased]` 中，尚未发布。本次调整把"窗口"设为强制模式的
必需参数，并去掉未发布的冗余 API（`set_decode_window`、`DecodeWindow::between`）。
模式仍通过原有名字指定，只是必须同时给出窗口：

- `SstvDecoder::with_mode(rate, mode, window)` —— 构造并指定模式 + 窗口；
- `SstvDecoder::set_forced_mode(mode, window)` —— 运行时重新指定；
- `SstvDecoder::clear_forced_mode()` —— 清除，恢复自动 VIS 检测；
- `SstvDecoder::forced_mode()` / `decode_window()` —— 读回状态。

对**已发布**接口无影响；`SstvMode`、`SstvEvent` 等公开类型签名未改动。
