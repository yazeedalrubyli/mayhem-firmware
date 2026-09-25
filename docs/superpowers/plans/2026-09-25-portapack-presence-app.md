# PortaPack Presence App Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a Mayhem external app "Presence" that shows EMPTY / PRESENT / MOVING / NO SIGNAL on the H4M from received-power fluctuations, built from this fork, flashed and verified on the owner's device over USB serial.

**Architecture:** A pure integer-only detector (`presence_dsp`) in `firmware/application/external/presence/` is fed by the stock capture baseband through two selectable sources (PEAK = `ChannelStatistics.max_db` at 10 Hz, MEAN = spectrum-bin mean at 50 Hz); a `PresenceView` draws a status banner, two score bars and a power trace with three small custom widgets. Host doctest tests cover the detector; the firmware is built in the `portapack-dev` Docker image and deployed with a pyserial tool that uploads, flashes, replaces `/APPS` and takes screenshots.

**Tech Stack:** C++17 (ARM GCC 9.2.1 in Docker for the device, g++ 13 on the host for tests), doctest (`firmware/test/include/doctest.h`), CMake + ninja, Python 3 + pyserial 3.5 (`/home/y/miniconda3/bin/python3`) for the serial tool.

**Spec:** `docs/superpowers/specs/2026-09-25-portapack-presence-app-design.md`

## Global Constraints

- Trunk `main` of `yazeedalrubyli/mayhem-firmware`, cut from upstream tag `v2.4.0`; every task commits to `main` and pushes (`git push origin main`). Commits authored as `Yazeed Alrubyli <yazeed.alrubyli@gmail.com>` and end with `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.
- No edits under `firmware/baseband/`, `firmware/common/`, or any other app. Only `external.cmake`, `external.ld`, the new `external/presence/` directory, `tools/presence/`, and `docs/superpowers/`.
- `m4_app_tag {'P','C','A','P'}`; the app uses `baseband::run_image(portapack::spi_flash::image_tag_capture)` and nothing else.
- Linker slot `ram_external_app_presence (rwx) : org = 0xAE040000, len = 32k` (0xAE000000 is taken by mdc_tx).
- `presence.ppma` (app code + 14,844-byte capture image) must be ≤ 32,768 bytes; `export_external_apps.py` fails the build otherwise.
- Everything in `namespace ui::external_app::presence` so the linker pattern `*ui*external_app*presence*` captures all code, including the detector.
- Integer arithmetic only in `presence_dsp` (no float, no heap after construction, no firmware headers).
- Units: power and scores in centi-dB (`cdb`, 1/100 dB). PEAK: `max_db * 100`. MEAN: `20 * sum(bins 4..95, 160..251) / 184 - 5100`.
- Default frequency on first run 5,560,000,000 Hz; bandwidth options {750k, 1.25M, 2.5M, 5M}, default index 2.
- Real device only: the deploy and verify steps talk to `/dev/ttyACM0` (Mayhem serial console, 115200); no simulated device.
- Formatting: run `./tools/clang-format.sh -i <files>` on every new or changed C++ file before committing (repo-pinned clang-format 18.1.8).

## Review Focus

1. **Serial-shell traffic while the app runs** — `screenframeshort` and `applist` must not corrupt the detector or the display; pinned by Task 4 step 5 (screenshot taken while the app is live, then a second screenshot 5 s later still shows a valid screen and advancing rate/power numbers).
2. **Bandwidth change while MEAN streaming** — changing `field_bw` mid-stream must not stall the spectrum FIFO; pinned by Task 4 step 6 (switch to MEAN, change bw, screenshot shows the measured rate near 50 Hz).
3. **Settings file from an older version with out-of-range values** — a `bw_index` of 9 or `source` of 7 must not index past the option lists; pinned by Task 2's constructor code (clamps) and by the test in Task 1 that `configure` clamps `rate_hz` and `hold_n`.
4. **Very large power steps** (antenna plugged while running, gain change) must not overflow the squared-deviation windows; pinned by Task 1 test "clamped deviations never overflow".
5. **Calibration pressed twice / pressed during calibration** must restart cleanly and end with valid thresholds; pinned by Task 1 test "calibration restarted mid-way still finishes".

---

### Task 1: The detector (`presence_dsp`) with host tests

**Files:**
- Create: `firmware/application/external/presence/presence_dsp.hpp`
- Create: `firmware/application/external/presence/presence_dsp.cpp`
- Create: `firmware/application/external/presence/test/test_presence_dsp.cpp`
- Create: `firmware/application/external/presence/test/run.sh`

**Interfaces:**
- Consumes: nothing from the firmware.
- Produces (used by Task 2): `ui::external_app::presence::{State, Config, Output, Detector, default_thr_motion_cdb, default_thr_still_cdb}`; `Detector(const Config&)`, `void configure(const Config&)`, `void reset()`, `void start_calibration()`, `void set_thresholds(int32_t motion_cdb, int32_t still_cdb)`, `const Output& push(int32_t power_cdb)`, `const Output& output() const`, `const Config& config() const`, `static uint8_t ema_shift(uint16_t rate_hz, uint16_t tau_ms)`.

- [ ] **Step 1: Write the test runner**

`firmware/application/external/presence/test/run.sh`:

```bash
#!/usr/bin/env bash
# Builds and runs the host tests for presence_dsp with the native g++.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../../.." && pwd)"
OUT="${TMPDIR:-/tmp}/presence_dsp_test"
g++ -std=c++17 -O1 -Wall -Wextra -Werror \
    -I "$ROOT/firmware/test/include" -I "$HERE/.." \
    "$HERE/test_presence_dsp.cpp" "$HERE/../presence_dsp.cpp" -o "$OUT"
"$OUT" "$@"
```

`chmod +x firmware/application/external/presence/test/run.sh`

- [ ] **Step 2: Write the failing tests**

`firmware/application/external/presence/test/test_presence_dsp.cpp`:

```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "presence_dsp.hpp"

using namespace ui::external_app::presence;

namespace {

const Output& feed_constant(Detector& d, size_t n, int32_t value) {
    for (size_t i = 0; i < n; i++) d.push(value);
    return d.output();
}

// 1 Hz square wave at 10 Hz: 5 samples high, 5 samples low.
void feed_square_10hz(Detector& d, size_t n, int32_t base, int32_t amplitude, bool& saw_moving) {
    for (size_t i = 0; i < n; i++) {
        const int32_t v = base + (((i / 5) % 2) ? amplitude : -amplitude);
        if (d.push(v).state == State::Moving) saw_moving = true;
    }
}

struct Lcg {
    uint32_t s;
    uint32_t next() {
        s = s * 1664525u + 1013904223u;
        return s >> 16;
    }
};

}  // namespace

TEST_CASE("ema_shift picks the nearest power of two samples per tau") {
    CHECK(Detector::ema_shift(10, 350) == 2);    // 3.5 -> 4 samples
    CHECK(Detector::ema_shift(10, 2800) == 5);   // 28 -> 32
    CHECK(Detector::ema_shift(50, 350) == 4);    // 17 -> 16
    CHECK(Detector::ema_shift(50, 2800) == 7);   // 140 -> 128
    CHECK(Detector::ema_shift(1, 350) == 0);     // 0 samples -> no smoothing
}

TEST_CASE("before any sample the output is NoSignal") {
    Detector d{Config{10, 1, -9000}};
    CHECK(d.output().state == State::NoSignal);
    CHECK(d.output().thr_motion_cdb == default_thr_motion_cdb);
    CHECK(d.output().thr_still_cdb == default_thr_still_cdb);
}

TEST_CASE("constant input settles to Empty with zero scores") {
    Detector d{Config{10, 1, -9000}};
    const auto& o = feed_constant(d, 100, -4000);
    CHECK(o.state == State::Empty);
    CHECK(o.motion_cdb == 0);
    CHECK(o.still_cdb == 0);
    CHECK(o.power_cdb == -4000);
    CHECK(o.confidence_pct == 100);
}

TEST_CASE("power below min_power_cdb is NoSignal and recovers") {
    Detector d{Config{10, 1, -7000}};
    const auto& o = feed_constant(d, 50, -8000);
    CHECK(o.state == State::NoSignal);
    CHECK(o.confidence_pct == 0);
    feed_constant(d, 100, -6000);
    CHECK(d.output().state != State::NoSignal);
}

TEST_CASE("a 1 Hz square wave of +-600 cdB is Moving with full confidence") {
    Detector d{Config{10, 1, -9000}};
    feed_constant(d, 50, -4000);
    bool saw_moving = false;
    feed_square_10hz(d, 40, -4000, 600, saw_moving);
    CHECK(d.output().state == State::Moving);
    CHECK(d.output().confidence_pct == 100);
}

TEST_CASE("a slow 0.3 Hz +-25 cdB triangle after calibration is Present and never Moving") {
    Detector d{Config{50, 1, -9000}};
    feed_constant(d, 100, -4000);
    d.start_calibration();
    feed_constant(d, 13 * 50, -4000);
    REQUIRE(d.output().state == State::Empty);
    CHECK(d.output().thr_motion_cdb == 20);
    CHECK(d.output().thr_still_cdb == 5);
    bool saw_moving = false;
    for (size_t i = 0; i < 1000; i++) {
        const int32_t phase = static_cast<int32_t>(i % 166);
        const int32_t tri = phase < 83 ? (-25 + (50 * phase) / 83) : (25 - (50 * (phase - 83)) / 83);
        if (d.push(-4000 + tri).state == State::Moving) saw_moving = true;
    }
    CHECK_FALSE(saw_moving);
    CHECK(d.output().state == State::Present);
    CHECK(d.output().confidence_pct > 0);
}

TEST_CASE("cal_remaining counts down through both phases") {
    Detector d{Config{10, 1, -9000}};
    feed_constant(d, 20, -4000);
    d.start_calibration();
    CHECK(d.output().state == State::Calibrating);
    CHECK(d.output().cal_remaining == 130);
    d.push(-4000);
    CHECK(d.output().cal_remaining == 129);
    CHECK(d.output().state == State::Calibrating);
    feed_constant(d, 49, -4000);
    CHECK(d.output().cal_remaining == 80);
    feed_constant(d, 79, -4000);
    CHECK(d.output().cal_remaining == 1);
    CHECK(d.output().state == State::Calibrating);
    d.push(-4000);
    CHECK(d.output().cal_remaining == 0);
    CHECK(d.output().state == State::Empty);
}

TEST_CASE("calibration raises thresholds above quantised noise and the same noise stays Empty") {
    Detector d{Config{10, 1, -9000}};
    Lcg rng{12345};
    auto noise = [&rng]() { return -4000 - 100 * static_cast<int32_t>(rng.next() & 1u); };
    for (size_t i = 0; i < 100; i++) d.push(noise());
    d.start_calibration();
    for (size_t i = 0; i < 130; i++) d.push(noise());
    REQUIRE(d.output().state != State::Calibrating);
    CHECK(d.output().thr_motion_cdb >= 100);
    CHECK(d.output().thr_still_cdb >= 5);
    bool false_positive = false;
    for (size_t i = 0; i < 600; i++) {
        const auto s = d.push(noise()).state;
        if (s == State::Moving || s == State::Present) false_positive = true;
    }
    CHECK_FALSE(false_positive);
}

TEST_CASE("hold_n 2 removes a periodic one-sample dip that hold_n 1 reports as Moving") {
    bool moving_with_hold_1 = false;
    bool moving_with_hold_2 = false;
    Detector d1{Config{10, 1, -9000}};
    Detector d2{Config{10, 2, -9000}};
    for (size_t i = 0; i < 300; i++) {
        const int32_t v = (i % 50 == 0) ? -5000 : -4000;
        if (d1.push(v).state == State::Moving) moving_with_hold_1 = true;
        if (d2.push(v).state == State::Moving) moving_with_hold_2 = true;
    }
    CHECK(moving_with_hold_1);
    CHECK_FALSE(moving_with_hold_2);
}

TEST_CASE("Moving persists through the hold time after motion stops, then decays") {
    Detector d{Config{10, 1, -9000}};
    feed_constant(d, 50, -4000);
    bool saw_moving = false;
    feed_square_10hz(d, 60, -4000, 600, saw_moving);
    REQUIRE(d.output().state == State::Moving);
    feed_constant(d, 15, -4000);
    CHECK(d.output().state == State::Moving);
    feed_constant(d, 45, -4000);
    CHECK(d.output().state != State::Moving);
    feed_constant(d, 150, -4000);
    CHECK(d.output().state == State::Empty);
}

TEST_CASE("set_thresholds restores persisted values and Moving has priority over Present") {
    Detector d{Config{10, 1, -9000}};
    d.set_thresholds(1000, 100);
    CHECK(d.output().thr_motion_cdb == 1000);
    CHECK(d.output().thr_still_cdb == 100);
    feed_constant(d, 50, -4000);
    bool saw_moving = false;
    feed_square_10hz(d, 100, -4000, 600, saw_moving);
    CHECK_FALSE(saw_moving);
    CHECK(d.output().state == State::Present);
    d.set_thresholds(500, 100);
    feed_square_10hz(d, 30, -4000, 600, saw_moving);
    CHECK(d.output().state == State::Moving);
    d.set_thresholds(1, 1);
    CHECK(d.output().thr_motion_cdb == 20);
    CHECK(d.output().thr_still_cdb == 5);
}

TEST_CASE("configure keeps thresholds, clears scores, and clamps rate and hold") {
    Detector d{Config{10, 1, -9000}};
    d.set_thresholds(300, 60);
    bool saw_moving = false;
    feed_square_10hz(d, 60, -4000, 600, saw_moving);
    d.configure(Config{50, 1, -9000});
    CHECK(d.output().state == State::NoSignal);
    CHECK(d.output().motion_cdb == 0);
    CHECK(d.output().thr_motion_cdb == 300);
    CHECK(d.output().thr_still_cdb == 60);
    d.configure(Config{0, 0, -9000});
    CHECK(d.config().rate_hz == 1);
    CHECK(d.config().hold_n == 1);
    d.configure(Config{500, 9, -9000});
    CHECK(d.config().rate_hz == 50);
    CHECK(d.config().hold_n == 4);
}

TEST_CASE("clamped deviations never overflow on huge steps") {
    Detector d{Config{50, 1, -20000}};
    for (size_t i = 0; i < 500; i++) d.push((i % 2) ? 0 : -12000);
    CHECK(d.output().state == State::Moving);
    CHECK(d.output().motion_cdb > 0);
    CHECK(d.output().motion_cdb <= 12000);
    CHECK(d.output().still_cdb <= 12000);
}

TEST_CASE("calibration restarted mid-way still finishes with valid thresholds") {
    Detector d{Config{10, 1, -9000}};
    feed_constant(d, 20, -4000);
    d.start_calibration();
    feed_constant(d, 30, -4000);
    d.start_calibration();
    CHECK(d.output().cal_remaining == 130);
    feed_constant(d, 130, -4000);
    CHECK(d.output().state == State::Empty);
    CHECK(d.output().thr_motion_cdb == 20);
    CHECK(d.output().thr_still_cdb == 5);
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `firmware/application/external/presence/test/run.sh`
Expected: compile error `presence_dsp.hpp: No such file or directory`.

- [ ] **Step 4: Write the header**

`firmware/application/external/presence/presence_dsp.hpp`:

```cpp
/*
 * Copyright (C) 2026 Yazeed Alrubyli
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/* Presence detector: turns a stream of received-power samples (centi-dB) into
 * EMPTY / PRESENT / MOVING / NO SIGNAL. Integer arithmetic only, no heap, no
 * firmware headers, so it is also compiled and tested on the host. */

#ifndef __PRESENCE_DSP_HPP__
#define __PRESENCE_DSP_HPP__

#include <cstddef>
#include <cstdint>

namespace ui::external_app::presence {

enum class State : uint8_t { NoSignal,
                             Calibrating,
                             Empty,
                             Present,
                             Moving };

struct Config {
    uint16_t rate_hz{10};        // input sample rate: 10 (PEAK) or 50 (MEAN)
    uint8_t hold_n{2};           // max-hold over the last n samples
    int32_t min_power_cdb{-9000};  // slow power below this => NoSignal
};

struct Output {
    State state{State::NoSignal};
    uint8_t confidence_pct{0};
    int32_t power_cdb{0};
    int32_t motion_cdb{0};
    int32_t still_cdb{0};
    int32_t thr_motion_cdb{0};
    int32_t thr_still_cdb{0};
    uint16_t cal_remaining{0};  // samples left in calibration, 0 when idle
};

constexpr int32_t default_thr_motion_cdb = 150;
constexpr int32_t default_thr_still_cdb = 40;
constexpr int32_t floor_thr_motion_cdb = 20;
constexpr int32_t floor_thr_still_cdb = 5;
constexpr uint16_t max_rate_hz = 50;
constexpr uint8_t max_hold_n = 4;
constexpr size_t motion_window_s = 2;
constexpr size_t still_window_s = 8;
constexpr size_t cal_delay_s = 5;    // owner leaves the area
constexpr size_t cal_measure_s = 8;  // baseline measurement
constexpr size_t cal_settle_s = 3;   // first seconds of the measurement are not averaged

/* Ring of squared deviations with a running sum. */
class RmsWindow {
   public:
    void init(int32_t* storage, size_t length);
    void clear();
    void push(int32_t deviation);  // |deviation| <= 46340
    uint32_t rms() const;

   private:
    int32_t* buf_{nullptr};
    size_t len_{0};
    size_t pos_{0};
    size_t count_{0};
    int64_t sum_{0};
};

/* Exponential moving average, alpha = 1 / 2^shift, Q16 state. */
class Ema {
   public:
    void init(uint8_t shift);
    void reset();
    int32_t update(int32_t x);  // returns the new value in Q8
    int32_t q8() const { return q16_ >> 8; }

   private:
    int32_t q16_{0};
    uint8_t shift_{0};
    bool primed_{false};
};

class Detector {
   public:
    explicit Detector(const Config& config);
    Detector(const Detector&) = delete;
    Detector& operator=(const Detector&) = delete;

    void configure(const Config& config);  // re-derives windows, then reset()
    void reset();                          // clears signal state, keeps thresholds
    void start_calibration();
    void set_thresholds(int32_t motion_cdb, int32_t still_cdb);
    const Output& push(int32_t power_cdb);
    const Output& output() const { return out_; }
    const Config& config() const { return cfg_; }

    static uint8_t ema_shift(uint16_t rate_hz, uint16_t tau_ms);

   private:
    static constexpr size_t motion_capacity = motion_window_s * max_rate_hz;
    static constexpr size_t still_capacity = still_window_s * max_rate_hz;
    static constexpr size_t lag_capacity = max_rate_hz / 5 + 1;

    void run_calibration(int32_t motion_q4, int32_t still_q8);
    void decide(int32_t motion_q4, int32_t still_q8, int32_t slow_q8);

    Config cfg_{};
    Output out_{};

    int32_t hold_buf_[max_hold_n]{};
    uint8_t hold_pos_{0};
    uint8_t hold_count_{0};

    int32_t lag_buf_[lag_capacity]{};
    size_t lag_len_{1};
    size_t lag_pos_{0};
    size_t lag_count_{0};

    Ema ema_fast_{};
    Ema ema_slow_{};

    int32_t motion_sq_[motion_capacity]{};
    int32_t still_sq_[still_capacity]{};
    RmsWindow motion_win_{};
    RmsWindow still_win_{};

    int32_t thr_motion_cdb_{default_thr_motion_cdb};
    int32_t thr_still_cdb_{default_thr_still_cdb};

    State state_{State::Empty};
    size_t below_{0};
    size_t moving_hold_{10};
    size_t present_hold_{30};

    uint8_t cal_phase_{0};  // 0 idle, 1 delay, 2 measure
    size_t cal_left_{0};
    int64_t cal_sum_m_{0};
    int64_t cal_sum_s_{0};
    size_t cal_n_{0};
};

}  // namespace ui::external_app::presence

#endif /* __PRESENCE_DSP_HPP__ */
```

- [ ] **Step 5: Write the implementation**

`firmware/application/external/presence/presence_dsp.cpp`:

```cpp
/*
 * Copyright (C) 2026 Yazeed Alrubyli
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "presence_dsp.hpp"

namespace ui::external_app::presence {

namespace {

constexpr int32_t deviation_clamp = 46340;  // floor(sqrt(INT32_MAX))

int32_t clamp_deviation(int64_t d) {
    if (d > deviation_clamp) return deviation_clamp;
    if (d < -deviation_clamp) return -deviation_clamp;
    return static_cast<int32_t>(d);
}

int64_t clamp_i64(int64_t v, int64_t lo, int64_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

uint32_t isqrt64(uint64_t v) {
    uint64_t res = 0;
    uint64_t bit = uint64_t{1} << 62;
    while (bit > v) bit >>= 2;
    while (bit != 0) {
        if (v >= res + bit) {
            v -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<uint32_t>(res);
}

}  // namespace

/* RmsWindow */

void RmsWindow::init(int32_t* storage, size_t length) {
    buf_ = storage;
    len_ = length;
    clear();
}

void RmsWindow::clear() {
    pos_ = 0;
    count_ = 0;
    sum_ = 0;
    for (size_t i = 0; i < len_; i++) buf_[i] = 0;
}

void RmsWindow::push(int32_t deviation) {
    if (len_ == 0) return;
    const int32_t sq = deviation * deviation;
    if (count_ == len_) {
        sum_ -= buf_[pos_];
    } else {
        count_++;
    }
    buf_[pos_] = sq;
    sum_ += sq;
    pos_++;
    if (pos_ == len_) pos_ = 0;
}

uint32_t RmsWindow::rms() const {
    if (count_ == 0) return 0;
    return isqrt64(static_cast<uint64_t>(sum_ / static_cast<int64_t>(count_)));
}

/* Ema */

void Ema::init(uint8_t shift) {
    shift_ = shift;
    reset();
}

void Ema::reset() {
    q16_ = 0;
    primed_ = false;
}

int32_t Ema::update(int32_t x) {
    const int32_t x_q16 = x * 65536;
    if (!primed_) {
        q16_ = x_q16;
        primed_ = true;
    } else {
        q16_ += (x_q16 - q16_) >> shift_;
    }
    return q16_ >> 8;
}

/* Detector */

uint8_t Detector::ema_shift(uint16_t rate_hz, uint16_t tau_ms) {
    const uint32_t n = (static_cast<uint32_t>(tau_ms) * rate_hz) / 1000;
    if (n < 2) return 0;
    uint8_t k = 0;
    while ((2u << k) <= n) k++;                       // 2^k <= n < 2^(k+1)
    if (n - (1u << k) >= (2u << k) - n) k++;          // nearest power of two, ties round up
    return k;
}

Detector::Detector(const Config& config) {
    configure(config);
}

void Detector::configure(const Config& config) {
    cfg_ = config;
    if (cfg_.rate_hz < 1) cfg_.rate_hz = 1;
    if (cfg_.rate_hz > max_rate_hz) cfg_.rate_hz = max_rate_hz;
    if (cfg_.hold_n < 1) cfg_.hold_n = 1;
    if (cfg_.hold_n > max_hold_n) cfg_.hold_n = max_hold_n;
    const size_t rate = cfg_.rate_hz;
    lag_len_ = rate / 5;
    if (lag_len_ < 1) lag_len_ = 1;
    if (lag_len_ > lag_capacity - 1) lag_len_ = lag_capacity - 1;
    ema_fast_.init(ema_shift(cfg_.rate_hz, 350));
    ema_slow_.init(ema_shift(cfg_.rate_hz, 2800));
    motion_win_.init(motion_sq_, motion_window_s * rate);
    still_win_.init(still_sq_, still_window_s * rate);
    moving_hold_ = rate;
    present_hold_ = 3 * rate;
    reset();
}

void Detector::reset() {
    hold_pos_ = 0;
    hold_count_ = 0;
    lag_pos_ = 0;
    lag_count_ = 0;
    ema_fast_.reset();
    ema_slow_.reset();
    motion_win_.clear();
    still_win_.clear();
    state_ = State::Empty;
    below_ = 0;
    cal_phase_ = 0;
    cal_left_ = 0;
    cal_sum_m_ = 0;
    cal_sum_s_ = 0;
    cal_n_ = 0;
    out_ = Output{};
    out_.thr_motion_cdb = thr_motion_cdb_;
    out_.thr_still_cdb = thr_still_cdb_;
}

void Detector::start_calibration() {
    cal_phase_ = 1;
    cal_left_ = cal_delay_s * cfg_.rate_hz;
    cal_sum_m_ = 0;
    cal_sum_s_ = 0;
    cal_n_ = 0;
    state_ = State::Calibrating;
    below_ = 0;
    out_.state = State::Calibrating;
    out_.confidence_pct = 0;
    out_.cal_remaining = static_cast<uint16_t>(cal_left_ + cal_measure_s * cfg_.rate_hz);
}

void Detector::set_thresholds(int32_t motion_cdb, int32_t still_cdb) {
    thr_motion_cdb_ = motion_cdb < floor_thr_motion_cdb ? floor_thr_motion_cdb : motion_cdb;
    thr_still_cdb_ = still_cdb < floor_thr_still_cdb ? floor_thr_still_cdb : still_cdb;
    out_.thr_motion_cdb = thr_motion_cdb_;
    out_.thr_still_cdb = thr_still_cdb_;
}

const Output& Detector::push(int32_t power_cdb) {
    // 1. max-hold over the last hold_n samples
    hold_buf_[hold_pos_] = power_cdb;
    hold_pos_ = static_cast<uint8_t>((hold_pos_ + 1) % cfg_.hold_n);
    if (hold_count_ < cfg_.hold_n) hold_count_++;
    int32_t h = hold_buf_[0];
    for (uint8_t i = 1; i < hold_count_; i++) {
        if (hold_buf_[i] > h) h = hold_buf_[i];
    }

    // 2. smoothed power
    const int32_t fast_q8 = ema_fast_.update(h);
    const int32_t slow_q8 = ema_slow_.update(h);

    // 3. motion: RMS of (h[n] - h[n - lag]) over the motion window, Q4
    int32_t lagged = h;
    if (lag_count_ >= lag_len_) lagged = lag_buf_[lag_pos_];
    lag_buf_[lag_pos_] = h;
    lag_pos_++;
    if (lag_pos_ == lag_len_) lag_pos_ = 0;
    if (lag_count_ < lag_len_) lag_count_++;
    motion_win_.push(clamp_deviation(static_cast<int64_t>(h - lagged) * 16));

    // 4. still: RMS of (fast - slow) over the still window, Q8
    still_win_.push(clamp_deviation(static_cast<int64_t>(fast_q8) - slow_q8));

    const int32_t motion_q4 = static_cast<int32_t>(motion_win_.rms());
    const int32_t still_q8 = static_cast<int32_t>(still_win_.rms());
    out_.power_cdb = fast_q8 >> 8;
    out_.motion_cdb = motion_q4 >> 4;
    out_.still_cdb = still_q8 >> 8;

    if (cal_phase_ != 0) run_calibration(motion_q4, still_q8);
    decide(motion_q4, still_q8, slow_q8);
    return out_;
}

void Detector::run_calibration(int32_t motion_q4, int32_t still_q8) {
    const size_t measure = cal_measure_s * cfg_.rate_hz;
    if (cal_phase_ == 1) {
        if (--cal_left_ == 0) {
            cal_phase_ = 2;
            cal_left_ = measure;
            motion_win_.clear();
            still_win_.clear();
        }
        out_.cal_remaining = static_cast<uint16_t>(cal_left_ + (cal_phase_ == 1 ? measure : 0));
        return;
    }
    if (cal_left_ <= (cal_measure_s - cal_settle_s) * cfg_.rate_hz) {
        cal_sum_m_ += motion_q4;
        cal_sum_s_ += still_q8;
        cal_n_++;
    }
    if (--cal_left_ == 0) {
        const int64_t base_m_q4 = cal_n_ ? cal_sum_m_ / static_cast<int64_t>(cal_n_) : 0;
        const int64_t base_s_q8 = cal_n_ ? cal_sum_s_ / static_cast<int64_t>(cal_n_) : 0;
        set_thresholds(static_cast<int32_t>(clamp_i64((3 * base_m_q4) >> 4, 0, 30000)),
                       static_cast<int32_t>(clamp_i64((3 * base_s_q8) >> 8, 0, 30000)));
        cal_phase_ = 0;
        state_ = State::Empty;
        below_ = 0;
    }
    out_.cal_remaining = static_cast<uint16_t>(cal_phase_ ? cal_left_ : 0);
}

void Detector::decide(int32_t motion_q4, int32_t still_q8, int32_t slow_q8) {
    if (cal_phase_ != 0) {
        out_.state = State::Calibrating;
        out_.confidence_pct = 0;
        return;
    }
    if ((slow_q8 >> 8) < cfg_.min_power_cdb) {
        state_ = State::NoSignal;
        below_ = 0;
        out_.state = State::NoSignal;
        out_.confidence_pct = 0;
        return;
    }

    const int64_t thr_m_q4 = static_cast<int64_t>(thr_motion_cdb_) * 16;
    const int64_t thr_s_q8 = static_cast<int64_t>(thr_still_cdb_) * 256;
    const bool motion_on = motion_q4 >= thr_m_q4;
    const bool motion_off = static_cast<int64_t>(motion_q4) * 10 < thr_m_q4 * 7;
    const bool still_on = still_q8 >= thr_s_q8;
    const bool still_off = static_cast<int64_t>(still_q8) * 10 < thr_s_q8 * 7;

    switch (state_) {
        case State::Moving:
            if (motion_off) {
                if (++below_ >= moving_hold_) {
                    state_ = still_on ? State::Present : State::Empty;
                    below_ = 0;
                }
            } else {
                below_ = 0;
            }
            break;
        case State::Present:
            if (motion_on) {
                state_ = State::Moving;
                below_ = 0;
            } else if (still_off) {
                if (++below_ >= present_hold_) {
                    state_ = State::Empty;
                    below_ = 0;
                }
            } else {
                below_ = 0;
            }
            break;
        default:
            below_ = 0;
            if (motion_on)
                state_ = State::Moving;
            else if (still_on)
                state_ = State::Present;
            else
                state_ = State::Empty;
            break;
    }

    const int64_t m_ratio = 50 * static_cast<int64_t>(motion_q4) / thr_m_q4;
    const int64_t s_ratio = 50 * static_cast<int64_t>(still_q8) / thr_s_q8;
    int64_t confidence = 0;
    switch (state_) {
        case State::Moving:
            confidence = m_ratio;
            break;
        case State::Present:
            confidence = s_ratio;
            break;
        default:
            confidence = 100 - (m_ratio > s_ratio ? m_ratio : s_ratio);
            break;
    }
    out_.state = state_;
    out_.confidence_pct = static_cast<uint8_t>(clamp_i64(confidence, 0, 100));
}

}  // namespace ui::external_app::presence
```

- [ ] **Step 6: Run the tests until they pass**

Run: `firmware/application/external/presence/test/run.sh`
Expected: `[doctest] test cases: 15 | 15 passed | 0 failed` and `Status: SUCCESS!`. If a numeric expectation misses by a small margin, adjust the test's amplitude or sample count only after reading the actual value with `d.output().motion_cdb` printed in a `MESSAGE()`; never loosen a state assertion.

- [ ] **Step 7: Format and commit**

```bash
./tools/clang-format.sh -i firmware/application/external/presence/presence_dsp.hpp firmware/application/external/presence/presence_dsp.cpp firmware/application/external/presence/test/test_presence_dsp.cpp
firmware/application/external/presence/test/run.sh
git add firmware/application/external/presence
git commit -m "presence: integer presence/motion detector with host tests"
git push origin main
```

---

### Task 2: The app (`PresenceView`), entry point, registration, firmware build

**Files:**
- Create: `firmware/application/external/presence/ui_presence.hpp`
- Create: `firmware/application/external/presence/ui_presence.cpp`
- Create: `firmware/application/external/presence/main.cpp`
- Modify: `firmware/application/external/external.cmake` (EXTCPPSRC block ending at line 343; EXTAPPLIST ending at line 428)
- Modify: `firmware/application/external/external.ld` (MEMORY block, after line 104 `ram_external_app_kiss_tnc`; SECTIONS, after line 585)

**Interfaces:**
- Consumes: Task 1's `Detector`, `Config`, `Output`, `State`, `default_thr_*`.
- Produces: `ui::external_app::presence::PresenceView` (pushed by `initialize_app`), `_application_information_presence` in section `.external_app.app_presence.application_information`; build outputs `build/firmware/application/presence.ppma` and `build/firmware/portapack-mayhem-firmware.bin`.

- [ ] **Step 1: Write the view header**

`firmware/application/external/presence/ui_presence.hpp`:

```cpp
/*
 * Copyright (C) 2026 Yazeed Alrubyli
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef __UI_PRESENCE_HPP__
#define __UI_PRESENCE_HPP__

#include "app_settings.hpp"
#include "baseband_api.hpp"
#include "file.hpp"
#include "message.hpp"
#include "presence_dsp.hpp"
#include "radio_state.hpp"
#include "receiver_model.hpp"
#include "string_format.hpp"
#include "ui.hpp"
#include "ui_freq_field.hpp"
#include "ui_navigation.hpp"
#include "ui_receiver.hpp"
#include "ui_widget.hpp"

#include <string>
#include <string_view>

using namespace std::literals;

namespace ui::external_app::presence {

/* "-43.2" for decimals 1, "0.21" for decimals 2. */
std::string format_db(int32_t cdb, uint8_t decimals);

class StatusBanner : public Widget {
   public:
    explicit StatusBanner(Rect parent_rect)
        : Widget{parent_rect} {}
    void set(State state, uint8_t confidence_pct, uint16_t cal_seconds);
    void paint(Painter& painter) override;

   private:
    State state_{State::NoSignal};
    uint8_t confidence_{0};
    uint16_t cal_seconds_{0};
};

class ScoreBar : public Widget {
   public:
    ScoreBar(Rect parent_rect, std::string label, uint8_t decimals)
        : Widget{parent_rect}, label_{std::move(label)}, decimals_{decimals} {}
    void set(int32_t value_cdb, int32_t threshold_cdb, bool active);
    void paint(Painter& painter) override;

   private:
    std::string label_;
    uint8_t decimals_;
    int32_t value_{0};
    int32_t threshold_{1};
    bool active_{false};
};

class TraceWidget : public Widget {
   public:
    static constexpr size_t capacity = 240;
    explicit TraceWidget(Rect parent_rect)
        : Widget{parent_rect} {}
    void push(int32_t value_cdb);
    void clear();
    void paint(Painter& painter) override;

   private:
    int16_t samples_[capacity]{};
    size_t pos_{0};
    size_t count_{0};
};

class PresenceView : public View {
   public:
    explicit PresenceView(NavigationView& nav);
    ~PresenceView();

    void focus() override;
    std::string title() const override { return "Presence"; }

   private:
    static constexpr rf::Frequency default_frequency = 5'560'000'000;
    static constexpr uint32_t source_peak = 0;
    static constexpr uint32_t source_mean = 1;

    void start_radio();
    void set_bandwidth(uint32_t rate);
    void apply_source();
    void feed(int32_t power_cdb);
    void on_statistics(const ChannelStatistics& statistics);
    void on_frame_sync();
    void refresh_display();
    void open_log();
    void close_log();
    static int32_t spectrum_mean_cdb(const ChannelSpectrum& spectrum);

    NavigationView& nav_;
    RxRadioState radio_state_{};

    // persisted
    uint32_t source_{source_peak};
    uint32_t bw_index_{2};
    int32_t min_power_cdb_{-9000};
    int32_t thr_motion_cdb_{default_thr_motion_cdb};
    int32_t thr_still_cdb_{default_thr_still_cdb};
    bool log_enabled_{false};

    app_settings::SettingsManager settings_{
        "rx_presence",
        app_settings::Mode::RX,
        {
            {"source"sv, &source_},
            {"bw_index"sv, &bw_index_},
            {"min_power_cdb"sv, &min_power_cdb_},
            {"thr_motion_cdb"sv, &thr_motion_cdb_},
            {"thr_still_cdb"sv, &thr_still_cdb_},
            {"log"sv, &log_enabled_},
        }};

    Detector detector_{Config{10, 2, -9000}};
    ChannelSpectrumFIFO* fifo_{nullptr};
    uint32_t push_count_{0};
    uint32_t rate_window_start_ms_{0};
    uint32_t rate_x10_{0};
    uint8_t redraw_counter_{0};
    bool was_calibrating_{false};
    File log_file_{};
    bool log_open_{false};

    Labels labels_{
        {{UI_POS_X(0), UI_POS_Y(0)}, "LNA:   VGA:   AMP:", Theme::getInstance()->fg_light->foreground}};
    LNAGainField field_lna_{{UI_POS_X(4), UI_POS_Y(0)}};
    VGAGainField field_vga_{{UI_POS_X(11), UI_POS_Y(0)}};
    RFAmpField field_rf_amp_{{UI_POS_X(18), UI_POS_Y(0)}};
    OptionsField field_log_{
        {UI_POS_X(22), UI_POS_Y(0)},
        7,
        {{"Log:off", 0}, {"Log:on ", 1}}};
    RxFrequencyField field_frequency_{{UI_POS_X(0), UI_POS_Y(1)}, nav_};
    OptionsField field_source_{
        {UI_POS_X(11), UI_POS_Y(1)},
        4,
        {{"PEAK", source_peak}, {"MEAN", source_mean}}};
    OptionsField field_bw_{
        {UI_POS_X(16), UI_POS_Y(1)},
        5,
        {{" 750k", 750000}, {"1.25M", 1250000}, {" 2.5M", 2500000}, {"   5M", 5000000}}};
    Button button_cal_{
        {UI_POS_X(0), UI_POS_Y(2), UI_POS_WIDTH(5), UI_POS_DEFAULT_HEIGHT},
        "Cal"};
    Text text_cal_{
        {UI_POS_X(6), UI_POS_Y(2), UI_POS_WIDTH(24), UI_POS_DEFAULT_HEIGHT},
        "Cal: not run"};
    StatusBanner banner_{{UI_POS_X(0), UI_POS_Y(3), screen_width, 3 * UI_POS_DEFAULT_HEIGHT}};
    ScoreBar bar_motion_{{UI_POS_X(0), UI_POS_Y(6), screen_width, 2 * UI_POS_DEFAULT_HEIGHT}, "Motion", 1};
    ScoreBar bar_still_{{UI_POS_X(0), UI_POS_Y(8), screen_width, 2 * UI_POS_DEFAULT_HEIGHT}, "Still", 2};
    TraceWidget trace_{{UI_POS_X(0), UI_POS_Y(10), screen_width, 9 * UI_POS_DEFAULT_HEIGHT}};
    Text text_numbers_{
        {UI_POS_X(0), UI_POS_Y(19), screen_width, UI_POS_DEFAULT_HEIGHT},
        ""};

    MessageHandlerRegistration message_handler_stats_{
        Message::ID::ChannelStatistics,
        [this](const Message* const p) {
            on_statistics(static_cast<const ChannelStatisticsMessage*>(p)->statistics);
        }};
    MessageHandlerRegistration message_handler_spectrum_config_{
        Message::ID::ChannelSpectrumConfig,
        [this](const Message* const p) {
            fifo_ = reinterpret_cast<const ChannelSpectrumConfigMessage*>(p)->fifo;
        }};
    MessageHandlerRegistration message_handler_frame_sync_{
        Message::ID::DisplayFrameSync,
        [this](const Message* const) {
            on_frame_sync();
        }};
};

}  // namespace ui::external_app::presence

#endif /* __UI_PRESENCE_HPP__ */
```

- [ ] **Step 2: Write the view implementation**

`firmware/application/external/presence/ui_presence.cpp`:

```cpp
/*
 * Copyright (C) 2026 Yazeed Alrubyli
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui_presence.hpp"

#include "oversample.hpp"
#include "portapack.hpp"
#include "ui_font_fixed_8x16.hpp"
#include "ui_spectrum.hpp"

#include <ch.h>

using namespace portapack;

namespace ui::external_app::presence {

std::string format_db(int32_t cdb, uint8_t decimals) {
    const bool negative = cdb < 0;
    uint32_t v = static_cast<uint32_t>(negative ? -cdb : cdb);
    uint32_t divisor = 100;
    if (decimals == 1) {
        v = (v + 5) / 10;
        divisor = 10;
    }
    std::string s = negative ? "-" : "";
    s += to_string_dec_uint(v / divisor);
    s += ".";
    s += to_string_dec_uint(v % divisor, decimals, '0');
    return s;
}

/* StatusBanner */

void StatusBanner::set(State state, uint8_t confidence_pct, uint16_t cal_seconds) {
    if (state == state_ && confidence_pct == confidence_ && cal_seconds == cal_seconds_) return;
    state_ = state;
    confidence_ = confidence_pct;
    cal_seconds_ = cal_seconds;
    set_dirty();
}

void StatusBanner::paint(Painter& painter) {
    const auto r = screen_rect();
    Color bg = Color::dark_grey();
    Color fg = Color::white();
    std::string text;
    switch (state_) {
        case State::Calibrating:
            bg = Color::blue();
            text = "CALIBRATING " + to_string_dec_uint(cal_seconds_) + "s";
            break;
        case State::Empty:
            bg = Color::green();
            fg = Color::black();
            text = "EMPTY " + to_string_dec_uint(confidence_) + "%";
            break;
        case State::Present:
            bg = Color::yellow();
            fg = Color::black();
            text = "PRESENT " + to_string_dec_uint(confidence_) + "%";
            break;
        case State::Moving:
            bg = Color::red();
            text = "MOVING " + to_string_dec_uint(confidence_) + "%";
            break;
        default:
            text = "NO SIGNAL";
            break;
    }
    painter.fill_rectangle(r, bg);
    const int text_width = static_cast<int>(text.size()) * 8;
    const Point p{r.left() + (r.width() - text_width) / 2, r.top() + (r.height() - 16) / 2};
    painter.draw_string(p, font::fixed_8x16, fg, bg, text);
}

/* ScoreBar */

void ScoreBar::set(int32_t value_cdb, int32_t threshold_cdb, bool active) {
    if (threshold_cdb < 1) threshold_cdb = 1;
    if (value_cdb == value_ && threshold_cdb == threshold_ && active == active_) return;
    value_ = value_cdb;
    threshold_ = threshold_cdb;
    active_ = active;
    set_dirty();
}

void ScoreBar::paint(Painter& painter) {
    const auto r = screen_rect();
    painter.fill_rectangle(r, Color::black());
    painter.draw_string({r.left(), r.top()}, font::fixed_8x16, Color::white(), Color::black(), label_);
    const std::string value_text = format_db(value_, decimals_) + "dB thr " + format_db(threshold_, decimals_);
    painter.draw_string({r.right() - static_cast<int>(value_text.size()) * 8, r.top()},
                        font::fixed_8x16, Color::grey(), Color::black(), value_text);

    const Rect bar{r.left(), r.top() + 18, r.width(), 12};
    const int64_t full_scale = static_cast<int64_t>(threshold_) * 2;
    int fill = static_cast<int>(static_cast<int64_t>(value_) * bar.width() / full_scale);
    if (fill < 0) fill = 0;
    if (fill > bar.width()) fill = bar.width();
    if (fill > 0) {
        painter.fill_rectangle({bar.left(), bar.top(), fill, bar.height()},
                               active_ ? Color::red() : Color::green());
    }
    painter.draw_rectangle(bar, Color::grey());
    painter.draw_vline({bar.left() + bar.width() / 2, bar.top() - 2}, bar.height() + 4, Color::white());
}

/* TraceWidget */

void TraceWidget::push(int32_t value_cdb) {
    if (value_cdb > 32767) value_cdb = 32767;
    if (value_cdb < -32768) value_cdb = -32768;
    samples_[pos_] = static_cast<int16_t>(value_cdb);
    pos_ = (pos_ + 1) % capacity;
    if (count_ < capacity) count_++;
}

void TraceWidget::clear() {
    pos_ = 0;
    count_ = 0;
    set_dirty();
}

void TraceWidget::paint(Painter& painter) {
    const auto r = screen_rect();
    painter.fill_rectangle(r, Color::black());
    if (count_ < 2) return;

    int32_t lo = 32767;
    int32_t hi = -32768;
    for (size_t i = 0; i < count_; i++) {
        const int32_t v = samples_[i];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    lo -= 100;
    hi += 100;
    const int32_t span = hi - lo;
    const int h = r.height() - 1;
    const size_t first = (pos_ + capacity - count_) % capacity;
    int prev_y = 0;
    for (size_t i = 0; i < count_; i++) {
        const int32_t v = samples_[(first + i) % capacity];
        const int y = r.bottom() - 1 - static_cast<int>(static_cast<int64_t>(v - lo) * h / span);
        const int x = r.right() - static_cast<int>(count_ - i);
        if (i == 0) prev_y = y;
        const int y0 = y < prev_y ? y : prev_y;
        const int y1 = y < prev_y ? prev_y : y;
        painter.draw_vline({x, y0}, y1 - y0 + 1, Color::green());
        prev_y = y;
    }
    painter.draw_string({r.left() + 1, r.top()}, font::fixed_8x16, Color::grey(), Color::black(), format_db(hi, 1));
    painter.draw_string({r.left() + 1, r.bottom() - 16}, font::fixed_8x16, Color::grey(), Color::black(), format_db(lo, 1));
}

/* PresenceView */

PresenceView::PresenceView(NavigationView& nav)
    : nav_{nav} {
    add_children({&labels_,
                  &field_lna_,
                  &field_vga_,
                  &field_rf_amp_,
                  &field_log_,
                  &field_frequency_,
                  &field_source_,
                  &field_bw_,
                  &button_cal_,
                  &text_cal_,
                  &banner_,
                  &bar_motion_,
                  &bar_still_,
                  &trace_,
                  &text_numbers_});

    if (!settings_.loaded()) field_frequency_.set_value(default_frequency);
    if (bw_index_ >= field_bw_.options().size()) bw_index_ = 2;
    if (source_ > source_mean) source_ = source_peak;
    field_source_.set_selected_index(source_);
    field_bw_.set_selected_index(bw_index_);
    field_log_.set_selected_index(log_enabled_ ? 1 : 0);
    detector_.set_thresholds(thr_motion_cdb_, thr_still_cdb_);
    thr_motion_cdb_ = detector_.output().thr_motion_cdb;
    thr_still_cdb_ = detector_.output().thr_still_cdb;
    text_cal_.set("Cal: M" + format_db(thr_motion_cdb_, 1) + " S" + format_db(thr_still_cdb_, 2));

    field_source_.on_change = [this](size_t, int32_t value) {
        source_ = static_cast<uint32_t>(value);
        apply_source();
    };
    field_bw_.on_change = [this](size_t index, int32_t rate) {
        bw_index_ = index;
        set_bandwidth(static_cast<uint32_t>(rate));
        detector_.reset();
        trace_.clear();
    };
    field_log_.on_change = [this](size_t, int32_t value) {
        log_enabled_ = value != 0;
        if (log_enabled_)
            open_log();
        else
            close_log();
    };
    button_cal_.on_select = [this](Button&) {
        detector_.start_calibration();
        was_calibrating_ = true;
        text_cal_.set("Cal: leave the area");
    };

    rate_window_start_ms_ = chTimeNow();
    start_radio();
    apply_source();
    if (log_enabled_) open_log();
}

PresenceView::~PresenceView() {
    close_log();
    if (source_ == source_mean) baseband::spectrum_streaming_stop();
    receiver_model.disable();
    baseband::shutdown();
}

void PresenceView::focus() {
    field_frequency_.focus();
}

void PresenceView::start_radio() {
    receiver_model.disable();
    baseband::shutdown();
    baseband::run_image(portapack::spi_flash::image_tag_capture);
    receiver_model.set_modulation(ReceiverModel::Mode::Capture);
    set_bandwidth(static_cast<uint32_t>(field_bw_.selected_index_value()));
    receiver_model.enable();
}

void PresenceView::set_bandwidth(uint32_t rate) {
    baseband::set_sample_rate(rate, get_oversample_rate(rate));
    const auto actual = get_actual_sample_rate(rate);
    receiver_model.set_sampling_rate(actual);
    receiver_model.set_baseband_bandwidth(filter_bandwidth_for_sampling_rate(actual));
}

void PresenceView::apply_source() {
    if (source_ == source_mean) {
        detector_.configure(Config{50, 1, min_power_cdb_});
        baseband::spectrum_streaming_start();
    } else {
        baseband::spectrum_streaming_stop();
        detector_.configure(Config{10, 2, min_power_cdb_});
    }
    detector_.set_thresholds(thr_motion_cdb_, thr_still_cdb_);
    trace_.clear();
    push_count_ = 0;
    redraw_counter_ = 0;
    rate_window_start_ms_ = chTimeNow();
    refresh_display();
}

int32_t PresenceView::spectrum_mean_cdb(const ChannelSpectrum& spectrum) {
    int32_t sum = 0;
    for (size_t i = 4; i < 96; i++) sum += spectrum.db[i];
    for (size_t i = 160; i < 252; i++) sum += spectrum.db[i];
    return (20 * sum) / 184 - 5100;
}

void PresenceView::on_statistics(const ChannelStatistics& statistics) {
    if (source_ != source_peak) return;
    feed(statistics.max_db * 100);
}

void PresenceView::on_frame_sync() {
    if (source_ == source_mean && fifo_) {
        ChannelSpectrum spectrum;
        while (fifo_->out(spectrum)) feed(spectrum_mean_cdb(spectrum));
    }
    const uint32_t now = chTimeNow();
    const uint32_t elapsed = now - rate_window_start_ms_;
    if (elapsed >= 1000) {
        rate_x10_ = push_count_ * 10000 / elapsed;
        push_count_ = 0;
        rate_window_start_ms_ = now;
    }
}

void PresenceView::feed(int32_t power_cdb) {
    const auto& out = detector_.push(power_cdb);
    push_count_++;
    trace_.push(power_cdb);
    if (log_open_) {
        log_file_.write_line(to_string_dec_uint(chTimeNow()) + "," + to_string_dec_int(power_cdb));
    }
    if (was_calibrating_ && out.cal_remaining == 0 && out.state != State::Calibrating) {
        was_calibrating_ = false;
        thr_motion_cdb_ = out.thr_motion_cdb;
        thr_still_cdb_ = out.thr_still_cdb;
        text_cal_.set("Cal: M" + format_db(thr_motion_cdb_, 1) + " S" + format_db(thr_still_cdb_, 2));
    }
    const uint8_t divider = source_ == source_mean ? 5 : 1;
    if (++redraw_counter_ >= divider) {
        redraw_counter_ = 0;
        refresh_display();
    }
}

void PresenceView::refresh_display() {
    const auto& out = detector_.output();
    const uint16_t rate = detector_.config().rate_hz;
    banner_.set(out.state, out.confidence_pct, static_cast<uint16_t>((out.cal_remaining + rate - 1) / rate));
    bar_motion_.set(out.motion_cdb, out.thr_motion_cdb, out.state == State::Moving);
    bar_still_.set(out.still_cdb, out.thr_still_cdb, out.state == State::Present);
    trace_.set_dirty();
    text_numbers_.set("P" + format_db(out.power_cdb, 1) +
                      " M" + format_db(out.motion_cdb, 1) +
                      " S" + format_db(out.still_cdb, 2) +
                      " " + to_string_dec_uint(rate_x10_ / 10) + "." + to_string_dec_uint(rate_x10_ % 10) + "Hz");
}

void PresenceView::open_log() {
    if (log_open_) return;
    ensure_directory(u"PRESENCE");
    const auto path = next_filename_matching_pattern(u"PRESENCE/PRESENCE_????.CSV");
    if (path.empty()) return;
    if (log_file_.create(path)) return;
    log_open_ = true;
}

void PresenceView::close_log() {
    if (!log_open_) return;
    log_file_.close();
    log_open_ = false;
}

}  // namespace ui::external_app::presence
```

- [ ] **Step 3: Write the entry point**

`firmware/application/external/presence/main.cpp`:

```cpp
/*
 * Copyright (C) 2026 Yazeed Alrubyli
 *
 * This file is part of PortaPack.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ui.hpp"
#include "ui_presence.hpp"
#include "ui_navigation.hpp"
#include "external_app.hpp"

namespace ui::external_app::presence {
void initialize_app(ui::NavigationView& nav) {
    nav.push<PresenceView>();
}
}  // namespace ui::external_app::presence

extern "C" {

__attribute__((section(".external_app.app_presence.application_information"), used)) application_information_t _application_information_presence = {
    /*.memory_location = */ (uint8_t*)0x00000000,
    /*.externalAppEntry = */ ui::external_app::presence::initialize_app,
    /*.header_version = */ CURRENT_HEADER_VERSION,
    /*.app_version = */ VERSION_MD5,

    /*.app_name = */ "Presence",
    /*.bitmap_data = */ {
        // person between two wave arcs, 16x16, LSB = leftmost pixel
        0x00, 0x00,
        0x80, 0x01,
        0xC0, 0x03,
        0xC0, 0x03,
        0x80, 0x01,
        0xC4, 0x23,
        0xE2, 0x47,
        0xEA, 0x57,
        0xEA, 0x57,
        0xCA, 0x53,
        0x62, 0x46,
        0x64, 0x26,
        0x60, 0x06,
        0x60, 0x06,
        0x00, 0x00,
        0x00, 0x00},
    /*.icon_color = */ ui::Color::green().v,
    /*.menu_location = */ app_location_t::RX,
    /*.desired_menu_position = */ -1,

    /*.m4_app_tag = portapack::spi_flash::image_tag_capture */ {'P', 'C', 'A', 'P'},
    /*.m4_app_offset = */ 0x00000000,  // will be filled at compile time
};
}
```

- [ ] **Step 4: Register the app**

In `firmware/application/external/external.cmake`, after the `#p25_tx` two source lines (before the `)` that closes `set(EXTCPPSRC`), add:

```cmake
	#presence
	external/presence/main.cpp
	external/presence/ui_presence.cpp
	external/presence/presence_dsp.cpp
```

In the same file, after the `p25_tx` line at the end of `set(EXTAPPLIST`, add:

```cmake
	presence
```

In `firmware/application/external/external.ld`, after the `ram_external_app_kiss_tnc` MEMORY line, add:

```
    ram_external_app_presence              (rwx) : org = 0xAE000000, len = 32k
```

and after the `.external_app_kiss_tnc` SECTIONS block (`} > ram_external_app_kiss_tnc`), add:

```
    .external_app_presence : ALIGN(4) SUBALIGN(4)
    {
        KEEP(*(.external_app.app_presence.application_information));
        *(*ui*external_app*presence*);
    } > ram_external_app_presence
```

- [ ] **Step 5: Build the firmware in Docker**

Run (from the repo root):

```bash
docker run -v "$PWD:/havoc" -u "$(id -u):$(id -g)" --rm portapack-dev ninja -j16 2>&1 | tail -20
```

Expected: exit 0 and the lines `Creating external application image for presence` with no `can not exceed 32kb`. Then:

```bash
ls -la build/firmware/application/presence.ppma build/firmware/portapack-mayhem-firmware.bin
python3 - <<'EOF'
import os
n = os.path.getsize('build/firmware/application/presence.ppma')
print('presence.ppma', n, 'bytes; app code', n - 14844, 'bytes; headroom', 32768 - n)
assert n <= 32768
EOF
```

If the size check fails: first remove the SD logging (`field_log_`, `open_log`, `close_log`, `log_file_`, the `log` setting), rebuild; then, if still too big, replace `TraceWidget` with the stock `RSSIGraph` widget.

If a compile error names a symbol from this plan, open the header the error points at (`firmware/common/ui_widget.hpp`, `firmware/application/ui/ui_freq_field.hpp`, `firmware/application/file.hpp`, `firmware/application/string_format.hpp`) and adapt the call to the real signature; do not change the behaviour.

- [ ] **Step 6: Re-run the host tests, format, commit**

```bash
firmware/application/external/presence/test/run.sh
./tools/clang-format.sh -i firmware/application/external/presence/ui_presence.hpp firmware/application/external/presence/ui_presence.cpp firmware/application/external/presence/main.cpp
git add firmware/application/external/presence firmware/application/external/external.cmake firmware/application/external/external.ld
git commit -m "presence: PortaPack external app (PEAK/MEAN sources, banner, score bars, trace, SD log)"
git push origin main
```

---

### Task 3: Serial deploy tool

**Files:**
- Create: `tools/presence/pp_serial.py`

**Interfaces:**
- Consumes: the Mayhem USB serial console on `/dev/ttyACM0` (commands `info`, `ls`, `unlink`, `filesize`, `fopen`, `fwb`, `fclose`, `crc32`, `flash`, `reboot`, `applist`, `appstart`, `screenframeshort`, `button`, `touch`).
- Produces: CLI `pp_serial.py {info|cmd|ls|upload|verify|flash|sync-apps|appstart|screenshot|button|touch}` used by Task 4.

- [ ] **Step 1: Write the tool**

`tools/presence/pp_serial.py`:

```python
#!/usr/bin/env python3
"""Talk to the PortaPack Mayhem USB serial console.

Usage:
  pp_serial.py info
  pp_serial.py cmd "<console command>"
  pp_serial.py ls /APPS
  pp_serial.py upload <local-file> </SD/PATH>
  pp_serial.py verify <local-file> </SD/PATH>
  pp_serial.py flash </FIRMWARE/x.bin>
  pp_serial.py sync-apps <dir-with-ppma>
  pp_serial.py appstart <callname>
  pp_serial.py screenshot <out.png>
  pp_serial.py button <n>           (1 up 2 down 3 left 4 right 5 select 6 encoder+ 7 encoder-)
  pp_serial.py touch <x> <y>

Requires pyserial and, for screenshot, Pillow. Fails loudly on any unexpected reply.
"""
import binascii
import os
import re
import sys
import time

import serial

PORT = os.environ.get("PP_PORT", "/dev/ttyACM0")
CHUNK = 16384


class Console:
    def __init__(self, port=PORT):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.s.reset_input_buffer()
        self.s.write(b"\r")
        self._read_until_prompt(2.0)

    def close(self):
        self.s.close()

    def _read_until_prompt(self, timeout):
        buf = b""
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = self.s.read(65536)
            if chunk:
                buf += chunk
                if buf.rstrip().endswith(b"ch>"):
                    break
        return buf

    @staticmethod
    def _clean(raw):
        txt = raw.decode(errors="replace")
        txt = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", txt).replace("\r", "")
        lines = [l for l in txt.split("\n")]
        # drop the echoed command line and the trailing prompt
        if lines and lines[-1].strip() == "ch>":
            lines = lines[:-1]
        if lines:
            lines = lines[1:]
        return "\n".join(lines).strip()

    def cmd(self, text, timeout=5.0, expect_ok=False):
        self.s.reset_input_buffer()
        self.s.write((text + "\r").encode())
        out = self._clean(self._read_until_prompt(timeout))
        if expect_ok and not out.rstrip().endswith("ok"):
            raise SystemExit(f"command failed: {text!r} -> {out!r}")
        return out

    def upload(self, local, remote):
        data = open(local, "rb").read()
        self.cmd(f"fopen {remote}", expect_ok=True)
        sent = 0
        try:
            while sent < len(data):
                part = data[sent:sent + CHUNK]
                self.s.reset_input_buffer()
                self.s.write(f"fwb {len(part)}\r".encode())
                head = b""
                t0 = time.time()
                while b"send " not in head and time.time() - t0 < 5:
                    head += self.s.read(256)
                if b"send " not in head:
                    raise SystemExit(f"fwb handshake failed after {sent} bytes: {head!r}")
                self.s.write(part)
                tail = self._read_until_prompt(30.0)
                if b"ok" not in tail:
                    raise SystemExit(f"fwb chunk not acknowledged after {sent} bytes: {tail!r}")
                sent += len(part)
                print(f"\r  {remote}: {sent}/{len(data)}", end="", flush=True)
        finally:
            self.cmd("fclose", expect_ok=True)
        print()
        self.verify(local, remote)

    def verify(self, local, remote):
        data = open(local, "rb").read()
        size_out = self.cmd(f"filesize {remote}")
        size = int(size_out.split()[0])
        if size != len(data):
            raise SystemExit(f"size mismatch for {remote}: device {size}, local {len(data)}")
        crc_out = self.cmd(f"crc32 {remote}", timeout=120.0)
        m = re.search(r"0x([0-9A-Fa-f]{8})", crc_out)
        if not m:
            raise SystemExit(f"no crc in reply: {crc_out!r}")
        device = int(m.group(1), 16)
        local_crc = binascii.crc32(data) & 0xFFFFFFFF
        if device != local_crc:
            raise SystemExit(f"crc mismatch for {remote}: device {device:08x}, local {local_crc:08x}")
        print(f"  verified {remote}: {size} bytes, crc32 {device:08x}")

    def screenshot(self, out_png):
        from PIL import Image
        self.s.reset_input_buffer()
        self.s.write(b"screenframeshort\r")
        raw = self._read_until_prompt(20.0)
        txt = raw.decode(errors="replace").replace("\r", "")
        rows = [l for l in txt.split("\n") if len(l) == 240]
        if len(rows) != 320:
            raise SystemExit(f"screenframeshort returned {len(rows)} rows of 240 chars, expected 320")
        img = Image.new("RGB", (240, 320))
        px = img.load()
        for y, row in enumerate(rows):
            for x, ch in enumerate(row):
                v = ord(ch) - 32
                px[x, y] = (((v >> 4) & 3) * 85, ((v >> 2) & 3) * 85, (v & 3) * 85)
        img.save(out_png)
        print(f"  screenshot saved to {out_png}")


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    op = argv[1]
    c = Console()
    try:
        if op == "info":
            print(c.cmd("info"))
        elif op == "cmd":
            print(c.cmd(" ".join(argv[2:]), timeout=30.0))
        elif op == "ls":
            print(c.cmd(f"ls {argv[2]}"))
        elif op == "upload":
            c.upload(argv[2], argv[3])
        elif op == "verify":
            c.verify(argv[2], argv[3])
        elif op == "flash":
            print(c.cmd(f"flash {argv[2]}", timeout=10.0))
        elif op == "sync-apps":
            listing = c.cmd("ls /APPS")
            for line in listing.split("\n"):
                name = line.strip()
                if name.lower().endswith(".ppma"):
                    c.cmd(f"unlink /APPS/{name}", expect_ok=True)
                    print(f"  removed /APPS/{name}")
            for name in sorted(os.listdir(argv[2])):
                if name.lower().endswith(".ppma"):
                    c.upload(os.path.join(argv[2], name), f"/APPS/{name}")
        elif op == "appstart":
            print(c.cmd(f"appstart {argv[2]}", timeout=10.0))
        elif op == "screenshot":
            c.screenshot(argv[2])
        elif op == "button":
            print(c.cmd(f"button {argv[2]}"))
        elif op == "touch":
            print(c.cmd(f"touch {argv[2]} {argv[3]}"))
        else:
            print(__doc__)
            return 2
    finally:
        c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
```

- [ ] **Step 2: Smoke-test against the device**

Run (device connected, hackrf.app tab closed):

```bash
/home/y/miniconda3/bin/python3 -c "import PIL" || /home/y/miniconda3/bin/pip install pillow
/home/y/miniconda3/bin/python3 tools/presence/pp_serial.py info
/home/y/miniconda3/bin/python3 tools/presence/pp_serial.py ls /APPS | head -3
/home/y/miniconda3/bin/python3 tools/presence/pp_serial.py screenshot /tmp/pp_before.png
```

Expected: `info` prints `Mayhem Version: v2.4.0`; `ls` prints `.ppma` names; the PNG shows the current screen.

- [ ] **Step 3: Test upload + verify with a small file**

```bash
head -c 20000 /dev/urandom > /tmp/pp_probe.bin
/home/y/miniconda3/bin/python3 tools/presence/pp_serial.py upload /tmp/pp_probe.bin /PP_PROBE.BIN
/home/y/miniconda3/bin/python3 tools/presence/pp_serial.py cmd unlink /PP_PROBE.BIN
```

Expected: `verified /PP_PROBE.BIN: 20000 bytes, crc32 ...` then `ok`. If the CRC comparison fails while the size matches, print the device value and compare against the non-reflected CRC-32 (`0x04C11DB7`, init/xor `0xFFFFFFFF`, no reflection): if that one matches, switch `verify` to that algorithm (implement it in the script) and note it in the tool's docstring.

- [ ] **Step 4: Commit**

```bash
git add tools/presence/pp_serial.py
git commit -m "tools: PortaPack serial console helper (upload/verify/flash/apps/screenshot)"
git push origin main
```

---

### Task 4: Deploy to the H4M and verify on the device

**Files:**
- Create: `docs/superpowers/specs/2026-09-25-presence-bringup/` (screenshots + notes)

**Interfaces:**
- Consumes: Task 2 build outputs, Task 3 tool.
- Produces: the device runs the new firmware with `presence` in the app list; screenshots proving it.

- [ ] **Step 1: Keep a rollback image on the SD**

```bash
T=/home/y/miniconda3/bin/python3; S=tools/presence/pp_serial.py
$T $S ls /FIRMWARE
```

If `portapack-mayhem_v2.4.0.bin` is not listed, download the v2.4.0 release asset `portapack-mayhem_v2.4.0_OCI.ppfw.tar` from `https://github.com/portapack-mayhem/mayhem-firmware/releases/tag/v2.4.0`, extract `FIRMWARE/portapack-mayhem_v2.4.0.bin` (verify the tar's `sha256` against the release notes) and upload it:

```bash
$T $S upload /tmp/v240/FIRMWARE/portapack-mayhem_v2.4.0.bin /FIRMWARE/portapack-mayhem_v2.4.0.bin
```

Also keep the stock `/APPS` set locally for rollback: `$T $S ls /APPS > docs/superpowers/specs/2026-09-25-presence-bringup/stock_apps.txt` and note that the release tar's `APPS/` folder restores them.

- [ ] **Step 2: Upload and flash**

```bash
SHA=$(git rev-parse --short HEAD)
$T $S upload build/firmware/portapack-mayhem-firmware.bin /FIRMWARE/presence-$SHA.bin
$T $S flash /FIRMWARE/presence-$SHA.bin
sleep 25
$T $S info
```

Expected: `flash` replies with progress lines, the device reboots, and `info` reports the new version (`Mayhem Version:` followed by the fork's git describe, not `v2.4.0`). If the port disappears for longer than 60 s, unplug and replug the USB hub cable, then `info` again.

- [ ] **Step 3: Replace the app set**

```bash
$T $S sync-apps build/firmware/application
$T $S cmd applist | tr ' ' '\n' | grep -i presence
```

Expected: every `.ppma` re-uploaded and verified; `applist` includes `presence`.

- [ ] **Step 4: Start the app and screenshot**

```bash
mkdir -p docs/superpowers/specs/2026-09-25-presence-bringup
$T $S appstart presence
sleep 3
$T $S screenshot docs/superpowers/specs/2026-09-25-presence-bringup/01_peak_start.png
```

Expected: the PNG shows `LNA: VGA: AMP:` on row 0, `5560.000M PEAK 2.5M`, the `Cal` button, a banner (EMPTY, NO SIGNAL or MOVING), two bars, the trace and the numbers row ending in `Hz`. Open the PNG with the Read tool and confirm visually.

- [ ] **Step 5: Live-update check (Review Focus 1)**

```bash
sleep 5
$T $S screenshot docs/superpowers/specs/2026-09-25-presence-bringup/02_peak_5s_later.png
```

Expected: both screenshots are valid screens; the numbers row differs (the power value or rate moved) and the rate reads between `9.5Hz` and `10.5Hz`.

- [ ] **Step 6: Switch to MEAN, change bandwidth, screenshot (Review Focus 2)**

Navigate with buttons: `button 2` (down) moves focus from the frequency field to row 2 only if the focus order permits; the reliable way is touch. Touch the source field (row 1 spans y 16..31; the field starts at x 88): `touch 100 24` then `button 6` (encoder +) to select MEAN; touch the bandwidth field `touch 150 24`, `button 6` to select 5M. Then:

```bash
sleep 4
$T $S screenshot docs/superpowers/specs/2026-09-25-presence-bringup/03_mean_5m.png
```

Expected: banner still valid, source shows `MEAN`, bandwidth `5M`, rate between `45.0Hz` and `55.0Hz`, trace redrawn. Then `button 7` on the bandwidth field back to 2.5M and on the source field back to PEAK, screenshot `04_back_to_peak.png`, rate near 10 Hz again.

- [ ] **Step 7: Calibration round trip**

Touch the Cal button (`touch 20 40`), then screenshots at 2 s (`05_cal_delay.png`: banner `CALIBRATING 11s` or similar countdown) and at 15 s (`06_cal_done.png`: banner EMPTY/PRESENT/MOVING, row 2 shows `Cal: M... S...`). Then leave the app (`button 3` left is "back" only in menus; use `cmd appstart` of another app is not needed: press the back key sequence `button 3` twice) and re-enter with `appstart presence`; screenshot `07_reentered.png` must show the same `Cal: M... S...` thresholds (persisted in `/SETTINGS/rx_presence.ini`).

- [ ] **Step 8: Record and commit**

Write `docs/superpowers/specs/2026-09-25-presence-bringup/NOTES.md` with: firmware version string from `info`, `presence.ppma` size, the observed power values (P) for PEAK at 5560 MHz and for MEAN, the observed rates, and any deviation from the expected screens. Commit:

```bash
git add docs/superpowers/specs/2026-09-25-presence-bringup
git commit -m "presence: bring-up screenshots and notes from the H4M"
git push origin main
```

---

### Task 5: Bring-up calibration of the defaults and the walking test

**Files:**
- Modify: `firmware/application/external/presence/ui_presence.hpp` (default `min_power_cdb_`), `presence_dsp.hpp` (default thresholds if the measurements say so)
- Modify: `docs/superpowers/specs/2026-09-25-presence-bringup/NOTES.md`

**Interfaces:**
- Consumes: the running device, the log CSV on the SD.
- Produces: measured defaults committed; walking-test result recorded.

- [ ] **Step 1: Measure the no-signal floor**

With the app running in PEAK at the default gains, tune to a quiet frequency (`touch 40 24`, then `button 6`/`7` steps, or use the keypad by `button 5` on the frequency field and type `1300000000` — simpler: `cmd setfreq 1300000000` from the console, which retunes the receiver), wait 5 s, screenshot `08_floor_peak.png`, read `P`. Repeat with the antenna unscrewed if the owner is present; otherwise note "antenna attached". Switch to MEAN, screenshot `09_floor_mean.png`, read `P`.

- [ ] **Step 2: Set `min_power_cdb_` defaults**

Set the default in `ui_presence.hpp` to `floor + 300` (3 dB above the measured PEAK floor, in centi-dB, rounded to 100). Because the setting is persisted, also document in NOTES.md that existing `/SETTINGS/rx_presence.ini` keeps its old value. Rebuild (Task 2 step 5), redeploy (Task 4 steps 2–3), screenshot `10_nosignal_gate.png` at 1300 MHz showing `NO SIGNAL`, then retune to 5560 MHz (`cmd setfreq 5560000000`) and confirm the banner leaves NO SIGNAL.

- [ ] **Step 3: Record raw data for tuning**

Enable `Log:on` (`touch 200 8`, `button 6`), leave the device still on the desk for 60 s, disable logging, pull the newest `/PRESENCE/PRESENCE_????.CSV` with `cmd fopen`, `cmd fread` lines (or `frb`) through the tool's `cmd`, save it under the bring-up folder as `desk_quiet_peak.csv`. Compute on the host the RMS of the lag-2 differences and the (fast−slow) band; confirm the calibrated thresholds from step 7 of Task 4 are about 3× those numbers.

- [ ] **Step 4: Walking test (owner needed)**

Ask the owner to: (a) place the device on a table, press Cal, leave the room for 15 s; (b) come back and walk past the device at ~2 m for 20 s; (c) sit still 1–2 m away for 30 s; (d) leave again for 60 s. Take a screenshot every 5 s during the whole sequence (`for i in $(seq 1 30); do $T $S screenshot .../walk_$i.png; sleep 5; done`) and tabulate the banner state per screenshot in NOTES.md against the spec's success criteria (§1 of the spec). If (b) never shows MOVING, increase the gains (LNA 40, VGA 30) and repeat; if (a)/(d) show MOVING, note the phone/laptop positions and repeat with them in place during Cal.

- [ ] **Step 5: Commit the measured defaults and the results**

```bash
firmware/application/external/presence/test/run.sh
git add firmware/application/external/presence docs/superpowers/specs/2026-09-25-presence-bringup
git commit -m "presence: measured defaults from the H4M bring-up and walking-test record"
git push origin main
```

---

## Self-review

- **Spec coverage:** §2 files/registration → Task 2 steps 1–4; §3 sources and units → Task 2 (`on_statistics`, `spectrum_mean_cdb`), radio setup → `start_radio`/`set_bandwidth`; §4.1–4.2 detector → Task 1; §4.3 tests → Task 1 step 2 (all eight listed cases plus the Review Focus ones); §4.4 screen → Task 2 widgets and layout; §4.5 log → Task 2 `open_log`/`close_log`; §5 build/deploy/verify → Tasks 2 step 5, 3, 4; §6 risks → size check in Task 2, floors and gains in Task 5.
- **Placeholder scan:** none; every code step has full code; the only conditional branches are the size fallback order (log → trace) and the CRC variant, both spelled out.
- **Type consistency:** `Config{rate_hz, hold_n, min_power_cdb}` aggregate order matches the header; `Detector::configure/reset/start_calibration/set_thresholds/push/output/config` used identically in tests and the view; `format_db(int32_t, uint8_t)` declared in the header and defined in the cpp; `field_bw_.selected_index_value()` returns `int32_t` and is cast to `uint32_t`.
- **Review Focus:** items 1–2 pinned by Task 4 steps 5–6; items 3–5 pinned by Task 1 tests ("configure keeps thresholds, clears scores, and clamps rate and hold", "clamped deviations never overflow on huge steps", "calibration restarted mid-way still finishes with valid thresholds") and Task 2's constructor clamps.
