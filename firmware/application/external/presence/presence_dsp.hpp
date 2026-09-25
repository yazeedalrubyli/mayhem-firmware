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
    uint16_t rate_hz{10};          // input sample rate: 10 (PEAK) or 50 (MEAN)
    uint8_t hold_n{2};             // max-hold over the last n samples
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

/* True when the measured message rate is more than 20% away from the rate the
 * detector was configured for, and is one the detector can be configured to.
 * The capture baseband's stats interval stretches when the M4 is overloaded
 * (4.9 Hz at 2.5 MHz instead of 10 Hz), and every window is sized in samples. */
inline bool rate_needs_reconfigure(uint32_t measured_hz, uint32_t expected_hz) {
    if (measured_hz < 1 || measured_hz > max_rate_hz) return false;
    const uint32_t diff = measured_hz > expected_hz ? measured_hz - expected_hz : expected_hz - measured_hz;
    return diff * 5 > expected_hz;
}
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
    void reset();                          // clears signal state, keeps thresholds; first 1 s is a warm-up
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
    size_t warmup_left_{0};  // samples still discarded after configure/reset (1 s)
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
