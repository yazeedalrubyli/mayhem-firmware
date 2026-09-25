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
    while ((2u << k) <= n) k++;               // 2^k <= n < 2^(k+1)
    if (n - (1u << k) >= (2u << k) - n) k++;  // nearest power of two, ties round up
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
    warmup_left_ = cfg_.rate_hz;
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
    // 0. warm-up: the first second after (re)configuration is discarded because the
    //    radio's first reports are not yet valid and would prime the filters wrongly
    if (warmup_left_ > 0) {
        warmup_left_--;
        out_.state = State::NoSignal;
        out_.confidence_pct = 0;
        return out_;
    }

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
