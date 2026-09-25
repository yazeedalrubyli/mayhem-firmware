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
    // A retune (field, keypad or console setfreq) changes the power level: restart the
    // filters instead of reporting the jump as motion.
    field_frequency_.updated = [this](rf::Frequency) { detector_.reset(); };
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
    last_rate_change_ms_ = rate_window_start_ms_ - 10000;  // allow one adaptation after the first window
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
        adapt_rate();
    }
}

/* The capture baseband emits statistics every 250k decimated samples, but at
 * high bandwidths the M4 cannot keep up and the interval stretches (4.9 Hz at
 * 2.5 MHz instead of 10 Hz). The detector's windows are sized in samples, so
 * re-configure it to the measured rate when it is more than 20% off, at most
 * once every 10 s. */
void PresenceView::adapt_rate() {
    // Never while calibrating: a reconfigure resets the detector and would discard the run.
    if (was_calibrating_ || detector_.output().state == State::Calibrating) return;
    const uint32_t measured = (rate_x10_ + 5) / 10;
    const uint32_t new_rate = rate_tracker_.offer(measured, detector_.config().rate_hz);
    if (new_rate == 0) return;
    const uint32_t now = chTimeNow();
    if (now - last_rate_change_ms_ < 10000) return;
    last_rate_change_ms_ = now;
    Config cfg = detector_.config();
    cfg.rate_hz = static_cast<uint16_t>(new_rate);
    detector_.configure(cfg);
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

void PresenceView::on_freqchg(int64_t freq) {
    field_frequency_.set_value(freq);  // on_change tunes the receiver model and fires `updated`
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
