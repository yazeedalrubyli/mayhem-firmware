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
    PresenceView(const PresenceView&) = delete;
    PresenceView& operator=(const PresenceView&) = delete;

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
    void adapt_rate();
    void on_freqchg(int64_t freq);  // console `setfreq` (FreqChangeCommandMessage), as in the other external apps

    MessageHandlerRegistration message_handler_freqchg{
        Message::ID::FreqChangeCommand,
        [this](Message* const p) {
            const auto message = static_cast<const FreqChangeCommandMessage*>(p);
            this->on_freqchg(message->freq);
        }};
    void refresh_display();
    void open_log();
    void close_log();
    static int32_t spectrum_mean_cdb(const ChannelSpectrum& spectrum);

    NavigationView& nav_;
    RxRadioState radio_state_{};

    // persisted
    uint32_t source_{source_peak};
    uint32_t bw_index_{2};
    int32_t min_power_cdb_{-3600};  // H4M receiver floor is -38..-39 dB at 1.1-5.6 GHz (bring-up 2026-09-25); +3 dB
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
    uint32_t last_rate_change_ms_{0};
    RateTracker rate_tracker_{};
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
    // The view is 19 rows tall (the title bar takes the 20th).
    TraceWidget trace_{{UI_POS_X(0), UI_POS_Y(10), screen_width, 8 * UI_POS_DEFAULT_HEIGHT}};
    Text text_numbers_{
        {UI_POS_X(0), UI_POS_Y(18), screen_width, UI_POS_DEFAULT_HEIGHT},
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
