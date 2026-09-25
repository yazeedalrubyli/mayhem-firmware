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
    CHECK(Detector::ema_shift(10, 350) == 2);   // 3.5 -> 4 samples
    CHECK(Detector::ema_shift(10, 2800) == 5);  // 28 -> 32
    CHECK(Detector::ema_shift(50, 350) == 4);   // 17 -> 16
    CHECK(Detector::ema_shift(50, 2800) == 7);  // 140 -> 128
    CHECK(Detector::ema_shift(1, 350) == 0);    // 0 samples -> no smoothing
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
        const int32_t v = (i % 50 == 25) ? -5000 : -4000;
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
    for (size_t i = 0; i < 500; i++) d.push(((i / 10) % 2) ? 0 : -12000);  // period 20 = 2x the lag
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

TEST_CASE("the first second after configure or reset is a warm-up: reported NoSignal and not scored") {
    Detector d{Config{10, 1, -9000}};
    // radio start-up garbage: a -3600 sample then real -1100 values
    d.push(-3600);
    CHECK(d.output().state == State::NoSignal);
    bool saw_moving = false;
    for (size_t i = 0; i < 9; i++) {
        if (d.push(-1100).state == State::Moving) saw_moving = true;
    }
    CHECK(d.output().state == State::NoSignal);  // still warming up (10 samples)
    for (size_t i = 0; i < 100; i++) {
        if (d.push(-1100).state == State::Moving) saw_moving = true;
    }
    CHECK_FALSE(saw_moving);
    CHECK(d.output().state == State::Empty);
    CHECK(d.output().motion_cdb == 0);
    d.reset();
    d.push(-9000);  // a garbage sample right after reset is discarded too
    feed_constant(d, 200, -1100);
    CHECK(d.output().state == State::Empty);
    CHECK(d.output().motion_cdb == 0);
}

TEST_CASE("the detector rate is re-derived only when the measured message rate is more than 20% off and within range") {
    // Measured 4.9 Hz at 2.5 MHz against the nominal 10 Hz: the windows must follow reality.
    CHECK(rate_needs_reconfigure(5, 10));
    CHECK(rate_needs_reconfigure(50, 10));
    CHECK(rate_needs_reconfigure(13, 10));
    // Within 20%: leave the detector alone (a reconfigure resets its state).
    CHECK_FALSE(rate_needs_reconfigure(10, 10));
    CHECK_FALSE(rate_needs_reconfigure(9, 10));
    CHECK_FALSE(rate_needs_reconfigure(12, 10));
    // No samples yet, or more than the detector can hold: never.
    CHECK_FALSE(rate_needs_reconfigure(0, 10));
    CHECK_FALSE(rate_needs_reconfigure(51, 10));
}

TEST_CASE("the un-calibrated defaults sit about 3x above the idle baselines measured on the H4M") {
    // Bring-up 2026-09-25 (docs/superpowers/specs/2026-09-25-presence-bringup/NOTES.md): idle motion RMS
    // 40-80 cdB and idle still RMS 26-69 cdB on quiet and live bands; the old 150/40 tripped PRESENT on
    // every live band before calibration.
    CHECK(default_thr_motion_cdb == 250);
    CHECK(default_thr_still_cdb == 150);
}

TEST_CASE("the rate tracker re-derives the rate only after two consecutive windows agree on a >20% change") {
    RateTracker rt;
    // nominal 10 Hz configured, the capture chain really delivers 5 Hz: second agreeing window wins
    CHECK(rt.offer(5, 10) == 0);
    CHECK(rt.offer(5, 10) == 5);
    // now configured at 5 Hz: one burst window (queued messages after an M0 stall) must not reconfigure
    CHECK(rt.offer(8, 5) == 0);
    CHECK(rt.offer(5, 5) == 0);
    CHECK(rt.offer(8, 5) == 0);
    CHECK(rt.offer(5, 5) == 0);
    // a real, persistent change still gets through
    CHECK(rt.offer(10, 5) == 0);
    CHECK(rt.offer(10, 5) == 10);
    // out-of-range or empty windows never count and break the streak
    CHECK(rt.offer(0, 5) == 0);
    CHECK(rt.offer(60, 5) == 0);
    CHECK(rt.offer(10, 5) == 0);
}
