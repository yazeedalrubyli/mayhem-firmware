# PortaPack "Presence" app — design spec

Date: 2026-09-25
Repo: `yazeedalrubyli/mayhem-firmware`, trunk `main` cut from upstream tag `v2.4.0` (5c71964).
Target device: PortaPack H4M on HackRF One, Mayhem v2.4.0 (verified over USB serial, `/dev/ttyACM0`).
Reference project: RuView (ruvnet/RuView) "RSSI mode": presence and motion from received-power fluctuations.

## 1. Goal and scope

A Mayhem external app named **Presence** that turns the handheld into a passive RF presence and motion
sensor. It listens to a transmitter that already exists in the home (the WiFi router by default, or a
cellular downlink carrier), tracks how the received in-band power fluctuates over seconds, and shows on the
H4M screen whether the space around the device is **EMPTY**, someone is **PRESENT** but still, or someone is
**MOVING**, with a confidence number, two live score meters, and a scrolling power trace.

The handheld only listens. One HackRF is half-duplex, so it cannot illuminate and receive at the same time;
this was decided with the owner ("Handheld listens only").

Out of scope (physically impossible on a 1x1, 8-bit, receive-only HackRF, or not wanted):
pose estimation, vital-sign numbers, person counting, any transmission, any laptop involvement at run time,
any change to the shared baseband code or to other apps.

Success criteria (walking test in the owner's home, device on a table, router as illuminator):

1. Room empty, calibrated: state stays EMPTY for 60 s with no false MOVING.
2. A person walks 2 m from the device: MOVING within 2 s of starting to walk, back to EMPTY or PRESENT
   within 3 s after stopping.
3. A person sits still 1–2 m from the device for 30 s: PRESENT for most of that time (still detection is
   best-effort and is expected to work reliably only with the MEAN source on a continuous carrier).
4. Antenna disconnected or tuned to a dead frequency: NO SIGNAL, never EMPTY.

## 2. Where it lives

```
firmware/application/external/presence/
  main.cpp             application_information_t + initialize_app (same shape as external/level/main.cpp)
  ui_presence.hpp/.cpp PresenceView: radio setup, message handlers, widgets
  presence_dsp.hpp/.cpp pure C++17, integer-only, no firmware includes; the whole detector
firmware/application/external/presence/test/test_presence_dsp.cpp   doctest host tests for presence_dsp
firmware/application/external/presence/test/run.sh                 builds + runs them with the host g++
```

Registration follows the existing pattern exactly:

- `firmware/application/external/external.cmake`: add the three `.cpp` files to `EXTCPPSRC` and `presence`
  to `EXTAPPLIST`.
- `firmware/application/external/external.ld`: one new 32k slot
  `ram_external_app_presence (rwx) : org = 0xAE000000, len = 32k` (all 79 slots `0xADB10000..0xADFF0000` are
  taken at v2.4.0; `external_app_info.py` allows up to `0xAE020000`, so `0xAE000000` and `0xAE010000` are free
  without touching the tool) plus the matching `SECTIONS` entry with
  `KEEP(*(.external_app.app_presence.application_information))`.
- `application_information_t`: `app_name "Presence"`, `menu_location app_location_t::RX`,
  `desired_menu_position -1`, `icon_color` green, 16x16 bitmap (person with two wave arcs),
  `m4_app_tag {'P','C','A','P'}`, `m4_app_offset 0`.

Baseband: the stock **capture** image (`portapack::spi_flash::image_tag_capture`), started with
`baseband::run_image(...)` exactly as Level (SPEC mode) and Detector RX do. No custom baseband, no image
switching at run time, no edits under `firmware/baseband/` or `firmware/common/`.

Budget: `export_external_apps.py` refuses an app whose `.ppma` (app code + capture image) exceeds 32 KiB at
v2.4.0. Detector RX ships with the same image and fits, so the app code has to stay at or below Detector RX's
size. The "Log to SD" feature in §4.5 is the first thing to drop if the budget is exceeded.

## 3. Measurement: what the capture baseband gives us

Both measurements below come from the stock capture image at the same time; the app selects which one
drives the detector. Both facts were verified in `firmware/baseband/proc_capture.cpp`,
`channel_stats_collector.hpp` and `spectrum_collector.cpp` at v2.4.0.

| Source | Message | Rate | Resolution | Time coverage | Right for |
|---|---|---|---|---|---|
| **PEAK** (default) | `ChannelStatistics.max_db` | 10 Hz (100 ms windows) | 1 dB integer | every sample in the window | packet illuminators: WiFi beacons and traffic |
| **MEAN** | `ChannelSpectrum.db[256]` | 50 Hz | 0.2 dB per bin, ~0.02 dB after averaging | one 256-sample snapshot per spectrum (~1% of the time) | continuous illuminators: LTE/5G downlink, broadcast carriers |

Why both: a WiFi router idles at ~10 beacons/s of ~1 ms each. The 50 Hz spectrum is a snapshot of
256 decimated samples, so it almost never lands on a packet and would mostly show the noise floor.
The channel statistics scan every sample and their peak is the strongest packet in each 100 ms window,
which for a nearby router is that router's beacon. Conversely a cellular downlink is on all the time, and
there the 0.2 dB spectrum bins averaged over ~180 bins give far finer resolution than the 1 dB peak.

MEAN in-band power per spectrum: bins are in FFT order (bin 0 = DC, 1..127 positive, 128..255 negative).
The app averages bins 4..95 and 160..251 (the inner 75 % of the band, excluding DC ±4 bins where the
HackRF DC spike sits) — 184 bins. Value in centi-dB: `mean_cdb = 20 * sum / 184 - 5100`
(bin value `v = 5*dB + 255`, so `v` units are 0.2 dB and `v = 0` is −51 dB).

PEAK in centi-dB: `max_db * 100`.

Radio configuration (same calls as Level SPEC mode): `receiver_model.set_modulation(Capture)`;
bandwidth field selects the decimated output rate `r` ∈ {750k, 1.25M, 2.5M, 5M};
`baseband::set_sample_rate(r, get_oversample_rate(r))`; `receiver_model.set_sampling_rate(get_actual_sample_rate(r))`;
`receiver_model.set_baseband_bandwidth(filter_bandwidth_for_sampling_rate(actual))`; `receiver_model.enable()`.
Default 2.5M (ADC at 10 MHz), which covers a good part of one 20 MHz WiFi channel without the
20 MSPS load of the 5M option. Spectrum streaming (`baseband::spectrum_streaming_start/stop`) runs only
while the MEAN source is selected.

Default frequency: **5560 MHz** (the owner's own router "TIM-5G" on 5 GHz channel 112, the strongest
in-home transmitter seen from the laptop; 2.4 GHz channels 1/11 and channel 124 at 5620 MHz are the
building's networks). The frequency field is free to change; the gains (LNA/VGA/AMP) default to LNA 32,
VGA 20, AMP off and are persisted.

## 4. The detector (`presence_dsp`)

Pure C++17, integer arithmetic only (the M0 application core has no FPU), no dynamic allocation after
construction, no firmware headers, so the same file compiles and runs under the host doctest harness.

### 4.1 Interface

```cpp
namespace presence_dsp {
enum class State : uint8_t { NoSignal, Calibrating, Empty, Present, Moving };
struct Config {
    uint16_t rate_hz;        // 10 (PEAK) or 50 (MEAN); sets window sizes and EMA shifts
    uint8_t  hold_n;         // max-hold length in samples: 2 for PEAK, 1 for MEAN
    int32_t  min_power_cdb;  // below this smoothed power the state is NoSignal
};
struct Output {
    State   state;
    uint8_t confidence_pct;  // 0..100 for the active state, 0 for NoSignal/Calibrating
    int32_t power_cdb;       // mid EMA of the (held) input
    int32_t motion_cdb;      // motion score (RMS, centi-dB)
    int32_t still_cdb;       // still-presence score (RMS, centi-dB)
    int32_t thr_motion_cdb, thr_still_cdb;
    uint16_t cal_remaining;  // samples left in calibration, 0 when not calibrating
};
class Detector {
  public:
    explicit Detector(const Config&);
    void reset();
    void start_calibration();                // 5 s leave-the-area delay + 8 s measurement
    void set_thresholds(int32_t motion_cdb, int32_t still_cdb); // restore persisted calibration
    Output push(int32_t power_cdb);          // one sample at rate_hz; returns the new output
    const Output& output() const;
};
}
```

### 4.2 Pipeline per sample

1. **Hold**: `h = max(x[n], …, x[n-hold_n+1])`. With PEAK the beacon interval (102.4 ms) is slightly longer
   than the 100 ms window, so one window in ~50 has no beacon from the dominant router; hold 2 removes
   the resulting dip.
2. **Three EMAs** on `h`, fixed point Q8, `alpha = 1/2^k`, `tau = 2^k / rate`:

   | | 10 Hz (PEAK) | 50 Hz (MEAN) |
   |---|---|---|
   | fast | k=2, 0.4 s | k=4, 0.32 s |
   | slow | k=5, 3.2 s | k=7, 2.56 s |

3. **Motion score** = RMS over the last 2 s of `(h[n] − h[n − lag])` with `lag = rate/5` samples (0.2 s at
   both rates): a rate-independent high-pass that passes walking-band swings (0.5–3 Hz) and attenuates
   the breathing band; it has nulls at multiples of 5 Hz, above any human motion.
4. **Still score** = RMS over the last 8 s of `(ema_fast − ema_slow)`: a band-pass around 0.1–0.5 Hz,
   the breathing / body-sway band, which walking also excites.
   Both RMS windows are ring buffers of squared deviations with int64 running sums and an integer square
   root; they use `min(count, window)` samples so scores are valid from the first sample.
5. **Calibration** (`start_calibration`): state `Calibrating` for `5 s` (owner leaves the area) then `8 s`
   during which the motion and still scores are averaged into `base_m`, `base_s`. Then
   `thr_motion = max(3 * base_m, 20 cdB)`, `thr_still = max(3 * base_s, 5 cdB)`. Until a calibration is
   run or restored, the defaults are `thr_motion = 150 cdB`, `thr_still = 40 cdB`.
6. **Decision** with hysteresis and hold:
   - `NoSignal` if `ema_slow < min_power_cdb` (takes priority over everything except Calibrating).
   - `Moving` entered when `motion ≥ thr_motion`; left when `motion < 0.7 * thr_motion` for 1 s.
   - else `Present` entered when `still ≥ thr_still`; left when `still < 0.7 * thr_still` for 3 s.
   - else `Empty`.
   - `confidence_pct = min(100, 50 * score / thr)` for the active state's score (100 % at twice the threshold).

### 4.3 Host tests (doctest, standalone runner)

Synthetic inputs are hermetic unit fixtures, allowed by the owner's rules; nothing synthetic runs on the device.

- constant input → `Empty`, scores 0, confidence 0.
- input below `min_power_cdb` → `NoSignal`.
- 1 Hz square wave ±300 cdB at 10 Hz → `Moving` within 2 s, confidence 100.
- 0.3 Hz sine ±30 cdB on a −4000 cdB base at 50 Hz → `Present` within 8 s, never `Moving`.
- calibration: 13 s of 1 dB quantised noise (PEAK-like flips) then thresholds ≥ 3× the measured baselines
  and the same noise afterwards stays `Empty`.
- hold: a single-sample dip of −1000 cdB every 50 samples with `hold_n = 2` produces no `Moving`.
- hysteresis: score decaying from 1.2× to 0.8× threshold keeps the state for the hold time, then drops.
- `set_thresholds` restores values and skips calibration.

### 4.4 Screen (240×320, 30 columns × 20 rows of 8×16)

```
row 0   LNA:32 VGA:20 AMP:0            (LNAGainField, VGAGainField, RFAmpField)
row 1   5560.000M  PEAK  2.5M          (FrequencyField, OptionsField source, OptionsField bandwidth)
row 2   [ Cal ]  Cal: leave area 4s    (Button, Text)
row 3-5 ┌───────────────────────────┐  StatusBanner: filled box, colour by state, big label + "87%"
        │      M O V I N G     87%  │  grey NO SIGNAL, blue CALIBRATING, green EMPTY,
        └───────────────────────────┘  yellow PRESENT, red MOVING
row 6-7 Motion [████████|▒▒▒▒]  1.8dB  ScoreBar: bar scaled to 2×threshold, tick at threshold
row 8-9 Still  [███|▒▒▒▒▒▒▒▒▒]  0.2dB  ScoreBar
row 10-18 power trace, 240 px wide × 144 px, last 240 samples (24 s PEAK / 4.8 s MEAN),
          autoscaled to min..max ± 1 dB, redrawn at most 10 times per second
row 19  P -43.2  M 1.8  S 0.21  10.0Hz  live power, scores, measured input rate
```

Widgets: existing `Labels`, `Text`, `Button`, `FrequencyField`, `OptionsField`, `LNAGainField`,
`VGAGainField`, `RFAmpField`; three small custom widgets in `ui_presence.cpp` (`StatusBanner`, `ScoreBar`,
`TraceWidget`) drawn with `Painter::fill_rectangle`, `draw_hline`, `draw_vline`, `draw_string`.

Input plumbing: `MessageHandlerRegistration` for `ChannelStatistics` (PEAK: one `push` per message),
`ChannelSpectrumConfig` (stores the FIFO) and `DisplayFrameSync` (MEAN: drain the spectrum FIFO, one
`push` per spectrum; both sources: throttle redraws). The measured input rate (pushes per second via
`chTimeNow()`) is displayed so a starving source is visible.

Settings persisted with `app_settings::SettingsManager settings_{"rx_presence", app_settings::Mode::RX, ...}`
(frequency and gains come with Mode::RX): `source`, `bandwidth index`, `min_power_cdb`,
`thr_motion_cdb`, `thr_still_cdb`, `log_to_sd`. Restored thresholds are applied with `set_thresholds`.

Changing source or bandwidth resets the detector (window sizes change) and keeps the thresholds.

### 4.5 Log to SD (optional, first to drop for size)

An OptionsField `Log {off,on}`. When on, each pushed sample is appended as `"<ms>,<power_cdb>\n"` to
`/PRESENCE/presence_<n>.csv` (new file per app start). Purpose: pull the raw stream over serial
(`fopen`/`fread`) to tune thresholds against real data instead of guessing.

## 5. Build, deploy, verify

Build (already proven on this laptop): Docker image `portapack-dev` from `dockerfile-nogit`
(Ubuntu noble, ARM GCC 9-2019-q4-major, CMake 3.28, ninja), run as
`docker run -v "$PWD:/havoc" -u "$(id -u):$(id -g)" --rm portapack-dev ninja -j24`
with all submodules initialised recursively (`hackrf` and its nested `libopencm3`; missing them fails
configure with "No SOURCES given to target"). Outputs:
`build/firmware/portapack-mayhem-firmware.bin`, `build/firmware/application/presence.ppma`,
`build/firmware/firmware_tar/APPS/*.ppma`.

Host tests: `firmware/application/external/presence/test/run.sh` (native g++ -std=c++17 against the repo's
`firmware/test/include/doctest.h`; the upstream `firmware/test` CMake tree does not build at v2.4.0).

Deploy over the USB serial console (`/dev/ttyACM0`, 115200), no button pressing needed:

1. Back up: confirm `FIRMWARE/portapack-mayhem_v2.4.0.bin` exists on the SD (copy the stock release there
   if not) — that is the rollback image, flashed with `flash /FIRMWARE/portapack-mayhem_v2.4.0.bin`.
2. Upload the new firmware `.bin` to `/FIRMWARE/presence-<sha>.bin` with `fopen` + `fwb` chunks + `fclose`,
   verify with `crc32` and `filesize` against the local file.
3. `flash /FIRMWARE/presence-<sha>.bin` (device reboots into the new firmware).
4. Replace **all** of `/APPS/*.ppma` with the new build's set (every `.ppma` embeds `VERSION_MD5`, the old
   ones are rejected by the new firmware): `unlink` each, upload each.
5. Verify: `info` shows the new version string; `applist` lists `presence`; `appstart presence`;
   `screenframeshort` decoded to PNG shows the Presence screen; drive `button`/`touch` to change source and
   press Cal, screenshot again.

Walking test with the owner (§1 success criteria), reading state from screenshots over serial while the
owner moves as instructed, then a final photo/screenshot set kept under `docs/superpowers/specs/`.

Rollback: `flash /FIRMWARE/portapack-mayhem_v2.4.0.bin` then restore the stock `/APPS` set from the
release tar.

## 6. Risks and how they are handled

- **32 KiB app + capture image budget**: Detector RX proves the image fits with a modest app; size is
  checked at build time by `export_external_apps.py`; drop Log-to-SD, then the trace widget, if needed.
- **Bursty illuminator**: handled by PEAK (full coverage) + hold; if the owner's router beacons are too
  weak at the chosen spot, switch the frequency to the building network or a cellular downlink with MEAN.
- **Other transmitters near the device** (phone traffic) will dominate PEAK windows and show as motion;
  calibration with the phone in its normal place absorbs the typical variance; documented in the app's
  help text and in this spec.
- **Threshold defaults are guesses until bring-up**: `min_power_cdb`, the un-calibrated thresholds and the
  default gains are revised from real on-device readings in the bring-up step and committed.
- **Timing**: `ChannelStatistics` at exactly 10 Hz and spectra at ~50 Hz are nominal; the app displays the
  measured rate and the detector's windows are defined in samples, so a slower rate only stretches the
  time constants.

## 7. Follow-ups (not in this spec)

- Per-packet RSSI clustering in a custom baseband (select one router's beacons by burst duration) if
  PEAK proves too noisy in busy homes.
- Upstream PR to portapack-mayhem once the app has been used for a while.
