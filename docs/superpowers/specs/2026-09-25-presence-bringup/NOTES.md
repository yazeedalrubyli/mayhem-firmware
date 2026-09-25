# Presence app — bring-up notes (H4M, Mayhem v2.4.0 fork), 2026-09-25

Spec: `docs/superpowers/specs/2026-09-25-portapack-presence-app-design.md`.
Plan: `docs/superpowers/plans/2026-09-25-portapack-presence-app.md`.
Everything below was measured on the owner's PortaPack H4M over the USB serial console; every number has a
screenshot or log in this folder.

## 1. Verdict against the spec's success criteria (§1)

| # | Criterion | Result |
|---|-----------|--------|
| 1 | Room empty, calibrated: EMPTY for 60 s, no false MOVING | **Met** in walking tests 2 and 3 (70–130 s empty → EMPTY throughout). Not met in test 1 (uncalibrated + a reset bug, fixed). |
| 2 | Person walks 2 m from the device: MOVING within 2 s | **Not met** with calibrated thresholds. Test 2 (2462 MHz): motion score 1.9 dB against a 5.3 dB threshold. Test 3 (100 MHz): 0.3–0.8 dB against 1.5 dB. Test 1 (uncalibrated 2.5 dB) did flag the walk, but also flagged the empty room. |
| 3 | Person sits still 1–2 m away: PRESENT | **Not met**, and in tests 2 and 3 it could not have been met by construction: the build under test pushed the still deviation in Q8 through the 46340 clamp, which capped the still score at 1.81 dB, below the calibrated 3.84 / 2.27 dB thresholds (found in the final review, fixed in the follow-up commit: still is now carried in Q4 like motion, ceiling 28.96 dB). The still scores measured while sitting (0.60 dB in test 2, 0.43–0.69 dB in test 3) were under the cap and 3–6× below the thresholds, so the verdict stands on the data, but every still number in this folder came from the capped build. |
| 4 | Dead frequency or no antenna: NO SIGNAL, never EMPTY | **Met** for dead frequencies (5560, 806, 942, 3500 MHz all show NO SIGNAL with the −36 dB gate). "Antenna disconnected" was not exercised (needs hands on the device). |

The app's mechanics are all verified on the device (§4). What is missing is signal: in this room the only strong
band (WiFi channel 11, 2462 MHz) is packet-bursty, and its burst-to-burst variability with the room empty
(1.3–1.8 dB) is the same size as the change a person causes at 2 m (0.6–1.9 dB). The 3× rule the spec asks
for therefore sets thresholds above the human effect. FM broadcast at 100 MHz is continuous but only ~6 dB
above this receiver's floor with the bundled antennas, so its 0.5 dB wobble is receiver noise and a person
does not change it measurably.

Recommended way to get RuView-like results with this app (untested here, needs the owner):
put a phone across the room streaming video over **2.4 GHz** WiFi (a continuous, strong, distant
illuminator, which is what RuView's router provides), tune Presence to that channel's centre
(2412/2437/2462 MHz), press Cal with the room empty, then walk between the phone and the device. Closer
range (≤1 m) also helps: the human effect grows steeply as the person approaches the antenna.

## 2. Walking tests (protocol: Cal at 0 s, out until 20 s, walk past at 2 m 20–40 s, sit 1–2 m 40–70 s, out 70–130 s)

Screenshots every 5 s in `walk/`, `walk2/`, `walk3/` (each has a `montage.png` with the Cal row, state row and
numbers row of every shot).

| Test | Band / source / antenna | Thresholds | 0–20 s (out) | 20–40 s (walk) | 40–70 s (sit) | 70–130 s (out) |
|------|------------------------|-----------|--------------|----------------|---------------|----------------|
| 1 (14:50) | 2462 MHz PEAK 2.5M, rubber duck | defaults 2.5/1.5 (Cal lost to a detector reset at 10 s — bug, fixed by RateTracker) | PRESENT/CAL/NO SIGNAL/MOVING, EMPTY 20–30 s | MOVING 35, 45 s; EMPTY 40 s | EMPTY 50–60 s, mixed 65–80 s | EMPTY 85–105 s, second reset at 110 s, then MOVING 115–130 s with the room empty (−8 dB bursts from a nearby transmitter) |
| 2 (14:59) | 2462 MHz PEAK 2.5M, rubber duck | Cal ok: M 5.3 / S 3.84 dB (3× idle 1.77/1.28) | CALIBRATING then EMPTY | EMPTY (motion 1.9 dB) | EMPTY (still 0.60 dB) | EMPTY (1.0/0.98 dB) |
| 3 (15:11) | 100 MHz PEAK 2.5M, longest bundled whip | Cal ok: M 1.50 / S 2.27 dB | CALIBRATING then EMPTY | EMPTY (motion 0.4–0.8 dB) | EMPTY (still 0.43–0.69 dB) | EMPTY (0.4–0.7 / 0.43–0.65 dB) |

P stayed at −28/−29 dB for the whole of test 3: the whip gained ≤1 dB over the rubber duck at 100 MHz.

## 3. Bands surveyed (PEAK 2.5 MHz, LNA 32, VGA 32, AMP 0, rubber duck unless noted)

| Frequency | Held-max P | Trace floor | Note |
|-----------|-----------|-------------|------|
| 88 / 92 / 96 / 100 / 104 / 108 MHz | −31 / −31 / −32 / −29 / −31 / −34 | | FM broadcast, weak; 100 MHz best (`51_fm_band_88_118MHz.png`) |
| 112 / 118 MHz | −34 / −35 | | outside FM → the FM-band energy is real broadcast, not laptop EMI |
| 806 MHz (LTE B20 DL) | −36 | | NO SIGNAL (`50_scan_806MHz.png`) |
| 942 MHz (GSM/UMTS 900 DL) | −37 | | NO SIGNAL |
| 1100 MHz | −33 | −34 | |
| 1300 MHz | −31 | | bursty |
| 1450 MHz | −35 | −38 | |
| 1700 MHz | −30 | | bursty |
| 1850 / 2000 / 2140 MHz | −37 / −36 / −35 | −38 | |
| 2412 MHz (WiFi ch 1) | −17 (bursts −11) | −40 | |
| 2437 MHz (WiFi ch 6) | −33 (bursts −6) | | |
| **2462 MHz (WiFi ch 11)** | **−9** | −14 | strongest band in this room; emitter not identified (no ch-11 AP visible from the laptop) |
| 3500 MHz (5G n78) | −36 | | NO SIGNAL |
| 5560 MHz (spec default, ch 112) | −37 | −39 | NO SIGNAL; quiet-desk log `desk_quiet_peak.csv` (315 rows, 62.7 s, 5.01 Hz, median −37.0) |
| 5560 MHz MEAN 2.5M / 750k | −31.4 / −35.7 | −38 | spectrum mean includes bursts |

The receiver's own floor is −38…−39 dB everywhere from 1.1 to 5.6 GHz, so the NO SIGNAL gate is −36 dB
(`min_power_cdb = −3600`, floor + 3 dB). An ini written by the earlier build keeps `min_power_cdb=-9000`
and never shows NO SIGNAL: delete `/SETTINGS/rx_presence.ini` or edit the key.

Detector replica on `desk_quiet_peak.csv` (room quiet, 5560 MHz): motion RMS(2 s) median 0.55 dB, still
RMS(8 s) median 0.26 dB. The idle levels on live bands were 0.4–0.8 dB motion and 0.26–0.69 dB still, which
is why the un-calibrated defaults are now 2.5/1.5 dB (about 3× idle, the same rule Cal applies).

## 4. Verified on the device

- Defaults on first start (stale ini removed): 5560.0000 MHz, PEAK, 2.5M, NO SIGNAL for the 1 s warm-up,
  then the measured state; rate 4.9 Hz (`10_start.png`, `11_after_5s.png`, `12_after_15s.png`).
- Source/bandwidth switching and the measured message rates: PEAK 4.9 Hz at 2.5M, 9.9 Hz at 1.25M and
  750k; MEAN 37 Hz at 2.5M, 25.7 Hz at 1.25M, 50.6 Hz at 750k (`13_mean.png`, `14*.png`, `15_peak_again.png`).
  The MEAN rate is set by the M4's idle time (SpectrumCollector::update runs in the idle thread); the PEAK
  rate is the baseband's count-based 100 ms interval stretched by M4 load.
- Calibration round trip: Cal → CALIBRATING countdown → thresholds shown as `Cal: Mx.x Sy.yy` and saved to
  the ini; re-entering the app restores source, bandwidth, thresholds and a 5560 MHz frequency (which needs
  the 64-bit settings fix, `16_cal_2s.png` … `20_reentered.png`, ini read back over the console).
- Log to SD on/off (`23_log_on.png`, `24_log_off.png`) and the resulting CSV.
- NO SIGNAL gate: 5560 MHz → NO SIGNAL, 1100 MHz → signal (`40_gate_5560_nosignal.png`, `41_gate_1100_signal.png`).
- Console `setfreq` retunes the app (FreqChangeCommand handler) and a retune resets the detector instead of
  reporting the level jump as MOVING (`30_before_retune.png` vs `31_floor_peak_1300.png`).
- Serial-shell traffic while the app runs (screenshots every 5 s for 130 s, three times) never corrupted the
  display; it does stall the M0 event loop, which is what exposed the rate-adaptation reset (fixed).
- Final review (one Opus verifier, whole branch): one Important finding, the still-score cap above, fixed with a host test written first; minors deferred (an interrupted calibration shows the old thresholds as if finished; Log:on fails silently when the CSV cannot be created; `pp_serial.py` exits 0 when the console answers `error` / `file not found.` to `appstart` / `flash`; the sync-apps retry loop aborts if the port has not re-enumerated).
- Observation, not a code finding: the AMP field read 1 in a few 14:32–14:36 screenshots although the app
  never writes it; not reproducible from the console, most likely a physical touch. Toggling it back changed
  neither P nor the floor.

- Where the app lives on the device: main menu → **Receive** → **NEXT >** → **Presence** (second page, first
  column, second row, between Scanner and ACARS; `62_receive_menu_page2_presence.png`). Firmware in flash
  now reports version ee3c8107 with all 85 apps resynced from that build.

## 5. Toolchain and the deploy loop

- Build: `docker run -v "$PWD:/havoc" -u "$(id -u):$(id -g)" --rm portapack-dev ninja -j16` in `build/`
  (ARM GCC 9.2.1). `export_external_apps.py` fails the build if `presence.ppma` exceeds 32,768 bytes
  (it is 29,660 bytes after the review fix; app slot `0xAE040000`, the first free one after `p25_tx`).
- Host tests: `firmware/application/external/presence/test/run.sh` (g++ 13, doctest; 18 cases / 94 assertions).
- Device tool: `tools/presence/pp_serial.py` (`info`, `cmd`, `upload`, `verify`, `flash`, `wait`,
  `sync-apps`, `appstart`, `screenshot`, `button`, `touch`). Uploads are paced like hackrf.app on macOS
  (4096-byte `fwb` chunks written as 64-byte pieces with a 2 ms pause); a single 16 KiB write wedged the
  device's USB stack and needed a hardware reset. `verify` uses CRC-32/BZIP2, which is what the console's
  `crc32` computes. Every console reply without a prompt is an error (never silent success).
- A firmware change needs: upload `.bin`, `flash`, wait for the first boot (200–300 s on this device), then
  `sync-apps` (the menu and `applist` hide apps whose version hash differs from the firmware's; `appstart`
  still runs them). An app-only change needs only the new `presence.ppma` uploaded, **but only while no
  commit has been made since the firmware in flash was configured**: ninja re-runs the cmake configure when
  `HEAD` moves, which changes `VERSION_MD5` for every app, and the rebuilt app is then invisible in the
  menu (this bit me on 2026-09-25: after committing, the fixed app ran via `appstart` but did not appear
  under Receive until the firmware was reflashed and all apps resynced from the same build).
- Console facts learned the hard way: `button` numbers are 1 Right 2 Left 3 Down 4 Up 5 Select 6 Dfu
  (opens the debug overlay; press again to cycle it off) 7 encoder −1 8 encoder +1; `touch x y` is in
  screen pixels, view row r is at y = 16(r+1); `setfreq` parses with `atol`, so anything ≥ 2 147 483 648 Hz
  is clamped (use the ini for 2.4/5 GHz); `fopen` never truncates (the tool sends `ftruncate` before
  `fclose`); upstream's `crc32` command leaked a `File` per call (fixed in this fork); the device may come
  back as `/dev/ttyACM1` after a reboot (the tool finds it by USB ID 1d50:6018); the version and build-time
  strings lag the commit (captured at cmake configure time).
- Settings ini `/SETTINGS/rx_presence.ini` keys used by the app: `source` (0 PEAK, 1 MEAN), `bw_index`,
  `min_power_cdb`, `thr_motion_cdb`, `thr_still_cdb`, `log`, `rx_frequency`, `lna`, `vga`, `rx_amp`. Exit
  the app before replacing the file; exit rewrites it.
- Rollback: stock `portapack-mayhem_v2.4.0.bin` + `APPS/` from `OCI_hackrf_mayhem_v2.4.0.ppfw.tar`
  (sha256 01217eb8…65329); `stock_apps.txt` lists the stock `/APPS` set.

## 6. Artifact index

`01–05_*.png` first PEAK run and 2462 MHz; `10–20_*.png` Task 4 verification pass (defaults, MEAN/bw, Cal,
exit/re-enter); `23/24` log on/off; `30–32` retune and 1300 MHz floor; `40–42` NO SIGNAL gate and live 2462;
`50_scan_*` 100/806/942/3500 MHz; `51_fm_band_88_118MHz.png`; `desk_quiet_peak.csv`; `walk*/` the three
walking tests; `stock_apps.txt`.
