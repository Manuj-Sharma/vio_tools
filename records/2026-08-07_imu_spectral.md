# 2026-08-07 — IMU spectral analysis: the fan, a bug, and what flight vibration actually is

Follow-on from the 08-06 frequency work. Three results, one of which retracts a
conclusion committed the previous day.

## 1. The 16.669 Hz line in the static log is the Jetson cooling fan

`imu_static_20260806-185934` carries a spectral line on 5 of 6 axes at 16.669 Hz,
up to 271x its local noise floor, stationary across all 415 s, no harmonics,
5–17% of total variance.

Proven by re-recording with the fan at a different speed
(`imu_fantest_20260807-131518`, 300 s, fan RPM logged alongside at 1 Hz):

    fan 1806 rpm (logged)     @16.669 Hz        @30.060 Hz
      static 08-06            184–244x           1.4–1.9x
      fantest 08-07           1.6–2.3x           184–620x

The line moved with the fan. 16.669 Hz = 1000.1 RPM; 30.060 Hz = 1803.6 RPM,
inside the 1800–1813 RPM logged during the capture.

Ruled out along the way: dropped samples (survives resampling), stale-register
re-reads (zero duplicate samples), sample-clock locking (decimating to fs/2 leaves
it at 16.669 Hz, and it is NOT fs/15 despite 16.665 being suspiciously close), an
internal IMX-5 process (absent from the flight bag), and the daemon poll schedule
(`POLL_PERIOD_US 4000` with extra work every 2nd/5th/25th poll = 125/50/10 Hz, no
every-15th anything).

Mechanism is rotor imbalance coupling structurally into the IMU mount, which is
why it is a pure harmonic-free tone that tracks RPM linearly. Blade-pass would
put energy at integer multiples; there is none (every harmonic slot reads
2.4–3.8x floor, indistinguishable from noise).

CONSEQUENCE: the Allan noise densities from 08-06 are UPPER BOUNDS. Gyro N
1.19e-4 and accel N 5.65e-4 both include fan vibration the estimator cannot
separate from sensor noise. Redo with the fan stopped — and make it multi-hour
while you are at it, since 415 s only resolves tau ~41 s and leaves bias
instability and rate random walk unresolved.

The fan is a MEASUREMENT problem only. It is absent in flight, buried under prop
energy. Nothing in the flight configuration needs to change for it.

## 2. imu_spectrum.py was closing up dropped samples — FIXED

The IMU delivers ~238 Hz against a 250 Hz nominal, so ~5% of the grid is missing.
The tool took `fs` from the median dt and then handed the raw array to the FFT,
which silently closes the gaps. Every gap is a phase discontinuity.

Measured against synthetic data with a KNOWN line at 16.6688 Hz, 4.9% dropped:

    no gaps at all (control)              16.663 Hz   781x floor
    gaps closed up (the bug)              17.441 Hz    53x floor
    resampled onto the true grid          16.663 Hz   738x floor

+0.77 Hz of error and a 15x loss of contrast. Real data shows the same signature:
gyro x reads 16.861 Hz at 16.6x closed-up, 16.663 Hz at 247.9x resampled.

The Welch implementation itself is correct — verified against white noise
(measured flat PSD 3.2033e-08 vs theoretical 2*sigma^2/fs = 3.2000e-08, Parseval
ratio 0.9989) and a known sinusoid (integral over the peak = A^2/2 exactly).
The bug was upstream of it.

`imu_gap_compare.py` draws both versions overlaid so the effect is visible per
channel. Note its internal control: gyro z, the one channel with no line, is the
one channel resampling does not help (contrast 1.0x). Interpolation is not
inventing peaks.

## 3. RETRACTED: the flight bags are not aliasing

Commit 7ca33dc states that both flight bags show accel power piling against
Nyquist and concludes content above 125 Hz is folding down. That was the
gap-closing bug.

    seg01 gap-train peaks [Hz]:  1.22  49.65  1.46  1.83  99.31  1.71
    accel y  as-is      115.5  117.5   99.2   99.6
             resampled   34.1    6.8  103.4   35.4
    accel z  as-is      115.5  118.4  102.9  115.0
             resampled   38.3   41.8   41.9   45.3

The near-Nyquist peaks dissolve once the gaps are respected, and the 99 Hz feature
matches the dropout train's own 99.31 Hz line exactly. On the static bag the
`top10%` aliasing metric falls from 8.2–9.8% to 2.8–4.4% after the fix — the flag
was largely measuring the artifact it was supposed to warn about.

## 4. Flight vibration is broadband and tracks TILT, not flight phase

Also disproven: swept prop harmonics. There are no diagonal streaks in the
seg01 spectrogram; the ridge tracker wanders 60–125 Hz exactly as it does on
channels known to have no line. The 34–45 Hz humps are genuinely broadband.

What is there is a 20 dB swing in broadband level — 100x in power, 10x in
amplitude — with constant spectral shape. Real, not an artifact: quiet and loud
windows have identical sample-fill rates (89.7–92.1% vs 91.0–92.9%) and the
longest gap anywhere is 8 slots inside an 8.2 s window.

    predictor                     r
    mean tilt off level       +0.802     <- vib_dB = 0.52*deg - 20.8, r^2 0.644
    roll+pitch rate rms       +0.657
    tilt oscillation (sd)     +0.363
    climb rate                +0.123
    altitude                  +0.006

87.3% of the variance is WITHIN flight phases, 12.7% between — cruise alone
carries 16.6 of the 20.0 dB. The two quiet windows (t 50–75 s, 128–150 s) are
level flight between turns, 3–8 deg off vertical, ~101 m. The loud stretch
(t 85–110 s) holds 20–29 deg of tilt while oscillating.

In engineering units, 50–125 Hz band RMS:

    quiet    0.476 m/s^2 = 0.049 g
    typical  1.939 m/s^2 = 0.198 g
    loud     4.759 m/s^2 = 0.485 g

CONSEQUENCE, and the reason this matters more than the aliasing story did:
OpenVINS takes ONE fixed `noise_density` for the whole run while the actual
unmodelled input moves 10x in amplitude within a single flight — low when level,
high when tilted. The level segments are exactly where a nadir VIO is otherwise
at its best, so the mismatch is worst where the vision is most usable. That is a
candidate contributor to the run-to-run scale spread in README (`flight_D_best`
0.857–0.984). NOT demonstrated — the correlation is established, an adaptive
noise_density has not been tested.

## Open

  * what drives the tilt in the first place — needs RCOU throttle from the
    ArduPilot .bin; this bag has only /imu0, /cam0/image_raw and two baro topics
  * true IMU noise density — needs a fan-off multi-hour static log
  * whether scaling noise_density with commanded attitude improves the
    trajectory — an A/B on the replay harness

## Tools added

    imu_gap_compare.py     gap-closed vs resampled PSD, overlaid
    imu_spectrogram.py     STFT; --track follows a peak, --overlay draws an rpm log
    imu_plot.py            raw time series; zoom with --t0/--t1, --gaps marks dropouts
    imu_vibe_vs_alt.py     band level vs altitude and climb rate
    imu_window_probe.py    what happens in a given window: yaw, tilt, thrust proxy
    record_imu.sh          log /imu0 to a bag with fan RPM alongside (no sudo needed)
