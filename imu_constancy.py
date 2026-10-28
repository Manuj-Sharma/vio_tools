#!/usr/bin/env python3
"""imu_constancy.py [SECS=600] [--win 20] [--keep-dir DIR]

Is the /imu0 rate constant over time? Records ONLY /imu0 (tiny load: ~60 kB/s) for SECS seconds while
the rig's sensors keep running as they are, and in the same period logs, once per 2 s, the hottest
thermal zone, the lowest/highest CPU clock, the 1-minute load average and the VDD_IN voltage and
current. Then prints one row per WIN-second window:

    imu Hz / miss% / worst gap   from the bag (receive times; miss% from the 4 ms header-stamp grid)
    driver Hz                    what imx5_spi_node itself logged (5 s reports) in that window
    Tj / CPUmin / load / VDD     the machine state in that window

and a summary: mean, standard deviation, min and max of the per-window IMU rate.

Needs the sensors running (imx5_spi_node). Never touches them. The bag (a few tens of MB) is kept.
"""
import argparse, glob, os, re, signal, sqlite3, statistics as st, struct, subprocess, sys, time

HOME = os.path.expanduser("~")
SENSORS_LOG = os.environ.get("SENSORS_LOG", "/tmp/vio_logs/sensors.log")


def hw_by_name(name):
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        try:
            if open(h + "/name").read().strip() == name:
                return h
        except OSError:
            pass
    return None


def rd(path, default=0):
    try:
        return int(open(path).read().strip())
    except (OSError, ValueError):
        return default


def sample(ina):
    tj = max([rd(z) for z in glob.glob("/sys/class/thermal/thermal_zone*/temp")] or [0]) / 1000
    fr = [rd(f) for f in glob.glob("/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq")]
    load = float(open("/proc/loadavg").read().split()[0])
    mv = rd(ina + "/in1_input") if ina else 0
    ma = rd(ina + "/curr1_input") if ina else 0
    return (time.time(), tj, min(fr) / 1000 if fr else 0, load, mv, ma)


def load_bag(bag):
    rx, hs = [], []
    for f in sorted(glob.glob(os.path.join(bag, "*.db3"))):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        t = c.execute("select id from topics where name='/imu0'").fetchone()
        for ts, h in c.execute("select timestamp, substr(data,5,8) from messages where topic_id=? order by timestamp", (t[0],)):
            s, n = struct.unpack("<iI", h)
            rx.append(ts); hs.append(s * 10**9 + n)
    order = sorted(range(len(rx)), key=rx.__getitem__)
    return [rx[i] for i in order], [hs[i] for i in order]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("secs", nargs="?", type=int, default=600)
    ap.add_argument("--win", type=float, default=20.0)
    ap.add_argument("--keep-dir", default=os.path.join(HOME, "rosbags", "imu_constancy_" + time.strftime("%Y%m%d-%H%M%S")))
    a = ap.parse_args()

    if subprocess.run(["pgrep", "-x", "imx5_spi_node"], capture_output=True).returncode != 0:
        sys.exit("imx5_spi_node is not running -- start the sensors first")
    if subprocess.run("pgrep -af 'ros2 bag record' | grep -v pgrep", shell=True, capture_output=True).stdout.strip():
        sys.exit("a bag recorder is already running")
    os.makedirs(a.keep_dir, exist_ok=True)
    bag = os.path.join(a.keep_dir, "imu_only")
    log_off = os.path.getsize(SENSORS_LOG) if os.path.exists(SENSORS_LOG) else 0
    ina = hw_by_name("ina3221")

    cmd = ("source /opt/ros/humble/setup.bash; source %s/vio_ar0234_icm20948/install/setup.bash; "
           "exec ros2 bag record -o %s /imu0" % (HOME, bag))
    rec = subprocess.Popen(["bash", "-c", cmd], stdin=subprocess.DEVNULL,
                           stdout=open(os.path.join(a.keep_dir, "rec.log"), "w"), stderr=subprocess.STDOUT)
    print(f"[constancy] recording /imu0 only for {a.secs} s -> {bag}  (started {time.strftime('%H:%M:%S')})", flush=True)
    tel = []
    t_end = time.time() + a.secs
    while time.time() < t_end:
        tel.append(sample(ina))
        time.sleep(2)
    rec.send_signal(signal.SIGINT)            # clean SIGINT => metadata.yaml is written
    try:
        rec.wait(timeout=60)
    except subprocess.TimeoutExpired:
        rec.kill()

    rx, hs = load_bag(bag)
    if len(rx) < 100:
        sys.exit(f"only {len(rx)} messages recorded, see {a.keep_dir}/rec.log")
    drv = []
    pat = re.compile(r"\[(\d+\.\d+)\] \[imx5_spi_node\]: /imu0 ([\d.]+) Hz")
    with open(SENSORS_LOG, errors="replace") as fh:
        fh.seek(log_off)
        for line in fh:
            m = pat.search(line)
            if m:
                drv.append((float(m.group(1)), float(m.group(2))))

    t0 = rx[0]; W = int(a.win * 1e9); rows = []
    k = 0
    while t0 + k * W < rx[-1]:
        lo, hi = t0 + k * W, min(t0 + (k + 1) * W, rx[-1] + 1)
        span = (hi - lo) / 1e9
        if span < 5:
            break
        ix = [i for i, t in enumerate(rx) if lo <= t < hi]
        r = [rx[i] for i in ix]; h = [hs[i] for i in ix]
        gaps = [(y - x) / 1e6 for x, y in zip(r, r[1:])]
        lost = sum(max(0, round(((y - x) / 1e6) / 4.0) - 1) for x, y in zip(h, h[1:]) if (y - x) / 1e6 > 6)
        d = [hz for t, hz in drv if lo / 1e9 < t <= hi / 1e9]
        ts = [s for s in tel if lo / 1e9 <= s[0] < hi / 1e9]
        rows.append((k * a.win, span, len(ix) / span, 100.0 * lost / (len(ix) + lost), max(gaps) if gaps else 0,
                     st.mean(d) if d else None, max(s[1] for s in ts) if ts else None,
                     min(s[2] for s in ts) if ts else None, st.mean(s[3] for s in ts) if ts else None,
                     min(s[4] for s in ts) if ts else None))
        k += 1

    f = lambda x, p: "  -  " if x is None else p.format(x)
    print(f"\n{len(rx)} /imu0 messages, windows of {a.win:g} s (first message = 0)")
    print(f"{'win':>9} | {'imu Hz':>7} {'miss%':>6} {'gap ms':>7} | {'driver':>6} | {'Tj C':>4} {'CPUmin':>6} {'load':>5} {'VDD mV':>7}")
    for s, span, hz, ms, wg, dh, tj, fm, ld, mv in rows:
        print(f"{s:4.0f}-{s + span:<4.0f} | {hz:7.1f} {ms:6.1f} {wg:7.0f} | {f(dh, '{:.0f}'):>6} | {f(tj, '{:.0f}'):>4} {f(fm, '{:.0f}'):>6} {f(ld, '{:.1f}'):>5} {f(mv, '{:.0f}'):>7}")
    hz = [r[2] for r in rows]
    print(f"\nper-window IMU rate: mean {st.mean(hz):.1f} Hz  std {st.pstdev(hz):.1f}  min {min(hz):.1f}  max {max(hz):.1f}"
          f"   windows below 225 Hz: {sum(1 for x in hz if x < 225)}/{len(hz)}   windows with a gap > 100 ms: {sum(1 for r in rows if r[4] > 100)}/{len(rows)}")
    print(f"bag + logs kept in {a.keep_dir}")


if __name__ == "__main__":
    main()
