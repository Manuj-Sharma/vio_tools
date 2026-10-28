#!/usr/bin/env python3
"""imu_poll_test.py [SECS=100] [PERIODS=4,2,3,4]

Does the driver's poll period change how many IMU samples we get?

For each period (ms) it starts imx5_spi_node ALONE (no cameras, no other load) with
`-p poll_period_ms:=P`, records /imu0 for SECS seconds, stops the node, and measures the bag.
Polling faster than the IMU makes samples can return the same sample twice, so the measure is
UNIQUE header stamps: unique-sample rate, duplicates, and the % of 4 ms grid slots with no sample
(the real loss), plus the unique rate per 20 s window. First and last 3 s are dropped.

Refuses to run if imx5_spi_node, a camera node or a recorder is already running (it needs the
SPI device to itself). Bags go to --out and are small (tens of MB per run).
"""
import glob, os, signal, sqlite3, statistics as st, struct, subprocess, sys, time

HOME = os.path.expanduser("~")
secs = int(sys.argv[1]) if len(sys.argv) > 1 else 100
periods = [int(x) for x in (sys.argv[2] if len(sys.argv) > 2 else "4,2,3,4").split(",")]
out = os.path.join(HOME, "rosbags", "imu_poll_" + time.strftime("%Y%m%d-%H%M%S"))
ENV = "source /opt/ros/humble/setup.bash; source %s/vio_ar0234_icm20948/install/setup.bash; " % HOME


def running(pat):
    return subprocess.run(["pgrep", "-x", pat], capture_output=True).returncode == 0


for p in ("imx5_spi_node", "vizionsdk_camer"):
    if running(p):
        sys.exit(f"{p} is running -- stop the sensors first (the node needs the SPI device to itself)")
if subprocess.run("pgrep -af 'ros2 bag record' | grep -v pgrep", shell=True, capture_output=True).stdout.strip():
    sys.exit("a bag recorder is already running")
os.makedirs(out)


def measure(bag, trim=3.0):
    rx, hs = [], []
    for f in sorted(glob.glob(bag + "/*.db3")):
        c = sqlite3.connect(f"file:{f}?immutable=1", uri=True)
        t = c.execute("select id from topics where name='/imu0'").fetchone()
        for ts, h in c.execute("select timestamp, substr(data,5,8) from messages where topic_id=? order by timestamp", (t[0],)):
            s, n = struct.unpack("<iI", h)
            rx.append(ts); hs.append(s * 10**9 + n)
    order = sorted(range(len(rx)), key=rx.__getitem__)
    rx = [rx[i] for i in order]; hs = [hs[i] for i in order]
    lo, hi = rx[0] + trim * 1e9, rx[-1] - trim * 1e9
    sel = [i for i, t in enumerate(rx) if lo <= t <= hi]
    seen, uniq = set(), []                       # first arrival of each distinct header stamp
    for i in sel:
        if hs[i] not in seen:
            seen.add(hs[i]); uniq.append(i)
    span = (rx[sel[-1]] - rx[sel[0]]) / 1e9
    dh = sorted(hs[i] for i in uniq)
    steps = [(b - a) / 1e6 for a, b in zip(dh, dh[1:])]
    lost = sum(max(0, round(s / 4.0) - 1) for s in steps if s > 6)
    wins = []
    t0 = rx[sel[0]]
    k = 0
    while t0 + k * 20e9 < rx[sel[-1]]:
        a, b = t0 + k * 20e9, min(t0 + (k + 1) * 20e9, rx[sel[-1]])
        if (b - a) / 1e9 >= 8:
            wins.append(sum(1 for i in uniq if a <= rx[i] < b) / ((b - a) / 1e9))
        k += 1
    return dict(n=len(sel), uniq=len(uniq), dups=len(sel) - len(uniq), hz=len(uniq) / span,
                miss=100.0 * lost / (len(uniq) + lost), worst=max(steps) if steps else 0, wins=wins)


results = []
for idx, P in enumerate(periods):
    bag = os.path.join(out, f"run{idx + 1}_poll{P}ms")
    log = open(bag + ".node.log", "w")
    node = subprocess.Popen(["bash", "-c", ENV + "exec ros2 run imx5_spi_driver imx5_spi_node --ros-args "
                             f"-p clock_source:=monotonic -p frame_id:=imu_link -p use_imu_raw:=true -p poll_period_ms:={P}"],
                            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
    up = False
    for _ in range(30):
        time.sleep(1)
        if subprocess.run(["bash", "-c", ENV + "ros2 topic list 2>/dev/null | grep -qx /imu0"]).returncode == 0:
            up = True; break
    if not up:
        node.send_signal(signal.SIGINT); sys.exit(f"/imu0 never appeared for poll {P} ms, see {bag}.node.log")
    time.sleep(3)
    print(f"[poll] run {idx + 1}/{len(periods)}: poll_period_ms={P}, recording {secs} s at {time.strftime('%H:%M:%S')}", flush=True)
    rec = subprocess.Popen(["bash", "-c", ENV + f"exec ros2 bag record -o {bag} /imu0"], stdin=subprocess.DEVNULL,
                           stdout=open(bag + ".rec.log", "w"), stderr=subprocess.STDOUT)
    time.sleep(secs)
    rec.send_signal(signal.SIGINT)
    try:
        rec.wait(timeout=60)
    except subprocess.TimeoutExpired:
        rec.kill()
    node.send_signal(signal.SIGINT)
    try:
        node.wait(timeout=15)
    except subprocess.TimeoutExpired:
        subprocess.run(["pkill", "-x", "imx5_spi_node"])
    time.sleep(2)
    r = measure(bag)
    r["P"] = P
    results.append(r)
    print(f"[poll]   {r['n']} msgs, {r['uniq']} unique, {r['dups']} duplicates -> unique {r['hz']:.1f} Hz, "
          f"missing {r['miss']:.1f}%, worst step {r['worst']:.0f} ms", flush=True)

print("\npoll   unique Hz  missing%  duplicates  worst step   per-20 s unique Hz")
for r in results:
    print(f"{r['P']:>2} ms  {r['hz']:9.1f}  {r['miss']:8.1f}  {r['dups']:10d}  {r['worst']:7.0f} ms   " +
          " ".join(f"{w:.0f}" for w in r["wins"]))
print(f"\nbags and node logs kept in {out}")
