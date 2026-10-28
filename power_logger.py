#!/usr/bin/env python3
"""power_logger.py [SECS=0] [--out /dev/shm/power_log.csv] [--period 1.0]

Once-per-period log of the Jetson's power state, read straight from sysfs (read-only, no ROS):
VDD_IN volts and amps (INA3221), CPU/GPU and SOC rail amps, CPU clock min/max, Tj, 1-min load,
and the three soctherm over-current event counters (oc1..oc3_event_cnt).

Writes ONLY to RAM (/dev/shm) so it cannot be blocked by the disk. SECS=0 runs until Ctrl+C.
Prints a summary at the end (min/mean/max, sag, new over-current events) and the CSV path.
Copy the CSV to disk before shutdown -- RAM files vanish.
"""
import glob, os, sys, time

secs = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else 0
out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else "/dev/shm/power_log.csv"
period = float(sys.argv[sys.argv.index("--period") + 1]) if "--period" in sys.argv else 1.0


def hwmon(name):
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        if open(h + "/name").read().strip() == name:
            return h
    sys.exit(f"hwmon '{name}' not found")


ina, oc = hwmon("ina3221"), hwmon("soctherm_oc")
cpus = sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"))
tj = next((z for z in glob.glob("/sys/devices/virtual/thermal/thermal_zone*")
           if open(z + "/type").read().strip() == "tj-thermal"), None)


def rd(path, default=float("nan")):
    try:
        return float(open(path).read())
    except (OSError, ValueError):
        return default


def sample():
    v = rd(ina + "/in1_input") / 1000.0          # mV -> V
    a = rd(ina + "/curr1_input") / 1000.0        # mA -> A
    freqs = [rd(c) / 1000.0 for c in cpus]       # kHz -> MHz
    return dict(
        t=time.time(), vdd_v=v, vdd_a=a, vdd_w=v * a,
        cpugpu_a=rd(ina + "/curr2_input") / 1000.0, soc_a=rd(ina + "/curr3_input") / 1000.0,
        cpu_min=min(freqs), cpu_max=max(freqs),
        tj=rd(tj + "/temp") / 1000.0 if tj else float("nan"),
        load1=os.getloadavg()[0],
        oc1=rd(oc + "/oc1_event_cnt"), oc2=rd(oc + "/oc2_event_cnt"), oc3=rd(oc + "/oc3_event_cnt"))


rows = []
f = open(out, "w")
keys = list(sample().keys())
f.write(",".join(keys) + "\n")
t0 = time.time()
try:
    while secs == 0 or time.time() - t0 < secs:
        s = sample()
        rows.append(s)
        f.write(",".join(f"{s[k]:.3f}" for k in keys) + "\n"); f.flush()
        time.sleep(max(0.0, period - (time.time() - s["t"])))
except KeyboardInterrupt:
    pass
f.close()
if not rows:
    sys.exit(0)


def stat(k):
    xs = [r[k] for r in rows]
    return min(xs), sum(xs) / len(xs), max(xs)


print(f"\n{len(rows)} samples over {rows[-1]['t'] - rows[0]['t']:.0f} s -> {out}")
for k, label in [("vdd_v", "VDD_IN V"), ("vdd_a", "VDD_IN A"), ("vdd_w", "VDD_IN W"), ("cpugpu_a", "CPU/GPU A"),
                 ("soc_a", "SOC A"), ("cpu_min", "CPU MHz min"), ("cpu_max", "CPU MHz max"),
                 ("tj", "Tj C"), ("load1", "load1")]:
    lo, mean, hi = stat(k)
    print(f"  {label:12s} min {lo:8.2f}  mean {mean:8.2f}  max {hi:8.2f}")
new_oc = [int(rows[-1][k] - rows[0][k]) for k in ("oc1", "oc2", "oc3")]
print(f"  over-current events during log: oc1 +{new_oc[0]}  oc2 +{new_oc[1]}  oc3 +{new_oc[2]}")
