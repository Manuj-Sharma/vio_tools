#!/usr/bin/env python3
"""Temperature logger for the IMU-rate investigation (read-only: sysfs + the node log, no SPI, no ROS).

  temp_watch.py [--interval 5] [--print-every 60] [--csv /tmp/vio_logs/temp_watch.csv] [--log /tmp/vio_logs/sensors.log] [--tj-alert 82]
Every --interval s appends a CSV row: IMX die temp (last `imx_status:` line of the imx5_spi_node log), Jetson tj/cpu/gpu/soc0 zones,
fan rpm and the CPU0 frequency. Prints one line every --print-every s (IMX temp with its change over the last 5 min) and an ALERT if Tj >= --tj-alert
or the IMX temp moved more than 1.5 C in 5 min. Join the CSV with rate_watch*.csv by time to look for temperature-dependent dips.
"""
import argparse, os, re, time, glob, csv, collections
ap = argparse.ArgumentParser()
ap.add_argument("--interval", type=float, default=5.0); ap.add_argument("--print-every", type=float, default=60.0)
ap.add_argument("--csv", default="/tmp/vio_logs/temp_watch.csv"); ap.add_argument("--log", default="/tmp/vio_logs/sensors.log")
ap.add_argument("--tj-alert", type=float, default=82.0); a = ap.parse_args()
ANSI = re.compile(r"\x1b\[[0-9;]*m"); IMX = re.compile(r"imx_status:.*?imxT=([\d.]+)C.*?Tj=([\d.]+)C fan=(\d+)/255")
zones = {}
for z in glob.glob("/sys/class/thermal/thermal_zone*"):
    try: zones[open(z + "/type").read().strip()] = z + "/temp"
    except OSError: pass
def zt(name):
    try: return float(open(zones[name]).read()) / 1000.0
    except (KeyError, OSError, ValueError): return float("nan")
def fan_rpm():
    for p in glob.glob("/sys/class/hwmon/hwmon*/rpm"):
        try: return int(open(p).read())
        except (OSError, ValueError): pass
    return -1
def cpu_mhz():
    try: return int(open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq").read()) / 1000.0
    except (OSError, ValueError): return float("nan")
def imx_temp():
    try:
        with open(a.log, "rb") as f:
            f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 65536)); txt = f.read().decode(errors="ignore")
    except OSError: return float("nan"), float("nan"), -1
    for line in reversed(txt.splitlines()):
        m = IMX.search(ANSI.sub("", line))
        if m: return float(m.group(1)), float(m.group(2)), int(m.group(3))
    return float("nan"), float("nan"), -1
os.makedirs(os.path.dirname(a.csv), exist_ok=True); new = not os.path.exists(a.csv); f = open(a.csv, "a"); cw = csv.writer(f)
if new: cw.writerow(["time", "imx_c", "tj_c", "cpu_c", "gpu_c", "soc0_c", "fan_rpm", "fan_pwm", "cpu0_mhz"])
print(f"{time.strftime('%H:%M:%S')} temp_watch started: csv {a.csv}, zones {sorted(zones)}", flush=True)
hist = collections.deque(maxlen=int(300 / a.interval) + 1); last_print = 0.0
while True:
    it, tj_log, pwm = imx_temp(); tj = zt("tj-thermal"); rpm = fan_rpm(); mhz = cpu_mhz()
    cw.writerow([time.strftime("%H:%M:%S"), f"{it:.1f}", f"{tj:.1f}", f"{zt('cpu-thermal'):.1f}", f"{zt('gpu-thermal'):.1f}", f"{zt('soc0-thermal'):.1f}", rpm, pwm, f"{mhz:.0f}"]); f.flush()
    hist.append(it); now = time.time()
    d5 = it - hist[0] if len(hist) > 1 and it == it and hist[0] == hist[0] else float("nan")
    if tj >= a.tj_alert or (d5 == d5 and abs(d5) > 1.5):
        print(f"ALERT {time.strftime('%H:%M:%S')} IMX {it:.1f} C ({d5:+.1f} in 5 min) Tj {tj:.1f} C fan {rpm} rpm", flush=True)
    elif now - last_print >= a.print_every:
        print(f"TEMP {time.strftime('%H:%M:%S')} IMX {it:.1f} C ({d5:+.1f} in 5 min) | Tj {tj:.1f} C cpu {zt('cpu-thermal'):.1f} gpu {zt('gpu-thermal'):.1f} soc0 {zt('soc0-thermal'):.1f} | fan {rpm} rpm pwm {pwm}/255 | cpu0 {mhz:.0f} MHz", flush=True)
        last_print = now
    time.sleep(a.interval)
