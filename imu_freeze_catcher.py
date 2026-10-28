#!/usr/bin/env python3
"""imu_freeze_catcher.py [SECS=360] [--out /dev/shm/freeze_events.txt]

Catches an imx5_spi_node FREEZE in the act. Subscribes to /imu0 and watches arrivals; when nothing
arrives for more than 1 s it snapshots every thread of imx5_spi_node from /proc (state, wchan = the
kernel function it sleeps in, the syscall it is inside and, for read/write, the file behind the fd),
plus the disk's write latency, and repeats once per second until /imu0 returns.

Writes ONLY to RAM (/dev/shm) so the watcher cannot itself be blocked by the disk it is watching.
Run it with the ROS environment sourced, next to whatever load you suspect (a recording, or a
ROS-free disk writer). Read-only: it never touches the node.
"""
import glob, os, subprocess, sys, time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

secs = int(sys.argv[1]) if len(sys.argv) > 1 else 360
out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else "/dev/shm/freeze_events.txt"
evt = open(out, "w")


def log(s):
    evt.write(s + "\n"); evt.flush()


def pid_of(name):
    r = subprocess.run(["pgrep", "-x", name], capture_output=True, text=True).stdout.split()
    return int(r[0]) if r else None


def diskstat():
    for l in open("/proc/diskstats"):
        f = l.split()
        if f[2] == "nvme0n1":
            return int(f[7]), int(f[10]), int(f[11])       # writes completed, ms writing, in flight


VMKEYS = ("allocstall_normal", "allocstall_movable", "allocstall_dma32", "pgscan_direct", "pgscan_kswapd",
          "pgmajfault", "nr_dirty", "nr_writeback", "pswpout")


def vm():
    d = {}
    for l in open("/proc/vmstat"):
        k, v = l.split()
        if k in VMKEYS:
            d[k] = int(v)
    for l in open("/proc/meminfo"):
        if l.startswith("MemAvailable"):
            d["MemAvail_MB"] = int(l.split()[1]) // 1024
    return d


def run_delay(pid):
    """per-thread time spent runnable-but-not-running (CPU starvation), ns"""
    d = {}
    for tid in os.listdir(f"/proc/{pid}/task"):
        try:
            d[int(tid)] = int(open(f"/proc/{pid}/task/{tid}/schedstat").read().split()[1])
        except (OSError, IndexError, ValueError):
            pass
    return d


def delta_report(a, b, pid):
    (_, va, ra), (_, vb, rb) = a, b
    log("  system counters over the freeze (now minus just-before):")
    log("     " + "  ".join(f"{k}:+{vb[k]-va[k]}" for k in VMKEYS if k in vb and vb[k] != va[k]) + f"  MemAvailable {va.get('MemAvail_MB')} -> {vb.get('MemAvail_MB')} MB")
    rd = sorted(((rb.get(t, 0) - ra.get(t, 0)) / 1e6, t) for t in rb)[::-1][:3]
    log("  CPU run-queue wait per thread (ms runnable-but-not-running), top 3: " + ", ".join(f"tid {t}: {ms:.0f} ms" for ms, t in rd))


def snapshot(pid, tag):
    log(f"  [{tag}] threads of pid {pid}:")
    for tid in sorted(os.listdir(f"/proc/{pid}/task"), key=int):
        base = f"/proc/{pid}/task/{tid}"
        try:
            st = open(base + "/stat").read()
            comm = st[st.find("(") + 1:st.rfind(")")]
            state = st[st.rfind(")") + 2:].split()[0]
        except OSError:
            continue
        try:
            wchan = open(base + "/wchan").read().strip() or "-"
        except OSError:
            wchan = "?"
        sysc = "?"
        try:
            parts = open(base + "/syscall").read().split()
            if parts and parts[0] not in ("running", "-1"):
                nr = int(parts[0]); a0 = int(parts[1], 16)
                names = {0: "read", 1: "write", 16: "ioctl", 17: "pread", 18: "pwrite", 74: "fsync", 75: "fdatasync",
                         202: "futex", 230: "clock_nanosleep", 232: "epoll_wait", 7: "poll", 270: "pselect6",
                         64: "write(arm64)", 63: "read(arm64)", 29: "ioctl(arm64)", 98: "futex(arm64)", 101: "nanosleep(arm64)", 73: "ppoll(arm64)"}
                sysc = f"{names.get(nr, 'syscall ' + str(nr))}"
                if nr in (1, 0, 64, 63, 16, 29, 74, 82, 17, 18):
                    try:
                        sysc += f" fd={a0} -> {os.readlink(f'/proc/{pid}/fd/{a0}')}"
                    except OSError:
                        pass
            else:
                sysc = parts[0] if parts else "?"
        except OSError:
            pass
        if state != "S" or wchan not in ("-", "0", "do_epoll_wait", "futex_wait_queue_me", "do_nanosleep", "poll_schedule_timeout.constprop.0"):
            log(f"     tid {tid:>6} {comm:<16} state={state} wchan={wchan}  syscall={sysc}")
        else:
            log(f"     tid {tid:>6} {comm:<16} state={state} wchan={wchan}  syscall={sysc}  (idle)")


class Watch(Node):
    def __init__(self):
        super().__init__("freeze_catcher")
        self.last = time.monotonic(); self.n = 0
        # /imu0 is published best-effort; a default (reliable) subscriber would never match it
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(Imu, "/imu0", self.cb, qos_profile_sensor_data)

    def cb(self, _):
        self.last = time.monotonic(); self.n += 1


rclpy.init()
w = Watch()
pid = pid_of("imx5_spi_node")
if pid is None:
    sys.exit("imx5_spi_node is not running")
t_end = time.monotonic() + secs
freeze = None
events = []
hist = []                    # (monotonic, vm(), run_delay) once per second, last 30 s
next_h = 0
log(f"watching /imu0 for {secs} s, imx5_spi_node pid {pid}, started {time.strftime('%H:%M:%S')}")
d0 = diskstat()
while time.monotonic() < t_end:
    rclpy.spin_once(w, timeout_sec=0.05)
    now = time.monotonic()
    gap = now - w.last
    if now >= next_h and freeze is None:
        hist.append((now, vm(), run_delay(pid))); hist[:] = hist[-30:]; next_h = now + 1.0
    if freeze is None and w.n > 0 and gap > 1.0:      # only after the first message: discovery takes a second or two
        freeze = {"t": time.strftime("%H:%M:%S"), "start": w.last, "snaps": 0}
        d1 = diskstat()
        log(f"FREEZE detected at {freeze['t']} (no /imu0 for {gap:.1f} s); disk since start: {d1[0]-d0[0]} writes, avg latency {(d1[1]-d0[1])/max(1,d1[0]-d0[0]):.1f} ms, in flight {d1[2]}")
        snapshot(pid, f"{gap:.1f} s")
        freeze["last_snap"] = now
        freeze["before"] = next((h for h in reversed(hist) if h[0] <= w.last), hist[0] if hist else None)
    elif freeze is not None:
        if w.last > freeze["start"] and gap < 0.5:       # data is flowing again
            dur = w.last - freeze["start"]
            log(f"RECOVERED after {dur:.1f} s (disk in flight now {diskstat()[2]})")
            if freeze.get("before"):
                delta_report(freeze["before"], (time.monotonic(), vm(), run_delay(pid)), pid)
            events.append((freeze["t"], dur)); freeze = None
        elif now - freeze["last_snap"] >= 1.0 and freeze["snaps"] < 14:
            freeze["snaps"] += 1; freeze["last_snap"] = now
            snapshot(pid, f"{now - freeze['start']:.1f} s")
log(f"done {time.strftime('%H:%M:%S')}: {w.n} messages seen, {len(events)} freeze(s) > 1 s: {[(t, round(d, 1)) for t, d in events]}")
print(open(out).read())
