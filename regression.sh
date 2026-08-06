#!/usr/bin/env bash
# regression.sh [CONFIG] [REPS] -- does the stack still work?
#
#   ./regression.sh                    flight_D_best, 3 reps   (the reference)
#   ./regression.sh flight_A 3         a different config
#   BAG=~/rosbags/field_data/seg01_from_ros1 ./regression.sh
#
# Run this after ANY rebuild, config change, or environment change. Exits 0 on
# pass, 1 on fail.
#
# WHY IT IS SPLIT IN TWO
#
# This pipeline is not deterministic -- flight_D_best has produced scale 0.857
# to 0.984 AND occasional collapses (0.047, 0.076) from the identical bag and
# config. A single run therefore cannot pass or fail anything, and a strict
# check on the trajectory would flag noise as a regression.
#
# So the checks are split:
#
#   HARD INVARIANTS -- must hold on EVERY rep. These are deterministic and a
#   breakage here means the build, config or environment is genuinely broken:
#     * the replay completes (2985 poses, 3033 tracked frames on seg01)
#     * the estimator keeps up (median lag < 5 ms)
#     * a use_baro config actually receives barometer measurements
#       (catches the OV_WS trap: a baro config on the pre-baro build parses the
#        key, ignores it, and runs plain VIO looking entirely plausible)
#
#   STATISTICAL BOUND -- a MAJORITY of reps must be healthy. Tolerates the known
#   failure mode without tolerating a genuine regression.
#
# The reference numbers below are measured, not guessed -- flight_D_best over
# ~11 runs on 2026-08-05/06. Update them only with evidence.
set -u

CFG_NAME="${1:-flight_D_best}"
REPS="${2:-3}"
case "$CFG_NAME" in
  flight_D_best|best) CFG=ov_9281_flight_D_best ;;
  flight_A)           CFG=ov_9281_flight_A ;;
  gold)               CFG=ov_9281_aerial_staticinit ;;
  *)                  CFG="$CFG_NAME" ;;
esac

S=/home/vc/sensor_ws/src/sensor_bringup/scripts
C=/home/vc/sensor_ws/src/sensor_bringup/config/$CFG/estimator_config.yaml
BAG="${BAG:-/home/vc/rosbags/field_data/seg01_ros2_baro}"
[ -f "$C" ] || { echo "no config: $C" >&2; exit 1; }
grep -qE "^use_baro:[[:space:]]*true" "$C" && export OV_WS="${OV_WS:-$HOME/openvins_baro_ws}"

set +u; source /opt/ros/humble/setup.bash; set -u
exec 9>/tmp/regression.lock
flock -n 9 || { echo "another regression run is in progress" >&2; exit 1; }

TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
echo "config : $CFG"
echo "bag    : $BAG"
echo "OV_WS  : ${OV_WS:-$HOME/openvins_ws}"
echo "reps   : $REPS"
echo

for r in $(seq 1 "$REPS"); do
  echo "--- rep $r/$REPS ---"
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play"); do kill -9 "$p" 2>/dev/null; done
  sleep 5
  M=$(mktemp); touch "$M"
  "$S/run_openvins_bag.sh" "$BAG" --config "$C" --no-rviz > "$TMP/out_$r" 2>&1 &
  PID=$!
  for _ in $(seq 1 120); do
    grep -q "playback finished" "$TMP/out_$r" 2>/dev/null && break
    kill -0 $PID 2>/dev/null || break
    sleep 4
  done
  # Drain until the estimator stops emitting poses. A fixed sleep truncates the
  # run when the backlog is deep -- that produced 2845/2921 poses instead of 2985
  # on 2026-08-06 and was scored as a stack failure when it was this script's
  # fault. Poll instead, with a ceiling.
  LL="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$M" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  prev=-1
  for _ in $(seq 1 30); do
    sleep 4
    cur=$(grep -c p_IinG "$LL" 2>/dev/null || echo 0)
    [ "$cur" = "$prev" ] && break
    prev=$cur
  done
  L="$(find /tmp/vio_logs -name 'openvins_bag_*.log' -newer "$M" -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  rm -f "$M"; kill -INT $PID 2>/dev/null; sleep 3; kill -KILL $PID 2>/dev/null
  for p in $(pgrep -x run_subscribe_m) $(pgrep -f "bag[ ]play"); do kill -9 "$p" 2>/dev/null; done
  if [ -z "$L" ]; then echo "  NO LOG PRODUCED"; else echo "$L" >> "$TMP/logs"; echo "  $L"; fi
done

echo
python3 - "$TMP/logs" "$C" <<'EOF'
import re, sys, numpy as np

# measured on flight_D_best, seg01_ros2_baro, ~11 runs 2026-08-05/06
FULL_POSES, FULL_FRAMES = 2985, 3033
MAX_LAG_MS   = 5.0          # healthy runs sit at 0.7; 99 was the worst ever seen
SCALE_LO, SCALE_HI = 0.80, 1.05   # healthy band 0.857-0.984
MSCKF_MIN    = 55.0         # healthy 60-66; collapses land at 6-44
MAJORITY     = 0.5          # fraction of reps that must be healthy
TOL          = 30           # poses/frames may fall this far short without failing          # fraction of reps that must be healthy

try:
    logs = [l.strip() for l in open(sys.argv[1]) if l.strip()]
except OSError:
    print("FAIL: no runs completed"); sys.exit(1)
wants_baro = bool(re.search(r"^use_baro:\s*true", open(sys.argv[2]).read(), re.M))

import warnings; warnings.filterwarnings("ignore")
import pandas as pd
df = pd.read_excel("/home/vc/rosbags/seg01_fc.xlsx", header=4)
df = df[pd.to_numeric(df["t_rel"], errors="coerce").notna()]
for c in ("t_rel", "gps_e_m", "gps_n_m"):
    df[c] = pd.to_numeric(df[c], errors="coerce")
d = df.dropna(subset=["t_rel", "gps_e_m", "gps_n_m"])
tg, ge, gn = d["t_rel"].values, d["gps_e_m"].values, d["gps_n_m"].values

def scale_vs_gps(P):
    # An incomplete run covers only the START of the flight; stretching it over
    # the whole GPS span returns a meaningless number (2845 poses -> "0.007").
    frac = min(1.0, len(P) / FULL_POSES)
    tv = np.linspace(tg[0], tg[0] + frac * (tg[-1] - tg[0]), len(P))
    gi = np.c_[np.interp(tv, tg, ge), np.interp(tv, tg, gn)]
    src, dst = P[:, :2], gi
    S, D = src - src.mean(0), dst - dst.mean(0)
    U, sig, Vt = np.linalg.svd(D.T @ S / len(src))
    if np.linalg.det(U @ Vt) < 0: sig[-1] *= -1
    return sig.sum() / (S ** 2).sum() * len(src)

hard_fail, healthy, rows = [], 0, []
for i, p in enumerate(logs, 1):
    t = open(p, errors="ignore").read()
    P = np.array([[float(x) for x in m.groups()] for m in
                  re.finditer(r"p_IinG = ([-\d.]+),([-\d.]+),([-\d.]+)", t)])
    ms = np.array([int(x) for x in re.findall(r"MSCKF update \((\d+) feats\)", t)])
    frames = len(re.findall(r"TrackKLT.cpp:196", t))
    lag = [float(x) for _, x in re.findall(r"\(([\d.]+) hz, ([-\d.]+) ms behind\)", t)]
    baro = len(re.findall(r"\[BARO\]: t=", t))
    if not len(P) or not len(ms):
        hard_fail.append(f"rep{i}: no poses"); continue
    med_lag = float(np.median(lag)) if lag else float("nan")
    pct = 100 * np.mean(ms > 0)
    s = scale_vs_gps(P)

    # Tolerance, not equality. A healthy F_barounlock rep tracked 3028 of 3033
    # frames on 2026-08-06 and was failed for it -- five frames in 3033 is noise,
    # not breakage. The check exists to catch TRUNCATION (2845 poses) and a dead
    # tracker, both of which are orders of magnitude larger than this.
    if len(P)  < FULL_POSES  - TOL: hard_fail.append(f"rep{i}: {len(P)} poses, expected ~{FULL_POSES}")
    if frames  < FULL_FRAMES - TOL: hard_fail.append(f"rep{i}: {frames} frames, expected ~{FULL_FRAMES}")
    if med_lag > MAX_LAG_MS:   hard_fail.append(f"rep{i}: median lag {med_lag:.1f} ms > {MAX_LAG_MS}")
    if wants_baro and baro == 0:
        hard_fail.append(f"rep{i}: config sets use_baro but NO barometer measurements arrived "
                         f"(wrong OV_WS, or the bag has no baro topic)")

    ok = (SCALE_LO <= s <= SCALE_HI) and pct >= MSCKF_MIN
    healthy += ok
    rows.append((i, len(P), frames, med_lag, pct, s, baro, "ok" if ok else "unhealthy"))

print(f"  {'rep':>3s} {'poses':>6s} {'frames':>7s} {'lag ms':>7s} {'MSCKF':>7s} {'scale':>7s} {'baro':>6s}  verdict")
for i, n, f, l, pct, s, b, v in rows:
    print(f"  {i:3d} {n:6d} {f:7d} {l:7.1f} {pct:6.1f}% {s:7.3f} {b:6d}  {v}")

print()
need = int(len(rows) * MAJORITY) + 1
if hard_fail:
    print("  HARD INVARIANT FAILURES -- the stack is broken, not just noisy:")
    for h in hard_fail: print(f"    {h}")
if not rows:
    print("  FAIL: no scoreable runs"); sys.exit(1)
print(f"  healthy {healthy}/{len(rows)} (need {need}: scale {SCALE_LO}-{SCALE_HI}, MSCKF >= {MSCKF_MIN}%)")
ok = (not hard_fail) and healthy >= need
print(f"\n  ===> {'PASS' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
EOF
