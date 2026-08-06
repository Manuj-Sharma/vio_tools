# baro_chi2_multipler sweep — 2026-08-06

Config `ov_9281_flight_D_best`, bag `seg01_ros2_baro`, one line changed per arm.
Nothing committed; the sweep configs `ov_9281_baro_chi2_{20,50,100}` are on disk
but untracked.

    mult   admits |res|   healthy   scale
       5        2.2 m      4/10     0.043 - 0.994    <- D_best, self-locks
      20        4.4 m       2/3     0.891 0.884 0.094
      50        6.9 m       3/3     0.953 0.938 0.959
     100        9.8 m       3/3     1.018 0.942 0.948
     500       21.9 m       9/9     0.936 - 0.992    <- F_barounlock

    admissible |res| = 0.5 * sqrt(3.841 * mult)
    (S collapses to sigma_baro^2 = 0.25 because P_zz goes to ~0)

## The gate creates the residuals it then rejects

With the gate OPEN, over 9 runs:

    |res| median  0.90 - 1.49 m
    |res| p95     3.65 - 4.57 m
    |res| MAX     5.31 - 7.08 m
    rejections    0

With mult 5 the residuals grow to 15-21 m -- but only BECAUSE rejection starts
during the climb and the error then accumulates unchecked. The large residuals
are a consequence of the gate, not a justification for it. This is the feedback
loop measured directly.

## Why this matters beyond altitude

Yesterday the self-lock was scored as an altitude problem (121-127 m vs 107.5).
It is a STABILITY problem: rejected corrections let depth error grow until the
filter tips over. On 2026-08-06, same machine and hour:

    D_best       (mult 5)     4 healthy / 10
    F_barounlock (mult 500)   9 healthy /  9

That also disproves the "machine degradation" idea proposed earlier the same
afternoon -- F ran clean in the window where D_best was failing 2 in 3.

## Reading

mult 50 is the smallest that held 3/3, but 6.9 m sits right at the observed
7.08 m maximum with no margin. mult 100 (9.8 m, ~40% headroom) is the better
candidate: still a genuine outlier test -- a blocked port or prop-wash transient
is tens of metres and would still be caught -- and 5x tighter than the 500 probe.

NOT yet established:
  * 3 reps per arm; mult 20's 2/3 could be luck either way. Want 5+ at 50 and 100.
  * Tuned to ONE bag, one climb profile. May not transfer to another flight.
  * The root cause is untouched: P_zz collapsing to ~0 is what makes a 2.2 m
    residual look like a 900-sigma outlier. Widening the gate treats the
    symptom. Fixing the covariance collapse would be the real repair.
