"""
Mystery Controller - rig simulation (instructor tool)
=====================================================

Predicts what each hidden controller should do in each experiment, so the
expected signatures can be checked BEFORE bench testing. Everything this
script prints or plots is a SIMULATION, not a measurement.

It copies the firmware's logic: 65 ms tick with measured dt, median-of-3 +
low-pass position filter, filtered velocity, P / PD / PID laws with the same
gains, integral cap, output clamp and servo limits. The analysis uses the
same functions as the student logger (ball_logger.py).

Usage:
    python sim_rig.py                 # table of predictions + sim_traces.png
    python sim_rig.py --seeds 10      # more noise realisations

Assumptions (edit the constants below to explore):
    x'' = K*(servo tilt + shim) - FRICTION*v, K = 1.69 cm/s^2 per servo degree
        (measured on aluminium rails with the ping-pong ball)
    servo: first-order lag SERVO_TAU, max speed SERVO_RATE
    sensor noise: Gaussian, SD NOISE_CM per reading, one reading per tick
    tick: 65 ms +- TICK_JITTER_MS (uniform)
    end stops at 2.8 and 27.6 cm, ball keeps REBOUND of its speed
"""

import argparse

import numpy as np

from ball_logger import analyse

# Plant and hardware assumptions
K = 1.69
FRICTION = 0.05          # 1/s
SERVO_TAU = 0.05         # s
SERVO_RATE = 600.0       # deg/s
NOISE_CM = 0.3
TICK_MS = 65.0
TICK_JITTER_MS = 2.0
STOP_NEAR, STOP_FAR = 2.8, 27.6
REBOUND = 0.2

# Firmware constants (keep in step with mystery_controller.ino)
SP = 15.0
KP, KD, KI = 1.3, 1.4, 0.3
I_LIMIT = 20.0
MAX_TILT = 30.0
LEVEL, SERVO_MIN, SERVO_MAX = 104, 74, 134
POS_ALPHA, VEL_ALPHA = 0.6, 0.5
NOMINAL_DT = TICK_MS / 1000.0

LAWS = ("P", "PD", "PID")
SHIM_SERVO_DEG = 5.0     # 1 deg of beam / G (0.198)


def simulate(law, x0, seconds, shim=0.0, hold_s=0.0, seed=0, settle_before=1.0):
    """Return arrays t (board time since START), x (filtered), tilt, and t0."""
    rng = np.random.default_rng(seed)
    x, v, servo = x0, 0.0, 0.0
    hist = [x0] * 3
    pos = prev = x0
    vel = integ = 0.0
    t_board, out = 0.0, []
    running = False
    total = settle_before + seconds
    t = 0.0
    while t < total:
        dt = (TICK_MS + rng.uniform(-TICK_JITTER_MS, TICK_JITTER_MS)) / 1000.0
        dt_c = min(max(dt, 0.5 * NOMINAL_DT), 2.0 * NOMINAL_DT)
        if not running and t >= settle_before:
            running = True
            vel = integ = 0.0
            prev = pos
            t_board = 0.0
        # sensor + filters (firmware updatePosition)
        raw = x + rng.normal(0, NOISE_CM)
        hist = hist[1:] + [raw]
        pos = POS_ALPHA * sorted(hist)[1] + (1 - POS_ALPHA) * pos
        vraw = (pos - prev) / dt_c
        prev = pos
        vel = VEL_ALPHA * vraw + (1 - VEL_ALPHA) * vel
        # controller (firmware computeTilt + writeServo)
        if running:
            e = SP - pos
            u = KP * e
            if law in ("PD", "PID"):
                u -= KD * vel
            if law == "PID":
                integ += e * dt_c
                integ = float(np.clip(integ, -I_LIMIT / KI, I_LIMIT / KI))
                u += KI * integ
            u = float(np.clip(u, -MAX_TILT, MAX_TILT))
            ang = int(np.clip(LEVEL + round(u), SERVO_MIN, SERVO_MAX))
            cmd = float(ang - LEVEL)
        else:
            cmd = 0.0
        # physics between ticks
        n = 10
        h = dt / n
        for _ in range(n):
            servo += float(np.clip((cmd - servo) / SERVO_TAU * h, -SERVO_RATE * h, SERVO_RATE * h))
            held = (not running) or (t_board < hold_s)
            if held:
                v = 0.0
                continue
            a = K * (servo + shim) - FRICTION * v
            v += a * h
            x += v * h
            if x < STOP_NEAR:
                x, v = STOP_NEAR, -REBOUND * v
            if x > STOP_FAR:
                x, v = STOP_FAR, -REBOUND * v
        if running:
            out.append((t_board, pos, cmd))
            t_board += dt
        t += dt
    o = np.array(out)
    return o[:, 0], o[:, 1], o[:, 2], hold_s


TESTS = {
    "release":  dict(x0=27.4, seconds=20, laws=LAWS),
    "period":   dict(x0=18.0, seconds=20, laws=("P",)),
    "tilt":     dict(x0=15.0, seconds=45, laws=("PD", "PID"), shim=SHIM_SERVO_DEG),
    "release8": dict(x0=23.0, seconds=20, laws=("PD", "PID")),
    "hold":     dict(x0=23.0, seconds=25, laws=("PD", "PID"), hold_s=5.0),
}


def fmt(vals, nd=1, unit=""):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "-"
    if max(vals) - min(vals) < 10 ** -nd:
        return f"{np.mean(vals):.{nd}f}{unit}"
    return f"{min(vals):.{nd}f}-{max(vals):.{nd}f}{unit}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--png", default="sim_traces.png")
    args = ap.parse_args()

    print("SIMULATED predictions (not measurements). Ranges are over", args.seeds, "noise seeds.\n")
    print("| Test | Law | Overshoot (cm) | Overshoot (%) | Settling (s) | Final mean (cm) | Final state | First period (s) |")
    print("|---|---|---|---|---|---|---|---|")
    traces = {}
    for test, spec in TESTS.items():
        for law in spec["laws"]:
            rs = []
            for s in range(args.seeds):
                t, x, tilt, t0 = simulate(law, spec["x0"], spec["seconds"], spec.get("shim", 0.0),
                                          spec.get("hold_s", 0.0), seed=s)
                rs.append(analyse(t, x, SP, t0))
                if s == 0:
                    traces[(test, law)] = (t, x, tilt, t0)
            nset = sum(r["settling_s"] is None for r in rs)
            sett = fmt([r["settling_s"] for r in rs]) + (f" ({nset}/{len(rs)} never)" if nset else "")
            states = "; ".join(sorted(set(r["final_state"] for r in rs)))
            print(f"| {test} | {law} | {fmt([r.get('overshoot_cm') for r in rs])} | "
                  f"{fmt([r.get('overshoot_pct') for r in rs], 0)} | {sett} | "
                  f"{fmt([r['final_mean_cm'] for r in rs])} | {states} | "
                  f"{fmt([r.get('first_period_s') for r in rs], 2)} |")
    w = 2 * np.pi / np.sqrt(K * KP)
    print(f"\nPredicted P period 2*pi/sqrt(K*Kp) = {w:.2f} s")

    # Servo jitter at rest (tick-to-tick tilt change, RMS)
    print("\nServo jitter with the ball at rest, RMS tilt change per tick (simulated;")
    print("P is not truly at rest - noise slowly grows into an oscillation):")
    for law in LAWS:
        t, x, tilt, _ = simulate(law, 15.0, 20, seed=1)
        d = np.diff(tilt[30:])
        print(f"  {law}: {np.sqrt(np.mean(d ** 2)):.2f} deg")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    col = {"P": "tab:blue", "PD": "tab:green", "PID": "tab:red"}
    panels = [
        (ax[0, 0], [("release", l) for l in LAWS], "Exp 1: release from far end stop"),
        (ax[0, 1], [("tilt", "PD"), ("tilt", "PID")], "Exp 2: 1 deg shim, ball starts at centre"),
        (ax[1, 0], [("release8", "PD"), ("hold", "PD"), ("release8", "PID"), ("hold", "PID")],
         "Exp 3: release from 23 cm mark, no hold vs 5 s hold"),
        (ax[1, 1], [("period", "P")], "Hand calc: P released 3 cm off-centre"),
    ]
    for a, keys, title in panels:
        a.axhspan(SP - 2, SP + 2, color="0.92")
        a.axhline(SP, color="0.5", lw=0.8, ls="--")
        for test, law in keys:
            t, x, tilt, t0 = traces[(test, law)]
            ls = "--" if test == "release8" else "-"
            lab = law + (" (hold)" if test == "hold" else " (no hold)" if test == "release8" else "")
            a.plot(t - t0, x, ls, color=col[law], lw=1.4, label=lab)
        a.set_title(title, fontsize=10)
        a.set_xlabel("time since release (s)")
        a.set_ylabel("ball position (cm)")
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.suptitle("SIMULATED responses (model assumptions in sim_rig.py) - not bench data", color="tab:red")
    fig.tight_layout()
    fig.savefig(args.png, dpi=130)
    print(f"\nTraces saved: {args.png}")


if __name__ == "__main__":
    main()
