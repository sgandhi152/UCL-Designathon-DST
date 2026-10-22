"""
Mystery Controller - student logger and analyser
================================================

Runs the Mystery Controller experiments over the kit's USB cable, saves
every run (CSV + plot), measures the response, and summarises repeated
trials for your results sheet. You never need to open the Arduino code.

Setup (once):
    pip install pyserial numpy matplotlib

Close the Arduino Serial Monitor and Serial Plotter first: only one program
can use the kit's USB port at a time.

Start the menu (recommended):
    python ball_logger.py --port COM5
    (Mac: --port /dev/cu.usbmodem14101   Linux: --port /dev/ttyACM0)

Other commands:
    python ball_logger.py run --port COM5 --letter A --test release
    python ball_logger.py analyse runs/A_release_1.csv
    python ball_logger.py summary runs
    python ball_logger.py calibrate --port COM5   (measure the ball's acceleration)

Tests:
    release   ball against the far end stop, level beam, START   (Exp 1)
    period    ball on the 18 cm mark (3 cm past centre), START    (hand calc)
    tilt      shim under the base, ball on the 15 cm mark, START  (Exp 2)
    release8  ball on the 23 cm mark, START and let go together   (Exp 3a)
    hold      ball on the 23 cm mark, START, hold 5 s, let go     (Exp 3b)

Every run starts the selected controller from a clean state (the kit clears
its memory on every START and every change of letter).
"""

import argparse
import csv
import glob
import os
import re
import sys
import time

import numpy as np

BAND_CM = 2.0            # target band for settling
FINAL_WINDOW_S = 5.0     # window for steady-state measures
STILL_SD_CM = 0.6        # "not moving" if position SD in the window is below this
STILL_RANGE_CM = 2.0     # ... and its range is below this
DRIFT_CM = 1.0           # "drifting" if the trend over the window exceeds this
SETTLE_HOLD_S = 3.0      # must stay in the band for at least this long at the end
NOMINAL_DT_MS = 65.0

TESTS = {
    "release": dict(
        seconds=20, hold_s=None, menu="Release from end stop (Experiment 1)",
        setup=["Beam level (kit is stopped).",
               "Rest the ball against the END STOP FURTHEST from the sensor."],
        go="Hands off. The controller starts at GO."),
    "period": dict(
        seconds=20, hold_s=None, menu="Period check, 3 cm start (hand calculation)",
        setup=["Use the controller you think is P.",
               "Hold the ball lightly on the 18 cm mark (3 cm past centre)."],
        go="Let go exactly at GO."),
    "tilt": dict(
        seconds=45, hold_s=None, menu="Shim test (Experiment 2)",
        setup=["Put the shim under the base on the side marked on your sheet.",
               "Hold the ball lightly on the 15 cm (centre) mark."],
        go="Let go exactly at GO, then leave it for the full 45 s."),
    "release8": dict(
        seconds=20, hold_s=None, menu="Baseline release from 23 cm mark (Experiment 3a)",
        setup=["Remove the shim. Beam level (kit is stopped).",
               "Hold the ball lightly with one fingertip on the 23 cm mark."],
        go="Let go exactly at GO."),
    "hold": dict(
        seconds=25, hold_s=5.0, menu="Hold 5 s on 23 cm mark, then release (Experiment 3b)",
        setup=["Remove the shim. Beam level (kit is stopped).",
               "Hold the ball with one fingertip on the 23 cm mark.",
               "Touch the BALL only, never the beam or linkage."],
        go="At GO the controller starts. KEEP HOLDING the ball on the mark."),
}


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------
def analyse(t, x, target, t0):
    """All metrics for one run. t0 = start of the response (release time)."""
    keep = t >= t0 - 1e-9
    t = t[keep] - t0
    d = x[keep] - target           # displacement from centre, + = away from sensor
    r = {"duration_s": float(t[-1]) if len(t) else 0.0}
    if len(t) < 20:
        r["error"] = "too little data"
        return r

    d0 = float(d[0])
    r["start_displacement_cm"] = d0

    # Overshoot: furthest excursion on the far side after first crossing centre
    if abs(d0) > 1.0:
        side = np.sign(d0)
        opp = np.where(np.sign(d) == -side)[0]
        if len(opp):
            os_cm = float(np.max(-side * d[opp[0]:]))
        else:
            os_cm = 0.0
        r["overshoot_cm"] = os_cm
        r["overshoot_pct"] = 100.0 * os_cm / abs(d0)

    # Settling into the target band: must stay in it for the rest of the run,
    # and for at least SETTLE_HOLD_S at the end
    outside = np.where(np.abs(d) > BAND_CM)[0]
    if len(outside) == 0:
        r["settling_s"] = 0.0
    else:
        enter = outside[-1] + 1
        if enter < len(t) and t[-1] - t[enter] >= SETTLE_HOLD_S:
            r["settling_s"] = float(t[enter])
        else:
            r["settling_s"] = None     # never settled at the target

    # Final window: signed mean displacement, spread, drift
    w = t >= t[-1] - FINAL_WINDOW_S
    dw, tw = d[w], t[w]
    r["final_mean_cm"] = float(np.mean(dw))
    r["final_sd_cm"] = float(np.std(dw))
    r["final_range_cm"] = float(np.ptp(dw))
    slope = np.polyfit(tw, dw, 1)[0] if len(tw) > 3 else 0.0
    r["final_drift_cm"] = float(slope * FINAL_WINDOW_S)
    still = (r["final_sd_cm"] < STILL_SD_CM and r["final_range_cm"] < STILL_RANGE_CM
             and abs(r["final_drift_cm"]) < DRIFT_CM)
    if still and np.max(np.abs(dw)) <= BAND_CM:
        r["final_state"] = "settled at target"
    elif still:
        r["final_state"] = "stationary off-centre"
    else:
        r["final_state"] = "no steady state (moving or drifting)"

    # Oscillation period: upward crossings of centre with +-1 cm hysteresis,
    # crossing time found by linear interpolation
    ups, below = [], False
    for i in range(1, len(d)):
        if d[i] < -1.0:
            below = True
        elif below and d[i] > 1.0:
            j = i
            while j > 0 and d[j - 1] > 0:
                j -= 1
            if j > 0:
                tc = t[j - 1] + (t[j] - t[j - 1]) * (-d[j - 1]) / (d[j] - d[j - 1])
            else:
                tc = t[j]
            ups.append((tc, j))
            below = False
    if len(ups) >= 2:
        times = np.array([u[0] for u in ups])
        per = np.diff(times)
        r["periods_s"] = [round(float(p), 2) for p in per]
        r["first_period_s"] = float(per[0])
        idx = [u[1] for u in ups]
        r["cycle_pp_cm"] = [round(float(np.ptp(d[a:b])), 1) for a, b in zip(idx[:-1], idx[1:])]
    return r


def timing_stats(dt_ms):
    dt_ms = np.asarray(dt_ms, dtype=float)
    if len(dt_ms) == 0:
        return {}
    return {"dt_mean_ms": float(np.mean(dt_ms)), "dt_max_ms": float(np.max(dt_ms)),
            "dt_late_pct": float(100 * np.mean(dt_ms > 1.5 * NOMINAL_DT_MS))}


def report(r, test):
    def f(v, unit, nd=2):
        return "-" if v is None else f"{v:.{nd}f} {unit}"
    print("\n--- Results -------------------------------------------------")
    if "error" in r:
        print("Not enough data:", r["error"])
        return
    print(f"Start displacement     : {f(r['start_displacement_cm'], 'cm')}")
    if "overshoot_cm" in r:
        print(f"Overshoot              : {f(r['overshoot_cm'], 'cm')}"
              f"  ({r['overshoot_pct']:.0f}% of start displacement)")
    s = r["settling_s"]
    print(f"Settling time (+-{BAND_CM:g} cm): {'never settled at target' if s is None else f(s, 's')}")
    print(f"Final {FINAL_WINDOW_S:g} s             : mean {r['final_mean_cm']:+.2f} cm, "
          f"SD {r['final_sd_cm']:.2f} cm, drift {r['final_drift_cm']:+.2f} cm")
    print(f"Final state            : {r['final_state']}")
    if "periods_s" in r:
        print(f"Oscillation periods    : {r['periods_s']} s")
        print(f"Peak-to-peak per cycle : {r['cycle_pp_cm']} cm")
        if test == "period":
            print("Use the FIRST period for your hand-calculation comparison.")
    if "dt_mean_ms" in r:
        print(f"Loop timing            : mean {r['dt_mean_ms']:.1f} ms, max {r['dt_max_ms']:.1f} ms,"
              f" {r['dt_late_pct']:.1f}% of ticks late")
    print("Check every number against the plot before you use it.\n")


def plot(t, x, tilt, target, title, png, t0, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6),
                                 gridspec_kw={"height_ratios": [3, 1.3]})
    a1.axhspan(target - BAND_CM, target + BAND_CM, color="0.9",
               label=f"target band +-{BAND_CM:g} cm")
    a1.axhline(target, color="0.4", lw=1, ls="--", label="centre (target)")
    a1.plot(t, x, lw=1.6, label="ball position")
    for a in (a1, a2):
        a.axvline(t0, color="tab:red", lw=1, ls=":")
    a1.text(t0, a1.get_ylim()[1], " release", color="tab:red", va="top", fontsize=9)
    a1.set_ylabel("ball position from sensor (cm)")
    a1.set_title(title)
    a1.legend(loc="upper right", fontsize=8)
    a1.grid(alpha=0.3)
    a2.plot(t, tilt, lw=1, color="tab:orange")
    a2.axhline(0, color="0.4", lw=0.8)
    a2.set_ylabel("servo tilt (deg)")
    a2.set_xlabel("board time since START (s)")
    a2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png, dpi=150)
    print(f"Plot saved: {png}")
    if show:
        print("(Close the plot window to continue.)")
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------
HEADER = ["t_s", "dt_ms", "letter", "running", "event",
          "ball_cm", "target_cm", "tilt_deg"]


def save_csv(path, rows):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(HEADER)
        w.writerows(rows)
    print(f"Data saved: {path}")


def load_csv(path):
    t, dt, x, tilt, ev = [], [], [], [], []
    target = 15.0
    with open(path) as fh:
        for row in csv.reader(fh):
            if not row or row[0] == "t_s" or row[0].startswith("#") or len(row) < 8:
                continue
            if row[3] != "1":
                continue
            t.append(float(row[0])); dt.append(float(row[1]))
            ev.append(row[4] == "1"); x.append(float(row[5]))
            target = float(row[6]); tilt.append(float(row[7]))
    t, x, tilt = np.array(t), np.array(x), np.array(tilt)
    ev = np.array(ev, dtype=bool)
    t0 = float(t[ev][0]) if ev.any() else (float(t[0]) if len(t) else 0.0)
    return t, np.array(dt), x, tilt, target, t0


def parse_name(path):
    m = re.match(r"([ABCK])_([a-z0-9]+)_(\d+)$", os.path.splitext(os.path.basename(path))[0])
    return (m.group(1), m.group(2), int(m.group(3))) if m else (None, None, None)


def next_name(folder, letter, test):
    os.makedirs(folder, exist_ok=True)
    n = 1
    while os.path.exists(os.path.join(folder, f"{letter}_{test}_{n}.csv")):
        n += 1
    return os.path.join(folder, f"{letter}_{test}_{n}"), n


def analyse_path(path, show):
    t, dt, x, tilt, target, t0 = load_csv(path)
    letter, test, n = parse_name(path)
    r = analyse(t, x, target, t0)
    r.update(timing_stats(dt))
    report(r, test or "")
    title = (f"Controller {letter} - {test} test (trial {n})" if letter
             else os.path.basename(path))
    plot(t, x, tilt, target, title, os.path.splitext(path)[0] + ".png", t0, show)
    return r


# ---------------------------------------------------------------------------
# Talking to the kit
# ---------------------------------------------------------------------------
class Kit:
    def __init__(self, port):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(2.0)            # in case the board resets when the port opens
        self.send("x")
        self.send("l")
        self.drain()

    def send(self, c):
        self.ser.write(c.encode())
        self.ser.flush()
        time.sleep(0.05)

    def drain(self, seconds=0.2, show_messages=True):
        """Read and discard for a short time. The board streams a row every
        tick, so never wait for the line to go quiet."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line.startswith("#") and show_messages:
                print("kit:", line[1:].strip())

    def close(self):
        try:
            self.send("x")
        finally:
            self.ser.close()


def countdown(kit, n=3):
    for k in range(n, 0, -1):
        print(f"  {k}...", flush=True)
        kit.drain(1.0)


def run_test(kit, letter, test, folder, show):
    spec = TESTS[test]
    kit.send("x")
    kit.send(letter.lower())
    kit.drain()
    print(f"\n=== Controller {letter}: {spec['menu']} ===")
    for s in spec["setup"]:
        print(" -", s)
    print(" -", spec["go"])
    input("Press Enter when ready (Ctrl+C during the run stops it early)... ")
    countdown(kit)

    rows, released = [], spec["hold_s"] is None
    kit.send("g")
    if released:
        kit.send("m")
    print("  GO" + ("" if released else f" - keep holding for {spec['hold_s']:g} s"))
    t_start = time.time()
    try:
        while time.time() - t_start < spec["seconds"]:
            if not released and time.time() - t_start >= spec["hold_s"]:
                kit.send("m")
                released = True
                print("  RELEASE NOW")
            line = kit.ser.readline().decode(errors="ignore").strip()
            if line.startswith("#"):
                print("kit:", line[1:].strip())
                if "Stopped" in line:
                    break
                continue
            parts = line.split(",")
            if len(parts) != 8 or parts[0] == "t_s":
                continue
            try:
                row = [float(parts[0]), float(parts[1]), parts[2], int(parts[3]),
                       int(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]
            except ValueError:
                continue
            if row[3] == 1:
                rows.append(row)
    except KeyboardInterrupt:
        print("\n  stopped early")
    kit.send("x")
    kit.drain()

    if len(rows) < 20:
        print("Hardly any data received - is the sensor seeing the ball? Run not saved.")
        return None
    base, n = next_name(folder, letter, test)
    save_csv(base + ".csv", rows)
    print(f"This was trial {n} of controller {letter}, {test} test.")
    return analyse_path(base + ".csv", show)


G_CM = 981.0
G_LINK = 0.198          # beam degrees per servo degree
CAL_TILT = 10.0         # servo degrees, must match CAL_TILT_DEG in the firmware


def calibrate(kit, folder, show):
    """Fixed-tilt run: fit x = x0 + v0 t + a t^2 / 2 and report K and c."""
    kit.send("x")
    kit.drain()
    print("\n=== Calibration: measure the ball's acceleration ===")
    print(" - Kit stopped, beam level, no shim.")
    print(" - Rest the ball against the NEAR end stop (the sensor end).")
    print(f" - At GO the beam tilts to a fixed {CAL_TILT:g} servo degrees (controller OFF)")
    print("   and the ball rolls away. Keep hands clear; it stops by itself.")
    input("Press Enter when ready... ")
    countdown(kit)
    kit.send("k")
    kit.send("m")
    print("  GO")
    rows, t_start = [], time.time()
    while time.time() - t_start < 6.0:
        line = kit.ser.readline().decode(errors="ignore").strip()
        if line.startswith("#"):
            if "Stopped" in line:
                break
            continue
        parts = line.split(",")
        if len(parts) != 8 or parts[0] == "t_s":
            continue
        try:
            if parts[3] == "1":
                rows.append([float(parts[0]), float(parts[1]), parts[2], 1, int(parts[4]),
                             float(parts[5]), float(parts[6]), float(parts[7])])
        except ValueError:
            continue
    kit.send("x")
    kit.drain()
    if len(rows) < 8:
        print("Too little data - did the ball roll? Not saved.")
        return None
    base, n = next_name(folder, "K", "cal")
    save_csv(base + ".csv", rows)
    return analyse_calibration(base + ".csv", show)


def analyse_calibration(path, show):
    t, dt, x, tilt, target, t0 = load_csv(path)
    use = (x > 5.0) & (x < 24.0) & (t > 0.15)
    if use.sum() < 6:
        print("Not enough points between 5 and 24 cm to fit.")
        return None
    c2, c1, c0 = np.polyfit(t[use], x[use], 2)
    a = 2 * c2
    K = a / CAL_TILT
    c = K / (G_CM * G_LINK * np.pi / 180)
    print("\n--- Calibration result ------------------------------------")
    print(f"Acceleration a         : {a:.1f} cm/s^2 at {CAL_TILT:g} servo deg")
    print(f"K = a / tilt           : {K:.2f} cm/s^2 per servo degree")
    print(f"Rolling factor c       : {c:.2f}   (5/7 = 0.71 for a solid ball on a flat track)")
    print("Repeat 3 times and average. Predicted P period: T = 2*pi / sqrt(K * Kp).\n")
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, x, ".", ms=4, label="measured")
    tf = np.linspace(t[use].min(), t[use].max(), 100)
    ax.plot(tf, c0 + c1 * tf + c2 * tf ** 2, "-", label=f"fit: a = {a:.1f} cm/s$^2$")
    ax.set_xlabel("time (s)"); ax.set_ylabel("ball position from sensor (cm)")
    ax.set_title(f"Calibration at {CAL_TILT:g} servo deg: K = {K:.2f}, c = {c:.2f}")
    ax.grid(alpha=0.3); ax.legend()
    fig.tight_layout()
    png = os.path.splitext(path)[0] + ".png"
    fig.savefig(png, dpi=150)
    print(f"Plot saved: {png}")
    if show:
        plt.show()
    plt.close(fig)
    return {"a": a, "K": K, "c": c}


def menu(args):
    kit = Kit(args.port)
    letter = "A"
    keys = {"1": "release", "2": "period", "3": "tilt", "4": "release8", "5": "hold"}
    try:
        while True:
            print("\n================ Mystery Controller ================")
            print(f" Selected controller: {letter}        (runs saved in '{args.folder}')")
            print("  a / b / c   choose controller")
            for k, test in keys.items():
                print(f"  {k}           {TESTS[test]['menu']}")
            print("  k           measure the ball's acceleration (K calibration)")
            print("  s           summary of all saved runs")
            print("  q           quit")
            choice = input("> ").strip().lower()
            if choice in ("a", "b", "c"):
                letter = choice.upper()
                kit.send("x"); kit.send(choice)
                print(f"Controller {letter} selected (kit stopped and reset).")
            elif choice in keys:
                run_test(kit, letter, keys[choice], args.folder, not args.no_show)
            elif choice == "k":
                calibrate(kit, args.folder, not args.no_show)
            elif choice == "s":
                summary(args.folder)
            elif choice == "q":
                break
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        kit.close()


# ---------------------------------------------------------------------------
# Summary of repeated trials
# ---------------------------------------------------------------------------
SUMMARY_FIELDS = ["overshoot_cm", "overshoot_pct", "settling_s", "final_mean_cm",
                  "final_sd_cm", "first_period_s"]


def summary(folder):
    files = sorted(glob.glob(os.path.join(folder, "*.csv")))
    groups = {}
    for p in files:
        letter, test, n = parse_name(p)
        if not letter or letter == "K":
            continue
        t, dt, x, tilt, target, t0 = load_csv(p)
        if len(t) < 20:
            continue
        r = analyse(t, x, target, t0)
        groups.setdefault((letter, test), []).append((n, r))
    if not groups:
        print("No runs found in", folder)
        return

    out = os.path.join(folder, "summary.csv")
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["letter", "test", "trials", "metric", "mean", "min", "max", "not_settled"])
        print("\nletter test      n  overshoot(cm)  overshoot(%)  settling(s)      final mean(cm)  final state")
        for (letter, test), runs in sorted(groups.items()):
            rs = [r for _, r in runs]
            def vals(k):
                return [r[k] for r in rs if r.get(k) is not None]
            for k in SUMMARY_FIELDS:
                v = vals(k)
                if v:
                    w.writerow([letter, test, len(rs), k, f"{np.mean(v):.2f}",
                                f"{np.min(v):.2f}", f"{np.max(v):.2f}",
                                sum(r.get("settling_s") is None for r in rs) if k == "settling_s" else ""])
            def ms(k, nd=2):
                v = vals(k)
                return "-" if not v else (f"{np.mean(v):.{nd}f} ({np.min(v):.{nd}f}-{np.max(v):.{nd}f})"
                                          if len(v) > 1 else f"{v[0]:.{nd}f}")
            ns = sum(r.get("settling_s") is None for r in rs)
            sett = ms("settling_s") + (f" [{ns} never]" if ns else "")
            states = sorted(set(r.get("final_state", "?") for r in rs))
            print(f"  {letter}    {test:<9} {len(rs)}  {ms('overshoot_cm'):<14} {ms('overshoot_pct', 0):<13} "
                  f"{sett:<16} {ms('final_mean_cm'):<15} {'; '.join(states)}")

    # Matched wind-up comparison: hold vs baseline release from the same mark
    print("\nWind-up check (same 23 cm start): hold overshoot minus baseline overshoot")
    for letter in "ABC":
        a = [r.get("overshoot_cm") for _, r in groups.get((letter, "hold"), [])]
        b = [r.get("overshoot_cm") for _, r in groups.get((letter, "release8"), [])]
        a = [v for v in a if v is not None]; b = [v for v in b if v is not None]
        if a and b:
            print(f"  {letter}: {np.mean(a):.2f} - {np.mean(b):.2f} = {np.mean(a) - np.mean(b):+.2f} cm"
                  f"  ({len(a)} hold, {len(b)} baseline trials)")
    print(f"\nSummary saved: {out}")


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Mystery Controller logger")
    p.add_argument("--port", help="kit USB port, e.g. COM5 or /dev/cu.usbmodem14101")
    p.add_argument("--folder", default="runs")
    p.add_argument("--no-show", action="store_true", help="save plots without opening them")
    sub = p.add_subparsers(dest="cmd")

    r = sub.add_parser("run", help="run one test without the menu")
    r.add_argument("--port", dest="port_run", required=True)
    r.add_argument("--letter", required=True, choices=list("ABCabc"))
    r.add_argument("--test", required=True, choices=list(TESTS))

    a = sub.add_parser("analyse", help="re-analyse a saved CSV")
    a.add_argument("csv")

    k = sub.add_parser("calibrate", help="measure the ball's acceleration (K)")
    k.add_argument("--port", dest="port_cal", required=True)

    s = sub.add_parser("summary", help="summarise all runs in a folder")
    s.add_argument("folder_s", nargs="?", default=None)

    args = p.parse_args()
    if args.cmd == "run":
        kit = Kit(args.port_run)
        try:
            run_test(kit, args.letter.upper(), args.test, args.folder, not args.no_show)
        finally:
            kit.close()
    elif args.cmd == "calibrate":
        kit = Kit(args.port_cal)
        try:
            calibrate(kit, args.folder, not args.no_show)
        finally:
            kit.close()
    elif args.cmd == "analyse":
        if parse_name(args.csv)[0] == "K":
            analyse_calibration(args.csv, not args.no_show)
        else:
            analyse_path(args.csv, not args.no_show)
    elif args.cmd == "summary":
        summary(args.folder_s or args.folder)
    else:
        if not args.port:
            sys.exit("Give the kit's port, e.g.  python ball_logger.py --port COM5")
        menu(args)


if __name__ == "__main__":
    main()
