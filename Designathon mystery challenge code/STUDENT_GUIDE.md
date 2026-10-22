# Mystery Controller challenge: student guide

**Module:** MECH0023 Dynamics and Control

Your kit's board holds three hidden controllers, labelled **A**, **B** and **C**. One is a P controller, one is PD and one is PID. They all use the same proportional gain Kp. Your job is to work out which letter is which **from experiments and data**, not from the code. The code is revealed at the end.

Each kit has its own shuffled letters, so another group's answers won't match yours.

---

## 1. What you need

- the ball-on-beam kit, connected to your laptop by its USB cable (no other wiring);
- the kit card: kit number, Kp and linkage gain G;
- the shim (a small spacer) for the tilt experiment;
- a ruler.

The beam has marks at **15 cm** (centre), **18 cm** and **23 cm**, measured from the sensor face. Check them with the ruler before you start.

## 2. Safety

- Touch the **ball** only while the kit is running. Never hold the beam or the linkage: they move suddenly.
- Keep fingers away from the servo crank and link (pinch points).
- Keep the USB cable slack so the rig can't be dragged off the bench.
- Use only the MechSpace power supply provided for the servo.
- If anything strains, buzzes continuously or smells hot, press Ctrl+C in the logger (it stops and levels the beam) and tell a demonstrator.
- The kit stops itself if it loses sight of the ball or stays at full tilt for 8 s.

---

## 3. Setup (once)

1. Install Python 3, then in a terminal:
   ```
   pip install pyserial numpy matplotlib
   ```
2. Find the kit's port: in the Arduino IDE, **Tools → Port** (e.g. `COM5` on Windows, `/dev/cu.usbmodem14101` on Mac, `/dev/ttyACM0` on Linux). Then **close the Arduino Serial Monitor and Serial Plotter**: only one program can use the port at a time.
3. Start the menu:
   ```
   python ball_logger.py --port COM5
   ```

You don't need to edit or open any Arduino code.

## 4. Using the menu

```
================ Mystery Controller ================
 Selected controller: A
  a / b / c   choose controller
  1           Release from end stop (Experiment 1)
  2           Period check, 3 cm start (hand calculation)
  3           Shim test (Experiment 2)
  4           Baseline release from 23 cm mark (Experiment 3a)
  5           Hold 5 s on 23 cm mark, then release (Experiment 3b)
  k           measure the ball's acceleration (K calibration)
  s           summary of all saved runs
  q           quit
```

- Choosing a letter always stops the kit and clears its memory.
- Every test follows the same pattern: setup instructions → you press Enter → **3, 2, 1, GO** → recording → results and a plot.
- Each run is saved in the `runs` folder as `<letter>_<test>_<trial>.csv` and `.png`. Keep them all: they are your evidence.
- Close the plot window to return to the menu.
- **Fallback without Python:** in the Arduino Serial Monitor (115200 baud) type `a`/`b`/`c` to select, `g` to start, `x` to stop, `p` for Serial Plotter output. You won't get saved CSVs or automatic metrics this way, so use it only to look around.

---

## 5. Procedures

Run **every test on every letter**, **three trials each** (minimum two). Use exactly the same procedure for A, B and C.

Before each experiment, write down a **prediction** on your sheet.

### Experiment 1: release test

1. Menu: choose the letter, then `1`.
2. Kit is stopped and level. Rest the ball against the **far** end stop (away from the sensor).
3. Hands off at GO. Records 20 s.

*Prediction question:* which terms (P, I, D) can remove energy from a rolling ball?

### Experiment 2: shim test

1. Measure the shim height h and the distance L_base between the base's supports. Record both.
2. Put the shim under the base on the marked side.
3. Menu: choose the letter, then `3`. Hold the ball on the 15 cm mark and let go at GO. Records 45 s; leave it alone.
4. Remove the shim afterwards.

*Prediction question:* the shim adds a constant tilt. Once the ball has stopped (v = 0), which terms can still produce a tilt to cancel it, and which need the ball to be off-centre to do so?

### Experiment 3: matched release vs hold

Both parts start from the **23 cm mark** (8 cm past centre). Shim removed.

- **3a baseline:** menu `4`. Hold the ball on the mark with one fingertip; let go exactly at GO. The controller starts at the same moment, with an empty memory.
- **3b hold:** menu `5`. Hold the ball on the mark. At GO the controller starts; **keep holding** for 5 s until the logger says **RELEASE NOW**, then let go.

The only difference between 3a and 3b is the 5 s the controller spends with the ball held off-centre.

*Prediction question:* which term "remembers" how long the ball has been off-centre? What should that memory do after release?

### Period check (for the hand calculation)

1. Use the letter you believe is P.
2. Menu `2`. Hold the ball on the 18 cm mark (3 cm past centre); let go at GO.
3. Use the **first** period the logger prints. Later cycles may grow and hit the end stops, which shortens them.

### Calibration: measure K

1. Menu `k`. Kit stopped and level, ball against the **near** end stop (sensor end).
2. At GO the beam tilts to a fixed 10 servo degrees with the controller off, and the ball rolls away.
3. The logger fits the ball's acceleration a and prints K = a/10 and c. Repeat three times.

### Bonus observation

With each letter holding the ball near the centre, listen to and watch the servo. Which letters twitch most? Why might that be?

---

## 6. What the logger measures

| Metric | Meaning |
|---|---|
| Overshoot | furthest the ball goes past the centre on the far side (cm, and % of the starting distance) |
| Settling time | time until the ball enters the ±2 cm band and stays there |
| Final mean | average position relative to centre over the last 5 s (+ = away from sensor) |
| Final state | **settled at target** (still, inside ±2 cm), **stationary off-centre** (still, but outside ±2 cm) or **no steady state** (still moving or drifting) |
| Periods | time for each full oscillation |
| Loop timing | how regularly the kit took readings |

"Stationary off-centre" is a real result, not a failed run: it means the ball stopped with an error.

After all runs, menu `s` prints a summary of every letter and test (mean and range over trials) and the **hold − baseline** overshoot difference for each letter. Copy those numbers into the tables below and check them against the plots.

---

## 7. Results sheet

**Kit number:** ____ **Group:** ____ **Shim h:** ____ mm **L_base:** ____ mm

### Table 1: measurements (mean over trials, range in brackets)

| Letter | Exp 1 overshoot (cm, %) | Exp 1 settling (s) | Exp 1 final state | Exp 2 final mean (cm) | Exp 2 final state | Exp 3a overshoot (cm) | Exp 3b overshoot (cm) | 3b − 3a (cm) | Servo twitching | Trials per test |
|---|---|---|---|---|---|---|---|---|---|---|
| A | | | | | | | | | | |
| B | | | | | | | | | | |
| C | | | | | | | | | | |

### Table 2: identification

| Letter | Our answer (P / PD / PID) | Deciding evidence (experiment, metric, plot file) | Why this rules out the other two |
|---|---|---|---|
| A | | | |
| B | | | |
| C | | | |

### Table 3: hand calculation

**Ideal model.** Derive the acceleration of a solid ball rolling on a beam at angle θ, then substitute P control to get the oscillation period.

| Quantity | Value |
|---|---|
| Ideal: a = ____·g·θ, derivation shown | |
| Kp (kit card, servo deg/cm) and G (beam deg per servo deg) | |
| Kp converted to beam rad per cm: G·(π/180)·Kp | |
| Ideal period T = 2π/√(c·g·G·(π/180)·Kp) with c = 5/7 | |
| Measured K and c from calibration (mean of 3) | |
| Calibrated period T = 2π/√(K·Kp) | |
| Measured first period (period check, mean of trials) | |
| Differences (%) and your explanation | |
| Shim: predicted tilt δ = (h/L_base)·(180/π) and predicted offset with no integral action, δ/(G·Kp) | |
| Measured offset of the letter that stopped off-centre | |

Things worth thinking about: why might c differ from 5/7 on two rails? What does the sensor filtering do to the measured period? Does the oscillation grow, stay steady or shrink, and why?

---

## 8. Scoring (out of 100)

| Component | Points | How it's awarded |
|---|---|---|
| Identification | 30 | 10 per correct letter |
| Evidence | 40 | per letter, up to 10 for deciding evidence (named plot and number that separate it from the other two); up to 10 for completeness (all tests and trials with run files) |
| Hand calculation | 20 | 8 for method and consistent units; 4 for measured K and c; 8 for comparing with your measured period and explaining the difference. Close agreement is not required: a well-explained discrepancy scores fully |
| Speed | 10 | first group with all three correct: 10, then 2 fewer per later group |

- A correct letter without evidence scores identification points only.
- Speed points need all three correct and at least one trial of every test.
- Ties are broken by evidence, then hand calculation, then time.

Submit: the results sheet, your whole `runs` folder (CSVs, plots, `summary.csv`), and your kit number.
