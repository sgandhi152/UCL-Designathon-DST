# UCL Designathon — Ball-on-Beam Control Kit (Team DST)

MECH0073 Designathon, October 2026. An educational kit for **MECH0023 Dynamics and Control**: a 3D-printed ball-on-beam rig driven by an Arduino Nano Every, a hobby servo and an ultrasonic distance sensor. The ball's position on the beam is measured by the sensor, the controller tilts the beam via the servo, and the goal is to hold the ball at the centre.

The kit has two stages:

1. **Balance the ball** — a working PID controller that demonstrates the rig closing the loop.
2. **The Mystery Controller challenge** — three hidden controllers (P, PD, PID) are pre-loaded as **A, B and C**. Students never see the source code; they must work out which letter is which by running experiments and reading the ball's response.

## Repository contents

| Path | What it is | Who it's for |
|---|---|---|
| `ball_on_beam_working_2026-10-22.ino` | Firmware for stage 1 — the plain working balance controller (PID, servo, ultrasonic sensor). Flash this to show the rig balancing. | Everyone |
| `Mystery Challenge/mystery_controller.ino` | Firmware for stage 2. Holds the P, PD and PID controllers; the letter→controller mapping is set per kit by `KIT_ID`. Controlled over USB serial (115200 baud): `a`/`b`/`c` select, `g` go, `x` stop, `m` release marker, `l` log, `p` plot. | Flash once per kit |
| `Mystery Challenge/ball_logger.py` | Student tool. Python menu that drives the kit over USB, runs each experiment, logs position vs time to CSV and analyses it (overshoot, settling time within ±2 cm, steady-state offset, oscillation period). Also has a `k` mode to calibrate the plant gain K. | Students |
| `Mystery Challenge/STUDENT_GUIDE.md` | The lab sheet: experiment procedure, what to record, hand calculation for the P-controller period, results table. | Students |
| `Mystery Challenge/sim_rig.py` | Instructor tool. Simulates the firmware's control laws on a model of the rig to predict each controller's signature before bench testing. Uses the same analysis functions as `ball_logger.py`. | Instructors |
| `Mystery Challenge/sim_output.txt` | Saved output of `sim_rig.py` — the predicted signatures. | Instructors |
| `Mystery Challenge/HOW_IT_WORKS.md` | Design notes: control laws, gains, filtering, timing, why each experiment isolates one controller. | Instructors / markers |
| `Mystery Challenge/kit_mapping.csv` | **Answer key** — which letter is P / PD / PID on each kit. Do not give to students. | Instructors only |

## The experiments

Each test isolates one controller's signature:

1. **Release** — release the ball from one end. P oscillates and never settles; PD and PID settle at the centre. *Separates P from PD/PID.*
2. **Period** — measure the P controller's oscillation period and compare with the hand calculation (ball-on-beam is a double integrator, so P alone gives simple harmonic motion and can't settle without derivative action).
3. **Tilt** — a shim under one side of the base gives a constant disturbance. PD settles off-centre and stays there; PID creeps back to the centre. *Separates PD from PID.*
4. **Hold and release** — hold the ball off-centre for several seconds, then release. PID overshoots much more (integral wind-up); PD responds as in the release test. *Confirms PID.*

Students justify each identification with plot evidence in a results table.

## Running it

**Hardware:** Arduino Nano Every, hobby servo, ultrasonic distance sensor, USB cable, laptop. No external power supply needed beyond USB.

**Firmware:** open the `.ino` in the Arduino IDE, select *Arduino Nano Every*, upload. For the challenge, set `KIT_ID` in `mystery_controller.ino` to match the kit before uploading.

**Student logger:**

```
pip install pyserial numpy matplotlib
cd "Mystery Challenge"
python ball_logger.py --port COM5      # or /dev/tty.usbmodem… on macOS
```

Follow the on-screen menu. Runs are saved as CSVs and analysed automatically.

**Instructor simulation:**

```
python sim_rig.py            # predictions table + plot
python sim_rig.py --seeds 10 # more noise realisations
```

## Learning outcomes addressed (MECH0023)

Interpreting time-domain responses; the roles of P, I and D action; steady-state error; damping; integral wind-up; comparing a hand-calculated model (SHM period under P control) with measured data. The data-logging and analysis step links to the MECH0073 capstone's core of taking measured data in and computing a result out.

## Team

Team DST — UCL Mechanical Engineering, MECH0073 Capstone, 2026/27.
