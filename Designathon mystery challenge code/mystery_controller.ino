// ===========================================================================
//  MYSTERY CONTROLLER  -  Ball-on-beam kit, MECH0023 Dynamics and Control
// ===========================================================================
//  INSTRUCTOR COPY - this source file is the ANSWER KEY. Flash a compiled
//  copy onto each kit; do not give students this file until the debrief.
//
//  This is a SEPARATE sketch from the normal balance sketch. The balance
//  sketch is not modified; re-flash it whenever the normal mode is needed.
//
//  Three controllers live on the board: P, PD and PID. Students pick them
//  as "A", "B" and "C" over the USB serial link and work out which is which
//  from experiments. The letter -> controller mapping depends on KIT_ID.
//  There are only 6 possible mappings, so kits 6 apart share a mapping;
//  consecutive kit numbers never do. See kit_mapping.csv.
//
//  All three share the SAME Kp. They differ ONLY by adding D, then I.
//
//  Hardware: unchanged from the working balance rig. NO extra parts.
//    Arduino Nano Every, HC-SR04 (Trig D2, Echo D3), servo on D5.
//    The pot on A0 is not used: the target is fixed at the beam centre.
//
//  SERIAL CONTROL (115200 baud; the student logger sends these for you)
//    a / b / c  select controller A, B or C (always stops and resets first)
//    g          go: start the selected controller from a clean state
//    x          stop: controller off, integral cleared, beam level
//    m          mark: tag the next data row as an event (ball released)
//    l          CSV output with board timestamps (used by the logger)
//    p          labelled output for the Arduino Serial Plotter
//    ?          status line (letter, running, kit number - never the law)
//    k          calibration: controller OFF, beam held at a fixed +10 deg
//               servo tilt so the ball's acceleration (and K) can be measured.
//               Reveals nothing about A/B/C. Stops when the ball reaches
//               the far end or after 4 s.
//
//  STATE RULES
//    Power-up / reset   -> stopped, letter A, beam level
//    Selecting a letter -> stop + reset first
//    START              -> integral, velocity filter and timing all reset
//    STOP               -> beam level, integral cleared
//    Sensor lost 0.6 s  -> automatic STOP
//    Output at its limit continuously for 8 s -> automatic STOP
// ===========================================================================

#include <Servo.h>

// ---------------------------------------------------------------------------
// KIT SETUP  (instructor sets this before flashing; print it on the kit card)
// ---------------------------------------------------------------------------
const uint8_t KIT_ID = 1;          // 1-99

// ---------------------------------------------------------------------------
// Pins (same as the balance sketch)
// ---------------------------------------------------------------------------
const uint8_t TRIG_PIN  = 2;
const uint8_t ECHO_PIN  = 3;
const uint8_t SERVO_PIN = 5;

// ---------------------------------------------------------------------------
// Servo calibration  -  RE-MEASURE ON EACH KIT (procedure in HOW_IT_WORKS.md)
//   LEVEL_ANGLE : servo angle with the beam level (spirit level on the rails)
//   DIRECTION   : +1 if increasing the servo angle rolls the ball AWAY from
//                 the sensor, otherwise -1
//   SERVO_MIN/MAX_DEG : the furthest the servo can go each way without the
//                 linkage binding or the beam hitting the base
// Values below are the last recorded calibration (level 104, limits 74-134).
// ---------------------------------------------------------------------------
const int   LEVEL_ANGLE   = 104;
const int   DIRECTION     = 1;
const int   SERVO_MIN_DEG = 74;
const int   SERVO_MAX_DEG = 134;
const float MAX_TILT      = 30.0;  // controller output limit, deg from level

// Target and valid sensor window (cm from the sensor face)
const float SETPOINT_CM  = 15.0;
const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 28.5;

// ---------------------------------------------------------------------------
// Gains. u is in servo degrees from level, e in cm, v in cm/s, integral in cm*s
//   Plant model: x'' = K*u, K ~ 1.69 cm/s^2 per servo degree, MEASURED on the
//   aluminium rails with the ping-pong ball. Re-measure on the current rails.
//   Kp = wn^2 / K       (wn = 1.5 rad/s)              -> 1.3 deg/cm
//   Kd = 2*zeta*wn / K  (zeta = 0.8)                  -> 1.4 deg per cm/s
//   Ki ~ 0.23 * Kp      (slow enough to see the creep) -> 0.3 deg per cm*s
//   If the balance sketch was retuned, copy its Kp and Kd here.
// ---------------------------------------------------------------------------
const float KP = 1.3;              // <- printed on the kit card
const float KD = 1.4;
const float KI = 0.3;

// Integral cap: limits the I CONTRIBUTION to +-20 deg. This is deliberately
// NOT full anti-windup, so wind-up stays visible in the hold test. It does
// not stop the TOTAL output saturating at +-MAX_TILT.
const float I_LIMIT_DEG = 20.0;

// ---------------------------------------------------------------------------
// Timing. The HC-SR04 datasheet asks for a measurement cycle over 60 ms,
// otherwise a late echo from the previous ping can be read as the ball.
// One ping per tick, so the tick is 65 ms (~15 Hz). The controller uses the
// MEASURED time between ticks (dt) for velocity and the integral, and dt is
// logged so jitter can be checked.
// ---------------------------------------------------------------------------
const unsigned long LOOP_MS  = 65;
const float NOMINAL_DT       = LOOP_MS / 1000.0;
const unsigned long ECHO_TIMEOUT_US = 20000UL;  // see note in readRawCm()

// Filters (identical for all three controllers)
const float POS_ALPHA = 0.6;       // low-pass on position (1 = no filtering)
const float VEL_ALPHA = 0.5;       // low-pass on velocity

const unsigned long SENSOR_LOST_MS = 600;
const unsigned long SAT_STOP_MS    = 8000;
const float CAL_TILT_DEG = 10.0;           // fixed tilt for the K calibration
const unsigned long CAL_MAX_MS = 4000;

// ---------------------------------------------------------------------------
// The shuffle
// ---------------------------------------------------------------------------
enum Ctrl : uint8_t { CTRL_P = 0, CTRL_PD = 1, CTRL_PID = 2 };

const uint8_t PERMS[6][3] = {
  {CTRL_P,   CTRL_PD,  CTRL_PID},
  {CTRL_P,   CTRL_PID, CTRL_PD },
  {CTRL_PD,  CTRL_P,   CTRL_PID},
  {CTRL_PD,  CTRL_PID, CTRL_P  },
  {CTRL_PID, CTRL_P,   CTRL_PD },
  {CTRL_PID, CTRL_PD,  CTRL_P  }
};
uint8_t permIndex() { return (uint8_t)((KIT_ID * 5 + 2) % 6); }

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
Servo servo;

uint8_t letter  = 0;        // 0 = A, 1 = B, 2 = C
bool    running = false;
bool    csvMode = false;
bool    eventFlag = false;
bool    calMode = false;        // fixed-tilt calibration run (no controller)

float hist[3];
float posCm   = SETPOINT_CM;
float prevPos = SETPOINT_CM;
float velCms  = 0;
float integ   = 0;
float tiltDeg = 0;          // tilt actually sent (after all clamps)
float dtUsed  = NOMINAL_DT;

unsigned long lastValidMs = 0;
unsigned long runStartMs  = 0;
unsigned long nextLoopMs  = 0;
unsigned long lastTickUs  = 0;
unsigned long satSinceMs  = 0;
bool          saturated   = false;

// ---------------------------------------------------------------------------
// Sensor  (if the working balance sketch reads the sensor differently and
// reliably, paste its method in here and keep everything else)
// ---------------------------------------------------------------------------
float readRawCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // 20 ms timeout: blocks at most 20 ms of the 65 ms tick. Do not cut it to
  // a few ms - pulseIn also counts the sensor's start-up delay before the
  // echo pin goes high, and a short timeout returns nothing at all.
  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1;
  float cm = us * 0.0343 / 2.0;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1;
  return cm;
}

float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

void updatePosition(float dt) {
  float raw = readRawCm();
  if (raw > 0) {
    hist[0] = hist[1]; hist[1] = hist[2]; hist[2] = raw;
    lastValidMs = millis();
  }
  float m = median3(hist[0], hist[1], hist[2]);
  posCm = POS_ALPHA * m + (1.0 - POS_ALPHA) * posCm;

  float vRaw = (posCm - prevPos) / dt;
  prevPos = posCm;
  velCms = VEL_ALPHA * vRaw + (1.0 - VEL_ALPHA) * velCms;
}

// ---------------------------------------------------------------------------
// The three controllers
// ---------------------------------------------------------------------------
float computeTilt(uint8_t ctrl, float dt) {
  // e > 0: ball too close to the sensor. u > 0: ball rolls AWAY from the
  // sensor. So positive error -> positive tilt -> ball pushed to the centre.
  float e = SETPOINT_CM - posCm;

  float u = KP * e;

  if (ctrl == CTRL_PD || ctrl == CTRL_PID) {
    // Target fixed, so de/dt = -v. The derivative term Kd*de/dt is therefore
    // -Kd*v ("derivative on measurement").
    u -= KD * velCms;
  }

  if (ctrl == CTRL_PID) {
    integ += e * dt;
    float iMax = I_LIMIT_DEG / KI;
    if (integ >  iMax) integ =  iMax;
    if (integ < -iMax) integ = -iMax;
    u += KI * integ;
  }

  if (u >  MAX_TILT) u =  MAX_TILT;
  if (u < -MAX_TILT) u = -MAX_TILT;
  return u;
}

// Final clamp in SERVO degrees, so the calibrated limits always win even if
// they are not symmetric about the level angle. Returns the tilt really sent.
float writeServo(float tilt) {
  int angle = LEVEL_ANGLE + DIRECTION * (int)round(tilt);
  if (angle < SERVO_MIN_DEG) angle = SERVO_MIN_DEG;
  if (angle > SERVO_MAX_DEG) angle = SERVO_MAX_DEG;
  servo.write(angle);
  tiltDeg = (float)(DIRECTION * (angle - LEVEL_ANGLE));
  return tiltDeg;
}

// ---------------------------------------------------------------------------
// Start / stop / select
// ---------------------------------------------------------------------------
void resetController() {
  integ = 0;
  velCms = 0;
  prevPos = posCm;
  saturated = false;
}

void stopRun() {
  running = false;
  calMode = false;
  resetController();
  writeServo(0);
}

void startRun() {
  if (millis() - lastValidMs > SENSOR_LOST_MS) {
    Serial.println(F("# Cannot start: no ball reading. Check the sensor."));
    return;
  }
  resetController();
  running = true;
  runStartMs = millis();
  lastTickUs = micros();
}

void startCalibration() {
  stopRun();
  startRun();
  if (running) calMode = true;
}

void selectLetter(uint8_t l) {
  stopRun();
  letter = l;
}

void printStatus() {
  Serial.print(F("# kit "));     Serial.print(KIT_ID);
  Serial.print(F(" letter "));   Serial.print((char)('A' + letter));
  Serial.print(F(" running "));  Serial.println(running ? 1 : 0);
}

void handleSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'a': case 'A': selectLetter(0); break;
      case 'b': case 'B': selectLetter(1); break;
      case 'c': case 'C': selectLetter(2); break;
      case 'g': case 'G': stopRun(); startRun(); break;
      case 'k': case 'K': startCalibration(); break;
      case 'x': case 'X': stopRun(); break;
      case 'm': case 'M': eventFlag = true; break;
      case 'l': case 'L':
        csvMode = true;
        Serial.println(F("t_s,dt_ms,letter,running,event,ball_cm,target_cm,tilt_deg"));
        break;
      case 'p': case 'P': csvMode = false; break;
      case '?': printStatus(); break;
      default: break;
    }
  }
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
void printData() {
  char L = calMode ? 'K' : (char)('A' + letter);
  if (csvMode) {
    float t = running ? (millis() - runStartMs) / 1000.0 : 0.0;
    Serial.print(t, 3);               Serial.print(',');
    Serial.print(dtUsed * 1000.0, 1); Serial.print(',');
    Serial.print(L);                  Serial.print(',');
    Serial.print(running ? 1 : 0);    Serial.print(',');
    Serial.print(eventFlag ? 1 : 0);  Serial.print(',');
    Serial.print(posCm, 2);           Serial.print(',');
    Serial.print(SETPOINT_CM, 1);     Serial.print(',');
    Serial.println(tiltDeg, 1);
  } else {
    Serial.print(F("ball_cm:"));    Serial.print(posCm, 2);
    Serial.print(F(",target:"));    Serial.print(SETPOINT_CM, 1);
    Serial.print(F(",upper_2cm:")); Serial.print(SETPOINT_CM + 2.0, 1);
    Serial.print(F(",lower_2cm:")); Serial.print(SETPOINT_CM - 2.0, 1);
    Serial.print(F(",tilt_deg:"));  Serial.print(tiltDeg, 1);
    Serial.print(F(",ctrl_A1_B2_C3:")); Serial.println(letter + 1);
  }
  eventFlag = false;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);
  writeServo(0);

  for (int i = 0; i < 3; i++) hist[i] = SETPOINT_CM;
  lastValidMs = millis();
  nextLoopMs  = millis();
  lastTickUs  = micros();

  Serial.print(F("# Mystery Controller kit "));
  Serial.print(KIT_ID);
  Serial.println(F(". Stopped on A. Keys: a/b/c select, g go, x stop, m mark, l csv, p plot, ? status."));
}

void loop() {
  handleSerial();

  unsigned long now = millis();
  if ((long)(now - nextLoopMs) < 0) return;
  nextLoopMs += LOOP_MS;
  if ((long)(now - nextLoopMs) > (long)LOOP_MS) nextLoopMs = now + LOOP_MS;  // fell behind: resync

  // Measured time since the previous tick, kept within sensible bounds
  unsigned long nowUs = micros();
  float dt = (nowUs - lastTickUs) / 1.0e6;
  lastTickUs = nowUs;
  if (dt < 0.5 * NOMINAL_DT) dt = 0.5 * NOMINAL_DT;
  if (dt > 2.0 * NOMINAL_DT) dt = 2.0 * NOMINAL_DT;
  dtUsed = dt;

  updatePosition(dt);

  if (running && millis() - lastValidMs > SENSOR_LOST_MS) {
    stopRun();
    Serial.println(F("# Stopped: lost the ball reading."));
  }

  if (running && calMode) {
    writeServo(CAL_TILT_DEG);
    if (posCm > MAX_VALID_CM - 3.0 || millis() - runStartMs > CAL_MAX_MS) {
      stopRun();
      Serial.println(F("# Stopped: calibration run complete."));
    }
  } else if (running) {
    uint8_t ctrl = PERMS[permIndex()][letter];
    float u = computeTilt(ctrl, dt);
    writeServo(u);

    bool atLimit = (fabs(u) >= MAX_TILT - 0.01);
    if (atLimit && !saturated) { saturated = true; satSinceMs = millis(); }
    if (!atLimit) saturated = false;
    if (saturated && millis() - satSinceMs > SAT_STOP_MS) {
      stopRun();
      Serial.println(F("# Stopped: output at its limit for 8 s."));
    }
  } else {
    writeServo(0);
  }

  printData();
}
