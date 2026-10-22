// SAVED WORKING VERSION, 22 Oct 2026 - ball balanced at the pivot.
// Copy of ../../ball_on_beamfable2.ino at the time; open this folder in the IDE to upload it as-is.
//
// Ball on beam - fresh start, 22 Oct 2026.
//
// Hardware: Arduino Nano Every, SG90 servo on D5, HC-SR04 (Trig D2, Echo D3).
// Geometry: sensor at one end of the beam, pivot ~17 cm from the sensor,
//           ball balances at servo ~140 deg, ball travel ~2.8 to ~27.6 cm.
//
// Sign convention: tilt > 0 makes the ball roll AWAY from the sensor.
//   servo angle = level + direction * tilt
//
// Serial 115200, one CSV line per 40 ms loop:
//   setpoint_cm,distance_cm,tilt_deg,raw_cm,ping_cm
//   distance_cm = estimated ball position, raw_cm = reading used this loop (-1 = rejected),
//   ping_cm = shorter of this loop's two pings (0 = no echo).
//
// Commands (type into the Serial Monitor, Enter to send):
//   t5      fixed tilt of 5 deg, control off  -> ball must roll AWAY from the sensor
//   h       hold the beam level, control off
//   g       go: balance the ball (this is the mode after reset)
//   r       reverse direction (use if t5 rolls the ball TOWARD the sensor)
//   s17     setpoint, cm from the sensor
//   l140    level servo angle
//   p0.8  d1.0  i0.3    PID gains
//   m20     max tilt either side of level (servo deg)
//   w3      max servo change per 40 ms loop (deg) - lower = smoother, slower beam
//   b3      slop compensation (servo deg jumped when the beam reverses); b0 = off
//   ?       print settings

#include <Servo.h>

// ---------- Pins ----------
const int PIN_SERVO = 5;
const int PIN_TRIG  = 2;
const int PIN_ECHO  = 3;

// ---------- Geometry / servo ----------
const float PIVOT_CM  = 17.0;   // pivot distance from the sensor; default setpoint
// Level by eye is ~137.5, but the ball balances ~2.5 deg higher: in deadzone_01 it
// only rolled away at +6 and back at -2, and runs 10-12 sat at +3 on average.
float levelDeg        = 140.0;  // servo angle where the ball balances
int   direction       = 1;
float maxTilt  = 20.0;          // servo deg either side of level (was 30: too violent)
float tiltStep = 3.0;           // max servo change per loop, deg (was 6: too jerky)
// Slop: the servo can move ~8 deg (deadzone_01) before the beam/ball respond.
// When the tilt command reverses, the output jumps by this much across the slop.
float backlash = 5.0;            // live value when the ball balanced (run_14); 3 also held (run_13)
const float BACKLASH_HYST = 0.5; // command must reverse this far to count as a reversal

// ---------- Loop ----------
const unsigned long LOOP_MS = 40;   // 25 Hz
const float DT = LOOP_MS / 1000.0;

// ---------- Sensor ----------
const unsigned long ECHO_TIMEOUT_US = 8000;
const unsigned long PING_GAP_MS = 18;  // let the first ping's echoes die out
const float RANGE_MIN_CM = 2.5;        // ball against the near stop reads ~2.8
const float RANGE_MAX_CM = 28.0;       // far stop ~27.6; the end block reads ~29.5
// With the ball nearer than ~13.5 cm the sensor often sees a false echo at
// 15.0-16.4 cm (the pivot area; run_08 had it on ~25% of loops with the ball at
// 9-11 cm). Readings in this band are only trusted while they agree closely
// with where the ball is predicted to be.
const float PHANTOM_LO_CM   = 14.8;
const float PHANTOM_HI_CM   = 17.3;
const float PHANTOM_OPEN_CM = 14.5;

// ---------- Position / velocity estimator (alpha-beta filter) ----------
const float ALPHA = 0.4;          // position correction, 0..1
const float BETA  = 0.1;          // velocity correction (lower = smoother velocity)
const float VEL_MAX = 40.0;       // cm/s; the ball never rolls faster than this
const float VEL_RESID_CM = 1.5;   // one noisy reading can't kick the velocity more than this
const float GATE_FAR_CM  = 3.5;   // 2.5 lost a fast ball in run_09 (real 12.1 rejected at 9.2)
const float GATE_NEAR_CM = 4.0;
const float PHANTOM_AGREE_CM = 1.0; // a phantom-band reading must be this close to the prediction
const int   REACQUIRE_MISSES = 5; // after 0.2 s of rejects, jump to the next sane reading
const int   LOST_MISSES = 25;     // after 1 s with nothing usable, level the beam and wait

// ---------- PID ----------
float Kp = 0.8;    // servo deg per cm
float Ki = 0.3;    // servo deg per cm*s
float Kd = 1.0;    // servo deg per cm/s
const float I_LIMIT = 8.0;    // integral clamp (servo deg)
const float I_ZONE_CM = 6.0;  // only integrate when this close to the setpoint
                              // (run_08: ball sat 6 cm off for 3 s with the integral off)

// ---------- State ----------
enum Mode { MODE_RUN, MODE_FIXED };
Mode  mode      = MODE_RUN;
float fixedTilt = 0.0;
float setpoint  = PIVOT_CM;
float est       = PIVOT_CM;   // estimated ball position (cm from the sensor)
float vel       = 0.0;        // estimated velocity (cm/s, + = away from the sensor)
bool  tracking  = false;
int   misses    = 0;
float integ     = 0.0;
float tilt      = 0.0;        // current tilt command (servo deg from level)
int   moveDir   = 1;          // +1 / -1: direction the tilt command last moved
float movePeak  = 0.0;        // furthest tilt reached in that direction
unsigned long nextLoop = 0;

Servo servo;
char cmdBuf[16];
int  cmdLen = 0;
unsigned long lastCharMs = 0;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Same scale as Servo::write() (0-180 deg -> 544-2400 us) but with sub-degree resolution.
void writeServo(float deg) {
  deg = clampf(deg, 0.0, 180.0);
  servo.writeMicroseconds((int)(544 + deg * (2400 - 544) / 180.0 + 0.5));
}

// One ping, returns cm or -1 on no echo.
float pingCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1.0;
  return us / 58.0;
}

// Two pings, keep the shorter: false echoes almost always read long.
float pingBestCm() {
  float a = pingCm();
  delay(PING_GAP_MS);
  float b = pingCm();
  if (a < 0) return b;
  if (b < 0) return a;
  return a < b ? a : b;
}

// Feeds one reading (-1 = none) to the estimator. Returns true if it was used.
bool updateEstimate(float raw) {
  bool inRange = raw >= RANGE_MIN_CM && raw <= RANGE_MAX_CM;
  bool phantom = raw >= PHANTOM_LO_CM && raw <= PHANTOM_HI_CM && est < PHANTOM_OPEN_CM;

  if (!tracking) {
    if (!inRange || phantom) return false;
    est = raw; vel = 0.0; misses = 0; tracking = true;
    return true;
  }

  float pred = est + vel * DT;
  float r = raw - pred;
  bool ok = inRange && r < GATE_FAR_CM + 0.5 * misses && r > -(GATE_NEAR_CM + misses);
  if (phantom && fabs(r) > PHANTOM_AGREE_CM) ok = false;

  if (!ok && inRange && !phantom && misses >= REACQUIRE_MISSES) {
    est = raw; vel = 0.0; misses = 0;   // lost the ball: jump to the new reading
    return true;
  }

  if (ok) {
    est = pred + ALPHA * r;
    vel = clampf(vel + BETA / DT * clampf(r, -VEL_RESID_CM, VEL_RESID_CM), -VEL_MAX, VEL_MAX);
    misses = 0;
  } else {
    est = pred;
    vel *= 0.8;
    misses++;
    if (misses >= LOST_MISSES) {
      tracking = false;
      est = PIVOT_CM;   // so the phantom check can't lock out the real ball
      vel = 0.0;
      return false;
    }
  }
  if (est < RANGE_MIN_CM || est > RANGE_MAX_CM) {   // ball is against a stop
    est = clampf(est, RANGE_MIN_CM, RANGE_MAX_CM);
    vel = 0.0;
  }
  return ok;
}

void printSettings() {
  Serial.print(F("# mode="));  Serial.print(mode == MODE_RUN ? F("RUN") : F("FIXED"));
  if (mode == MODE_FIXED) { Serial.print(F(" tilt=")); Serial.print(fixedTilt, 1); }
  Serial.print(F(" sp="));     Serial.print(setpoint, 1);
  Serial.print(F(" Kp="));     Serial.print(Kp, 2);
  Serial.print(F(" Ki="));     Serial.print(Ki, 2);
  Serial.print(F(" Kd="));     Serial.print(Kd, 2);
  Serial.print(F(" max="));    Serial.print(maxTilt, 1);
  Serial.print(F(" step="));   Serial.print(tiltStep, 1);
  Serial.print(F(" slop="));   Serial.print(backlash, 1);
  Serial.print(F(" level="));  Serial.print(levelDeg, 1);
  Serial.print(F(" dir="));    Serial.println(direction);
}

void runCommand(const char *c) {
  float v = atof(c + 1);
  bool hasVal = c[1] != '\0';
  switch (tolower(c[0])) {
    case 't': mode = MODE_FIXED; fixedTilt = clampf(v, -30.0, 30.0); break;
    case 'h': mode = MODE_FIXED; fixedTilt = 0.0; break;
    case 'g': mode = MODE_RUN; integ = 0.0; break;
    case 'r': direction = -direction; break;
    case 's': if (hasVal) setpoint = clampf(v, 5.0, 25.0); break;
    case 'l': if (hasVal) levelDeg = clampf(v, 90.0, 180.0); break;
    case 'p': if (hasVal) Kp = v; break;
    case 'i': if (hasVal) { Ki = v; integ = 0.0; } break;
    case 'd': if (hasVal) Kd = v; break;
    case 'm': if (hasVal) maxTilt = clampf(v, 2.0, 30.0); break;
    case 'w': if (hasVal) tiltStep = clampf(v, 0.5, 10.0); break;
    case 'b': if (hasVal) backlash = clampf(v, 0.0, 10.0); break;
    case '?': break;
    default:  Serial.println(F("# unknown command")); return;
  }
  printSettings();
}

// Collects a command line. Also runs it after 100 ms of quiet, so it works
// even with the Serial Monitor set to "No line ending".
void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r' || c == ' ') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; runCommand(cmdBuf); cmdLen = 0; }
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
  if (cmdLen > 0 && millis() - lastCharMs > 100) {
    cmdBuf[cmdLen] = '\0'; runCommand(cmdBuf); cmdLen = 0;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  servo.attach(PIN_SERVO);
  writeServo(levelDeg);
  delay(1000);
  Serial.println(F("# ball on beam - fresh start. t5 = direction test, g = balance, ? = settings"));
  printSettings();
  Serial.println(F("setpoint_cm,distance_cm,tilt_deg,raw_cm,ping_cm"));
  nextLoop = millis();
}

void loop() {
  readCommands();
  if ((long)(millis() - nextLoop) < 0) return;
  nextLoop += LOOP_MS;

  // --- Sensor + estimate ---
  float ping = pingBestCm();
  bool used = updateEstimate(ping);

  // --- Control ---
  float target;
  if (mode == MODE_FIXED) {
    target = fixedTilt;
  } else if (!tracking) {
    target = 0.0;                 // no ball: level and wait
    integ = 0.0;
  } else {
    float err = setpoint - est;   // + = ball too close to the sensor
    if (fabs(err) < I_ZONE_CM)
      integ = clampf(integ + Ki * err * DT, -I_LIMIT, I_LIMIT);
    target = clampf(Kp * err + integ - Kd * vel, -maxTilt, maxTilt);
  }
  tilt += clampf(target - tilt, -tiltStep, tiltStep);

  // --- Slop compensation: track which way the command is moving ---
  if (moveDir > 0) {
    if (tilt > movePeak) movePeak = tilt;
    else if (tilt < movePeak - BACKLASH_HYST) { moveDir = -1; movePeak = tilt; }
  } else {
    if (tilt < movePeak) movePeak = tilt;
    else if (tilt > movePeak + BACKLASH_HYST) { moveDir = 1; movePeak = tilt; }
  }
  float out = tilt + moveDir * backlash / 2.0;
  if (mode == MODE_FIXED) out = tilt;   // keep t/h tests exact

  writeServo(levelDeg + direction * out);

  // --- Log ---
  Serial.print(setpoint, 1);  Serial.print(',');
  Serial.print(est, 1);       Serial.print(',');
  Serial.print(tilt, 1);      Serial.print(',');
  Serial.print(used ? ping : -1.0, 1); Serial.print(',');
  Serial.println(ping > 0 ? ping : 0.0, 2);
}
