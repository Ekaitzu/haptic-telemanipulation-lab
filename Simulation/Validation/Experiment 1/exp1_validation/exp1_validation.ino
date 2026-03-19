// =============================================================================
// exp1_validation.ino
// Experiment 1 Validation — Local Virtual Environment (Spring-Damper + Wall)
//
// PURPOSE:
//   Stripped-down 1 kHz haptic loop for hardware-vs-simulation comparison.
//   Identical sensing pipeline, kinematics, filter, control law, and motor
//   output to the original master firmware. Everything non-essential has been
//   removed. Serial output streams CSV data for post-processing in MATLAB or
//   Python.
//
// HARDWARE:
//   Hapkit board (ATmega328P) — Master module only. No slave required.
//
// SERIAL OUTPUT (500000 baud, CSV, one line per ms):
//   t_ms, x_mm, dx_mms, forceN, motorEnabled
//
//   t_ms        : time since trigger [ms]
//   x_mm        : handle position    [mm]
//   dx_mms      : handle velocity    [mm/s]
//   forceN      : rendered force     [N]
//   motorEnabled: 1 = motor on, 0 = off
//
// OPERATION:
//   1. Flash to the master Hapkit board.
//   2. Open a serial terminal at 500000 baud — you will see a menu.
//   3. (Optional) Adjust K, B, RestPos, WallPos, WallK via the menu before
//      each run (see EXPERIMENT PARAMETERS section below for defaults).
//   4. Hold the handle at the desired initial displacement (~5 mm from rest).
//   5. Send 'G' (Go) over serial. The motor enables and data logging starts.
//   6. Release the handle.
//   7. Send 'S' (Stop) to disable the motor and stop logging.
//   8. Copy the CSV output into MATLAB / Python and plot against simulation.
//
// MENU COMMANDS (send over serial, uppercase):
//   L        — Toggle live position monitor (10 Hz readout before a run)
//   G        — Start: enable motor, begin logging
//   S        — Stop:  disable motor, stop logging
//   R        — Reset calibration (re-zero the sensor)
//   K<value> — Set spring stiffness   e.g. "K30.0"  → K = 30 N/m
//   B<value> — Set damping            e.g. "B1.5"   → B = 1.5 Ns/m
//   P<value> — Set rest position      e.g. "P0.0"   → RestPos = 0 m
//   W<value> — Set wall position      e.g. "W0.02"  → WallPos = 0.02 m
//   Q<value> — Set wall stiffness     e.g. "Q0.0"   → WallK = 0 (disabled)
//   ?        — Print current parameters
//
// =============================================================================

// ── Pin Definitions (identical to master firmware) ────────────────────────
#define PWM_PIN     5
#define DIR_PIN     8
#define SENSOR_PIN  A2

// ── Physical Constants (identical to master firmware) ─────────────────────
const double rh            = 0.090;      // Handle radius [m]
const double rs            = 0.075;      // Sector radius [m]
const double rp            = 0.010;      // Pinion radius [m]
const double TORQUE_CONST  = 0.0183;     // Motor torque constant [Nm/A]
const double DEG_PER_COUNT = 0.013892;   // Sensor deg/count
const long   OFFSET        = 902;        // Magnetic wraparound threshold

// ── Experiment Parameters (edit here or via serial menu) ──────────────────
// These are the defaults loaded at startup. Change with serial commands
// before sending 'G', or hard-code different sets for each test run.
//
// For the thesis validation, three recommended operating points are:
//   STABLE       K=30,  B=1.5, no wall  → clearly decaying
//   NEAR-CRITICAL K=60, B=0.5, no wall  → sustained / slow decay
//   UNSTABLE     K=100, B=0.3, no wall  → growing oscillation (brief test!)
double K       = 30.0;   // Spring stiffness    [N/m]
double B       = 1.5;    // Damping coefficient [Ns/m]
double RestPos = 0.0;    // Rest position       [m]
double WallPos = 0.02;   // Wall position       [m]  (disabled when WallK=0)
double WallK   = 0.0;    // Wall stiffness      [N/m] (0 = no wall)

// ── Logging Duration ──────────────────────────────────────────────────────
// How many milliseconds to record after 'G' is received.
// 3000 ms matches the simulation horizon for Experiment 1.
const unsigned long LOG_DURATION_MS = 500;

// ── Velocity Filter Coefficient (identical to master firmware) ────────────
// alpha = 0.92 gives a cutoff of ~12 Hz at 1 kHz, matching the original.
const double ALPHA = 0.92;

// ── Force Limits (identical to master firmware) ───────────────────────────
const double FORCE_MAX   =  3.0;   // [N]
const double FORCE_MIN   = -3.0;   // [N]
const double DEADBAND    =  0.00;  // [N]  — suppresses micro-jitter at zero

// ── Internal State ────────────────────────────────────────────────────────
int  rawPos         = 0;
int  lastRawPos     = 0;
int  lastLastRawPos = 0;
bool flipped        = false;
long flipNumber     = 0;

long   pos0              = 0;
long   pos0_sum          = 0;
int    zeroCounter       = 0;
bool   zeroCaptured      = false;
const int ZERO_SETTLE_MS = 100;
const int ZERO_AVG_MS    = 200;

double x_prev  = 0.0;
double dx_f    = 0.0;

bool         motorEnabled  = false;
bool         logging       = false;
bool         liveMode      = false;   // Stream position to serial at 10 Hz
unsigned long logStartMs   = 0;
unsigned long lastLiveTick = 0;
unsigned long systemTimeMs = 0;

// ── Serial input buffer ───────────────────────────────────────────────────
char   cmdBuf[16];
uint8_t cmdIdx = 0;

// ── Timer setup (identical to master firmware) ───────────────────────────
void setupPWM31kHz() {
  // Timer 0: 31.25 kHz PWM on pin 5 — reduces audible motor whine.
  TCCR0A = (TCCR0A & 0b11111100) | 0x01;
  TCCR0B = (TCCR0B & 0b11111000) | 0x01;
}

void setupTimer1Timebase() {
  // Timer 1: CTC mode, 1 kHz tick (16 MHz / 8 / 2000 = 1000 Hz)
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A  = 1999;
  TCNT1  = 0;
  TIFR1 |= (1 << OCF1A);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(500000);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(PWM_PIN,    OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);

  setupPWM31kHz();
  setupTimer1Timebase();

  printBanner();
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  // ── Serial command processing ──────────────────────────────────────────
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else if (cmdIdx < 15) {
      cmdBuf[cmdIdx++] = c;
    }
  }

  // ── Strict 1 kHz execution via Timer 1 compare flag ───────────────────
  if (TIFR1 & (1 << OCF1A)) {
    TIFR1 |= (1 << OCF1A);   // Clear flag immediately
    systemTimeMs++;
    run_1khz_loop();
  }
}

// =============================================================================
// 1 kHz HAPTIC LOOP  (sensor → kinematics → control → motor → log)
// =============================================================================
void run_1khz_loop() {

  // ── Step 1: Sensor read & wraparound detection (identical to original) ─
  rawPos  = analogRead(SENSOR_PIN);

  int rawDiff     = rawPos - lastRawPos;
  int lastRawDiff = rawPos - lastLastRawPos;
  int rawOffset   = abs(rawDiff);
  int lastRawOffset = abs(lastRawDiff);

  lastLastRawPos = lastRawPos;
  lastRawPos     = rawPos;

  if ((lastRawOffset > 700) && (!flipped)) {
    if (lastRawDiff > 0) flipNumber--;
    else                 flipNumber++;
    flipped = true;
  } else {
    flipped = false;
  }

  long updatedPos = (long)rawPos + flipNumber * OFFSET;

  // ── Step 2: Auto-calibration — average 200 samples after 100 ms settle ─
  if (!zeroCaptured) {
    zeroCounter++;
    if (zeroCounter > ZERO_SETTLE_MS &&
        zeroCounter <= (ZERO_SETTLE_MS + ZERO_AVG_MS)) {
      pos0_sum += updatedPos;
    } else if (zeroCounter > (ZERO_SETTLE_MS + ZERO_AVG_MS)) {
      pos0       = pos0_sum / ZERO_AVG_MS;
      x_prev     = 0.0;
      dx_f       = 0.0;
      zeroCaptured = true;
      Serial.println(F("# Calibration complete. Send 'G' to start."));
    }
    analogWrite(PWM_PIN, 0);
    return;
  }

  // ── Step 3: Forward kinematics (identical to original) ────────────────
  long   posRel   = updatedPos - pos0;
  double theta    = ((double)posRel * DEG_PER_COUNT) * (PI / 180.0);
  double x_m      = rh * theta;                          // handle pos [m]

  double dx_m     = (x_m - x_prev) / 0.001;              // raw velocity [m/s]
  x_prev          = x_m;
  dx_f            = ALPHA * dx_f + (1.0 - ALPHA) * dx_m; // filtered vel [m/s]

  // ── Step 4: Instability watchdog ──────────────────────────────────────
  if (abs(dx_f) > 5.0) {
    // Velocity physically impossible for a human → latch motor off
    motorEnabled = false;
    logging      = false;
    liveMode     = false;
    analogWrite(PWM_PIN, 0);
    Serial.println(F("# INSTABILITY DETECTED — motor latched off. Send 'R' then 'G'."));
    return;
  }

  // ── Step 4b: Live position monitor (10 Hz, active when liveMode = true) ─
  // Prints a single human-readable line so you can see exactly where the
  // handle is before committing to a run. Does NOT affect the control loop.
  if (liveMode && !logging && (systemTimeMs - lastLiveTick >= 100)) {
    lastLiveTick = systemTimeMs;
    Serial.print(F("# POS: "));
    Serial.print(x_m * 1000.0, 2);
    Serial.println(F(" mm"));
  }

  // ── Step 5: Experiment 1 control law (identical to original) ──────────
  double forceN = 0.0;

  if (motorEnabled) {
    // Spring-damper centred at RestPos
    forceN = -(K * (x_m - RestPos)) - (B * dx_f);

    // Optional unilateral wall
    if (WallK > 0.0 && x_m > WallPos) {
      forceN += -WallK * (x_m - WallPos);
    }
  }

  // ── Step 6: Safety clamps (identical to original) ─────────────────────
  if (forceN >  FORCE_MAX) forceN =  FORCE_MAX;
  if (forceN <  FORCE_MIN) forceN =  FORCE_MIN;
  if (fabs(forceN) < DEADBAND) forceN = 0.0;

  // ── Step 7: Motor output (identical to original) ──────────────────────
  double Tp   = forceN * rh * rp / rs;
  digitalWrite(DIR_PIN, (Tp >= 0) ? LOW : HIGH);

  double duty = sqrt(fabs(Tp) / TORQUE_CONST);
  if (duty > 1.0) duty = 1.0;
  analogWrite(PWM_PIN, (int)(duty * 255.0));

  // ── Step 8: CSV data logging ───────────────────────────────────────────
  if (logging) {
    unsigned long elapsed = systemTimeMs - logStartMs;

    // Print one CSV row: t_ms, x_mm, dx_mms, forceN, motorEnabled
    Serial.print(elapsed);
    Serial.print(',');
    Serial.print(x_m  * 1000.0, 4);   // convert m → mm
    Serial.print(',');
    Serial.print(dx_f * 1000.0, 3);   // convert m/s → mm/s
    Serial.print(',');
    Serial.print(forceN, 4);
    Serial.print(',');
    Serial.println(motorEnabled ? 1 : 0);

    // Auto-stop after LOG_DURATION_MS
    if (elapsed >= LOG_DURATION_MS) {
      motorEnabled = false;
      logging      = false;
      analogWrite(PWM_PIN, 0);
      Serial.println(F("# Logging complete."));
    }
  }
}

// =============================================================================
// SERIAL COMMAND PROCESSOR
// =============================================================================
void processCommand(const char* cmd) {
  char c = cmd[0];

  switch (c) {

    case 'L': case 'l':
      liveMode = !liveMode;
      if (liveMode) Serial.println(F("# Live mode ON  — move handle to target position, then send 'G'."));
      else          Serial.println(F("# Live mode OFF."));
      break;

    case 'G': case 'g':
      if (!zeroCaptured) {
        Serial.println(F("# Not calibrated yet. Wait for 'Calibration complete'."));
        return;
      }
      motorEnabled = true;
      logging      = true;
      liveMode     = false;   // Stop live stream — CSV takes over
      logStartMs   = systemTimeMs;
      // Print CSV header so the output is self-describing
      Serial.println(F("# --- RUN START ---"));
      Serial.print(F("# K="));    Serial.print(K,1);
      Serial.print(F(" B="));     Serial.print(B,3);
      Serial.print(F(" RP="));    Serial.print(RestPos*1000.0,1);  Serial.print(F("mm"));
      Serial.print(F(" WP="));    Serial.print(WallPos*1000.0,1);  Serial.print(F("mm"));
      Serial.print(F(" WK="));    Serial.println(WallK,1);
      Serial.println(F("t_ms,x_mm,dx_mms,forceN,motorEnabled"));
      break;

    case 'S': case 's':
      motorEnabled = false;
      logging      = false;
      liveMode     = false;
      analogWrite(PWM_PIN, 0);
      Serial.println(F("# Stopped."));
      break;

    case 'R': case 'r':
      motorEnabled = false;
      logging      = false;
      liveMode     = false;
      analogWrite(PWM_PIN, 0);
      zeroCaptured = false;
      zeroCounter  = 0;
      pos0_sum     = 0;
      flipNumber   = 0;
      x_prev       = 0.0;
      dx_f         = 0.0;
      Serial.println(F("# Calibration reset. Hold handle at rest..."));
      break;

    case 'K': case 'k':
      K = atof(cmd + 1);
      Serial.print(F("# K = ")); Serial.println(K, 2);
      break;

    case 'B': case 'b':
      B = atof(cmd + 1);
      Serial.print(F("# B = ")); Serial.println(B, 3);
      break;

    case 'P': case 'p':
      RestPos = atof(cmd + 1);
      Serial.print(F("# RestPos = ")); Serial.print(RestPos * 1000.0, 2);
      Serial.println(F(" mm"));
      break;

    case 'W': case 'w':
      WallPos = atof(cmd + 1);
      Serial.print(F("# WallPos = ")); Serial.print(WallPos * 1000.0, 2);
      Serial.println(F(" mm"));
      break;

    case 'Q': case 'q':
      WallK = atof(cmd + 1);
      Serial.print(F("# WallK = ")); Serial.println(WallK, 1);
      break;

    case '?':
      printParams();
      break;

    default:
      Serial.print(F("# Unknown command: "));
      Serial.println(cmd);
      break;
  }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
void printBanner() {
  Serial.println(F("# ================================================"));
  Serial.println(F("# Experiment 1 Validation — Haptic Lab System"));
  Serial.println(F("# THWS / Ekaitz Uria / Bachelor Thesis 2026"));
  Serial.println(F("# ------------------------------------------------"));
  Serial.println(F("# Commands:"));
  Serial.println(F("#   L         Live position monitor (toggle)"));
  Serial.println(F("#   G         Start logging"));
  Serial.println(F("#   S         Stop"));
  Serial.println(F("#   R         Reset calibration"));
  Serial.println(F("#   K<val>    Set K [N/m]     e.g. K30.0"));
  Serial.println(F("#   B<val>    Set B [Ns/m]    e.g. B1.5"));
  Serial.println(F("#   P<val>    Set RestPos [m] e.g. P0.0"));
  Serial.println(F("#   W<val>    Set WallPos [m] e.g. W0.02"));
  Serial.println(F("#   Q<val>    Set WallK [N/m] e.g. Q0.0"));
  Serial.println(F("#   ?         Print parameters"));
  Serial.println(F("# ------------------------------------------------"));
  Serial.println(F("# Holding handle at rest. Calibrating..."));
  Serial.println(F("# ================================================"));
}

void printParams() {
  Serial.println(F("# --- Current Parameters ---"));
  Serial.print(F("#   K       = ")); Serial.print(K, 2);       Serial.println(F(" N/m"));
  Serial.print(F("#   B       = ")); Serial.print(B, 3);       Serial.println(F(" Ns/m"));
  Serial.print(F("#   RestPos = ")); Serial.print(RestPos * 1000.0, 2); Serial.println(F(" mm"));
  Serial.print(F("#   WallPos = ")); Serial.print(WallPos * 1000.0, 2); Serial.println(F(" mm"));
  Serial.print(F("#   WallK   = ")); Serial.print(WallK, 1);   Serial.println(F(" N/m (0=disabled)"));
  Serial.println(F("# --------------------------"));
}
