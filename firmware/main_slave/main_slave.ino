#include <util/atomic.h>

// =========================================================================
// 1. HARDWARE DEFINITIONS & CONSTANTS
// =========================================================================
#define PWM_PIN     5
#define DIR_PIN     8
#define SENSOR_PIN  A2

// --- KINEMATIC CONSTANTS ---
const double rh = 0.090;             // Handle radius (m) [cite: 1]
const double rs = 0.075;             // Sector radius (m) [cite: 2]
const double rp = 0.010;             // Pinion radius (m) [cite: 2]
const double TORQUE_CONST = 0.0183;  // Motor torque constant (Nm/A) [cite: 2]

// --- SLAVE CALIBRATION (Yellow Axis) ---
const double DEG_PER_COUNT = 0.014634; // [cite: 3]
const long OFFSET = 913;               // [cite: 3]

// =========================================================================
// 2. GLOBAL SYSTEM STATE
// =========================================================================

struct HapticParams {
  float K; float B; float M;
  float RestPos; float WallPos; float WallK;

  void operator=(const volatile HapticParams& other) {
    K = other.K; B = other.B; M = other.M;
    RestPos = other.RestPos; WallPos = other.WallPos; WallK = other.WallK;
  }

  void operator=(const HapticParams& other) volatile {
    K = other.K; B = other.B; M = other.M;
    RestPos = other.RestPos; WallPos = other.WallPos; WallK = other.WallK;
  }
};

volatile HapticParams activeParams = {100.0, 2.0, 0.0, 0.00, 0.02, 300.0}; // [cite: 8]
volatile int currentExperiment = 1;      
volatile bool motorEnabled = false;
volatile unsigned long systemTimeMs = 0; // [cite: 9]

// --- HAPTIC STATE VARIABLES ---
int rawPos = 0, lastRawPos = 0, lastLastRawPos = 0;
int rawDiff = 0, lastRawDiff = 0; // [cite: 10]
int rawOffset = 0, lastRawOffset = 0;
bool flipped = false;
long flipNumber = 0; // [cite: 11]

bool zeroCaptured = false;
int zeroCounter = 0;
long pos0_sum = 0;
long pos0 = 0;
const int ZERO_SETTLE_MS = 100; // [cite: 12]
const int ZERO_AVG_MS = 200;

double x_prev = 0.0;
double dx_f = 0.0;
volatile double remote_x = 0.0; // [cite: 13]
volatile double remote_dx = 0.0;

// --- UART & WATCHDOG VARIABLES ---
uint8_t packetType = 0;
int rxState = 0; // [cite: 14]
uint8_t rxLen = 0;
uint8_t rxCount = 0;
uint8_t rxBuf[30];
uint8_t rxCks = 0; // [cite: 15]

uint8_t localResetCounter = 0;
volatile unsigned long lastValidRxTimeMs = 0; // ADDED: Safety Watchdog Timer

// --- EXP 3: DELAY RING BUFFER ---
#define MAX_DELAY_MS 20 
volatile float delayBufX[MAX_DELAY_MS];
volatile float delayBufDX[MAX_DELAY_MS]; // [cite: 16]
volatile int bufWriteIdx = 0;

// =========================================================================
// 3. HARDWARE TIMING SETUPS
// =========================================================================

void setupPWM31kHz() {
  TCCR0A = (TCCR0A & 0b11111100) | 0x01; // [cite: 17]
  TCCR0B = (TCCR0B & 0b11111000) | 0x01; 
}

void setupTimer1Timebase() {
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11); // [cite: 18]
  OCR1A = 1999;                        
  TCNT1 = 0;                           
  TIFR1 |= (1 << OCF1A); // [cite: 19]
}

void setup() {
  Serial.begin(500000); 

  pinMode(SENSOR_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);

  setupPWM31kHz();
  setupTimer1Timebase(); // [cite: 20]
}

// =========================================================================
// 4. MAIN BACKGROUND LOOP
// =========================================================================

void loop() {
  uart_rx_process();

  if (TIFR1 & (1 << OCF1A)) {
    TIFR1 |= (1 << OCF1A);
    systemTimeMs++; // [cite: 21]        
    run_1khz_haptic_loop();
  }
}

// =========================================================================
// 5. THE VIP LOOP: 1 kHz HAPTICS
// =========================================================================

void run_1khz_haptic_loop() {
  // --- Step 1: Read Sensor & Handle Wraparound ---
  rawPos = analogRead(SENSOR_PIN);
  rawDiff = rawPos - lastRawPos; 
  lastRawDiff = rawPos - lastLastRawPos; // [cite: 22]
  rawOffset = abs(rawDiff); 
  lastRawOffset = abs(lastRawDiff);
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos; // [cite: 23]

  if ((lastRawOffset > 700) && (!flipped)) {
    if (lastRawDiff > 0) flipNumber--;
    else                 flipNumber++; // [cite: 24]
    flipped = true; // [cite: 25]
  } else {
    flipped = false; // [cite: 26]
  }
  
  long updatedPos = (long)rawPos + flipNumber * OFFSET;

  // --- Step 2: Auto-Calibration ---
  if (!zeroCaptured) {
    zeroCounter++;
    if (zeroCounter > ZERO_SETTLE_MS && zeroCounter <= (ZERO_SETTLE_MS + ZERO_AVG_MS)) { // [cite: 27]
      pos0_sum += updatedPos; // [cite: 28]
    } 
    else if (zeroCounter > (ZERO_SETTLE_MS + ZERO_AVG_MS)) {
      pos0 = pos0_sum / ZERO_AVG_MS;
      x_prev = 0.0; dx_f = 0.0; // [cite: 29]
      zeroCaptured = true;
    }
    analogWrite(PWM_PIN, 0); 
    return; // [cite: 30]
  }

  // --- Step 3: Forward Kinematics ---
  long posRel = updatedPos - pos0;
  double theta_rad = ((double)posRel * DEG_PER_COUNT) * (PI / 180.0);
  double x_m = rh * theta_rad; // [cite: 31]

  double dx_m = (x_m - x_prev) / 0.001; 
  x_prev = x_m;
  dx_f = 0.92 * dx_f + (1.0 - 0.92) * dx_m; // [cite: 32]

  // --- Step 4: Safely load variables ---
  HapticParams g;
  int localExp;
  bool localEnabled;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // [cite: 33]
    g = activeParams;
    localExp = currentExperiment;
    localEnabled = motorEnabled;
  }

  double forceN = 0.0;

  // --- ADDED: LOCAL SAFETY WATCHDOG ---
  if ((localExp == 2 || localExp == 3) && (systemTimeMs - lastValidRxTimeMs > 250)) {
     localEnabled = false; // Kill motor if Master is disconnected
  }

  // --- RING BUFFER RECORDING ---
  delayBufX[bufWriteIdx] = remote_x; // [cite: 34]
  delayBufDX[bufWriteIdx] = remote_dx;

  // --- Step 5: Physics Engine ---
  if (localEnabled) { // [cite: 35]
    if (localExp == 1) {
       forceN = -(g.K * (x_m - g.RestPos)) - (g.B * dx_f);
       if (g.WallK > 0.0 && x_m > g.WallPos) { // [cite: 36]
           forceN += -g.WallK * (x_m - g.WallPos);
       } // [cite: 37]
    }
    else if (localExp == 2) {
       forceN = -g.K * (x_m - remote_x) - g.B * (dx_f - remote_dx);
    } // [cite: 38]
    else if (localExp == 3) {
       int delayMs = (int)g.RestPos;
       delayMs = constrain(delayMs, 0, MAX_DELAY_MS - 1); // [cite: 39]

       int bufReadIdx = bufWriteIdx - delayMs;
       if (bufReadIdx < 0) bufReadIdx += MAX_DELAY_MS;
       double delayed_x = delayBufX[bufReadIdx]; // [cite: 40]
       double delayed_dx = delayBufDX[bufReadIdx];

       forceN = -g.K * (x_m - delayed_x) - g.B * (dx_f - delayed_dx);
    } // [cite: 41]
  }

  // Iterate Ring Buffer
  bufWriteIdx++;
  if (bufWriteIdx >= MAX_DELAY_MS) bufWriteIdx = 0;
  
  // --- Step 6: Safety Clamps ---
  forceN = constrain(forceN, -3.0, 3.0);
  if (abs(forceN) < 0.008) forceN = 0.0;  // [cite: 42]

  // --- Step 7: Motor Output ---
  double Tp = forceN * rh * rp / rs;
  
  // ADDED: MOTOR DIRECTION FIX
  digitalWrite(DIR_PIN, (Tp >= 0) ? LOW : HIGH); 

  double duty = sqrt(fabs(Tp) / TORQUE_CONST);
  if (duty > 1.0) duty = 1.0; // [cite: 44]
  
  analogWrite(PWM_PIN, (int)(duty * 255.0));

  // --- Step 8: Transmit ---
  if (localExp == 2 || localExp == 3) { // [cite: 45]
    uart_tx_kinematics();
  } // [cite: 46]
}

// =========================================================================
// 6. UART COMMS
// =========================================================================

void uart_rx_process() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    switch (rxState) { // [cite: 47]
      case 0: 
        if (b == 0xAA) { rxState = 1; packetType = 1; } // [cite: 48]     
        else if (b == 0xBB) { rxState = 1; packetType = 2; } // [cite: 49]
        break;
      case 1: // [cite: 50]
        if (b == 0x55) rxState = 2;
        else rxState = 0; // [cite: 51]
        break;
      case 2: 
        rxLen = b;
        if ((packetType == 1 && rxLen == 27) || (packetType == 2 && rxLen == 8)) { // [cite: 52]
          rxCount = 0;
          rxCks = b; rxState = 3; // [cite: 53]
        } else {
          rxState = 0;
        } // [cite: 54]
        break;
      case 3: 
        rxBuf[rxCount++] = b;
        rxCks ^= b; // [cite: 55]
        if (rxCount >= rxLen) rxState = 4;
        break;
      case 4: // [cite: 56]
        if (b == rxCks) {
          if (packetType == 1) apply_received_config();
          else if (packetType == 2) apply_received_kinematics(); // [cite: 57]
        }
        rxState = 0; 
        break;
    } // [cite: 58]
  }
}

void apply_received_config() {
  uint8_t newExp = rxBuf[0];
  bool newEnabled = (rxBuf[1] == 1);
  uint8_t newResetCnt = rxBuf[2];
  HapticParams newParams; // [cite: 59]
  memcpy(&newParams.K,       &rxBuf[3], 4);
  memcpy(&newParams.B,       &rxBuf[7], 4);
  memcpy(&newParams.M,       &rxBuf[11], 4); // [cite: 60]
  memcpy(&newParams.RestPos, &rxBuf[15], 4);
  memcpy(&newParams.WallPos, &rxBuf[19], 4);
  memcpy(&newParams.WallK,   &rxBuf[23], 4);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // [cite: 61]
    currentExperiment = newExp;
    motorEnabled = newEnabled;
    activeParams = newParams;
    if (newResetCnt != localResetCounter) { // [cite: 62]
      zeroCaptured = false;
      zeroCounter = 0; pos0_sum = 0;
      localResetCounter = newResetCnt; // [cite: 63]
    }
  } 
}

void apply_received_kinematics() {
  float rec_x, rec_dx;
  memcpy(&rec_x, &rxBuf[0], 4);
  memcpy(&rec_dx, &rxBuf[4], 4);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // [cite: 64]
    remote_x = (double)rec_x;
    remote_dx = (double)rec_dx;
    lastValidRxTimeMs = systemTimeMs; // ADDED: Feed the safety watchdog!
  }
}

void uart_tx_kinematics() {
  uint8_t txBuf[12]; 
  txBuf[0] = 0xBB;
  txBuf[1] = 0x55; // [cite: 65]
  txBuf[2] = 8;    

  float send_x = (float)x_prev;
  float send_dx = (float)dx_f;
  
  memcpy(&txBuf[3], &send_x, 4);
  memcpy(&txBuf[7], &send_dx, 4);

  uint8_t cks = txBuf[2]; // [cite: 66]
  for (int i = 3; i < 11; i++) cks ^= txBuf[i];
  txBuf[11] = cks;

  Serial.write(txBuf, 12); // [cite: 67]
}
