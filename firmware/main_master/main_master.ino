#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <util/atomic.h>

// =========================================================================
// 1. HARDWARE DEFINITIONS & CONSTANTS
// =========================================================================

// --- OLED Display Pins ---
#define OLED_DC     7
#define OLED_CS     10
#define OLED_RESET  9
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

// --- User Interface Pins ---
#define LED_EXP1    3
#define LED_EXP2    4
#define LED_EXP3    6
#define BTN_PARAM   2
#define BTN_EXP     A4  
#define BTN_RESET   A1
#define BTN_ENABLE  A5
#define POT_PIN     A0

// --- Motor & Sensor Pins ---
#define PWM_PIN     5
#define DIR_PIN     8
#define SENSOR_PIN  A2

// --- Kinematic & Physical Constants ---
const double rh = 0.090;             // Handle radius (m)
const double rs = 0.075;             // Sector radius (m)
const double rp = 0.010;             // Pinion radius (m)
const double TORQUE_CONST = 0.0183;  // Motor torque constant (Nm/A)
const double DEG_PER_COUNT = 0.013892; // Sensor resolution
const long OFFSET = 902;             // Sensor magnetic wrap offset

// =========================================================================
// 2. GLOBAL SYSTEM STATE
// =========================================================================

// --- Shared Haptic Parameters ---
struct HapticParams {
  float K; 
  float B; 
  float M;
  float RestPos; 
  float WallPos; 
  float WallK;
  
  void operator=(const volatile HapticParams& other) {
    K = other.K; 
    B = other.B; 
    M = other.M;
    RestPos = other.RestPos; 
    WallPos = other.WallPos; 
    WallK = other.WallK;
  }
};

volatile HapticParams activeParams = {100.0, 2.0, 0.0, 0.00, 0.02, 0.0};
volatile int currentExperiment = 1;      
volatile bool motorEnabled = false;
volatile unsigned long systemTimeMs = 0;

// --- Haptic Physics Variables ---
int rawPos = 0;
int lastRawPos = 0;
int lastLastRawPos = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
bool flipped = false;
long flipNumber = 0;

// --- Calibration Variables ---
bool zeroCaptured = false;
int zeroCounter = 0;
long pos0_sum = 0;
long pos0 = 0;
const int ZERO_SETTLE_MS = 100;
const int ZERO_AVG_MS = 200;

// --- Kinematic States ---
double x_prev = 0.0;
double dx_f = 0.0;
volatile double remote_x = 0.0;
volatile double remote_dx = 0.0;

// --- Safety Watchdogs ---
volatile unsigned long lastValidRxTimeMs = 0;
volatile uint8_t systemError = 0; // 0 = OK, 1 = COMMS LOST, 2 = INSTABILITY

// --- UART Communication Variables ---
int rxState = 0; 
uint8_t rxLen = 0;
uint8_t rxCount = 0; 
uint8_t rxBuf[12]; 
uint8_t rxCks = 0;

// --- User Interface State ---
int currentParameter = 0;
unsigned long lastUiTick = 0;
unsigned long lastOledTick = 0;
bool uiNeedsUpdate = true;
bool configNeedsSending = true;

bool lastBtnExp = HIGH;
bool lastBtnParam = HIGH;
bool lastBtnEnable = HIGH;
bool lastBtnReset = HIGH;

int lastDisplayedValue = -1;
float smoothedPotValue = 0.0;
volatile uint8_t resetCounter = 0;

// --- Menu Interaction Variables ---
bool isEditingParam = false;
unsigned long lastBlinkTick = 0;
bool blinkState = true;

// --- Exp 3: Time Delay Ring Buffer ---
#define MAX_DELAY_MS 20 
volatile float delayBufX[MAX_DELAY_MS];
volatile float delayBufDX[MAX_DELAY_MS];
volatile int bufWriteIdx = 0;

// --- Hardware Objects ---
bool displayActive = false; // Memory safety shield
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// =========================================================================
// 3. HARDWARE INITIALIZATION
// =========================================================================

void setupPWM31kHz() {
  TCCR0A = (TCCR0A & 0b11111100) | 0x01;
  TCCR0B = (TCCR0B & 0b11111000) | 0x01; 
}

void setupTimer1Timebase() {
  TCCR1A = 0; 
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A = 1999;
  TCNT1 = 0;                           
  TIFR1 |= (1 << OCF1A);
}

void setup() {
  Serial.begin(500000); 

  pinMode(LED_EXP1, OUTPUT); 
  pinMode(LED_EXP2, OUTPUT); 
  pinMode(LED_EXP3, OUTPUT);
  
  pinMode(BTN_PARAM, INPUT_PULLUP); 
  pinMode(BTN_EXP, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP); 
  pinMode(BTN_ENABLE, INPUT_PULLUP);
  
  pinMode(SENSOR_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  analogWrite(PWM_PIN, 0);
  digitalWrite(DIR_PIN, LOW);

  // Safely Boot OLED before configuring hardware timers
  if(display.begin(SSD1306_SWITCHCAPVCC)) {
    displayActive = true;
    display.setTextColor(SSD1306_WHITE);
  } else {
    displayActive = false; // Graceful degradation if memory fails
  }
  
  setupPWM31kHz();
  setupTimer1Timebase(); 
  updateLEDs();
}

// =========================================================================
// 4. MAIN BACKGROUND LOOP (Non-Real-Time Tasks)
// =========================================================================

void loop() {
  // 1. Service incoming packets as fast as possible
  uart_rx_process();

  // 2. Hardware Timer Metronome (Strict 1kHz execution)
  if (TIFR1 & (1 << OCF1A)) {
    TIFR1 |= (1 << OCF1A);
    systemTimeMs++;
    run_1khz_haptic_loop();
  }

  // 3. User Input Polling (100 Hz)
  if (systemTimeMs - lastUiTick >= 10) {
    lastUiTick = systemTimeMs;
    read_buttons_debounce();
    read_potentiometer();
  }

  // 4. Screen Redraw (25 Hz)
  if (uiNeedsUpdate && displayActive && (systemTimeMs - lastOledTick >= 40)) {
    lastOledTick = systemTimeMs;
    drawScreen();
    uiNeedsUpdate = false;
  }

  // 5. Flashing UI Timer (3.3 Hz)
  if (systemTimeMs - lastBlinkTick >= 300) {
    lastBlinkTick = systemTimeMs;
    blinkState = !blinkState;
    
    if (isEditingParam || systemError != 0) uiNeedsUpdate = true;

    // Siren effect if instability is detected
    if (systemError != 0) {
      digitalWrite(LED_EXP1, blinkState ? HIGH : LOW);
      digitalWrite(LED_EXP2, blinkState ? HIGH : LOW);
      digitalWrite(LED_EXP3, blinkState ? HIGH : LOW);
    } else {
      updateLEDs(); 
    }
  }

  // 6. Push Configuration to Slave if changed
  if (configNeedsSending) {
    uart_tx_state();
    configNeedsSending = false;
  }
}

// =========================================================================
// 5. HARD-REAL-TIME LOOP (1 kHz Haptics)
// =========================================================================

void run_1khz_haptic_loop() {
  unsigned int startTicks = TCNT1;

  // --- Step 1: Read Sensor & Handle Magnetic Wraparound ---
  rawPos = analogRead(SENSOR_PIN);
  rawDiff = rawPos - lastRawPos; 
  lastRawDiff = rawPos - lastLastRawPos;
  rawOffset = abs(rawDiff); 
  lastRawOffset = abs(lastRawDiff);
  
  lastLastRawPos = lastRawPos; 
  lastRawPos = rawPos;

  if ((lastRawOffset > 700) && (!flipped)) {
    if (lastRawDiff > 0) flipNumber--;
    else flipNumber++;
    flipped = true;
  } else {
    flipped = false;
  }
  
  long updatedPos = (long)rawPos + flipNumber * OFFSET;

  // --- Step 2: Auto-Calibration (Wait for settling) ---
  if (!zeroCaptured) {
    zeroCounter++;
    if (zeroCounter > ZERO_SETTLE_MS && zeroCounter <= (ZERO_SETTLE_MS + ZERO_AVG_MS)) {
      pos0_sum += updatedPos;
    } 
    else if (zeroCounter > (ZERO_SETTLE_MS + ZERO_AVG_MS)) {
      pos0 = pos0_sum / ZERO_AVG_MS;
      x_prev = 0.0; 
      dx_f = 0.0; 
      zeroCaptured = true;
    }
    analogWrite(PWM_PIN, 0); 
    return;
  }

  // --- Step 3: Forward Kinematics ---
  long posRel = updatedPos - pos0;
  double theta_rad = ((double)posRel * DEG_PER_COUNT) * (PI / 180.0);
  double x_m = rh * theta_rad;
  
  double dx_m = (x_m - x_prev) / 0.001; 
  x_prev = x_m;
  dx_f = 0.92 * dx_f + (1.0 - 0.92) * dx_m; // Low-pass filter

  // --- Step 4: Safely load variables for physics calculations ---
  HapticParams g; 
  int localExp; 
  bool localEnabled;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    g = activeParams; 
    localExp = currentExperiment; 
    localEnabled = motorEnabled;
  }

  double forceN = 0.0;

  // Always record current remote data into the ring buffer for Exp 3
  delayBufX[bufWriteIdx] = remote_x;
  delayBufDX[bufWriteIdx] = remote_dx;

  // --- Step 5: System Health Diagnosis (FDI) ---
  if (abs(dx_f) > 5.0) {
    systemError = 2;       // INSTABILITY: Velocity is impossible for a human
    motorEnabled = false;  // LATCH OFF for physical safety
  } 
  else if (systemError != 2) {
    if ((localExp == 2 || localExp == 3) && (systemTimeMs - lastValidRxTimeMs > 250)) {
      systemError = 1;     // COMMS LOST: Ghost Slave prevention
    } else {
      systemError = 0;     // SYSTEM HEALTHY
    }
  }

  // --- Step 6: Teleoperation Physics Engine ---
  if (localEnabled && systemError == 0) {
    
    // Virtual Environment (MDS + Wall)
    if (localExp == 1) {
       forceN = -(g.K * (x_m - g.RestPos)) - (g.B * dx_f);
       if (g.WallK > 0.0 && x_m > g.WallPos) {
         forceN += -g.WallK * (x_m - g.WallPos);
       }
    }
    
    // Instant Bilateral Teleoperation
    else if (localExp == 2) {
       forceN = -g.K * (x_m - remote_x) - g.B * (dx_f - remote_dx);
    }
    
    // Delayed Bilateral Teleoperation
    else if (localExp == 3) {
       int delayMs = (int)g.RestPos;
       delayMs = constrain(delayMs, 0, MAX_DELAY_MS - 1);
       
       int bufReadIdx = bufWriteIdx - delayMs;
       if (bufReadIdx < 0) bufReadIdx += MAX_DELAY_MS;
       
       forceN = -g.K * (x_m - delayBufX[bufReadIdx]) - g.B * (dx_f - delayBufDX[bufReadIdx]);
    }
  }

  // Iterate Ring Buffer
  bufWriteIdx++;
  if (bufWriteIdx >= MAX_DELAY_MS) bufWriteIdx = 0;

  // --- Step 7: Safety Clamps & Hardware Failsafes ---
  forceN = constrain(forceN, -3.0, 3.0);
  if (abs(forceN) < 0.08) forceN = 0.0;  // Deadband to stop micro-jitter
  if (systemError != 0) forceN = 0.0;    // Absolute override if watchdog triggered

  // --- Step 8: Motor Output Translation ---
  double Tp = forceN * rh * rp / rs;
  digitalWrite(DIR_PIN, (Tp >= 0) ? LOW : HIGH); 
  
  double duty = sqrt(fabs(Tp) / TORQUE_CONST);
  if (duty > 1.0) duty = 1.0;
  
  analogWrite(PWM_PIN, (int)(duty * 255.0));

  // Transmit kinematics only if operating in a bilateral mode
  if (localExp == 2 || localExp == 3) {
    uart_tx_kinematics();
  }
}

// =========================================================================
// 6. USER INTERFACE LOGIC
// =========================================================================

void read_buttons_debounce() {
    bool currentBtnExp = digitalRead(BTN_EXP);
    bool currentBtnParam = digitalRead(BTN_PARAM);
    bool currentBtnEnable = digitalRead(BTN_ENABLE);
    bool currentBtnReset = digitalRead(BTN_RESET);

    // Switch Experiments
    if (currentBtnExp == LOW && lastBtnExp == HIGH) {
      currentExperiment++;
      if (currentExperiment > 3) currentExperiment = 1;
      currentParameter = 0; 
      isEditingParam = false; // Kick out of edit mode if experiment changes
      updateLEDs();
      uiNeedsUpdate = true; 
      configNeedsSending = true;
    }
    
    // Toggle Edit Mode (Select Button)
    else if (currentBtnParam == LOW && lastBtnParam == HIGH) {
      isEditingParam = !isEditingParam; 
      uiNeedsUpdate = true;
    }
    
    // Enable Motor / Reset Errors
    static unsigned long lastEnableTapTime = 0;
    if (currentBtnEnable == LOW && lastBtnEnable == HIGH) {
      if (systemTimeMs - lastEnableTapTime > 250) { 
        motorEnabled = !motorEnabled;
        if (systemError == 2) systemError = 0; // Clear the latched instability error
        uiNeedsUpdate = true; 
        configNeedsSending = true;
        lastEnableTapTime = systemTimeMs;
      }
    }
    lastBtnEnable = currentBtnEnable;

    // Reset System Origin
    if (currentBtnReset == LOW && lastBtnReset == HIGH) {
      zeroCaptured = false;
      zeroCounter = 0; 
      pos0_sum = 0; 
      resetCounter++; 
      configNeedsSending = true;
    }

    lastBtnExp = currentBtnExp; 
    lastBtnParam = currentBtnParam;
    lastBtnReset = currentBtnReset;
} 

void read_potentiometer() {
  int rawValue = analogRead(POT_PIN);
  smoothedPotValue = (0.1 * rawValue) + (0.9 * smoothedPotValue); 
  int newPotValue = (int)smoothedPotValue;
  static int lastPotValue = -1;

  if (abs(newPotValue - lastPotValue) > 2) { 
    lastPotValue = newPotValue;

    if (!isEditingParam) {
      // --- Menu Scrolling Mode ---
      int maxParams = 0;
      if (currentExperiment == 1) maxParams = 5;
      else if (currentExperiment == 3) maxParams = 3;
      else maxParams = 2; // Exp 2 only exposes K, B, M

      currentParameter = (newPotValue * (maxParams + 1)) / 1024;
      currentParameter = constrain(currentParameter, 0, maxParams);
      uiNeedsUpdate = true;
    } 
    else {
      // --- Parameter Editing Mode ---
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (currentParameter == 0)      activeParams.K = map(newPotValue, 0, 1023, 0, 200);
        else if (currentParameter == 1) activeParams.B = map(newPotValue, 0, 1023, 0, 100) / 10.0;
        else if (currentParameter == 2) activeParams.M = map(newPotValue, 0, 1023, 0, 10) / 10.0;
        else if (currentParameter == 3) {
          if (currentExperiment == 3) activeParams.RestPos = map(newPotValue, 0, 1023, 0, MAX_DELAY_MS);
          else activeParams.RestPos = map(newPotValue, 0, 1023, -50, 50) / 1000.0;
        }
        else if (currentParameter == 4) activeParams.WallPos = map(newPotValue, 0, 1023, -50, 50) / 1000.0;
        else if (currentParameter == 5) activeParams.WallK = map(newPotValue, 0, 1023, 0, 600);
      }
      uiNeedsUpdate = true; 
      configNeedsSending = true;
    }
  }
}

void updateLEDs() {
  digitalWrite(LED_EXP1, currentExperiment == 1 ? HIGH : LOW);
  digitalWrite(LED_EXP2, currentExperiment == 2 ? HIGH : LOW);
  digitalWrite(LED_EXP3, currentExperiment == 3 ? HIGH : LOW);
}

// =========================================================================
// 7. MEMORY-OPTIMIZED OLED DRAWING
// =========================================================================

void drawScreen() {
  display.clearDisplay();

  // --- Header Status Bar ---
  display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
  display.setCursor(2, 3); 
  display.setTextSize(1);
  
  display.print(F("EXP "));
  display.print(currentExperiment);
  display.print(F(": "));
  
  if (currentExperiment == 1) display.print(F("MSD+Wall"));
  else if (currentExperiment == 2) display.print(F("Teleop"));
  else display.print(F("Delayed"));

  // Motor Status (Top Right)
  display.setCursor(104, 3);
  if (motorEnabled) display.print(F("[ON]"));
  else display.print(F(" OFF"));

  // --- Highlight & Draw Logic for Parameters ---
  auto drawParam = [](int x, int y, const __FlashStringHelper* label, float val, bool isSelected) {
    display.setCursor(x, y);
    
    if (isSelected) {
      if (isEditingParam && !blinkState) {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // Flash effect off
      } else {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Highlight block on
      }
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);   // Normal text
    }
    
    display.print(label); 
    display.print(val, 1);
    
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // Safety reset
  };

  HapticParams displayGains;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { displayGains = activeParams; }

  // Draw Physics Parameters
  drawParam(2, 20, F(" K:"), displayGains.K, currentParameter == 0);
  drawParam(2, 31, F(" B:"), displayGains.B, currentParameter == 1);
  drawParam(2, 42, F(" M:"), displayGains.M, currentParameter == 2);

  if (currentExperiment == 1) {
    drawParam(64, 20, F(" RP:"), displayGains.RestPos, currentParameter == 3);
    drawParam(64, 31, F(" WP:"), displayGains.WallPos, currentParameter == 4);
    drawParam(64, 42, F(" WK:"), displayGains.WallK,   currentParameter == 5);
  } else if (currentExperiment == 3) {
    drawParam(64, 20, F(" Del:"), displayGains.RestPos, currentParameter == 3);
  }

  // --- Dynamic Error Banner (Bottom) ---
  uint8_t currentError;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { currentError = systemError; }

  if (currentError != 0 && blinkState) {
    display.fillRect(0, 53, 128, 11, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); 
    
    if (currentError == 1) {
      display.setCursor(22, 55);
      display.print(F("! COMMS LOST !"));
    }
    else if (currentError == 2) {
      display.setCursor(19, 55);
      display.print(F("! INSTABILITY !"));
    }
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }

  display.display();
}

// =========================================================================
// 8. UART COMMUNICATION (State Machine & Data Exchange)
// =========================================================================

void uart_rx_process() {
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    
    // Packet Structure: Header (BB, 55) -> Length (8) -> Payload -> Checksum
    switch (rxState) {
      case 0: if (b == 0xBB) rxState = 1; break;
      case 1: if (b == 0x55) rxState = 2; else rxState = 0; break;
      case 2: 
        rxLen = b;
        if (rxLen == 8) { 
          rxCount = 0; 
          rxCks = b; 
          rxState = 3;
        } else {
          rxState = 0; 
        }
        break;
      case 3: 
        rxBuf[rxCount++] = b; 
        rxCks ^= b;
        if (rxCount >= rxLen) rxState = 4; 
        break;
      case 4: 
        if (b == rxCks) {
          float rec_x, rec_dx;
          memcpy(&rec_x, &rxBuf[0], 4);
          memcpy(&rec_dx, &rxBuf[4], 4);
          
          ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            remote_x = (double)rec_x;
            remote_dx = (double)rec_dx;
            lastValidRxTimeMs = systemTimeMs; // Feed the safety watchdog
          }
        }
        rxState = 0;
        break;
    }
  }
}

void uart_tx_kinematics() {
  uint8_t txBuf[12]; 
  txBuf[0] = 0xBB; 
  txBuf[1] = 0x55; 
  txBuf[2] = 8;
  
  float send_x = (float)x_prev; 
  float send_dx = (float)dx_f;
  
  memcpy(&txBuf[3], &send_x, 4); 
  memcpy(&txBuf[7], &send_dx, 4);

  uint8_t cks = txBuf[2];
  for (int i = 3; i < 11; i++) cks ^= txBuf[i];
  txBuf[11] = cks;

  Serial.write(txBuf, 12);
}

void uart_tx_state() {
  uint8_t txBuf[31]; 
  txBuf[0] = 0xAA; 
  txBuf[1] = 0x55; 
  txBuf[2] = 27;   
  
  uint8_t exp, enabled, resetCnt;
  HapticParams p;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    exp = (uint8_t)currentExperiment; 
    enabled = motorEnabled ? 1 : 0;
    resetCnt = resetCounter;
    p = activeParams;
  }

  txBuf[3] = exp; 
  txBuf[4] = enabled; 
  txBuf[5] = resetCnt;
  
  memcpy(&txBuf[6],  &p.K, 4);
  memcpy(&txBuf[10], &p.B, 4); 
  memcpy(&txBuf[14], &p.M, 4);
  memcpy(&txBuf[18], &p.RestPos, 4); 
  memcpy(&txBuf[22], &p.WallPos, 4); 
  memcpy(&txBuf[26], &p.WallK, 4);

  uint8_t cks = txBuf[2];
  for (int i = 3; i < 30; i++) cks ^= txBuf[i];
  txBuf[30] = cks;

  Serial.write(txBuf, 31);
}
