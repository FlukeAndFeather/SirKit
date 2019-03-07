#include <Arduino.h>
#include <Metro.h>

/*---------------Constants----------------------------------*/
// IR
#define IR_E1_PIN 16
#define IR_E2_PIN 21
#define IR_L_PIN 2
#define IR_M_PIN 3
#define IR_R_PIN 4
#define IR_A_PIN 5

// Driving
#define SPD_L_PIN 22 //Based on forward orientation.
#define DIR_L_PIN 14
#define SPD_R_PIN 23
#define DIR_R_PIN 15
#define MAX_SPD 120
#define ADJUST_FACTOR 0.65

// Timer durations (msec)
#define BACKING_T 750
#define PIVOTING_T 1800
#define DEPRESSING_T 500
#define RELOADING_T 3000
#define FIRING_T 5000
#define NOISE_T 2000  // usec

// Limit switch
#define BUMP_PIN 0

/*---------------Enumerations-------------------------------*/
typedef enum { 
  SENSOR_L, SENSOR_M, SENSOR_R, SENSOR_A
} Sensor_t;

typedef enum {
  STATE_DRIVING, STATE_BACKING1, STATE_PIVOTING1, STATE_DEPRESSING, 
  STATE_RELOADING, STATE_BACKING2, STATE_PIVOTING2, STATE_SEEKING, 
  STATE_FIRING, STATE_PIVOTING3, STATE_STOPPING
} States_t;

/*---------------Module Function Prototypes-----------------*/
// Read sensors
bool overTape(uint32_t now, Sensor_t sensor);
bool bump(void);
bool navExpire(void);

// Driving
void driveOnTape(void);
void driveStraight(void);
void rotate(bool turnLeft);
void setDir(bool dir);
void setDir(bool leftDir, bool rightDir);
void setSpd(uint16_t left, uint16_t right);

// Tape sensing
bool isTapeAligned(void);
bool isTapeLeft(void);
bool isTapeRight(void);
bool isTapeJunction(void);
bool isTapeAxle(void);
bool isTapeMiddle(void);

// Handle events
void handleDriving(void);
void handleBacking(void);
void handlePivoting(void);
void handleDepressing(void);
void handleReloading(void);
void handleSeeking(void);
void handleFiring(void);

// Utilities
void respondKeyboard(String key);

/*---------------Interrupt Handler Prototypes---------------*/
void toggleIr(void);
void irRisingL(void);
void irRisingM(void);
void irRisingR(void);
void irRisingA(void);

/*---------------Module Variables---------------------------*/
static States_t state;

// Driving
static Metro navTimer = Metro(0);

// IR emit
static uint16_t irFreq = 450; // 1kHz
static uint16_t irPeriod = 1000000 / irFreq; // in microseconds
static uint16_t irSwitch = irPeriod / 2;
static IntervalTimer irTimer;
volatile static bool irHigh;

// IR receive
volatile static uint32_t lastRiseL;
volatile static uint32_t lastRiseM;
volatile static uint32_t lastRiseR;
volatile static uint32_t lastRiseA;

// Print state timer
static Metro stateTimer = Metro(500);

/*---------------Teensy Main Functions----------------------*/
void setup() {
  // Connect to Serial
  Serial.begin(9600);
  while(!Serial);

  // IR pins
  pinMode(IR_E1_PIN, OUTPUT);
  pinMode(IR_E2_PIN, OUTPUT);
  pinMode(IR_L_PIN, INPUT);
  attachInterrupt(IR_L_PIN, irRisingL, RISING);
  pinMode(IR_M_PIN, INPUT);
  attachInterrupt(IR_M_PIN, irRisingM, RISING);
  pinMode(IR_R_PIN, INPUT);
  attachInterrupt(IR_R_PIN, irRisingR, RISING);
  pinMode(IR_A_PIN, INPUT);
  attachInterrupt(IR_A_PIN, irRisingA, RISING);
  irTimer.begin(toggleIr, irSwitch);
  irHigh = true;

  // SPD and DIR pins
  pinMode(SPD_L_PIN, OUTPUT);
  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(SPD_R_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);

  // Bump pin
  pinMode(BUMP_PIN, INPUT);

  // Start the motor
  //state = STATE_BACKING2;
  //handleBacking();
  state = STATE_DRIVING;
  handleDriving();
}

void loop() {
  if(stateTimer.check()) {
    //Serial.println(state);
  }

  switch (state) {
    case STATE_DRIVING:
      driveOnTape();
      if (bump()) {
        state = STATE_BACKING1;
        handleBacking();
      }
      break;
    case STATE_BACKING1:
      if (navExpire()) {
        state = STATE_PIVOTING1;
        handlePivoting();
      }
      break;
    case STATE_PIVOTING1:
      if (navExpire()) {
        state = STATE_DEPRESSING;
        handleDepressing();
      }
      break;
    case STATE_DEPRESSING:
      if (bump()) {
        state = STATE_RELOADING;
        handleReloading();
      }
      break;
    case STATE_RELOADING:
      if (navExpire()) {
        state = STATE_BACKING2;
        handleBacking();
      }
      break;
    case STATE_BACKING2:
      if (isTapeAxle()) {
        state = STATE_PIVOTING2;
        handlePivoting();
      }
      break;
    case STATE_PIVOTING2:
      if (isTapeMiddle()) {
        state = STATE_SEEKING;
        handleSeeking();
      }
      break;
    case STATE_SEEKING:
      driveOnTape();
      if (isTapeAxle()) {
        state = STATE_FIRING;
        handleFiring();
      }
      break;
    case STATE_FIRING:
      if (navExpire()) {
        state = STATE_PIVOTING3;
        handlePivoting();
      }
      break;
    case STATE_PIVOTING3:
      if (isTapeMiddle()) {
        state = STATE_DRIVING;
        handleDriving();
      }
    case STATE_STOPPING:
      break;
  }
}

/*---------------Module Functions---------------------------*/
// Read sensors
String readKeyboard(void) {
  String result = "";
  if (Serial.available()) {
    result = Serial.readString();
  }
  return result;
}

// Read an individual IR sensor
bool overTape(uint32_t now, Sensor_t sensor) {
  uint32_t lastRise = 0;
  switch (sensor) {
    case SENSOR_L:
      lastRise = lastRiseL;
      break;
    case SENSOR_R:
      lastRise = lastRiseR;
      break;
    case SENSOR_M:
      lastRise = lastRiseM;
      break;
    case SENSOR_A:
      lastRise = lastRiseA;
      break;
  }
  uint32_t since_rise = now - lastRise;
  if (since_rise > irPeriod) {
    return true;
  } else {
    return false;
  }
}

// Detect limit switch bump
bool bump(void) {
  bool notBumped = digitalRead(BUMP_PIN);
  return !notBumped;
}

// Navigation timer expires
bool navExpire(void) {
  return navTimer.check();
}

// Driving
void driveOnTape(void) {
  if (isTapeAligned()) {
    driveStraight();
  } else if (isTapeJunction()) {
    driveStraight();
  } else if (isTapeLeft()) {
    rotate(true);
  } else if (isTapeRight()) {
    rotate(false);
  } else {
    // This is just noise
  }
}

void driveStraight(void) {
  setDir(true);
  setSpd(MAX_SPD, MAX_SPD);
}

void rotate(bool turnLeft) {
  uint16_t adjustSpd = (int) (ADJUST_FACTOR * MAX_SPD);
  setSpd(adjustSpd, adjustSpd);

  bool left = turnLeft;
  bool right = !turnLeft;
  setDir(left, right);
}

void setDir(bool dir) {
  digitalWrite(DIR_L_PIN, !dir);
  digitalWrite(DIR_R_PIN, !dir);
}

void setDir(bool leftDir, bool rightDir) {
  digitalWrite(DIR_L_PIN, !leftDir);
  digitalWrite(DIR_R_PIN, !rightDir);
}

void setSpd(uint16_t left, uint16_t right) {
  analogWrite(SPD_L_PIN, left);
  analogWrite(SPD_R_PIN, right);
}

// Tape sensing
// Tape aligned when sensors straddle it
bool isTapeAligned(void) {
  uint32_t now = micros();
  Sensor_t l_sensor = state == STATE_DRIVING ? SENSOR_L : SENSOR_M;
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  bool L = overTape(now, l_sensor);
  bool R = overTape(now, r_sensor);
  return !L && !R;
}

bool isTapeLeft(void) {
  uint32_t now = micros();
  Sensor_t l_sensor = state == STATE_DRIVING ? SENSOR_L : SENSOR_M;
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  Sensor_t t_sensor = state == STATE_DRIVING ? SENSOR_R : SENSOR_L;
  bool L = overTape(now, l_sensor);
  bool R = overTape(now, r_sensor);
  bool T = overTape(now, t_sensor);
  if (T) return false;
  if (L && !R) return true;
  return false;
}

bool isTapeRight(void) {
  uint32_t now = micros();
  Sensor_t l_sensor = state == STATE_DRIVING ? SENSOR_L : SENSOR_M;
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  Sensor_t t_sensor = state == STATE_DRIVING ? SENSOR_R : SENSOR_L;
  bool L = overTape(now, l_sensor);
  bool R = overTape(now, r_sensor);
  bool T = overTape(now, t_sensor);
  if (T) return false;
  if (!L && R) return true;
  return false;
}

bool isTapeJunction(void) {
  uint32_t now = micros();
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  Sensor_t t_sensor = state == STATE_DRIVING ? SENSOR_R : SENSOR_L;
  bool R = overTape(now, r_sensor);
  bool T = overTape(now, t_sensor);
  return R && T;
}

// Only counts if we get five trips in under NOISE_T usec
bool isTapeAxle(void) {
  uint32_t now = micros();
  bool tapeAxle = overTape(now, SENSOR_A);
  if (!tapeAxle) return false;
  int tapeCounter = 1;
  uint32_t now2 = micros();
  while (now2 - now < NOISE_T) {
    if (overTape(now2, SENSOR_A)) tapeCounter++;
    if (tapeCounter > 10) return true;
    now2 = micros();
  }
  return false;
}

bool isTapeMiddle(void) {
  uint32_t now = micros();
  bool tapeMiddle = overTape(now, SENSOR_M);
  if (!tapeMiddle) return false;
  int tapeCounter = 1;
  uint32_t now2 = micros();
  while (now2 - now < NOISE_T) {
    if (overTape(now2, SENSOR_M)) tapeCounter++;
    if (tapeCounter > 10) return true;
    now2 = micros();
  }
  return false;
}

// Handle events
void handleDriving(void) {
  setSpd(MAX_SPD, MAX_SPD);
  setDir(true);
}

void handleBacking(void) {
  navTimer.interval(BACKING_T);
  navTimer.reset();
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Back up
  setDir(false);
  setSpd(MAX_SPD, MAX_SPD);
}

void handlePivoting(void) {
  navTimer.interval(PIVOTING_T);
  navTimer.reset();
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Rotate right
  rotate(false);
}

void handleDepressing(void) {
  navTimer.interval(DEPRESSING_T);
  navTimer.reset();
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Drive forward
  driveStraight();
}

void handleReloading(void) {
  setSpd(0, 0);
  navTimer.interval(RELOADING_T);
  navTimer.reset();
}

void handleSeeking(void) {
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Go forwards
  setSpd(MAX_SPD, MAX_SPD);
  setDir(true);
}

void handleFiring(void) {
  Serial.println("FIRING");
  setSpd(0, 0);
  // !!!FIXME!!!
  //navTimer.interval(FIRING_T);
  //navTimer.reset();
}

/*---------------Interrupt Handler Prototypes---------------*/
void toggleIr(void) {
  digitalWrite(IR_E1_PIN, irHigh ? HIGH : LOW);
  digitalWrite(IR_E2_PIN, irHigh ? HIGH : LOW);
  irHigh = !irHigh;
}

void irRisingL(void) {
  lastRiseL = micros();
}

void irRisingM(void) {
  lastRiseM = micros();
}

void irRisingR(void) {
  lastRiseR = micros();
}

void irRisingA(void) {
  lastRiseA = micros();
}
