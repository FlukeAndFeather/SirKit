#include <Arduino.h>

/*---------------Constants----------------------------------*/
// IR
#define IR_E_PIN1 16
#define IR_E_PIN2 21
#define IR_L_PIN 5
#define IR_R_PIN 2
#define IR_T_PIN 4

// Driving
#define SPD_PIN_RIGHT 22 //Based on forward orientation.
#define DIR_PIN_RIGHT 14
#define SPD_PIN_LEFT 23
#define DIR_PIN_LEFT 15
#define MAX_SPD 110
#define ADJUST_FACTOR 0.65

/*---------------Enumerations-------------------------------*/
typedef enum { 
  SENSOR_L, SENSOR_R, SENSOR_T
} Sensor_t;

typedef enum {
  STATE_DRIVING, STATE_BACKING_LEFT, STATE_DEPRESSING,
  STATE_RELOADING, STATE_BACKING_RIGHT, STATE_REVERSING,
  STATE_FIRING, STATE_STOPPING, STATE_REVERSE
} States_t;

/*---------------Module Function Prototypes-----------------*/
// Read sensors
String readKeyboard(void);
bool timerExpired(void);
bool overTape(uint32_t now, Sensor_t sensor);

// Driving
void driveOnTape(void);
void driveStraight(void);
void adjustRight(void);
void adjustLeft(void);
void rotate(bool turnLeft);
void setDir(bool dir);
void setDir(bool leftDir, bool rightDir);
void setSpd(uint16_t left, uint16_t right);

// Tape sensing
uint8_t readTape(void);
bool isTapeAligned(uint32_t now);
bool isTapeLeft(uint32_t now);
bool isTapeRight(uint32_t now);

// Handle events
void handleReverse(void);
void handleForward(void);
void handleStopping(void);

// Utilities
void respondKeyboard(String key);

/*---------------Interrupt Handlers Prototypes--------------*/
void toggleIr(void);
void irRisingL(void);
void irRisingR(void);
void irRisingT(void);

/*---------------Module Variables---------------------------*/
static States_t state;

// Driving
static bool dirBwd = false;
static uint16_t spd = MAX_SPD; // From 0 to 1023
static IntervalTimer navTimer;

// IR emit
static uint16_t irFreq = 450; // 1kHz
static uint16_t irPeriod = 1000000 / irFreq; // in microseconds
static uint16_t irSwitch = irPeriod / 2;
static IntervalTimer irTimer;
volatile static bool irHigh;

// IR receive
volatile static uint32_t lastRiseL;
volatile static uint32_t lastRiseR;
volatile static uint32_t lastRiseT;

/*---------------Teensy Main Functions----------------------*/
void setup() {
  // Connect to Serial
  Serial.begin(9600);
  while(!Serial);

  // IR pins
  pinMode(IR_E_PIN1, OUTPUT);
  pinMode(IR_E_PIN2, OUTPUT);
  pinMode(IR_L_PIN, INPUT);
  attachInterrupt(IR_L_PIN, irRisingL, RISING);
  pinMode(IR_R_PIN, INPUT);
  attachInterrupt(IR_R_PIN, irRisingR, RISING);
  pinMode(IR_T_PIN, INPUT);
  attachInterrupt(IR_T_PIN, irRisingT, RISING);
  irTimer.begin(toggleIr, irSwitch);
  irHigh = true;

  // SPD and DIR pins
  pinMode(SPD_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(SPD_PIN_RIGHT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);

  // Start the motor
  state = STATE_DRIVING;
  setDir(dirBwd);
  setSpd(spd, spd);
}

void loop() {
  // Read keyboard
  String key = readKeyboard();

  switch (state) {
    case STATE_DRIVING:
      driveOnTape();
      break;
    case STATE_REVERSE:
      driveOnTape();
      break;
    case STATE_STOPPING:
      break;
  }
}

/*----------------Module Functions--------------------------*/
void driveOnTape(void) {
  uint32_t now = micros();
  if (isTapeAligned(now)) {
    //Serial.println("Aligned");
    driveStraight();
  } else if (isTapeLeft(now)) {
    //Serial.println("Tape left");
    adjustLeft();
  } else if (isTapeRight(now)) {
    //Serial.println("Tape right");
    adjustRight();
  } else {
    // This shouldn't happen
    //Serial.println("Both tape");
    adjustLeft();
  }
}

void driveStraight(void) {
  setDir(dirBwd);
  setSpd(MAX_SPD, MAX_SPD);
}

void adjustLeft(void) {
  rotate(true);
}

void adjustRight(void) {
  rotate(false);
}

// Tape aligned when it's between L and R
bool isTapeAligned(uint32_t now) {
  bool L = overTape(now, SENSOR_L);
  bool R = overTape(now, SENSOR_R);
  return !L && !R;
}
bool isTapeLeft(uint32_t now) {
  bool L = overTape(now, SENSOR_L);
  bool R = overTape(now, SENSOR_R);
  return L && !R;
}
bool isTapeRight(uint32_t now) {
  bool L = overTape(now, SENSOR_L);
  bool R = overTape(now, SENSOR_R);
  //bool T = overTape(now, SENSOR_T);
  return !L && R; // && !T [FIXME];
}

// Read an individual sensor
bool overTape(uint32_t now, Sensor_t sensor) {
  uint32_t lastRise = 0;
  switch (sensor) {
    case SENSOR_L:
      lastRise = lastRiseL;
      break;
    case SENSOR_R:
      lastRise = lastRiseR;
      break;
    case SENSOR_T:
      lastRise = lastRiseT;
      break;
  }
  uint32_t since_rise = now - lastRise;
  if (since_rise > irPeriod) {
    return true;
  } else {
    return false;
  }
}

/*---------------Interrupt Handlers-------------------------*/
void toggleIr(void) {
  digitalWrite(IR_E_PIN1, irHigh ? HIGH : LOW);
  digitalWrite(IR_E_PIN2, irHigh ? HIGH : LOW);
  irHigh = !irHigh;
}

void irRisingL(void) {
  lastRiseL = micros();
}
void irRisingR(void) {
  lastRiseR = micros();
}
void irRisingT(void) {
  lastRiseT = micros();
}

void setDir(bool dir) {
  digitalWrite(DIR_PIN_LEFT, dir);
  digitalWrite(DIR_PIN_RIGHT, dir);
}
void setDir(bool leftDir, bool rightDir) {
  digitalWrite(DIR_PIN_LEFT, leftDir);
  digitalWrite(DIR_PIN_RIGHT, rightDir);
}

void rotate(bool turnLeft) {
  uint16_t adjustSpd = ADJUST_FACTOR * spd;
  setSpd(adjustSpd, adjustSpd);

  bool leftBwd = turnLeft;
  bool rightBwd = !turnLeft;
  setDir(leftBwd, rightBwd);
}

void setSpd(uint16_t left, uint16_t right) {
  analogWrite(SPD_PIN_LEFT, left);
  analogWrite(SPD_PIN_RIGHT, right);
}

void handleReverse(void) {
  // Come to a stop for 0.25s
  setSpd(0, 0);
  delay(250);

  // Go backwards
  dirBwd = false;
  setDir(dirBwd);
  setSpd(spd, spd);
}

void handleForward(void) {
  // Come to a stop for 0.25s
  setSpd(0, 0);
  delay(250);

  // Go forwards
  dirBwd = true;
  setDir(dirBwd);
  setSpd(spd, spd);
}

void handleStopping(void) {
  setSpd(0, 0);
}

String readKeyboard(void) {
  String result = "";
  if (Serial.available()) {
    result = Serial.readString();
  }
  return result;
}

void respondKeyboard(String key) {
  if (key == " ") {
    state = STATE_REVERSE;
    handleReverse();
  } else if (key == "s") {
    state = STATE_STOPPING;
    handleStopping();
  }
}
