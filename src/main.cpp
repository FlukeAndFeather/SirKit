#include <Arduino.h>
#include <Metro.h>

/*---------------Constants----------------------------------*/
// Writing to an analog pin repeatedly causes a race condition 
// and the last one gets dropped. Waiting 10 msec prevents that
// without compromising performance.
#define ANALOG_DELAY 10

// IR
// Two Teensy pins control IR emission for all four IR sensors
#define IR_E1_PIN 16
#define IR_E2_PIN 21
// Four Teensy pins read the IR sensors: left, middle, right
// and axle
#define IR_L_PIN 2
#define IR_M_PIN 3
#define IR_R_PIN 4
#define IR_A_PIN 5

// Driving
// The two drive motors are controlled by a speed and direction
// pin each
#define SPD_L_PIN 22 //Based on forward orientation.
#define DIR_L_PIN 14
#define SPD_R_PIN 23
#define DIR_R_PIN 15
// Drive motor speed is mapped from 0-255, which we cap at 175
#define MAX_SPD 175
// When rotating, we slow the motors down by a factor of 0.65
#define ADJUST_FACTOR 0.65
// ROT_LEFT and ROT_RIGHT make the `rotate` function more readable
#define ROT_LEFT true
#define ROT_RIGHT false

// Firing and loading
// The launch system has two motors: one for the flywheel and 
// another to load wildfire into the flywheel. They have separate
// speed pins but share a direction pin, which is always set to
// forward
#define FIRE_SPD_PIN 9
#define FIRE_DIR_PIN 12
#define LOAD_SPD_PIN 10
// Launch system duty cycles range 0-255, which 0 being full duty
// cycle and 255 being 0% (i.e. PWM_OFF). Firing and loading motors 
// run at separate duty cycles.
#define FIRE_DUTY 80
#define LOAD_DUTY 50
#define PWM_OFF 255

// Timer durations (msec)
// These timers correspond to the time spent in different states. All
// are in msec.
#define BACKING_T 500
#define PIVOTING_T 1500
#define DEPRESSING_T 1000
#define RELOADING_T 3000
#define AIMING_T 1500
#define FIRING_T 10000
// The delay between starting the flywheel and the loader to allow 
// the flywheel to reach full speed. In msec.
#define FIRE_DELAY 1500
// The IR sensors can be noisy. To confirm we're over the tape in a 
// certain position, the sensor must be triggered 10 times in less
// than NOISE_T (in usec).
#define NOISE_T 2000

// Limit switch
#define BUMP_PIN 0

/*---------------Enumerations-------------------------------*/
// For IR sensors corresponding to left, middle, right, and axle
typedef enum { 
  SENSOR_L, SENSOR_M, SENSOR_R, SENSOR_A
} Sensor_t;

// See finite state machine diagram for details
typedef enum {
  STATE_DRIVING, STATE_BACKING1, STATE_PIVOTING1, STATE_DEPRESSING, 
  STATE_RELOADING, STATE_BACKING2, STATE_PIVOTING2, STATE_SEEKING,
  STATE_AIMING, STATE_FIRING, STATE_LOADING, STATE_STOPPING
} States_t;

/*---------------Module Function Prototypes-----------------*/
// Read sensors
bool overTape(uint32_t now, Sensor_t sensor);
bool bump(void);
bool navExpire(void);
bool fireExpire(void);

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

// Firing
void startFiring(void);
void startLoading(void);
void stopFiring(void);

// Handle events
void handleDriving(void);
void handleBacking(void);
void handlePivoting(void);
void handleDepressing(void);
void handleReloading(void);
void handleSeeking(void);
void handleAiming(void);
void handleFiring(void);
void handleLoading(void);
void handleStopping(void);

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
// The navTimer is used to time navigation states (e.g. backing
// or pivoting).
static Metro navTimer = Metro(0);

// IR emit
// IR sensors emit at 450 Hz
static uint16_t irFreq = 450; 
static uint16_t irPeriod = 1000000 / irFreq; // in usec
static uint16_t irSwitch = irPeriod / 2;
// irTimer regulates the IR frequency
static IntervalTimer irTimer;
volatile static bool irHigh;

// IR receive
// The variables track the most recent rising edge for each
// IR sensor. Rising edges mean the sensor is over white so if 
// the last rising edge was less than one IR period ago then 
// the sensor isn't over tape.
volatile static uint32_t lastRiseL;
volatile static uint32_t lastRiseM;
volatile static uint32_t lastRiseR;
volatile static uint32_t lastRiseA;

// Firing
// This timer creates a delay between starting the flywheel and 
// the loader, so the flywheel can get up to speed.
static Metro fireTimer = Metro(0);

// Print state timer
// Log the state to Serial at 2Hz
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

  // Driving pins
  pinMode(SPD_L_PIN, OUTPUT);
  pinMode(DIR_L_PIN, OUTPUT);
  pinMode(SPD_R_PIN, OUTPUT);
  pinMode(DIR_R_PIN, OUTPUT);

  // Firing pins
  pinMode(FIRE_SPD_PIN, OUTPUT);
  pinMode(LOAD_SPD_PIN, OUTPUT);
  pinMode(FIRE_DIR_PIN, OUTPUT);
  // Set the launch motors to off
  analogWrite(FIRE_SPD_PIN, PWM_OFF); 
  analogWrite(LOAD_SPD_PIN, PWM_OFF);
  digitalWrite(FIRE_DIR_PIN, HIGH);

  // Bump pin
  pinMode(BUMP_PIN, INPUT);

  // Start the motor
  state = STATE_DRIVING;
  handleDriving();
}

void loop() {
  switch (state) {
    // Drive forward along the tape until bumping into the
    // armory wall. The left and middle IR sensors straddle
    // the tape.
    case STATE_DRIVING:
      driveOnTape();
      if (bump()) {
        state = STATE_BACKING1;
        handleBacking();
      }
      break;
    // Back away from the wall for a set time
    case STATE_BACKING1:
      if (navExpire()) {
        state = STATE_PIVOTING1;
        handlePivoting();
      }
      break;
    // Pivot (for a set time) to face the reload trigger
    case STATE_PIVOTING1:
      if (navExpire()) {
        state = STATE_DEPRESSING;
        handleDepressing();
      }
      break;
    // Drive forward to depress the reload trigger. If the
    // bump trigger fails, a backup timer advances the state.
    case STATE_DEPRESSING:
      if (bump() || navExpire()) {
        state = STATE_RELOADING;
        handleReloading();
      }
      break;
    // Hold still for a set time while we reload the WARRIOR
    case STATE_RELOADING:
      if (navExpire()) {
        state = STATE_BACKING2;
        handleBacking();
      }
      break;
    // Back away from the reload trigger until the axle is
    // in line with the tape
    case STATE_BACKING2:
      if (isTapeAxle()) {
        state = STATE_PIVOTING2;
        handlePivoting();
      }
      break;
    // Pivot until the middle sensor is over the tape
    case STATE_PIVOTING2:
      if (isTapeMiddle()) {
        state = STATE_SEEKING;
        handleSeeking();
      }
      break;
    // Drive until the axle is in line with the T, which is
    // our firing position. The middle and right IR sensors
    // straddle the tape.
    case STATE_SEEKING:
      driveOnTape();
      if (isTapeAxle()) {
        //state = STATE_AIMING;
        //handleAiming();
        state = STATE_FIRING;
        handleFiring();
      }
      break;
    // Back up a fraction of a second to align with King's Landing.
    case STATE_AIMING:
      if (navExpire()) {
        state = STATE_FIRING;
        handleFiring();
      }
      break;
    // Start the fly wheel motor so it can come up to speed
    case STATE_FIRING:
      if (fireExpire()) {
        state = STATE_LOADING;
        handleLoading();
      }
      break;
    // After a short delay, start the loading motor
    case STATE_LOADING:
      if (navExpire()) {
        state = STATE_STOPPING;
        handleStopping();
      }
    // After a set time, stop everything
    case STATE_STOPPING:
      break;
  }
}

/*---------------Module Functions---------------------------*/
// Read an individual IR sensor
// When a sensor is over white, the signal conditioning circuit
// produces a square wave. Over tape, it's always low. So the
// sensor is over tape if the most recent rising edge was more 
// than one IR period ago.
bool overTape(uint32_t now, Sensor_t sensor) {
  uint32_t lastRise = 0;

  // Get the time of the last rising edge for the selected sensor
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
  // This could probably be `return now - lastRise > irPeriod`
  uint32_t since_rise = now - lastRise;
  if (since_rise > irPeriod) {
    return true;
  } else {
    return false;
  }
}

// Detect limit switch bump
// Looks confusing, but the bump pin reads LOW when the limit
// switch is depressed.
bool bump(void) {
  bool notBumped = digitalRead(BUMP_PIN);
  return !notBumped;
}

// Timers expire
// The navTimer is for "set time" navigation states (e.g. backing, 
// pivoting, etc)
bool navExpire(void) {
  return navTimer.check();
}
// The fireTimer introduces a delay between starting the flywheel and 
// the loading motor
bool fireExpire(void) {
  return fireTimer.check();
}

// Driving
// The logic for staying on the tape is:
// Straddling the line (i.e. tape aligned)? Drive straight
// Crossing the tape junction? Drive straight
// If the tape is on the left or right (but not at the junction),
// rotate in the direction of the tape.
void driveOnTape(void) {
  if (isTapeAligned()) {
    driveStraight();
  } else if (isTapeJunction()) {
    driveStraight();
  } else if (isTapeLeft()) {
    rotate(ROT_LEFT);
  } else if (isTapeRight()) {
    rotate(ROT_RIGHT);
  } else {
    // This is just noise
  }
}

// Drive straight by setting direction and speed
void driveStraight(void) {
  // Should define DIR_FWD true and DIR_BWD false
  setDir(true);
  setSpd(MAX_SPD, MAX_SPD);
}
// Rotate in place by slowing both motros (by ADJUST_FACTOR) and
// setting the directions opposite
void rotate(bool turnLeft) {
  uint16_t adjustSpd = (int) (ADJUST_FACTOR * MAX_SPD);
  setSpd(adjustSpd, adjustSpd);
  
  bool left = turnLeft;
  bool right = !turnLeft;
  setDir(left, right);
}

// setDir is overloaded. Give it one direction to set both motors the same.  
// We write !dir because the motors are on backwards.
void setDir(bool dir) {
  digitalWrite(DIR_L_PIN, !dir);
  digitalWrite(DIR_R_PIN, !dir);
}

// Give setDir two directions to set the motors independently.
void setDir(bool leftDir, bool rightDir) {
  digitalWrite(DIR_L_PIN, !leftDir);
  digitalWrite(DIR_R_PIN, !rightDir);
}

// Set the left and right drive motor speeds
// We delay by ANALOG_DELAY (10 msec) to prevent a race condition due to 
// writing to an analog pin in rapid succession.
void setSpd(uint16_t left, uint16_t right) {
  delay(ANALOG_DELAY);
  analogWrite(SPD_L_PIN, left);
  analogWrite(SPD_R_PIN, right);
}

// Tape sensing
// Tape is aligned when sensors straddle it
bool isTapeAligned(void) {
  uint32_t now = micros();
  // In STATE_DRIVING, the left and middle sensors straddle the tape
  // In STATE_SEEKING, the middle and right sensors straddle the tape
  Sensor_t l_sensor = state == STATE_DRIVING ? SENSOR_L : SENSOR_M;
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  bool L = overTape(now, l_sensor);
  bool R = overTape(now, r_sensor);
  // The WARRIOR is aligned over the tape if neither sensor is over tape
  return !L && !R;
}
// Generally, tape is on the left if L && !R. But the junction can be tricky,
// so this is false if the junction is detected.
bool isTapeLeft(void) {
  uint32_t now = micros();
  Sensor_t l_sensor = state == STATE_DRIVING ? SENSOR_L : SENSOR_M;
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  // In STATE_DRIVING, the right sensor detects the junction. 
  // In STATE_SEEKING, the left sensor does it. 
  Sensor_t t_sensor = state == STATE_DRIVING ? SENSOR_R : SENSOR_L;
  bool L = overTape(now, l_sensor);
  bool R = overTape(now, r_sensor);
  bool T = overTape(now, t_sensor);
  if (T) return false;
  if (L && !R) return true;
  return false;
}
// Same but opposite of isTapeLeft
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
// Checks for the T 
bool isTapeJunction(void) {
  uint32_t now = micros();
  // I think this only matters in STATE_DRIVING, so some of this logic
  // could probably be stripped out. 
  Sensor_t r_sensor = state == STATE_DRIVING ? SENSOR_M : SENSOR_R;
  Sensor_t t_sensor = state == STATE_DRIVING ? SENSOR_R : SENSOR_L;
  bool R = overTape(now, r_sensor);
  bool T = overTape(now, t_sensor);
  return R && T;
}

// Tape sensing is noisy. That doesn't matter for continuous movement
// e.g. line sensing, but if we just want to sense when we've reached
// a point it causes issues. So the axle is only over the tape if we
// detect tape 10 times in 2msec.
bool isTapeAxle(void) {
  // Note the time
  uint32_t now = micros();
  // Should be `if(!overTape(now, SENSOR_A)) return false;
  bool tapeAxle = overTape(now, SENSOR_A);
  if (!tapeAxle) return false;
  // Start the tape counter at 1
  int tapeCounter = 1;
  // Keep  track of the number of detections in 2 msec
  uint32_t now2 = micros();
  while (now2 - now < NOISE_T) {
    if (overTape(now2, SENSOR_A)) tapeCounter++;
    // If we get 10 detections, the axle is definitely over the tape
    if (tapeCounter > 10) return true;
    now2 = micros();
  }
  // Otherwise, just noise
  return false;
}
// Same idea as isTapeAxle, but for the middle sensor instead of the axle
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

// Firing
// Use a short delay to prevent analogWrite race conditions
void startFiring(void) {
  delay(ANALOG_DELAY);
  analogWrite(FIRE_SPD_PIN, FIRE_DUTY);
}
// Use a short delay to prevent analogWrite race conditions
void startLoading(void) {
  delay(ANALOG_DELAY);
  analogWrite(LOAD_SPD_PIN, LOAD_DUTY);
}
// Use a short delay to prevent analogWrite race conditions
void stopFiring(void) {
  delay(ANALOG_DELAY);
  analogWrite(FIRE_SPD_PIN, PWM_OFF);
  analogWrite(LOAD_SPD_PIN, PWM_OFF);
}

// Handle events
// Each of these handlers correspond to entering a state
// Drive by turning both drive motors forwards and on
void handleDriving(void) {
  Serial.println("DRIVING");
  setSpd(MAX_SPD, MAX_SPD);
  setDir(true);
}
// Back up for a set time
void handleBacking(void) {
  navTimer.interval(BACKING_T);
  navTimer.reset();
  Serial.println("BACKING");
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Back up
  setDir(false);
  setSpd(MAX_SPD, MAX_SPD);
}
// Pivot for a set time
void handlePivoting(void) {
  navTimer.interval(PIVOTING_T);
  navTimer.reset();
  Serial.println("PIVOTING");
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Rotate right
  rotate(ROT_RIGHT);
}
// Drive into the reloading trigger (for a set time or until the bump).
void handleDepressing(void) {
  navTimer.interval(DEPRESSING_T);
  navTimer.reset();
  Serial.println("DEPRESSING");
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Drive forward
  driveStraight();
}
// Sit still for a set time while we reload the warrior
void handleReloading(void) {
  // Turn off motors
  setSpd(0, 0);
  navTimer.interval(RELOADING_T);
  navTimer.reset();
  Serial.println("RELOADING");
}
// Follow the line from the armory back to the firing position (the T)
void handleSeeking(void) {
  Serial.println("SEEKING");
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  // Go forwards
  setSpd(MAX_SPD, MAX_SPD);
  setDir(true);
}
// Back up a fraction of a second to align with King's Landing
void handleAiming(void) {
  Serial.println("AIMING");
  // Come to a stop
  setSpd(0, 0);
  delay(100);
  navTimer.interval(AIMING_T);
  navTimer.reset();
  // Go backwards
  setSpd(MAX_SPD, MAX_SPD);
  setDir(false);
}
// Start the firing sequence
void handleFiring(void) {
  // Stop the drive motors
  setSpd(0, 0);
  // Start a timer for the WHOLE firing sequence
  navTimer.interval(FIRING_T);
  navTimer.reset();
  // Start a timer to delay the loading motor
  fireTimer.interval(FIRE_DELAY);
  fireTimer.reset();

  Serial.println("FIRING");
  // Turn on the flywheel motor
  startFiring();
}
// Start loading wildfire
void handleLoading(void) {
  // Drive motors should be off anyway, but just to be safe
  setSpd(0, 0);

  Serial.println("LOADING");
  // Turn on the loading motor
  startLoading();
}
// Stop all the motors (drive, flywheel, and loading)
void handleStopping(void) {
  setSpd(0, 0);
  stopFiring();
  Serial.println("STOPPING");
}

/*---------------Interrupt Handler Prototypes---------------*/
// When the irTimer expires, flip the signal for both IR emission
// pins
void toggleIr(void) {
  digitalWrite(IR_E1_PIN, irHigh ? HIGH : LOW);
  digitalWrite(IR_E2_PIN, irHigh ? HIGH : LOW);
  irHigh = !irHigh;
}
// These interrupt handlers are connected to rising edges on the 
// four IR sensor receiving pins. They keep track of when the last
// rising edge was so we can tell apart tape and white
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
