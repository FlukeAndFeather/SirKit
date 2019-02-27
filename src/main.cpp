#include <Arduino.h>

/*---------------Constants----------------------------------*/
#define IR_E_PIN1 16
#define IR_E_PIN2 21
#define IR_R_PIN1 9
#define IR_R_PIN2 10
#define IR_R_PIN3 11
#define IR_R_PIN4 12
#define S1_MASK 0b1000
#define S2_MASK 0b0100
#define S3_MASK 0b0010
#define S4_MASK 0b0001

#define MOTOR_SPD 100

/*---------------Enumerations-------------------------------*/
typedef enum { SENSOR1, SENSOR2, SENSOR3, SENSOR4 } sensor_t;

/*---------------Module Function Prototypes-----------------*/
void drive(void);
void adjust_right(void);
void adjust_left(void);
uint8_t read_tape(void);
bool is_tape_aligned(uint32_t now);
bool is_tape_left(uint32_t now);
bool is_tape_right(uint32_t now);
bool over_tape(uint32_t now, sensor_t sensor);

/*---------------Interrupt Handlers Prototypes--------------*/
void toggle_ir(void);
void ir_rising1(void);
void ir_rising2(void);
void ir_rising3(void);
void ir_rising4(void);

/*---------------Module Variables---------------------------*/
// IR emit
uint16_t ir_freq = 450; // 1kHz
uint16_t ir_period = 1000000 / ir_freq; // in microseconds
uint16_t ir_switch = ir_period / 2;
IntervalTimer ir_timer;
volatile bool ir_high;

// IR receive
volatile uint32_t last_rise1;
volatile uint32_t last_rise2;
volatile uint32_t last_rise3;
volatile uint32_t last_rise4;

/*---------------Teensy Main Functions----------------------*/
void setup() {
  // Connect to Serial
  Serial.begin(9600);
  while(!Serial);

  // Set pin input/outputs
  pinMode(IR_E_PIN1, OUTPUT);
  pinMode(IR_E_PIN2, OUTPUT);
  pinMode(IR_R_PIN1, INPUT);
  attachInterrupt(IR_R_PIN1, ir_rising1, RISING);
  pinMode(IR_R_PIN2, INPUT);
  attachInterrupt(IR_R_PIN2, ir_rising2, RISING);
  pinMode(IR_R_PIN3, INPUT);
  attachInterrupt(IR_R_PIN3, ir_rising3, RISING);
  pinMode(IR_R_PIN4, INPUT);
  attachInterrupt(IR_R_PIN4, ir_rising4, RISING);
  
  ir_timer.begin(toggle_ir, ir_switch);
  ir_high = true;
}

void loop() {
  // Read IR sensors
  Serial.println(read_tape(), BIN);
  uint32_t now = micros();
  if(is_tape_aligned(now)) {
    drive();
  } else if(is_tape_left(now)) {
    adjust_right();
  } else if(is_tape_right(now)) {
    adjust_left();
  } else {
    Serial.println("OFF TAPE");
  }
  delay(1000);
}

/*----------------Module Functions--------------------------*/
void drive(void) {
  Serial.println("DRIVE");
}
void adjust_right(void) {
  Serial.println("ADJUST_RIGHT");
}
void adjust_left(void) {
  Serial.println("ADJUST_LEFT");
}

// Return 4 bits corresponding to S1 S2 S3 S4
uint8_t read_tape(void) {
  uint32_t now = micros();
  uint8_t result = over_tape(now, SENSOR4) |
    (over_tape(now, SENSOR3) << 1) |
    (over_tape(now, SENSOR2) << 2) |
    (over_tape(now, SENSOR1) << 3);
  return result;
}

bool is_tape_aligned(uint32_t now) {
  bool s2 = over_tape(now, SENSOR2);
  bool s3 = over_tape(now, SENSOR3);
  return s2 && s3;
}
bool is_tape_left(uint32_t now) {
  bool s2 = over_tape(now, SENSOR2);
  bool s3 = over_tape(now, SENSOR3);
  return s2 && !s3;
}
bool is_tape_right(uint32_t now) {
  bool s2 = over_tape(now, SENSOR2);
  bool s3 = over_tape(now, SENSOR3);
  return !s2 && s3;
}

// Read an individual sensor
bool over_tape(uint32_t now, sensor_t sensor) {
  uint32_t last_rise = 0;
  switch(sensor) {
    case SENSOR1:
      last_rise = last_rise1;
      break;
    case SENSOR2:
      last_rise = last_rise2;
      break;
    case SENSOR3:
      last_rise = last_rise3;
      break;
    case SENSOR4:
      last_rise = last_rise4;
      break;
  }
  uint32_t since_rise = now - last_rise;
  if(since_rise > ir_period) {
    return true;
  } else {
    return false;
  }
}

/*---------------Interrupt Handlers-------------------------*/
void toggle_ir(void) {
  digitalWrite(IR_E_PIN1, ir_high ? HIGH : LOW);
  digitalWrite(IR_E_PIN2, ir_high ? HIGH : LOW);
  ir_high = !ir_high;
}

void ir_rising1(void) {
  last_rise1 = micros();
}
void ir_rising2(void) {
  last_rise2 = micros();
}
void ir_rising3(void) {
  last_rise3 = micros();
}
void ir_rising4(void) {
  last_rise4 = micros();
}
