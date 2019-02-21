#include <Arduino.h>

/*---------------Constants----------------------------------*/
#define IR_E_PIN 21
#define IR_R_PIN 19

/*---------------Module Function Prototypes-----------------*/


/*---------------Interrupt Handlers Prototypes--------------*/
void toggle_ir(void);

/*---------------Module Variables---------------------------*/
uint16_t ir_freq = 10000; // 10kHz
uint16_t ir_period = 1000000 / ir_freq; // in microseconds
uint16_t ir_switch = ir_period / 2;
IntervalTimer ir_timer;
volatile bool ir_high;

/*---------------Teensy Main Functions----------------------*/
void setup() {
  // Connect to Serial
  Serial.begin(9600);
  while(!Serial);

  // Set pin input/outputs
  pinMode(IR_E_PIN, OUTPUT);
  pinMode(IR_R_PIN, INPUT);
  
  ir_timer.begin(toggle_ir, ir_switch);
  ir_high = true;
}

void loop() {
  // Check for signal
  /*
  uint16_t ir_signal = analogRead(IR_R_PIN);
  Serial.println(ir_signal);
  delay(1000);
  */
}

/*----------------Module Functions--------------------------*/

/*---------------Interrupt Handlers-------------------------*/
void toggle_ir(void) {
  digitalWrite(IR_E_PIN, ir_high ? HIGH : LOW);
  ir_high = !ir_high;
}
