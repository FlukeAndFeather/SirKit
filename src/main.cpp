#include <Arduino.h>

/*---------------Constants----------------------------------*/
#define IR_E_PIN 21
#define IR_R_PIN 19

/*---------------Module Function Prototypes-----------------*/


/*---------------Module Variables---------------------------*/


/*---------------Teensy Main Functions----------------------*/
void setup() {
  // Connect to Serial
  Serial.begin(9600);
  while(!Serial);

  // Set pin input/outputs
  pinMode(IR_E_PIN, OUTPUT);
  pinMode(IR_R_PIN, INPUT);

  // Emit IR
  digitalWrite(IR_E_PIN, HIGH);
}

void loop() {
  // Check for signal
  uint16_t ir_signal = analogRead(IR_R_PIN);
  Serial.println(ir_signal);
  delay(1000);
}

/*----------------Module Functions--------------------------*/
