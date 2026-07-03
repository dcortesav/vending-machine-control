#include <Arduino.h>

#define MOTOR_CONVEYOR_FORWARD  26
#define MOTOR_CONVEYOR_BACKWARDS  27

void setup() {
  // Initialize setup
  Serial.begin(115200);
  delay(50);

  // Initialize motor control pins
  pinMode(MOTOR_CONVEYOR_FORWARD, OUTPUT); pinMode(MOTOR_CONVEYOR_BACKWARDS, OUTPUT);
  delay(50);
  digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
  digitalWrite(MOTOR_CONVEYOR_BACKWARDS, LOW);
  delay(50);
}

void loop() {
  if (Serial.available() > 0) {
    int value = Serial.parseInt();
    Serial.print("Value: ");
    Serial.println(value);
    if (value == 1) {
      digitalWrite(MOTOR_CONVEYOR_FORWARD, HIGH);
      digitalWrite(MOTOR_CONVEYOR_BACKWARDS, LOW);
    } else if (value == 2) {
      digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
      digitalWrite(MOTOR_CONVEYOR_BACKWARDS, HIGH);
    } else if (value == 3) {
      digitalWrite(MOTOR_CONVEYOR_FORWARD, LOW);
      digitalWrite(MOTOR_CONVEYOR_BACKWARDS, LOW);
    }
  }
}