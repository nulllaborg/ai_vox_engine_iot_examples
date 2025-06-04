#include "motordriver_iot_controller.h"

#include "Arduino.h"

MotorDriverIOTController::MotorDriverIOTController(int aIn1, int aIn2, int bIn1, int bIn2, int maxSpeed) {
  // Initialize pins
  motorAIn1 = aIn1;
  motorAIn2 = aIn2;
  motorBIn1 = bIn1;
  motorBIn2 = bIn2;
  this->maxSpeed = maxSpeed;
}

void MotorDriverIOTController::Begin() {
  pinMode(motorAIn1, OUTPUT);
  pinMode(motorAIn2, OUTPUT);
  pinMode(motorBIn1, OUTPUT);
  pinMode(motorBIn2, OUTPUT);
  BothStop();
}

void MotorDriverIOTController::MotorRun(char motor, bool direction, int speed) {
  speed = constrain(speed, 0, maxSpeed);
  int in1, in2;

  // Select motor
  if (motor == 'A' || motor == 'a') {
    in1 = motorAIn1;
    in2 = motorAIn2;
  } else if (motor == 'B' || motor == 'b') {
    in1 = motorBIn1;
    in2 = motorBIn2;
  } else {
    return;  // Invalid motor identification
  }

  // Set direction
  if (direction == FORWARD) {
    analogWrite(in1, speed);
    digitalWrite(in2, LOW);
  } else {
    analogWrite(in1, speed);
    digitalWrite(in2, HIGH);
  }
}

void MotorDriverIOTController::BothRun(bool direction, int speedA, int speedB) {
  MotorRun('A', direction, speedA);
  MotorRun('B', direction, speedB);
}

void MotorDriverIOTController::MotorStop(char motor) {
  MotorRun(motor, FORWARD, 0);  // Set the speed to 0 to stop
}

void MotorDriverIOTController::BothStop() {
  MotorStop('A');
  MotorStop('B');
}