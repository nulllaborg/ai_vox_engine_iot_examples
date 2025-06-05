#include "us04_iot_controller.h"

US04IOTController::US04IOTController(int trigPin, int echoPin) {
  _trigPin = trigPin;
  _echoPin = echoPin;
}

void US04IOTController::Begin() {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

int US04IOTController::Measure() {
  // Send trigger signal
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  // Measure echo time
  long duration = pulseIn(_echoPin, HIGH);

  // Calculate distance (in centimeters)
  return (int)duration * 0.034 / 2;
}