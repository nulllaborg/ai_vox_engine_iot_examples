
#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include <Arduino.h>

class US04IOTController {
 public:
  US04IOTController(int trigPin, int echoPin);  // Constructor function
  void Begin();                                 // Initialize pins
  int Measure();                                // Measure distance (return centimeter value)

 private:
  int _trigPin;
  int _echoPin;
};

#endif