#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

const bool FORWARD = true;   // Indicate forward rotation
const bool REVERSE = false;  // Indicate reversal

class MotorDriverIOTController {
 private:
  // Motor control pins
  int motorAIn1;
  int motorAIn2;
  int motorBIn1;
  int motorBIn2;

  int maxSpeed;  // Maximum speed value(0-255)

 public:
  // Constructor function
  MotorDriverIOTController(int motorAIn1, int motorAIn2, int motorBIn1, int motorBIn2, int maxSpeed = 255);

  void Begin();

  // Single motor control (with added directional parameters)
  void MotorRun(char motor, bool direction, int speed);

  // Dual motor control (with added directional parameters)
  void BothRun(bool direction, int speedA, int speedB);

  // Stop function
  void MotorStop(char motor);
  void BothStop();
};

#endif