// ServoIOTController.h
#ifndef _SERVO_CONTROLLER_H
#define _SERVO_CONTROLLER_H

#include <Arduino.h>
#include <driver/ledc.h>

class ServoIOTController {
 public:
  /**
   * Constructor, initialize two servos
   * @param pinA servo A control pin
   * @param pinB servo B control pin
   * @param channelA LEDC channel A (0-15)
   * @param channelB LEDC channel B (0-15)
   * @param freq PWM frequency (usually 50Hz)
   * @param resolution Resolution (12 bits recommended)
   */
  ServoIOTController(uint8_t pinA,
                     uint8_t pinB,
                     ledc_channel_t channelA = LEDC_CHANNEL_0,
                     ledc_channel_t channelB = LEDC_CHANNEL_1,
                     uint32_t freq = 50,
                     uint8_t resolution = 12);

  /**
   * Initialize the servo controller
   */
  void Init();

  /**
   * Set servo angle
   * @param servo Steering gear identification ('A 'or' B ')
   * @param angle target angle（0-180）
   */
  void SetAngle(char servo, uint8_t angle);

  /**
   * Release PWM resources
   */
  ~ServoIOTController();

 private:
  // Parameters of servo A
  uint8_t _pinA;
  ledc_channel_t _channelA;

  // Parameters of servo B
  uint8_t _pinB;
  ledc_channel_t _channelB;

  uint32_t _freq;
  uint8_t _resolution;

  // Minimum/maximum pulse width (microseconds)
  static const uint16_t MIN_PULSE;
  static const uint16_t MAX_PULSE;

  /**
   * Initialize a single servo motor
   * @param pin Control pins
   * @param channel LEDC channel
   */
  void InitSingle(uint8_t pin, ledc_channel_t channel);

  /**
   * Calculate PWM duty cycle
   * @param angle target angle
   * @return Duty cycle value
   */
  uint32_t CalculateDuty(uint8_t angle);
};

#endif