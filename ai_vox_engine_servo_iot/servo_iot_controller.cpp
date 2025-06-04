// ServoIOTController.cpp
#include "servo_iot_controller.h"

const uint16_t ServoIOTController::MIN_PULSE = 500;
const uint16_t ServoIOTController::MAX_PULSE = 2500;

ServoIOTController::ServoIOTController(
    uint8_t pinA, uint8_t pinB, ledc_channel_t channelA, ledc_channel_t channelB, uint32_t freq, uint8_t resolution)
    : _pinA(pinA), _pinB(pinB), _channelA(channelA), _channelB(channelB), _freq(freq), _resolution(resolution) {
}

void ServoIOTController::Init() {
  InitSingle(_pinA, _channelA);
  InitSingle(_pinB, _channelB);
}

void ServoIOTController::InitSingle(uint8_t pin, ledc_channel_t channel) {
  // Configure Timer
  ledc_timer_config_t timer_conf = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                    .duty_resolution = static_cast<ledc_timer_bit_t>(_resolution),
                                    .timer_num = LEDC_TIMER_0,  // 两个舵机共用一个定时器（需确保通道不冲突）
                                    .freq_hz = _freq,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ledc_timer_config(&timer_conf);

  // Configure channels
  ledc_channel_config_t channel_conf = {.gpio_num = pin,
                                        .speed_mode = LEDC_LOW_SPEED_MODE,
                                        .channel = channel,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .timer_sel = LEDC_TIMER_0,
                                        .duty = 0,
                                        .hpoint = 0};
  ledc_channel_config(&channel_conf);
}

void ServoIOTController::SetAngle(char servo, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  uint32_t duty = CalculateDuty(angle);

  switch (servo) {
    case 'A':
      ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelA, duty);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelA);
      break;
    case 'B':
      ledc_set_duty(LEDC_LOW_SPEED_MODE, _channelB, duty);
      ledc_update_duty(LEDC_LOW_SPEED_MODE, _channelB);
      break;
    default:
      return;  // Invalid servo motor identifier
  }
}

uint32_t ServoIOTController::CalculateDuty(uint8_t angle) {
  // Map the angle to the pulse width（500-2500μs）
  uint32_t pulse_width = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
  // Calculate duty cycle (based on frequency and resolution)
  return (pulse_width * (1 << _resolution)) / (1000000 / _freq);
}

ServoIOTController::~ServoIOTController() {
  // Release two channel resources
  ledc_stop(LEDC_LOW_SPEED_MODE, _channelA, 0);
  ledc_stop(LEDC_LOW_SPEED_MODE, _channelB, 0);
}