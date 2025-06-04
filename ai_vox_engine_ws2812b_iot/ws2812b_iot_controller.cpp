// WS2812BIOTController.cpp

#include "ws2812b_iot_controller.h"

#include <Arduino.h>

WS2812BIOTController::WS2812BIOTController(uint16_t numPixels, uint8_t pin)
    : _numPixels(numPixels), _pin(pin), _strip(numPixels, pin, NEO_GRB + NEO_KHZ800) {
}

void WS2812BIOTController::Begin() {
  _strip.begin();
  _strip.setBrightness(128);  // Set the default brightness to medium
  _strip.show();              // Turn off all LEDs during initialization
}

void WS2812BIOTController::SetPixelColor(uint16_t pixelIndex, uint8_t red, uint8_t green, uint8_t blue) {
  if (pixelIndex < _numPixels) {
    _strip.setPixelColor(pixelIndex, _strip.Color(red, green, blue));
  }
}

void WS2812BIOTController::SetPixelColorHSV(uint16_t pixelIndex, uint16_t hue, uint8_t sat, uint8_t val) {
  if (pixelIndex < _numPixels) {
    uint32_t color = _strip.gamma32(_strip.ColorHSV(hue, sat, val));
    _strip.setPixelColor(pixelIndex, color);
  }
}

void WS2812BIOTController::Show() {
  _strip.show();
}

void WS2812BIOTController::Clear() {
  _strip.clear();
  _strip.show();
}

uint16_t WS2812BIOTController::NumPixels() const {
  return _numPixels;
}

// Implementation of brightness control function
void WS2812BIOTController::SetBrightness(uint8_t brightness) {
  _strip.setBrightness(brightness);
}

uint8_t WS2812BIOTController::GetBrightness() const {
  return _strip.getBrightness();
}

// Implementation of brightness control function and color filling function - RGB version
void WS2812BIOTController::Fill(uint8_t red, uint8_t green, uint8_t blue) {
  uint32_t color = _strip.Color(red, green, blue);
  _strip.fill(color);
}

// Implementation of Color Fill Function - HSV version
void WS2812BIOTController::FillHSV(uint16_t hue, uint8_t sat, uint8_t val) {
  uint32_t color = _strip.gamma32(_strip.ColorHSV(hue, sat, val));
  _strip.fill(color);
}

// Implementation of Region Filling Function - RGB version
void WS2812BIOTController::FillRange(uint16_t startPixel, uint16_t endPixel, uint8_t red, uint8_t green, uint8_t blue) {
  // Ensure that the starting and ending pixels are within the valid range
  if (startPixel >= _numPixels || endPixel >= _numPixels || startPixel > endPixel) {
    return;
  }

  uint32_t color = _strip.Color(red, green, blue);

  // Implementation of Region Filling Function
  for (uint16_t i = startPixel; i <= endPixel; i++) {
    _strip.setPixelColor(i, color);
  }
}

// Implementation of Region Filling Function - HSV version
void WS2812BIOTController::FillRangeHSV(uint16_t startPixel, uint16_t endPixel, uint16_t hue, uint8_t sat, uint8_t val) {
  // Ensure that the starting and ending pixels are within the valid range
  if (startPixel >= _numPixels || endPixel >= _numPixels || startPixel > endPixel) {
    return;
  }

  uint32_t color = _strip.gamma32(_strip.ColorHSV(hue, sat, val));

  // Fill the designated area
  for (uint16_t i = startPixel; i <= endPixel; i++) {
    _strip.setPixelColor(i, color);
  }
}