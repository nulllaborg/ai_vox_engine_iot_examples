// WS2812BIOTController.h

#ifndef WS2812B_CONTROLLER_H
#define WS2812B_CONTROLLER_H

#include <Adafruit_NeoPixel.h>

class WS2812BIOTController {
 public:
  WS2812BIOTController(uint16_t numPixels, uint8_t pin);
  void Begin();
  void SetPixelColor(uint16_t pixelIndex, uint8_t red, uint8_t green, uint8_t blue);
  void SetPixelColorHSV(uint16_t pixelIndex, uint16_t hue, uint8_t sat = 255, uint8_t val = 255);
  void Show();
  void Clear();
  uint16_t NumPixels() const;

  // Brightness control function
  void SetBrightness(uint8_t brightness);
  uint8_t GetBrightness() const;

  // Color fill function
  void Fill(uint8_t red, uint8_t green, uint8_t blue);
  void FillHSV(uint16_t hue, uint8_t sat = 255, uint8_t val = 255);

  // Region filling function
  void FillRange(uint16_t startPixel, uint16_t endPixel, uint8_t red, uint8_t green, uint8_t blue);
  void FillRangeHSV(uint16_t startPixel, uint16_t endPixel, uint16_t hue, uint8_t sat = 255, uint8_t val = 255);

 private:
  Adafruit_NeoPixel _strip;
  const uint16_t _numPixels;
  const uint8_t _pin;
};

#endif