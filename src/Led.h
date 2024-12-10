#ifndef LED_H
#define LED_H

#pragma once

#include <Arduino.h>

class Led {
public:
  Led(uint8_t pin);
  ~Led() = default;

  void update();

  void on();
  void onFor(uint32_t ms);
  void off();

  void setFadeTime(uint32_t ms);

private:
  uint8_t _pin;
  uint32_t _end_time, _fade_time;
  double _step, _brightness;

  uint32_t toOff();
};

#endif