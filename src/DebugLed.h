#ifndef DEBUGLED_H
#define DEBUGLED_H

#pragma once

#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Ticker.h>
#include <string>
#include <map>

#ifndef LED_DEBUG_PREFIX
  #define LED_DEBUG_PREFIX "\t[DebugLed] "
#endif
#ifndef LED_DEBUG_DISABLE
  #define LED_DEBUG_ENABLE
#endif

class DebugLed {
public:
  DebugLed(const uint8_t pin, const bool invert = false);
  DebugLed(const uint8_t pin, SoftwareSerial* s, const bool invert = false);
  ~DebugLed() = default;

  void addState(const std::string& name, const uint32_t delay, const uint8_t priority = 0, const bool on = false);
  bool setState(const std::string& name, const bool on);

private:
  void update();
  void write(bool on);

  uint8_t _pin;
  bool _invert;

  struct state {
    uint32_t delay;
    uint8_t priority;
    bool on;
  };

  std::map<std::string, state> _states;

  struct tick {
    Ticker t;
    std::map<std::string, state>::iterator current;
  } _ticker;

  SoftwareSerial* _s = nullptr;
};

#endif