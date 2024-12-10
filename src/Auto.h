#ifndef AUTO_H
#define AUTO_H

#pragma once

#include <Arduino.h>
#include <functional>

class Auto {
public:
  Auto(const uint8_t pin, bool enable = true, double day = 0.0, double night = 1.0);
  ~Auto() = default;

  Auto& registerDayCB(std::function<void()> cb) { _day.callback = cb; return *this; };
  Auto& registerNightCB(std::function<void()> cb) { _night.callback = cb; return *this; };
  Auto& setDayThreshold(double th) { _day.threshold = th; return *this; };
  double getDayThreshold() { return _day.threshold; };
  Auto& setNightThreshold(double th) { _night.threshold = th; return *this; };
  double getNightThreshold() { return _night.threshold; };

  double read();
  void enabled(bool en) { _enable = en; }
  bool enabled() { return _enable; }

  void update();

private:
  uint8_t _pin;
  bool _enable;

  struct Trigger {
    double threshold;
    std::function<void()> callback;
    bool called;
  } _day, _night;
};

#endif