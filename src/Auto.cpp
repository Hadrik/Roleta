#include "Auto.h"

Auto::Auto(const uint8_t pin, bool enable, double day, double night) {
  _pin = pin;
  _enable = enable;
  _day.threshold = day;
  _day.called = false;
  _night.threshold = night;
  _night.called = false;
  pinMode(_pin, INPUT);
}

double Auto::read() {
  return analogRead(_pin) / 1024.0;
}

void Auto::update() {
  if (!_enable) return;
  double light = read();

  if (light < _night.threshold) {
    if (_night.callback && !_night.called) {
      _night.callback();
      _night.called = true;
      _day.called = false;
    }
  } else if (light > _day.threshold) {
    if (_day.callback && !_day.called) {
      _day.callback();
      _day.called = true;
      _night.called = false;
    }
  }
}
