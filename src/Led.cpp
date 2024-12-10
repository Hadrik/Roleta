#include "Led.h"

Led::Led(uint8_t pin) {
  pinMode(pin, OUTPUT);
  _pin = pin;
  _fade_time = 200;
  _step = 255.0 / _fade_time;
  off();
}

void Led::update() {
  long dist = _end_time - millis();
  if (dist > static_cast<long>(toOff())) {
    if (_brightness < 255 - _step) {
      _brightness += _step;
    } else {
      _brightness = 255;
    }
  } else {
    if (_brightness > _step) {
      _brightness -= _step;
    } else {
      _brightness = 0;
    }
  }
  analogWrite(_pin, static_cast<uint8_t>(_brightness));
}

void Led::on() {
  _end_time = (0 - 1);
}

void Led::off() {
  _end_time = millis() + _fade_time;
}

void Led::setFadeTime(uint32_t ms) {
  _fade_time = ms;
  _step = 255.0 / _fade_time;
}

void Led::onFor(uint32_t ms) {
  _end_time = millis() + ms;
}

uint32_t Led::toOff() {
  return static_cast<uint32_t>((_brightness / 255.0) * _fade_time);
}
