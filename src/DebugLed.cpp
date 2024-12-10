#include "DebugLed.h"

#ifdef LED_DEBUG_ENABLE
  #define debug_led(...)           \
    if (_s) {                      \
      _s->print(LED_DEBUG_PREFIX); \
      _s->printf(__VA_ARGS__);     \
    }
#else
  #define debug_led(...)
#endif

DebugLed::DebugLed(const uint8_t pin, const bool invert) {
  pinMode(pin, OUTPUT);
  write(LOW);
  _pin = pin;
  _invert = invert;
}

DebugLed::DebugLed(const uint8_t pin, SoftwareSerial* s, const bool invert) {
  pinMode(pin, OUTPUT);
  write(LOW);
  _pin = pin;
  _s = s;
  _invert = invert;
}

void DebugLed::addState(const std::string& name,
                        const uint32_t delay,
                        const uint8_t priority,
                        const bool on) {
  _states.emplace(name, state{delay, priority, on});
  debug_led("Added debug led state: %s ; priority: %d\n", name.c_str(), priority);
  update();
}

bool DebugLed::setState(const std::string& name, const bool on) {
  debug_led("Setting debug led state: %s ; on: %d\n", name.c_str(), on);
  const auto state = _states.find(name);
  if (state == _states.end()) {
    debug_led("State not found\n");
    return false;
  }

  state->second.on = on;
  update();
  return true;
}

void DebugLed::update() {
  debug_led("Updating debug led\n");
  // search all the states that are on and find the one with the highest priority
  auto state = _states.end();
  for (auto it = _states.begin(); it != _states.end(); ++it) {
    debug_led("Checking state: %s ; current highest: %s\n", it->first.c_str(), (state == _states.end() ? "none" : state->first.c_str()));
    if (it->second.on && (it->second.priority > state->second.priority || state == _states.end())) {
      debug_led("State is on and has higher priority\n");
      state = it;
    }
  }

  // if no state is on, turn off the led
  if (state == _states.end()) {
    debug_led("No state is on\n");
    write(LOW);
    _ticker.t.detach();
    return;
  }
  debug_led("Highest priority state is: %s\n", state->first.c_str());

  // if the state is different from the current state, update the ticker
  if (_ticker.current != state) {
    debug_led("State is different from current, updating\n");
    _ticker.current = state;
    _ticker.t.attach_ms(state->second.delay, [this]() { digitalWrite(_pin, !digitalRead(_pin)); });
  }
}

void DebugLed::write(bool on) {
  digitalWrite(_pin, on ^ _invert);
}
