#include "Motor.h"

#ifdef MOTOR_DEBUG_ENABLE
  #define debug_motor(...)           \
    if (s) {                         \
      s->print(MOTOR_DEBUG_PREFIX);  \
      s->printf(__VA_ARGS__);        \
    }
#else
  #define debug_motor(...)
#endif

Motor::Motor(HardwareSerial* port,
             float r_sense,
             uint8_t enable,
             uint8_t direction,
             uint8_t step,
             uint8_t home,
             uint32_t total,
             SoftwareSerial* debug)
    : _port(port), s(debug), _driver(port, r_sense), _stepper(_stepper.DRIVER, step, direction) {
  _pin_en = enable;
  _pin_dir = direction;
  _pin_step = step;
  _pin_home = home;
  _total_steps = total;
  _stepper.setEnablePin(_pin_en);
  _stepper.setPinsInverted(false, false, true);
}

void Motor::begin() {
  pinMode(_pin_en, OUTPUT);
  pinMode(_pin_dir, OUTPUT);
  pinMode(_pin_step, OUTPUT);
  debug_motor("Set motor pins\n");
  if (s) s->end();

  _port->begin(115200);
  delay(500);
  _port->swap();
  delay(500);

  if (s) s->begin(9600);
  debug_motor("Stepper driver connected\n");
  digitalWrite(_pin_en, LOW);
  _driver.begin();
  _driver.toff(3);
  _driver.rms_current(300);
  _driver.microsteps(4);
  _driver.en_spreadCycle(false);
  _driver.pwm_autoscale(true);
  debug_motor("Driver setup finished\n");
}

void Motor::update() {
  // if (!_stepper.isRunning()) {
  //   _stepper.disableOutputs();
  // } else {
  //   _stepper.enableOutputs();
  //   _stepper.run();
  // }
  if (!_stepper.run()) {
    _stepper.disableOutputs();
  }
}

void Motor::home(bool invert) {
  if (_pin_home == 255) return;
  _stepper.enableOutputs();

  _stepper.move(-_total_steps * 1.5);
  while (digitalRead(_pin_home) == invert) {
    _stepper.run();
  }
  _stepper.setCurrentPosition(0);
  _stepper.setAcceleration(MAXFLOAT);
  _stepper.runToNewPosition(0);
}

void Motor::estop() {
  _stepper.setAcceleration(MAXFLOAT);
  _stepper.stop();
  _stepper.disableOutputs();
}

uint32_t Motor::getRemainingSteps() {
  return _stepper.distanceToGo();
}

Motor& Motor::setTargetP(double target) {
  if (target > 1.0 || target < 0.0) return *this;
  _stepper.enableOutputs();
  _stepper.moveTo(target * _total_steps);
  return *this;
}

Motor& Motor::setTarget(uint32_t target) {
  if (target > _total_steps) return *this;
  _stepper.enableOutputs();
  _stepper.moveTo(target);
  return *this;
}

Motor& Motor::forceTarget(uint32_t target) {
  _stepper.enableOutputs();
  _stepper.moveTo(target);
  return *this;
}

double Motor::getTargetP() {
  return getTarget() / static_cast<double>(_total_steps);
}

uint32_t Motor::getTarget() {
  return _stepper.targetPosition();
}

Motor& Motor::setPositionP(double position) {
  setPosition(position * _total_steps);
  return *this;
}

Motor& Motor::setPosition(uint32_t position) {
  _stepper.setCurrentPosition(position);
  return *this;
}

double Motor::getPositionP() {
  return getPosition() / static_cast<double>(_total_steps);
}

uint32_t Motor::getPosition() {
  return _stepper.currentPosition();
}

Motor& Motor::setTotalSteps(uint32_t steps) {
  _total_steps = steps;
  return *this;
}

uint32_t Motor::getTotalSteps() {
  return _total_steps;
}

Motor& Motor::setAcceleration(float acceleration) {
  _stepper.setAcceleration(acceleration);
  return *this;
}

float Motor::getAcceleration() {
  return _stepper.acceleration();
}

Motor& Motor::setSpeed(float speed) {
  _stepper.setMaxSpeed(speed);
  return *this;
}

float Motor::getSpeed() {
  return _stepper.maxSpeed();
}

Motor& Motor::setCurrent(uint16_t mA) {
  _driver.rms_current(mA);
  return *this;
}

uint16_t Motor::getCurrent() {
  return _driver.rms_current();
}

Motor& Motor::setMicrosteps(uint16_t steps) {
  _driver.microsteps(steps);
  return *this;
}

uint16_t Motor::getMicrosteps() {
  return _driver.microsteps();
}

Motor& Motor::useStealthChop(bool use) {
  _driver.en_spreadCycle(!use);
  return *this;
}
