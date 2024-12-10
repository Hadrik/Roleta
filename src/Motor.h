#ifndef MOTOR_H
#define MOTOR_H

#pragma once

#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>

#ifndef MOTOR_DEBUG_PREFIX
  #define MOTOR_DEBUG_PREFIX "\t[Motor] "
#endif
#ifndef MOTOR_DEBUG_DISABLE
  #define MOTOR_DEBUG_ENABLE
#endif

class Motor {
public:
  /**
   * @param port Serial instance connected to driver
   * @param r_sense drivers sense resistor (```0.11```?)
   * @param enable drivers EN pin
   * @param direction driver DIR pin
   * @param step drivers STEP pin
   * @param home homing pin (```255``` for unused)
   * @param total total steps
   * @param debug SoftwareSerial to print debug output (or ```nullptr```)
   */
  Motor(HardwareSerial* port,
        float r_sense,
        uint8_t enable,
        uint8_t direction,
        uint8_t step,
        uint8_t home,
        uint32_t total,
        SoftwareSerial* debug = nullptr);
  ~Motor() = default;

  void begin();
  void update();
  // Blocking!
  void home(bool invert);
  void estop();

  uint32_t getRemainingSteps();
  Motor& setTargetP(double target);
  Motor& setTarget(uint32_t target);
  Motor& forceTarget(uint32_t target);
  double getTargetP();
  uint32_t getTarget();
  Motor& setPositionP(double position);
  Motor& setPosition(uint32_t position);
  double getPositionP();
  uint32_t getPosition();
  Motor& setTotalSteps(uint32_t steps);
  uint32_t getTotalSteps();
  Motor& setAcceleration(float acceleration);
  float getAcceleration();
  Motor& setSpeed(float speed);
  float getSpeed();

  Motor& setCurrent(uint16_t mA);
  uint16_t getCurrent();
  Motor& setMicrosteps(uint16_t steps);
  uint16_t getMicrosteps();
  Motor& useStealthChop(bool use);

private:
  HardwareSerial* _port;
  SoftwareSerial* s;
  TMC2208Stepper _driver;
  AccelStepper _stepper;
  uint8_t _pin_en, _pin_dir, _pin_step, _pin_home;
  uint32_t _total_steps;
};

#endif