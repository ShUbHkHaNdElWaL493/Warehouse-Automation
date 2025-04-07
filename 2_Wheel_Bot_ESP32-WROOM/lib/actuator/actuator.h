/*
	Shubh Khandelwal
*/

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>

class motor
{

  private:

  uint8_t encoderA, encoderB;

  volatile int pulseCount;

  virtual void rotateAntiClockwise(int) = 0;
  virtual void rotateClockwise(int) = 0;
  virtual void stop() = 0;

  public:

  motor(uint8_t, uint8_t);

  void rotate(int);

  void changePulseCount0();
  void changePulseCount1();

  int getPosition();
  int getSpeed();

};

class MD10C : public motor
{

  private:

  uint8_t direction, pwm;

  void rotateAntiClockwise(int);
  void rotateClockwise(int);
  void stop();

  public:

  MD10C(uint8_t, uint8_t, uint8_t, uint8_t);

};

class ZK5AD : public motor
{

  private:

  uint8_t motorA, motorB;

  void rotateAntiClockwise(int);
  void rotateClockwise(int);
  void stop();
  
  public:

  ZK5AD(uint8_t, uint8_t, uint8_t, uint8_t);

};

class L298N : public motor
{

  private:

  uint8_t enable, motorA, motorB;

  void rotateAntiClockwise(int);
  void rotateClockwise(int);
  void stop();

  public:

  L298N(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

};

class TB6600 : public motor
{

  private:
  
  uint8_t direction, pulse;
  int step_duration;

  void rotateAntiClockwise(int);
  void rotateClockwise(int);
  void stop();
  
  public:
  
  TB6600(uint8_t, uint8_t, int, uint8_t, uint8_t);
  
};

#endif