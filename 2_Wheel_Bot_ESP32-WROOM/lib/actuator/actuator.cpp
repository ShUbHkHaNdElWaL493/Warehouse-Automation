/*
  Shubh Khandelwal
*/

#include "actuator.h"

motor::motor(uint8_t encoderA, uint8_t encoderB) : encoderA(encoderA), encoderB(encoderB), pulseCount(0)
{
  pinMode(this->encoderA, INPUT_PULLUP);
  pinMode(this->encoderB, INPUT_PULLUP);
}

void motor::rotate(int speed)
{
  if (speed > 0)
  {
    rotateAntiClockwise(speed);
  } else if (speed < 0)
  {
    rotateClockwise((-1) * speed);
  } else
  {
    stop();
  }
}

void motor::changePulseCount0()
{
  if (digitalRead(this->encoderB) == LOW)
  {
    this->pulseCount++;
  } else
  {
    this->pulseCount--;
  }
}

void motor::changePulseCount1()
{
  if (digitalRead(this->encoderA) == LOW)
  {
    this->pulseCount--;
  } else
  {
    this->pulseCount++;
  }
}
  
int motor::getPosition()
{
  return this->pulseCount;
}

int motor::getSpeed()
{
  int pulseCountI = this->pulseCount;
  delay(10);
  int pulseCountF = this->pulseCount;
  return (pulseCountF - pulseCountI) * 100;
}

void MD10C::rotateAntiClockwise(int speed)
{
  digitalWrite(direction, LOW);
  analogWrite(pwm, speed);
}

void MD10C::rotateClockwise(int speed)
{
  digitalWrite(direction, HIGH);
  analogWrite(pwm, speed);
}

void MD10C::stop()
{
  analogWrite(pwm, 0);
}

MD10C::MD10C(uint8_t direction, uint8_t pwm, uint8_t encoderA, uint8_t encoderB) : direction(direction), pwm(pwm), motor(encoderA, encoderB)
{
  pinMode(this->direction, OUTPUT);
  pinMode(this->pwm, OUTPUT);
}

void ZK5AD::rotateAntiClockwise(int speed)
{
  analogWrite(motorB, 0);
  analogWrite(motorA, speed);
}

void ZK5AD::rotateClockwise(int speed)
{
  analogWrite(motorA, 0);
  analogWrite(motorB, speed);
}

void ZK5AD::stop()
{
  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
}

ZK5AD::ZK5AD(uint8_t motorA, uint8_t motorB, uint8_t encoderA, uint8_t encoderB) : motorA(motorA), motorB(motorB), motor(encoderA, encoderB)
{
  pinMode(this->motorA, OUTPUT);
  pinMode(this->motorB, OUTPUT);
}

void L298N::rotateAntiClockwise(int speed)
{
  digitalWrite(motorB, LOW);
  digitalWrite(motorA, HIGH);
  analogWrite(enable, speed);
}

void L298N::rotateClockwise(int speed)
{
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, HIGH);
  analogWrite(enable, speed);
}

void L298N::stop()
{
  analogWrite(enable, 0);
}

L298N::L298N(uint8_t enable, uint8_t motorA, uint8_t motorB, uint8_t encoderA, uint8_t encoderB) : enable(enable), motorA(motorA), motorB(motorB), motor(encoderA, encoderB)
{
  pinMode(this->enable, OUTPUT);
  pinMode(this->motorA, OUTPUT);
  pinMode(this->motorB, OUTPUT);
}

void TB6600::rotateAntiClockwise(int steps)
{
  digitalWrite(direction, LOW);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(pulse, HIGH);
    delayMicroseconds(step_duration);
    digitalWrite(pulse, LOW);
    delayMicroseconds(step_duration);
  }
}

void TB6600::rotateClockwise(int steps)
{
  digitalWrite(direction, HIGH);
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(pulse, HIGH);
    delayMicroseconds(step_duration);
    digitalWrite(pulse, LOW);
    delayMicroseconds(step_duration);
  }
}

void TB6600::stop()
{
  digitalWrite(pulse, LOW);
}

TB6600::TB6600(uint8_t direction, uint8_t pulse, int step_duration, uint8_t encoderA, uint8_t encoderB) : direction(direction), pulse(pulse), step_duration(step_duration), motor(encoderA, encoderB)
{
  pinMode(this->direction, OUTPUT);
  pinMode(this->pulse, OUTPUT);
}
