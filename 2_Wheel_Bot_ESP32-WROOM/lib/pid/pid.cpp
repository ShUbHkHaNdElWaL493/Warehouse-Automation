/*
  Shubh Khandelwal
*/

#include "pid.h"

PID::PID(motor *Motor, float Kp, float Ki, float Kd) : Motor(Motor), Kp(Kp), Ki(Ki), Kd(Kd), target(0), speed_pwm(0), component_p(0), component_i(0), component_d(0)
{}

void PID::set_target(int target)
{
  this->target = target;
  this->component_p = 0;
  this->component_i = 0;
  this->component_d = 0;
}

void PID::implement_position_PID()
{
  this->component_d = this->component_p;
  this->component_p = this->component_i;
  this->component_i = this->target - this->Motor->getPosition();
  this->component_p = this->component_i - this->component_p;
  this->component_d = this->component_p - this->component_d;
  this->speed_pwm += Kp * this->component_p + Ki * this->component_i + Kd * this->component_d;
  if (speed_pwm < -(PWMLimit))
  {
    speed_pwm = -(PWMLimit);
  }
  if (speed_pwm > PWMLimit)
  {
    speed_pwm = PWMLimit;
  }
}

void PID::implement_velocity_PID()
{
  this->component_d = this->component_p;
  this->component_p = this->target - this->Motor->getSpeed();
  this->component_i += this->component_p;
  this->component_d = this->component_p - this->component_d;
  this->speed_pwm += Kp * this->component_p + Ki * this->component_i + Kd * this->component_d;
  if (speed_pwm < -(PWMLimit))
  {
    speed_pwm = -(PWMLimit);
  }
  if (speed_pwm > PWMLimit)
  {
    speed_pwm = PWMLimit;
  }
}

int PID::get_pwm()
{
  return this->speed_pwm;
}

void PID::get_pwm(int speed_pwm)
{
  this->speed_pwm = speed_pwm;
}
