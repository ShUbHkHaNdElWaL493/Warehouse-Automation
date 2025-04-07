/*
  Shubh Khandelwal
*/

#ifndef PID_H
#define PID_H

#include "actuator.h"

#ifndef PWMLimit
#define PWMLimit 255
#endif

class PID
{

  private:

  motor *Motor;

  float Kp, Ki, Kd;
  int target;
  float component_p, component_i, component_d;
  int speed_pwm;

  public:

  PID(motor *, float, float, float);

  void set_target(int);

  void implement_position_PID();
  void implement_velocity_PID();

  int get_pwm();
  void set_pwm(int);

};

#endif
