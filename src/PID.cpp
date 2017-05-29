#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  d_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; // difference with previous cte
  p_error = cte; // current cte
  i_error += cte; // total cte
}

double PID::TotalError(double speed_factor) {
  return (-(Kp * p_error * speed_factor) - (Kd * d_error) - (Ki * i_error * (1 - speed_factor)));
}