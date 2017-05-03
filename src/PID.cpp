#include "PID.h"
#include <algorithm>
#include <iostream>

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
  Dp = Kp/4;
  Di = Ki/4;
  Dd = 0.1;
  totalErr = 0;
  bestErr = 1e9;
  step = 0;
  tolerance = 0.05;
  pidFlag = 0;
  loopFlag = 1;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  if (step > interval)
    totalErr += cte * cte;
  steer = -Kp * p_error - Ki * i_error - Kd * d_error;
  steer = min(max(steer, -1.0), 1.0);
  step++;
}

double PID::TotalError() {
  if (step <= interval) return 0.;
  return totalErr / (step - interval);
}

void PID::Twiddle() {
  double *tau = &Kp;
  double *tau_d = &Dp;
  // to determine which parameter to update
  if (pidFlag==1) {
    tau = &Ki;
    tau_d = &Di;
  } else if (pidFlag==2) {
    tau = &Kd;
    tau_d = &Dd;
  }
  // the first step of the circle
  if (loopFlag == step == 1) {
    *tau += *tau_d;
  }
  // if the circle hasn't finished , do nothing
  if (step <= interval * 2 && Dp + Di + Dd <= tolerance)
    return;
  // circle is finished, calculate error and set step to 0
  double err = TotalError();
  step = 0;
  // loopFlag 1 (outer loop) update
  if (err < bestErr && loopFlag==1) {
    bestErr = err;
    *tau_d *= 1.1;
    pidFlag = (pidFlag + 1) % 3;
  } else if (loopFlag == 1) {
    *tau -= 2 * (*tau_d);
    loopFlag = 2;
  }
  // loopFlag 2 (inner loop) update
  if (err < bestErr && loopFlag == 2) {
    bestErr = err;
    *tau_d *= 1.1;
    loopFlag = 1;
    pidFlag = (pidFlag + 1) % 3;
  }  else if (loopFlag == 2) {
    *tau += *tau_d;
    *tau_d *= 0.9;
    loopFlag = 1;
    pidFlag = (pidFlag + 1) % 3;
  }
}
