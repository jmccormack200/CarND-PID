#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, ControlType controlType) {
  p_error  = 0;
  i_error  = 0;
  d_error  = 0;
  prev_cte = 0;

  ps[0] = Kp;
  ps[1] = Ki;
  ps[2] = Kd;

  dps[0] = 1;
  dps[1] = 1;
  dps[2] = 1;

  sum_sq_error = 0;
  best_error = 1e100;
  current_pos = 0;
  iterator = 0;
  twiddle_direction = UP;

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->controlType = controlType;

  first = true;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error = cte + i_error;
  d_error = cte - prev_cte;
  prev_cte = cte;

  sum_sq_error = cte * cte;
  iterator++;
}

double PID::TotalError() {
  Kp = ps[0];
  Ki = ps[1];
  Kd = ps[2];
  if (controlType == P) {
    return -Kp * p_error;
  } else if (controlType == Pd) {
    return -Kp * p_error - Kd * d_error;
  } else {
    return -Kp * p_error - Kd * d_error - Ki * i_error;
  }
}

void PID::Twiddle() {

  double mean_sq_error = sum_sq_error / iterator;
  sum_sq_error = 0;
  iterator = 0;

  cout<<"--------------------------------"<<endl;
  cout << "parameter: " << current_pos << endl;
  cout << "direction: " << twiddle_direction << endl;
  cout << "New Params: (ps) P: " << ps[0] << " I: " << ps[1] << " D: " << ps[2] << endl;
  cout << "New Params: (Ks) Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
  cout << "Errors: " << dps[0] << " " << dps[1] << " " << dps[2] << endl;
  cout << " Mean Squre Error: " << mean_sq_error << endl;
  cout << " BEST ERROR " << best_error << endl;

  if (first) {
    best_error = mean_sq_error;
    ps[0] += dps[0];
    first = false;
    return;
  }

  if (mean_sq_error < best_error) {
    best_error = mean_sq_error;
    dps[current_pos] *= 1.1;
    cout<<"*** NEW BEST ***"<<endl;
  } else {
    if (twiddle_direction == UP) {
      twiddle_direction = DOWN;
      ps[current_pos] -= 2 * dps[current_pos];
      return;
    } else {
      ps[current_pos] += dps[current_pos];
      dps[current_pos] *= 0.9;
    }
  }

  current_pos = (current_pos + 1) % 3;
  twiddle_direction = UP;
  ps[current_pos] += dps[current_pos];

  cout<<"--------------------------------"<<endl;
}

double PID::TwiddleError() {
  return ps[0]/dps[0] + ps[1]/dps[1] + ps[2]/dps[2];
}
