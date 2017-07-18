#ifndef PID_H
#define PID_H

enum ControlType {
  P, Pd, PId
};

enum TwiddleDirection {
  UP, DOWN
};

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double ps[3];
  double dps[3];

  double iterator;
  double sum_sq_error;
  double best_error;

  double prev_cte;
  int current_pos;
  ControlType controlType;
  TwiddleDirection twiddle_direction;
  bool first;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, ControlType controlType);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();

  double TwiddleError();
};

#endif /* PID_H */
