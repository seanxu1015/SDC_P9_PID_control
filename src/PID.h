#ifndef PID_H
#define PID_H

class PID {
private:
  int interval = 50;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  double Dp;
  double Di;
  double Dd;

  double totalErr;
  double bestErr;
  double tolerance;
  // steering value
  double steer;
  // counter to record steps
  int step;
  // p, i, d flag, to control which parameter to update
  int pidFlag;
  // loop flag, to determine which loop (in the python code) to run in the twiddle algorithm
  int loopFlag;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * The twiddle algorithm described in the PID course
   */
  void Twiddle();

};

#endif /* PID_H */
