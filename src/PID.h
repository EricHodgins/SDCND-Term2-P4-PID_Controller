#ifndef PID_H
#define PID_H

#include <uWS/uWS.h>

class PID {
private:
double p_run_error;
double i_run_error;
double d_run_error;

void twiddle_next_parameter();

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


  void Twiddle();
  void Twiddle_Up();
  void Twiddle_Down();
  bool Restart(uWS::WebSocket<uWS::SERVER> ws);

};

#endif /* PID_H */
