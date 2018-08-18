#ifndef PID_H
#define PID_H

class PID {
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

  int N_STEPS;
  double run_error;
  // Current CTE
  double cte;
  // Previous CTE
  double prev_cte;
  // Integral CTE
  double integral_cte;

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


  double getDiffCTE();
  double getIntCTE();
  void setCTE(double cte_);
  void reset();
  double calculateSteeringValue();
  double run();
  void twiddle();
};

#endif /* PID_H */
