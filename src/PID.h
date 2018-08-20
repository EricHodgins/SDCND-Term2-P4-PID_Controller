#ifndef PID_H
#define PID_H

class PID {
private:
  bool is_first_twiddle_check_complete;
  bool is_second_twiddle_check_complete;
  bool continue_running;
  bool should_twiddle_up;
  bool should_twiddle_down;
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
  double best_error;
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
  void updateTwiddleFlags();
  void twiddle_p();
  void twiddle_i();
  void twiddle_d();

  bool twiddling_p;
  bool twiddling_i;
  bool twiddling_d;
  void twiddle_next_parameter();
};

#endif /* PID_H */
