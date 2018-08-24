#include "PID.h"
#include <limits>
#include <iostream>
#include <uWS/uWS.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

double ERROR_SUM = 0.0;
double BEST_ERROR = numeric_limits<double>::max();
int N_STEPS = 0;
bool FINISHED_RUN = false;
bool STILL_TWIDDLING = false;
bool RESTART_CAR = false;
bool IS_DONE_FIRST_RUN = false;
bool IS_DONE_SECOND_RUN = false;
bool IS_DONE_THIRD_RUN = false;

bool TWIDDLING_P = true;
bool TWIDDLING_D = true;
bool TWIDDLING_I = false;

bool IS_INITIAL_RUN_DONE = false;
bool WITHIN_TOLERANCE = false;

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0.001;
  i_error = 0.001;
  d_error = 0.001;

  p_run_error = 0.0;
  i_run_error = 0.0;
  d_run_error = 0.0;
}

void PID::UpdateError(double cte) {
  // d_error = cte - p_error;
  // p_error = cte;
  // i_error += cte;

  d_run_error = cte - p_run_error;
  p_run_error = cte;
  i_run_error += cte;

  N_STEPS += 1;
  if (N_STEPS > 100) {
    ERROR_SUM += cte*cte;
    if (N_STEPS > 200) {
      ERROR_SUM = ERROR_SUM / 100;
      N_STEPS = 0;

      if (!IS_INITIAL_RUN_DONE) {
        IS_INITIAL_RUN_DONE = true;
        BEST_ERROR = ERROR_SUM;
        ERROR_SUM = 0.0;
        i_run_error = 0.0;
        Twiddle();
      } else {
        i_run_error = 0.0;
        Twiddle();
      }

    }
  }

}

double PID::TotalError() {
  double steeringValue = (-Kp*p_run_error) - (Kd*d_run_error) - (Ki*i_run_error);
  return steeringValue;
}

void PID::Twiddle() {

  if (IS_DONE_FIRST_RUN == false) {
    cout << "FIRST RUN..." << ERROR_SUM << ", " << BEST_ERROR << endl;
    //if (TWIDDLING_P) Kp += p_error;
    if (TWIDDLING_D) Kd += d_error;
    //if (TWIDDLING_I) Ki += i_error;
    IS_DONE_FIRST_RUN = true;
    RESTART_CAR = true;
    ERROR_SUM = 0.0;
    return;
  }

  if (IS_DONE_SECOND_RUN == false) {
    cout << "SECOND RUN..." << ERROR_SUM << ", " << BEST_ERROR << endl;
    if (ERROR_SUM < BEST_ERROR) {
      BEST_ERROR = ERROR_SUM;
      Twiddle_Up();
      twiddle_next_parameter();
      ERROR_SUM = 0.0;
      return;
    } else {
      //if (TWIDDLING_P) Kp -= 2*p_error;
      if (TWIDDLING_D) Kd -= 2*d_error;
      //if (TWIDDLING_I) Ki -= 2*i_error;
      IS_DONE_SECOND_RUN = true;
      RESTART_CAR = true;
      ERROR_SUM = 0.0;
      return;
    }
  }

  if (IS_DONE_THIRD_RUN == false) {
    cout << "THIRD RUN..." << ERROR_SUM << ", " << BEST_ERROR << endl;
    IS_DONE_THIRD_RUN = false;
    IS_DONE_SECOND_RUN = false;

    if (ERROR_SUM < BEST_ERROR) {
      BEST_ERROR = ERROR_SUM;
      Twiddle_Up();
      twiddle_next_parameter();
    } else {
      Twiddle_Down();
      twiddle_next_parameter();
    }

    ERROR_SUM = 0.0;
    return;
  }

}

void PID::Twiddle_Up() {
  //if (TWIDDLING_P) p_error *= 1.1;
  if (TWIDDLING_D) d_error *= 1.1;
  //if (TWIDDLING_I) i_error *= 1.1;
}

void PID::Twiddle_Down() {
  // if (TWIDDLING_P) {
  //   Kp += p_error;
  //   p_error *= 0.9;
  // }
  if (TWIDDLING_D) {
    Kd += d_error;
    d_error *= 0.9;
  }
  if (TWIDDLING_I) {
    Ki += i_error;
    i_error *= 0.9;
  }
}

void PID::twiddle_next_parameter() {
  cout << "Just Twiddled:" << TWIDDLING_P << TWIDDLING_D << TWIDDLING_I << endl;
  if (TWIDDLING_P) {
    TWIDDLING_P = true;
    TWIDDLING_D = true;
    Kd += d_error;
    return;
  }

  return;



  if (TWIDDLING_D) {
    TWIDDLING_D = false;
    TWIDDLING_I = true;
    Ki += i_error;
    return;
  }

  if (TWIDDLING_I) {
    TWIDDLING_I = false;
    TWIDDLING_P = true;
    Kp += p_error;
  }
}


bool PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  if (RESTART_CAR == true) {
    std::string msg = "42[\"reset\"],{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    RESTART_CAR = false;
    p_run_error = 0.001;
    i_run_error = 0.001;
    d_run_error = 0.001;

    cout << "Kp:" << Kp << ", Ki:" << Ki << ", Kd:" << Kd << endl;
    cout << "p_error:" << p_error << ", i_error:" << i_error << ", d_error:" << d_error << endl;

    return true;
  }

  return false;
}
