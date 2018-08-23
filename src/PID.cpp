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
bool TWIDDLING_D = false;
bool TWIDDLING_I = false;
bool CTE_TOO_LARGE = false;

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 1.0;
  i_error = 1.0;
  d_error = 1.0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  N_STEPS += 1;
  if (N_STEPS > 500) {
    ERROR_SUM += cte*cte;
    if (N_STEPS > 1000) {
      ERROR_SUM = ERROR_SUM / 100;
      N_STEPS = 0;
      Twiddle();
    }
  }

}

double PID::TotalError() {
  cout << "i_error = " << i_error << endl;
  double steeringValue = -Kp*p_error - Kd*d_error - Ki*i_error;
  if (steeringValue < -1) return -1;
  if (steeringValue > 1) return 1;
  return steeringValue;
}

void PID::Twiddle() {
  double total_error = p_error + d_error;
  if (fabs(total_error) < 0.002) return;

  if (IS_DONE_FIRST_RUN == false) {
    cout << "FIRST RUN..." << ERROR_SUM << ", " << BEST_ERROR << endl;
    if (TWIDDLING_P) Kp += p_error;
    if (TWIDDLING_D) Kd += d_error;
    if (TWIDDLING_I) Ki += i_error;
    IS_DONE_FIRST_RUN = true;
    RESTART_CAR = true;
    i_error = 0.0;
    ERROR_SUM = 0.0;
    return;
  }

  if (IS_DONE_SECOND_RUN == false) {
    cout << "SECOND RUN..." << ERROR_SUM << ", " << BEST_ERROR << endl;
    if (ERROR_SUM < BEST_ERROR) {
      BEST_ERROR = ERROR_SUM;
      if (TWIDDLING_P) p_error *= 1.1;
      if (TWIDDLING_D) d_error *= 1.1;
      if (TWIDDLING_I) i_error *= 1.1;
      twiddle_next_parameter();
      IS_DONE_FIRST_RUN = false;
      i_error = 0.0;
      ERROR_SUM = 0.0;
      return;
    } else {
      if (TWIDDLING_P) Kp -= 2*p_error;
      if (TWIDDLING_D) Kd -= 2*d_error;
      if (TWIDDLING_I) Ki -= 2*i_error;
      IS_DONE_SECOND_RUN = true;
      FINISHED_RUN = false;
      RESTART_CAR = true;
      i_error = 0.0;
      ERROR_SUM = 0.0;
      return;
    }
  }

  if (IS_DONE_THIRD_RUN == false) {
    cout << "THIRD RUN..." << ERROR_SUM << ", " << BEST_ERROR << endl;
    IS_DONE_THIRD_RUN = false;
    IS_DONE_SECOND_RUN = false;
    IS_DONE_FIRST_RUN = false;
    if (ERROR_SUM < BEST_ERROR) {
      BEST_ERROR = ERROR_SUM;
      if (TWIDDLING_P) p_error *= 1.1;
      if (TWIDDLING_D) d_error *= 1.1;
      if (TWIDDLING_I) i_error *= 1.1;
      twiddle_next_parameter();
    } else {
      if (TWIDDLING_P) {
        Kp += p_error;
        p_error *= 0.9;
      }
      if (TWIDDLING_D) {
        Kd += d_error;
        d_error *= 0.9;
      }
      if (TWIDDLING_I) {
        Ki += i_error;
        i_error *= 0.9;
      }
      twiddle_next_parameter();
    }
    i_error = 0.0;
    ERROR_SUM = 0.0;
    return;
  }

}

void PID::twiddle_next_parameter() {
  cout << "Just Twiddled:" << TWIDDLING_P << TWIDDLING_D << TWIDDLING_I << endl;
  if (TWIDDLING_P) {
    TWIDDLING_P = false;
    TWIDDLING_D = true;
    return;
  }

  if (TWIDDLING_D) {
    TWIDDLING_D = false;
    TWIDDLING_I = true;
    return;
  }

  if (TWIDDLING_I) {
    TWIDDLING_I = false;
    TWIDDLING_P = true;
  }
}


bool PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  if (RESTART_CAR == true) {
    std::string msg = "42[\"reset\"],{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    RESTART_CAR = false;

    cout << "Kp:" << Kp << ", Ki:" << Ki << ", Kd:" << Kd << endl;
    cout << "p_error:" << p_error << ", i_error:" << i_error << ", d_error:" << d_error << endl;

    return true;
  }

  return false;
}