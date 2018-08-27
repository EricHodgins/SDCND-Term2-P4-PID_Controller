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

int TIMESTEPS = 0.0;
int N_STEPS = 200;
double ERR_TOTAL = 0.0;
double BEST_ERR = numeric_limits<double>::max();
bool INITIAL_RUN = true;
bool RESTART = false;

bool TWIDDLED_UP = false;
bool TWIDDLED_DOWN = false;
bool STILL_TWIDDLING = false;

bool AT_START_OF_TWIDDLE = true;
bool AT_FIRST_CHECK = false;
bool AT_SECOND_CHECK = false;

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0.001;
  i_error = 0.001;
  d_error = 1.0;

  p_run_error = 0.0;
  i_run_error = 0.0;
  d_run_error = 0.0;

  is_twiddle_on = true;
}

void PID::UpdateError(double cte) {
  d_run_error = cte - p_run_error;
  p_run_error = cte;
  i_run_error += cte;

  if (!is_twiddle_on) return;

  TIMESTEPS += 1;
  if (TIMESTEPS >= N_STEPS) {
    ERR_TOTAL += cte*cte;
    if (TIMESTEPS >= N_STEPS*2) {
      TIMESTEPS = 0;
      if (INITIAL_RUN) {
        cout << "INITIAL_RUN" << endl;
        BEST_ERR = ERR_TOTAL;
        INITIAL_RUN = false;
        ERR_TOTAL = 0.0;
        i_run_error = 0.0;
        return;
      }

      if (AT_START_OF_TWIDDLE) {
        cout << "AT_START_OF_TWIDDLE" << endl;
        Increase_Params();
        ERR_TOTAL = 0.0;
        RESTART = true;
        i_run_error = 0.0;
        AT_START_OF_TWIDDLE = false;
        AT_FIRST_CHECK = true;
        return;
      }

      if (AT_FIRST_CHECK) {
        cout << "AT_FIRST_CHECK" << endl;
        if (ERR_TOTAL < BEST_ERR) {
          cout << "First check: ERR_TOTAL < BEST_ERR" << ERR_TOTAL << "," << BEST_ERR << endl;
          BEST_ERR = ERR_TOTAL;
          // p_error *= 1.1;
          //i_error *= 1.1;
          d_error *= 1.1;
          ERR_TOTAL = 0.0;
          AT_START_OF_TWIDDLE = true;
          AT_FIRST_CHECK = false;
          RESTART = true;
          i_run_error = 0.0;
          return;
        } else {
          cout << "First check: ERR_TOTAL > BEST_ERR" <<  ERR_TOTAL << "," << BEST_ERR << endl;
          Decrease_Params_To_Original();
          ERR_TOTAL = 0.0;
          RESTART = true;
          i_run_error = 0.0;
          AT_SECOND_CHECK = true;
          AT_FIRST_CHECK = false;
          return;
        }
      }

      if (AT_SECOND_CHECK) {
        cout << "AT_SECOND_CHECK" << endl;
        if (ERR_TOTAL < BEST_ERR) {
          cout << "Second check: ERR_TOTAL < BEST_ERR" << ERR_TOTAL << "," << BEST_ERR << endl;
          BEST_ERR = ERR_TOTAL;
          // p_error *= 1.1;
          //i_error *= 1.1;
          d_error *= 1.1;
          AT_START_OF_TWIDDLE = true;
          AT_SECOND_CHECK = false;
          RESTART = true;
          i_run_error = 0.0;
          ERR_TOTAL = 0.0;
          return;
        } else {
          cout << "Second check: ERR_TOTAL > BEST_ERR" << ERR_TOTAL << "," << BEST_ERR << endl;
          Increase_Params();
          // p_error *= 0.9;
          //i_error *= 0.9;
          d_error *= 0.9;
          RESTART = true;
          i_run_error = 0.0;
          ERR_TOTAL = 0.0;
          AT_START_OF_TWIDDLE = true;
          AT_SECOND_CHECK = false;
          return;
        }
      }

    }
  }
}

double PID::TotalError() {
  return -Kp*p_run_error - Ki*i_run_error - Kd*d_run_error;
}

void PID::Increase_Params() {
  //Kp += p_error;
  //Ki += i_error;
  Kd += d_error;
}

void PID::Decrease_Params_To_Original() {
  //Kp -= 2*p_error;
  //Ki -= 2*i_error;
  Kd -= 2*d_error;
}

bool PID::Restart(uWS::WebSocket<uWS::SERVER> ws) {
  if (RESTART) {
    RESTART = false;
    ERR_TOTAL = 0.0;

    cout << "Kp:" << Kp << " ,Ki:" << Ki << " ,Kd:" << Kd << endl;
    cout << "p_error:" << p_error << ", i_error:" << i_error << ", d_error:" << d_error << endl;

    std::string msg = "42[\"reset\"],{}]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
  }
}
