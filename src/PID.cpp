#include "PID.h"
#include <limits>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

double ERROR_SUM = 0.0;
double BEST_ERROR = numeric_limits<double>::max();
int N_STEPS = 0;
bool STILL_TWIDDLING = false;
bool TWIDDLING_P = true;
bool TWIDDLING_D = false;
bool TWIDDLING_I = false;

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
	if (N_STEPS > 100) {
		ERROR_SUM += cte*cte;
		if (N_STEPS > 200) {
			N_STEPS = 0;
			ERROR_SUM = ERROR_SUM / 100;

			if (BEST_ERROR == numeric_limits<double>::max()) {
				BEST_ERROR = ERROR_SUM;
				return;
			}

			if (STILL_TWIDDLING == true) {
				if (ERROR_SUM < BEST_ERROR) {
					BEST_ERROR = ERROR_SUM;
					if (TWIDDLING_P) p_error *= 1.1;
					if (TWIDDLING_D) d_error *= 1.1;
					if (TWIDDLING_I) i_error *= 1.1;
					cout << "Twiddling Up - Second Check - " << TWIDDLING_P << TWIDDLING_D << TWIDDLING_I << " - " << p_error << "," << d_error << "," << i_error << endl;
				} else {
					if (TWIDDLING_P) Kp += p_error;
					if (TWIDDLING_D) Kd += d_error;
					if (TWIDDLING_I) Ki += i_error;

					if (TWIDDLING_P) p_error *= 0.9;
					if (TWIDDLING_D) d_error *= 0.9;
					if (TWIDDLING_I) i_error *= 0.9;

					cout << "Twiddling Down - " << TWIDDLING_P << TWIDDLING_D << TWIDDLING_I << " - " << p_error << "," << d_error << "," << i_error << endl;
				}

				reset();
				return;
			}

			if (STILL_TWIDDLING == false) {
				if (TWIDDLING_P) Kp += p_error;
				if (TWIDDLING_D) Kd += d_error;
				if (TWIDDLING_I) Ki += i_error;
			}

			if (ERROR_SUM < BEST_ERROR) {
				BEST_ERROR = ERROR_SUM;
				if (TWIDDLING_P) p_error *= 1.1;
				if (TWIDDLING_D) d_error *= 1.1;
				if (TWIDDLING_I) i_error *= 1.1;
				cout << "Twiddling Up - First Check - " << TWIDDLING_P << TWIDDLING_D << TWIDDLING_I << " - " << p_error << "," << d_error << "," << i_error << endl;
				reset();
			} else {
				if (TWIDDLING_P) p_error -= 2*p_error;
				if (TWIDDLING_D) d_error -= 2*d_error;
				if (TWIDDLING_I) i_error -= 2*i_error;
				STILL_TWIDDLING = true;
				cout << "STILL_TWIDDLING - " << TWIDDLING_P << TWIDDLING_D << TWIDDLING_I << " - " << p_error << "," << d_error << "," << i_error << endl;
				return;
			}
		}
	}
}

double PID::TotalError() {
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}

void PID::reset() {
	STILL_TWIDDLING = false;
	ERROR_SUM = 0.0;
	BEST_ERROR = numeric_limits<double>::max();
	i_error = 0.0;
	twiddle_next_parameter();
}

void PID::twiddle_next_parameter() {
	if (TWIDDLING_P) {
		TWIDDLING_D = true;
		TWIDDLING_P = false;
		return;
	}

	if (TWIDDLING_D) {
		TWIDDLING_I = true;
		TWIDDLING_D = false;
		return;
	}

	if (TWIDDLING_I) {
		TWIDDLING_P = true;
		TWIDDLING_I = false;
	}
}