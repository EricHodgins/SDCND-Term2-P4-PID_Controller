#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

	p_error = 1.0;
	i_error = 1.0;
	d_error = 1.0;

	cte = 0.0;
	prev_cte = 0.0;

	integral_cte = 0.0;

	N_STEPS = 0;
	run_error = 0.0;
}

void PID::UpdateError(double cte) {

}

double PID::TotalError() {
	return p_error + i_error + d_error;
}

double PID::calculateSteeringValue() {
	return -Kp*cte - Kd*getDiffCTE() - Ki*getIntCTE();
}

void PID::setCTE(double cte_) {
	prev_cte = cte;
	cte = cte_;
}

double PID::getDiffCTE() {
	return cte - prev_cte;
}

double PID::getIntCTE() {
	integral_cte += cte;
	return integral_cte;
}

void PID::reset() {
	N_STEPS = 0;
	integral_cte = 0.0;
	run_error = 0.0;
}

double PID::run() {
	N_STEPS += 1;
	if (N_STEPS < 200) {
		if (N_STEPS > 100) {
			run_error += cte * cte;
		}
	}

	if (N_STEPS == 200) {
		return run_error;
	}
	return -1;
}

void PID::twiddle() {
	double tolerance = 0.2;
	double best_error = run();
	double error = 0.0;

	if (best_error < 0) return;

	if (TotalError() > tolerance) {
		Kp += p_error;
		error = run();
		if (error < 0) return;
		if (error < best_error) {
			best_error = error;
			p_error *= 1.1;
		} else {
			Kp -= 2*p_error;
			error = run();
			if (error < 0) return;
			if (error < best_error) {
				best_error = error;
				p_error *= 1.1;
			} else {
				Kp += p_error;
				p_error *= 0.9;
			}
		}
	}

	reset();
}
