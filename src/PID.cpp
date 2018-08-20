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
	best_error = 1000;

	is_first_twiddle_check_complete = false;
	is_second_twiddle_check_complete = false;
	continue_running = false;
    should_twiddle_up = true;
 	should_twiddle_down = false;

 	twiddling_p = true;
  	twiddling_i = false;
 	twiddling_d = false;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

	if (N_STEPS > 100) {
		run_error += cte*cte;
		if (N_STEPS > 200) {
			if (twiddling_p) Kp += p_error;
			if (twiddling_d) Kd += d_error;
			if (twiddling_i) Ki += i_error;
			N_STEPS = 0;
			updateTwiddleFlags();
		}
	}
	N_STEPS += 1;
}

double PID::TotalError() {
	return run_error / 100;
}

double PID::calculateSteeringValue() {
	return -Kp*p_error - Kd*d_error - Ki*i_error;
}

void PID::twiddle() {
	double tolerance = 0.2;
	double error_sum = p_error + i_error + d_error;
	if (error_sum < tolerance) return;

	// Flag - Check First Error is complete
	if (!is_first_twiddle_check_complete) return;

	if (should_twiddle_up) {
		if (run_error < best_error) {
			best_error = run_error;
			if (twiddling_p) p_error *= 1.1;
			if (twiddling_d) d_error *= 1.1;
			if (twiddling_i) i_error *= 1.1;
			should_twiddle_up = true;
			should_twiddle_down = false;
			continue_running = false;
		} else {
			if (twiddling_p) Kp -= 2*p_error;
			if (twiddling_d) Kd -= 2*d_error;
			if (twiddling_i) Ki -= 2*i_error;
			should_twiddle_up = false;
			should_twiddle_down = true;
			continue_running = true;
		}
	} else if (should_twiddle_down) {
		if (!is_second_twiddle_check_complete) return;

		if (run_error < best_error) {
			best_error = run_error;
			if (twiddling_p) Kp *= 1.1;
			if (twiddling_d) Kd *= 1.1;
			if (twiddling_i) Ki *= 1.1;
			should_twiddle_up = true;
			should_twiddle_down = false;
		} else {
			if (twiddling_p) Kp += p_error;
			if (twiddling_d) Kd += d_error;
			if (twiddling_i) Ki += i_error;
			p_error *= 0.9;
			should_twiddle_down = true;
			should_twiddle_up = false;
		}

		continue_running = false;
	}

}


void PID::updateTwiddleFlags() {
	if (is_first_twiddle_check_complete == false && continue_running == false) {
		is_first_twiddle_check_complete = true;
		is_second_twiddle_check_complete = false;
		//twiddle_next_parameter();
		return;
	}

	if (is_first_twiddle_check_complete == true && continue_running == true) {
		is_second_twiddle_check_complete = true;
		is_first_twiddle_check_complete = true;
		twiddle_next_parameter();
		return;
	}
}

void PID::twiddle_next_parameter() {
	if (twiddling_p) {
		twiddling_p = false;
		twiddling_d = true;
	}

	if (twiddling_d) {
		twiddling_d = false;
		twiddling_i = true;
	}

	if (twiddling_i) {
		twiddling_i = false;
		twiddling_p = true;
	}
}


