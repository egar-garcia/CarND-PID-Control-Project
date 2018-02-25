#include "PID.h"
#include <math.h>
#include <iostream>

#define TWIDDLE_SAMPLE_LENGTH 100


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double d_Kp, double d_Ki, double d_Kd) {
    value = 0.0;

    prev_cte = 0.0;
    int_cte = 0.0;

    total_err = 0.0;
    total_count = 0;

    // Setting up PID parametes
    params[0] = Kp;
    params[1] = Kd;
    params[2] = Ki;

    // Setting up parameters' delta for Twiddle
    d_params[0] = d_Kp;
    d_params[1] = d_Kd;
    d_params[2] = d_Ki;

    // Initializing variables for Twiddle
    twiddle_sample_len = TWIDDLE_SAMPLE_LENGTH;
    twiddle_round = -1;
    twiddle_param_idx = 0;
    twiddle_param_step = 0;
    twiddle_count = 0;
    twiddle_total_error = 0.0;
    twiddle_best_error = 0.0;
    twiddle_error = 0.0;
}

void PID::Init(double Kp, double Ki, double Kd) {
    Init(Kp, Ki, Kd, 1.0, 1.0, 1.0);
}

void PID::UpdateError(double cte) {
    // Calculating the differential
    double diff_cte = cte - prev_cte;
    // Calculating the integral
    int_cte += cte;

    // Calculates the value according to the given parametes
    value = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte;

    // Recording total error
    total_err += pow(cte, 2);
    total_count++;

    // Recording previous CTE for next iteration
    prev_cte = cte;

    // A continum Twiddle is being performed to continuosly optimize the parametes
    doTwiddle(cte);
}

double PID::TotalError() {
    return total_err / total_count;
}

double PID::getValue() {
    return value;
}

void PID::doTwiddle(double cte) {
    // Implementaion of Twiddle optimization algorithm taking as base
    // a sample of CTE measurements of size TWIDDLE_SAMPLE_LENGTH

    if (twiddle_count >= twiddle_sample_len) {

        if (twiddle_round < 0) {
            twiddle_best_error = twiddle_total_error / twiddle_count;
            twiddle_param_idx = 0;
            twiddle_param_step = 0;
        }

        if (twiddle_param_step == 2) {
            twiddle_error = twiddle_total_error / twiddle_count;
            if (twiddle_error < twiddle_best_error) {
                twiddle_best_error = twiddle_error;
                d_params[twiddle_param_idx] *= 1.1;
            } else {
                params[twiddle_param_idx] += d_params[twiddle_param_idx];
                d_params[twiddle_param_idx] *= 0.9;
            }
            twiddle_param_idx = (twiddle_param_idx + 1) % 3;
            twiddle_param_step = 0;
        }
        if (twiddle_param_step == 1) {
            twiddle_error = twiddle_total_error / twiddle_count;
            if (twiddle_error < twiddle_best_error) {
                twiddle_best_error = twiddle_error;
                d_params[twiddle_param_idx] *= 1.1;
                twiddle_param_idx = (twiddle_param_idx + 1) % 3;
                twiddle_param_step = 0;
            } else {
                params[twiddle_param_idx] -= 2 * d_params[twiddle_param_idx];
                twiddle_param_step = 2;
            }
        }
        if (twiddle_param_step == 0) {
            params[twiddle_param_idx] += d_params[twiddle_param_idx];
            twiddle_param_step = 1;
        }

        twiddle_total_error = 0.0;
        twiddle_count = 0;
        twiddle_round++;
    }

    twiddle_total_error += pow(cte, 2);
    twiddle_count++;
}
