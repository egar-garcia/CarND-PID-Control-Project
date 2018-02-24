#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki, double d_Kp, double d_Kd, double d_Ki) {

    // params: [0.3189,6.72171,0.00581], d_params: [0.0970299,0.72171,0.00072171]
    // params: [0.3189,6,0.00581], d_params: [0.0785942,0.584585,0.000584585]
    //** params: [0.39455,5.5,0.005005], d_params: [0.021095,0.191773,0.000234389]

    value = 0.0;

    prev_cte = 0.0;
    int_cte = 0.0;

    total_err = 0.0;
    total_count = 0;

    params[0] = Kp;
    params[1] = Kd;
    params[2] = Ki;

    d_params[0] = d_Kp;
    d_params[1] = d_Kd;
    d_params[2] = d_Ki;

    twiddle_sample_len = 5;
    twiddle_round = -1;
    twiddle_param_idx = 0;
    twiddle_param_step = 0;
    twiddle_count = 0;
    twiddle_total_error = 0.0;
    twiddle_best_error = 0.0;
    twiddle_error = 0.0;
}

void PID::Init(double Kp, double Kd, double Ki) {
    Init(Kp, Kd, Ki, 0.1, 1.0, 0.001);
}

void PID::UpdateError(double cte) {
    double diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;

    value = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte;
    if (value < -1.0) {
        value = -1.0;
    } else if (value > 1.0) {
        value = 1.0;
    }

    total_err += pow(cte, 2);
    total_count++;

    doTwiddle(cte);
}

double PID::TotalError() {
    return total_err / total_count;
}

void PID::doTwiddle(double cte) {
    if (twiddle_count >= twiddle_sample_len) {
        //cout << "TWIDDLE ROUND: " << twiddle_round << endl;

        if (twiddle_round < 0) {
            twiddle_best_error = twiddle_total_error / twiddle_count;
            twiddle_param_idx = 0;
            twiddle_param_step = 0;
        }

        if (twiddle_param_step == 2) {
            twiddle_error = twiddle_total_error / twiddle_count;
            //cout << "-- idx: " << twiddle_param_idx << ", step: " << twiddle_param_step << endl;
            //cout << "-- best-error: " << twiddle_best_error << endl;
            //cout << "-- ERROR: " << twiddle_error << endl;
            if (twiddle_error < twiddle_best_error) {
                twiddle_best_error = twiddle_error;
                //d_params[twiddle_param_idx] *= 1.1;
            } else {
                params[twiddle_param_idx] += d_params[twiddle_param_idx];
                //d_params[twiddle_param_idx] *= 0.9;
            }
            twiddle_param_idx = (twiddle_param_idx + 1) % 3;
            twiddle_param_step = 0;
            //cout << "** params: [" << params[0] << "," << params[1] << "," << params[2] << "]" << endl;
            //cout << "** params: [" << params[0] << "," << params[1] << "," << params[2] << "]" << ", "
            //     << "d_params: [" << d_params[0] << "," << d_params[1] << "," << d_params[2] << "]"  << endl;
        }
        if (twiddle_param_step == 1) {
            twiddle_error = twiddle_total_error / twiddle_count;
            //cout << "-- idx: " << twiddle_param_idx << ", step: " << twiddle_param_step << endl;
            //cout << "-- best-error: " << twiddle_best_error << endl;
            //cout << "-- ERROR: " << twiddle_error << endl;
            if (twiddle_error < twiddle_best_error) {
                twiddle_best_error = twiddle_error;
                //d_params[twiddle_param_idx] *= 1.1;
                twiddle_param_idx = (twiddle_param_idx + 1) % 3;
                twiddle_param_step = 0;
            } else {
                params[twiddle_param_idx] -= 2 * d_params[twiddle_param_idx];
                twiddle_param_step = 2;
            }
            //cout << "** params: [" << params[0] << "," << params[1] << "," << params[2] << "]" << ", "
            //     << "d_params: [" << d_params[0] << "," << d_params[1] << "," << d_params[2] << "]" << endl;
            //cout << "** params: [" << params[0] << "," << params[1] << "," << params[2] << "]" << endl;
        }
        if (twiddle_param_step == 0) {
            //cout << "-- idx: " << twiddle_param_idx << ", step: " << twiddle_param_step << endl;
            //cout << "-- best-error: " << twiddle_best_error << endl;
            params[twiddle_param_idx] += d_params[twiddle_param_idx];
            twiddle_param_step = 1;
            //cout << "** params: [" << params[0] << "," << params[1] << "," << params[2] << "]" << ", "
            //     << "d_params: [" << d_params[0] << "," << d_params[1] << "," << d_params[2] << "]" << endl;
            //cout << "** params: [" << params[0] << "," << params[1] << "," << params[2] << "]" << endl;
        }
        //cout << "-- BEST-ERROR: " << twiddle_best_error << endl;

        twiddle_total_error = 0.0;
        twiddle_count = 0;
        twiddle_round++;
    }

    twiddle_total_error += pow(cte, 2);
    twiddle_count++;
}
