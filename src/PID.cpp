#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;

    /*
    throttle = 0.1;
    steer = 0.0;
    */
    value = 0.0;

    prev_cte = 0.0;
    int_cte = 0.0;

    total_err = 0.0;
    total_count = 0;

    params[0] = Kp;
    params[1] = Kd;
    params[2] = Ki;

    d_params[0] = 1.0;
    d_params[1] = 0.5;
    d_params[2] = 0.005;

    twiddle_sample_len = 100;
    twiddle_round = -1;
    twiddle_param_idx = 0;
    twiddle_param_step = 0;
    twiddle_count = 0;
    twiddle_total_error = 0.0;
    twiddle_best_error = 0.0;
    twiddle_error = 0.0;
    /*
    parcial_err = 0;
    parcial_count = 0;
    parcial_length = 100;
    parcial_number = -1;
    steer_param[0] = Kp;
    steer_param[1] = Kd;
    steer_param[2] = Ki;
    */
}

void PID::UpdateError(double cte) {
    double diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;

    //value = -Kp * cte - Kd * diff_cte - Ki * int_cte;
    value = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte;
    if (value < -1.0) {
        value = -1.0;
    } else if (value > 1.0) {
        value = 1.0;
    }

    //throttle = pow(1 - abs(steer), 4);
    /*
    if (cte > 1.0) {
        throttle = 0.0;
    } else {
        throttle = 1 - sqrt(1 - pow(1 - abs(steer), 2));
    }
    */
    /*
    throttle = 1.0 - 2.0 * abs(steer);
    if (throttle < 0.0) {
       throttle = 0.0;
    }
    */

    //throttle = 1.0 - sqrt(abs(steer)) * 0.9;
    //throttle = 0.3;
    /*
    if (abs(steer) > 0.3) {
        throttle = 0.01;
    } else {
        throttle += (1.0 - abs(steer)) * 0.01;
    }

    if (throttle > 1.0) {
        throttle = 1.0;
    } else if (throttle < 0.01) {
        throttle = 0.01;
    }
    */

    total_err += pow(cte, 2);
    total_count++;

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

        cout << "*** " << twiddle_round << " - twiddle_best_err: " << twiddle_best_error
             << ", twiddle_error: " << twiddle_error << endl;
        cout << "+++ " << twiddle_round
             << " params: [" << params[0] << "," << params[1] << "," << params[2] << "]"
             << ", d_params: [" << d_params[0] << "," << d_params[1] << "," << d_params[2] << "]"
             << "twiddle_param_idx: " << twiddle_param_idx << ", steep: " << twiddle_param_step << endl;

        twiddle_total_error = 0.0;
        twiddle_count = 0;
        twiddle_round++;
    }

    twiddle_total_error += pow(cte, 2);
    twiddle_count++;

    /*
    twiddle_param_idx = 0;
    twiddle_param_second_chance = false;
    //twiddle_count = 0;
    //twiddle_best_error = 0.0;
    twiddle_error = 0.0;
    */
}

double PID::TotalError() {
    return total_err / total_count;
}
