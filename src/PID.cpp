#include "PID.h"
#include <math.h>

//using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this -> Kp = Kp;
    this -> Ki = Ki;
    this -> Kd = Kd;

    throttle = 0.1;
    steer = 0.0;
    prev_cte = 0.0;
    int_cte = 0.0;

    total_err = 0.0;
    total_count = 0;
}

void PID::UpdateError(double cte) {
    double diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;
    steer = -Kp * cte - Kd * diff_cte - Ki * int_cte;
    if (steer < -1.0) {
        steer = -1.0;
    } else if (steer > 1.0) {
        steer = 1.0;
    }

    //throttle = 1.0 - sqrt(abs(steer)) * 0.9;
    throttle = 0.3;
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
}

double PID::TotalError() {
    return total_err / total_count;
}
