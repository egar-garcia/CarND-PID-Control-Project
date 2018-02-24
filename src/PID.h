#ifndef PID_H
#define PID_H

class PID {
public:

  double value;

  /*
  double throttle;
  double steer;
  */

  double prev_cte;
  double int_cte;

  double params[3];
  double d_params[3];

  double total_err;
  long total_count;

  int twiddle_sample_len;
  int twiddle_round;
  int twiddle_param_idx;
  int twiddle_param_step;
  int twiddle_count;
  double twiddle_total_error;
  double twiddle_best_error;
  double twiddle_error;

  /*
  double parcial_err;
  long parcial_count;
  long parcial_length;
  long parcial_number;

  double steer_param[3];
  */

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki);

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Kd, double Ki, double d_Kp, double d_Kd, double d_Ki);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  void doTwiddle(double cte);
};

#endif /* PID_H */
