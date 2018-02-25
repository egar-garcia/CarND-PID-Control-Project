#ifndef PID_H
#define PID_H

class PID {
  public:
    /**
     * Constructor
     */
    PID();

    /**
     * Destructor.
     */
    virtual ~PID();

    /**
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);

    /**
     * Initialize PID, with the delta values used for Twiddle.
     */
    void Init(double Kp, double Ki, double Kd, double d_Kp, double d_Ki, double d_Kd);

    /**
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);

    /**
     * Calculate the total PID error.
     */
    double TotalError();

    /**
     * Gets the current value calculated by the PID controller.
     */
    double getValue();

  private:
    ///* The currently calculated value of the PID controller.
    double value;

    ///* The previous CTE
    double prev_cte;
    ///* The integral CTE
    double int_cte;

    /**
     * The parametes used for the controller:
     * 0: Position
     * 1: Diferential
     * 3: integral
     */
    double params[3];

    ///* The sum of all CTEs
    double total_err;
    ///* Count for the number of CTEs
    long total_count;

    ///* Delta of paremeters used by Twiddle
    double d_params[3];

    ///* Variables used by Twiddle's algorithm
    int twiddle_sample_len;
    int twiddle_round;
    int twiddle_param_idx;
    int twiddle_param_step;
    int twiddle_count;
    double twiddle_total_error;
    double twiddle_best_error;
    double twiddle_error;

    ///* Executes Twiddle optimization of parameters
    void doTwiddle(double cte);
};

#endif /* PID_H */
