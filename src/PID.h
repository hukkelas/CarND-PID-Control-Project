#ifndef PID_H
#define PID_H
#include <vector>

using std::vector;

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
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Updates the twiddle optimizer
   */
  void updateTwiddle(double err);

  /**
   * Calculates the predicted steering angle
   */
  double nextSteeringAngle(double cte, double prev_cte, double int_cte);
  double best_err;
  bool twiddle_converged = true;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  vector<double> p;

  vector<double> cte_errors;

  // Twiddle parameters
  
  int twiddle_state = -1;
  int current_twiddle_parameter = 0;
  
  vector<double> dp = {1, 1, .0001};
  double twiddle_tolerance = 0.001;

  void shiftTwiddleParameter();

};

#endif  // PID_H