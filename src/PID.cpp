#include "PID.h"
#include <iostream>


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p = vector<double>();
  p.push_back(Kp_);
  p.push_back(Kd_);
  p.push_back(Ki_);
  cte_errors = vector<double>();

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  cte_errors.push_back(cte);

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double cte_sum = 0.0;
  for(int i = 0; i < cte_errors.size(); ++i){
    cte_sum += cte_errors[i];
  }
  return cte_sum;  // TODO: Add your total error calc here!
}

double PID::nextSteeringAngle(double cte, double prev_cte, double int_cte){
  return -p[0] * cte - p[1]*(cte - prev_cte) - p[2] * int_cte;
}

void PID::shiftTwiddleParameter(){
  current_twiddle_parameter += 1;
  if(current_twiddle_parameter == p.size()){
    current_twiddle_parameter = 0;
  }
}

void PID::updateTwiddle(double err){
  std::cout << "T_p: " << p[0] << " T_d: " << p[1] << " T_i: " << p[2] << std::endl;
  if(twiddle_state == -1){
    best_err = err;
    twiddle_state = 0;
  }
  if(twiddle_converged){
    std::cout << "Twiddle already converged" << std::endl;
    return;
  }
  if((dp[0] + dp[1] + dp[2]) < twiddle_tolerance){
    std::cout << "Twiddle is converged" << std::endl;
    twiddle_converged = true;
  }
  // Initial state, we want to update the parameter
  if(twiddle_state == 0){

    p[current_twiddle_parameter] += dp[current_twiddle_parameter];
    twiddle_state = 1;
    return;
  }
  // We get feedback from the update -> check if error is smaller or not
  else if(twiddle_state == 1){
    if(err < best_err){
      best_err = err;
      dp[current_twiddle_parameter] *= 1.1;
      shiftTwiddleParameter();
      p[current_twiddle_parameter] += dp[current_twiddle_parameter];
      twiddle_state = 1;
      return;
    }
    // Error was not smaller -> We update PID parameter and run the track again.
    else{
      p[current_twiddle_parameter] -= 2 * dp[current_twiddle_parameter];
      twiddle_state = 2;
      return;
    }
  }
  // Error was not smaller and we have gotten feedback for tuning other way
  //  --> check if error is smaller or not
  else if(twiddle_state == 2){
    if (err < best_err){
      dp[current_twiddle_parameter] *= 1.1;
      best_err = err;
      shiftTwiddleParameter();
      p[current_twiddle_parameter] += dp[current_twiddle_parameter];
      twiddle_state = 1;
      return;
    }
    // Shifting up or down was not successfull -> We reduce dp and go to next paramter.
    else{
      p[current_twiddle_parameter] += dp[current_twiddle_parameter];
      dp[current_twiddle_parameter] *= 0.9;
      shiftTwiddleParameter();
      p[current_twiddle_parameter] += dp[current_twiddle_parameter];
      twiddle_state = 1;
      return;
    }
  }



}