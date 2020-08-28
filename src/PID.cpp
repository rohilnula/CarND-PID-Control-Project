#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // PID Co-efficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // PID Errors
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  //Other variables required to peform TWIDDLE
  p = { Kp_, Kd_ , Ki_};
  dp = { 0.1 * Kp_, 0.1 * Kd_ , 0.1 * Ki_ };
  perform_twiddle = false;
  twiddle_on = STEER;
  best_error = 2.0;
  tolerance = 0.02;
  index_current = 0;
  operation_to_perform = INCREASE_P;
  number_of_iterations = 0;
  
  if (twiddle_on == SPEED) {
    best_error = 10;
    tolerance = 5;
  }
}

void PID::UpdateError(double cte) {
 
  if (perform_twiddle) {
    Twiddle(cte);
  }
  
  // Updated PID Errors
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte; 
}

void PID::Twiddle(double cte) {
  double sum_dp = dp[0] + dp[1] + dp[2]; 
  if (number_of_iterations < 1000 && sum_dp > tolerance) {
    if (operation_to_perform == INCREASE_P) {
      p[index_current] += dp[index_current];
      operation_to_perform = INCREASE_D;
      number_of_iterations += 1;
      return;
    }
    else if (operation_to_perform == INCREASE_D) {
      if (cte < best_error) {
        best_error = cte;
        dp[index_current] *= 1.1;
        operation_to_perform = INCREASE_P;
        index_current = (index_current + 1) % 3;
        number_of_iterations += 1;
        return;
      }
      else {
        p[index_current] -= 2 * dp[index_current];
        operation_to_perform = DECREASE_D;
        number_of_iterations += 1;
        return;
      }
    }
    else {
      if (cte < best_error) {
        best_error = cte;
        dp[index_current] *= 1.1;
      }
      else {
        p[index_current] += dp[index_current];
        dp[index_current] *= 0.9;
      }
      operation_to_perform = INCREASE_P;
      index_current = (index_current + 1) % 3;
      number_of_iterations += 1;
      return;
    }
  }
  else {
    std::cout << "Exceeded Max Iteration" << std::endl;
    std::cout << "Kp = " << p[0] << std::endl;
    std::cout << "Ki = " << p[2] << std::endl;
    std::cout << "Kd = " << p[1] << std::endl;
  }
}

double PID::TotalError() {
  
  // Calculate Total Error
  double total_error;
  total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
  if (perform_twiddle) {
    std::cout << "Kp = " << p[0] << std::endl;
    std::cout << "Ki = " << p[2] << std::endl;
    std::cout << "Kd = " << p[1] << std::endl;
    
    total_error = -p[0] * p_error - p[1] * d_error - p[2] * i_error;
  }
  
  return total_error;
}