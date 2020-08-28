#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>

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
   * Perform TWIDDLE to update PID Parameters.
   * @param cte The current cross track error
   */
  void Twiddle(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

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
  double Kp;
  double Ki;
  double Kd;
  
  /**
   * Parameters Used to perform TWIDDLE
   */ 
  std::vector<double> p;
  std::vector<double> dp;
  bool perform_twiddle;
  enum twiddle_for_stering_or_speed {STEER, SPEED} twiddle_on;
  double best_error;
  double tolerance;
  int index_current;
  enum operation_string {INCREASE_P, INCREASE_D, DECREASE_D} operation_to_perform;
  int number_of_iterations;
};

#endif  // PID_H