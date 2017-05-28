#ifndef PID_H
#define PID_H

#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <chrono>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  bool is_updated;
  std::chrono::high_resolution_clock::time_point prev_clock;


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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Predict steering value.
  */
  double Predict();

};

#endif /* PID_H */
