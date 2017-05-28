#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  total_error = 0.0;

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  is_updated = false;
}

void PID::UpdateError(double cte) {
  if (!is_updated) {
    p_error = cte;
    prev_clock = std::chrono::high_resolution_clock::now();
    is_updated = true;
    return;
  }

  auto cur_clock = std::chrono::high_resolution_clock::now();
  double diff_clock = std::chrono::duration<double>(cur_clock - prev_clock).count();
  prev_clock = cur_clock;

  d_error = (cte - p_error) / diff_clock;
  i_error += cte * diff_clock;
  p_error = cte;
  total_error += cte * cte;
}

double PID::TotalError() {
  return total_error;
}

double PID::Predict() {
  double steer_value = -Kp * p_error - Kd * d_error - Ki * i_error;
  return steer_value;
}
