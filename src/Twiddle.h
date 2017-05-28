#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <iostream>

class Twiddle {
public:
  int twiddle_param;
  std::vector<double> dp;
  double min_error;
  int max_try_road;
  bool is_initialized;
  bool is_is_initialized;
  bool valid_minus;

  Twiddle();

  virtual ~Twiddle();

  void Init(std::vector<double> dp);

  double dp_sum();

  std::vector<double> UpdateParam(double error, std::vector<double> p, int try_road);

};

#endif
