#include "Twiddle.h"

using namespace std;

Twiddle::Twiddle(){};

Twiddle::~Twiddle() {};

void Twiddle::Init(std::vector<double> dp) {
  this->dp = dp;
  min_error = 100000000000.0;
  max_try_road = 0;
  twiddle_param = 0;
  is_initialized = false;
  is_is_initialized = false;
  valid_minus = false;
}

double Twiddle::dp_sum(){
  return dp[0] + dp[1] + dp[2];
}


std::vector<double> Twiddle::UpdateParam(double error, std::vector<double> p, int try_road){
  bool is_error = ((error < min_error) & (try_road == max_try_road));

  if (!is_is_initialized) {
    max_try_road = try_road;
    min_error = error;
    is_is_initialized = true;
    return p;
  }

  if (!is_initialized) {
    p[twiddle_param] += dp[twiddle_param];
    is_initialized = true;
    return p;
  }

  if ((error < min_error) & ((try_road + 200) > max_try_road)){
    std::cout << "Update!!" << std::endl;
    min_error = error;
    dp[twiddle_param] *= 1.0;
  } else {
    if (!valid_minus) {
      p[twiddle_param] -= 2 * dp[twiddle_param];
      valid_minus = true;
      return p;
    }
    p[twiddle_param] += dp[twiddle_param];
    dp[twiddle_param] *= 0.4;
    valid_minus = false;
  }

  twiddle_param += 1;
  if (twiddle_param == p.size()) {
    twiddle_param = 0;
  }
  is_initialized = false;
  return p;
}


// std::vector<double> Twiddle::UpdateParam(double error, std::vector<double> p, int try_road){
//   bool is_error = ((error < min_error) & (try_road == max_try_road));
//
//   if (!is_is_initialized) {
//     max_try_road = try_road;
//     min_error = error;
//     is_is_initialized = true;
//     return p;
//   }
//
//   if (!is_initialized) {
//     p[twiddle_param] += dp[twiddle_param];
//     // if ((try_road > max_try_road) || is_error) {
//     //   max_try_road = try_road;
//     //   min_error = error;
//     // }
//     is_initialized = true;
//     return p;
//   }
//
//   if (try_road > max_try_road) {
//     min_error = error;
//     dp[twiddle_param] *= 1.1;
//     max_try_road = try_road;
//   } else if (is_error) {
//     min_error = error;
//     dp[twiddle_param] *= 1.1;
//   } else {
//     if (!valid_minus) {
//       p[twiddle_param] -= 2 * dp[twiddle_param];
//       valid_minus = true;
//       return p;
//     }
//     p[twiddle_param] += dp[twiddle_param];
//     dp[twiddle_param] *= 0.6;
//     valid_minus = false;
//   }
//
//   twiddle_param += 1;
//   if (twiddle_param == p.size()) {
//     twiddle_param = 0;
//   }
//   is_initialized = false;
//   return p;
// }
