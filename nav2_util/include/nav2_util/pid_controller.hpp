#ifndef NAV2_UTIL__PID_CONTROLLER_HPP
#define NAV2_UTIL__PID_CONTROLLER_HPP

#include <cmath>
#include <algorithm>
#include <iostream>

namespace nav2_util
{

class PidController {
public:
  // Constructor
  PidController() = default;

  // Reset internal state
  void reset(){
    prev_error_ = 0.0;
    integral_ = 0.0;
  };

  void setKp(double kp) {
    kp_ = kp;
  }

  void setKi(double ki) {
    ki_ = ki;
  }

  void setKd(double kd) {
    kd_ = kd;
  }

  void setIntegralLimit(double integral_limit) {
    integral_limit_ = integral_limit;
  }

  void setOutputLimit(double output_limit) {
    output_limit_ = output_limit;
  }

  // Compute PID output given error and time step
  double update(double error, double dt){
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    p_term_ = kp_ * error;
    i_term_ = ki_ * integral_;
    if (dt > 0) {
      d_term_ = kd_ * (error - prev_error_)/dt;
      prev_error_ = error;
    } else {
      d_term_ = 0;
    }

    double pid = p_term_ + i_term_ + d_term_;

    return std::clamp(pid, -output_limit_, output_limit_);
  };

// private:
  // PID gains
  double kp_;
  double ki_;
  double kd_;

  // Windup protection
  double integral_limit_;
  double output_limit_;

  // Internal state
  double prev_error_;
  double integral_;

  // Debug data
  double p_term_;
  double i_term_;
  double d_term_;
};

}

#endif  // NAV2_UTIL__PID_CONTROLLER_HPP
