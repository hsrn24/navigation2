#ifndef INTERPOLATE_HPP
#define INTERPOLATE_HPP

#include <rclcpp/rclcpp.hpp>

namespace nav2_util
{

class Interpolate {
public:
  // Constructor
  Interpolate()
  : slope(0.0), prev_data(0.0), prev_time(rclcpp::Time(0, 0, RCL_ROS_TIME))
  {};

  // Reset internal state
  void reset() {
    slope = 0.0;
    prev_data = 0.0;
    prev_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  };

  void update(double data, rclcpp::Time time) {
    if (time > prev_time) {
      slope = (data - prev_data) / (time - prev_time).seconds();
    }
    prev_data = data;
    prev_time = time;
  };

  double getValueAtTime(rclcpp::Time time) {
    double data = 0.0;

    if (time > prev_time) {
      data = (time - prev_time).seconds() * slope + prev_data;
    }

    return data;
  };

private:
  double slope, prev_data;
  rclcpp::Time prev_time;
};

}

#endif  // INTERPOLATE_HPP
