#ifndef NAV2_UTIL__EXTRAPOLATE_HPP
#define NAV2_UTIL__EXTRAPOLATE_HPP

#include <rclcpp/rclcpp.hpp>

namespace nav2_util
{

// This class provides a way to perform linear extrapolation into the future for time-based single-valued functions

class LinearExtrapolator {
public:
  // Constructor
  // Constructor function to initialize the class variables
  LinearExtrapolator()
  : slope(0.0), prev_data(0.0), prev_time(rclcpp::Time(0, 0, RCL_ROS_TIME))
  {};

  // Reset internal state
  // This function resets all the class variables to their initial state
  void reset() {
    slope = 0.0;
    prev_data = 0.0;
    prev_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  };

  // This function updates slope and time.
  // The slope will only be updated when time > prev_time.
  void update(double data, rclcpp::Time time) {
    if (time > prev_time) {
      slope = (data - prev_data) / (time - prev_time).seconds();
    }
    prev_data = data;
    prev_time = time;
  };

  // This function uses the previously calculated slope and data to extrapolate the value at given time. 
  // If the time less than previously recorded time, It raises an error
  double getValueAtTime(rclcpp::Time time) {
    if (time < prev_time)
    {
      throw std::runtime_error("Requested an extrapolation into the past");
    }
    
    double data = (time - prev_time).seconds() * slope + prev_data;
    return data;
  };

private:
  double slope, prev_data;
  rclcpp::Time prev_time;
};

}

#endif  // NAV2_UTIL__EXTRAPOLATE_HPP