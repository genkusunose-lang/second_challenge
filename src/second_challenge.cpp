#include "second_challenge/second_challenge.hpp"
#include <cmath>

using namespace std::chrono_literals;

SecondChallenge::SecondChallenge()
: Node("second_challenge"),
  front_distance_(0.0),
  scan_received_(false)
{
  this->declare_parameter("target_dist", 1.0);
  this->declare_parameter("velocity", 0.4);

  this->get_parameter("target_dist", stop_distance_);
  this->get_parameter("velocity", forward_speed_);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    10,
    std::bind(&SecondChallenge::scan_callback, this, std::placeholders::_1)
  );

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10
  );

  timer_ = this->create_wall_timer(
    100ms,
    std::bind(&SecondChallenge::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Second Challenge Started");
}

void SecondChallenge::timer_callback()
{
  if (!scan_received_) {
    return;
  }

  if (is_goal()) {
    run(0.0, 0.0);
  }
  else if (can_move()) {
    run(forward_speed_, 0.0);
  }
  else {
    run(0.0, 0.0);
  }

  set_cmd_val();
}

void SecondChallenge::scan_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  int center_index = static_cast<int>(
    (0.0 - msg->angle_min) / msg->angle_increment
  );

  if (center_index < 0 ||
      center_index >= static_cast<int>(msg->ranges.size()))
  {
    return;
  }

  double distance = msg->ranges[center_index];

  if (std::isnan(distance) || std::isinf(distance)) {
    return;
  }

  front_distance_ = distance;
  scan_received_ = true;

  RCLCPP_INFO(this->get_logger(),
  "front_distance = %f", front_distance_);
}

bool SecondChallenge::can_move()
{
  return front_distance_ > stop_distance_;
}

bool SecondChallenge::is_goal()
{
  return front_distance_ <= stop_distance_;
}

double SecondChallenge::calc_distance()
{
  return front_distance_;
}

void SecondChallenge::run(float velocity, float omega)
{
  cmd_.linear.x = velocity;
  cmd_.linear.y = 0.0;
  cmd_.linear.z = 0.0;

  cmd_.angular.x = 0.0;
  cmd_.angular.y = 0.0;
  cmd_.angular.z = omega;
}

void SecondChallenge::set_cmd_val()
{
  cmd_pub_->publish(cmd_);
}