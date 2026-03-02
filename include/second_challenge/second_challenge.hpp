#ifndef SECOND_CHALLENGE_HPP_
#define SECOND_CHALLENGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SecondChallenge : public rclcpp::Node
{
public:
  SecondChallenge();

private:
  void timer_callback();
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  bool can_move();
  bool is_goal();
  double calc_distance();

  void run(float velocity, float omega);
  void set_cmd_val();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  geometry_msgs::msg::Twist cmd_;

  double front_distance_;
  bool scan_received_;

double stop_distance_;
double forward_speed_;
};

#endif