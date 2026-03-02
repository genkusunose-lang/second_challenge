#include "second_challenge/second_challenge.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<SecondChallenge> schallenge = std::make_shared<SecondChallenge>();
  rclcpp::spin(schallenge);
  rclcpp::shutdown();

  return 0;
}
