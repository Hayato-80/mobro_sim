#include <iostream>
#include "mobro_base/mobro_base.hpp"

Mobro::Mobro() : Node("robot_base")
{
  subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
  std::bind(&Mobro::callback, this,std::placeholders::_1));

  left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_speed",10);
  right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_speed",10);
}


void Mobro::callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  double wheel_radius = 0.2;
  double wheel_base = 0.55 * 2;

  double v = msg->linear.x;
  double w = msg ->angular.z;

  double left_vel = (v - w * wheel_base / 2) / wheel_radius;
  double right_vel = (v + w * wheel_base / 2) / wheel_radius;

  std_msgs::msg::Float64 left_msg;
  left_msg.data = left_vel;
  left_wheel_pub_->publish(left_msg);

  std_msgs::msg::Float64 right_msg;
  right_msg.data = right_vel;
  right_wheel_pub_->publish(right_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Mobro>());
  rclcpp::shutdown();
  return 0;
}
