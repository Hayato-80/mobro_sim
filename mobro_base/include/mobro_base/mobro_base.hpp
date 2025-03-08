#include <memory>
#include <rclcpp/rclcpp.hpp>

// #include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>


class Mobro : public rclcpp::Node
{
public:
  Mobro();

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
};

