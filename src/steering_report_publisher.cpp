#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class SteeringAnglePublisher : public rclcpp::Node
{
public:
  SteeringAnglePublisher()
  : Node("steering_angle_publisher")
  {
    publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 10, std::bind(&SteeringAnglePublisher::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    autoware_auto_vehicle_msgs::msg::SteeringReport steering_msg;
    steering_msg.stamp = this->now();
    steering_msg.steering_tire_angle = static_cast<float>(yaw);

    publisher_->publish(steering_msg);
  }

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringAnglePublisher>());
  rclcpp::shutdown();
  return 0;
}
