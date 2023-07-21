#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"

class SteeringAnglePublisher : public rclcpp::Node
{
public:
  SteeringAnglePublisher()
  : Node("steering_angle_publisher")
  {
    publisher_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&SteeringAnglePublisher::publish_steering_status, this));
  }

private:
  void publish_steering_status()
  {
    autoware_auto_vehicle_msgs::msg::SteeringReport steering_msg;
    steering_msg.stamp = this->now();
    steering_msg.steering_tire_angle = 0.0f;

    publisher_->publish(steering_msg);
  }

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringAnglePublisher>());
  rclcpp::shutdown();
  return 0;
}
