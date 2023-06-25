#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

class VehicleControlNode : public rclcpp::Node
{
public:
    VehicleControlNode()
    : Node("vehicle_control_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/tugbot/cmd_vel", 10);
        subscription_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 10, std::bind(&VehicleControlNode::control_command_callback, this, std::placeholders::_1));
    }

private:
    void control_command_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = msg->longitudinal.speed;
        twist_msg.angular.z = msg->lateral.steering_tire_angle;

        publisher_->publish(twist_msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleControlNode>());
    rclcpp::shutdown();
    return 0;
}
