#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"

class VehicleControlNode : public rclcpp::Node
{
public:
    VehicleControlNode()
    : Node("vehicle_control_node"), speed_scale_(1.0), steering_angle_scale_(1.0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/tugbot/cmd_vel", 10);
        subscription_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 10, std::bind(&VehicleControlNode::control_command_callback, this, std::placeholders::_1));

        // Declare parameters
        this->declare_parameter<double>("speed_scale", 1.0);
        this->declare_parameter<double>("steering_angle_scale", 1.0);
    }

private:
    void control_command_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
    {
        // Get parameters
        this->get_parameter("speed_scale", speed_scale_);
        this->get_parameter("steering_angle_scale", steering_angle_scale_);

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = speed_scale_ * msg->longitudinal.speed; // Apply speed scale
        twist_msg.angular.z = steering_angle_scale_ * msg->lateral.steering_tire_angle; // Apply steering angle scale

        publisher_->publish(twist_msg);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr subscription_;
    double speed_scale_;
    double steering_angle_scale_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleControlNode>());
    rclcpp::shutdown();
    return 0;
}
