// src/robot_driver_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "bumperbot_firmware_v2/robot_controller.hpp"

class RobotDriverNode : public rclcpp::Node {
private:
    RobotController robot_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::string serial_port_;
    
public:
    RobotDriverNode() : Node("robot_driver") {
        // Declare parameter for serial port
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        serial_port_ = this->get_parameter("serial_port").as_string();
        
        // Connect to Arduino
        if (!robot_.connect(serial_port_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to robot on %s", serial_port_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Robot connected on %s", serial_port_.c_str());
        
        // Subscribe to velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, 
            std::bind(&RobotDriverNode::cmd_vel_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Robot driver node started");
    }
    
    ~RobotDriverNode() {
        robot_.stopRobot();
        robot_.disconnect();
    }
    
private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Convert ROS Twist to motor speeds
        double linear = msg->linear.x;   // m/s forward/backward
        double angular = msg->angular.z; // rad/s rotation
        
        // Simple differential drive conversion
        // You'll need to tune these scale factors for your robot
        double left_speed = (linear - angular * 0.5) * 100.0;  // Scale to motor range
        double right_speed = (linear + angular * 0.5) * 100.0;
        
        // Send to robot
        robot_.moveRobot(static_cast<int>(left_speed), static_cast<int>(right_speed));
        
        RCLCPP_DEBUG(this->get_logger(), "Sending: left=%d, right=%d", 
                    static_cast<int>(left_speed), static_cast<int>(right_speed));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotDriverNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
