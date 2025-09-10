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
       
	double linear = msg->linear.x;   
	double angular = msg->angular.z; 
    
	double left_velocity = linear - (angular * 0.075);   // Estimate wheelbase for now
	double right_velocity = linear + (angular * 0.075);
    
	robot_.moveRobotVelocity(left_velocity, right_velocity);
	
        RCLCPP_DEBUG(this->get_logger(), "Sending: left=%d, right=%d", 
                    static_cast<int>(left_velocity), static_cast<int>(right_velocity));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotDriverNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
