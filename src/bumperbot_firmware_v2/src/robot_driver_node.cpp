// src/robot_driver_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "bumperbot_firmware_v2/robot_controller.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class RobotDriverNode : public rclcpp::Node {
private:
    RobotController robot_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::string serial_port_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Odometry state variables
    double x_ = 0.0;
    double y_ = 0.0; 
    double theta_ = 0.0;
    rclcpp::Time last_time_;
    
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

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        last_time_ = this->now();
        
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
	
	updateOdometry(msg->linear.x, msg->angular.z);
    }


    void updateOdometry(double linear_vel, double angular_vel) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        // Dead reckoning (basic odometry)
        x_ += linear_vel * cos(theta_) * dt;
        y_ += linear_vel * sin(theta_) * dt;
        theta_ += angular_vel * dt;
        
        // Publish transform
        publishOdometry(current_time, linear_vel, angular_vel);
        last_time_ = current_time;
    }
    
    void publishOdometry(rclcpp::Time current_time, double linear_vel, double angular_vel) {
        // TF transform
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(odom_trans);
        
        // Odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = odom_trans.transform.rotation;
        
        odom.twist.twist.linear.x = linear_vel;
        odom.twist.twist.angular.z = angular_vel;
        
        odom_pub_->publish(odom);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotDriverNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
