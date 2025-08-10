#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace bumperbot_motion
{

class PDMotionPlanner : public nav2_core::Controller 
{

public:
	PDMotionPlanner() = default;
	~PDMotionPlanner() = default;

	void configure(
		const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
		std::string name,
		std::shared_ptr<tf2_ros::Buffer> tf,
		std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
	) override;

	void cleanup() override;
	void activate() override;
	void deactivate() override;

	geometry_msgs::msg::TwistStamped computeVelocityCommands(
			const geometry_msgs::msg::PoseStamped & robot_pose,
			const geometry_msgs::msg::Twist & velocity, 
			nav2_core::GoalChecker * goal_checker
			) override;


	void setPlan(const nav_msgs::msg::Path & path) override;
	void setSpeedLimit(const double & speed_limit, const bool & percentage) override;


private:
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;
	
	double kp_;
	double kd_;
	double step_size_;
	double max_linear_velocity_;
	double max_angular_velocity_;
	nav_msgs::msg::Path global_plan_;

	double prev_angular_error_;
	double prev_linear_error_;
	
	rclcpp::Time last_cycle_time_;
	std::string plugin_name_;
	std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
	rclcpp::Logger logger_{rclcpp::get_logger("PDMotionPlanner")};
	rclcpp::Clock::SharedPtr clock_;
	rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
	std::shared_ptr<tf2_ros::Buffer> tf_;

	bool transformPlan(const std::string & frame);

	geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);
			
// End of PDMotionPlanner Class
};

// End of namespace
}
