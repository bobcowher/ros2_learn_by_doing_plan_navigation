#include "bumperbot_planning/dijkstra_planner.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rmw/types.h>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <variant>
#include <vector>
#include "nav_msgs/msg/path.hpp"
#include "rmw/qos_profiles.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <queue>

namespace bumperbot_planning
{
	
void DijkstraPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, 
	       std::string name, 
	       std::shared_ptr<tf2_ros::Buffer> tf, 
	       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) 
{
	node_ = parent.lock();
	name_ = name;
	tf_ = tf;
	costmap_ = costmap_ros->getCostmap();
	global_frame_ = costmap_ros->getGlobalFrameID();
}

void DijkstraPlanner::cleanup()
{
	RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type DijkstraPlanner", name_.c_str());	
}

void DijkstraPlanner::activate()
{
	RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type DijkstraPlanner", name_.c_str());	
}

void DijkstraPlanner::deactivate()
{
	RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type DijkstraPlanner", name_.c_str());	
}

GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
	int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
	int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
	return GraphNode(grid_x, grid_y);
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start,
		   	       			const geometry_msgs::msg::PoseStamped & goal,
		                         	std::function<bool()>) 
{
	std::vector<std::pair<int, int>> explore_directions = {
		{-1, 0}, {1, 0}, {0, -1}, {0, 1}
	};

	std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;

	std::vector<GraphNode> visited_nodes;

	pending_nodes.push(worldToGrid(start.pose));

	GraphNode active_node;

	while(!pending_nodes.empty() && rclcpp::ok()){
		active_node = pending_nodes.top();
		pending_nodes.pop();

		if(worldToGrid(goal.pose) == active_node){
			break;
		}

		for(const auto & dir : explore_directions){
			GraphNode new_node = active_node + dir;
			double new_node_current_cost = costmap_->getCost(new_node.x, new_node.y);
			if(std::find(visited_nodes.begin(), 
				     visited_nodes.end(), 
				     new_node) == visited_nodes.end() &&
				     poseOnMap(new_node) &&
				     new_node_current_cost < 99 &&
				     new_node_current_cost >= 0)
			{
				new_node.cost = active_node.cost + 1 + new_node_current_cost;
				new_node.prev = std::make_shared<GraphNode>(active_node);
				pending_nodes.push(new_node);
				visited_nodes.push_back(new_node);
			}
		}
	}
	
	nav_msgs::msg::Path path;
	path.header.frame_id = global_frame_;
	
	while(active_node.prev && rclcpp::ok()){
		geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
		geometry_msgs::msg::PoseStamped last_pose_stamped;
		last_pose_stamped.header.frame_id = global_frame_;
		last_pose_stamped.pose = last_pose;
		path.poses.push_back(last_pose_stamped);
		active_node = *active_node.prev;
	}

	std::reverse(path.poses.begin(), path.poses.end());
	return path;

}

bool DijkstraPlanner::poseOnMap(const GraphNode & node) 
{
	return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) &&
		node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
{
	return node.y * costmap_->getSizeInCellsX() + node.x;
}

geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node)
{
	geometry_msgs::msg::Pose pose;
	pose.position.x = node.x * costmap_->getResolution() + costmap_->getOriginX();
	pose.position.y = node.y * costmap_->getResolution() + costmap_->getOriginY();
	
	return pose;
}

// End of namespace
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_planning::DijkstraPlanner, nav2_core::GlobalPlanner)


