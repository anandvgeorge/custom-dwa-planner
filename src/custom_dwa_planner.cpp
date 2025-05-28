#include "custom_dwa_planner/custom_dwa_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <cmath>
#include <algorithm>

namespace custom_dwa_planner
{

CustomDWAPlanner::CustomDWAPlanner() : node_(nullptr), plugin_name_(""), tf_buffer_(nullptr), costmap_ros_(nullptr)
{
}

void CustomDWAPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        if (!node_) {
          throw std::runtime_error("Failed to lock LifecycleNode in CustomDWAPlanner::configure");
        }
        plugin_name_ = name;
        tf_buffer_ = tf;
        costmap_ros_ = costmap_ros;
      
        // Declare and get parameters
        node_->declare_parameter(name + ".desired_linear_vel", 0.5);
        node_->declare_parameter(name + ".max_vel_y", 0.0);
        node_->declare_parameter(name + ".max_angular_vel", 1.0);
        node_->declare_parameter(name + ".lookahead_time", 1.0);
        node_->declare_parameter(name + ".linear_granularity", 0.05);
        node_->declare_parameter(name + ".angular_granularity", 0.1);
        node_->declare_parameter(name + ".goal_weight", 1.0);
        node_->declare_parameter(name + ".obstacle_weight", 1.0);
        node_->declare_parameter(name + ".speed_weight", 0.5);
        node_->declare_parameter(name + ".smoothness_weight", 0.5);
      
        max_vel_x_ = node_->get_parameter(name + ".desired_linear_vel").as_double();
        max_vel_y_ = node_->get_parameter(name + ".max_vel_y").as_double();
        max_vel_theta_ = node_->get_parameter(name + ".max_angular_vel").as_double();
        sim_time_ = node_->get_parameter(name + ".lookahead_time").as_double();
        linear_granularity_ = node_->get_parameter(name + ".linear_granularity").as_double();
        angular_granularity_ = node_->get_parameter(name + ".angular_granularity").as_double();
        goal_weight_ = node_->get_parameter(name + ".goal_weight").as_double();
        obstacle_weight_ = node_->get_parameter(name + ".obstacle_weight").as_double();
        speed_weight_ = node_->get_parameter(name + ".speed_weight").as_double();
        smoothness_weight_ = node_->get_parameter(name + ".smoothness_weight").as_double();

        selected_traj_pub_ = node_->create_publisher<nav_msgs::msg::Path>("dwa_selected_trajectory", 10);
      
        RCLCPP_INFO(node_->get_logger(), "Custom DWA Planner configured: %s", plugin_name_.c_str());
      }
      

void CustomDWAPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Custom DWA Planner activated");
}

void CustomDWAPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Custom DWA Planner deactivated");
}

void CustomDWAPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Custom DWA Planner cleaned up");
}

void CustomDWAPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
  RCLCPP_INFO(node_->get_logger(), "Custom DWA Planner received new path with %zu poses", path.poses.size());
}

geometry_msgs::msg::TwistStamped CustomDWAPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;

  // Get costmap
  auto costmap = costmap_ros_->getCostmap();
  double robot_x = pose.pose.position.x;
  double robot_y = pose.pose.position.y;
  double robot_yaw = tf2::getYaw(pose.pose.orientation);

  // Simple DWA: Sample velocities and score trajectories
  double best_cost = 50000.0;
  double best_vx = 0.0, best_vy = 0.0, best_vth = 0.0;

  for (double vx = 0; vx <= max_vel_x_; vx += linear_granularity_) {
    for (double vth = -max_vel_theta_; vth <= max_vel_theta_; vth += angular_granularity_) {
      // Simulate trajectory
      double x = robot_x, y = robot_y, yaw = robot_yaw;
      bool collision_free = true;
      double obstacle_cost = 0.0;
      std::vector<std::tuple<double, double, double>> trajectory;

      // Simulate trajectory over sim_time
      for (double t = 0; t < sim_time_; t += 0.1) {
        x += vx * cos(yaw) * 0.1;
        y += vx * sin(yaw) * 0.1;
        yaw += vth * 0.1;
        trajectory.emplace_back(x, y, yaw);

        unsigned int mx, my;
        if (costmap->worldToMap(x, y, mx, my)) {
          double cost = costmap->getCost(mx, my);
          // RCLCPP_INFO(node_->get_logger(), "Cost at (%f, %f): %d", x, y, cost);
          if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            collision_free = false;
            obstacle_cost += obstacle_weight_ * 100.0; // High penalty for lethal obstacles
            break;
          } else if (cost > 0) {
            obstacle_cost += obstacle_weight_ * cost; // Accumulate cost for non-lethal obstacles
          }
        } else {
          collision_free = false;
          obstacle_cost += obstacle_weight_ * 100.0; // Penalty for out-of-bounds
          break;
        }
      }
      
      if (!collision_free) {
        continue; // Skip trajectories with collisions
      }

      // Goal cost: Distance from trajectory endpoint to closest global path point
      double goal_dist = std::numeric_limits<double>::max();
      for (const auto & path_pose : global_path_.poses) {
        double dx = x - path_pose.pose.position.x;
        double dy = y - path_pose.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        goal_dist = std::min(goal_dist, dist);
      }
      double goal_cost = goal_weight_ * goal_dist;

      // Speed cost: Encourage higher linear speeds
      double speed_cost = speed_weight_ * (max_vel_x_ - vx);

      // Smoothness cost: Penalize high angular velocities
      double smoothness_cost = smoothness_weight_ * std::abs(vth) * (0.1 + 0.2 * (vx / max_vel_x_));

      // Total cost
      double total_cost = goal_cost + obstacle_cost + speed_cost + smoothness_cost;
      
      if (total_cost < best_cost) {
        RCLCPP_INFO(node_->get_logger(), "============================================================");
        RCLCPP_INFO(node_->get_logger(), "New best trajectory found with cost: %.2f", total_cost);
        RCLCPP_INFO(node_->get_logger(), "Obstacle cost for trajectory: %.2f", obstacle_cost);
        RCLCPP_INFO(node_->get_logger(), "Goal cost for trajectory: %.2f", goal_cost);
        RCLCPP_INFO(node_->get_logger(), "Speed cost for trajectory: %.2f", speed_cost);
        RCLCPP_INFO(node_->get_logger(), "Smoothness cost for trajectory: %.2f", smoothness_cost);
        RCLCPP_INFO(node_->get_logger(), "vx: %.2f, vth: %.2f", vx, vth);
        best_cost = total_cost;
        best_vx = vx;
        best_vy = 0.0; // Differential drive: no lateral movement
        best_vth = vth;
        best_trajectory_ = trajectory;
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Best trajectory found with cost: %.2f", best_cost);

  cmd_vel.twist.linear.x = best_vx;
  cmd_vel.twist.linear.y = best_vy;
  cmd_vel.twist.angular.z = best_vth;

  publishTrajectory();

  RCLCPP_INFO(node_->get_logger(), "Computed velocity: vx=%.2f, vth=%.2f", best_vx, best_vth);
  return cmd_vel;
}

void CustomDWAPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_vel_x_ *= speed_limit;
    max_vel_theta_ *= speed_limit;
  } else {
    max_vel_x_ = speed_limit;
    max_vel_theta_ = speed_limit;
  }
  RCLCPP_INFO(node_->get_logger(), "Speed limit set: max_vel_x=%.2f, max_vel_theta=%.2f", max_vel_x_, max_vel_theta_);
}

void CustomDWAPlanner::publishTrajectory()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_->now();

  for (const auto & [x, y, yaw] : best_trajectory_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // convert yaw to quaternion
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }

  selected_traj_pub_->publish(path);
  RCLCPP_INFO(node_->get_logger(), "Published selected trajectory with %zu poses", path.poses.size());
}

} // namespace custom_dwa_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_dwa_planner::CustomDWAPlanner, nav2_core::Controller)