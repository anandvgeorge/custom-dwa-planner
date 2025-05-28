#ifndef CUSTOM_DWA_PLANNER_H
#define CUSTOM_DWA_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <string>

namespace custom_dwa_planner
{

class CustomDWAPlanner : public nav2_core::Controller
{
public:
    CustomDWAPlanner();
    ~CustomDWAPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker
    ) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string plugin_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav_msgs::msg::Path global_path_;
    std::vector<std::tuple<double, double, double>> best_trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_traj_pub_;
    double max_vel_x_, max_vel_y_, max_vel_theta_;
    double sim_time_;
    double linear_granularity_, angular_granularity_;
    double goal_weight_, obstacle_weight_, speed_weight_, smoothness_weight_;

    void publishTrajectory();
};

} // namespace custom_dwa_planner

#endif // CUSTOM_DWA_PLANNER_H
