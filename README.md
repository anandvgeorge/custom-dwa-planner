# Custom DWA Planner
A custom DAW planner implemented as a Nav2 controller plugin. It works with a Turtlebot3 burger model robot to move to a given goal, while avoiding obstacles. Since the planner is integrated with the nav2 stack, the main behaviour tree, global planning, local and global costmaps, slam etc. are not separately implemented. Relevant parameters of the planner are specified below in this README. 

Below GIF shows the demo of the planner controlling a Turtlebot3 Burger robot in a Gazebo world. Also the same video is present [here](assets/plan.mp4).

[![Watch the video](assets/demo.gif)](assets/plan.mp4)

## How to Run?

### Setup the environment and the package
1. Install ROS2 Humble.
2. Install turtlebot3 packages.
3. Install nav2 packages.
4. Clone this repo to a ros workspace and build it. <br/>
```
cd <ros_workspace>/src
git clone https://github.com/anandvgeorge/custom_dwa_planner.git
cd ../
colcon build --packages-select custom_dwa_planner
source install/setup.bash
```
### Start simulation
Start Gazebo simulator with turtlebot3 
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Bring up nav2 with custom dwa controller
```
cd <ros_workspace>
source install/setup.bash
ros2 launch custom_dwa_planner nav2_with_slam.launch.py 
```

Launch rviz2 with the config file
```
ros2 launch custom_dwa_planner launch_rviz.launch.py
```

Mark the goal with 2D Goal Pose

The robot should move to the goal!

## Parameters
Below are the parameters of DWA planner, and these default values are configured in the params file.
| Parameter Name        | Default Value | Description                                |
|-----------------------|---------------|--------------------------------------------|
| `desired_linear_vel`  | 0.5           | Desired forward velocity of the robot (m/s)|
| `max_angular_vel`     | 1.0           | Maximum allowed angular velocity (rad/s)   |
| `lookahead_time`      | 1.0           | Time horizon for trajectory simulation (s) |
| `linear_granularity`  | 0.05          | Step size in linear velocity sampling      |
| `angular_granularity` | 0.1           | Step size in angular velocity sampling     |
| `goal_weight`         | 3.0           | Weight for goal proximity in cost function |
| `obstacle_weight`     | 1.0           | Weight for obstacle avoidance              |
| `speed_weight`        | 1.0           | Weight for higher velocity trajectories    |
| `smoothness_weight`   | 0.0           | Weight for smoother trajectory curves      |

## Limitations
1. *Smoothness cost related* -- When the smoothness cost is enabled (above zero), the robot does not move to the goal if the initial global path requires the robot to rotate too much towards the path. If the global path is already algned in almost the same direction of the robot, the robot moves towars the goal
2. *Dymanic obstacles* -- Not tested yet.
