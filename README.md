# Custom DWA Planner
A custom DAW planner implemented as a Nav2 controller plugin. It works with a Turtlebot3 burger model robot to move to a given goal, while avoiding obstacles. Since the planner is integrated with the nav2 stack, the main behaviour tree, global planning, local and global costmaps, slam etc. are not separately implemented. 

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
ros2 launch nav2_bringup bringup_launch.py\
    slam:=True \
    use_sim_time:=True \
    params_file:=src/custom_dwa_planner/config/nav2_params.yaml \
    map:=dummy.yaml
```

Launch rviz2 with the config file
```
cd <ros_workspace>
rviz2 -d src/custom_dwa_planner/config/turtlebot_cutom_controller.rviz 
```

Mark the goal with 2D Goal Pose

The robot should move to the goal!

## Limitations
1. *Smoothness cost related* -- When the smoothness cost is enabled (above zero), the robot does not move to the goal if the initial global path requires the robot to rotate too much towards the path. If the global path is already algned in almost the same direction of the robot, the robot moves towars the goal
2. *Dymanic obstacles* -- Not tested yet.
