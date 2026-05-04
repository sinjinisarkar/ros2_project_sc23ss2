# ROS2 Project - sc23ss2

## Description
A ROS2 Python package that autonomously navigates a TurtleBot3 robot to detect RGB coloured boxes and stop within 1 metre of the blue box.

## How to run
1. Launch Gazebo:
ros2 launch turtlebot3_gazebo turtlebot3_task_world_2026.launch.py

2. Launch Navigation:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc23ss2/map/map.yaml

3. Set 2D Pose Estimate in RViz

4. Run the project:
ros2 run ros2_project_sc23ss2 project
