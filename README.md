# COMP3631 ROS2 Project - sc23ss2

## Description
A ROS2 Python package that autonomously navigates a TurtleBot3 robot in a simulated Gazebo environment. The robot combines Nav2 motion planning with OpenCV computer vision to detect RGB coloured boxes (red, green and blue) and stop within 1 metre of the blue box.

## Task
- Autonomously explore the environment using heuristic waypoints
- Detect all three RGB coloured boxes using HSV colour thresholding and display bounding boxes
- Visually approach the blue box using camera feedback and stop within 1 metre

## Motion Planning Approach
The robot uses **heuristic waypoint navigation** to explore the map efficiently:

1. The provided map was loaded in RViz and the **Publish Point** tool was used to identify strategic coordinates in the map's configuration space (x, y, θ)
2. **Waypoint 0** (-2.12, -4.99) — A central point in the map selected because from this position the robot has line of sight to both the red and green boxes. The robot performs a full 360° spin here to maximise colour detection coverage.
3. **Waypoint 1** (-5.96, -9.06) — A position approximately 1 metre from the blue box, selected using the grid on the map (each grid square = 1x1m). From here the robot switches to visual approach using the camera.

Nav2 automatically plans a collision-free path to each waypoint using the provided map.

## How to Run

**Terminal 1 — Launch Gazebo:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_task_world_2026.launch.py
```

**Terminal 2 — Launch Navigation:**
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/ros2_ws/src/ros2_project_sc23ss2/map/map.yaml
```

**Terminal 3 — Set 2D Pose Estimate in RViz:**
- Click "2D Pose Estimate" in RViz toolbar
- Click on the map where the robot is in Gazebo and drag to set the direction it is facing
- Wait for the laser scan (red dots) to align with the walls — this confirms correct localisation

**Terminal 3 — Run the project:**
```bash
ros2 run ros2_project_sc23ss2 project
```

## How it Works
1. Robot navigates to central waypoint (-2.12, -4.99)
2. Robot performs a full 360° spin to detect red and green boxes
3. Robot navigates to second waypoint (-5.96, -9.06) near the blue box
4. Robot detects blue box using HSV thresholding and uses contour centre of mass to steer towards it
5. Robot uses contour area as a proxy for distance and stops when within 1 metre of the blue box

## Computer Vision
- Colours detected using HSV thresholding in OpenCV
- Red uses two HSV ranges as red wraps around the HSV spectrum
- Bounding boxes drawn around detected colours in the camera feed
- Blue box approach uses `cv2.moments()` for steering and `cv2.contourArea()` for distance estimation
