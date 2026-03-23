## Description
ROS2 workspace to run simulation pipeline including Stonefish, ROS2, Turtlebot packages. This workspace is created to run assignments in Hands-on Localization and Hands-on Planning courses, taught by Universtiy of Girona in AY25-26.

## Install dependencies
```bash
pip3 install bresenham --break-system-packages
sudo apt install ros-jazzy-tf-transformations
```

Copy the missing packages to the workspace.

## Run simulaton
In the first terminal:
```bash
ros2 launch turtlebot_simulation turtlebot_hoi_circuit2.launch.py
```

In the second terminal:
```bash
ros2 run grid_mapping occupancy_grid
```

In the third terminal:
```bash
ros2 run online_motion_planning rrt_tb
```