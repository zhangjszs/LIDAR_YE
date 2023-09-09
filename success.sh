#!/bin/bash

# Source your ROS environment setup (change to your ROS distro)
source /opt/ros/melodic/setup.bash

# Navigate to your catkin workspace (change to your workspace path)
source devel/setup.bash

# Start ROS nodes using roslaunch (change to your launch file and parameters)
gnome-terminal -- bash -c "roslaunch cone_position cone.launch"

# Add more roslaunch commands as needed for additional nodes
# roslaunch your_package another_launch_file.launch

# Optionally, you can add more commands or setup here
# For example:
# rosparam set parameter_name parameter_value
# rosrun your_package your_node
