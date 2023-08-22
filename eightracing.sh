#!/bin/bash

sourceBash="$1"
echo "$sourceBash"

run_command() {
    gnome-terminal --window -- bash -c "$1"
}

# run_command "bash ${sourceBash}/LIDAR.sh" &&
# sleep 3
# run_command "bash ${sourceBash}/interface.sh" &&
# sleep 3
# run_command "bash ${sourceBash}/ASENSINGIMU.sh" &&
# sleep 3 
gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash && source /home/kerwin/LIDAR_ye/devel/setup.bash && rosrun control PP_car ; $SHELL"
sleep 2
gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash && source /home/kerwin/LIDAR_ye/devel/setup.bash && roslaunch cone_position cone.launch ; $SHELL"


