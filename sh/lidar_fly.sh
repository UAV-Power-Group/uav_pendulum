#!/bin/bash
##Vicon
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyTHS1:921600"; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; rosrun px4_command move; exec bash"' \
--tab -e 'bash -c "sleep 10; roslaunch px4_command tfmini.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch rplidar_ros rplidar.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; source /home/amov/catkin_ws/devel_isolated/setup.bash; roslaunch cartographer_ros rplidar.launch; exec bash"' \
