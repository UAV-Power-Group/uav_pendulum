#!/bin/bash

## VICON质点法平衡杆

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch vrpn_client_ros sample.launch; exec bash "' \
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600"; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command px4_pos_controller.launch; exec bash"' \
--tab -e 'bash -c "sleep 5;source  ~/px4_ws/devel/setup.bash; roslaunch px4_command pendulum_PID.launch; exec bash"' \
