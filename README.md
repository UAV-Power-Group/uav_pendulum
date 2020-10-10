# UAV_Pendulum

## px4_command
Send command to PX4 using Mavros package

PX4 Version: 1.8.2

## File Tree
```
└── px4_command
    ├── CMakeLists.txt
    ├── config
    │   ├── autonomous_landing.yaml
    │   ├── circle_tracking_8_tracking.yaml
    │   ├── collision_avoidance_streo.yaml
    │   ├── collision_avoidance.yaml
    │   ├── Parameter_for_control.yaml
    │   ├── payload_drop.yaml
    │   ├── pendulum_PID.yaml
    │   ├── point_pendulum.yaml
    │   ├── px4_pos_estimator.yaml
    │   ├── square.yaml
    │   └── target_tracking.yaml
    ├── include
    │   ├── command_to_mavros.h
    │   ├── Frame_tf_utils.h
    │   ├── HighPassFilter.h
    │   ├── LeadLagFilter.h
    │   ├── LowPassFilter.h
    │   ├── math_utils.h
    │   ├── pos_controller_NE.h
    │   ├── pos_controller_Passivity.h
    │   ├── pos_controller_PID (copy).h
    │   ├── pos_controller_PID.h
    │   ├── pos_controller_UDE.h
    │   ├── serial.hpp
    │   ├── test
    │   ├── TFmini.h
    │   └── UDE paper
    ├── launch
    │   ├── autonomous_landing.launch
    │   ├── circle_tracking_8_tracking.launch
    │   ├── collision_avoidance.launch
    │   ├── collision_avoidance_streo.launch
    │   ├── payload_drop.launch
    │   ├── pendulum_PID.launch
    │   ├── point_pendulum.launch
    │   ├── px4_pos_controller.launch
    │   ├── px4_pos_estimator.launch
    │   ├── px4_sender.launch
    │   ├── SITL.launch
    │   ├── square.launch
    │   ├── target_tracking.launch
    │   └── tfmini.launch
    ├── LICENSE
    ├── msg
    │   ├── command.msg
    │   └── data_log.msg
    ├── package.xml
    ├── README.md
    ├── sh
    │   ├── circle_fly.sh
    │   ├── collision_avoidance.sh
    │   ├── lidar_fly.sh
    │   ├── my_square_imu.sh
    │   ├── pendulum_PID.sh
    │   ├── point_pendulum.sh
    │   ├── sh_for_odroid
    │   ├── sh_for_P200
    │   ├── sh_for_simulation
    │   ├── sh_origin
    │   └── test
    └── src
        ├── Application
        ├── px4_pos_controller.cpp
        ├── px4_pos_estimator (copy).cpp
        ├── px4_pos_estimator.cpp
        ├── px4_sender.cpp
        ├── Test
        └── Utilities
```

## Installation

1. Install the mavros packgae by Binary installation
   
    Please ref to: https://github.com/mavlink/mavros
    
    If you has install the mavros package by source code. Plaese deleat it firstly.
   
2. Create a new ros space called "px4_ws" in your home folder
  
    `mkdir -p ~/px4_ws/src`
  
    `cd ~/px4_ws/src`
  
    `catkin_init_workspace`
    
    Please source manually, open a new terminal
    
    `gedit .bashrc`  
    
    Add the path `source /home/$(your computer name)/px4_ws/devel/setup.bash` in the end of the bashrc.txt file

3. Git clone the px4_command package
    
    `cd ~/px4_ws/src`
    
    download from github repository

    `git clone https://github.com/amov-lab/px4_command`
    
    `cd ..`
    
    `catkin_make`
    
## Coordinate frames

   Here we are using **ENU** frames.

  >  MAVROS does translate Aerospace NED frames, used in FCUs to ROS ENU frames and vice-versa. For translate airframe related data we simply apply rotation 180° about ROLL (X) axis. For local we apply 180° about ROLL (X) and 90° about YAW (Z) axes

## Branch

fsc_lab branch is used for fsc_lab quadrotor experiment.
    
Use this command to switch to fsc_lab branch

`git checkout fsc_lab`

