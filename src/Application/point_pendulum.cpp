/***************************************************************************************************************************
 * point_pendulum.cpp
 *
 * Author: Yuxi
 *
 * Update Time: 2020.6.1
 *
 * 说明: 质点法平衡倒立摆
 *
***************************************************************************************************************************/

#include <ros/ros.h>
#include <iostream>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <px4_command/command.h>


using namespace std;
//全局变量
Eigen::Vector3d pos_pendulum_mocap; // 倒立摆位置
Eigen::Vector3d pos_uav_mocap; // 无人机位置
Eigen::Vector3d vel_uav_mocap; // 无人机位置
Eigen::Vector3d vel_pen_mocap;
enum Command
{
    Idle,
    Takeoff,
    Move_ENU,
    Move_Body,
    Hold,
    Land,
    Disarm,
    Failsafe_land,
};
px4_command::command Command_now;
//bool ready_for_pub = false;
//void pos_pub_timer_cb(const ros::TimerEvent& TE);
//---------------------------------------飞行参数---------------------------------------------
float x_lim;    // x轴地理围栏限制
float y_lim;    // y轴地理围栏限制
float z_lim;    // z轴地理围栏限制
float z_max;
float sleep_time;
float length;
float Kp1;
float Kd;
float delta;
// 倒立摆参数读取
void pendulum_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Read the Pendulum Position from the Vrpn Package [Frame: Vicon]
    Eigen::Vector3d pos_pendulum_mocap_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    pos_pendulum_mocap = pos_pendulum_mocap_enu;
}
// 无人机参数读取
void uav_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Read the Pendulum Position from the Vrpn Package [Frame: Vicon]
    Eigen::Vector3d pos_uav_mocap_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    pos_uav_mocap = pos_uav_mocap_enu;
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // Read the Pendulum Position from the Vrpn Package [Frame: Vicon]
    Eigen::Vector3d vel_pen_mocap_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
    vel_pen_mocap = vel_pen_mocap_enu;
}

void vel_uav_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // Read the Pendulum Position from the Vrpn Package [Frame: Vicon]
    Eigen::Vector3d vel_uav_mocap_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
    vel_uav_mocap = vel_uav_mocap_enu;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_pendulum");
    ros::NodeHandle nh("~");

    // 频率 [50hz]
    ros::Rate rate(100.0);

    // 【订阅】订阅倒立摆的位置数据（Vicon）
    ros::Subscriber pendulum_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/pendulum/pose",1000,pendulum_cb);
    // 【订阅】订阅无人机的位置数据（Vicon）
    ros::Subscriber uav_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV_6_3/pose",1000,uav_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/pendulum/twist",1000,vel_cb);
    ros::Subscriber vel_uav_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/UAV_6_3/twist",1000,vel_uav_cb);
    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);
    // ros::Timer
    //ros::Timer pos_pub_timer = nh.createTimer(ros::Duration(1.0/20.0), pos_pub_timer_cb);

    //参数读取
    nh.param<float>("x_lim", x_lim, 2.0);
    nh.param<float>("y_lim", y_lim, 2.0);
    nh.param<float>("z_lim", z_lim, 0.5);
    nh.param<float>("z_max", z_max, 2.5);
    nh.param<float>("sleep_time", sleep_time, 5.0);
    nh.param<float>("length", length, 0.5);
    nh.param<float>("Kp1", Kp1, 1.0);
    nh.param<float>("Kd", Kd, 1.0);
    nh.param<float>("delta", delta, 0.0);

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "x_lim: " << x_lim << "[m]" << endl;
    cout << "y_lim: " << y_lim << "[m]" << endl;
    cout << "z_lim: " << z_lim << "[m]" << endl;
    cout << "z_max: " << z_max << "[m]" << endl;
    cout << "Kp1: " << Kp1 << endl;
    cout << "Kd: " << Kd << endl;
    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;

    if(check_flag != 1)
        return -1;

    int i = 0;
    int comid = 0;
    while(i < sleep_time){

        Command_now.command = Move_ENU;       //Move模式
        Command_now.sub_mode = 0;             //子模式：位置控制模式
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = 1.5;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;

        move_pub.publish(Command_now);

        rate.sleep();

        i++;
    }
    
    while (ros::ok())
    {
        // 回调一次
        ros::spinOnce();

	float error_x = delta + pos_uav_mocap[0] - pos_pendulum_mocap[0];
	float pos_de = -Kp1 * error_x;

	float error_vx = vel_uav_mocap[0] - vel_pen_mocap[0];
	float vel_de = -Kd * error_vx;

        Command_now.command = Move_ENU;
        Command_now.sub_mode = 2;
        Command_now.vel_sp[0] = pos_de + vel_de;
        Command_now.vel_sp[1] = 0;
        Command_now.pos_sp[2] = pos_uav_mocap[2];
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;

        if(pos_pendulum_mocap[0] < x_lim && pos_pendulum_mocap[0] > -x_lim 
        && pos_pendulum_mocap[1] < y_lim && pos_pendulum_mocap[1] > -y_lim
        && pos_pendulum_mocap[2] > z_lim && pos_pendulum_mocap[2] < z_max){
            move_pub.publish(Command_now);                
        }
        else
        {
            Command_now.command = Land;
            move_pub.publish(Command_now);
        }

        rate.sleep();
    }
    
    return 0;
}
