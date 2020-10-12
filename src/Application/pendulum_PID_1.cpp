/***************************************************************************************************************************
 * pendulum_PID.cpp
 *
 * Author: Yuxi
 *
 * Update Time: 2020.7.10
 *
 * 说明: PID控制倒立摆位置与速度
 *
***************************************************************************************************************************/

#include <ros/ros.h>

#include <iostream>
#include <math.h>


#include <math_utils.h>
#include <Frame_tf_utils.h>

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
//---------------------------------------vicon定位相关------------------------------------------
Eigen::Quaterniond q_uav_mocap;                           // 无人机当前姿态(四元数)

Eigen::Vector3d Euler_uav_mocap;                              //无人机当前姿态 (vicon)

Eigen::Vector3d pose_pen_mocap;
Eigen::Vector3d pose_uav_mocap;

//Eigen::Vector3d twist_uav_mocap;
//Eigen::Vector3d twist_pen_mocap;

float error_last = 0;

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
//---------------------------------------飞行参数---------------------------------------------
float x_lim;    // x轴地理围栏限制
float y_lim;    // y轴地理围栏限制
float z_lim;    // z轴地理围栏限制
float sleep_time;
float z_max;
float delta;
float height;
float time_delay;

float control_P;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>FUZZY CONTROL<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float Offset_Table[7] = {-15, -10, -5, 0, 5, 10, 15};
float Kp_Offset_Table[7] = {-0.16, -0.15, -0.12, 0, 0.12, 0.15, 0.16};
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_dt(ros::Time last);                                                        // 获取时间
float cal_error_deriv(float error_now, float dt);                                    // 求微分
void Get_fuzzy(float theta);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void euler_uav_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Vector3d pose_uav_mocap_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    pose_uav_mocap = pose_uav_mocap_enu;
    // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
    Eigen::Quaterniond q_uav_mocap_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_uav_mocap = q_uav_mocap_enu;

    // Transform the Quaternion to Euler Angles
    Euler_uav_mocap = quaternion_to_euler(q_uav_mocap);

}

void euler_pen_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Vector3d pose_pen_mocap_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    pose_pen_mocap = pose_pen_mocap_enu;
}
/*
void twist_uav_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    Eigen::Vector3d twist_uav_mocap_enu(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
    twist_uav_mocap = twist_uav_mocap_enu;
}

void twist_pen_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    Eigen::Vector3d twist_pen_mocap_enu(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
    twist_pen_mocap = twist_pen_mocap_enu;
}
*/
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pendulum_PID");
    ros::NodeHandle nh("~");

    // 频率 [20hz]
    ros::Rate rate(50.0);

    // 【订阅】订阅倒立摆的位置数据（Vicon）
        // 【订阅】vicon uav Euler角
    ros::Subscriber vicon_uav_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV_6_3/pose", 10, euler_uav_cb);

    // 【订阅】vicon pendulum Euler角
    ros::Subscriber euler_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/pendulum/pose", 10, euler_pen_cb);

    // 【订阅】vicon uav Euler角
    //ros::Subscriber vicon_uav_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/UAV_6_3/twist", 10, twist_uav_cb);

    // 【订阅】vicon pendulum Euler角
    //ros::Subscriber euler_pen_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/pendulum/twist", 10, twist_pen_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);


    //参数读取
    nh.param<float>("x_lim", x_lim, 2.0);
    nh.param<float>("y_lim", y_lim, 2.0);
    nh.param<float>("z_lim", z_lim, 0.5);
    nh.param<float>("z_max", z_max, 2.5);
    nh.param<float>("sleep_time", sleep_time, 5.0);
    nh.param<float>("delta", delta, 0.0);
    nh.param<float>("height", height, 0.1);
    nh.param<float>("time_delay", time_delay, 0.02);

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "x_lim" << x_lim << "[m]" << endl;
    cout << "y_lim" << y_lim << "[m]" << endl;
    cout << "z_lim" << z_lim << "[m]" << endl;
    cout << "z_max" << z_max << "[m]" << endl;
    cout << "delta: " << delta << "[m]" << endl;
    cout << "height: " << height << "[m]" << endl;
    cout << "time_delay: " << time_delay << endl;

    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;

    if(check_flag != 1)
        return -1;

    int i = 0;
    int comid = 0;

    // write into files
    ofstream ofile;
    ofile.open("/home/amov/fly_logs/theta_P_fuzzy.txt");
    while(i < sleep_time){

        Command_now.command = Move_ENU;       //Move模式
        Command_now.sub_mode = 0;             //子模式：位置控制模式
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.pos_sp[2] = 1;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;

        move_pub.publish(Command_now);

        rate.sleep();

        i++;
    }

    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;

    if(check_flag != 1)
        return -1;

    ros::Time begin_time = ros::Time::now();
    float last_time = get_dt(begin_time);
    float dt = 0;

    while (ros::ok())
    {
        // 回调一次
        ros::spinOnce();

        float cur_time = get_dt(begin_time);
        dt = cur_time  - last_time;

	ofile << cur_time  << "	   ";

        if(dt < 0.005)
            dt = 0.005;

        if(dt > 0.03)
            dt = 0.03;
        
        last_time = cur_time;
        
        float uav_theta = Euler_uav_mocap[1];

	float error_pos = pose_pen_mocap[0] - pose_uav_mocap[0] - delta - height * sin(uav_theta);
	//float error_pos = pose_pen_mocap[0] - pose_uav_mocap[0] - delta;
        float pen_theta = asin(error_pos / 0.24);

        float theta_uav = pen_theta - uav_theta;
	float theta_0 = pen_theta;

        float theta_dot = cal_error_deriv(theta_0, dt);

	if(theta_dot > 8)
	    theta_dot = 8;
	else if(theta_dot < -8)
	    theta_dot = -8;

        //float theta = pen_theta + time_delay * theta_dot - uav_theta;
	float theta = pen_theta + time_delay * theta_dot;

	ofile << theta_dot << "	" << theta * (180/3.14) << endl;

        //float twist_theta = twist_pen_mocap[0] - twist_uav_mocap[0];

	Get_fuzzy(theta);

	if(theta_0 * theta_uav < 0 && abs(theta_uav)*(180/3.14) > 8)
	    control_P = -control_P;

        Command_now.command = Move_ENU;
        Command_now.sub_mode = 7;			// mode for fuzzy
        Command_now.pos_sp[0] = control_P;
        Command_now.pos_sp[1] = pose_uav_mocap[1];
        Command_now.pos_sp[2] = pose_uav_mocap[2];
        Command_now.vel_sp[0] = 0 - theta_dot;
        Command_now.yaw_sp = 0;
        Command_now.comid = comid;
        comid++;

        if(pose_uav_mocap[0] < x_lim && pose_uav_mocap[0] > -x_lim 
        && pose_uav_mocap[1] < y_lim && pose_uav_mocap[1] > -y_lim
        && pose_uav_mocap[2] > z_lim && pose_uav_mocap[2] < z_max){
            move_pub.publish(Command_now);                
        }
        else
        {
            Command_now.command = Land;
            move_pub.publish(Command_now);
        }

        rate.sleep();
    }
    
    ofile.close();
    return 0;
}


//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

float cal_error_deriv(float error_now, float dt)
{
    float error_dot_now;
    error_dot_now = (error_now - error_last) / dt;

    error_last = error_now;

    return error_dot_now;
}

void Get_fuzzy(float theta){
    int i = 0;
    if(theta*(180/3.14) <= Offset_Table[0])
	theta = Offset_Table[0];
    else if(theta*(180/3.14) >= Offset_Table[6])
	theta = Offset_Table[6];
    else
	theta = theta * (180/3.14);

    for(i = 0; i < 6; i++){
	if(theta >= Offset_Table[i] && theta < Offset_Table[i+1]){
	    control_P = Kp_Offset_Table[i] + (theta - Offset_Table[i]) * (Kp_Offset_Table[i+1] - Kp_Offset_Table[i]) / (Offset_Table[i+1] - Offset_Table[i]);
	    break;
	}
    }
    if(theta <= Offset_Table[0]){
      control_P = Kp_Offset_Table[0];
    }
    if(theta >= Offset_Table[6]){
      control_P = Kp_Offset_Table[6];
    }
}
