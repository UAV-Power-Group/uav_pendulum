/***************************************************************************************************************************
 * circle_tracking_8_tracking.cpp
 *
 * Author: Yuxi
 *
 * Update Time: 2019.9.17
 *
 * 说明: mavros无人机圆形以及8字形飞行(依据位置控制)
 *
***************************************************************************************************************************/
//头文件
#include <ros/ros.h>
//#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <Eigen/Eigen>
#include <math.h>


#include <math_utils.h>
#include <Frame_tf_utils.h>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>


using namespace std;
//---------------------------------------vicon定位相关------------------------------------------
Eigen::Quaterniond q_uav_mocap;                           // 无人机当前姿态(四元数)
Eigen::Quaterniond q_pen_mocap;

Eigen::Vector3d Euler_uav_mocap;                              //无人机当前姿态 (vicon)
Eigen::Vector3d Euler_pen_mocap;

Eigen::Vector3d pose_pen_mocap;
Eigen::Vector3d pose_uav_mocap;

Eigen::Vector3d twist_uav_mocap;
Eigen::Vector3d twist_pen_mocap;

float error_last = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float get_dt(ros::Time last);                                                        // 获取时间
float cal_error_deriv(float error_now, float dt);                                    // 求微分
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
    // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
    Eigen::Quaterniond q_pen_mocap_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q_pen_mocap = q_pen_mocap_enu;

    // Transform the Quaternion to Euler Angles
    Euler_pen_mocap = quaternion_to_euler(q_pen_mocap);


    Eigen::Vector3d pose_pen_mocap_enu(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    pose_pen_mocap = pose_pen_mocap_enu;
}

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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_demo");
    ros::NodeHandle nh("~");

    // 【订阅】vicon uav Euler角
    ros::Subscriber vicon_uav_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/UAV_6_3/pose", 10, euler_uav_cb);

    // 【订阅】vicon pendulum Euler角
    ros::Subscriber euler_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/pendulum/pose", 10, euler_pen_cb);

    // 【订阅】vicon uav Euler角
    ros::Subscriber vicon_uav_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/UAV_6_3/twist", 10, twist_uav_cb);

    // 【订阅】vicon pendulum Euler角
    ros::Subscriber euler_pen_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/pendulum/twist", 10, twist_pen_cb);

    ros::Rate rate(50.0);


    ros::Time begin_time = ros::Time::now();
    float last_time = get_dt(begin_time);
    float dt = 0;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        float cur_time = get_dt(begin_time);
        dt = cur_time  - last_time;

        if(dt < 0.01)
            dt = 0.01;

        if(dt > 0.03)
            dt = 0.03;
        
        last_time = cur_time;
        
        float uav_theta = Euler_uav_mocap[1];

	float error_pos = pose_pen_mocap[0] - pose_uav_mocap[0] - 0.012 - 0.1 * sin(uav_theta);
        float pen_theta = asin(error_pos / 0.24);

	float pen_theta_1 = Euler_pen_mocap[1] - uav_theta;

        float theta = pen_theta - uav_theta;

        float theta_dot = cal_error_deriv(theta, dt);

        float twist_theta = twist_pen_mocap[0] - twist_uav_mocap[0];

        cout << "------------------------------------------------" << endl;
	cout << "pen_theta: " << pen_theta * (180/3.14) << endl;
	cout << "uav_theta: " << uav_theta * (180/3.14) << endl;
        cout << "theta:     " << theta * (180/3.14) << endl;
        cout << "theta_dot:     " << theta_dot << endl;
        cout << "twist_theta:   " << twist_theta << endl;


        rate.sleep();
    }

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
