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

#include <ros/ros.h>
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
float radius;                       //飞行半径
float height_circle;                //飞行高度
float sleep_time;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_tracking_8_tracking");
    ros::NodeHandle nh("~");

    // 频率 [10hz]
    ros::Rate rate(10.0);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher move_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //参数读取
    nh.param<float>("radius", radius, 1);
    nh.param<float>("height_circle", height_circle, 1.0);
    nh.param<float>("sleep_time", sleep_time, 10.0);

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "radius: " << radius << "[m]"<<endl;
    cout << "height_circle: " << height_circle << "[m]" << endl;
    cout << "Please check the parameter and setting，1 for go on， else for quit: " << endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    int tracking_mod;
    cout << "Please choose the tracking mode, 0 for circle tracking, 1 for 8 tracking: " <<endl;
    cin >> tracking_mod;

    int i = 0;
    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主程序<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //circle_tracking
    if (tracking_mod == 0) {

        float angle = 0;
        float angle_step = 4;
        // 飞行到ｘ轴点
        while (i < sleep_time) {

            Command_now.command = Move_ENU; //Move模式
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = radius;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            i++;
        }

        // 飞　圆
        while (angle <= 360){
            angle+=angle_step;

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle + radius * sin(angle / 180 * M_PI);
            Command_now.yaw_sp = angle / 180 * M_PI;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

        }

        i = 0;
        while (i < sleep_time) {

            Command_now.command = Move_ENU; //Move模式
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = radius;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            i++;
        }
    }

    // 8 tracking
    else{

        float angle = 0;
        float angle_step = 4;
        // 飞行到ｘ轴点
        while (i < sleep_time)
        {

            Command_now.command = Move_ENU; //Move模式
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            i++;
        }

        while (angle <= 360)
        {

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = -radius + radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle + radius * sin(angle / 180 * M_PI);
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            angle+=angle_step;
        }

        while (angle <= 720)
        {
            // float angle_step = 5;

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = radius - radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle + radius * sin(angle / 180 * M_PI);
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            angle+=angle_step;
        }
	
        angle = 0;
        // 飞行到ｘ轴点
        while (i < sleep_time)
        {

            Command_now.command = Move_ENU; //Move模式
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            i++;
        }

        while (angle <= 360)
        {

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[0] = -radius + radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle + radius * sin(angle / 180 * M_PI);
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            angle+=angle_step;
        }

        while (angle <= 720)
        {
            // float angle_step = 5;

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[0] = radius - radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle + radius * sin(angle / 180 * M_PI);
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            angle+=angle_step;
        }

        angle = 0;
        // 飞行到ｘ轴点
        while (i < sleep_time)
        {

            Command_now.command = Move_ENU; //Move模式
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            i++;
        }

        while (angle <= 360)
        {

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = radius * sin(angle / 180 * M_PI);
            Command_now.pos_sp[1] = -radius + radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            angle+=angle_step;
        }

        while (angle <= 720)
        {
            // float angle_step = 5;

            Command_now.command = Move_ENU;
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = radius * sin(angle / 180 * M_PI);
            Command_now.pos_sp[1] = radius - radius * cos(angle / 180 * M_PI);
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            angle+=angle_step;
        }

        i = 0;
        while (i < sleep_time) {

            Command_now.command = Move_ENU; //Move模式
            Command_now.sub_mode = 0;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.pos_sp[2] = height_circle;
            Command_now.yaw_sp = 0;
            Command_now.comid = comid;
            comid++;

            move_pub.publish(Command_now);

            rate.sleep();

            i++;
        }

    }



    //降落
    Command_now.command = Land;
    move_pub.publish(Command_now);

    rate.sleep();

    cout << "Land"<<endl;

    return 0;
}
