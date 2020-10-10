/***************************************************************************************************************************
 * collision_avoidance.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.4.17
 *
 * 说明: 避障程序
 *
 *
 * 初版，需要遵循固定场景，具体场景设置请参看教程文件
 * 具体功能待完善
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <command_to_mavros.h>
//topic 头文件
#include <iostream>
#include <px4_command/command.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>


/*
 * 主要功能:
 * 1.获取激光雷达数据
 * 2.根据距离判断是否启用避障策略
 * 3.如启用避障策略,产生控制指令
 *
 */

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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

#define RAD2DEG(x) ((x)*180./M_PI)
//--------------------------------------------输入--------------------------------------------------
// sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
// Eigen::Vector3d pos_drone;                                  //无人机当前位置
// float target_x;                                                 //期望位置_x
// float target_y;                                                 //期望位置_y

// int range_min;                                                //激光雷达探测范围 最小角度
// int range_max;                                                //激光雷达探测范围 最大角度

// //--------------------------------------------算法相关--------------------------------------------------
// float R_outside,R_inside;                                       //安全半径 [避障算法相关参数]
// float p_R;                                                      //大圈比例参数
// float p_r;                                                      //小圈比例参数

// float distance_c,angle_c;                                       //最近障碍物距离 角度
// float distance_cx,distance_cy;                                  //最近障碍物距离XY
// float vel_collision[2];                                         //躲避障碍部分速度
// float vel_collision_max;                                        //躲避障碍部分速度限幅

// float p_xy;                                                     //追踪部分位置环P
// float vel_track[2];                                             //追踪部分速度
// float vel_track_max;                                            //追踪部分速度限幅
 int flag_land;                                                  //降落标志位
// //--------------------------------------------输出--------------------------------------------------
// std_msgs::Bool flag_collision_avoidance;                       //是否进入避障模式标志位
// float vel_sp_body[2];                                           //总速度
// float vel_sp_max;                                               //总速度限幅
 px4_command::command Command_now;                               //发送给position_control.cpp的命令
// //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// void cal_min_distance();
// float satfunc(float data, float Max);
// void printf();                                                                       //打印函数
// void printf_param();    


//--------------------------------------------输入--------------------------------------------------
sensor_msgs::LaserScan Laser;                                   //激光雷达点云数据
Eigen::Vector3d pos_drone;                                      //无人机当前位置 (来自fcu)
float target_x;                                                 //期望位置_x
float target_y;                                                 //期望位置_y
float target_z;                                                 //期望位置_z
float target_temp_x;
float target_temp_y;
int xiangxian = 0;
//--------------------------------------------算法相关--------------------------------------------------
float p_xy;                                                     //追踪部分位置环P
float p_z;
float vel_track[2];                                             //追踪部分速度
//--------------------------------------------输出--------------------------------------------------
Eigen::Vector3d vel_sp_body;                                           //总速度
float vel_sp_max;                                               //总速度限幅
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float satfunc(float data, float Max);
void printf();                                                                       //打印函数
void printf_param();                                                                 //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float barriar_threshols = 2; //设置判断是否为障碍物距离
float pi = 3.1415926;
typedef struct
{
	int amount; 				//障碍物数量
	float distan_min;			//据某个障碍物最短距离
	float direc_start;			//障碍物起始方向
	float distan_start;			//障碍物起始距离 
	float direc_stop;			//障碍物结束方向	
	float distan_stop;			//障碍物结束距离
	float size; 				//障碍物大小
	float frame_x;				//障碍物相当飞机的坐标
	float frame_y;
	float world_x;				//世界坐标系ENU
	float world_y;				
	float distan_min_vert_range; //某个障碍物最短距离点据最短路经的垂直距离
	float distan_min_away_target;//某个障碍物据目标点的距离
}S_BARRIER;
int barrier_num = 0;
int barrier_obj = 0;
float uav2targetDistan;
S_BARRIER s_barrier[10];


                                                             //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//接收雷达的数据，并做相应处理,然后计算前后左右四向最小距离
// void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//     Laser = *scan;

//     int count;    //count = 359 or 358
//     count = Laser.scan_time / Laser.time_increment;

//     //-179°到180°
//     //cout << "Angle_range : "<< RAD2DEG(Laser.angle_min) << " to " << RAD2DEG(Laser.angle_max) <<endl;

//     //剔除inf的情况
//     for(int i = 0; i <= count; i++)
//     {
//         //判断是否为inf
//         int a = isinf(Laser.ranges[i]);

//         //如果为inf，则赋值上一角度的值
//         if(a == 1)
//         {
//             if(i == 0)
//             {
//                 Laser.ranges[i] = Laser.ranges[count];
//             }
//             else
//             {
//                 Laser.ranges[i] = Laser.ranges[i-1];
//             }
//         }
//     }

//     //计算前后左右四向最小距离
//     cal_min_distance();

// }
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    Laser = *scan;
    int count = 359; 
	float distan_min_direc = 0;   
	for(int i=0; i<= barrier_num; i++)
	{
		s_barrier[i].distan_min = 0;
 		s_barrier[i].direc_start = 0;
		s_barrier[i].direc_stop = 0;
 		s_barrier[i].distan_start = 0;
		s_barrier[i].distan_stop = 0;
		s_barrier[i].frame_x = 0;
		s_barrier[i].frame_y = 0;
		s_barrier[i].world_x = 0;
		s_barrier[i].world_y = 0;
		s_barrier[i].distan_min_vert_range = 0;
		s_barrier[i].size = 0;
		
	}
	barrier_num = 0;
	/*检测障碍物*/
    for(int temp = 0; temp <= count; temp++)
    {
		if(isinf(Laser.ranges[temp]))
		{
			
		}
		else if(Laser.ranges[temp] <= barriar_threshols)
		{
				barrier_num++;
				s_barrier[barrier_num].direc_start = temp;
				s_barrier[barrier_num].distan_min = Laser.ranges[temp];
				s_barrier[barrier_num].distan_start = Laser.ranges[temp];
				distan_min_direc = temp;
			while(1)
			{
				//cout << "Laser.ranges[temp] : " << Laser.ranges[temp] << endl;
				temp++;
				if(s_barrier[barrier_num].distan_min > Laser.ranges[temp])
				{
					s_barrier[barrier_num].distan_min = Laser.ranges[temp];
					distan_min_direc = temp;
					//cout << "s_barrier[barrier_num].distan_min : " << s_barrier[barrier_num].distan_min << endl;
				}
				if(Laser.ranges[temp] > barriar_threshols || temp >= 359)
				{
					s_barrier[barrier_num].direc_stop = temp;
					s_barrier[barrier_num].distan_stop = Laser.ranges[temp-1];
					//余弦定理
					s_barrier[barrier_num].size =  sqrt(s_barrier[barrier_num].distan_start*s_barrier[barrier_num].distan_start+s_barrier[barrier_num].distan_stop*s_barrier[barrier_num].distan_stop-2*s_barrier[barrier_num].distan_stop*s_barrier[barrier_num].distan_start*cos((s_barrier[barrier_num].direc_stop- s_barrier[barrier_num].direc_start)*pi/180));
					s_barrier[barrier_num].frame_x = -s_barrier[barrier_num].distan_min * cos(distan_min_direc/180*3.1415926);
					s_barrier[barrier_num].frame_y = -s_barrier[barrier_num].distan_min * sin(distan_min_direc/180*3.1415926);
					s_barrier[barrier_num].world_x = s_barrier[barrier_num].frame_x + pos_drone[0];
					s_barrier[barrier_num].world_y = s_barrier[barrier_num].frame_y + pos_drone[1];
					//点到直线距离公式
					float k = (pos_drone[1] - target_y)/(pos_drone[0] - target_x);
					s_barrier[barrier_num].distan_min_vert_range = (s_barrier[barrier_num].world_y-k*s_barrier[barrier_num].world_x+k*target_x-target_y)/sqrt(1+k*k);
					//勾股定理
					s_barrier[barrier_num].distan_min_away_target = sqrt((s_barrier[barrier_num].world_x-target_x)*(s_barrier[barrier_num].world_x-target_x)+(s_barrier[barrier_num].world_y-target_y)*(s_barrier[barrier_num].world_y-target_y));
					break;
				}
			}
		}
		else
		{
			
		}
	
    }	
	/*避障策略*/
	if(barrier_num >0)
	{	
		//cout << "barrier_num : " << barrier_num  <<endl;
		float barrier_distan_min = 1;
		int barrier_temp = 0;
		for(int temp=1;temp<=barrier_num;temp++)
		{
			//cout << "barrier_num : " << barrier_num  <<endl;
			uav2targetDistan = sqrt((target_x - pos_drone[0])*(target_x - pos_drone[0]) + (target_y - pos_drone[1])*(target_y - pos_drone[1]));//飞机与目标点之间的距离
			if(s_barrier[temp].distan_min_vert_range < 1 && s_barrier[temp].distan_min_vert_range > -1 && s_barrier[temp].distan_min_away_target < uav2targetDistan)//找到满足条件的障碍物点
			{
				// cout << "temp : " << temp <<endl;
				if(barrier_distan_min > s_barrier[temp].distan_min)//寻找离飞机最近的障碍物，前提要障碍物据飞机与目标点直线距离不小于1m,而且障碍物在飞机的前方。
				{
					barrier_distan_min = s_barrier[temp].distan_min;
					barrier_temp = temp;
   					cout << "barrier_temp : " << barrier_temp <<endl;
				}
				if (xiangxian == 1 || xiangxian == 2 || xiangxian == 3 || xiangxian == 4) 
				{
					if (s_barrier[temp].distan_min_vert_range > -0.2 && s_barrier[temp].distan_min_vert_range < 0.2 )
				    {
				 	   s_barrier[temp].distan_min_vert_range = abs(s_barrier[temp].distan_min_vert_range);
				    }
				}
				
			}

		}
		if(barrier_temp > 0 && xiangxian == 1)
		{
 			if ((s_barrier[barrier_temp].distan_min_vert_range > 0.2)&&(s_barrier[barrier_temp].distan_min_vert_range < 0.8))		//正负代表障碍物在uav到目标点的直线的哪一侧
			{
				target_temp_x =  s_barrier[barrier_temp].world_x+1/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"11111111111111111111111111111111" <<endl;
			}
			else if ((s_barrier[barrier_temp].distan_min_vert_range < -0.2)&&(s_barrier[barrier_temp].distan_min_vert_range > -0.8))
			{
				target_temp_x = s_barrier[barrier_temp].world_x-1/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"2222222222222222222222222222222222" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range < 0.2)&&((s_barrier[barrier_temp].distan_min_vert_range > 0)))
			{
				target_temp_x =  s_barrier[barrier_temp].world_x+1/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"333333333333333333333333333333" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range > -0.2)&&((s_barrier[barrier_temp].distan_min_vert_range < 0)))
			{
				target_temp_x = s_barrier[barrier_temp].world_x-1/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"4444444444444444444444444444444444" <<endl;
			}

			
			
			// vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
			// vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);	
			// cout <<"55555555555555555555555555555555555" <<endl;
			// cout << "target_temp_x : " << target_temp_x << " m "<<endl;
			// cout << "s_barrier[barrier_temp].world_x : " << s_barrier[barrier_temp].world_x << " m "<<endl;
    		// cout << "target_temp_y : " << target_temp_y << " m "<<endl;	
			// cout << "s_barrier[barrier_temp].world_y : " << s_barrier[barrier_temp].world_y << " m "<<endl;	
			// cout << "s_barrier[barrier_temp].distan_min_vert_range " << s_barrier[barrier_temp].distan_min_vert_range << " m "<<endl;
		}
		else if(barrier_temp > 0 && xiangxian == 2)
		{
 			if ((s_barrier[barrier_temp].distan_min_vert_range > 0.2)&&(s_barrier[barrier_temp].distan_min_vert_range < 0.8))		//正负代表障碍物在uav到目标点的直线的哪一侧
			{
				target_temp_x =  s_barrier[barrier_temp].world_x-1/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"11111111111111111111111111111111" <<endl;
			}
			else if ((s_barrier[barrier_temp].distan_min_vert_range < -0.2)&&(s_barrier[barrier_temp].distan_min_vert_range > -0.8))
			{
				target_temp_x = s_barrier[barrier_temp].world_x+1/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"2222222222222222222222222222222222" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range < 0.2)&&((s_barrier[barrier_temp].distan_min_vert_range > 0)))
			{
				target_temp_x =  s_barrier[barrier_temp].world_x+5/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y+5/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"333333333333333333333333333333" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range > -0.2)&&((s_barrier[barrier_temp].distan_min_vert_range < 0)))
			{
				target_temp_x = s_barrier[barrier_temp].world_x-5/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y-5/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"4444444444444444444444444444444444" <<endl;
			}
		}
		else if(barrier_temp > 0 && xiangxian == 3)
		{
 			if ((s_barrier[barrier_temp].distan_min_vert_range > 0.2)&&(s_barrier[barrier_temp].distan_min_vert_range < 0.8))		//正负代表障碍物在uav到目标点的直线的哪一侧
			{
				target_temp_x =  s_barrier[barrier_temp].world_x+1/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"11111111111111111111111111111111" <<endl;
			}
			else if ((s_barrier[barrier_temp].distan_min_vert_range < -0.2)&&(s_barrier[barrier_temp].distan_min_vert_range > -0.8))
			{
				target_temp_x = s_barrier[barrier_temp].world_x-1/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"2222222222222222222222222222222222" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range < 0.2)&&((s_barrier[barrier_temp].distan_min_vert_range > 0)))
			{
				target_temp_x =  s_barrier[barrier_temp].world_x-5/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y+5/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"333333333333333333333333333333" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range > -0.2)&&((s_barrier[barrier_temp].distan_min_vert_range < 0)))
			{
				target_temp_x = s_barrier[barrier_temp].world_x+5/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y-5/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"4444444444444444444444444444444444" <<endl;
			}
		}
		else if(barrier_temp > 0 && xiangxian == 4)
		{
 			if ((s_barrier[barrier_temp].distan_min_vert_range > 0.2)&&(s_barrier[barrier_temp].distan_min_vert_range < 0.8))		//正负代表障碍物在uav到目标点的直线的哪一侧
			{
				target_temp_x =  s_barrier[barrier_temp].world_x-1/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"11111111111111111111111111111111" <<endl;
			}
			else if ((s_barrier[barrier_temp].distan_min_vert_range < -0.2)&&(s_barrier[barrier_temp].distan_min_vert_range > -0.8))
			{
				target_temp_x = s_barrier[barrier_temp].world_x+1/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+1/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"2222222222222222222222222222222222" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range < 0.2)&&((s_barrier[barrier_temp].distan_min_vert_range > 0)))
			{
				target_temp_x =  s_barrier[barrier_temp].world_x-5/s_barrier[barrier_temp].distan_min;
				target_temp_y =  s_barrier[barrier_temp].world_y-5/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"333333333333333333333333333333" <<endl;
			}
			else if((s_barrier[barrier_temp].distan_min_vert_range > -0.2)&&((s_barrier[barrier_temp].distan_min_vert_range < 0)))
			{
				target_temp_x = s_barrier[barrier_temp].world_x+5/s_barrier[barrier_temp].distan_min;
				target_temp_y = s_barrier[barrier_temp].world_y+5/s_barrier[barrier_temp].distan_min;
				vel_sp_body[0] = 0.7 * (target_temp_x - pos_drone[0]);
				vel_sp_body[1] = 0.7 * (target_temp_y - pos_drone[1]);
				cout <<"4444444444444444444444444444444444" <<endl;
			}
		}

		else
		{
			vel_sp_body[0] = p_xy * (target_x - pos_drone[0]);
			vel_sp_body[1] = p_xy * (target_y - pos_drone[1]);
			cout <<"66666666666666666666666666666666666666" <<endl;
		}
	}
	else
	{
		vel_sp_body[0] = p_xy * (target_x - pos_drone[0]);
		vel_sp_body[1] = p_xy * (target_y - pos_drone[1]);
		cout <<"77777777777777777777777" <<endl;
	}
	
	// cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>s_barrier<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
	if(barrier_num > 0)
	{
		for(int i=1;i<=barrier_num;i++)
		{
			cout << "s_barrier_number : " << i << endl;
			// cout <<	"s_barrier_distan_min : " << s_barrier[i].distan_min << endl;
			// cout <<	"s_barrier_direc_start : " << s_barrier[i].direc_start << endl;
			// cout <<	"s_barrier_direc_stop: " << s_barrier[i].direc_stop << endl;
			// cout <<	"s_barrier_distan_stop: " << s_barrier[i].distan_stop << endl;
			// cout <<	"s_barrier_distan_start: " << s_barrier[i].distan_start << endl;
			// cout <<	"s_barrier_size: " << s_barrier[i].size << endl;
			// cout <<	"s_barrier_frame_x: " << s_barrier[i].frame_x << endl;
			// cout <<	"s_barrier_frame_y: " << s_barrier[i].frame_y << endl;
			// cout <<	"s_barrier_world_x: " << s_barrier[i].world_x << endl;
			// cout <<	"s_barrier_world_y: " << s_barrier[i].world_y << endl;
			cout <<	"s_barrier_distan_min_vert_range: " << s_barrier[i].distan_min_vert_range << endl;
			cout <<	"xiangxian?: " << xiangxian << endl;
			// cout <<	"s_barrier_distan_min_away_target: " << s_barrier[i].distan_min_away_target << endl;
		}
	}

}



void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone = pos_drone_fcu_enu;
}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh("~");

    // 频率 [20Hz]
    ros::Rate rate(20.0);

    //【订阅】Lidar数据
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);

    //【订阅】无人机当前位置 坐标系 NED系
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<px4_command::command>("/px4/command", 10);

    //读取参数表中的参数
    nh.param<float>("target_x", target_x, 1.0);
    nh.param<float>("target_y", target_y, 1.0);
    nh.param<float>("p_xy", p_xy, 0.5);
    nh.param<float>("vel_sp_max", vel_sp_max, 0.0);
    if(target_x>0 && target_y>0)
	{
		xiangxian = 1;
	}
	else if (target_x<0 && target_y>0)
	{
		xiangxian = 2;
	}
	else if (target_x<0 && target_y<0)
	{
		xiangxian = 3;
	}
	else if (target_x>0 && target_y<0)
	{
		xiangxian = 4;
	}
	else if (target_x>0 && target_y ==0)
	{
		xiangxian = 1;
	}
	else if (target_x==0 && target_y >0)
	{
		xiangxian = 2;
	}
	else if (target_x<0 && target_y ==0)
	{
		xiangxian = 3;
	}
	else if (target_x==0 && target_y <0)
	{
		xiangxian = 4;
	}

    // nh.param<float>("R_outside", R_outside, 2);
    // nh.param<float>("R_inside", R_inside, 1);
    // nh.param<float>("vel_track_max", vel_track_max, 0.5);
    // nh.param<float>("p_R", p_R, 0.0);
    // nh.param<float>("p_r", p_r, 0.0);
    // nh.param<float>("vel_collision_max", vel_collision_max, 0.0);
    // nh.param<int>("range_min", range_min, 0.0);
    // nh.param<int>("range_max", range_max, 0.0);


    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    //初值
    // vel_track[0]= 0;
    // vel_track[1]= 0;

    // vel_collision[0]= 0;
    // vel_collision[1]= 0;

    vel_sp_body[0]= 0;
    vel_sp_body[1]= 0;

    //四向最小距离 初值
    flag_land = 0;

    //输出指令初始化
    int comid = 1;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        //1. 更新雷达点云数据，存储在Laser中,并计算四向最小距离
        ros::spinOnce();

        // //2. 根据最小距离判断：是否启用避障策略
        // if (distance_c >= R_outside )
        // {
        //     flag_collision_avoidance.data = false;
        // }
        // else
        // {
        //     flag_collision_avoidance.data = true;
        // }

        // //3. 计算追踪速度
        // vel_track[0] = p_xy * (target_x - pos_drone[0]);
        // vel_track[1] = p_xy * (target_y - pos_drone[1]);

        // //速度限幅
        // for (int i = 0; i < 2; i++)
        // {
        //     vel_track[i] = satfunc(vel_track[i],vel_track_max);
        // }
        // vel_collision[0]= 0;
        // vel_collision[1]= 0;

        // //4. 避障策略
        // if(flag_collision_avoidance.data == true)
        // {
        //     distance_cx = distance_c * cos(angle_c/180*3.1415926);
        //     distance_cy = distance_c * sin(angle_c/180*3.1415926);

        //     distance_cx = - distance_cx;

        //     float F_c;

        //     F_c = 0;

        //     if(distance_c > R_outside)
        //     {
        //         //对速度不做限制
        //         vel_collision[0] = vel_collision[0] + 0;
        //         vel_collision[1] = vel_collision[1] + 0;
        //         cout << " Forward Outside "<<endl;
        //     }

        //     //小幅度抑制移动速度
        //     if(distance_c > R_inside && distance_c <= R_outside)
        //     {
        //         F_c = p_R * (R_outside - distance_c);

        //     }

        //     //大幅度抑制移动速度
        //     if(distance_c <= R_inside )
        //     {
        //         F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        //     }

        //     if(distance_cx > 0)
        //     {
        //         vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        //     }else{
        //         vel_collision[0] = vel_collision[0] - F_c * distance_cx /distance_c;
        //     }

        //     if(distance_cy > 0)
        //     {
        //         vel_collision[1] = vel_collision[1] - F_c * distance_cy / distance_c;
        //     }else{
        //         vel_collision[1] = vel_collision[1] - F_c * distance_cy /distance_c;
        //     }


        //     //避障速度限幅
        //     for (int i = 0; i < 2; i++)
        //     {
        //         vel_collision[i] = satfunc(vel_collision[i],vel_collision_max);
        //     }
        // }

        // vel_sp_body[0] = vel_track[0] + vel_collision[0];
        // vel_sp_body[1] = vel_track[1] + vel_collision[1];

        for (int i = 0; i < 2; i++)
        {
            vel_sp_body[i] = satfunc(vel_sp_body[i],vel_sp_max);
        }

        //5. 发布Command指令给position_controller.cpp
        Command_now.command = Move_Body;     //机体系下移动
        Command_now.comid = comid;
        comid++;
        Command_now.sub_mode = 2; // xy 速度控制模式 z 位置控制模式
        Command_now.vel_sp[0] =  vel_sp_body[0];
        Command_now.vel_sp[1] =  vel_sp_body[1];  //ENU frame
        Command_now.pos_sp[2] =  0;
        Command_now.yaw_sp = 0 ;

        float abs_distance;
        abs_distance = sqrt((pos_drone[0] - target_x) * (pos_drone[0] - target_x) + (pos_drone[1] - target_y) * (pos_drone[1] - target_y));

        if(abs_distance < 0.3 || flag_land == 1)
        {
            Command_now.command = 3;     //Land
            flag_land = 1;
        }

        command_pub.publish(Command_now);

        //打印
        printf();

        rate.sleep();

    }

    return 0;

}


// //计算前后左右四向最小距离
// void cal_min_distance()
// {

//     distance_c = Laser.ranges[range_min];
//     angle_c = 0;
//     for (int i = range_min; i <= range_max; i++)
//     {
//         if(Laser.ranges[i] < distance_c)
//         {
//             distance_c = Laser.ranges[i];
//             angle_c = i;
//         }
//     }


// }


//饱和函数
float satfunc(float data, float Max)
{
    if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}


void printf()
{

//    cout <<  "uav2targetDistan :" << uav2targetDistan << "m" <<endl;
//   cout << "vel_sp_x : " << vel_sp_body[0] << " [m/s] "<<endl;
//    cout << "vel_sp_y : " << vel_sp_body[1] << " [m/s] "<<endl;
	

}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "target_x : "<< target_x << endl;
    cout << "target_y : "<< target_y << endl;
	cout << "target_z : "<< target_z << endl;

    cout << "p_xy : "<< p_xy << endl;
	cout << "p_z : "<< p_z << endl;

    cout << "vel_sp_max : "<< vel_sp_max << endl;

}


