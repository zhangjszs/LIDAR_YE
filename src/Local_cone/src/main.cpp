/**
 * @file main.cpp
 * @author yzh (631230001@qq.com)
 * @brief  Complete the global coordinate acquisition of the cone barrel
 * @version 1.0
 * @date 2023-05-31
 * 
 * @copyright Copyright (c) 2023 HUAT-FSAC
 */
#include <ros/ros.h>                                                                             
#include "common_msgs/HUAT_ASENSING.h"                         
#include "common_msgs/HUAT_cone.h"     
#include "common_msgs/Cone.h"
#include "common_msgs/HUAT_Carstate.h"
#include "common_msgs/HUAT_map.h"

#define PII 3.14159265358979                     

double  enu_xyz[3]; 
double first_lat, first_lon, first_alt;
bool isFirstMsgReceived = false;            //看车状态回调是否更新
int idx = 0;   //锥筒全局变量唯一值



ros::Publisher conepub;
ros::Publisher INSpub;
ros::Publisher Yemap;
common_msgs::HUAT_Carstate Carstate;
common_msgs::HUAT_ASENSING my_ins;
common_msgs::Cone Dcone;
common_msgs::HUAT_cone Ycone;
common_msgs::HUAT_map  Ymap;
void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
{
    double a, b, f, e_sq;
	 a = 6378137;
	 b = 6356752.3142;
	f = (a - b) / a;e_sq = f * (2 - f);
	// 站点（非原点）
	double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
	lamb = lat;  
	phi = lon;
	s = sin(lamb);
	N = a / sqrt(1 - e_sq * s * s);

	sin_lambda = sin(lamb);
	cos_lambda = cos(lamb);
	sin_phi = sin(phi);
	cos_phi = cos(phi);

	x = (h + N) * cos_lambda * cos_phi;
	y = (h + N) * cos_lambda * sin_phi;
	z = (h + (1 - e_sq) * N) * sin_lambda;
	// 原点坐标转换
	double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
	lamb0 = lat0;
	phi0 = lon0;
	s0 = sin(lamb0);
	N0 = a / sqrt(1 - e_sq * s0 * s0);

	sin_lambda0 = sin(lamb0);
	cos_lambda0 = cos(lamb0);
	sin_phi0 = sin(phi0);
	cos_phi0 = cos(phi0);

	x0 = (h0 + N0) * cos_lambda0 * cos_phi0;
	y0 = (h0 + N0) * cos_lambda0 * sin_phi0;
	z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0;
	// ECEF 转 ENU
	double xd, yd, zd, t;
	xd = x - x0;
	yd = y - y0;
	zd = z - z0;
	t = -cos_phi0 * xd - sin_phi0 * yd;

	enu_xyz[0] = -sin_phi0 * xd + cos_phi0 * yd;
	enu_xyz[1] = t * sin_lambda0 + cos_lambda0 * zd;
	enu_xyz[2] = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd;
    Carstate.car_state.x = enu_xyz[0];
    Carstate.car_state.y = enu_xyz[1];
    INSpub.publish(Carstate);
    isFirstMsgReceived = true;

}



 void doINSMsg(const common_msgs::HUAT_ASENSING::ConstPtr &msgs){
     ROS_DEBUG("进入回调函数进行处理");
     my_ins.east_velocity= msgs->east_velocity;
	 my_ins.north_velocity = msgs->north_velocity;
	 my_ins.ground_velocity = msgs->ground_velocity;
     if (!isFirstMsgReceived) {
        Carstate.car_state.theta = (msgs->azimuth)*PII/180;               //后续需要考虑弧度还是角度
        Carstate.V=sqrt(pow(my_ins.east_velocity,2)+pow( my_ins.north_velocity,2)+pow(my_ins.ground_velocity,2));
        first_lat = msgs->latitude;
        first_lon = msgs->longitude;
        first_alt = msgs->altitude;
        isFirstMsgReceived = true;
    } else {
        Carstate.car_state.theta = (msgs->azimuth)*PII/180;               //后续需要考虑弧度还是角度
        Carstate.V=sqrt(pow(my_ins.east_velocity,2)+pow( my_ins.north_velocity,2)+pow(my_ins.ground_velocity,2));
        GeoDetic_TO_ENU((msgs->latitude) * PII / 180, (msgs->longitude) * PII / 180, msgs->altitude,
                            first_lat * PII / 180, first_lon * PII / 180, first_alt, &enu_xyz[0]);
    }
}


 void doConeMsg(const common_msgs::Cone::ConstPtr &msgs){
        ROS_DEBUG("进入锥筒回调函数进行处理");
        // 检查INS数据是否已经更新
       if (!isFirstMsgReceived)
      {
        ROS_WARN("INS 数据没更新!");
        return;
      }
       for (int i = 0; i < msgs->points.size(); ++i) {                                                          //先这样写   不一定是这样
              Ycone.position_baseLink.x = msgs->points[i].x;
              Ycone.position_baseLink.y = msgs->points[i].y;
              Ycone.position_global.x =  msgs->points[i].x+Carstate.car_state.x;
              Ycone.position_global.y =  msgs->points[i].y+Carstate.car_state.y;
              Ycone.id = idx++;
              Ymap.cone.push_back(Ycone);
       }
        conepub.publish(Ycone);
        Yemap.publish(Ymap);

 }

 int main(int argc, char **argv) {
        setlocale(LC_ALL,"");
        ros::init(argc, argv, "Loacl_cone");
        ros::NodeHandle n;
        ros::Subscriber INSsub = n.subscribe<common_msgs::HUAT_ASENSING>("/INS/ASENSING_INS",10000,doINSMsg);
        ros::Subscriber conesub = n.subscribe<common_msgs::Cone>("/cone_position",10000,doConeMsg);
        conepub = n.advertise<common_msgs::HUAT_map>("/NEWconeposition", 10000);
        INSpub =  n.advertise<common_msgs::HUAT_Carstate>("/Carstate", 10000);
        Yemap = n.advertise<common_msgs::HUAT_map>("/coneMap",10000);
        ros::Rate rate(1);                                                 // 控制循环频率从而可以控制发布频率       频率越高 程序执行的速度越快
        ros::Duration(1).sleep();                                  //可以更轻松地监视输出。暂停程序1s
        while (ros::ok())
        {  
                  ros::spinOnce();   
                  rate.sleep();                                              //休眠  0.1s
        }
       return 0; 

  }













