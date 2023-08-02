#include <ros/ros.h>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ios>
#include <iterator>
#include <string.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "huat_msgs/HUAT_ASENSING.h"
#include "huat_msgs/HUAT_CarState.h"
#include "huat_msgs/HUAT_ControlCommand.h"
#include "geometry_msgs/Point.h"
#include "huat_msgs/HAUT_PathLimits.h"
#include "huat_msgs/HUAT_VehcileCmd.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdtree++/kdtree.hpp>
using namespace std;

#define l 1.55;
#define zhongxin 0.8525
#define pi 3.141592653589779
#define g 9.8
#define MINNUM 0.0000000001
double enu_xyz[3];
double first_lat = 32.65343529999999816482;
double first_lon = 110.72996019999999361971;
double first_alt = 292.16100000000000136424;

huat_msgs::HUAT_ASENSING car_location;
float lookahead = 2.5; // 前视距离
int num_steps = 5;
double dt = 0.01;
float current_speed, long_error, long_current, sum_error;

vector<double> ghe;
vector<double> refx;
vector<double> refy;
std::vector<std::vector<double>> points;

struct Point
{
    double x;
    double y;
};

class PPControl
{
public:
    PPControl()
    {
        // 初始化
        my_steering = 90;
        my_brake_force = 0;
        my_pedal_ratio = 0;
        my_gear_position = 0;
        my_working_mode = 0;
        my_racing_num = 0;
        my_racing_status = 0;
        my_checksum = 100;

        pub_cmd = nh.advertise<huat_msgs::HUAT_ControlCommand>("my_topic", 1);
        pub_finall_cmd = nh.advertise<huat_msgs::HUAT_VehcileCmd>("finall_cad", 100);
        path_pub = nh.advertise<nav_msgs::Path>("path", 1);
        sub = nh.subscribe("/INS/ASENSING_INS", 100, &PPControl::posecallback, this);                         // 回调函数在此只是一个声明，只有遇到ros::spin()或ros::spinOnce()才开始处理被调用的数据
        sub_path = nh.subscribe("/skidpad_detection_node/log_path", 100, &PPControl::locationcallback, this); // 订阅轨迹信息
        sub_cmd = nh.subscribe("my_topic", 100, &PPControl::anglecallback, this);
        pose_sub_ = nh.subscribe("/Carstate", 1000, &PPControl::positionback, this);
    }

public:
    void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
    {
        // 定义一些常量，表示参考椭球体的参数
        double a, b, f, e_sq;
        a = 6378137;        // 长半轴，单位为米
        b = 6356752.3142;   // 短半轴，单位为米
        f = (a - b) / a;    // 扁率
        e_sq = f * (2 - f); // 第一偏心率平方

        // 计算站点（非原点）的ECEF坐标（地心地固坐标系）
        double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
        lamb = lat;                     // 纬度，单位为度
        phi = lon;                      // 经度，单位为度
        s = sin(lamb);                  // 纬度的正弦值
        N = a / sqrt(1 - e_sq * s * s); // 卯酉圈曲率半径

        sin_lambda = sin(lamb); // 纬度的正弦值
        cos_lambda = cos(lamb); // 纬度的余弦值
        sin_phi = sin(phi);     // 经度的正弦值
        cos_phi = cos(phi);     // 经度的余弦值

        x = (h + N) * cos_lambda * cos_phi;    // x坐标，单位为米
        y = (h + N) * cos_lambda * sin_phi;    // y坐标，单位为米
        z = (h + (1 - e_sq) * N) * sin_lambda; // z坐标，单位为米

        // 计算原点的ECEF坐标（地心地固坐标系）
        double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
        lamb0 = lat0;                      // 原点的纬度，单位为度
        phi0 = lon0;                       // 原点的经度，单位为度
        s0 = sin(lamb0);                   // 原点纬度的正弦值
        N0 = a / sqrt(1 - e_sq * s0 * s0); // 原点卯酉圈曲率半径

        sin_lambda0 = sin(lamb0); // 原点纬度的正弦值
        cos_lambda0 = cos(lamb0); // 原点纬度的余弦值
        sin_phi0 = sin(phi0);     // 原点经度的正弦值
        cos_phi0 = cos(phi0);     // 原点经度的余弦值

        x0 = (h0 + N0) * cos_lambda0 * cos_phi0;   // 原点x坐标，单位为米
        y0 = (h0 + N0) * cos_lambda0 * sin_phi0;   // 原点y坐标，单位为米
        z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0; // 原点z坐标，单位为米

        // 计算站点相对于原点的ECEF坐标差
        double xd, yd, zd, t;
        xd = x - x0;
        yd = y - y0;
        zd = z - z0;

        // 计算站点的ENU坐标（本地东北天坐标系）
        t = -cos_phi0 * xd - sin_phi0 * yd;

        enu_xyz[0] = -sin_phi0 * xd + cos_phi0 * yd;                                               // 东方向坐标，单位为米
        enu_xyz[1] = t * sin_lambda0 + cos_lambda0 * zd;                                           // 北方向坐标，单位为米
        enu_xyz[2] = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd; // 天空方向坐标，单位为米

        // 将计算得到的ENU坐标存储在全局变量gx、gy和gz中
        gx = enu_xyz[0]; // 东方向坐标，单位为米
        gy = enu_xyz[1]; // 北方向坐标，单位为米
        gz = enu_xyz[2]; // 天空方向坐标，单位为米
        cout << "gx=" << gx << " gy=" << gy << " gz=" << gz << endl;
    }

    void positionback(const huat_msgs::HUAT_CarState::ConstPtr &carposition)
    {
        // 提取位置信息
        gx = carposition->car_state.x;
        gy = carposition->car_state.y;
        ljyheading = carposition->car_state.theta;
        cout << "gx=" << gx << " gy=" << gy << endl;
    }

    void posecallback(const huat_msgs::HUAT_ASENSING::ConstPtr &msgs)
    {
        // 将消息中的速度信息赋值给car_location结构体中对应的成员变量
        car_location.east_velocity = msgs->east_velocity;
        car_location.north_velocity = msgs->north_velocity;
        car_location.ground_velocity = msgs->ground_velocity;
        /*double first_lat = msgs->latitude;
        double first_lon = msgs->longitude;
        double first_alt = msgs->altitude;

        double first_lat = 32.65343529999999816482;
        double first_lon = 110.72996019999999361971;
        double first_alt = 292.16100000000000136424;*/

        // 将方位角（azimuth）从角度转换为弧度，并存储到变量heading0中
        double heading0 = (msgs->azimuth) * pi / 180; // 弧度转换成角度
        double heading1 = (heading0 > pi) ? (heading0 - 2 * pi) : (heading0 < -pi) ? (heading0 + 2 * pi)
                                                                                   : heading0;
        // 将heading0的值限制在 -π 到 π 之间，以确保方位角的范围在正常的数值范围内
        // 并将结果赋值给PPControl类的静态成员变量heading
        if (heading1 > 0)
        {
            heading1 -= pi;
        }
        else
        {
            heading1 += pi;
        }
        PPControl::heading = heading1;
        // PPControl::heading = ljyheading;
        // 将原始的方位角（azimuth）值存入名为ghe的容器中，可能用于后续处理或调试
        ghe.push_back(msgs->azimuth);

        // 将经度（longitude）、纬度（latitude）、高度（altitude）转换为ENU局部坐标系中的坐标
        /*GeoDetic_TO_ENU((msgs->latitude) * pi / 180, (msgs->longitude) * pi / 180, msgs->altitude,
                        first_lat * pi / 180, first_lon * pi / 180, first_alt, &enu_xyz[0]);*/

        // 输出转换后的方位角值heading0，方便在控制台上进行调试或观察
        // cout << "first_lat=" << first_lat << " first_lon=" << first_lon << " first_alt=" << first_alt << endl;
        // cout << "heading1= " << heading1 << " heading0= " << heading0 << " heading= " << heading << endl;

        //SaveHeading(heading0, heading, ljyheading);
    }

    void locationcallback(const nav_msgs::Path::ConstPtr &msgs)
    {

        // ofstream outfile;
        // outfile.open("/home/zhangjszs/Pure_Pursuit/src/control/src/refx&refy.txt", ios::out | ios::app);

        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "velodyne"; // Set the frame ID
        for (const auto &pose : msgs->poses)
        {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;

            double transformed_x = x * cos(heading) - y * sin(heading);
            double transformed_y = x * sin(heading) + y * cos(heading);

            // double transformed_x = x;
            // double transformed_y = y;

            refx.push_back(transformed_x);
            refy.push_back(transformed_y);

            // points.push_back({transformed_x, transformed_y});

            // cout << "refx=" << transformed_x << endl;
            // cout << "refy=" << transformed_y << endl;
            // outfile << "refx=" << transformed_x << " refy=" << transformed_y << endl; // 以追加的方式写入文件
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "velodyne";
            pose_stamped.pose.position.x = transformed_x;
            pose_stamped.pose.position.y = transformed_y;
            path_msg.poses.push_back(pose_stamped);
        }
        path_pub.publish(path_msg);
        // outfile.close();
    }

    unsigned int get_goal_idx()
    {

        std::vector<double> distances;
        distances.reserve(refx.size());
        // float sqrtdistances = 0.0;
        for (unsigned int i = 0; i < refx.size(); i++)
        {
            double dx = refx[i] - gx;
            double dy = refy[i] - gy;
            if (dx <= MINNUM && dy <= MINNUM)
            {
                flag = true;
            }
            double sqrtdistances = dx * dx + dy * dy;
            double Angle = atan2(dy, dx);
            if (Angle <= 0 || Angle > 3.1415)
            {
                sqrtdistances = 100000;
            }
            distances.push_back(sqrtdistances);
        }
        auto min_itr = std::min_element(distances.begin(), distances.end());
        unsigned int idx = std::distance(distances.begin(), min_itr);
        // cout << "gx=" << gx << " gy=" << gy << endl;
        // cout << "refx=" << refx[idx] << " refy=" << refy[idx] << endl;
        if (flag)
        {
            idx++;
            flag = false;
        }
        last_index = idx;
        return idx;
    }
    // unsigned int get_goal_idx()
    // {
    //     typedef KDTree::KDTree<2, std::vector<double>> KDTreeType;
    //     KDTreeType kdtree(points.begin(), points.end());
    //     std::vector<double> targetPoint = {gx, gy};

    //     auto nearestNeighbor = kdtree.find_nearest(targetPoint);
    //     size_t idx = std::distance(kdtree.begin(), nearestNeighbor.first);
    //     return idx;
    // }

    void controlcommand(huat_msgs::HUAT_ControlCommand &cmd)
    {
        float delta_max = 0.4;
        int goal_idx = get_goal_idx();

        // SaveGoal_idx(goal_idx, refx, refy);

        if (goal_idx >= 0 && goal_idx < (int)refx.size() && goal_idx < (int)refy.size())
        {
            //float zhuanxiang = atan2(refx[goal_idx] - gx, refy[goal_idx] - gy);
            float zhuanxiang = atan2(refy[goal_idx] - gy, refx[goal_idx] - gx);
            float alpha = zhuanxiang - heading;
            alpha = (alpha > pi) ? (alpha - 2 * pi) : (alpha < -pi) ? (alpha + 2 * pi)
                                                                    : alpha;
            cout << "alpha2=" << alpha << endl;

            // SaveAlpha( refx[goal_idx],  refy[goal_idx],  gx,  gy,  zhuanxiang,  alpha);

            //float delta = atan2(2 * sin(alpha), lookahead);
            float delta = atan(2 * 1.87 *  sin(alpha) / lookahead);
            delta = max(min(delta_max, delta), -delta_max);
            cmd.steering_angle.data = delta;

            current_speed = sqrt(pow(car_location.east_velocity, 2) + pow(car_location.north_velocity, 2) + pow(car_location.ground_velocity, 2));
            cout << "current_speed is: " << current_speed;

            long_error = 4.0 - current_speed;
            sum_error += long_error;
            long_current = 0.5 * long_error + 0.1 * sum_error;

            if (current_speed > 2)
            {
                long_current = 10;
            }
            if (long_current > 30)
            {
                long_current = 30;
            }
            if (current_speed <= 0.3)
            {
                long_current = 50;
            }

            cmd.throttle.data = int(long_current);
            cout << "cmd.throttle.data = " << cmd.throttle.data;
            pub_cmd.publish(cmd);

            if (goal_idx >= (int)(refx.size() - 1))
            {
                cmd.steering_angle.data = 0;
                cmd.throttle.data = 0;
                cmd.racing_status = 4;
                pub_cmd.publish(cmd);
                cout << "已经到达离线轨迹的最后一个点，停止跟踪！" << endl;
                ros::shutdown();
            }
        }
        else
        {
            cout << "得不到有效的惯导路径信息" << endl;
        }
    }

    void vehcile_finall_cmd(huat_msgs::HUAT_VehcileCmd &finall_cmd)
    {
        cout << "read!" << endl;
        if (input_mode == 2)
        {
            my_gear_position = 1; // 齿轮
            my_working_mode = 1;

            if (my_steering < 0)
            {
                my_steering = 0;
            }
            else if (my_steering > 220)
            {
                my_steering = 220;
            }

            if (my_pedal_ratio < 0) // 踏板率
            {
                my_pedal_ratio = 0;
            }
            else if (my_pedal_ratio > 100) // 踏板率
            {
                my_pedal_ratio = 100;
            }

            if (state.racing_status == 4)
            {
                my_racing_status = 4;
                my_brake_force = 80;
            }
            finall_cmd.head1 = 0XAA;
            finall_cmd.head2 = 0X55;
            finall_cmd.length = 10;
            finall_cmd.steering = my_steering;
            finall_cmd.brake_force = my_brake_force;
            finall_cmd.pedal_ratio = my_pedal_ratio;
            finall_cmd.gear_position = my_gear_position;
            finall_cmd.working_mode = my_working_mode;
            finall_cmd.racing_num = my_racing_num;
            finall_cmd.racing_status = my_racing_status;
            finall_cmd.checksum = my_steering + my_brake_force + my_pedal_ratio + my_gear_position + my_working_mode + my_racing_num + my_racing_status;
            pub_finall_cmd.publish(finall_cmd);
        }
    }

    void anglecallback(const huat_msgs::HUAT_ControlCommand::ConstPtr &msgs)
    {

        my_steering = int(msgs->steering_angle.data * 180 / 3.14159265358979 * 3.73) + 110; // 这里的角度数加了110  他这里110才是中心
        cout << "steering " << msgs->steering_angle.data << endl;                           // 没有转换的转角
        cout << "my_steering " << my_steering << endl;                                      // 转角

        my_pedal_ratio = int(msgs->throttle.data); // 踏板率
        cout << "my_pedal_ratio " << my_pedal_ratio << endl;

        // SaveSteering(msgs->steering_angle.data, my_steering, my_pedal_ratio);
    }

    void SaveHeading(double heading0, double heading, double ljyheading)
    {
        ofstream outfile1;
        outfile1.open("/home/zhangjszs/Pure_Pursuit/src/control/src/heading.txt", ios::out | ios::app);
        outfile1 << " heading0= " << heading0 << " heading= " << heading << " ljyheading= " << ljyheading << endl; // 以追加的方式写入文件
        outfile1.close();
    }

    void SaveSteering(double steering_angle, int my_steering, int my_pedal_ratio)
    {
        ofstream outfile2;
        outfile2.open("/home/zhangjszs/Pure_Pursuit/src/control/src/steering.txt", ios::out | ios::app);
        // outfile2 << "steering " << msgs->steering_angle.data << " my_steering " << my_steering << " my_pedal_ratio " << my_pedal_ratio << endl; // 以追加的方式写入文件
        outfile2 << steering_angle << " " << my_steering << " " << my_pedal_ratio << endl; // 以追加的方式写入文件
        outfile2.close();
    }

    void SaveGoal_idx(auto goal_idx, auto refx, auto refy)
    {
        ofstream outfile3;
        outfile3.open("/home/zhangjszs/Pure_Pursuit/src/control/src/goal_idx.txt", ios::out | ios::app);
        outfile3 << "goal_idx=" << goal_idx << " refx=" << refx[goal_idx] << " refy=" << refy[goal_idx] << endl; // 以追加的方式写入文件
        outfile3.close();
    }

    void SaveAlpha(double refx, double refy, double gx, double gy, double zhuanxiang, double alpha)
    {
        ofstream outfile4;
        outfile4.open("/home/zhangjszs/Pure_Pursuit/src/control/src/alpha.txt", ios::out | ios::app);
        outfile4 << "refx[goal_idx]=" << refx << " refy[goal_idx]=" << refy << endl; // 以追加的方式写入文件
        outfile4 << "gx=" << gx << " gy=" << gy << endl;
        outfile4 << "偏转向量=" << zhuanxiang << endl;
        outfile4 << "refx[goal_idx] - gx=" << refx - gx << " refy[goal_idx] - gy=" << refy - gy << endl;
        outfile4 << "alpha1=" << alpha << endl;
        outfile4 << "heading=" << heading << endl;
        outfile4 << "alpha2=" << alpha << endl;
        outfile4.close();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub, sub_path, sub_cmd, pose_sub_;
    ros::Publisher pub_cmd, pub_finall_cmd, path_pub;
    double heading, gx, gy, gz;
    huat_msgs::HUAT_VehcileCmd vehcile_cmd;
    huat_msgs::HUAT_ControlCommand state;
    huat_msgs::HUAT_CarState Carstate;
    // 速度  刹车力 踏板比例 齿轮位置 工作模式 比赛号码 赛车状态
    int my_steering, my_brake_force, my_pedal_ratio, my_gear_position, my_working_mode, my_racing_num, my_racing_status;
    long my_checksum; // 检查数
    int last_index;
    bool flag; // 判断是否需要下一个路径坐标点
    double ljyheading;
    int input_mode = 3; // 输入模式 分为1 2 3 4测试，直线，八字，高速
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");
    PPControl car;
    huat_msgs::HUAT_ControlCommand cc;
    huat_msgs::HUAT_VehcileCmd a;
    ros::Rate rate(10);
    ros::Duration(1).sleep();
    while (ros::ok())
    {
        ros::spinOnce();
        car.controlcommand(cc);
        car.vehcile_finall_cmd(a);
        rate.sleep();
    }
    return 0;
}
