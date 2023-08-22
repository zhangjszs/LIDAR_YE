#include <ros/ros.h>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
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
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
using namespace std;

#define l 1.55;
#define zhongxin 0.8525
#define pi 3.141592653589779
double enu_xyz[3];
// double first_lat = 32.65343529999999816482;
// double first_lon = 110.72996019999999361971;
// double first_alt = 292.16100000000000136424;

float lookahead = 2.5; // 前视距离
int num_steps = 5;
double dt = 0.01;
float current_speed, long_error, long_current, sum_error;

vector<double> ghe;
vector<double> refx;
vector<double> refy;

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

        pub_cmd = nh.advertise<huat_msgs::HUAT_ControlCommand>("my_topic", 10);
        pub_finall_cmd = nh.advertise<huat_msgs::HUAT_VehcileCmd>("finall_cad", 100);
        sub = nh.subscribe("/Carstate", 10, &PPControl::posecallback, this);                                  // 回调函数在此只是一个声明，只有遇到ros::spin()或ros::spinOnce()才开始处理被调用的数据
        sub_path = nh.subscribe("/skidpad_detection_node/log_path", 100, &PPControl::locationcallback, this); // 订阅轨迹信息
        sub_cmd = nh.subscribe("my_topic", 10, &PPControl::anglecallback, this);
    }

public:
    // void GeoDetic_TO_ENU(double lat, double lon, double h, double lat0, double lon0, double h0, double enu_xyz[3])
    // {
    // double a, b, f, e_sq;
    //  a = 6378137;
    //  b = 6356752.3142;
    // f = (a - b) / a;e_sq = f * (2 - f);
    // // 站点（非原点）
    // double lamb, phi, s, N, sin_lambda, cos_lambda, sin_phi, cos_phi, x, y, z;
    // lamb = lat;
    // phi = lon;
    // s = sin(lamb);
    // N = a / sqrt(1 - e_sq * s * s);

    // sin_lambda = sin(lamb);
    // cos_lambda = cos(lamb);
    // sin_phi = sin(phi);
    // cos_phi = cos(phi);

    // x = (h + N) * cos_lambda * cos_phi;
    // y = (h + N) * cos_lambda * sin_phi;
    // z = (h + (1 - e_sq) * N) * sin_lambda;
    // // 原点坐标转换
    // double lamb0, phi0, s0, N0, sin_lambda0, cos_lambda0, sin_phi0, cos_phi0, x0, y0, z0;
    // lamb0 = lat0;
    // phi0 = lon0;
    // s0 = sin(lamb0);
    // N0 = a / sqrt(1 - e_sq * s0 * s0);

    // sin_lambda0 = sin(lamb0);
    // cos_lambda0 = cos(lamb0);
    // sin_phi0 = sin(phi0);
    // cos_phi0 = cos(phi0);

    // x0 = (h0 + N0) * cos_lambda0 * cos_phi0;
    // y0 = (h0 + N0) * cos_lambda0 * sin_phi0;
    // z0 = (h0 + (1 - e_sq) * N0) * sin_lambda0;
    // // ECEF 转 ENU
    // double xd, yd, zd, t;
    // xd = x - x0;
    // yd = y - y0;
    // zd = z - z0;
    // t = -cos_phi0 * xd - sin_phi0 * yd;

    // enu_xyz[0] = -sin_phi0 * xd + cos_phi0 * yd;
    // enu_xyz[1] = t * sin_lambda0 + cos_lambda0 * zd;
    // enu_xyz[2] = cos_lambda0 * cos_phi0 * xd + cos_lambda0 * sin_phi0 * yd + sin_lambda0 * zd;

    // gx = enu_xyz[0];     //这里的gxgygz是惯导得到的实时的，
    // gy = enu_xyz[1];
    // gz = enu_xyz[2];
    // cout<<"gx=" << gx << " gy= " << gy << endl;
    // }
    void posecallback(const huat_msgs::HUAT_CarState::ConstPtr &msgs)
    {
        geometry_msgs::Pose pose;
        pose.position.x = msgs->car_state.x;
        pose.position.y = msgs->car_state.y;
        pose.position.z = 0;
        tf::Quaternion qAux;
        qAux.setRPY(0.0, 0.0, msgs->car_state.theta);
        tf::quaternionTFToMsg(qAux, pose.orientation);
        tf::poseMsgToEigen(pose, this->localTf_);
        this->localTf_ = this->localTf_.inverse();
        this->localTfValid_ = true;

        gx = msgs->car_state.x;
        gy = msgs->car_state.y;
        current_speed = msgs->V;
        heading = -(msgs->car_state.theta);
        // cout << "gx=" << gx << endl;
        // cout << "gy=" << gy << endl;
        // cout << "heading = " << heading << endl;
        // SaveCarPosition(gx, gy);
        // SaveHeading(heading);
    }

    void locationcallback(const nav_msgs::Path::ConstPtr &msgs)
    {
        if (not this->localTfValid_)
        {
            ROS_WARN("[PP_car] CarState not being received.");
            return;
        }
        refx.clear();
        refy.clear();
        for (const auto &pose : msgs->poses)
        {
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            refx.push_back(x);
            refy.push_back(y);
        }
        pathmode++;
    }

    // refx.push_back(msgs->path.front().x);
    // refy.push_back(msgs->path.front().y);
    // cout << "refx=" << refx.back() <<endl;
    // cout << "refy=" << refy.back() <<endl;

    int get_goal_idx()
    {                                                        // 这里是寻找距离后轮中心最近的点
        double min_distance = numeric_limits<double>::max(); // 初始化为一个较大的值
        int idx = -1;                                        // 初始化为无效索引

        for (int i = 0; i < refx.size(); i++)
        {
            double distance = pow(gx - refx[i], 2) + pow(gy - refy[i], 2);

            if (distance < min_distance)
            {
                min_distance = distance;
                idx = i;
            }
        }
        // if (idx >= 0 && idx < refx.size())
        //     cout << "refx = " << refx[idx] << "refy =" << refy[idx] << endl;
        return idx;
    }
    // 接下来寻找经过前视距离ld0后的最接近的点
    int get_lookahead_indices(int current_idx, double lookahead, const vector<double> &refx, const vector<double> &refy)
    {
        double distance_sum = 0.0;
        int idx = current_idx;

        while (distance_sum < lookahead && idx < refx.size() - 1)
        {
            double dx = refx[idx + 1] - refx[idx];
            double dy = refy[idx + 1] - refy[idx];
            double distance = sqrt(dx * dx + dy * dy);

            if (distance_sum + distance <= lookahead)
            {
                distance_sum += distance;
                idx++;
            }
            else
            {
                break;
            }
        }
        if (idx >= 0 && idx < refx.size())
            cout << "追踪的点：refx_f = " << refx[idx] << " refy_r  =" << refy[idx] << endl;
        return idx;
    }

    void controlcommand(huat_msgs::HUAT_ControlCommand &cmd)
    {
        cout << "车位于路径" << pathmode << endl;
        float delta_max = 0.4;
        int goal_idx = get_goal_idx();
        int lookhead_idx = get_lookahead_indices(goal_idx, lookahead, refx, refy);
        // SaveGoal_idx(lookhead_idx, refx, refy);
        if (goal_idx >= 0 && goal_idx < refx.size() && goal_idx < refy.size())
        {
            Eigen::Vector3d product = localTf_ * Eigen::Vector3d(refx[lookhead_idx], refy[lookhead_idx], 0.0);
            double goalX = product.x();
            double goalY = product.y();
            float alpha = atan2(goalY, goalX);
            // float alpha = atan2(refy[lookhead_idx] - gy, refx[lookhead_idx] - gx) - heading;
            alpha = (alpha > pi) ? (alpha - 2 * pi) : (alpha < -pi) ? (alpha + 2 * pi)
                                                                    : alpha;

            float delta = atan2(3.1 * sin(alpha) / lookahead, 1.0);
            delta = max(min(delta_max, delta), -delta_max);
            if (abs(delta - fangle) > 0.2)
            {
                delta = (delta + fangle * 0.8) / 2.0;
            }
            fangle = delta;
            cmd.steering_angle.data = delta;
            // SaveAlpha( refx[goal_idx],  refy[goal_idx],  gx,  gy,  alpha,  delta);
            cout << "delta = " << delta << endl;

            cout << "current_speed is: " << current_speed << endl;

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

            if (goal_idx >= refx.size() - 1)
            {
                cmd.steering_angle.data = 0;
                cmd.throttle.data = 0;
                cmd.racing_status = 4;
                pub_cmd.publish(cmd);
                cout << "已经到达离线轨迹的最后一个点，停止跟踪！" << endl;
                // ros::shutdown();
            }
        }
        else
        {
            cout << "得不到有效的惯导路径信息" << endl;
        }
    }
    void anglecallback(const huat_msgs::HUAT_ControlCommand::ConstPtr &msgs)
    {

        my_steering = int(msgs->steering_angle.data * 180 / 3.14159265358979 * 3.73) + 110; // 这里的角度数加了110  他这里110才是中心
        cout << "my_steering " << my_steering << endl;                                      // 转角

        my_pedal_ratio = int(msgs->throttle.data); // 踏板率
        cout << "my_pedal_ratio " << my_pedal_ratio << endl;
        // SaveSteering(msgs->steering_angle.data, my_steering, my_pedal_ratio);
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

    void SaveHeading(double heading0)
    {
        ofstream outfile1;
        outfile1.open("heading.txt", ios::out | ios::app);
        outfile1 << "heading " << heading0 << endl; // 以追加的方式写入文件
        outfile1.close();
    }

    void SaveSteering(double steering_angle, int my_steering, int my_pedal_ratio)
    {
        ofstream outfile2;
        outfile2.open("srcsteering.txt", ios::out | ios::app);
        // outfile2 << "steering " << msgs->steering_angle.data << " my_steering " << my_steering << " my_pedal_ratio " << my_pedal_ratio << endl; // 以追加的方式写入文件
        outfile2 << steering_angle << " " << my_steering << " " << my_pedal_ratio << endl; // 以追加的方式写入文件
        outfile2.close();
    }

    void SaveGoal_idx(auto goal_idx, auto refx, auto refy)
    {
        ofstream outfile3;
        outfile3.open("goal_idx.txt", ios::out | ios::app);
        outfile3 << "goal_idx=" << goal_idx << " refx=" << refx[goal_idx] << " refy=" << refy[goal_idx] << endl; // 以追加的方式写入文件
        outfile3.close();
    }

    void SaveAlpha(double refx, double refy, double gx, double gy, double zhuanxiang, double delta)
    {
        ofstream outfile4;
        outfile4.open("alpha.txt", ios::out | ios::app);
        outfile4 << "跟踪点 refx[goal_idx] = " << refx << " refy[goal_idx] = " << refy << endl; // 以追加的方式写入文件
        outfile4 << "车身位置 gx = " << gx << " gy = " << gy << endl;
        outfile4 << "atan2(y,x) = " << zhuanxiang * 180 / 3.14 << endl;
        outfile4 << "delta = " << (delta * 180 / 3.14159265358979 * 3.73) + 110 << endl;
        outfile4.close();
    }
    void SaveCarPosition(double gx, double gy)
    {
        ofstream outfile5;
        outfile5.open("CarPosition.txt", ios::out | ios::app);
        outfile5 << "gx = " << gx << " gy = " << gy << endl;
        outfile5.close();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub, sub_path, sub_cmd;
    ros::Publisher pub_cmd, pub_finall_cmd;
    double heading, gx, gy, gz;
    huat_msgs::HUAT_VehcileCmd vehcile_cmd;
    huat_msgs::HUAT_ControlCommand state;
    // 速度  刹车力 踏板比例 齿轮位置 工作模式 比赛号码 赛车状态
    int my_steering, my_brake_force, my_pedal_ratio, my_gear_position, my_working_mode, my_racing_num, my_racing_status;
    long my_checksum;   // 检查数
    int input_mode = 3; // 输入模式 分为1 2 3 4测试，直线，八字，高速
    double first_lat = 0;
    double first_lon = 0;
    double first_alt = 0;
    bool imuflag = true;
    Eigen::Affine3d localTf_;
    bool localTfValid_ = false;
    double fangle = 0;
    double nowangle = 0;
    int pathmode = 0;
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