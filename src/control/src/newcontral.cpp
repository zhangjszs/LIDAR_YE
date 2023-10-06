#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Point.h"
#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/HUAT_Carstate.h"
#include "common_msgs/HUAT_ControlCommand.h"
#include "common_msgs/HUAT_PathLimits.h"
#include "common_msgs/HUAT_VehcileCmd.h"

using namespace std;
#define l 1.55
float lookahead = 2.1; // 前视距离
float current_speed, long_error, long_current, sum_error;

vector<double> ghe;
vector<double> refx;
vector<double> refy;
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
        head1 = 0XAA;
        head2 = 0X55;
        my_steering = 90;
        my_brake_force = 0;
        my_pedal_ratio = 0;
        my_gear_position = 0;
        my_working_mode = 0;
        my_racing_num = 0;
        my_racing_status = 0;
        my_checksum = 100;
        pub_finall_cmd = nh.advertise<common_msgs::HUAT_VehcileCmd>("vehcileCMDMsg", 100);
        sub = nh.subscribe("/Carstate", 10, &PPControl::posecallback, this);                                  // 回调函数在此只是一个声明，只有遇到ros::spin()或ros::spinOnce()才开始处理被调用的数据
        sub_path = nh.subscribe("/skidpad_detection_node/log_path", 100, &PPControl::locationcallback, this); // 订阅轨迹信息
        sub_approachingGoalPub = nh.subscribe("/skidpad_detection_node/approaching_goal", 100, &PPControl::approachingGoalPubcallback, this);
    }

public:
    void approachingGoalPubcallback(const std_msgs::Bool::ConstPtr &msg)
    {
        approachingGoalPub = msg->data;
        std::cout << "approachingGoalPub = " << approachingGoalPub << std::endl;
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
            // SavePath(x, y);
            refx.push_back(x);
            refy.push_back(y);
        }
        // 从txt文件中读取数据
        // std::ifstream inputFile("/home/kerwin/LIDAR_ye/src/control/src/Path.txt");
        // if (inputFile.is_open())
        // {
        //     std::string line;
        //     while (std::getline(inputFile, line))
        //     {
        //         std::istringstream iss(line);
        //         double x, y;
        //         if (!(iss >> x >> y))
        //         {
        //             ROS_ERROR("Error reading data from file.");
        //             break;
        //         }
        //         refx.push_back(x);
        //         refy.push_back(y);
        //     }
        //     inputFile.close();
        // }
        pathmode++;
    }

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
            cout << "追踪的点:refx_f = " << refx[idx] << " refy_r  =" << refy[idx] << endl;
        return idx;
    }

    void controlcommand(common_msgs::HUAT_ControlCommand &cmd, common_msgs::HUAT_VehcileCmd &finall_cmd)
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
                delta = (delta * 1.2 + fangle * 0.8) / 2.0;
            }
            fangle = delta;
            cmd.steering_angle.data = delta;
            // SaveAlpha( refx[goal_idx],  refy[goal_idx],  gx,  gy,  alpha,  delta);
            cout << "delta = " << delta << endl;
            my_steering = int(cmd.steering_angle.data * 180 / M_PI * 3.73) + 110; // 看旧工控机代码
            cout << "my_steering " << my_steering << endl;                        // 转角
            cout << "current_speed is: " << current_speed << endl;
            long_error = 4.0 - current_speed;
            sum_error += long_error;
            long_current = 0.5 * long_error + 0.1 * sum_error;
            if (current_speed <= 0.8)
            {
                long_current = 5; // 50
            }
            else if (current_speed > 3)
            {
                long_current = 5; // 10
            }
            else if (long_current > 30)
            {
                long_current = 5; // 30
            }
            cmd.throttle.data = int(long_current);
            cout << "cmd.throttle.data = " << cmd.throttle.data;
            my_pedal_ratio = int(cmd.throttle.data); // 踏板率
            cout << "my_pedal_ratio " << my_pedal_ratio << endl;

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
                if ((goal_idx >= refx.size() - 1) || approachingGoalPub)
                {
                    my_steering = 110;
                    my_pedal_ratio = 0;
                    my_brake_force = 80;
                    my_racing_status = 4;
                    cout << "已经到达离线轨迹的最后一个点或者接近终点，停止跟踪！" << endl;
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
        else
        {
            cout << "得不到有效的惯导路径信息" << endl;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub, sub_partial_path, sub_cmd, sub_full_path, stopSub;
    ros::Publisher pub_cmd, pub_finall_cmd;
    double heading, gx, gy, gz;
    common_msgs::HUAT_VehcileCmd vehcile_cmd;
    // 速度  刹车力 踏板比例 齿轮位置 工作模式 比赛号码 赛车状态
    int head1, head2, my_steering, my_brake_force, my_pedal_ratio, my_gear_position, my_working_mode, my_racing_num, my_racing_status;
    long my_checksum;   // 检查数
    int input_mode = 3; // 输入模式 分为1 2 3 4测试，直线，八字，高速
    bool imuflag = true;
    Eigen::Affine3d localTf_;
    bool localTfValid_ = false;
    double fangle = 0;
    double nowangle = 0;
    int pathmode = 0;
    bool approachingGoalPub = false;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "control");
    PPControl car;
    common_msgs::HUAT_ControlCommand cc;
    common_msgs::HUAT_VehcileCmd a;
    ros::Rate rate(10);
    ros::Duration(1).sleep(); // 是否需要这个
    while (ros::ok())
    {
        ros::spinOnce();
        car.controlcommand(cc, a);
        rate.sleep();
    }
    return 0;
}