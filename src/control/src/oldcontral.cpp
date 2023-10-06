#include <ros/ros.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>

#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/HUAT_Carstate.h"
#include "common_msgs/HUAT_ControlCommand.h"
#include "common_msgs/HUAT_stop.h"
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
        sub = nh.subscribe("/Carstate", 10, &PPControl::posecallback, this); // 回调函数在此只是一个声明，只有遇到ros::spin()或ros::spinOnce()才开始处理被调用的数据
        sub_partial_path = nh.subscribe("/AS/P/pathlimits/partial", 10, &PPControl::locationcallback, this);
        sub_full_path = nh.subscribe("/AS/P/pathlimits/full", 10, &PPControl::fullCallBack, this);
        stopSub = nh.subscribe("/stopTheCar", 10, &PPControl::stopCallBack, this);
    }

public:
    void fullCallBack(const common_msgs::HUAT_PathLimits::ConstPtr &msgs)
    {
        // fstream f;
        // f.open("/home/ros/tracking/src/control/src/delta.txt",ios::app);
        // f<<"full****************"<<endl;
        // f.close();
        refx.clear();
        refy.clear();
        for (int i = 0; i < msgs->path.size(); i++)
        {
            refx.push_back(msgs->path[i].x);
            refy.push_back(msgs->path[i].y);
        }
    }

    void locationcallback(const common_msgs::HUAT_PathLimits::ConstPtr &msgs)
    {
        // if (not this->localTfValid_) {
        //     ROS_WARN("[PP_car] CarState not being received.");
        //     return;
        // }
        refx.clear();
        refy.clear();
        for (int i = 0; i < msgs->path.size(); i++)
        {
            refx.push_back(msgs->path[i].x);
            refy.push_back(msgs->path[i].y);
        }
        reciveMsg = true;
    }

    // // 已经用不上
    void posecallback(const common_msgs::HUAT_Carstate::ConstPtr &msgs)
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
        // fstream f;
        // f.open("/home/ros/tracking/src/control/src/refx&refy.txt",ios::app);
        // f<<"x = " << gx << "\ty = " <<gy <<endl;
        // f.close();
    }

    void stopCallBack(const common_msgs::HUAT_stop::ConstPtr &msgs)
    {
        if (msgs->stop)
            stop_ = true;
    }

    Point getNextPointyzh()
    { // 这一步是为什么  为什么要这样去找这个前瞻点   测试下这个代码的准确性
        Point p;
        int idx = 0;
        int temp_idx = 0;
        double lastDist = 0;
        for (int i = 0; i < refx.size(); i++)
        {
            if (refx[i] > 0)
            {
                float dis = sqrt(pow(refy[i], 2) + pow(refx[i], 2));
                if (dis <= lookahead)
                {
                    if (dis > lastDist)
                    {
                        lastDist = dis;
                        temp_idx = i;
                    }
                    temp_idx = i;
                }
            }

            // else{
            // 	break;
            // }
        }
        p.x = refx[temp_idx];
        p.y = refy[temp_idx];
        getPoint = true;
        return p;
    }

    Point getNextPoint()
    { // 这一步是为什么  为什么要这样去找这个前瞻点
        Point p;
        for (int i = 0; i < refx.size(); i++)
        {
            if (refx[i] > 1.2 && refx[i] < 2 && abs(refy[i]) < 2)
            {
                p.x = refx[i];
                p.y = refy[i];
                if (abs(pow(lookahead, 2) - pow(p.x, 2) - pow(p.y, 2)) < 1)
                {
                    break;
                }
            }
        }
        getPoint = true;
        return p;
    }

    void controlcommand(common_msgs::HUAT_ControlCommand &cmd, common_msgs::HUAT_VehcileCmd &finall_cmd)
    {
        if (!reciveMsg)
        {
            return;
        }
        float delta_max = 0.4;
        Point p = getNextPointyzh();                                                         // 这个在干什么
        Eigen::Vector3d product = this->localTf_.inverse() * Eigen::Vector3d(p.x, p.y, 0.0); // 调试
        fstream f;
        f.open("/home/tb/tracking/src/control/src/refx&refy.txt", ios::app);
        f << product.x() << "\t" << product.y() << endl;
        f.close();
        // if (goal_idx >= 0 && goal_idx < refx.size() && goal_idx < refy.size())
        if (getPoint)
        {
            // float alpha = atan2( goalY , goalX );   //单纯只靠这个 不一定可行  参考北理开源
            float alpha = atan2(p.y, p.x);
            alpha = (alpha > M_PI) ? (alpha - 2 * M_PI) : (alpha < -M_PI) ? (alpha + 2 * M_PI)
                                                                          : alpha;
            cout << "alpha = " << alpha << endl;
            float delta = atan2(3.1 * sin(alpha), lookahead); // 公式为什么这么写
            // float delta = atan2(2*l* sin(alpha) / lookahead, 1.0);  这是我写的考虑这个  现场调试
            if (delta > M_PI)
            {
                delta -= 2 * M_PI;
            }
            else if (delta < -M_PI)
            {
                delta += 2 * M_PI;
            }
            delta = max(min(delta_max, delta), -delta_max);
            if (abs(delta - last_delta) <= 0.1)
            {
                last_delta = delta;
            }
            else
            {
                delta = (last_delta * 1.4 + delta * 0.6) / 2.0;
                last_delta = delta;
            } // 防止突变
            cmd.steering_angle.data = delta;
            cout << "delta = " << delta << endl;
            fstream f;
            f.open("/home/tb/tracking/src/control/src/delta.txt", ios::app);
            f << delta << "\t" << p.x << "\t" << p.y << endl;
            f.close();
            my_steering = int(cmd.steering_angle.data * 180 / M_PI * 3.73) + 110; // 看旧工控机代码
            cout << "my_steering " << my_steering << endl;                        // 转角
            // fstream f;
            f.open("/home/tb/tracking/src/control/src/steering.txt", ios::app);
            f << my_steering << endl;
            f.close();

            cout << "current_speed is: " << current_speed << endl;
            long_error = 3.0 - current_speed; // 速度还是需要调
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

            if (input_mode == 4)
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
                if (stop_)
                {
                    my_steering = 110;
                    my_pedal_ratio = 0;
                    my_brake_force = 80;
                    my_racing_status = 4;
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
    int input_mode = 4; // 输入模式 分为1 2 3 4测试，直线，八字，高速
    bool imuflag = true;
    double last_delta;
    bool stop_ = false;
    bool getPoint = false;
    bool reciveMsg = false; // 获取轨迹标志位

    /**
     * @brief The transform between global and local frame.
     */
    Eigen::Affine3d localTf_;

    /**
     * @brief Whether or not \a localTf_ is valid.
     */
    bool localTfValid_ = false;
};

int main(int argc, char **argv)
{
    fstream f;
    f.open("/home/tb/tracking/src/control/src/delta.txt", std::ios::out | std::ios::trunc);
    f.close();
    f.open("/home/tb/tracking/src/control/src/refx&refy.txt", std::ios::out | std::ios::trunc);
    f.close();
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