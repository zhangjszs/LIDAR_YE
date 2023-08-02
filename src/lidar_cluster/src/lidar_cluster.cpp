#include "utility.h"
#include <iomanip>
#include <sys/time.h>
#include <unistd.h>
// #include <utility.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <imu_subscriber.hpp>
// #include <distortion_adjust.hpp>
// #include <distortion_adjust.hpp>
using namespace lidar_distortion;



lidar_cluster::lidar_cluster(ros::NodeHandle node,
                 ros::NodeHandle private_nh){
    // buff_mutex_ = true;
    nh_ = node;
    private_nh_ = private_nh;
    // std::cout<<"init()"<<std::endl;
    init();
    // sub_point_cloud_ = nh_.subscribe(input_topic , 1 , &lidar_cluster::point_cb, this);
    // sub_point_cloud_ = nh_.subscribe("/velodyne_points",1, &lidar_cluster::point_cb, this);
    sub_point_cloud_ = nh_.subscribe("/velodyne_points",1, &lidar_cluster::point_cb, this);
    //have problem
    // sub_ins_ = nh_.subscribe("/insMsg",1000000, &IMUData_Subscriber::SetInsInfo, &imu_sub_ptr__);
    markerPub = nh_.advertise<visualization_msgs::MarkerArray>("/BoundingBox",1);
    pub_filtered_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/PassThrough_points", 1);
    pub_filtered_points__ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
    pub_filtered_points___ = nh_.advertise<sensor_msgs::PointCloud2>("/SAC", 1);
    lidarClusterPublisher_ = nh_.advertise<sensor_msgs::PointCloud>("/perception/lidar_cluster",1);
    adjust_check_front = nh_.advertise<sensor_msgs::PointCloud2>("/adjust_check_front", 1);
    adjust_check_back = nh_.advertise<sensor_msgs::PointCloud2>("/adjust_check_back", 1);

    logging_pub = nh_.advertise<nav_msgs::Path>("/log_path",10);
    cone_position = nh_.advertise<common_msgs::Cone>("/cone_position", 1);
    skidpad_detection = nh_.advertise<sensor_msgs::PointCloud2>("/skidpad_detection",1);
    //skidpad_detection = nh_.advertise<common_msgs::Cone>("/skidpad_detection",1);修改在下方
    // ROS_INFO_STREAM("init done;");
}
void lidar_cluster::runAlgorithm(){
  // pcl::StopWatch time;
  // if(!getPointClouds){
  //   // ROS_WARN_STREAM("Not received Point Cloud!");
  //   return ;
  // }
  // getPointClouds = false;
  lidar_mutex.lock();
  // ROS_INFO("DistortionAdjust");
  
  // std::cout<<"size Sync: "<<unsynced_imu_.size()<<"   cloud_time:"<<std::setprecision(20)<<cloud_time<<std::endl;

  // auto startTime = std::chrono::steady_clock::now();
  // auto startTimeWhole = std::chrono::steady_clock::now();
  
  PassThrough(current_pc_ptr,road_type_,z_up_,z_down_);

  // auto endTime = std::chrono::steady_clock::now();
  // auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  // std::cout << "PassThrough took " << elapsedTime.count() << " milliseconds" << std::endl;
  pcl::toROSMsg(*current_pc_ptr, pub_pc);
  pub_pc.header = in_pc.header;
  pub_filtered_points_.publish(pub_pc);
  //ground_Segmentation
  cloud_filtered=current_pc_ptr;
  // startTime = std::chrono::steady_clock::now();
  ground_segmentation(cloud_filtered,g_not_ground_pc);

  // endTime = std::chrono::steady_clock::now();
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  // std::cout << "ground_segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
  pcl::toROSMsg(*g_not_ground_pc, pub_pc);
  pub_pc.header = in_pc.header;
  pub_filtered_points___.publish(pub_pc);

  // startTime = std::chrono::steady_clock::now();
  if(sensor_model_ == 16)
    clusterMethod16();
  else if(sensor_model_ == 32)
    clusterMethod32();

  // endTime = std::chrono::steady_clock::now();
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "cluster took " << elapsedTime.count() << " milliseconds" << std::endl;
  lidarClusterPublisher_.publish(out_pc);
  lidar_mutex.unlock();
  // endTime = std::chrono::steady_clock::now();
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  // elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTimeWhole);
    // std::cout << "whole took " << elapsedTime.count() << " milliseconds" << std::endl;
  // logging_pub.publish(ros_path_);
  
  // logging_pub = n.advertise<nav_msgs::Path>("log_path",10);
}
// std::ofstream out("data_test.txt",std::ios::app);

lidar_cluster::~lidar_cluster(){}
// void lidar_cluster::Spin(){}

void lidar_cluster::point_cb(/*const common_msgs::ins_p2ConstPtr& ins_data,*/ const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    
    // ROS_INFO_STREAM("Velodyne scan received at"<< ros::Time::now());
    lidar_mutex.lock();
    out_pc.points.clear();
    in_pc = *in_cloud_ptr;//save header

    // std::cout<<"runAlgoIn1"<<getPointClouds<<std::endl;
    // pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr_in);
    
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);//out
    
    // std::cout<<"runAlgoIn2"<<getPointClouds<<std::endl;
    out_pc.header.frame_id = "/velodyne";
    out_pc.header.stamp = in_cloud_ptr->header.stamp;
    getPointClouds = true;
    cloud_time = in_cloud_ptr->header.stamp.toSec();
    lidar_mutex.unlock();
    // unsynced_imu_front=
    // disadjust->SetMotionInfo(0.1,synced_data);
    // disadjust->AdjustCloud(current_pc_ptr_in,current_pc_ptr);
    // lidar_cluster::publishToCheckAdjust(current_pc_ptr_in,current_pc_ptr);
    // in_pc = 
}
// void lidar_cluster::transMatrix(const std_msgs::Float64MultiArray &msgs){
//   int element_counter = 0;
//   for (int i = 0; i < 4; i++) {
//     for (int j = 0; j < 4; j++) {
//       transMat_(i, j) = msgs.data[element_counter];
//       element_counter++;
//     }
//   }
// }

void lidar_cluster::publishToCheckAdjust(pcl::PointCloud<PointType>::Ptr cloud_in,pcl::PointCloud<PointType>::Ptr cloud_out){
  pcl::toROSMsg(*current_pc_ptr, pub_pc);
  pub_pc.header = in_pc.header;
  adjust_check_front.publish(pub_pc);

  pcl::toROSMsg(*cloud_out, pub_pc);
  pub_pc.header = in_pc.header;
  adjust_check_back.publish(pub_pc);
}

