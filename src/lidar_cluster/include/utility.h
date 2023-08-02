#ifndef UTILITY_HPP_H// #pragma once
#define UTILITY_HPP_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_set>
#include <imu_subscriber.hpp>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "std_msgs/Float64MultiArray.h"
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/time.h>
//#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unistd.h>
//#include <pcl/ModelCoefficients.h>
#include <boost/thread/thread.hpp>
#include <distortion_adjust.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/common/transforms.h>
#include <common_msgs/Cone.h>
#include <common_msgs/ins_p2.h>

// #include "utility.h"
// #include <iostream>
// #include "SVD.h"
// #include <vector>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace lidar_distortion;
#define Eight_Ring 1
#define Linear_Acc 2
#define High_Speed_Tracking 3
using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;
typedef pcl::PointXYZ PointType;
class lidar_cluster
{

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_ins_;
    ros::Publisher markerPub;
    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_filtered_points__;
    ros::Publisher pub_filtered_points___;
    ros::Publisher cone_position;
    ros::Publisher lidarClusterPublisher_;
    ros::Publisher adjust_check_front;
    ros::Publisher adjust_check_back;

    ros::Publisher skidpad_detection;

    ros::Publisher logging_pub;
    nav_msgs::Path ros_path_;
    geometry_msgs::PoseStamped pose;
    Eigen::Matrix4f transMat_;


    Eigen::Matrix4f m;
    vector<double> tmp_comparison_min_x_first_v;
    vector<double> tmp_comparison_min_x_second_v;
    vector<double> tmp_comparison_y_first_v;
    vector<double> tmp_comparison_y_second_v;

    double tmp_comparison_min_x_first;
    double tmp_comparison_min_x_second;
    double tmp_comparison_y_first=0;
    double tmp_comparison_y_second=0;

    int filter_centroid=0;
    double base_x = 1.0;
    double base_y= 0;
    bool matchFlag=true;
    std::string no_ground_topic , ground_topic , all_points_topic, input_topic;
    ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
    std::string point_topic_;
    bool getPointClouds;
    bool inverse_flag;//matrix.inverse()???
  	int i;
  	int useED=1;
  	float ed[4]={0};
    int marker_id = 0;
    int sensor_model_;
    double sensor_height_, clip_height_, min_distance_, max_distance_,z_up_,z_down_;
    int vis_;
    int num_seg_ = 1;
    int road_type_ = 1;
    int num_iter_, num_lpr_;
    double th_seeds_, th_dist_;
    double intensity,euc;
    int intensity_count;
    std::string str_range_,str_seg_distance_;
    geometry_msgs::Point p;
    std::string text;
    float d_, th_dist_d_;
    MatrixXf normal_;

  common_msgs::Cone position;
  sensor_msgs::PointCloud2 pub_pc;
  sensor_msgs::PointCloud2 cluster_for_publish;
  bool vis_init_status;
public:
	pcl::PointCloud<PointType>::Ptr g_seeds_pc;
	pcl::PointCloud<PointType>::Ptr g_ground_pc;
	pcl::PointCloud<PointType>::Ptr g_not_ground_pc;
	pcl::PointCloud<PointType>::Ptr g_all_pc;
	pcl::PointCloud<PointType>::Ptr current_pc_ptr;
  pcl::PointCloud<PointType>::Ptr distortion_out;//distortion_Adjust
	pcl::PointCloud<PointType>::Ptr cloud_filtered;
  pcl::PointCloud<PointType>::Ptr skidpad_detection_pc;

  sensor_msgs::PointCloud out_pc;
  
  sensor_msgs::PointCloud2 in_pc;

  
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker bbox_marker;
  visualization_msgs::Marker euc_marker;
    // DistortionAdjust distadjust;
  bool hasIMu = false;
  std::mutex lidar_mutex;
  std::deque<IMUData> unsynced_imu_;

  std::vector<double> dis_range;
  std::vector<double> seg_distances;
    
  public:
  void init();
  void estimate_plane_(void);
  // void transMatrix(const std_msgs::Float64MultiArray &msgs);
  void extract_initial_seeds_(const pcl::PointCloud<PointType> &p_sorted);
  void ground_segmentation(const pcl::PointCloud<PointType>::Ptr &in_pc,
                                        pcl::PointCloud<PointType>::Ptr &g_not_ground_pc);
  void vis_for_marker(const pcl::PointXYZ max,const pcl::PointXYZ min,float euc);
  bool vis_init();
  void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
  // void point_clip(const pcl::PointCloud<PointType>::Ptr in,
  //                         pcl::PointCloud<PointType>::Ptr out);
  void SACSMethod(pcl::PointCloud<PointType>::Ptr& cloud_filtered,int maxIterations,float distanceThreshold,float planarThreshold);
  void SVDGroundMethod(pcl::PointCloud<PointType>::Ptr& cloud_filtered,float distanceThreshold,float seedThreshold,int Nlpr);
    //EuclideanCluster
  void PassThrough(pcl::PointCloud<PointType>::Ptr& cloud_filtered,int type,double z_up,double z_down);
  void EucClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud,std::vector<pcl::PointIndices>& cluster_indices,const double& max_cluster_dis);
  void EuclideanAdaptiveClusterMethod(pcl::PointCloud<PointType>::Ptr inputcloud, std::vector<std::vector<pcl::PointIndices>>& cluster_indices,
  std::vector<pcl::PointCloud<PointType>::Ptr> cloud_segments_array);
  void splitString(const std::string& in_string, std::vector<double>& out_array);
  void clusterMethod16();
  void clusterMethod32();
  void runAlgorithm();
  void publishToCheckAdjust(pcl::PointCloud<PointType>::Ptr cloud_in,pcl::PointCloud<PointType>::Ptr cloud_out);
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  std::shared_ptr<DistortionAdjust> disAdjust;
  double cloud_time;
  lidar_cluster(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~lidar_cluster();
  void Spin();
};




#endif