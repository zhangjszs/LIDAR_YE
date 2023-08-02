#ifndef LINEAR_ACCELERATE_HPP_
#define LINEAR_ACCELERATE_HPP_
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <common_msgs/Cone.h>
#include <common_msgs/vehicle_cmd.h>
#include <common_msgs/vehicle_status.h>
typedef pcl::PointXYZ PointType;
struct comp{
  bool operator()(pcl::PointXYZ a,pcl::PointXYZ b){
    if (a.x<b.x) return true;
    else if (a.x>b.x) return false;
    else if (a.y<b.y) return true;
    else if (a.y>b.y) return false;
    else if (a.z>b.z) return true;
    else if (a.z>b.z) return false;
  }
};
namespace linear_acc{
class Linear_Acc{
public:
  Linear_Acc(ros::NodeHandle node,ros::NodeHandle private_nh);
  void createPath();
  void init();
  ~Linear_Acc();
    int vis_ = 0;
    int vised = 0;
    int forward_distance_ = 0;
    int inverse_flag_ = 0;
    int end_x_ = 10;

    bool getPath = false;
    double path_length = 80;
    double allow_angle_error = 1.0;
    common_msgs::Cone cluster;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cone_points_set_;
    ros::Subscriber sub_ins_;
    ros::Publisher pub_vehicle_cmd;
    ros::Publisher linear_acc_path;
    nav_msgs::Path ros_path_;
    geometry_msgs::PoseStamped pose;
    common_msgs::vehicle_cmd v_cmd;
    common_msgs::vehicle_status v_status;
    geometry_msgs::Point end_point;
    // ros::Publisher markerPub;
    std::set<pcl::PointXYZ,comp> orderCone;

   //  pcl::PointCloud<PointType>::Ptr g_seeds_pc;
  	// pcl::PointCloud<PointType>::Ptr g_ground_pc;
  	// pcl::PointCloud<PointType>::Ptr g_not_ground_pc;
  	// pcl::PointCloud<PointType>::Ptr g_all_pc;
  	// pcl::PointCloud<PointType>::Ptr current_pc_ptr;
  	// pcl::PointCloud<PointType>::Ptr distortion_out;//distortion_Adjust
	  // pcl::PointCloud<PointType>::Ptr cloud_filtered;
  	// pcl::PointCloud<PointType>::Ptr skidpad_detection_pc;
  void vehicleStatus_callback(const common_msgs::vehicle_statusConstPtr& vehicle_status);
  void cone_position_callback(const common_msgs::ConeConstPtr& cone_position);
  void visualizeLinearPath(double at2_angle_mid);
};
}
#endif