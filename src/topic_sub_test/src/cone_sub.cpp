#include <unordered_set>
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
#include <sensor_msgs/PointCloud2.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unistd.h>
#include <boost/thread/thread.hpp>
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
#include "common_msgs/Cone.h"
#include "common_msgs/ins_p2.h"
#include<fstream>
#include<sstream>
#include <algorithm>

void doCone(const common_msgs::Cone::ConstPtr& cone);

int main(int argc, char  *argv[]){  
     ros::init(argc,argv,"cone_sub_test");
     ros::NodeHandle nh;
     ros::Subscriber cone_sub=nh.subscribe("/cone_position",10,doCone);
     ros::spin();
     return 0;
}




void doCone(const common_msgs::Cone::ConstPtr& cone){
    std::stringstream ss;
    for (int i = 0; i < cone->points.size(); ++i) {
        ss << cone->points[i].x << "\t" << cone->points[i].y << "\t" << cone->points[i].z << std::endl;
    }
    std::string str = ss.str();

    std::ofstream f;
    f.open("/home/yeziheng/LIDAR_ye/src/topic_sub_test/txt/conedata.txt");
    f << str;
    f.close();
}

