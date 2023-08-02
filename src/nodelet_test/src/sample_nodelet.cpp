#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sample_nodelet.h>

PLUGINLIB_EXPORT_CLASS(lidar_cluster::SampleNodelet, nodelet::Nodelet)

namespace lidar_cluster{

	void SampleNodelet::onInit(){
		NODELET_DEBUG("INITIALIZING NODELET...");
		ROS_INFO("Nodelet is Ok for sample!!");
	}
}