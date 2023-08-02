
#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <linear_acc.hpp>

namespace linear_acc
{
class Linear_Acc_Nodelet: public nodelet::Nodelet
{
private:
  virtual void onInit(void);
  boost::shared_ptr<Linear_Acc> la_; ///< driver implementation class
};

void Linear_Acc_Nodelet::onInit()
{
  la_.reset(new Linear_Acc(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace velodyne_driver

PLUGINLIB_EXPORT_CLASS(linear_acc::Linear_Acc_Nodelet, nodelet::Nodelet)
