/* Author: Filippo Grazioli */

#include <kalman_filter_odometry/kf_odom_node.hpp>

using namespace kf_odom;

KfOdomNode::KfOdomNode()
{
  pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("kf_odom/odom", 10);
  imu_sub_ = node_.subscribe("imu_data", 10, &KfOdomNode::imuCallback, this);
  gps_sub_ = node_.subscribe("gps", 10, &KfOdomNode::gpsCallback, this);
  //listener_.lookupTransform("/base_link", "/world", ros::Time(0), imu_transform_);
};

KfOdomNode::~KfOdomNode()
{
};

void KfOdomNode::imuCallback(const ImuConstPtr& imu)
{
  ROS_INFO("Got imu");
  //KF prediction
  //KF update
};

void KfOdomNode::gpsCallback(const NavSatFixConstPtr& gps)
{
  ROS_INFO("Got gps");
  //KF prediction
  //KF update
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kf_odom_node");

  KfOdomNode kf_odom_node;

  ros::spin();

  return 0;
}
