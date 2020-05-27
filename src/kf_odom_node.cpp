/* Author: Filippo Grazioli */

#include <kalman_filter_odometry/kf_odom_node.hpp>

using namespace kf_odom;

static unsigned int INIT_STEPS = 100;

KfOdomNode::KfOdomNode() :
  kf_(new Kf()),
  imu_callback_counter_(0),
  gps_callback_counter_(0)
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

  if (imu_callback_counter_ < INIT_STEPS) kf_->initImu(imu);
  //ToDo: KF prediction
  //ToDo: KF update
  output_ = kf_->getPose();
  pose_pub_.publish(output_);
  imu_callback_counter_++;
};

void KfOdomNode::gpsCallback(const NavSatFixConstPtr& gps)
{
  ROS_INFO("Got gps");

  //ToDo: KF prediction
  //ToDo: KF update
  output_ = kf_->getPose();
  pose_pub_.publish(output_);
  gps_callback_counter_++;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kf_odom_node");

  KfOdomNode kf_odom_node;

  ros::spin();

  return 0;
}
