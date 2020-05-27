/* Author: Filippo Grazioli */

#ifndef KF_ODOM_NODE_HPP
#define KF_ODOM_NODE_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <memory>
#include <boost/thread/mutex.hpp>

#include <kalman_filter_odometry/kf.hpp>

namespace kf_odom
{
  typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
  typedef boost::shared_ptr<sensor_msgs::NavSatFix const> NavSatFixConstPtr;

  class KfOdomNode
  {
  public:
    KfOdomNode();

    virtual ~KfOdomNode();

  private:
    void imuCallback(const ImuConstPtr& imu);

    void gpsCallback(const NavSatFixConstPtr& gps);

    ros::NodeHandle node_;
    ros::Publisher pose_pub_;
    ros::Subscriber imu_sub_, gps_sub_;
    ros::Time lastTime_;
    //tf::TransformListener listener_;
    //tf::StampedTransform imu_transform_;

    geometry_msgs::PoseWithCovarianceStamped  output_;

    std::unique_ptr<Kf> kf_;

    unsigned int imu_callback_counter_, gps_callback_counter_;
  };
} // namespace kf_odom

#endif // KF_ODOM_NODE_HPP
