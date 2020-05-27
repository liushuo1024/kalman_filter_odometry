/* Author: Filippo Grazioli */

#ifndef KF_HPP
#define KF_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <memory>

#include <kalman_filter_odometry/kinematic.hpp>

namespace kf_odom
{
  typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
  typedef boost::shared_ptr<sensor_msgs::NavSatFix const> NavSatFixConstPtr;

  /**
   * Class for the Kalman filter
   */
  class Kf
  {
  public:
    Kf();

    virtual ~Kf();

    void updateTime();
    void initImu(const ImuConstPtr imu);
    void setOrientation(const ImuConstPtr imu);
    geometry_msgs::PoseWithCovarianceStamped getPose() const;

  private:
    std::unique_ptr<KinematicModel> kinematic_;
    ros::Time lastTime_;
  };
}

#endif // KF_HPP
