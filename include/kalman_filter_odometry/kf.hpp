/* Author: Filippo Grazioli */

#ifndef KF_HPP
#define KF_HPP

#include <ros/ros.h>

#include <kalman_filter_odometry/kinematic.hpp>

namespace kf_odom
{
  class Kf
  {
  public:
    Kf(Matrix<float, 12, 1> init_state);

    virtual ~Kf();

    void updateTime();

  private:
    KinematicModel kinematic_;
    ros::Time lastTime_;
  };
}

#endif // KF_HPP
