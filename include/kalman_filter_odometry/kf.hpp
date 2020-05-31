//MIT License

//Copyright (c) 2020 Filippo Grazioli

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#ifndef KF_HPP
#define KF_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

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
    void initState(const tf::StampedTransform& tfTransform);
    void getPose(geometry_msgs::PoseWithCovarianceStamped& pose) const;
    void predict(const ImuConstPtr imu);

  private:
    std::unique_ptr<KinematicModel> kinematic_;
    ros::Time timeStamp_;
    Eigen::Matrix<double, 9, 9> P_;  // Variance of the latent variable ~ N(state, P)
    Eigen::Matrix<double, 9, 9> F_;
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix<double, 9, 6> L_;
  };
}

#endif // KF_HPP
