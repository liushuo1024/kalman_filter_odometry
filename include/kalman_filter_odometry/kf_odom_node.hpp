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

#ifndef KF_ODOM_NODE_HPP
#define KF_ODOM_NODE_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
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
    void imuPredictionCallback(const ImuConstPtr& imu);
    void gpsUpdateCallback(const NavSatFixConstPtr& gps);
    void imuUpdateCallback(const ImuConstPtr& imu);

    ros::NodeHandle node_;
    ros::Publisher pose_pub_;
    ros::Subscriber imu_pred_sub_, gps_upate_sub_, imu_update_sub_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped tfInitMsg_;
    tf::StampedTransform tfInitSt_;

    geometry_msgs::PoseWithCovarianceStamped output_;

    std::unique_ptr<Kf> kf_;

    unsigned int init_counter_, imu_prediction_counter_, gps_update_counter_, imu_update_counter_;
  };
} // namespace kf_odom

#endif // KF_ODOM_NODE_HPP
