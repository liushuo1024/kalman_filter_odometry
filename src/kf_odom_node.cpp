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

#include <kalman_filter_odometry/kf_odom_node.hpp>

using namespace kf_odom;

static unsigned int INIT_STEPS = 100;

KfOdomNode::KfOdomNode() :
  tfListener_(tfBuffer_),
  kf_(new Kf()),
  init_counter_(0),
  imu_prediction_counter_(0),
  gps_update_counter_(0),
  imu_update_counter_(0),
  param_imu_acc_(0.1),
  param_imu_w_(0.1)
{
  node_.param("par_imu_acc", param_imu_acc_, 0.1);
  node_.param("par_imu_w", param_imu_w_, 0.1);
  kf_->setParams(param_imu_acc_, param_imu_w_);

  pose_pub_       = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("kf_odom/odom", 10);
  imu_pred_sub_   = node_.subscribe("imu_prediction", 10, &KfOdomNode::imuPredictionCallback, this);
  gps_upate_sub_  = node_.subscribe("gps_update", 10, &KfOdomNode::gpsUpdateCallback, this);
  imu_update_sub_ = node_.subscribe("imu_update", 10, &KfOdomNode::imuUpdateCallback, this);
};

KfOdomNode::~KfOdomNode()
{
};

void KfOdomNode::imuPredictionCallback(const ImuConstPtr& imu)
{
  if (init_counter_ < INIT_STEPS)
  {
    try{
      tfInitMsg_ = tfBuffer_.lookupTransform("world", "imu_link", ros::Time(0));
      tf::transformStampedMsgToTF(tfInitMsg_, tfInitSt_);
      kf_->initState(tfInitSt_);
      init_counter_++;
      ROS_INFO("Imu init");
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  else
  {
    kf_->predict(imu);
    imu_prediction_counter_++;
    ROS_INFO("KF prediction");
  }
};

void KfOdomNode::gpsUpdateCallback(const NavSatFixConstPtr& gps)
{
  if (init_counter_ >= INIT_STEPS)
  {
    //ToDo: KF update gps
    kf_->getPose(output_);
    pose_pub_.publish(output_);
    gps_update_counter_++;
    ROS_INFO("KF GPS update");
  }
};

void KfOdomNode::imuUpdateCallback(const ImuConstPtr& imu)
{
  if (init_counter_ >= INIT_STEPS)
  {
    //ToDo: KF update imu correction
    kf_->getPose(output_);
    pose_pub_.publish(output_);
    imu_update_counter_++;
    ROS_INFO("KF IMU update");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kf_odom_node");

  KfOdomNode kf_odom_node;

  ros::spin();

  return 0;
}
