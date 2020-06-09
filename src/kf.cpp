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

#include <kalman_filter_odometry/kf.hpp>
#include <kalman_filter_odometry/kinematic.hpp>

using namespace kf_odom;

Kf::Kf() :
  kinematic_(new KinematicModel()),
  timeStamp_(ros::Time::now()),
  F_(Eigen::Matrix<double, 9, 9>::Identity()),
  Q_(Eigen::Matrix<double, 6, 6>::Identity()),
  L_(Eigen::Matrix<double, 9, 6>::Zero()),
  P_(Eigen::Matrix<double, 9, 9>::Identity()),
  param_imu_acc_(0.1),
  param_imu_w_(0.1)
{
};

Kf::~Kf()
{
};

void Kf::updateTime() // must be called before the prediction step and after initState to compute dt
{
  //ToDo : shift the timestamp member to the Kinematic class
  ros::Time now = ros::Time::now();
  double dt = now.toSec() - timeStamp_.toSec();
  kinematic_->updateDt(dt);
  timeStamp_ = now;
};

void Kf::initState(const tf::StampedTransform& tfTransform)
{
  Matrix<double, 10, 1> init_state;
  init_state(0, 0) = tfTransform.getOrigin().getX();
  init_state(1, 0) = tfTransform.getOrigin().getY();
  init_state(2, 0) = tfTransform.getOrigin().getZ();
  init_state(6, 0) = tfTransform.getRotation().x();
  init_state(7, 0) = tfTransform.getRotation().y();
  init_state(8, 0) = tfTransform.getRotation().z();
  init_state(9, 0) = tfTransform.getRotation().w();
  kinematic_->initState(init_state);
  this->updateTime();
};

void Kf::getPose(geometry_msgs::PoseWithCovarianceStamped& pose) const
{
  Matrix<double, 7, 1> kinematic_pose = kinematic_->getPose();
  pose.pose.pose.position.x = kinematic_pose(0, 0);
  pose.pose.pose.position.y = kinematic_pose(1, 0);
  pose.pose.pose.position.z = kinematic_pose(2, 0);

  tf::Quaternion q(kinematic_pose(3, 0),
                   kinematic_pose(4, 0),
                   kinematic_pose(5, 0),
                   kinematic_pose(6, 0));
  quaternionTFToMsg(q, pose.pose.pose.orientation);

  //for (int i = 0; i < P_.size(); i++)
    //*(pose.pose.covariance.data() + i) = *(P_.data() + i);
};

void Kf::predict(const ImuConstPtr imu)
{
  Eigen::Vector3d ang_vel(imu->angular_velocity.x,
                          imu->angular_velocity.y,
                          imu->angular_velocity.z);

  Eigen::Vector3d lin_acc(imu->linear_acceleration.x,
                          imu->linear_acceleration.y,
                          imu->linear_acceleration.z);

  Eigen::Quaterniond orientation_quat = kinematic_->getQuaternion();
  Eigen::Matrix3d rotation_mat = orientation_quat.toRotationMatrix();

  this->updateTime();

  kinematic_->predictNextState(ang_vel, lin_acc);

  // Update F
  F_.block<3, 3>(0, 3) = kinematic_->dt() * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d skew_lin_acc;
  skew_lin_acc << 0, -lin_acc(2), lin_acc(1),
                  lin_acc(2), 0, -lin_acc(0),
                  -lin_acc(1), lin_acc(0), 0;
  F_.block<3, 3>(3, 6) = rotation_mat  * (-skew_lin_acc) * kinematic_->dt();

  // Update Q
  Q_.block<3, 3>(0, 0) = param_imu_acc_ * Q_.block<3, 3>(0, 0);
  Q_.block<3, 3>(3, 3) = param_imu_w_ * Q_.block<3, 3>(3, 3);
  Q_ = Q_ * (this->getKinematic().dt() * this->getKinematic().dt());

  // Update L
  L_.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
  L_.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();

  P_ = F_ * P_ * F_.transpose() + L_ * Q_ * L_.transpose();
};

void Kf::setParams(double param_imu_acc, double param_imu_w)
{
  param_imu_acc_ = param_imu_acc;
  param_imu_w_ = param_imu_w;
};
