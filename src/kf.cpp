/* Author: Filippo Grazioli */

#include <kalman_filter_odometry/kf.hpp>
#include <kalman_filter_odometry/kinematic.hpp>

using namespace kf_odom;

Kf::Kf() :
  kinematic_(new KinematicModel()),
  lastTime_(ros::Time::now())
{
};

Kf::~Kf()
{
};

void Kf::updateTime() // this must be called before the prediction step to compute dt
{
  ros::Time now = ros::Time::now();
  float dt = now.toSec() - lastTime_.toSec();
  kinematic_->updateDt(dt);
  lastTime_ = now;
};

void Kf::initImu(const ImuConstPtr imu)
{
  setOrientation(imu);
  //ToDo: set linear velocity
  //ToDO: set angular velocity
};

void Kf::setOrientation(const ImuConstPtr imu)
{
  double roll, pitch, yaw;
  MatrixXf pose(3, 1);
  tf::Quaternion q(imu->orientation.x,
                   imu->orientation.y,
                   imu->orientation.z,
                   imu->orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  pose << roll, pitch, yaw;
  kinematic_->setRPY(pose);
}

geometry_msgs::PoseWithCovarianceStamped Kf::getPose() const
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  tf::Quaternion q;
  Matrix<float, 6, 1> kinematic_pose = kinematic_->getPose();
  q.setRPY(kinematic_pose(3, 0), kinematic_pose(4, 0), kinematic_pose(5, 0));
  quaternionTFToMsg(q, pose.pose.pose.orientation);
  pose.pose.pose.position.x = kinematic_pose(0, 0);
  pose.pose.pose.position.y = kinematic_pose(1, 0);
  pose.pose.pose.position.z = kinematic_pose(2, 0);

  return pose;
}
