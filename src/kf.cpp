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
  lastTime_(ros::Time::now())
{
};

Kf::~Kf()
{
};

void Kf::updateTime() // this must be called before the prediction step to compute dt
{
  ros::Time now = ros::Time::now();
  double dt = now.toSec() - lastTime_.toSec();
  kinematic_->updateDt(dt);
  lastTime_ = now;
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
};

void Kf::getPose(geometry_msgs::PoseWithCovarianceStamped& pose) const
{
  Matrix<double, 7, 1> kinematic_pose = kinematic_->getPose();
  pose.pose.pose.position.x = kinematic_pose(0, 0);
  pose.pose.pose.position.y = kinematic_pose(1, 0);
  pose.pose.pose.position.z = kinematic_pose(2, 0);
  tf::Quaternion q(kinematic_pose(6, 0),
                   kinematic_pose(7, 0),
                   kinematic_pose(8, 0),
                   kinematic_pose(9, 0));
  quaternionTFToMsg(q, pose.pose.pose.orientation);
};

void Kf::predict(const ImuConstPtr imu)
{
  //ToDo: continue here
};
