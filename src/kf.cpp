/* Author: Filippo Grazioli */

#include <kalman_filter_odometry/kf.hpp>

using namespace kf_odom;

Kf::Kf(Matrix<float, 12, 1> init_state) :
  kinematic_(init_state),
  lastTime_(ros::Time::now())
{
};

Kf::~Kf()
{
};

void Kf::updateTime() // this must be called in prediction step
{
  ros::Time now = ros::Time::now();
  float dt = now.toSec() - lastTime_.toSec();
  kinematic_.updateDt(dt);
  lastTime_ = now;
};
