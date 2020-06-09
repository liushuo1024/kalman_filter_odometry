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

#include <kalman_filter_odometry/kinematic.hpp>

using namespace kf_odom;

KinematicModel::KinematicModel() :
  dt_(0.1),
  state_(MatrixXd::Zero(10, 1)),
  g_(Eigen::Vector3d(0, 0, 9.80665))
{
  state_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
};

KinematicModel::~KinematicModel()
{
};

void KinematicModel::updateDt(const double dt)
{
  dt_ = dt;
};

double KinematicModel::dt() const
{
  return dt_;
};

void KinematicModel::predictNextState(const Eigen::Vector3d& ang_vel,
                                      const Eigen::Vector3d& lin_acc)
{
  Eigen::Quaterniond orientation_quat = this->getQuaternion();
  Eigen::Matrix3d rotation_mat = orientation_quat.toRotationMatrix();

  // Position
  state_.segment(0, 3) = state_.segment(0, 3) +
                         dt_ * state_.segment(3, 3) +
                         0.5 * dt_ * dt_ * (rotation_mat * lin_acc - g_);

  // Velocity
  state_.segment(3, 3) = state_.segment(3, 3) + dt_* (rotation_mat * lin_acc - g_);

  // Orientation
  Eigen::Quaterniond delta_quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(ang_vel.x() * dt_, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(ang_vel.y() * dt_, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ang_vel.z() * dt_, Eigen::Vector3d::UnitZ()));

  Eigen::Quaterniond predicted_quat = delta_quat * orientation_quat;
  state_.segment(6, 4) = Eigen::Vector4d(predicted_quat.x(),
                                         predicted_quat.y(),
                                         predicted_quat.z(),
                                         predicted_quat.w());
};

Matrix<double, 7, 1> KinematicModel::getPose() const
{
  Matrix<double, 7, 1> pose;
  pose(0, 0) = state_(0, 0); // x
  pose(1, 0) = state_(1, 0); // y
  pose(2, 0) = state_(2, 0); // z
  pose(3, 0) = state_(6, 0); // qx
  pose(4, 0) = state_(7, 0); // qy
  pose(5, 0) = state_(8, 0); // qz
  pose(6, 0) = state_(9, 0); // qw
  return pose;
};

void KinematicModel::initState(const Matrix<double, 10, 1>& state)
{
  state_ = state;
}

Eigen::Quaterniond KinematicModel::getQuaternion() const
{
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(state_(6, 0),
                                                     state_(7, 0),
                                                     state_(8, 0),
                                                     state_(9, 0)); //ToDo : double check
  return quaternion;
};
