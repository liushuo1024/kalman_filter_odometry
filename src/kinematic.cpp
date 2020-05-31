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
  state_(MatrixXd::Zero(10, 1))
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

Matrix<double, 10, 1> KinematicModel::predictNextState(const Eigen::Vector3d& ang_vel,
                                                       const Eigen::Vector3d& lin_acc,
                                                       const Eigen::MatrixXd& F,
                                                       const Eigen::MatrixXd& Q,
                                                       const Eigen::MatrixXd& L,
                                                       Eigen::MatrixXd& P)
{
  //ToDo
};

Matrix<double, 7, 1> KinematicModel::getPose() const
{
  //ToDo
  MatrixXd pose(7, 1);
  return pose;
};

void KinematicModel::initState(const Matrix<double, 10, 1>& state)
{
  state_ = state;
}

