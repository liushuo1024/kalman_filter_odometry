/* Author: Filippo Grazioli */

#include <kalman_filter_odometry/kinematic.hpp>

using namespace kf_odom;

KinematicModel::KinematicModel() :
  dt_(0.1),
  A_(MatrixXf::Identity(12, 12)),
  state_t_minus_1_(MatrixXf::Zero(12, 1))
{
  update_A();
};

KinematicModel::~KinematicModel()
{
};

void KinematicModel::updateDt(const float dt)
{
  dt_ = dt;
  update_A();
};

Matrix<float, 12, 1> KinematicModel::predictNextState()
{
  Matrix<float, 12, 1> state_t = A_ * state_t_minus_1_;
  state_t_minus_1_ = state_t;
  return state_t;
};

Matrix<float, 6, 1> KinematicModel::getPose() const
{
  MatrixXf pose(6, 1);
  pose = state_t_minus_1_.block(0, 0, 6, 1);
  return pose;
};

void KinematicModel::setState(const Matrix<float, 12, 1> state)
{
  state_t_minus_1_ = state;
};

void KinematicModel::setRPY(const Matrix<float, 3, 1> orientation)
{
  state_t_minus_1_(3, 0) = orientation(0, 0);
  state_t_minus_1_(4, 0) = orientation(1, 0);
  state_t_minus_1_(5, 0) = orientation(2, 0);
};

void KinematicModel::update_A()
{
  for (int i = 0; i < 6; i++)
  {
    A_(0 + i, 6 + i) = dt_;
  }
};
