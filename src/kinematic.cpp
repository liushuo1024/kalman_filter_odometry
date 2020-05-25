/* Author: Filippo Grazioli */

#include <kalman_filter_odometry/kinematic.hpp>

using namespace kf_odom;

KinematicModel::KinematicModel() :
  dt_(0.1)
{
  update_A();
  update_B();
};

KinematicModel::~KinematicModel()
{
};

void KinematicModel::update_A()
{
  Matrix<float, 9, 9> A = MatrixXf::Identity(9, 9);

  A(0, 3) = dt_;
  A(0, 4) = dt_;
  A(0, 5) = dt_;

  A_ = A;
};

void KinematicModel::update_B()
{
  Matrix<float, 9, 6> B = MatrixXf::Zero(9, 6);

  B(0, 0) = dt_ * dt_ * 0.5;
  B(1, 1) = dt_ * dt_ * 0.5;
  B(2, 2) = dt_ * dt_ * 0.5;
  B(3, 0) = dt_;
  B(4, 1) = dt_;
  B(5, 2) = dt_;
  B(6, 3) = dt_;
  B(7, 4) = dt_;
  B(8, 5) = dt_;

  B_ = B;
};

void KinematicModel::updateDt(float dt)
{
  dt_ = dt;
  update_A();
  update_B();
};

Matrix<float, 9, 1> KinematicModel::predictState(Matrix<float,6,1> u_t)
{
  Matrix<float, 9, 1> state_t = A_ * state_t_minus_1_ + B_ * u_t;
  state_t_minus_1_ = state_t;
  return state_t;
};

Matrix<float, 6, 1> KinematicModel::getPose()
{
  MatrixXf pose(6, 1);
  pose(0, 0) = state_t_minus_1_(0, 0);
  pose(1, 0) = state_t_minus_1_(1, 0);
  pose(2, 0) = state_t_minus_1_(2, 0);
  pose(6, 0) = state_t_minus_1_(6, 0);
  pose(7, 0) = state_t_minus_1_(7, 0);
  pose(8, 0) = state_t_minus_1_(8, 0);
};
