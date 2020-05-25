/* Author: Filippo Grazioli */

#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <Eigen/Dense>

using namespace Eigen;

namespace kf_odom {

  class KinematicModel
  {
  public:
    KinematicModel();

    virtual ~KinematicModel();

    void updateDt(float);
    Matrix<float, 9, 1> predictState(Matrix<float,6,1>);
    Matrix<float, 6, 1> getPose();

  private:
    void update_A();
    void update_B();

    float dt_;
    Matrix<float, 9, 9> A_;
    Matrix<float, 9, 6> B_;
    Matrix<float, 9, 1> state_t_minus_1_;
  };
} // namespace kf_odom

#endif // KINEMATIC_HPP
