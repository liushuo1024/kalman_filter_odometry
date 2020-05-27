/* Author: Filippo Grazioli */

#ifndef KINEMATIC_HPP
#define KINEMATIC_HPP

#include <Eigen/Dense>

using namespace Eigen;

namespace kf_odom
{
  /**
   * Class for the constant velocity model
   */
  class KinematicModel
  {
  public:
    KinematicModel();

    virtual ~KinematicModel();

    void updateDt(const float);
    Matrix<float, 12, 1> predictNextState();
    Matrix<float, 6, 1> getPose() const;
    void setState(const Matrix<float, 12, 1>);
    void setRPY(const Matrix<float, 3, 1>);

  private:
    void update_A();

    float dt_;
    Matrix<float, 12, 12> A_;
    Matrix<float, 12, 1> state_t_minus_1_;
  };
} // namespace kf_odom

#endif // KINEMATIC_HPP
