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

    void updateDt(const double dt);
    predictNextState(const Eigen::Vector3d& ang_vel,
                     const Eigen::Vector3d& lin_acc,
                     const Eigen::MatrixXd& F,
                     const Eigen::MatrixXd& Q,
                     const Eigen::MatrixXd& L,
                     Eigen::MatrixXd& P);
    Matrix<double, 7, 1> getPose() const;
    void initState(const Matrix<double, 10, 1>& state);

  private:
    double dt_;
    Matrix<double, 10, 1> state_; // state = [x y z vx vy vz qx qy qz qw]
  };
} // namespace kf_odom

#endif // KINEMATIC_HPP
