#include "closest_rotation.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  // Perform SVD on M: https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::Matrix3d omega;
  double det = (U * V.transpose()).determinant();
  omega << 1, 0, 0,
           0, 1, 0,
           0, 0, det;
  R = U * omega * V.transpose();
}
