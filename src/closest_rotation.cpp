#include "closest_rotation.h"
#include <Eigen/Dense>
#include <Eigen/SVD>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d omega = Eigen::Matrix3d::Identity();
  omega(2,2) = ((U * V).transpose()).determinant();
  R = U * omega * V.transpose();
}
