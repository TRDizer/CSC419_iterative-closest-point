#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <iostream>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  // Assuming R* given, we can solve for t* which will turn out to be a difference between the pre and post transformed centroid
  // Substitute t with the representation we just solved for that contains R*, and let tilde_x = x - x.centroid and tilde_p = p - p.centroid
  // tilde_x_i * R^T * tilde_p_i
  Eigen::RowVector3d x_centroid = X.colwise().sum() / X.rows();
  Eigen::RowVector3d p_centroid = P.colwise().sum() / P.rows();


  Eigen::MatrixXd tilde_x = X.rowwise() - x_centroid;
  Eigen::MatrixXd tilde_p = P.rowwise() - p_centroid;

  closest_rotation((tilde_p.transpose() * tilde_x).transpose(), R);
  // sub R* with the solved R to update t
  t = p_centroid - (R * x_centroid.transpose()).transpose();
}

