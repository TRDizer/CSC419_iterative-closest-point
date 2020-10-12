#include "hausdorff_lower_bound.h"
#include "point_mesh_distance.h"
#include "random_points_on_mesh.h"

double hausdorff_lower_bound(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int n)
{
  // Replace with your code
  Eigen::MatrixXd query_x, proj_x, normal_x;
  Eigen::VectorXd per_p_dist;

  random_points_on_mesh(n, VX, FX, query_x);
  point_mesh_distance(query_x, VY, FY, per_p_dist, proj_x, normal_x);
  
  return per_p_dist.maxCoeff();
}
