#include "icp_single_iteration.h"
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
#include "point_to_plane_rigid_matching.h"
#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"
#include <iostream>

void icp_single_iteration(
  const Eigen::MatrixXd & VX,
  const Eigen::MatrixXi & FX,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  const int num_samples,
  const ICPMethod method,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd query_points;
  random_points_on_mesh(num_samples, VX, FX, query_points);

  Eigen::MatrixXd projection, normal; 
  Eigen::VectorXd dist;
  point_mesh_distance(query_points, VY, FY, dist, projection, normal);

  if (method == ICP_METHOD_POINT_TO_PLANE) {
    point_to_plane_rigid_matching(query_points, projection, normal, R, t);
    std::cout << "Currently on pont 2 plane" << std::endl;
  }
  else if (method == ICP_METHOD_POINT_TO_POINT) {
    std::cout << "Currently on pont 2 point" << std::endl;
    point_to_point_rigid_matching(query_points, projection, R, t);
  }
  else {
    std::cout << "no match!" << std::endl;
  }

}
