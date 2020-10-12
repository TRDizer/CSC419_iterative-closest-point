#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>
#include <iostream>
#include <limits> // the only thing I grasped in 418

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
  P.resizeLike(X);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  D.resize(X.rows());

  Eigen::MatrixXd per_face_normal;
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1), per_face_normal);
  
  double min_dist, temp_dist;
  Eigen::RowVector3d temp_p;
  for (int query_i = 0; query_i < X.rows(); query_i++) {
    min_dist = std::numeric_limits<double>::infinity();

    for (int face_i = 0; face_i < FY.rows(); face_i++) {
      point_triangle_distance(
        X.row(query_i), 
        VY.row(FY(face_i, 0)),
        VY.row(FY(face_i, 1)),
        VY.row(FY(face_i, 2)),
        temp_dist,
        temp_p
      );

      if (temp_dist < min_dist) {
        min_dist = temp_dist;
        D(query_i) = min_dist;
        P.row(query_i) << temp_p;
        N.row(query_i) << per_face_normal.row(face_i);
      }
    }
  }
}
