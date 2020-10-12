#include "point_to_plane_rigid_matching.h"
#include "closest_rotation.h"
#include <Eigen/Dense>
#include <igl/cat.h>
#include <iostream>

void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code
  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  int num_points = X.rows();

  Eigen::MatrixXd diagonal_temp(num_points, num_points * 2);
  Eigen::MatrixXd diagonal_final(num_points, num_points * 3);
  diagonal_temp.setZero();
  diagonal_final.setZero();
  // have to feed that into a constructor to make igl::cat happy :(
  igl::cat(2, Eigen::MatrixXd(N.col(0).asDiagonal()), Eigen::MatrixXd(N.col(1).asDiagonal()), diagonal_temp);
  igl::cat(2, diagonal_temp, Eigen::MatrixXd(N.col(2).asDiagonal()), diagonal_final);

  Eigen::VectorXd ones = Eigen::VectorXd::Ones(num_points);
  Eigen::VectorXd zeros = Eigen::VectorXd::Zero(num_points);

  Eigen::MatrixXd A(num_points * 3, 6);
  //       0     a3   -a2    1    0    0
  // A =  -a3    0     a1    0    1    0
  //       a2   -a1    0     0    0    1

  // cannot determine num_points at compile time and using dynamic-size variant will crash the program
  // A.block<num_points, 1>(0,1)              <<  X.col(2);
  // A.block<num_points, 1>(0,2)              << -X.col(1);
  // A.block<num_points, 1>(num_points,0)     << -X.col(2);
  // A.block<num_points, 1>(num_points,2)     <<  X.col(0);
  // A.block<num_points, 1>(num_points * 2,0) <<  X.col(1);
  // A.block<num_points, 1>(num_points * 2,1) << -X.col(0);

  // A.block<num_points, 1>(0,3)              << ones;
  // A.block<num_points, 1>(num_points,4)     << ones;
  // A.block<num_points, 1>(num_points * 2,5) << ones;

  A.col(0) << zeros, X.col(2), -X.col(1);;
  A.col(1) << -X.col(2), zeros, X.col(0);
  A.col(2) << X.col(1), -X.col(0), zeros;

  A.col(3) << ones, zeros, zeros;
  A.col(4) << zeros, ones, zeros;
  A.col(5) << zeros, zeros, ones;

  Eigen::VectorXd B(num_points * 3);
  B << X.col(0) - P.col(0), 
       X.col(1) - P.col(1), 
       X.col(2) - P.col(2);

  A = diagonal_final * A;
  B = diagonal_final * B;

  // std::cout << "I am here!!! help!" << std::endl; 

  Eigen::VectorXd a;
  a = (A.transpose() * A).inverse() * (-A.transpose() * B);

  //  0    -a3    a2  
  //  a3    0    -a1  
  // -a2    a1    0   
  Eigen::Matrix3d M;
  M <<    1,   -a(2),  a(1),
         a(2),  1,    -a(0),
        -a(1),  a(0),  1;

  // std::cout << "my m:\n" << M << std::endl;      
  closest_rotation(M, R);
  t << a(3), a(4), a(5);
}
