#include "random_points_on_mesh.h"
#include <stdlib.h>
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <iostream>

#define GET_RAND_RATIO()    (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX))

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  // Not uniform unfortunately nor is this over all possible points on a mess
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(std::rand()%V.rows());

  Eigen::MatrixXd per_face_samples(F.rows(), 3);

  auto get_vertex = [&] (int face_index, int vertex_index) {
    return V.row(F(face_index, vertex_index));
  };

  double a1, a2;
  for (int F_i = 0; F_i < F.rows(); F_i++) {
    a1 = GET_RAND_RATIO();
    a2 = GET_RAND_RATIO();
    if (a1 + a2 > 1) {
      // reflected triangle
      a1 = 1 - a1;
      a2 = 1 - a2;
    }

    per_face_samples.row(F_i) << 
      get_vertex(F_i, 0) + 
      a1 * (get_vertex(F_i, 1) - get_vertex(F_i, 0)) + 
      a2 * (get_vertex(F_i, 2) - get_vertex(F_i, 0));
  }

  Eigen::VectorXd area, cumulative_area;

  igl::doublearea(V, F, area);
  igl::cumsum(area, 1, cumulative_area);
  // Normalized cumulative vector acts as a lookup table to determine which face is selected as the sampled face 
  cumulative_area /= cumulative_area(cumulative_area.size() - 1);

  // std::cout << F.size() << std::endl;
  // std::cout << cumulative_area.size() << std::endl;

  // cannot use auto for recursive lambda
  std::function<int(int, int, double)> select_face = [&] (int start, int end, double target) {
    if (target <= cumulative_area(start)) {
      return start;
    }
    // target >= 1 for this to happen, although sounding a bit unrealistic, still left it in in case of some wonky float point math
    else if (target >= cumulative_area(end)) {
      return end;
    }
    else {
      int df = end - start;
      if (df == 1) {
        // sandwitched 
        return end;
      }
      int mid = start + (int)(df / 2);
      return select_face(
        cumulative_area(mid) > target ? start : mid,
        cumulative_area(mid) > target ? mid : end,
        target
      );
    }
  };

  auto draw_vertex = [&] () {
    return per_face_samples.row(select_face(0, cumulative_area.size() - 1, GET_RAND_RATIO()));
  };

  for (int sample_count = 0; sample_count < X.rows(); sample_count++) {
    X.row(sample_count) << draw_vertex();
  }

  // std::cout << X.block<3,3>(0,0) << std::endl;
  // std::cout << X.block<3,3>(X.rows() - 3, X.cols() - 3) << std::endl;
}
