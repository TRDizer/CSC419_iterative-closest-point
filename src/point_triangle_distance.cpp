#include "point_triangle_distance.h"
#include <Eigen/Geometry>
#include <iostream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
  // Replace with your code
  // Projection onto triangular plain and work out whether projected coordinate is on the mesh or outside of the mesh
  // reference: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.104.4264&rep=rep1&type=pdf
  // Eigen::RowVector3d ax = x - a;
  // Eigen::RowVector3d bx = x - b;
  // Eigen::RowVector3d cx = x - c;
  // Eigen::RowVector3d normal = (b-a).cross(c-a);

  // double cos_alpha = ax.dot(normal) / (ax.norm() * normal.norm());
  // double dis_xp = cos_alpha * ax.norm();
  // Eigen::RowVector3d xp = -dis_xp * normal.normalized();
  // // p_prime is on the triangular plain but not necessarily on the mesh
  // Eigen::RowVector3d p_prime = x + xp;
  // Eigen::RowVector3d V1, V2, V3, R, p_prime2p_pp, p_pp, p1, p2;
  // V1 = (a-b).normalized() + (c-a).normalized();
  // V2 = (b-c).normalized() + (b-a).normalized();
  // V3 = (c-a).normalized() + (c-b).normalized();
  // double u, w, v, t;
  // u = (V1.cross(a - p_prime)).dot(normal);
  // w = (V2.cross(b - p_prime)).dot(normal);
  // v = (V3.cross(c - p_prime)).dot(normal);
  // // fx > 0 for CCW
  // // Sandwitched by V2 and V1
  // if (u >= 0 && w <= 0) {
  //   p1 = a;
  //   p2 = b;
  // }
  // // Sandwitched by V3 and V2
  // else if (w >= 0 && v <= 0) {
  //   p1 = b;
  //   p2 = c;
  // }
  // else {
  //   p1 = c;
  //   p2 = a;
  // }

  // Eigen::RowVector3d p_prime_p1 = p1 - p_prime;
  // Eigen::RowVector3d p_prime_p2 = p2 - p_prime;

  // // on the mesh
  // if ((p_prime_p1.cross(p_prime_p2)).dot(normal) < 0) {
  //   p = p_prime;
  //   d = (x - p_prime).norm();
  //   return;
  // }

  // // not on mesh
  // R = ((p_prime_p1).cross(p_prime_p2)).cross(p1-p2);
  // double cos_gamma = (p_prime_p1).dot(R) / p_prime_p1.norm() / R.norm();
  // p_prime2p_pp = cos_gamma * (p_prime_p1).dot(R) * R.normalized();
  // p_pp = p_prime + p_prime2p_pp;
  // // p_pp is the projection of p_prim onto p1_p2
  // // set p accordingly
  // t = (p_pp - p1).norm() / (p2 - p1).norm();
  // if (t >= 0 && t <= 1) {
  //   p = p_pp;
  //   d = (p_pp - x).norm();
  // }
  // if (t < 0) {
  //   p = p1;
  //   d = (p1 - x).norm();
  // }
  // else {
  //   p = p2;
  //   d = (p2 - x).norm();
  // }
  
  // refine the old implementation a bit
  Eigen::RowVector3d ab = b - a;
  Eigen::RowVector3d ac = c - a;
  Eigen::RowVector3d normal = ((ab).cross(ac)).normalized();
  Eigen::RowVector3d p_prime = x + (normal).dot(x - a) * normal;

  double u = (a - p_prime).cross(a - b).dot(normal);
	double w = (b - p_prime).cross(b - c).dot(normal);
	double v = (c - p_prime).cross(c - a).dot(normal);

	if (u >= 0 && w >= 0 && v >= 0) {
		p = p_prime;
    d = (x - p).norm();
    return;
  }

  Eigen::RowVector3d p1, p2;
  if (u < 0) {
    p1 = a;
    p2 = b;
	} else if (w < 0) {
    p1 = b;
    p2 = c;
	} else if (v < 0) {
    p1 = c;
    p2 = a;
	} else {
    std::cout << "How did I get here?" << std::endl;
    std::cout << "u: " << u << std::endl;
    std::cout << "w: " << w << std::endl;
    std::cout << "v: " << v << std::endl;
    std::cout << "====================" << std::endl;
    p1 = a;
    p2 = b;
  }

  p = (p1 - p_prime).dot(p1 - p2) * (p1 - p2) + p1;
	d = (x - p).norm();
}
