#include "point_triangle_distance.h"
#include <Eigen/Dense>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{ 
  // Replace with your code

  // First Define x0 as closest point from x to plane spanned by abc
  Eigen::RowVector3d aToB = b - a;
  Eigen::RowVector3d aToC = c - a;
  Eigen::RowVector3d aToX = x - a;
  // n is normalized normal vector of plane spanned by abc
  Eigen::RowVector3d n = aToB.cross(aToC).normalized();


  double dist = abs(aToX.dot(n));
  Eigen::RowVector3d x0 = x - dist * n;
  Eigen::RowVector3d aToX0 = x0 - a;
  
  // Secondly Determine Barycentric Coordinates: https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
  double ab = aToB.dot(aToB);
  double ab_ac = aToB.dot(aToC);
  double ac = aToC.dot(aToC);
  double aX0_ab = aToX0.dot(aToB);
  double aX0_ac = aToX0.dot(aToC);
  double denom = ab * ac - ab_ac*ab_ac;
  double v = (ac * aX0_ab - ab_ac * aX0_ac) / denom; 
  double w = (ab * aX0_ac - ab_ac * aX0_ab) / denom; 
  double u = 1 - v - w;
  // We notice two main scenarios:
  // 1. Projection of the point that lays on the plane spanned by abc is inside the triangle (represented by v,w,u)
  // 2. Else, in which we have to select a point on the line segements either AB, AC or BC
  // We can clamp scenario two so that we have scenario 1
  
  if (u<0){
    u = 0;
  }
  if (u>1){
    u = 1;
  }
  if (v<0){
    v = 0;
  }
  if (v>1){
    v = 1;
  }
  w = 1 - u - v;
  

  p = u*a + v*b + w*c;
  d = (p - x).norm();

}