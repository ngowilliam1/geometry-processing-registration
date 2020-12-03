#include "point_to_plane_rigid_matching.h"
#include <Eigen/Dense>
void point_to_plane_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & N,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{

  Eigen::MatrixXd A(6,6);
  A << 0,0,0,0,0,0,
       0,0,0,0,0,0,
       0,0,0,0,0,0,
       0,0,0,0,0,0,
       0,0,0,0,0,0,
       0,0,0,0,0,0;
  
  Eigen::VectorXd B(6);
  B << 0,0,0,0,0,0;
  
  // Calculate A and B by looping over element in x (to perform a sum over i=1 to k in the readme)
  for (int i=0; i<X.rows(); i++){
    // For A
    Eigen::MatrixXd tempA(6,6);
    // tempALeft stores [xi.cross(ni), ni] as a column vector
    Eigen::VectorXd tempALeft(6);
    Eigen::Vector3d iRowX = X.row(i);
    Eigen::Vector3d iRowN = N.row(i);

    Eigen::Vector3d XiCrossNi = iRowX.cross(iRowN);
    tempALeft << XiCrossNi,
                 iRowN;
   
    Eigen::RowVectorXd tempARight(6);
    tempARight << XiCrossNi.transpose(), iRowN.transpose();
    
    tempA = tempALeft * tempARight;
    A += tempA;

    // For B
    Eigen::VectorXd tempB(6);
    Eigen::Vector3d iRowP = P.row(i);
    tempB = tempALeft * iRowN.transpose() * (iRowP-iRowX);
    B += tempB;
  }
  // solving Au = B
  Eigen::VectorXd u = A.llt().solve(B);

  // a stores first 3 entries in u
  Eigen::Vector3d a;
  a << u(0), u(1), u(2);
  // last 3 entries in u represent t
  t(0) = u(3);
  t(1) = u(4);
  t(2) = u(5);

  
  double theta = a.norm();
  Eigen::Vector3d wHat = a/theta;
  Eigen::Matrix3d W;
  W << 0,      -wHat(2), wHat(1),
       wHat(2),    0,    -wHat(0),
       -wHat(1), wHat(0), 0;
  // perform axis-angle to matrix formula :https://github.com/alecjacobson/geometry-processing-registration#aa
  R = Eigen::Matrix3d::Identity() +  sin(theta)*W + (1-cos(theta))*W*W;
}
