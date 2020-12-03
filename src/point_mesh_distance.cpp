#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <igl/per_face_normals.h>

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
  N.resizeLike(X);
  D.resize(X.rows());

  //Determine faceNormals
  Eigen::MatrixXd faceNormals;
  igl::per_face_normals(VY, FY, Eigen::Vector3d(1,1,1).normalized(), faceNormals);

  double minDist, tempDist;
  int idxOfMin;
  for (int row = 0; row < X.rows(); row++){
    double minDist = -0.5;
    Eigen::RowVector3d pointToTest;
    Eigen::RowVector3d currentShortestPoint;
    Eigen::RowVector3d closest_n;
    // Loop over each item in FY
    for(int face=0; face < FY.rows();face++){
      int idxA = FY(face,0);
      int idxB = FY(face,1);
      int idxC = FY(face,2);
      point_triangle_distance(X.row(row), VY.row(idxA), VY.row(idxB), VY.row(idxC), tempDist, pointToTest);
      if (minDist < 0 || tempDist < minDist){
        currentShortestPoint = pointToTest;
        minDist = tempDist;
        closest_n = faceNormals.row(face);
      }
    }
    P.row(row) = currentShortestPoint;
    D(row) = minDist;
    N.row(row) = closest_n;
  }
  
  
}
