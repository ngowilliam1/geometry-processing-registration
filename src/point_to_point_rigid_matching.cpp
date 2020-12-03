#include "point_to_point_rigid_matching.h"
#include "closest_rotation.h"

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t) {
    
    // compute centroids for P and X
    Eigen::RowVector3d xCentroid = X.colwise().mean();
    Eigen::RowVector3d pCentroid = P.colwise().mean();

    // compute coordinates for the points in X and P relative to their centroids
    Eigen::MatrixXd XbarMatrix;
    Eigen::MatrixXd PbarMatrix;
    
    XbarMatrix.resizeLike(X);
    PbarMatrix.resizeLike(P);
    XbarMatrix = X.rowwise() - xCentroid;
    PbarMatrix = P.rowwise() - pCentroid;
 
    
    // compute M using the relative coordinates
    Eigen::Matrix3d M = PbarMatrix.transpose()*XbarMatrix;

    
    closest_rotation(M, R);
    t = (pCentroid.transpose() - R*xCentroid.transpose()).transpose();
    
}