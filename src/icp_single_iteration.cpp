#include "icp_single_iteration.h"
#include "point_to_point_rigid_matching.h" 
#include "point_to_plane_rigid_matching.h" 
#include "random_points_on_mesh.h"
#include "point_mesh_distance.h"
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
  Eigen::MatrixXd X;
  random_points_on_mesh(num_samples,VX,FX,X);
  Eigen::VectorXd D; 
  Eigen::MatrixXd P;
  Eigen::MatrixXd N;
  point_mesh_distance(X,VY,FY,D,P,N);
  
  if(method == ICP_METHOD_POINT_TO_POINT){
    point_to_point_rigid_matching(X,P,R,t);
  } else if (method == ICP_METHOD_POINT_TO_PLANE) {
    point_to_plane_rigid_matching(X,P,N,R,t);
  }
  
  
  // I returned R transpose as, in the readme it is Rx + t where x is column vector
  // But in main.cpp, we do instead XR + t, thus to achieve the correct rotation we need to perform transpose on R
  R = R.transpose().eval();
}
