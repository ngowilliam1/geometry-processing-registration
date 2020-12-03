#include "random_points_on_mesh.h"
#include <igl/doublearea.h>
#include <igl/cumsum.h>
#include <random>

int IDXBinarySearch(Eigen::VectorXd const &C, int start, int end, double const &rv, int const &trueEnd){
  if (end > start){
    int mid = (start+end)/2;
    if (C(mid) > rv){
      return IDXBinarySearch(C, start, mid, rv, trueEnd);
    }
    else {
      return IDXBinarySearch(C, mid+1, end, rv, trueEnd);
    }
  }
  // In the possibility that rv=1, end will evaluate to C.rows() so we will take a precaution and take C.rows()-1 instead
  // rv should basically never evaluate to exactly 1 anyways...
  return std::min(end,trueEnd);
}

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
  // REPLACE WITH YOUR CODE:
  X.resize(n,3);
  // for(int i = 0;i<X.rows();i++) X.row(i) = V.row(i%V.rows());


  // Define a uniform random generator as in http://www.cplusplus.com/reference/random/uniform_real_distribution/
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  
  // first calculate a vector of cumulative sum of the relative areas
  Eigen::VectorXd C;
  Eigen::VectorXd area;
  igl::doublearea(V, F, area);
  area /= 2;
  igl::cumsum(area, 1, C);
  double totalArea = C(C.rows()-1);
  C /= totalArea;


  for (int i =0; i < n ; i++){
    // first choose which triangle to sample from
    double idx_random_uniform = distribution(generator);
    int triangleIdx = IDXBinarySearch(C, 0, C.rows(), idx_random_uniform, (C.rows()-1));
    
    Eigen::RowVector3d v1 = V.row(F(triangleIdx, 0));
    Eigen::RowVector3d v2 = V.row(F(triangleIdx, 1));
    Eigen::RowVector3d v3 = V.row(F(triangleIdx, 2));

    // secondly choose a point within the triangle
    double a1 = distribution(generator);
    double a2 = distribution(generator);
    if (a1 + a2 > 1){
      a1 = 1-a1;
      a2 = 1-a2;
    }
    X.row(i) = v1 + a1*(v2-v1) + a2*(v3-v1); 
  }
}

