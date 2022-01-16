#include <iostream>
#include <utility>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cassert>


#include "parse_stl.h"


using Eigen::MatrixXd;



int main(int argc, char* argv[])
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
  srand(time(NULL));
  double R = (double)(rand()%1000)/1000;
  printf("%lf\n", R);
  R = (double)(rand()%1000)/1000;
  printf("%lf\n", R);
  R = (double)(rand()%1000)/1000;
  printf("%lf\n", R);
  R = (double)(rand()%1000)/1000;
  printf("%lf\n", R);
  R = (double)(rand()%1000)/1000;
  printf("%lf\n", R);

  std::string stl_file_name = "../stl_models/compresor_cover.stl";

  if (argc == 2) {
    stl_file_name = argv[1];
  } else if (argc > 2) {
    std::cout << "ERROR: Too many command line arguments" << std::endl;
  }

  auto info = stl::parse_stl(stl_file_name);

  std::vector<stl::triangle> triangles = info.triangles;
  std::cout << "STL HEADER = " << info.name << std::endl;
  std::cout << "# triangles = " << triangles.size() << std::endl;
  
  // for (auto t : info.triangles) {
  //   std::cout << t << std::endl ;
  // }
  
  
  return 0;

}
