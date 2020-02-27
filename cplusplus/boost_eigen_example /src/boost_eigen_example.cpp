#include "Eigen/Core"
#include "Eigen/Geometry"
#include <boost/variant.hpp>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{

  boost::variant<int, double, std::string> v;
  v = 1;
  std::cout << boost::get<int>(v) << std::endl;
  v = "hello world";
  std::cout << boost::get<std::string>(v) << std::endl;

  return 0;
}
