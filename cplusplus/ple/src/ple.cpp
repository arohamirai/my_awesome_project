#include "libmatlabPLE.h"
#include "mclmcr.h"
#include "matrix.h"
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  if( !libmatlabPLEInitialize())
  {
    std::cout << "Could not initialize libmyFunc!" << std::endl;
    return -1;
  }
  std::cout << "Matlab library initialize success!" << std::endl;

  std::cout << "FUCK" << std::endl;
  try
  {
    std::cout << "FUCK2" << std::endl;
    ple_algorithm();
  }
  catch (std::exception& e)
  {
    std::cout << "e: " << e.what() << std::endl;
  }
  return 0;
}
