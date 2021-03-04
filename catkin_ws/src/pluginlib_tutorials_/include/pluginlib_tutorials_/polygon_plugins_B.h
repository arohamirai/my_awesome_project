#ifndef PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_B
#define PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_B
#include <pluginlib_tutorials_/polygon_base.h>
#include <pluginlib_tutorials_/third_part.h>
#include <cmath>

namespace polygon_plugins
{
  class Square : public polygon_base::RegularPolygon
  {
    public:
      Square(){}

      void initialize(double side_length)
      {
        side_length_ = side_length;
func();
      }

      double area()
      {
        return side_length_ * side_length_;
      }
    private:
      double side_length_;

  };
};
#endif
