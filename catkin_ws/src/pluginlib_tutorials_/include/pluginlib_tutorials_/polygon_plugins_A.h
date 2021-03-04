#ifndef PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_
#define PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_
#include <pluginlib_tutorials_/polygon_base.h>
#include <pluginlib_tutorials_/third_part.h>
#include <cmath>

namespace polygon_plugins
{
  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      Triangle(){}

      void initialize(double side_length)
      {
        side_length_ = side_length;
        func();
      }

      double area();

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
      }
    private:
      double side_length_;
  };

};
#endif
