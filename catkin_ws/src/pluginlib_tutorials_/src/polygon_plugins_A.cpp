#include <pluginlib/class_list_macros.h>
#include <pluginlib_tutorials_/polygon_base.h>
#include <pluginlib_tutorials_/polygon_plugins_A.h>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)

      double polygon_plugins::Triangle::area()
      {
        return 0.5 * side_length_ * getHeight();
      }
