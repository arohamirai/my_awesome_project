#ifndef PLUGINLIB_TUTORIALS__POLYGON_BASE_H_
#define PLUGINLIB_TUTORIALS__POLYGON_BASE_H_

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      virtual void initialize(double side_length);
      virtual double area() = 0;
      virtual ~RegularPolygon(){}
      void message();

    protected:
      RegularPolygon(){}
  };
};
#endif
