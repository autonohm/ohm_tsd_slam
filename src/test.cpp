#include <ros/ros.h>
#include "registration/Registration.h"
#include <tinyxml2.h>
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"

int main(int argc, char** argv)
{
ros::init(argc, argv, "teset");
ros::NodeHandle nh;
 obvious::SensorPolar2D sensor(200, 0.1, 0.1);
 TsdGrid grid(0.05, obvious::LAYOUT_1024x1024, obvious::LAYOUT_8192x8192);  

try
{
 Registration registration(grid, sensor); 
}
catch(char const*& e)
{
  std::cout << e << '\n';
}

 
 //TsdPdfMatcher matcher(sensor);
return 0;
}