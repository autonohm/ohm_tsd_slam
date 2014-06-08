#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

#include "Localization.h"
#include "ThreadSLAM.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"
#include "obcore/base/Logger.h"

using namespace ohm_tsd_slam;
using namespace obvious;

#define CELLSIZE 0.025
#define TRUNCATION_RADIUS 3.0
#define MAX_RANGE 30.0
#define MIN_RANGE 0.01
#define S_X_F 0.5                  //start point in x (S_F_X * sizeX)
#define S_Y_F 0.5                  //start point in y (S_F_Y * sizeY)
#define THETA_INIT 0.0  //used in degrees
#define MAP_T 2.0      //time between map generations

ros::Subscriber _lasSubs;
ros::Publisher _posePub;
bool _initialized = false;

bool* _mask = NULL;

obvious::TsdGrid* _grid;
obvious::SensorPolar2D* _sensor;

Localization* _localizer = NULL;
ThreadMapping* _threadMapping = NULL;
ThreadGrid* _threadGrid = NULL;
boost::mutex _pubMutex;

void laserScanCallBack(const sensor_msgs::LaserScan& scan)
{
  bool mask[scan.ranges.size()];
  unsigned int cnt = 0;
  for(std::vector<float>::const_iterator iter = scan.ranges.begin(); iter != scan.ranges.end(); iter++)
  {
    mask[cnt++] = !isnan(*iter) && !isinf(*iter) && (fabs(*iter)>10e-6);
  }

  if(!_initialized)
  {
    _sensor = new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, MAX_RANGE);
    _sensor->setRealMeasurementData(scan.ranges, 1.0);
    _sensor->setRealMeasurementMask(mask);

    double phi   = THETA_INIT * M_PI / 180.0;
    double gridWidth = _grid->getCellsX() * _grid->getCellSize();
    double gridHeight = _grid->getCellsY() * _grid->getCellSize();
    double tf[9] = {cos(phi), -sin(phi), gridWidth * S_X_F,
        sin(phi),  cos(phi), gridHeight * S_Y_F,
        0,              0,             1};
    obvious::Matrix Tinit(3, 3);
    Tinit.setData(tf);
    _sensor->transform(&Tinit);

    _threadMapping = new ThreadMapping(_grid);

    ros::NodeHandle nh;
    _localizer = new Localization(_grid, _threadMapping, nh, &_pubMutex);

    for(int i=0; i<1; i++)
      _threadMapping->queuePush(_sensor);

    _threadGrid = new ThreadGrid(_grid, nh, &_pubMutex);

    _initialized = true;
  }
  else
  {
    _sensor->setRealMeasurementData(scan.ranges, 1.0);
    _sensor->setRealMeasurementMask(mask);
    _localizer->localize(_sensor);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ohm_tsd_slam_node");
  ros::NodeHandle nh;

  ros::Time lastMap = ros::Time::now();
  ros::Duration durLastMap = ros::Duration(MAP_T);

  //initialize representation
  double cellSize = CELLSIZE;
  _grid = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, obvious::LAYOUT_8192x8192);
  _grid->setMaxTruncation(TRUNCATION_RADIUS * cellSize);

  ros::NodeHandle prvNh("~");
  std::string strVar;
  prvNh.param("laser_topic", strVar, std::string("/scan"));
  _lasSubs = nh.subscribe(strVar, 1, &laserScanCallBack);

  LOGMSG_CONF("slamlog.log", Logger::file_on|Logger::screen_off, DBG_DEBUG, DBG_ERROR);

  ros::Rate rate(40);
  while(ros::ok())
  {
    ros::spinOnce();
    if(_initialized)
    {
      ros::Time curTime = ros::Time::now();
      if((curTime - lastMap).toSec() > durLastMap.toSec())
      {
        _threadGrid->unblock();     //toDO: This call shall be timed.
        lastMap = ros::Time::now();
      }
    }
    rate.sleep();
  }
//  _threadMapping->terminateThread();
//  _threadGrid->terminateThread();
//  while(_threadMapping->alive(1) && _threadGrid->alive(1))
//  {
//    std::cout << __PRETTY_FUNCTION__ << "Waiting for threads to terminate...\n";
//    usleep(1000 * 500);
//  }
//  delete _mask;
//  delete _sensor;
//  delete _grid;
//  delete _localizer;
//
//
//  delete _threadMapping;
//  delete _threadGrid;
}

