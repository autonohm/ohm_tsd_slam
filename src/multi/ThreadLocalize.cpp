/*
 * ThreadLocalize.cpp
 *
 *  Created on: 28.10.2014
 *      Author: phil
 */

#include "ThreadLocalize.h"

#include "MultiSlamNode.h"
#include "ThreadMapping.h"
#include "Localization.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"

#include <boost/bind.hpp>

#include <cstring>
#include <unistd.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, boost::mutex* pubMutex, std::string nameSpace,
    const double xOffFactor, const double yOffFactor):
                    _grid(*grid),
                    _mapper(*mapper),
                    _nameSpace(nameSpace),
                    _localizer(NULL),
                    _sensor(NULL),
                    _newScan(false),
                    _initialized(false),
                    _mask(NULL)
{
  ros::NodeHandle prvNh("~");

  std::string laserTopic;
  prvNh.param("laser_topic", laserTopic, std::string("scan"));
  laserTopic = nameSpace + "/" + laserTopic;
  prvNh.param<double>(nameSpace + "/x_offset"        , _xOffset, 0.0);
  prvNh.param<double>(nameSpace + "/y_offset"        , _yOffset, 0.0);
  prvNh.param<double>(nameSpace + "/yaw_offset"      , _yawOffset, 0.0);
  prvNh.param<double>(nameSpace + "/max_range"       , _maxRange, 30.0);
  prvNh.param<double>(nameSpace + "/min_range"       , _minRange, 0.001);
  prvNh.param<double>(nameSpace + "/footprint_width" , _footPrintWidth, 0.0);
  prvNh.param<double>(nameSpace + "/footprint_height", _footPrintHeight, 0.0);
  std::cout << __PRETTY_FUNCTION__ << " min range set but currently not used." << std::endl;

  _gridWidth  = grid->getCellsX() * grid->getCellSize();
  _gridHeight = grid->getCellsY() * grid->getCellSize();
  _xOffFactor = xOffFactor;
  _yOffFactor = yOffFactor;
  _localizer  = new Localization(grid, mapper, pubMutex, _xOffFactor, _yOffFactor, nameSpace);
  _lasSubs    = _nh.subscribe(laserTopic, 1, &ThreadLocalize::laserCallBack, this);
}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _localizer;
  delete _mask;
}

void ThreadLocalize::eventLoop(void)
{
  while(_stayActive)
  {
    ros::spinOnce();
    if(_newScan)
    {
      _localizer->localize(_sensor);
    }
    _newScan = false;
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  _sensor = new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, static_cast<double>(_maxRange));
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _mask = new bool[scan.ranges.size()];
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=!isnan(scan.ranges[i]);
  }
  _sensor->setRealMeasurementMask(_mask);

  double phi    = _yawOffset;
  double startX = _gridWidth * _xOffFactor + _xOffset;
  double startY = _gridWidth * _yOffFactor + _yOffset;
  double tf[9]  = {std::cos(phi), -std::sin(phi), startX,
      std::sin(phi),  std::cos(phi), startY,
      0,              0,      1};

  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  _sensor->transform(&Tinit);
  obfloat t[2] = {startX, startY};
  if(!_grid.freeFootprint(t, _footPrintWidth, _footPrintHeight))
    std::cout << __PRETTY_FUNCTION__ << " warning! Footprint could not be freed!\n";
  _mapper.initPush(_sensor);
  _initialized = true;

}

void ThreadLocalize::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";   //toDo: print ID of referring robot
    this->init(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
    return;
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=!isnan(scan.ranges[i]);
  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);
  _sensor->maskDepthDiscontinuity(obvious::deg2rad(3.0));
  _newScan = true;
}

} /* namespace ohm_tsd_slam */
