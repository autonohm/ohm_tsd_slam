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

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
    const double xOffFactor, const double yOffFactor):
                                _nh(nh),
                                _grid(*grid),
                                _mapper(*mapper),
                                _localizer(NULL),
                                _sensor(NULL),
                                _newScan(false),
                                _initialized(false),
                                _nameSpace(nameSpace),
                                _mask(NULL)
{
  _gridWidth  = grid->getCellsX() * grid->getCellSize();
  _gridHeight = grid->getCellsY() * grid->getCellSize();
  _xOffFactor = xOffFactor;
  _yOffFactor = yOffFactor;
  _localizer  = new Localization(grid, mapper, *_nh, _xOffFactor, _yOffFactor, nameSpace);
}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _localizer;
  delete _mask;
}

bool ThreadLocalize::setData(const sensor_msgs::LaserScan& scan)
{

  if(!_dataMutex.try_lock())
    return false;
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";   //toDo: print ID of referring robot
    this->init(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
    _dataMutex.unlock();
    return false;
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=!isnan(scan.ranges[i]);
  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);
  _sensor->maskDepthDiscontinuity(obvious::deg2rad(3.0));
  _sensor->maskZeroDepth();
  _newScan = true;
  _dataMutex.unlock();
  return true;
}

void ThreadLocalize::eventLoop(void)
{
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);
    if(_newScan)
    {
      _dataMutex.lock();
      _localizer->localize(_sensor);
      _dataMutex.unlock();
    }
    _newScan = false;
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  double xOffset   = 0.0;
  double yOffset= 0.0;
  double yawOffset= 0.0;
  double maxRange = 0.0;
  double minRange = 0.0;
  double lowReflectivityRange = 0.0;
  double footPrintWidth= 0.0;
  double footPrintHeight= 0.0;
  double footPrintXoffset= 0.0;

  ros::NodeHandle prvNh("~");

  prvNh.param<double>(_nameSpace + "/x_offset"              , xOffset             , 0.0);
  prvNh.param<double>(_nameSpace + "/y_offset"              , yOffset             , 0.0);
  prvNh.param<double>(_nameSpace + "/yaw_offset"            , yawOffset           , 0.0);
  prvNh.param<double>(_nameSpace + "/max_range"             , maxRange            , 30.0);
  prvNh.param<double>(_nameSpace + "/min_range"             , minRange            , 0.001);
  prvNh.param<double>(_nameSpace + "/low_reflectivity_range", lowReflectivityRange, 2.0);
  prvNh.param<double>(_nameSpace + "/footprint_width"       , footPrintWidth      , 0.0);
  prvNh.param<double>(_nameSpace + "/footprint_height"      , footPrintHeight     , 0.0);
  prvNh.param<double>(_nameSpace + "/footprint_x_offset"    , footPrintXoffset    , 0.28);

  double phi    = yawOffset;
  double startX = _gridWidth * _xOffFactor + xOffset;
  double startY = _gridWidth * _yOffFactor + yOffset;
  double tf[9]  = {std::cos(phi), -std::sin(phi), startX,
      std::sin(phi),  std::cos(phi), startY,
      0,              0,      1};
  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);

  _sensor = new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, maxRange, minRange, lowReflectivityRange);
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _mask = new bool[scan.ranges.size()];
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=!isnan(scan.ranges[i]);
  }
  _sensor->setRealMeasurementMask(_mask);
  _sensor->transform(&Tinit);
  obfloat t[2] = {startX + footPrintXoffset, startY};
  if(!_grid.freeFootprint(t, footPrintWidth, footPrintHeight))
    std::cout << __PRETTY_FUNCTION__ << " warning! Footprint could not be freed!\n";
  if(!_mapper.initialized())
    _mapper.initPush(_sensor);
  _initialized = true;
}

} /* namespace ohm_tsd_slam */
