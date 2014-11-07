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
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, boost::mutex* pubMutex, MultiSlamNode& parentNode, std::string nameSpace):
        _sensor(NULL),
        _newScan(false),
        _initialized(false),
        _parentNode(parentNode)
{
  /**
   * width
   * height
   * namespace
   * laser topic
   * x offset
   * y offset
   * yaw offset
   * tf_base
   * tf_child
   */

  ros::NodeHandle prvNh("~");

  std::string laserTopic;
  prvNh.param("laser_topic", laserTopic, std::string("scan"));
  laserTopic = nameSpace + "/" + laserTopic;

  std::string xOffParamServer;
  xOffParamServer = nameSpace + "/x_offset";
  prvNh.param<double>(xOffParamServer, _xOffset, 0.0);

  std::string yOffParamServer;
  yOffParamServer = nameSpace + "/y_offset";
  prvNh.param<double>(yOffParamServer, _yOffset, 0.0);

  std::string yawOffParamServer;
  yOffParamServer = nameSpace + "/yaw_offset";
  prvNh.param<double>(yOffParamServer, _yawOffset, 0.0);

  std::string maxRangeParamServer;
  maxRangeParamServer = nameSpace + "/max_range";
  prvNh.param<double>(maxRangeParamServer, _maxRange, 30.0);

  std::string minRangeParamServer;
  maxRangeParamServer = nameSpace + "/min_range";
  prvNh.param<double>(maxRangeParamServer, _minRange, 0.001);

  _gridWidth = grid->getCellsX() * grid->getCellSize();
  _gridHeight = grid->getCellsY() * grid->getCellSize();
  double _gridHeight;

  _xOffFactor = parentNode.xOffFactor();
  _yOffFactor = parentNode.yOffFactor();

  _localizer = new Localization(grid, mapper, pubMutex, parentNode, nameSpace);
  _sensor = NULL;

  _lasSubs = _nh.subscribe(laserTopic, 1, &ThreadLocalize::laserCallBack, this);
}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _localizer;
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
    else
    {
      //maybe publish pose only ?
    }
    _newScan = false;
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  //  _mask = new bool[scan.ranges.size()];
  //  for(unsigned int i=0; i < scan.ranges.size(); i++)
  //  {
  //    _mask[i] = !isnan(scan.ranges[i]) && !isinf(scan.ranges[i])&&(std::abs(scan.ranges[i])>10e-6);
  //  }

  _sensor = new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, static_cast<double>(_maxRange));
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  //_sensor->setRealMeasurementMask(_mask);

  double phi    = _yawOffset;
  double startX = _gridWidth*_xOffFactor; //toDo: add offset from this zero point from lauch
  double startY = _gridWidth*_yOffFactor;
  double tf[9]  = {std::cos(phi), -std::sin(phi), startX,
                   std::sin(phi),  std::cos(phi), startY,
                               0,              0,      1};


  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  std::cout << __PRETTY_FUNCTION__ << "Tinit = \n" << Tinit << std::endl;
  _sensor->transform(&Tinit);
  _parentNode.initPush(_sensor);
  _initialized = true;

}

void ThreadLocalize::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";   //toDo: print ID of referring robot
    this->init(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
  }
//  for(unsigned int i=0;i<scan.ranges.size();i++)
//  {
//
//    _mask[i] = !isnan(scan.ranges[i]) && !isinf(scan.ranges[i]) && (fabs(scan.ranges[i]) > 10e-6);
//    if((_rangeFilter)&&_mask[i])
//      _mask[i]=(scan.ranges[i]>_minRange)&&(scan.ranges[i]<_maxRange);
//  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  //_sensor->setRealMeasurementMask(_mask);
  _newScan = true;
}

} /* namespace ohm_tsd_slam */
