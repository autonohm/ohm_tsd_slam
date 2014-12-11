/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#include "SlamNode.h"
#include "Localization.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"
#include "obcore/math/mathbase.h"
#include <unistd.h>

namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  ros::NodeHandle prvNh("~");
  std::string strVar;
  int octaveFactor        = 0;
  double cellside         = 0.0;
  double dVar             = 0;
  int iVar                = 0;
  double truncationRadius = 0.0;
  prvNh.param        ("laser_topic", strVar, std::string("simon/scan"));
  prvNh.param<double>("x_off_factor", _xOffFactor, 0.2);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);
  prvNh.param<double>("yaw_offset", _yawOffset, 0.0);
  prvNh.param<int>   ("cell_octave_factor", octaveFactor, 10);
  prvNh.param<double>("cellsize", cellside, 0.025);
  prvNh.param<int>   ("truncation_radius", iVar, 3);
  truncationRadius = static_cast<double>(iVar);
  prvNh.param<double>("min_range", _minRange, 0.01);
  prvNh.param<double>("max_range", _maxRange, 30.0);
  prvNh.param<double>("low_reflectivity_range", _lowReflectivityRange, 2.0);
  prvNh.param<double>("occ_grid_time_interval", _gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", _loopRate, 40.0);
  prvNh.param<double>("footprint_width" , _footPrintWidth, 0.1);
  prvNh.param<double>("footprint_height", _footPrintHeight, 0.1);

  _laserSubs=_nh.subscribe(strVar, 1, &SlamNode::laserScanCallBack, this);

  unsigned int uiVar = static_cast<unsigned int>(octaveFactor);
  if((uiVar > 15) || (uiVar < 5))
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Unknown / Invalid cell_octave_factor -> set to default!" << std::endl;
    uiVar = 10;
  }
  _initialized = false;
  _grid        = new obvious::TsdGrid(cellside, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(uiVar));
  _grid->setMaxTruncation(truncationRadius * cellside);

  unsigned int cellsPerSide = std::pow(2, uiVar);
  std::cout << __PRETTY_FUNCTION__ << " creating representation with " << cellsPerSide << "x" << cellsPerSide;
  double sideLength = static_cast<double>(cellsPerSide) * cellside;
  std::cout << " cells, representating "<< sideLength << "x" << sideLength << "m^2" << std::endl;

  _sensor        = NULL;
  _localizer     = NULL;
  _threadMapping = NULL;
  _threadGrid    = NULL;
}

SlamNode::~SlamNode()
{
  if(_initialized)
  {
    _threadMapping->terminateThread();
    _threadGrid->terminateThread();
    delete _threadGrid;
    delete _threadMapping;
  }
  if(_localizer) delete _localizer;
  if(_grid) delete _grid;
  if(_sensor) delete _sensor;
}

void SlamNode::start(void)
{
  this->run();
}

void SlamNode::initialize(const sensor_msgs::LaserScan& initScan)
{
  _sensor = new obvious::SensorPolar2D(initScan.ranges.size(), initScan.angle_increment, initScan.angle_min, static_cast<double>(_maxRange), static_cast<double>(_minRange), static_cast<double>(_lowReflectivityRange));
  _sensor->setRealMeasurementData(initScan.ranges, 1.0);

  const double phi       = _yawOffset;
  const double gridWidth =_grid->getCellsX()*_grid->getCellSize();
  const double gridHeight=_grid->getCellsY()*_grid->getCellSize();
  const obfloat startX = static_cast<obfloat>(gridWidth * _xOffFactor);
  const obfloat startY = static_cast<obfloat>(gridHeight * _yOffFactor);

  double tf[9]     ={cos(phi), -sin(phi), gridWidth*_xOffFactor,
      sin(phi),  cos(phi), gridHeight*_yOffFactor,
      0,         0,               1};
  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  _sensor->transform(&Tinit);

  _threadMapping = new ThreadMapping(_grid);
  const obfloat t[2] = {startX, startY};
  if(!_grid->freeFootprint(t, _footPrintWidth, _footPrintHeight))
    std::cout << __PRETTY_FUNCTION__ << " warning! Footprint could not be freed!\n";
  _threadMapping->initPush(_sensor);

  _localizer   = new Localization(_grid, _threadMapping, _nh, _xOffFactor, _yOffFactor);
  _threadGrid  = new ThreadGrid(_grid, _nh, _xOffFactor, _yOffFactor);
  _initialized = true;
}

void SlamNode::run(void)
{
  ros::Time lastMap=ros::Time::now();
  ros::Duration durLastMap=ros::Duration(_gridPublishInterval);
  ros::Rate rate(_loopRate);
  std::cout << __PRETTY_FUNCTION__ << " waiting for first laser scan to initialize node...\n";
  while(ros::ok())
  {
    ros::spinOnce();
    if(_initialized)
    {
      ros::Time curTime=ros::Time::now();
      if((curTime-lastMap).toSec()>durLastMap.toSec())
      {
        _threadGrid->unblock();
        lastMap=ros::Time::now();
      }
    }
    rate.sleep();
  }
}

void SlamNode::laserScanCallBack(const sensor_msgs::LaserScan& scan)
{
  sensor_msgs::LaserScan tmpScan = scan;
  for(std::vector<float>::iterator iter = tmpScan.ranges.begin(); iter != tmpScan.ranges.end(); iter++)
    if(isnan(*iter))
    {
      *iter = 0.0;
    }
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";
    this->initialize(tmpScan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
    return;
  }
  _sensor->setRealMeasurementData(tmpScan.ranges, 1.0);
  _sensor->resetMask();
  _sensor->maskDepthDiscontinuity(obvious::deg2rad(3.0));
  _localizer->localize(_sensor);
}

} /* namespace ohm_tsdSlam2 */
