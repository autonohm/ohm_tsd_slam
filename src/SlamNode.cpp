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

namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  ros::NodeHandle prvNh("~");
  std::string strVar;
  int iVar = 0;
  prvNh.param("laser_topic", strVar, std::string("simon/scan"));
  _laserSubs=_nh.subscribe(strVar, 1, &SlamNode::laserScanCallBack, this);
  //_laserSubs=_nh.subscribe(strVar, 1, &SlamNode::multiScanCallBack, this);
  prvNh.param<double>("max_range", _maxRange, 30.0);
  prvNh.param<double>("low_reflectivity_range", _lowReflectivityRange, 2.0);
  prvNh.param<int>("multi_scans", iVar, 2);
  _multiScanNumber = static_cast<unsigned int>(iVar);
  _sensor      = NULL;
  _mask        = NULL;
  _localizer   = NULL;
  std::cout << __PRETTY_FUNCTION__ << " low ref = " << _lowReflectivityRange << std::endl;
}

SlamNode::~SlamNode()
{
  if(_initialized)
  {
    delete _localizer;
    delete _sensor;
    delete [] _mask;
  }
}

//void SlamNode::start(void)
//{
//  this->run();
//}

void SlamNode::initialize(const sensor_msgs::LaserScan& initScan)
{
  _mask=new bool[initScan.ranges.size()];
  for(unsigned int i=0;i<initScan.ranges.size();i++)
  {
    _mask[i]=!isnan(initScan.ranges[i])&&!isinf(initScan.ranges[i])&&(fabs(initScan.ranges[i])>10e-6);
  }

  _sensor = new obvious::SensorPolar2D(initScan.ranges.size(), initScan.angle_increment, initScan.angle_min, static_cast<double>(_maxRange), 0.0, _lowReflectivityRange);
  _sensor->setRealMeasurementData(initScan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);

  double phi       = _yawOffset;
  double gridWidth =_grid->getCellsX()*_grid->getCellSize();
  double gridHeight=_grid->getCellsY()*_grid->getCellSize();
  double tf[9]     ={cos(phi), -sin(phi), gridWidth*_xOffFactor,
      sin(phi),  cos(phi), gridHeight*_yOffFactor,
      0,         0,               1};
  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  _sensor->transform(&Tinit);

  _threadMapping=new ThreadMapping(_grid);

  for(int i=0; i<INIT_PSHS; i++)
    _threadMapping->queuePush(_sensor);


  _localizer=new Localization(_grid, _threadMapping, &_pubMutex, _xOffFactor, _yOffFactor);

  for(int i=0; i<INIT_PSHS; i++)
    _threadMapping->queuePush(_sensor);

  _threadGrid=new ThreadGrid(_grid, _nh, &_pubMutex, _xOffFactor, _yOffFactor);

  _initialized=true;
}

void SlamNode::run(void)
{
  std::cout << __PRETTY_FUNCTION__ << " waiting for first laser scan to initialize node...\n";
  while(ros::ok())
  {
    ros::spinOnce();
    if(_initialized)
      this->timedGridPub();
  }
  _loopRate->sleep();
}

void SlamNode::laserScanCallBack(const sensor_msgs::LaserScan& scan)
{
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";
    this->initialize(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
    return;
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=(!isnan(scan.ranges[i])&&(fabs(scan.ranges[i])>10e-6));
  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);
  _sensor->maskDepthDiscontinuity(obvious::deg2rad(3.0));
  _localizer->localize(_sensor);
}

void SlamNode::multiScanCallBack(const sensor_msgs::LaserScan& scan)
{
  static unsigned int ctr = 0;
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";
    this->initialize(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
    return;
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=!isnan(scan.ranges[i]) && (fabs(scan.ranges[i])>10e-6);
  }
//  if(ctr == 0)
//  {
//
//  }
  if(ctr++ < _multiScanNumber)
  {
    obvious::SensorPolar2D* sensor = new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, static_cast<double>(_maxRange), 0.0, _lowReflectivityRange);
    sensor->setRealMeasurementData(scan.ranges, 1.0);
    sensor->setRealMeasurementMask(_mask);
    sensor->setTransformation(_sensor->getTransformation());
    _multiScans.push_back(sensor);
  }
  else
  {
    _sensor->setRealMeasurementData(scan.ranges, 1.0);
    _sensor->setRealMeasurementMask(_mask);
    _localizer->multiScanLocalize(_multiScans, _sensor);
    for (unsigned int i = 0; i < _multiScans.size(); i++)
    {
      delete _multiScans[i];   //toDo: check local output. Registration error will crash everything
      _multiScans.clear();
    }
    ctr = 0;
  }
}

} /* namespace ohm_tsdSlam2 */
