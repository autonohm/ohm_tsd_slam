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

namespace ohm_tsd_slam
{

SlamNode::SlamNode()
{
  _initialized=false;
  _mask=NULL;

  _grid=new obvious::TsdGrid(CELLSIZE, obvious::LAYOUT_32x32, obvious::LAYOUT_8192x8192);   //LAYOUT_8192x8192 1024x1024
  _grid->setMaxTruncation(TRUNCATION_RADIUS * CELLSIZE);
  _sensor=NULL;
  _mask=NULL;

  _localizer=NULL;
  _threadMapping=NULL;
  _threadGrid=NULL;

  ros::NodeHandle prvNh("~");
  std::string strVar;
  prvNh.param("laser_topic", strVar, std::string("scan"));
  _laserSubs=_nh.subscribe(strVar, 1, &SlamNode::laserScanCallBack, this);
  prvNh.param<double>("x_off_factor", _xOffFactor, 0.0);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.0);
}

SlamNode::~SlamNode()
{
  if(_initialized)
  {
    _threadMapping->terminateThread();
    _threadGrid->terminateThread();
    while(_threadMapping->alive(1) && _threadGrid->alive(1))
    {
      std::cout << __PRETTY_FUNCTION__ << "Waiting for threads to terminate...\n";
      usleep(1000 * 500);
    }
    delete _sensor;
    delete _mask;
    delete _localizer;
    delete _threadMapping;
    delete _threadGrid;
  }
  delete _grid;
}

void SlamNode::start(void)
{
  this->run();
}

double SlamNode::xOffFactor(void)const
{
  return _xOffFactor;
}

double SlamNode::yOffFactor(void)const
{
  return _yOffFactor;
}

void SlamNode::initialize(const sensor_msgs::LaserScan& initScan)
{
  _mask=new bool[initScan.ranges.size()];
  for(unsigned int i=0;i<initScan.ranges.size();i++)
  {
    _mask[i]=!isnan(initScan.ranges[i])&&!isinf(initScan.ranges[i])&&(fabs(initScan.ranges[i])>10e-6);
    //toDO: Move mask generation into SensorPolar2D
  }

  _sensor=new obvious::SensorPolar2D(initScan.ranges.size(), initScan.angle_increment, initScan.angle_min, MAX_RANGE);
  _sensor->setRealMeasurementData(initScan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);

  double phi       =THETA_INIT * M_PI / 180.0;
  double gridWidth =_grid->getCellsX()*_grid->getCellSize();
  double gridHeight=_grid->getCellsY()*_grid->getCellSize();
  double tf[9]     ={cos(phi), -sin(phi), gridWidth*_xOffFactor,
      sin(phi),  cos(phi), gridHeight*_yOffFactor,
      0,         0,               1};
  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  _sensor->transform(&Tinit);

  _threadMapping=new ThreadMapping(_grid);

  _localizer=new Localization(_grid, _threadMapping, _nh, &_pubMutex, *this);

  for(int i=0; i<INIT_PSHS; i++)
    _threadMapping->queuePush(_sensor);

  _threadGrid=new ThreadGrid(_grid, _nh, &_pubMutex, *this);

  _initialized=true;
}

void SlamNode::run(void)
{
  ros::Time lastMap=ros::Time::now();
  ros::Duration durLastMap=ros::Duration(MAP_T);
  ros::Rate rate(40);   //toDO: launch file parameters
  while(ros::ok())
  {
    ros::spinOnce();
    if(_initialized)
    {
      ros::Time curTime=ros::Time::now();
      if((curTime-lastMap).toSec()>durLastMap.toSec())
      {
        _threadGrid->unblock();     //toDO: This call shall be timed.
        lastMap=ros::Time::now();
      }
    }
    rate.sleep();
  }
}

void SlamNode::laserScanCallBack(const sensor_msgs::LaserScan& scan)
{
  if(!_initialized)
  {
    this->initialize(scan);
    return;
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _mask[i]=!isnan(scan.ranges[i])&&!isinf(scan.ranges[i])&&(fabs(scan.ranges[i])>10e-6);
    //toDO: Move mask generation into SensorPolar2D->setRealMeasurmentData
  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);
  _localizer->localize(_sensor);

}

} /* namespace ohm_tsdSlam2 */
