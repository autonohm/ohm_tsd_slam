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
#include "ThreadLocalize.h"
#include "LaserCallBackObject.h"

#include "obcore/math/mathbase.h"

namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  std::cout << __PRETTY_FUNCTION__ << " entry" << std::endl;
  ros::NodeHandle prvNh("~");
  std::string strVar;
  std::string nodeControlTopic;
  prvNh.param("laser_topic", _laserTopic, std::string("simon/scan"));
  //prvNh.param<std::string>("node_control_topic", nodeControlTopic, "node_control");
  prvNh.param<double>("max_range", _maxRange, 30.0);

  double dVar      = 0;
  int iVar         = 0;
  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);

  //double truncationRadius = 0.0;
  double gridPublishInterval = 0.0;

  prvNh.param<double>("x_off_factor", _xOffFactor, 0.5);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);
  prvNh.param<double>("yaw_start_offset", _yawOffset, 0.0);
  prvNh.param<int>("cell_octave_factor", iVar, 10);
  _octaveFactor = static_cast<unsigned int>(iVar);
  prvNh.param<double>("cellsize", _cellSize, 0.025);
  prvNh.param<int>("truncation_radius", iVar, 3);
  _truncationRadius = static_cast<double>(iVar);
  prvNh.param<double>("occ_grid_time_interval", gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", _rateVar, 40.0);
  //prvNh.param<bool>("use_icpsac", _icpSac, false);
  _loopRate = new ros::Rate(_rateVar);
  _gridInterval = new ros::Duration(gridPublishInterval);
  //unsigned int uiVar = static_cast<unsigned int>(octaveFactor);
  if(_octaveFactor > 15)
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Unknown cell_octave_factor -> set to default!" << std::endl;
    _octaveFactor = 10;
  }

  _grid = new obvious::TsdGrid(_cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(_octaveFactor));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(_truncationRadius * _cellSize);

  unsigned int cellsPerSide = pow(2, _octaveFactor);
  std::cout << __PRETTY_FUNCTION__ << " creating representation with " << cellsPerSide << "x" << cellsPerSide;
  double sideLength = static_cast<double>(cellsPerSide) * _cellSize;
  std::cout << " cells, representating "<< sideLength << "x" << sideLength << "m^2" << std::endl;

  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid,_nh, &_pubMutex, _xOffFactor, _yOffFactor);
  _initialized = true;  //toDo: obsolete

  ThreadLocalize* threadLocalize = NULL;
  LaserCallBackObject* laserCallBackObject = NULL;
  ros::Subscriber* subs = NULL;
  std::string nameSpace;

  if(robotNbr == 1)  //single slam
  {
    nameSpace = "";   //empty namespace
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, _xOffFactor, _yOffFactor);
    _localizers.push_back(threadLocalize);
    laserCallBackObject = new LaserCallBackObject(*this, threadLocalize, &_nh, nameSpace + "/scan", nameSpace);  //toDO: variable scan topics
    _laserCallBacks.push_back(laserCallBackObject);
    ROS_INFO_STREAM("Single SLAM started" << std::endl);
  }
  else
  {
    for(unsigned int i = 0; i < robotNbr; i++)   //multi slam
    {
      std::stringstream sstream;
      sstream << "robot";
      sstream << i << "/namespace";
      std::string dummy = sstream.str();
      prvNh.param(dummy, nameSpace, std::string("default_ns"));
      threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, _xOffFactor, _yOffFactor);
      _localizers.push_back(threadLocalize);
      laserCallBackObject = new LaserCallBackObject(*this, threadLocalize, &_nh, nameSpace + "/scan", nameSpace);  //toDO: variable scan topics
      _laserCallBacks.push_back(laserCallBackObject);
      ROS_INFO_STREAM("started for thread for " << nameSpace << std::endl);
    }
    ROS_INFO_STREAM("Multi SLAM started!");
  }
  _pause = false;

}

SlamNode::~SlamNode()
{

  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
  delete _loopRate;
    delete _gridInterval;
    if(_initialized)
    {
      _threadGrid->terminateThread();
      while(_threadGrid->alive(THREAD_TERM_MS))
        usleep(THREAD_TERM_MS);
      delete _threadGrid;
      _threadMapping->terminateThread();
      while(_threadMapping->alive(THREAD_TERM_MS))
        usleep(THREAD_TERM_MS);
      delete _threadMapping;
    }
    delete _grid;
  //  if(_initialized)
  //  {
  //    delete _localizer;
  //    delete _sensor;
  //    delete [] _mask;
  //  }
}

void SlamNode::timedGridPub(void)
{
  static ros::Time lastMap = ros::Time::now();
  ros::Time curTime = ros::Time::now();
  if((curTime - lastMap).toSec() > _gridInterval->toSec())
  {
    _threadGrid->unblock();
    lastMap = ros::Time::now();
  }
}

//void SlamNode::start(void)
//{
//  this->run();
//}

//void SlamNode::initialize(const sensor_msgs::LaserScan& initScan)
//{
//  _mask=new bool[initScan.ranges.size()];
//  for(unsigned int i=0;i<initScan.ranges.size();i++)
//  {
//    _mask[i]=!isnan(initScan.ranges[i])&&!isinf(initScan.ranges[i])&&(fabs(initScan.ranges[i])>10e-6);
//  }
//
//  _sensor=new obvious::SensorPolar2D(initScan.ranges.size(), initScan.angle_increment, initScan.angle_min, static_cast<double>(_maxRange));
//  _sensor->setRealMeasurementData(initScan.ranges, 1.0);
//  _sensor->setRealMeasurementMask(_mask);
//
//  ros::NodeHandle prvNh("~");
//  //  double xOffFactor = 0.0;
//  //  double yOffFactor = 0.0;
//  //  double yawOffset  = 0.0;
//  double footPrintWidth = 0.0;
//  double footPrintHeight = 0.0;
//  double footPrintXoffset = 0.0;
//
//  //  prvNh.param<double>("x_off_factor", xOffFactor, 0.5);
//  //  prvNh.param<double>("y_off_factor", yOffFactor, 0.5);
//  //  prvNh.param<double>("yaw_start_offset", yawOffset, 0.0);
//  prvNh.param<double>("footprint_width" , footPrintWidth, 0.1);
//  prvNh.param<double>("footprint_height", footPrintHeight, 0.1);
//  prvNh.param<double>("footprint_x_offset", footPrintXoffset, 0.28);
//
//  const double phi        = _yawOffset;
//  const double gridWidth  = _grid->getCellsX() * _grid->getCellSize();
//  const double gridHeight = _grid->getCellsY() * _grid->getCellSize();
//  const obfloat startX    = static_cast<obfloat>(gridWidth  * _xOffFactor);
//  const obfloat startY    = static_cast<obfloat>(gridHeight * _yOffFactor);
//  double tf[9]     ={cos(phi), -sin(phi), gridWidth * _xOffFactor,
//      sin(phi),  cos(phi), gridHeight * _yOffFactor,
//      0,         0,               1};
//  obvious::Matrix Tinit(3, 3);
//  Tinit.setData(tf);
//  _sensor->transform(&Tinit);
//
//  _threadMapping=new ThreadMapping(_grid);
//
//  const obfloat t[2] = {startX + footPrintXoffset, startY};
//  if(!_grid->freeFootprint(t, footPrintWidth, footPrintHeight))
//    std::cout << __PRETTY_FUNCTION__ << " warning! Footprint could not be freed!\n";
//  _threadMapping->initPush(_sensor);
//
//  _localizer = new Localization(_grid, _threadMapping, _nh, _xOffFactor, _yOffFactor);
//
//  _threadGrid = new ThreadGrid(_grid, _nh, &_pubMutex, _xOffFactor, _yOffFactor);
//
//  _initialized = true;
//}

void SlamNode::run(void)
{
  std::cout << __PRETTY_FUNCTION__ << " waiting for first laser scan to initialize node...\n";
  while(ros::ok())
  {
    ros::spinOnce();
    if(!_pause)
    {
      if(_initialized)
        this->timedGridPub();
    }
    _loopRate->sleep();
  }

}

//void SlamNode::laserScanCallBack(const sensor_msgs::LaserScan& scan)
//{
//  if(!_initialized)
//  {
//    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initialize node...\n";
//    this->initialize(scan);
//    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
//    return;
//  }
//  for(unsigned int i=0;i<scan.ranges.size();i++)
//  {
//    _mask[i]=!isnan(scan.ranges[i]);
//  }
//  _sensor->setRealMeasurementData(scan.ranges, 1.0);
//  _sensor->setRealMeasurementMask(_mask);
//  _sensor->maskDepthDiscontinuity(obvious::deg2rad(3.0));
//  _sensor->maskZeroDepth();
//  _localizer->localize(_sensor);
//}
//
//
//bool SlamNode::pause(void)
//{
//  if(!_pause)
//  {
//    _laserSubs.shutdown();
//    _pause = true;
//    return true;
//  }
//  else
//    return false;
//}
//
//bool SlamNode::unPause(void)
//{
//  if(_pause)
//  {
//    _laserSubs = _nh.subscribe(_laserTopic, 1, &SlamNode::laserScanCallBack, this);
//    _pause = false;
//    return true;
//  }
//  return false;
//}

bool SlamNode::reset(void)
{
//  if(!this->pause())
//  {
//    std::cout << __PRETTY_FUNCTION__ << " error suspending slam!" << std::endl;
//    return false;
//  }
//  const unsigned int sleepTimeUs = 1000 * 1000 * 2.0;
//  std::cout << __PRETTY_FUNCTION__ << " wating " << sleepTimeUs << " us for push thread to empty queue... " << std::endl;
//  usleep(sleepTimeUs);
//  std::cout << __PRETTY_FUNCTION__ << " wating complete. Reset grid... " << std::endl;
//
//  if(!this->resetGrid())
//  {
//    std::cout << __PRETTY_FUNCTION__ << " Error! Grid not initialized!" << std::endl;
//    return false;
//  }
//  delete _mask;
//  _mask = NULL;
//  delete _sensor;
//  _sensor = NULL;
//  _threadGrid->terminateThread();
//  while(_threadGrid->alive(THREAD_TERM_MS))
//    usleep(THREAD_TERM_MS);
//  delete _threadGrid;
//  _threadMapping->terminateThread();
//  while(_threadMapping->alive(THREAD_TERM_MS))
//    usleep(THREAD_TERM_MS);
//  delete _threadMapping;
//  _threadGrid = NULL;
//  _threadMapping = NULL;
//  _initialized = false;
//  if(!this->unPause())
//  {
//    std::cout << __PRETTY_FUNCTION__ << " error restarting slam!" << std::endl;
//    return false;
//  }
  return true;
}

} /* namespace ohm_tsdSlam2 */
