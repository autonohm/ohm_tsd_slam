/*
 * MultiSlamNode.cpp
 *
 *  Created on: Oct 30, 2014
 *      Author: phil
 */

#include "MultiSlamNode.h"
#include "ThreadGrid.h"
#include "ThreadMapping.h"
#include "ThreadLocalize.h"

#include <ros/ros.h>

#include <string>
#include <strstream>
#include <iostream>

namespace ohm_tsd_slam
{

MultiSlamNode::MultiSlamNode()
{
  ros::NodeHandle prvNh("~");
  std::string strVar;
  int octaveFactor = 0;
  double cellsize = 0.0;
  double dVar      = 0;
  int iVar         = 0;
  double truncationRadius = 0.0;
  double rateVar = 0.0;
  //    prvNh.param("laser_topic", strVar, std::string("simon/scan"));
  //    _laserSubs=_nh.subscribe(strVar, 1, &SlamNode::laserScanCallBack, this);


  prvNh.param<double>("x_off_factor", _xOffFactor, 0.2);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);
  //prvNh.param<double>("yaw_start_offset", _yawOffset, 0.0);
  prvNh.param<int>("cell_octave_factor", octaveFactor, 10);
  prvNh.param<double>("cellsize", cellsize, 0.025);
  prvNh.param<int>("truncation_radius", iVar, 3);
  truncationRadius = static_cast<double>(iVar);
  //    prvNh.param<bool>("range_filter", _rangeFilter, false);
  //prvNh.param<double>("min_range", dVar, 0.01);
  //_minRange = static_cast<float>(dVar);
  //    prvNh.param<double>("max_range", dVar, 30.0);
  //    _maxRange = static_cast<float>(dVar);
  prvNh.param<double>("occ_grid_time_interval", _gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", rateVar, 40.0);

  _loopRate = new ros::Rate(rateVar);

  unsigned int uiVar = static_cast<unsigned int>(octaveFactor);
  if(uiVar > 15)
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Unknown cell_octave_factor -> set to default!" << std::endl;
    uiVar = 10;
  }


  _grid=new obvious::TsdGrid(cellsize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(uiVar));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellsize);

  unsigned int cellsPerSide = pow(2, uiVar);
  std::cout << __PRETTY_FUNCTION__ << " creating representation with " << cellsPerSide << "x" << cellsPerSide;
  double sideLength = static_cast<double>(cellsPerSide) * cellsize;
  std::cout << " cells, representating "<< sideLength << "x" << sideLength << "m^2" << std::endl;

  //    _sensor=NULL;
  //    _mask=NULL;
  //    _localizer=NULL;

  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid,_nh, &_pubMutex, *this);

  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);
  ThreadLocalize* threadLocalize = NULL;
  std::string nameSpace;

  for(unsigned int i = 0; i < robotNbr; i++)
  {
    std::stringstream sstream;
    sstream << "robot";
    sstream << i << "_namespace";
    prvNh.param(sstream.str(), nameSpace, std::string("default_ns"));
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_pubMutex, *this, nameSpace);
    _localizers.push_back(threadLocalize);
    std::cout << __PRETTY_FUNCTION__ << " started thread for " << nameSpace << std::endl;
  }
  this->run();
}

MultiSlamNode::~MultiSlamNode()
{
  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    delete *iter;
  }
  _threadGrid->terminateThread();
  _threadMapping->terminateThread();
}

void MultiSlamNode::start(void)
{

}

void MultiSlamNode::run(void)
{
  ros::Time lastMap=ros::Time::now();
  ros::Duration durLastMap=ros::Duration(_gridPublishInterval);
  std::cout << __PRETTY_FUNCTION__ << " waiting for first laser scan to initialize node...\n";
  while(ros::ok())
  {
    ros::Time curTime=ros::Time::now();
    if((curTime-lastMap).toSec()>durLastMap.toSec())
    {
      _threadGrid->unblock();
      lastMap=ros::Time::now();
    }
    _loopRate->sleep();
  }
}


} //end namespace
