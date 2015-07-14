/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#include "SlamNode.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"
#include "ThreadLocalize.h"

#include "obcore/math/mathbase.h"

namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  ros::NodeHandle prvNh("~");

  prvNh.param<double>("max_range", _maxRange, 30.0);

  double dVar      = 0;
  int iVar         = 0;
  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);

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
  prvNh.param<std::string>("laser_topic", _laserTopic, "scan");

  _loopRate = new ros::Rate(_rateVar);
  _gridInterval = new ros::Duration(gridPublishInterval);

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
  _threadGrid    = new ThreadGrid(_grid, &_nh, _xOffFactor, _yOffFactor);
  _initialized = true;  //toDo: obsolete

  ThreadLocalize* threadLocalize = NULL;
  ros::Subscriber subs;
  std::string nameSpace;

  if(robotNbr == 1)  //single slam
  {
    nameSpace = "";   //empty namespace
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, _xOffFactor, _yOffFactor);
    subs = _nh.subscribe(_laserTopic, 1, &ThreadLocalize::laserCallBack, threadLocalize);
    _subsLaser.push_back(subs);
    _localizers.push_back(threadLocalize);
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
      subs = _nh.subscribe(nameSpace + "/" +_laserTopic, 1, &ThreadLocalize::laserCallBack, threadLocalize);
      _subsLaser.push_back(subs);
      _localizers.push_back(threadLocalize);
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

} /* namespace ohm_tsdSlam2 */
