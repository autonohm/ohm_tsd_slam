/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#include "SlamNode.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"




#include "obcore/math/mathbase.h"


namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  ros::NodeHandle prvNh("~");
  int iVar                   = 0;
  double gridPublishInterval = 0.0;
  double loopRateVar         = 0.0;
  double truncationRadius    = 0.0;
  double cellSize            = 0.0;
  unsigned int octaveFactor  = 0;
  double xOffset = 0.0;
  double yOffset = 0.0;
  std::string topicLaser;
  std::string topicServiceStartStop;
  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);
  prvNh.param<double>("x_off_factor", _xOffFactor, 0.5);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);
  prvNh.param<double>("x_offset", xOffset, 0.0);
  prvNh.param<double>("y_offset", yOffset, 0.0);


  prvNh.param<int>("map_size", iVar, 10);
  octaveFactor = static_cast<unsigned int>(iVar);
  prvNh.param<double>("cellsize", cellSize, 0.025);
  prvNh.param<int>("truncation_radius", iVar, 3);
  truncationRadius = static_cast<double>(iVar);
  prvNh.param<double>("occ_grid_time_interval", gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", loopRateVar, 40.0);
  prvNh.param<std::string>("laser_topic", topicLaser, "scan");
  prvNh.param<std::string>("topic_service_start_stop", topicServiceStartStop, "start_stop_slam");

  _loopRate = new ros::Rate(loopRateVar);
  _gridInterval = new ros::Duration(gridPublishInterval);

  if(octaveFactor > 15)
  {
    ROS_ERROR_STREAM("Error! Unknown map size -> set to default!" << std::endl);
    octaveFactor = 10;
  }
  //instanciate representation
  _grid = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(octaveFactor));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellSize);
  unsigned int cellsPerSide = pow(2, octaveFactor);
  double sideLength = static_cast<double>(cellsPerSide) * cellSize;
  ROS_INFO_STREAM("Creating representation with " << cellsPerSide << "x" << cellsPerSide << "cells, representating " <<
                  sideLength << "x" << sideLength << "m^2" << std::endl);
  //instanciate mapping threads
  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid, &_nh, xOffset, yOffset);

  ThreadLocalize* threadLocalize = NULL;
  TaggedSubscriber subs;
  std::string nameSpace = "";

  //instanciate localization threads
  if(robotNbr == 1)  //single slam
  {
    threadLocalize = new ThreadLocalize(_grid, _threadMapping, &_nh, xOffset, yOffset, nameSpace);
    subs = TaggedSubscriber(topicLaser, *threadLocalize, _nh);
    subs.switchOn();
    //subs = _nh.subscribe(topicLaser, 1, &ThreadLocalize::laserCallBack, threadLocalize);
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
      threadLocalize = new ThreadLocalize(_grid, _threadMapping, &_nh, xOffset, yOffset, nameSpace);
//      subs = _nh.subscribe(nameSpace + "/" + topicLaser, 1, &ThreadLocalize::laserCallBack, threadLocalize);
      subs = TaggedSubscriber(nameSpace + "/" + topicLaser, *threadLocalize, _nh);
      _subsLaser.push_back(subs);
      _localizers.push_back(threadLocalize);
      ROS_INFO_STREAM("started for thread for " << nameSpace << std::endl);
    }
    ROS_INFO_STREAM("Multi SLAM started!");
  }
  _serviceStartStopSLAM = _nh.advertiseService(topicServiceStartStop, &SlamNode::callBackServiceStartStopSLAM, this);
}

SlamNode::~SlamNode()
{
  //stop all localization threads
  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
  delete _loopRate;
  delete _gridInterval;
  //stop mapping threads
  _threadGrid->terminateThread();
  while(_threadGrid->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadGrid;
  _threadMapping->terminateThread();
  while(_threadMapping->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadMapping;
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
  ROS_INFO_STREAM("Waiting for first laser scan to initialize node...\n");
  while(ros::ok())
  {
    ros::spinOnce();
    this->timedGridPub();
    _loopRate->sleep();
  }
}

bool SlamNode::callBackServiceStartStopSLAM(ohm_tsd_slam::StartStopSLAM::Request& req, ohm_tsd_slam::StartStopSLAM::Response& res)
{
  TaggedSubscriber* subsCur = NULL;
  for(auto iter = _subsLaser.begin(); iter < _subsLaser.end(); iter++)
  {
    if(iter->topic(req.topic))
      subsCur = &*iter;
  }
  if(!subsCur)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error! Topic " << req.topic << " invalid!");
    return false;
  }
  if(req.startStop == req.START)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Started SLAM for topic " << req.topic);
    subsCur->switchOn();
  }
  else if(req.startStop == req.STOP)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Stopped SLAM for topic " << req.topic);
    subsCur->switchOff();
  }
  else
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error. Unknown request for service");
    return false;
  }
  return true;
}

} /* namespace ohm_tsd_slam */
