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
#include "LaserCallBackObject.h"

#include <ros/ros.h>

#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>

#define INIT_PSHS 1
#define THREAD_TERM_MS 1   //time in ms waiting for thread to terminate

namespace ohm_tsd_slam
{

MultiSlamNode::MultiSlamNode()
{
  ros::NodeHandle prvNh("~");
  std::string generalLaserTopic;
  int iVar                = 0;

  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid,_nh, &_pubMutex, _xOffFactor, _yOffFactor);
  _initialized   = true;

  prvNh.param<int>("robot_nbr", iVar, 1);
  _robotNbr = static_cast<unsigned int>(iVar);
  ThreadLocalize* threadLocalize = NULL;
  LaserCallBackObject* laserCallBackObject = NULL;
  ros::Subscriber* subs = NULL;
  std::string nameSpace;

  for(unsigned int i = 0; i < _robotNbr; i++)
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
    std::cout << __PRETTY_FUNCTION__ << " started for " << nameSpace << std::endl;
  }
}

MultiSlamNode::~MultiSlamNode()
{
  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
}

void MultiSlamNode::run(void)
{
  while(ros::ok())
  {
    ros::spinOnce();
    this->timedGridPub();
    _loopRate->sleep();
  }
}

bool MultiSlamNode::reset(void)
{
  ros::NodeHandle prvNh("~");
  bool fail = false;
  for(std::vector<LaserCallBackObject*>::iterator iter = _laserCallBacks.begin(); iter < _laserCallBacks.end(); iter++)
  {
    if(!(*iter)->pause())
    {
      fail = true;
      break;
    }
  }
  if(fail)
  {
    std::cout << __PRETTY_FUNCTION__ << " Error stopping callback(s)" << std::endl;
    return false;
  }
  const unsigned int sleepTimeUs = 1000 * 1000 * 2.0;
  std::cout << __PRETTY_FUNCTION__ << " wating " << sleepTimeUs << " us for push thread to empty queue... " << std::endl;
  usleep(sleepTimeUs);
  std::cout << __PRETTY_FUNCTION__ << " wating complete. Reset grid... " << std::endl;
  if(!this->resetGrid())
  {
    std::cout << __PRETTY_FUNCTION__ << " Error! Grid not initialized!" << std::endl;
    return false;
  }

  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
  _localizers.clear();
  for(std::vector<LaserCallBackObject*>::iterator iter = _laserCallBacks.begin(); iter < _laserCallBacks.end(); iter++)
  {
    delete *iter;
  }
  _laserCallBacks.clear();
  _threadGrid->terminateThread();
  while(_threadGrid->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadGrid;
  _threadMapping->terminateThread();
  while(_threadMapping->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadMapping;
  _threadGrid = NULL;
  _threadMapping = NULL;

  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid,_nh, &_pubMutex, _xOffFactor, _yOffFactor);
  _initialized   = true;

  ThreadLocalize* threadLocalize = NULL;
  LaserCallBackObject* laserCallBackObject = NULL;
  ros::Subscriber* subs = NULL;
  std::string nameSpace;

  for(unsigned int i = 0; i < _robotNbr; i++)
  {
    std::stringstream sstream;
    sstream << "robot";
    sstream << i << "/namespace";
    std::string dummy = sstream.str();
    prvNh.param(dummy, nameSpace, std::string("default_ns"));
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, _xOffFactor, _yOffFactor);
    _localizers.push_back(threadLocalize);
    laserCallBackObject = new LaserCallBackObject(*this, threadLocalize, &_nh, nameSpace + "/scan", nameSpace);
    _laserCallBacks.push_back(laserCallBackObject);
    std::cout << __PRETTY_FUNCTION__ << " started for " << nameSpace << std::endl;
  }
  return true;
}

} //end namespace
