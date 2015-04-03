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
  std::string strVar;
  int iVar                = 0;

  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid,_nh, &_pubMutex, _xOffFactor, _yOffFactor);
  _initialized   = true;

  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);
  ThreadLocalize* threadLocalize = NULL;
  LaserCallBackObject* laserCallBackObject = NULL;
  ros::Subscriber* subs = NULL;
  std::string nameSpace;

  for(unsigned int i = 0; i < robotNbr; i++)
  {
    std::stringstream sstream;
    sstream << "robot";
    sstream << i << "/namespace";
    std::string dummy = sstream.str();
    prvNh.param(dummy, nameSpace, std::string("default_ns"));
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, _xOffFactor, _yOffFactor);
    _localizers.push_back(threadLocalize);
    laserCallBackObject = new LaserCallBackObject(threadLocalize);
    subs = new ros::Subscriber;
    *subs =  _nh.subscribe(nameSpace + "/scan", 1, &LaserCallBackObject::laserCallBack, laserCallBackObject);
    _laserCallBacks.push_back(laserCallBackObject);
    _subs.push_back(subs);
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

} //end namespace
