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
  int octaveFactor        = 0;
  double cellsize         = 0.0;
  double dVar             = 0;
  int iVar                = 0;
  double truncationRadius = 0.0;
  double rateVar          = 0.0;

  prvNh.param<double>("x_off_factor", _xOffFactor, 0.5);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);

  prvNh.param<int>("cell_octave_factor", octaveFactor, 10);
  prvNh.param<double>("cellsize", cellsize, 0.025);
  prvNh.param<int>("truncation_radius", iVar, 3);
  truncationRadius = static_cast<double>(iVar);
  prvNh.param<double>("occ_grid_time_interval", _gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", rateVar, 40.0);

  _loopRate = new ros::Rate(rateVar);

  unsigned int uiVar = static_cast<unsigned int>(octaveFactor);
  if(uiVar > 15)
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Unknown cell_octave_factor -> set to default!" << std::endl;
    uiVar = 10;
  }

  _grid=new obvious::TsdGrid(cellsize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(uiVar));
  _grid->setMaxTruncation(truncationRadius * cellsize);

  unsigned int cellsPerSide = pow(2, uiVar);
  std::cout << __PRETTY_FUNCTION__ << " creating representation with " << cellsPerSide << "x" << cellsPerSide;
  double sideLength = static_cast<double>(cellsPerSide) * cellsize;
  std::cout << " cells, representating "<< sideLength << "x" << sideLength << "m^2" << std::endl;

  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid,_nh, &_pubMutex, _xOffFactor, _yOffFactor);

  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);
  ThreadLocalize* threadLocalize = NULL;
  std::string nameSpace;

  for(unsigned int i = 0; i < robotNbr; i++)
  {
    std::stringstream sstream;
    sstream << "robot";
    sstream << i << "/namespace";
    std::string dummy = sstream.str();
    prvNh.param(dummy, nameSpace, std::string("default_ns"));
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_pubMutex, nameSpace, _xOffFactor, _yOffFactor);
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
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
  _threadGrid->terminateThread();
  while(_threadGrid->alive(THREAD_TERM_MS))
     usleep(THREAD_TERM_MS);
  delete _threadGrid;
  _threadMapping->terminateThread();
  while(_threadMapping->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadMapping;
}

void MultiSlamNode::start(void)
{
  this->run();
}

void MultiSlamNode::run(void)
{
  ros::Time lastMap        = ros::Time::now();
  ros::Duration durLastMap = ros::Duration(_gridPublishInterval);
  //std::cout << __PRETTY_FUNCTION__ << " waiting for first laser scan to initialize node...\n";
  while(ros::ok())
  {
    ros::Time curTime = ros::Time::now();
    if((curTime-lastMap).toSec()>durLastMap.toSec())
    {
      _threadGrid->unblock();
      lastMap=ros::Time::now();
    }
    _loopRate->sleep();
  }
}

} //end namespace
