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
SlamNode::SlamNode(const std::string& content, obvious::EnumTsdGridLoadSource source)
{
  ros::NodeHandle prvNh("~");
  _localizeRunning = true;
  int iVar                   = 0;
  double gridPublishInterval = 0.0;
  double loopRateVar         = 0.0;
  double truncationRadius    = 0.0;
  double cellSize            = 0.0;
  double xOffset             = 0.0;
  double yOffset             = 0.0;
  unsigned int octaveFactor  = 0;
  std::string topicLaser;
  std::string storeMapTopic;
  std::string topicStopLocalization;
  prvNh.param<int>("robot_nbr", iVar, 1);
  unsigned int robotNbr = static_cast<unsigned int>(iVar);

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
  prvNh.param<std::string>("store_map_topic", storeMapTopic, "store_map");
  prvNh.param<std::string>("topic_stop_locazation", topicStopLocalization, "stop_localization");

  _loopRate       = new ros::Rate(loopRateVar);
  _gridInterval   = new ros::Duration(gridPublishInterval);
  _storeMapServer = _nh.advertiseService(storeMapTopic, &SlamNode::storeMapServiceCallBack, this);
  _serverStopLocalization = _nh.advertiseService(topicStopLocalization, &SlamNode::callBackStopLocalization, this);

  if(content.size())
  {
    ROS_INFO_STREAM("Node in localize only mode" << std::endl);
    _grid = new obvious::TsdGrid(content, source);
    _localizeOnly = true;
    ROS_INFO_STREAM("Load map with " << _grid->getCellsX() << " x " << _grid->getCellsY() << " cells, " << _grid->getCellSize() <<
                    " cellsize" << std::endl);
  }
  else
  {
    ROS_INFO_STREAM("Slam node started" << std::endl);
    if(octaveFactor > 15)
    {
      ROS_ERROR_STREAM("Error! Unknown cell_octave_factor -> set to default!" << std::endl);
      octaveFactor = 10;
    }
    _grid        = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(octaveFactor));
    _grid->setMaxTruncation(truncationRadius * cellSize);
    _localizeOnly = false;
    unsigned int cellsPerSide = std::pow(2, octaveFactor);
    double sideLength = static_cast<double>(cellsPerSide) * cellSize;
    ROS_INFO_STREAM("Creating representation with " << cellsPerSide << "x" << "cellsPerSide cells, representating " <<
        sideLength << "x" << sideLength << "m^2" << std::endl);
  }
  if(!_localizeOnly)
    _threadMapping = new ThreadMapping(_grid);
  else
    _threadMapping = NULL;
  _threadGrid    = new ThreadGrid(_grid, &_nh, xOffset, yOffset);

  ThreadLocalize* threadLocalize = NULL;
  ros::Subscriber subs;
  std::string nameSpace;

  //instanciate localization threads
  if(robotNbr == 1)  //single slam
  {
    nameSpace = "";   //empty namespace
    threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, xOffset, yOffset);
    subs = _nh.subscribe(topicLaser, 1, &ThreadLocalize::laserCallBack, threadLocalize);
    _subsLaser.push_back(subs);
    _localizers.push_back(threadLocalize);
    _topicsLaser.push_back(topicLaser);
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
      threadLocalize = new ThreadLocalize(_grid,_threadMapping, &_nh, nameSpace, xOffset, yOffset);
      subs = _nh.subscribe(nameSpace + "/" + topicLaser, 1, &ThreadLocalize::laserCallBack, threadLocalize);
      _subsLaser.push_back(subs);
      _localizers.push_back(threadLocalize);
      _topicsLaser.push_back(nameSpace + "/" + topicLaser);
      ROS_INFO_STREAM("started for thread for " << nameSpace << std::endl);
    }
    ROS_INFO_STREAM("Multi SLAM started!");
  }
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
  //ROS_INFO_STREAM("Waiting for first laser scan to initialize node...\n");
  while(ros::ok())
  {
    ros::spinOnce();
    this->timedGridPub();
    _loopRate->sleep();
  }
}

bool SlamNode::storeMapServiceCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return _threadGrid->requestStoreTsdGrid();
}

bool SlamNode::callBackStopLocalization(ohm_tsd_slam::switch_local_on_off::Request& req, ohm_tsd_slam::switch_local_on_off::Response& res)
{
  if(req.command == req.ON && _localizeRunning)
    return false;
  else if((req.command == req.OFF && !_localizeRunning))
    return false;

  if(req.command == req.OFF)
  {
    for(std::vector<ros::Subscriber>::iterator iter = _subsLaser.begin(); iter < _subsLaser.end(); iter++)
      iter->shutdown();
    _localizeRunning = false;
  }
  else if(req.command == req.ON)
  {
    std::vector<std::string>::iterator topicIter = _topicsLaser.begin();
    std::vector<ThreadLocalize*>::iterator iterLocalizer = _localizers.begin();
    for(std::vector<ros::Subscriber>::iterator iter = _subsLaser.begin(); iter < _subsLaser.end(); iter++, topicIter++, iterLocalizer++)
    {
      *iter = _nh.subscribe(*topicIter, 1, &ThreadLocalize::laserCallBack, *iterLocalizer);
    }
    _localizeRunning = true;
  }
  return true;
}

} /* namespace ohm_tsd_slam */
