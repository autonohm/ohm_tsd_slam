/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 * Refactored by: Christian Wendt
 */

#include "SlamNode.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"

#include "obcore/math/mathbase.h"
#include <functional>
#include <memory>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

namespace ohm_tsd_slam
{
SlamNode::SlamNode()
  : Node("slam_node")
{ }

void SlamNode::initialize()
{
  int iVar                   = 0;
  double gridPublishInterval = 0.0;
  double truncationRadius    = 0.0;
  double cellSize            = 0.0;
  unsigned int octaveFactor  = 0;
  double xOffset = 0.0;
  double yOffset = 0.0;
  const std::string topicLaser = std::string(get_name()) + "/laser";
  const std::string topicServiceStartStop = "start_stop_slam";
  std::string nameSpace = get_name();

  declare_parameter<int>("robot_nbr", 1);
  declare_parameter<double>("x_off_factor", 0.5);
  declare_parameter<double>("y_off_factor", 0.5);
  declare_parameter<double>("x_offset", 0.0);
  declare_parameter<double>("y_offset", 0.0);

  iVar = get_parameter("robot_nbr").as_int();
  unsigned int robotNbr = static_cast<unsigned int>(iVar);
  _xOffFactor = get_parameter("x_off_factor").as_double();
  _yOffFactor = get_parameter("y_off_factor").as_double();
  xOffset = get_parameter("x_offset").as_double();
  yOffset = get_parameter("y_offset").as_double();

  declare_parameter<int>("map_size", 10);
  declare_parameter<double>("cellsize", 0.025);
  declare_parameter<int>("truncation_radius", 3);
  declare_parameter<double>("occ_grid_time_interval", 2.0);
  // declare_parameter<std::string>("topic_service_start_stop", "start_stop_slam");
  declare_parameter<std::string>("tf_map_frame", "map");

  iVar = get_parameter("map_size").as_int();
  octaveFactor = static_cast<unsigned int>(iVar);
  cellSize = get_parameter("cellsize").as_double();
  iVar = get_parameter("truncation_radius").as_int();
  truncationRadius = static_cast<double>(iVar);
  gridPublishInterval = get_parameter("occ_grid_time_interval").as_double();
  // topicLaser = get_parameter("laser_topic").as_string(); // TODO: code smell! Should be configured via topic remapping.
  // topicServiceStartStop = get_parameter("topic_service_start_stop").as_string();

  _gridInterval = std::make_unique<rclcpp::Duration>(rclcpp::Duration::from_seconds(gridPublishInterval));

  if(octaveFactor > 15)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Error! Unknown map size -> set to default!");
    octaveFactor = 10;
  }
  //instanciate representation
  _grid = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(octaveFactor));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellSize);
  // unsigned int cellsPerSide = pow(2, octaveFactor);
  // double sideLength = static_cast<double>(cellsPerSide) * cellSize;
  RCLCPP_INFO_STREAM(get_logger(), "Creating representation with " << _grid->getCellsX() << "x" << _grid->getCellsY() << "cells, representating " <<
                     _grid->getCellsX() * _grid->getCellSize() << "x" << _grid->getCellsY() * _grid->getCellSize() << "m^2");

  //instanciate mapping threads
  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid, shared_from_this(), xOffset, yOffset);

  ThreadLocalize* threadLocalize = nullptr;
  TaggedSubscriber subs;

  //instanciate localization threads
  if(robotNbr == 1)  //single slam
  {
    threadLocalize = new ThreadLocalize(_grid, _threadMapping, shared_from_this(), "", xOffset, yOffset);
    subs = TaggedSubscriber(topicLaser, *threadLocalize, shared_from_this());
    subs.switchOn();
    _subsLaser.push_back(subs);
    _localizers.push_back(threadLocalize);
    RCLCPP_INFO_STREAM(get_logger(), "Single SLAM started");
  }
  else
  {
    // TODO: not tested after change to ROS2!
    for(unsigned int i = 0; i < robotNbr; i++)   //multi slam
    {
      // get parameter robot_i.name and use it as namespace for each robot
      std::stringstream sstream;
      sstream << "robot_" << i;
      const std::string name_parameter_robot = sstream.str();
      const std::string parameter_name =  name_parameter_robot + "/name";
      declare_parameter<std::string>(parameter_name, name_parameter_robot);
      const std::string robot_name = get_parameter(parameter_name).as_string();
      nameSpace = std::string(get_name()) + "/" + robot_name;

      threadLocalize = new ThreadLocalize(_grid, _threadMapping, shared_from_this(), robot_name, xOffset, yOffset);
      subs = TaggedSubscriber(nameSpace + "/" + topicLaser, *threadLocalize, shared_from_this());
      _subsLaser.push_back(subs);
      _localizers.push_back(threadLocalize);
      RCLCPP_INFO_STREAM(get_logger(), "started for thread for " << nameSpace);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Multi SLAM started!");
  }

  _serviceStartStopSLAM = create_service<ohm_tsd_slam::srv::StartStopSLAM>(
    topicServiceStartStop,
    std::bind(&SlamNode::callBackServiceStartStopSLAM, this, std::placeholders::_1, std::placeholders::_2)
  );
  _timer = rclcpp::create_timer(this, get_clock(), *_gridInterval, std::bind(&SlamNode::timedGridPub, this));
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

void SlamNode::timedGridPub()
{
  _threadGrid->unblock();
}

bool SlamNode::callBackServiceStartStopSLAM(const std::shared_ptr<ohm_tsd_slam::srv::StartStopSLAM::Request> req,
                                            std::shared_ptr<ohm_tsd_slam::srv::StartStopSLAM::Response>)
{
  TaggedSubscriber* subsCur = NULL;
  for(auto iter = _subsLaser.begin(); iter < _subsLaser.end(); iter++)
  {
    if(iter->topic(req->topic))
      subsCur = &*iter;
  }
  if(!subsCur)
  {
    RCLCPP_ERROR_STREAM(get_logger(), __PRETTY_FUNCTION__ << " Error! Topic " << req->topic << " invalid!");
    return false;
  }
  if(req->start_stop == req->START)
  {
    RCLCPP_INFO_STREAM(get_logger(), __PRETTY_FUNCTION__ << " Started SLAM for topic " << req->topic);
    subsCur->switchOn();
  }
  else if(req->start_stop == req->STOP)
  {
    RCLCPP_INFO_STREAM(get_logger(), __PRETTY_FUNCTION__ << " Stopped SLAM for topic " << req->topic);
    subsCur->switchOff();
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(), __PRETTY_FUNCTION__ << " Error. Unknown request for service");
    return false;
  }
  return true;
}

} /* namespace ohm_tsd_slam */
