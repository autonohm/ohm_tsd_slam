/*
 * slam_node.cpp
 *
 *  Created on: Apr 28, 2014
 *      Author: phil
 */

#include "SlamNode.h"

#include <ros/ros.h>

#include "obcore/base/Logger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_node");

  LOGMSG_CONF("slamlog.log", obvious::Logger::file_off|obvious::Logger::screen_off, DBG_DEBUG, DBG_ERROR);
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
      {
         ros::console::notifyLoggerLevelsChanged();
      }

  ohm_tsd_slam::SlamNode slamNode;
  slamNode.start();
}

