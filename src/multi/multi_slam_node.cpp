/*
 * multi_slam_node.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: phil
 */

#include "MultiSlamNode.h"
#include <ros/ros.h>

#include "obcore/base/Logger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_slam_node");

  LOGMSG_CONF("slamlog.log", obvious::Logger::file_off|obvious::Logger::screen_off, DBG_DEBUG, DBG_ERROR);

  ohm_tsd_slam::MultiSlamNode multiSlamNode;
  multiSlamNode.start();

}
