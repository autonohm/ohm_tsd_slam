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

  try
  {
    ohm_tsd_slam::SlamNode slamNode;
    slamNode.start();
  }
  catch(char const*& e)
  {
    std::cout << e << '\n';
    exit(1);
  }
  
  
  
}

