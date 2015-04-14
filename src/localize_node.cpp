/*
 * localize_node.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: phil
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "SlamNode.h"

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obcore/base/Logger.h"

#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localize_node");
  LOGMSG_CONF("slamlog.log", obvious::Logger::file_off|obvious::Logger::screen_on, DBG_DEBUG, DBG_ERROR);

  std::string dataPath;

  ros::NodeHandle prvNh("~");
  ohm_tsd_slam::SlamNode* localizeNode = NULL;

    prvNh.param<std::string>("data_path", dataPath, "/home/user/.ros/tsd_grid.dat");
    localizeNode = new ohm_tsd_slam::SlamNode(dataPath, obvious::FILE);

  localizeNode->start();
  delete localizeNode;
}
