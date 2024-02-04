/*
 * slam_node.cpp
 *
 *  Created on: Apr 28, 2014
 *      Author: phil
 */

#include "SlamNode.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  LOGMSG_CONF("slamlog.log", obvious::Logger::file_off|obvious::Logger::screen_off, DBG_DEBUG, DBG_ERROR);

  auto node = std::make_shared<ohm_tsd_slam::SlamNode>();
  node->initialize();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}

