/*
 * multi_slam_node.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: phil
 */

#include "MultiSlamNode.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_slam_node");

  ohm_tsd_slam::MultiSlamNode multiSlamNode;

}
