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

static std::string _gridData;
static bool _gridDataReceived;

void gridDatacallBack(const std_msgs::String& gridData);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localize_node");
  LOGMSG_CONF("slamlog.log", obvious::Logger::file_off|obvious::Logger::screen_on, DBG_DEBUG, DBG_ERROR);

  std::string dataPath;                     //toDo: integrate sevice call to hive
  std::string tsdDataTopic;
  bool hiveSource = false;
  ros::NodeHandle prvNh("~");
  ros::NodeHandle nh;
  ros::Subscriber tsdgridDataSubs;
  prvNh.param<std::string>("data_source", dataPath, "/home/user/.ros/tsd_grid.dat");
  prvNh.param<bool>("call_hive", hiveSource, false);
  ohm_tsd_slam::SlamNode* localizeNode = NULL;
  if(hiveSource)
  {
    std::cout << __PRETTY_FUNCTION__ << " load grid from hive..." << std::endl;
    _gridDataReceived = false;
    int tries = 0;
    int ctr = 0;
    prvNh.param<std::string>("tsd_data_topic", tsdDataTopic, "map_data");
    prvNh.param<int>("map_receive_tries", tries, 30);
    tsdgridDataSubs = nh.subscribe(tsdDataTopic, 1, gridDatacallBack);
    ros::Rate r(1);

    while(ros::ok() && ctr++ < tries)
    {
      ros::spinOnce();
      if(_gridDataReceived)
        break;
      r.sleep();
    }
    if(!_gridDataReceived)
    {
      std::cout << __PRETTY_FUNCTION__ << " error! Receiving tsd_grid_data failed!" << std::endl;
      std::exit(3);
    }
    localizeNode = new ohm_tsd_slam::SlamNode(_gridData, obvious::STRING_SOURCE);
  }
  else
  {
    prvNh.param<std::string>("data_path", dataPath, "/home/user/.ros/tsd_grid.dat");
    localizeNode = new ohm_tsd_slam::SlamNode(dataPath, obvious::FILE_SOURCE);
    tsdgridDataSubs.shutdown();
  }
  localizeNode->start();
  delete localizeNode;
}


void gridDatacallBack(const std_msgs::String& gridData)
{
  _gridData = gridData.data;
  _gridDataReceived = true;
}
