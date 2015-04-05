/*
 * LaserCallBackObject.h
 *
 *  Created on: Jan 15, 2015
 *      Author: phil
 */

#ifndef SRC_MULTI_LASERCALLBACKOBJECT_H_
#define SRC_MULTI_LASERCALLBACKOBJECT_H_

#include <string>
#include "ThreadLocalize.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "ohm_srvs/NodeControl.h"

namespace ohm_tsd_slam
{

class MultiSlamNode;

class LaserCallBackObject
{
public:
  LaserCallBackObject(MultiSlamNode& parentNode, ThreadLocalize* localizeThread,
                      ros::NodeHandle* const nh, const std::string& laserTopic, const std::string& nameSpace);
  virtual ~LaserCallBackObject();
  void laserCallBack(const sensor_msgs::LaserScan& scan);
  bool pause(void);
  bool unPause(void);
  bool resetMapping(void);
private:
  bool nodeControlCallBack(ohm_srvs::NodeControl::Request& req, ohm_srvs::NodeControl::Response& res);
  MultiSlamNode& _parentNode;
  ThreadLocalize* _localizeThread;
  std::string _laserTopic;
  ros::NodeHandle* const _nh;
  ros::Subscriber _laserSubs;
  ros::ServiceServer _nodeControl;

  bool _pause;
};

} /* namespace ohm_tsd_slam */

#endif /* SRC_MULTI_LASERCALLBACKOBJECT_H_ */
