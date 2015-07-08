/*
 * LaserCallBackObject.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: phil
 */

#include "LaserCallBackObject.h"
#include "SlamNode.h"

namespace ohm_tsd_slam
{

LaserCallBackObject::LaserCallBackObject(SlamNode& parentNode, ThreadLocalize* localizeThread,
                                         ros::NodeHandle* const nh, const std::string& laserTopic,
                                         const std::string& nameSpace):
    _parentNode(parentNode),
    _localizeThread(localizeThread),
    _laserTopic(laserTopic),
    _nh(nh),
    _pause(false)
{
  ros::NodeHandle prvNh("~");
  std::string nodeControlTopic;
  std::string nodeControlParamServer = nameSpace + "/node_control"; //toDO: variable topic
  prvNh.param<std::string>(nodeControlParamServer, nodeControlTopic, nameSpace + "/node_control");
  nodeControlTopic = nameSpace + "/" + nodeControlTopic;
  if(!_pause)
    _laserSubs = _nh->subscribe(_laserTopic, 1, &LaserCallBackObject::laserCallBack, this);
}

LaserCallBackObject::~LaserCallBackObject()
{
 this->pause();
}

void LaserCallBackObject::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(_localizeThread->setData(scan))
    _localizeThread->unblock();
}

bool LaserCallBackObject::pause(void)
{
  if(!_pause)
    {
      _laserSubs.shutdown();
      _pause = true;
      return true;
    }
    else
      return false;
}

bool LaserCallBackObject::unPause(void)
{
  if(_pause)
    {
      _laserSubs = _nh->subscribe(_laserTopic, 1, &LaserCallBackObject::laserCallBack, this);
      _pause = false;
      return true;
    }
    return false;
}

bool LaserCallBackObject::resetMapping(void)
{
  return _parentNode.reset();
}

} /* namespace ohm_tsd_slam */
