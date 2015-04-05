/*
 * LaserCallBackObject.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: phil
 */

#include "LaserCallBackObject.h"
#include "MultiSlamNode.h"

namespace ohm_tsd_slam
{

LaserCallBackObject::LaserCallBackObject(MultiSlamNode& parentNode, ThreadLocalize* localizeThread,
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
  _nodeControl = _nh->advertiseService(nodeControlTopic, &LaserCallBackObject::nodeControlCallBack, this);
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

bool LaserCallBackObject::nodeControlCallBack(ohm_srvs::NodeControl::Request& req, ohm_srvs::NodeControl::Response& res)
{
  bool noError = true;
    switch(req.action)
    {
    case ohm_srvs::NodeControl::Request::PAUSE:
      if(!this->pause())
      {
        std::cout << __PRETTY_FUNCTION__ << " Error suspending callback "<< _laserTopic << std::endl;
        noError = false;
      }
      break;
    case ohm_srvs::NodeControl::Request::START:
      if(!this->unPause())
      {
        std::cout << __PRETTY_FUNCTION__ << " Error starting / restarting "<< _laserTopic << std::endl;
        noError = false;
      }
      break;
    case ohm_srvs::NodeControl::Request::STOP:
      if(!this->pause())
      {
        std::cout << __PRETTY_FUNCTION__ << " Error suspending " << _laserTopic << std::endl;
        noError = false;
      }
      break;
    case ohm_srvs::NodeControl::Request::RESTART:
      if(!this->resetMapping())
      {
        std::cout << __PRETTY_FUNCTION__ << " Error resetting SLAM called by " << _laserTopic << std::endl;
        noError = false;
      }
      break;
    default:
      std::cout << __PRETTY_FUNCTION__ << " error! Unkown command" << req.action << std::endl;
      noError = false;
      break;
    }
    res.accepted = noError;
    return noError;
}

} /* namespace ohm_tsd_slam */
