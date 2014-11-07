/*
 * MultiSlamNode.h
 *
 *  Created on: Oct 30, 2014
 *      Author: phil
 */

#ifndef SRC_MULTI_MULTISLAMNODE_H_
#define SRC_MULTI_MULTISLAMNODE_H_

#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include <ros/ros.h>

#include <boost/thread.hpp>

namespace ohm_tsd_slam
{

class ThreadLocalize;
class ThreadMapping;
class ThreadGrid;

class MultiSlamNode
{
public:
  MultiSlamNode();
  virtual ~MultiSlamNode();
  void start(void);
  double xOffFactor(void)const{return _xOffFactor;}
  double yOffFactor(void)const{return _yOffFactor;}
  void initPush(obvious::SensorPolar2D* sensor);
private:
  void run(void);
  void init(void);
  std::vector<ThreadLocalize*> _localizers;
  ThreadGrid* _threadGrid;
  ThreadMapping* _threadMapping;
  obvious::TsdGrid*            _grid;

  double _xOffFactor;
  double _yOffFactor;
  double _gridPublishInterval;

  ros::NodeHandle _nh;
  ros::Rate* _loopRate;
  boost::mutex _pubMutex;


};

}

#endif /* SRC_MULTI_MULTISLAMNODE_H_ */
