/*
 * SlamBase.h
 *
 *  Created on: Nov 17, 2014
 *      Author: phil
 */

#ifndef SRC_SLAMBASE_H_
#define SRC_SLAMBASE_H_

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "obvision/reconstruct/grid/TsdGrid.h"

#define INIT_PSHS 1
#define THREAD_TERM_MS 1   //time in ms waiting for thread to terminate

namespace ohm_tsd_slam
{

class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;

class SlamBase
{
public:
  SlamBase();
  virtual ~SlamBase();
  void start(void);
protected:
  virtual void run(void) = 0;
  void timedGridPub(void);
  ros::NodeHandle _nh;
  obvious::TsdGrid* _grid;
  ThreadMapping* _threadMapping;
  ThreadGrid* _threadGrid;
  boost::mutex _pubMutex;
  double _xOffFactor;
  double _yOffFactor;
  double _yawOffset;
  double _rateVar;
  ros::Rate* _loopRate;
  bool _initialized;
  ros::Duration* _gridInterval;
  bool _icpSac;
};

} /* namespace ohm_tsd_slam */

#endif /* SRC_SLAMBASE_H_ */
