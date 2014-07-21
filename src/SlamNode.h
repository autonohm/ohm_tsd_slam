/*
 * SlamNode.h
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#ifndef SLAMNODE_H_
#define SLAMNODE_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include <boost/thread.hpp>

#define CELLSIZE 0.025   //toDo: Launch file parameters
#define TRUNCATION_RADIUS 3.0
#define MAX_RANGE 30.0
#define MIN_RANGE 0.01
#define THETA_INIT 0.0  //used in degrees
#define MAP_T 2.0      //time between map generations
#define INIT_PSHS 50//number of initial puhses into the grid
#define LAS_OFFS_X -0.19 //offset of the laser scanner to the base footprint

namespace ohm_tsd_slam
{
class Localization;
class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;
class SlamNode
{
public:
  SlamNode();

  virtual ~SlamNode();

  void start(void);

  double xOffFactor(void)const;

  double yOffFactor(void)const;

private:
  void initialize(const sensor_msgs::LaserScan& initScan);

  void run(void);

  void laserScanCallBack(const sensor_msgs::LaserScan& scan);

  ros::NodeHandle _nh;

  ros::Subscriber _laserSubs;

  bool _initialized;

  obvious::TsdGrid* _grid;

  obvious::SensorPolar2D* _sensor;

  bool* _mask;

  Localization* _localizer;

  ThreadMapping* _threadMapping;

  ThreadGrid* _threadGrid;

  boost::mutex _pubMutex;

  double _xOffFactor;

  double _yOffFactor;

};

} /* namespace ohm_tsdSlam */

#endif /* SLAMNODE_H_ */
