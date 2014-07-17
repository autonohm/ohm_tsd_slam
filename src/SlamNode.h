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

#include "Localization.h"
#include "ThreadSLAM.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"
#include "obcore/base/Logger.h"

#define CELLSIZE 0.025   //toDo: Launch file parameters
#define TRUNCATION_RADIUS 3.0
#define MAX_RANGE 30.0
#define MIN_RANGE 0.01
#define S_X_F 0.2                  //start point in x (S_F_X * sizeX)
#define S_Y_F 0.5                  //start point in y (S_F_Y * sizeY)
#define THETA_INIT 0.0  //used in degrees
#define MAP_T 2.0      //time between map generations
#define INIT_PSHS 50//number of initial puhses into the grid
#define LAS_OFFS_X -0.19 //offset of the laser scanner to the base footprint

namespace ohm_tsd_slam
{

class SlamNode
{
public:
  SlamNode();

  virtual ~SlamNode();

  void start(void);

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

};

} /* namespace ohm_tsdSlam */

#endif /* SLAMNODE_H_ */
