/*
 * ThreadLocalize.h
 *
 *  Created on: 28.10.2014
 *      Author: phil
 */

#ifndef THREADLOCALIZE_H_
#define THREADLOCALIZE_H_

#include "ThreadSLAM.h"

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"

#define ITERATIONS 25
#define TRNS_THRESH 1.5            //Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH 0.6             //the Transformation is not taken over
#define TRNS_MIN 0.05              //Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN 0.09               //greater than than one of these values


namespace ohm_tsd_slam
{

class MultiSlamNode;
class ThreadMapping;
class Localization;

class ThreadLocalize: public ThreadSLAM
{
public:
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, boost::mutex* pubMutex, MultiSlamNode& parentNode, std::string nameSpace);
  virtual ~ThreadLocalize();
protected:
  virtual void eventLoop(void);
private:
  void laserCallBack(const sensor_msgs::LaserScan& scan);
  void init(const sensor_msgs::LaserScan& scan);
  ros::NodeHandle _nh;
  ros::Subscriber _lasSubs;

  obvious::SensorPolar2D* _sensor;
  bool _newScan;
  Localization* _localizer;
  double _maxRange;
  double _minRange;
  double _yawOffset;
  double _xOffset;
  double _yOffset;
  double _gridWidth;
  double _gridHeight;
  double _xOffFactor;
  double _yOffFactor;
  bool _initialized;
  MultiSlamNode& _parentNode;


};

} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */