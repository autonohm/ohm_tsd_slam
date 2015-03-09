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

#include <string>

namespace ohm_tsd_slam
{

class MultiSlamNode;
class ThreadMapping;
class Localization;

class ThreadLocalize: public ThreadSLAM
{
public:
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, boost::mutex* pubMutex, std::string nameSpace,
      const double xOffFactor, const double yOffFactor);
  virtual ~ThreadLocalize();
  bool setData(const sensor_msgs::LaserScan& scan);
protected:
  virtual void eventLoop(void);
private:
  void laserCallBack(const sensor_msgs::LaserScan& scan);
  void init(const sensor_msgs::LaserScan& scan);

  ros::NodeHandle _nh;
  ros::Subscriber _lasSubs;
  obvious::TsdGrid& _grid;
  ThreadMapping& _mapper;
  std::string _nameSpace;
  obvious::SensorPolar2D* _sensor;
  Localization* _localizer;
  bool _newScan;
  bool _initialized;
  double _xOffset;
  double _yOffset;
  double _yawOffset;
  double _maxRange;
  double _minRange;
  double _footPrintWidth;
  double _footPrintHeight;
  double _footPrintXoffset;
  double _gridWidth;
  double _gridHeight;
  double _xOffFactor;
  double _yOffFactor;
  bool* _mask;
  boost::mutex _dataMutex;
  sensor_msgs::LaserScan _scan;
};

} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */
