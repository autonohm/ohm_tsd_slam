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
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
      const double xOffFactor, const double yOffFactor);
  virtual ~ThreadLocalize();
  bool setData(const sensor_msgs::LaserScan& scan);
protected:
  virtual void eventLoop(void);
private:
  void init(const sensor_msgs::LaserScan& scan);

  ros::NodeHandle* _nh;
  ros::Subscriber _lasSubs;
  obvious::TsdGrid& _grid;
  ThreadMapping& _mapper;
  obvious::SensorPolar2D* _sensor;
  Localization* _localizer;
  bool _newScan;
  bool _initialized;
  std::string _nameSpace;
  double _gridWidth;
  double _gridHeight;
  double _xOffFactor;
  double _yOffFactor;
  bool* _mask;
  boost::mutex _dataMutex;
};

} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */
