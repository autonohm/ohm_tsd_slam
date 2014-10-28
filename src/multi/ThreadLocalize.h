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
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/icp/icp_def.h"

#define ITERATIONS 25
#define TRNS_THRESH 1.5            //Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH 0.6             //the Transformation is not taken over
#define TRNS_MIN 0.05              //Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN 0.09               //greater than than one of these values


namespace ohm_tsd_slam
{

class SlamNode;
class ThreadMapping;

class ThreadLocalize: public ThreadSLAM
{
public:
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, boost::mutex* pubMutex, SlamNode& parentNode);
  virtual ~ThreadLocalize();
  void localize(obvious::SensorPolar2D* sensor);
protected:
  virtual void eventLoop(void);
private:
  void laserCallBack(const sensor_msgs::LaserScan& scan);
  double calcAngle(obvious::Matrix* T);
  bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);
  void init(const sensor_msgs::LaserScan& scan);
  bool _initialized;
  bool _newScan;
  obvious::SensorPolar2D* _sensor;
  bool* _mask;
  double _minRange;
  double _maxRange;
  double _xOffset;
  double _yOffset;
  double _yawOffset;
  bool _rangeFilter;


  ros::NodeHandle _nh;
  ros::Subscriber _laserSubs;
  ThreadMapping* _mapper;
  double* _modelCoords;
  double* _modelNormals;
  obvious::RayCastPolar2D* _rayCaster;
  obvious::TsdGrid* _grid;
  double* _scene;
  obvious::PairAssignment* _assigner;
  obvious::OutOfBoundsFilter2D* _filterBounds;
  obvious::DistanceFilter* _filterDist;
  obvious::ReciprocalFilter* _filterReciprocal;
  obvious::IRigidEstimator* _estimator;
  obvious::Icp* _icp;
  double _trnsMax;
  double _rotMax;
  obvious::Matrix* _lastPose;
  ros::Publisher _posePub;
  geometry_msgs::PoseStamped _poseStamped;
  tf::TransformBroadcaster _tfBroadcaster;
  tf::StampedTransform _tf;
  boost::mutex* _pubMutex;
  double _xOffFactor;
  double _yOffFactor;
  double _lasXOffset;
};

} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */
