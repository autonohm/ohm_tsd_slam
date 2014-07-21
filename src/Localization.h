#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/icp/icp_def.h"

#include <boost/signal.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#define ITERATIONS 100
#define TRNS_THRESH 1.5            //Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH 0.6             //the Transformation is not taken over
#define TRNS_MIN 0.05              //Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN 0.09               //greater than than one of these values

namespace ohm_tsd_slam
{

class SlamNode;
class ThreadMapping;
/**
 * @class Localization
 * @author Philipp Koch, Stefan May
 * @date 08.06.2014
 */
class Localization
{
public:

  Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, boost::mutex* pubMutex, SlamNode& parentNode);

  virtual ~Localization();

  void localize(obvious::SensorPolar2D* sensor);

 // const sensor_msgs::LaserScan& getScanToPush(void)const{return *_scanToPush;}

private:

  double calcAngle(obvious::Matrix* T);

  bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);

  ThreadMapping* _mapper;

  double* _modelCoords;

  double* _modelNormals;

  /**
   * reconstruction
   */
  obvious::RayCastPolar2D* _rayCaster;

  obvious::TsdGrid* _grid;

  double* _scene;

  /**
   * registration //toDo: move in a separate module
   */
  obvious::PairAssignment* _assigner;

  obvious::OutOfBoundsFilter2D* _filterBounds;

  obvious::DistanceFilter* _filterDist;

  obvious::ReciprocalFilter* _filterReciprocal;

  obvious::IRigidEstimator* _estimator;

  obvious::Icp* _icp;

  double _trnsMax;

  double _rotMax;

  obvious::Matrix* _lastPose;

  //sensor_msgs::LaserScan* _scanToPush;

  ros::Publisher _posePub;

  geometry_msgs::PoseStamped _poseStamped;

  tf::TransformBroadcaster _tfBroadcaster;

  tf::StampedTransform _tf;

  boost::mutex* _pubMutex;

  double _xOffFactor;

  double _yOffFactor;
};

} /* namespace */

#endif /* LOCALIZATION_H_ */
