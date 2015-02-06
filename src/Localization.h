/*
 * Localization.h
 *
 *  Created on: 17.04.2014
 *      Author: Philipp Koch, Stefan May
 */

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

#define ITERATIONS  25
#define TRNS_THRESH 1.25            //Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH  0.9             //the Transformation is not taken over
#define TRNS_MIN    0.05              //Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN     0.09               //greater than than one of these values

namespace ohm_tsd_slam
{

class SlamNode;
class ThreadMapping;

/**
 * @class Localization
 * @brief Localizes a laser scanner in a obvious::TsdGrid
 * @author Philipp Koch, Stefan May
 */
class Localization
{
public:

  /**
   * Constructor
   * @param grid Representation
   * @param mapper Mapping thread
   * @param nh Ros node handle
   * @param pubMutex Mutex synchronizing ros publishing
   * @param parentNode Pointer to main node instance
   * @param ransac use RANSAC pre-registration
   */
  Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, const double xOffFactor, const double yOffFactor, const bool ransac=true);

  /**
   * Destructor
   */
  virtual ~Localization();

  /**
   * localize
   * Method to calculate the localization to a given sensor instance
   * @param sensor
   */
  void localize(obvious::SensorPolar2D* sensor);

private:

  /**
   * calcAngle
   * Method to calculate the yaw angle out of a given 2D transformation
   * @param T Source
   * @return angle
   */
  double calcAngle(obvious::Matrix* T);

  /**
   * isPoseChangeSignificant
   * Method determining whether a given transformation contains a significant pose change
   * @param lastPose Source
   * @param curPose Source
   * @return necessary
   */
  bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);

  const double _gridOffSetX;
  const double _gridOffSetY;

  /**
   * Pointer to main node instance
   */
  ThreadMapping* _mapper;

  /**
   * Buffer for model coordinates
   */
  double* _modelCoords;

  /**
   * Buffer for model normals
   */
  double* _modelNormals;

  /**
   * reconstruction
   */
  obvious::RayCastPolar2D* _rayCaster;

  /**
   * Representation
   */
  obvious::TsdGrid* _grid;

  /**
   * Buffer for scene coordinates
   */
  double* _scene;

  /**
   * ICP pair assigner
   */
  obvious::PairAssignment* _assigner;

  /**
   * ICP out of bounds filter
   */
  obvious::OutOfBoundsFilter2D* _filterBounds;

  /**
   * ICP distance filter
   */
  obvious::DistanceFilter* _filterDist;

  /**
   * ICP reciprogal filter
   */
  obvious::ReciprocalFilter* _filterReciprocal;

  /**
   * ICP transformation estimator
   */
  obvious::IRigidEstimator* _estimator;

  /**
   * ICP main icp instance
   */
  obvious::Icp* _icp;

  /**
   * ICP translation threshold
   */
  double _trnsMax;

  /**
   * ICP rotation threshold
   */
  double _rotMax;

  /**
   * Last pose
   */
  obvious::Matrix* _lastPose;

  /**
   * Ros pose publisher
   */
  ros::Publisher _posePub;

  /**
   * Ros current pose
   */
  geometry_msgs::PoseStamped _poseStamped;

  /**
   * Ros tf interface
   */
  tf::TransformBroadcaster _tfBroadcaster;

  /**
   * Ros current transform
   */
  tf::StampedTransform _tf;

  /**
   * Starting x offset
   */
  double _xOffFactor;

  /**
   * Starting y offset
   */
  double _yOffFactor;

  /**
   * Static sensor offset to kinematic center x-axis
   */
  double _lasXOffset;

  /**
   * RANSAC registration flag
   */
  bool _ransac;
};

} /* namespace ohm_tsd_slam*/

#endif /* LOCALIZATION_H_ */
