#ifndef THREADLOCALIZATION_H_
#define THREADLOCALIZATION_H_

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/registration/icp/icp_def.h"
#include "ThreadSLAM.h"

#include <boost/thread.hpp>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#define ITERATIONS  50

#define TRNS_THRESH 1.0            // Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH  0.5             // the Transformation is not taken over

#define TRNS_MIN    0.05            // Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN     0.03            // greater than one of these values

namespace ohm_tsd_slam
{

class SlamNode;
class ThreadMapping;

/**
 * @class ThreadLocalization
 * @brief Localizes a laser scanner in a obvious::TsdGrid
 * @author Philipp Koch, Stefan May
 */
class ThreadLocalization : public ThreadSLAM
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
  ThreadLocalization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, const double xOffFactor, const double yOffFactor, const bool ransac=true);

  /**
   * Destructor
   */
  virtual ~ThreadLocalization();

  /**
   * localize
   * Method to calculate the localization to a given sensor instance
   * @param sensor
   */
  void localize(obvious::SensorPolar2D* sensor);

  /**
   * Idle flag indicating whether localization is still in progress
   * @return idle flag
   */
  bool isIdle();

  /**
   * Trigger registration of newly available data (if localization module is idle)
   * @param sensor Sensor instance
   */
  void triggerRegistration(obvious::SensorPolar2D* sensor);

private:

  /**
   * eventLoop
   * Thread event loop
   */
  virtual void eventLoop(void);

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
   * Mask of model
   */
  bool* _maskM;

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
   * Mask of scene
   */
  bool* _maskS;

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
   * RANSAC registration flag
   */
  bool _ransac;

  /**
   * Mutex for triggering registration of new data
   */
  boost::mutex _pushMutex;

  /**
   * Pointer to recent sensor instance
   */
  obvious::SensorPolar2D* _sensor;

};

} /* namespace ohm_tsd_slam*/

#endif /* THREADLOCALIZATION_H_ */
