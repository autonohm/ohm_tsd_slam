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
#include "obvision/registration/icp/icp_def.h"

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

#define ITERATIONS 25
#define TRNS_THRESH 0.25            //Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH 0.17             //the Transformation is not taken over
#define TRNS_MIN 0.05              //Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN 0.09               //greater than than one of these values

namespace ohm_tsd_slam
{

class MultiSlamNode;
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
   */
  Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, const double xOffFactor, const double yOffFactor,
      std::string nameSpace = "");

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

  bool togglePushServiceCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * doRegistration
   * Encapsulate the registration process. For most parameters the member variable are used.
   * It is possible to use Ransac or not.
   * @param sensor Sensor used for SLAM
   * @param M Unfiltered matrix of the model scan.
   * @param N Unfiltered matrix of the model normals.
   * @param S Unfiltered matrix of the scene scan.
   * @param useRansac Set true if pre registration with ransac is required
   */
  obvious::Matrix doRegistration(obvious::SensorPolar2D* sensor,
      obvious::Matrix* M,
      obvious::Matrix* Mvalid,
      obvious::Matrix* N,
      obvious::Matrix* Nvalid,
      obvious::Matrix* S,
      obvious::Matrix* Svalid,
      const bool useRansac
  );
  /**
   * isRegistrationError
   * Proves if the calculated transformation between two scans
   * is bigger than member variables _rotMax and _transMax.
   * @param T Transformation between scene and model scan.
   * @return True if the difference is to big. False if it is reasonable.
   */
  bool isRegistrationError(obvious::Matrix* T, const double trnsMax, const  double rotMax);


  /**
   * sendTransform
   * Publishes the new sensor position and updates TF
   * @param T Transformation Matrix with current Sensor Pose (after sucessful registration)
   */
  void sendTransform(obvious::Matrix* T);

  /**
   * sendNanTransform
   * Publish a NAN message in ROS to signal everybody a registration error
   */
  void sendNanTransform();

  ros::NodeHandle* _nh;

  const double _gridOffSetX;

  const double _gridOffSetY;

  ros::ServiceServer _togglePushService;

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
   * Pointer to publishing mutex
   */
  //boost::mutex* _pubMutex;

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

  /*
   * RANSAC Reduction: Use to scale down the number of points for ransac
   * Example: Points in Scan = 1080
   *          ransacReduceFactor = 4
   *          -> points for ransac = 1080 /ransacReduceFactor = 270;
   */
  unsigned int _ransacReduceFactor;

  /**
   * Flag to disable push (active = no altering of the map)
   */
  bool _noPush;

  unsigned int _ranTrials;
  double _ranEpsThresh;
  unsigned int _ranSizeCtrlSet;

  /**
   * namespace for all topics and services
   */
  std::string _nameSpace;

};

obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints);
void maskToOneDegreeRes(bool* const mask, const double resolution, const unsigned int maskSize);
void reduceResolution(bool* const maskIn, const obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
    unsigned int pointsIn, unsigned int pointsOut, unsigned int reductionFactor);

} /* namespace ohm_tsd_slam*/

#endif /* LOCALIZATION_H_ */
