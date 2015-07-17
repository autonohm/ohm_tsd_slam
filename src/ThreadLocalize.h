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
#include "obvision/registration/icp/icp_def.h"

#include <string>

#define ITERATIONS 25               //toDo: remove these maybe in a nameless namespace
#define TRNS_THRESH 0.25            //Thresholds for registration. If the gained transformation is out of these bounds,
#define ROT_THRESH 0.17             //the Transformation is not taken over
#define TRNS_MIN 0.05              //Minimal values for the pose change. Push is only needed when pose change
#define ROT_MIN 0.03               //greater than than one of these values

namespace ohm_tsd_slam
{

class SlamNode;
class ThreadMapping;
class Localization;

class ThreadLocalize: public ThreadSLAM
{
  enum EnumRegModes
  {
    ICP = 0,    ///< Registration with Icp only
    EXP         ///< Experimental Registration scheme, use with caution
  };

public:
  /**
   * Constructor
   * @param grid Pointer to representation
   * @param mapper Pointer to mapping thread instance
   * @param nh Pointer to main node handle
   * @param nameSpace Namespace of this localization thread
   * @param xOffFactor Origin x position
   * @param yOffFactor Origin y position
   */
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
      const double xOffFactor, const double yOffFactor);

  /**
   * Destructor
   */
  virtual ~ThreadLocalize();

  /**
   * laserCallBack
   * Callback method for laser scan message
   * @param scan Laser data
   */
  void laserCallBack(const sensor_msgs::LaserScan& scan);
protected:

  /**
   * eventLoop
   * Thread event loop
   */
  virtual void eventLoop(void);
private:

  /**
   * init
   * Init function automatically called by firs received laser data
   * @param scan Initial scan used to initialize parameters of the thread
   */
  void init(const sensor_msgs::LaserScan& scan);

  /**
   * calAngle
   * Method to analyze a 2D transformation matrix
   * @param T Pointer to transformation matrix
   * @return Calculated angle
   */
  double calcAngle(obvious::Matrix* T);

  /**
   * isPoseChangeSignificant
   * Method to determine whether the localized sensor has been moved significantly (value above thresh).
   * A map change is only initiated in case this method returns true.
   * @param lastPose Pointer to last known pose
   * @param curPose Pointer to current pose
   * @return True in case of a significant pose change
   */
  bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);

  /**
   * doRegistration
   * Main registration method. Aligns two laser scans and calculates the referring 2D transformation matrix.
   * @param sensor Laser data container
   * @param M Pointer to model points
   * @param Mvalid
   * @param N Pointer to the normals of the model
   * @param Nvalid
   * @param S Pointer to scene points
   * @param Svalid
   * @param experimental Flag to enable experimental matching
   * @return 2D transformation matrix
   */
  obvious::Matrix doRegistration(obvious::SensorPolar2D* sensor,
      obvious::Matrix* M,
      obvious::Matrix* Mvalid,
      obvious::Matrix* N,
      obvious::Matrix* Nvalid,
      obvious::Matrix* S,
      obvious::Matrix* Svalid,
      const bool experimental
  );

  /**
   * isRegistrationError
   * Method to prevent registration errors by comparing the computed transformation to thresholds.
   * @param T Current 2D transformation
   * @param trnsMax Translation thresh
   * @param rotMax Rotation thresh
   * @return True in case of an error
   */
  bool isRegistrationError(obvious::Matrix* T, const double trnsMax, const  double rotMax);

  /**
   * sendTransform
   * Method to broadcast the gained transformation via ros::geometry_msgs::PoseStamped and ros::tf.
   * @param T Broadcasted transformation
   */
  void sendTransform(obvious::Matrix* T);

  /**
   * sendNanTransform
   * Method sending an irregular pose and tf (consisting only of NAN values), broadcasting a detected registration error.
   */
  void sendNanTransform();

  /**
   * maskMatrix
   * Method to remove certain values in a matrix using a given mask.
   * @param Mat Input data
   * @param mask Value mask
   * @param maskSize Amount of values in the mask
   * @param validPoints Value determining the number of values in the output matrix
   * @return Filtered matrix
   */
  obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints);

  /**
   * reduceResolution
   * Method to reduce the values in a matrix with a given factor.
   * @param maskIn Input mask
   * @param matIn Input data
   * @param maskOut Output mask
   * @param matOut Filtered data
   * @param pointsIn Amount of points in un filtered data
   * @param pointsOut Amount of points in filtered data
   * @param reductionFactor Reduction factor
   */
  void reduceResolution(bool* const maskIn, const obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
      unsigned int pointsIn, unsigned int pointsOut, unsigned int reductionFactor);


  /**
   * Pointer to main NodeHandle
   */
  ros::NodeHandle* _nh;

  /**
   * Pointer to mapping thread
   */
  ThreadMapping& _mapper;

  /**
   * Sensor container for handeling the current laser input and pose
   */
  obvious::SensorPolar2D* _sensor;

  /**
   * Flag signifying successful initialization of this thread
   */
  bool _initialized;

  /**
   * Width of tsd grid in m
   */
  const double _gridWidth;

  /**
   * Height of tsd grid in m
   */
  const double _gridHeight;

  /**
   * Mutex to synchronize main thread (subscriber) and thread event loop toDO: either this or the flas are obsolete
   */
  boost::mutex _dataMutex;

  /**
   * Grid origin x offset
   */
  const double _gridOffSetX;

  /**
   * Grid origin y offset
   */
  const double _gridOffSetY;

  /**
   * ICP translation threshold
   */
  double _trnsMax;

  /**
   * ICP rotation threshold
   */
  double _rotMax;

  /**
   * Starting x offset
   */
  const double _xOffFactor;

  /**
   * Starting y offset
   */
  const double _yOffFactor;

  /**
   * Registration mode
   */
  EnumRegModes _regMode;

  /*
   * RANSAC Reduction: Use to scale down the number of points for ransac
   * Example: Points in Scan = 1080
   *          ransacReduceFactor = 4
   *          -> points for ransac = 1080 /ransacReduceFactor = 270;
   */
  //unsigned int _ransacReduceFactor;

  /**
   * Iterations for experimental registration alorithm
   */
  unsigned int _ranTrials;

  /**
   * Threshold for experimental registration algorithm
   */
  double _ranEpsThresh;

  /**
   * Control size set for experimental registration algorithm
   */
  unsigned int _ranSizeCtrlSet;

  /**
   * Angle threshold for experimental registration algorithm
   */
  double _ranPhiMax;

  /**
   * namespace for all topics and services
   */
  std::string _nameSpace;

  /**
   * Container for laser sensor data (filled by callback)
   */
  std::deque<sensor_msgs::LaserScan*> _laserData;

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
   * Buffer for scene coordinates
   */
  double* _scene;

  /**
   * Mask of scene
   */
  bool* _maskS;

  /**
   * reconstruction
   */
  obvious::RayCastPolar2D* _rayCaster;

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
};



} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */
