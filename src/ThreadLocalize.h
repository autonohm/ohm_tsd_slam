/*
 * ThreadLocalize.h
 *
 *  Created on: 28.10.2014
 *      Author: phil
 */

#ifndef THREADLOCALIZE_H_
#define THREADLOCALIZE_H_

#include "ThreadSLAM.h"
#include "ControllerOdom.h"

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/registration/ransacMatching/TwinPointMatching.h"
#include "obvision/registration/ransacMatching/RandomNormalMatching.h"
#include "obvision/registration/ransacMatching/PDFMatching.h"
#include "obvision/registration/ransacMatching/TSD_PDFMatching.h"

#include "obgraphic/Obvious2D.h" //debugging tsd_pdf
#include "obcore/base/tools.h" //debugging tsd_pdf
#include <iostream> //debugging tsd_pdf
#include <fstream> //debugging tsd_pdf

#include <string>
#include <cmath>

namespace ohm_tsd_slam
{

class SlamNode;
class ThreadMapping;
class Localization;


namespace
{   //default values in case no launch parameters are set
const unsigned int ICP_ITERATIONS = 25;
const double TRNS_THRESH = 0.25;            //Thresholds for registration. If the gained transformation is out of these bounds,
const double ROT_THRESH = 0.17;
const double TRNS_VEL_MAX = 1.5;
const double ROT_VEL_MAX = 2 * M_PI;       //the Transformation is not taken over
const double TRNS_MIN = 0.05;              //Minimal values for the pose change. Push is only needed when pose change
const double ROT_MIN = 0.03;               //greater than than one of these values
const double DIST_FILT_MIN = 0.1;
const double DIST_FILT_MAX = 1.0;
const int RANSAC_TRIALS =  50;
const double RANSAC_EPS_THRESH = 0.15;
const int RANSAC_CTRL_SET_SIZE = 180;
const double RANSAC_PHI_MAX = 30.0;
}

class ThreadLocalize: public ThreadSLAM
{
  enum EnumRegModes
  {
    ICP = 0,    ///< Registration with Icp only
    EXP = 1,
    PDF = 2,
    TSD = 3
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
      const double xOffset, const double yOffset);

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
      obvious::Matrix* Svalid
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
  void reduceResolution(bool* const maskIn, obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
      unsigned int pointsIn, unsigned int pointsOut, unsigned int reductionFactor);

  /**
   * odomRescueInit
   * Method to initialize odom recover system
   */
  void odomRescueInit();

  /**
   * odomRescueUpdate
   * updates odometry data if a new scan comes in
   */
  void odomRescueUpdate();

  /**
   * odomRescueCheck
   * check if slam transformation is plausible and overwrites T with odometry as transformation if not
   * @param T Transformation matrix to check and correct
   */
  void odomRescueCheck(obvious::Matrix& T);

//  /**
//   * obviouslyMatrix3x3ToTf
//   * converts an 3x3 obvious matrix to a tf matrix
//   * @param ob Obvious matrix to convert
//   * @return transformed tf matrix
//   */
//  tf::Transform obviouslyMatrix3x3ToTf(obvious::Matrix& ob);
//
//  /**
//   * tfToObviouslyMatrix3x3
//   * converts an tf matrix to a 3x3 obvious matrix
//   * @param tf tf matrix to transform
//   * @return transformed obvious matrix
//   */
//  obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf);

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
  double _trnsVelocityMax;

  /**
   * ICP rotation threshold
   */
  double _rotMax;
  double _rotVelocityMax;

  /**
   * Starting x offset
   */
  const double _xOffset;

  /**
   * Starting y offset
   */
  const double _yOffset;

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
   * Matcher instance
   */
  obvious::RandomNormalMatching* _RandomNormalMatcher;
  obvious::PDFMatching* _PDFMatcher;
  obvious::TSD_PDFMatching* _TSD_PDFMatcher;

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
  tf::TransformListener _tfListener;

  /**
   * Container for reading tfs
   */
  tf::StampedTransform _tfReader;

  /**
   * Ros current transform
   */
  tf::StampedTransform _tf;

  /**
   * Odom Transforms
   */
  tf::Transform _tfOdomOld;
  tf::Transform _tfOdom;
  tf::Transform _tfRelativeOdom;

  /**
   * Transform from base footprint to laser
   */
  tf::Transform _tfLaser;

  /**
   * ros tf frame ids
   */
  std::string _tfFootprintFrameId;
  std::string _tfOdomFrameId;
  std::string _tfBaseFrameId;
  std::string _tfChildFrameId;

  /**
   * use odom rescue flag
   */
  bool _useOdomRescue;

  /**
   * state of the actual odom tf
   */
  bool _odomTfIsValid;

  /**
   * time to wait for synced odom tf
   */
  ros::Duration _waitForOdomTf;

  /**
   * Laser time stamps
   */
  ros::Time _stampLaser;
  ros::Time _stampLaserOld;

  /**
   * Scan passed in clockwise rotation (mathematically negative increment)
   */
  bool _reverseScan;

  ControllerOdom _odom;
};



} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */
