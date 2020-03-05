/*
 * ThreadLocalize.h
 *
 *  Created on: Nov 6, 2018
 *      Refactured by: jasmin
 */

#ifndef THREADLOCALIZE_H_
#define THREADLOCALIZE_H_

#include "ThreadSLAM.h"
#include "OdometryAnalyzer.h"
#include "registration/Registration.h"

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/registration/ransacMatching/TwinPointMatching.h"
#include "obvision/registration/ransacMatching/RandomNormalMatching.h"
#include "obvision/registration/ransacMatching/PDFMatching.h"
#include "obvision/registration/ransacMatching/TSD_PDFMatching.h"
#include "registration/Registration.h"

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
class OdometryAnalyzer;

/**
 * default values in case no launch parameters are set
 */
namespace
 {
//   const unsigned int ICP_ITERATIONS 	= 25;
   const double TRNS_THRESH 			= 0.25;			//thresholds for registration, if calculated transf is out of these bounds, transf is not taken over
   const double ROT_THRESH				= 0.17;
   const double TRNS_VEL_MAX			= 1.5;
   const double ROT_VEL_MAX			= 2 * M_PI;
   const double TRNS_MIN 				= 0.05;			//minimal values for pose change, if pose change is greater than one of these vals -> push
   const double ROT_MIN			   	= 0.03;
//   double DIST_FILT_MIN				= 0.1;
//   double DIST_FILT_MAX				= 1.0;
//   int RANSAC_TRIALS 					= 50;
//   double RANSAC_EPS_THRESH			= 0.15;
//   int RANSAC_CTRL_SET_SIZE			= 180;
//   double RANSAC_PHI_MAX 				= 30.0;
}

class ThreadLocalize: public ThreadSLAM
{
  // enum EnumRegModes
  // {
  // 	ICP = 0,			///< Registration with ICP only
  // 	EXP = 1,
  // 	PDF = 2,
  // 	TSD = 3
  // };

public:
  /**
   * Constructor
   * @param grid Pointer to representation
   * @param mapper Pointer to mapping thread instance
   * @param nh pointer to main node handle
   * @param nameSpace Namespace of this localization thread
   * @param xOffset Origin x position
   * @param yOffset Origin y position
   */
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
  				const double xOffset, const double yOffset);

  /**
   * Destructor
   */
  virtual ~ThreadLocalize();
  /**
   * Callback method for laserscans
   * @param scan Laserscan data
   */
  void laserCallBack(const sensor_msgs::LaserScan& scan);

protected:
  /**
   * Abstract function derived from base class ThreadSLAM.
   * Consists of a loop the thread never leaves until a termination call occurs.
   */
  virtual void eventLoop(void);

private:
  /**
   * Init function automatically called by first received laser data.
   * @param scan Initial scan used to initialize all parameters of the thread
   */
  void init(const sensor_msgs::LaserScan& scan);

  /**
   * Method to analyze 2D transformation matrix.
   * @param T Pointer to transformation matrix
   * @return Calculated angle
   */
 // double calcAngle(obvious::Matrix* T);

  /**
   * Method to determine whether the localized sensor has been moved significantly (value higher than thresh).
   * A map update is only initiated in case this method returns true.
   * @param lastPose Pointer to last known pose
   * @param curPose Pointer to current pose
   * @return True in case of significant pose change
   */
 // bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);

  /**
   * Method to broadcast the calculated transformation via ros::geometry_msgs::PoseStamped and ros::tf.
   * @param T Calculated Transformation that is being broadcasted
   */
  void sendTransform(obvious::Matrix* T);

  /**
   * Method sending an irregular pose and tf (consisting only of NAN values), broadcasting a detected registration error.
   */
  void sendNanTransform();

  /**
   * Method to convert an 3x3 obvious::Matrix to a tf matrix
   * @param ob obvious::Matrix input to be converted
   * @return tf matrix tf::Transform
   */
  //tf::Transform obviouslyMatrix3x3ToTf(obvious::Matrix& ob);  //TODO: utilities

  /**
   * Converts a tf matrix to a 3x3 obvious::Matrix
   * @param tf matrix to be converted
   * @return converted obvious::Matrix
   */
  //obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf); //TODO: utilities

  /**
   * Pointer to main NodeHandle
   */
  ros::NodeHandle* _nh;
  /**
   * Pointer to mapping thread
   */
  ThreadMapping& _mapper;
  /**
   * Sensor container for handling the current laser input and pose
   */
  obvious::SensorPolar2D* _sensor;
  /**
   * Flag for successful initialization of this thread
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
   * Grid origin x offset
   */
  const double _gridOffSetX;
  /**
   * Grid origin y offset
   */
  const double _gridOffSetY;
  /**
   * Initial xOffset
   */
  const double _xOffset;
  /**
   * Initial yOffset
   */
  const double _yOffset;
  /**
   * namespace for all topics and services
   */
  std::string _nameSpace;
  /**
   * Laser time stamp now
   */
  ros::Time _stampLaser;

  boost::mutex _dataMutex;

  double _threshMinPoseChangeLin;

  double _threshMinPoseChangeAng;

  /**
   * Registration mode
   */
  RegModes _regMode;   //TODO: move to config file of registration class

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
   * 2D reconstruction done by Raycaster
   */
  obvious::RayCastPolar2D* _rayCaster;
  
  /**
   * Last pose stored in Matrix
   */
  obvious::Matrix* _lastPose;
  /**
   * ROS pose publisher
   */
  ros::Publisher _posePub;

  ros::Publisher _pubPoseStCov;  ///<  publisher for pose with covariance and stamp
  /**
   * ROS current pose
   */
  geometry_msgs::PoseStamped _poseStamped;

  geometry_msgs::PoseWithCovarianceStamped _poseStampedCov;
  /**
   * ROS tf interface - broadcaster
   */
  tf::TransformBroadcaster _tfBroadcaster;
  /**
   * ROS tf interface - listener
   */
  tf::TransformListener _tfListener;
  /**
   * Containre for reading tfs
   */
  tf::StampedTransform _tfReader;
  /**
   * ROS current transform
   */
  tf::StampedTransform _tf;
  /**
   * ROS tf Transform from base_footprint to laser
   */
  tf::Transform _tfLaser;
  /**
   * ROS tf frame ids
   */
  std::string _tfBaseFrameId;
  /**
   * ROS tf frame ids
   */
  std::string _tfChildFrameId;
  /**
   * ROS tf frame ids
   */
  std::string _tfOdomFrameId;
  /**
   * ROS tf frame ids
   */
  std::string _tfFootprintFrameId;
  /**
   * use odom rescue flag
   */
  bool _useOdomRescue;
  /**
   * state of the actual odom tf transform
   */
  bool _odomTfIsValid;
  /**
   * time to wait for synchronized odom tf
   */
  ros::Duration _waitForOdomTf;
  /**
   * Laser time stamp old
   */
  ros::Time _stampLaserOld;
  /**
   * Scan passed in clockwise rotation (mathematically negative increment)
   */
  bool _reverseScan;
  /**
   * Minimum laser range
   */
  double _lasMinRange;

  tf::StampedTransform _tfFrameSensorMount;

  std::unique_ptr<Registration> _registration;

};

} /* namespace ohm_tsd_slam_ref */

#endif /* THREADLOCALIZE_H_ */
