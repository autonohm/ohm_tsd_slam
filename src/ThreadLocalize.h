/*
 * ThreadLocalize.h
 *
 *  Created on: Nov 6, 2018
 *      Refactured by: jasmin
 */
#pragma once
#include "ThreadSLAM.h"
// #include "OdometryAnalyzer.h"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/registration/ransacMatching/TwinPointMatching.h"
#include "obvision/registration/ransacMatching/RandomNormalMatching.h"
#include "obvision/registration/ransacMatching/PDFMatching.h"
#include "obvision/registration/ransacMatching/TSD_PDFMatching.h"

// #include <obgraphic/Obvious2D.h> //debugging tsd_pdf
#include <obcore/base/tools.h> //debugging tsd_pdf
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
  const unsigned int ICP_ITERATIONS 	= 25;
  const double TRNS_THRESH 			= 0.25;			//thresholds for registration, if calculated transf is out of these bounds, transf is not taken over
  const double ROT_THRESH				= 0.17;
  const double TRNS_VEL_MAX			= 1.5;
  const double ROT_VEL_MAX			= 2 * M_PI;
  const double TRNS_MIN 				= 0.05;			//minimal values for pose change, if pose change is greater than one of these vals -> push
  const double ROT_MIN				= 0.03;
  double DIST_FILT_MIN				= 0.1;
  double DIST_FILT_MAX				= 1.0;
  int RANSAC_TRIALS 					= 50;
  double RANSAC_EPS_THRESH			= 0.15;
  int RANSAC_CTRL_SET_SIZE			= 180;
  double RANSAC_PHI_MAX 				= 30.0;
}

class ThreadLocalize: public ThreadSLAM
{
  enum EnumRegModes
  {
  	ICP = 0,			///< Registration with ICP only
  	EXP = 1,
  	PDF = 2,
  	TSD = 3
  };

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
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, const std::shared_ptr<rclcpp::Node>& node,
                 const std::string& robot_name, const double xOffset, const double yOffset);

  /**
   * Destructor
   */
  virtual ~ThreadLocalize();
  /**
   * Callback method for laserscans
   * @param scan Laserscan data
   */
  void laserCallBack(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan);

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
  void init(const sensor_msgs::msg::LaserScan& scan);

  /**
   * Method to analyze 2D transformation matrix.
   * @param T Pointer to transformation matrix
   * @return Calculated angle
   */
  double calcAngle(obvious::Matrix* T);

  /**
   * Method to determine whether the localized sensor has been moved significantly (value higher than thresh).
   * A map update is only initiated in case this method returns true.
   * @param lastPose Pointer to last known pose
   * @param curPose Pointer to current pose
   * @return True in case of significant pose change
   */
  bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);

  /**
   * Main registration method. Aligns two laser scans and calculates the referring 2D transformation matrix.
   * @param sensor Laser data container
   * @param M Pointer to model points
   * @param Mvalid Pointer to valid model points
   * @param N Pointer to normals of the model
   * @param Nvalid Pointer to valid normals of the model
   * @param S Pointer to scene points
   * @param Svalid Pointer to valid scene points
   * @return 2D transformation matrix of type obvious::Matrix
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
   * Method to prevent registration errors by comparing the computed transformation to thresholds.
   * @param T Current 2D transformation
   * @param trnsMax Translation thresh
   * @param rotMax Rotation thresh
   * @return True in case of an error
   */
  bool isRegistrationError(obvious::Matrix* T, const double trnsMax, const double rotMax);

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
   * Method to remove certain values in a matrix using a given mask.
   * @param Mat Input matrix data
   * @param mask Value mask
   * @param maskSize Amount of values in the mask
   * @param validPoints Value determining the number of values in the output matrix
   * @return Filtered matrix of type obvious::Matrix
   */
  obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints);

  /**
   * Method to reduce the values in a matrix with a given reduction factor.
   * @param maskIn Input mask
   * @param matIn Input matrix data
   * @param maskOut Output mask
   * @param matOut Filtered matrix data
   * @param pointsIn Amount of points in unfiltered data
   * @param pointsOut Amount of points in filtered data
   * @param reductionFactor Reduction factor
   */
  void reduceResolution(bool* const maskIn, obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
  						unsigned int pointsIn, unsigned int pointsOut, unsigned int reductionFactor);

  /**
   * Method to convert an 3x3 obvious::Matrix to a tf matrix
   * @param ob obvious::Matrix input to be converted
   * @return tf matrix tf::Transform
   */
  tf2::Transform obviouslyMatrix3x3ToTf(obvious::Matrix& ob);

  /**
   * Converts a tf matrix to a 3x3 obvious::Matrix
   * @param tf matrix to be converted
   * @return converted obvious::Matrix
   */
  obvious::Matrix tfToObviouslyMatrix3x3(const tf2::Transform& tf);

  /**
   * Pointer to main NodeHandle
   */
  std::shared_ptr<rclcpp::Node> _node;
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
  std::string _robotName;
  std::string _nameSpace;
  /**
   * Laser time stamp now
   */
  rclcpp::Time _stampLaser;


  /**
   * Pointer to odometry analyzer class
   */
  //OdometryAnalyzer* _odomAnalyzer;
  /**
   * Mutex to synchronize main thread (subscriber) and thread event loop
   * @todo either this or flags obsolete
   */
  boost::mutex _dataMutex;

  /**
   * ICP translation threshold
   */
  double _trnsMax;
  /**
   * ICP translation threshold
   */
  double _trnsVelocityMax;
  /**
   * ICP rotation threshold
   */
  double _rotMax;
  /**
   * ICP rotation threshold
   */
  double _rotVelocityMax;
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
   * Iterations for registration algorithm
   */
  unsigned int _ranTrials;
  /**
   * Threshold for experimental registration algorithm
   * @todo was ist das?
   */
  double _ranEpsThresh;
  /**
   * Control size set for registration algorithm
   */
  unsigned int _ranSizeCtrlSet;
  /**
   * Angle threshold for registration algorithm
   */
  double _ranPhiMax;
  /**
   * Container for laser sensor data (filled by callback)
   */
  std::deque<std::shared_ptr<sensor_msgs::msg::LaserScan>> _laserData;
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
   * ICP pair assigners
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
   * ICP reciprocal filter
   * @todo what is this?
   */
  obvious::ReciprocalFilter* _filterReciprocal;
  /**
   * ICP transformation estimator
   */
  obvious::IRigidEstimator* _estimator;
  /**
   * ICP main instance
   */
  obvious::Icp* _icp;
  /**
   * Matcher instance
   */
  obvious::RandomNormalMatching* _RandomNormalMatcher;
  /**
   * Matcher instance
   */
  obvious::PDFMatching* _PDFMatcher;
  /**
   * Matcher instance
   */
  obvious::TSD_PDFMatching* _TSD_PDFMatcher;
  /**
   * Last pose stored in Matrix
   */
  obvious::Matrix* _lastPose;
  /**
   * ROS pose publisher
   */
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> _posePub;
  /**
   * ROS current pose
   */
  geometry_msgs::msg::PoseStamped _poseStamped;
  /**
   * ROS tf interface - broadcaster
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;
  /**
   * ROS tf interface - listener
   */
  std::unique_ptr<tf2_ros::TransformListener> _tf_transform_listener;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  /**
   * Containre for reading tfs
   */
  
  tf2::Transform _tfReader;
  /**
   * ROS current transform
   */
  geometry_msgs::msg::TransformStamped _tf;
  /**
   * ROS tf Transform from base_footprint to laser
   */
  tf2::Transform _tfLaser;
  /**
   * ROS tf frame ids
   */
  std::string _tfLaserFrameId;
  /**
   * ROS tf frame ids
   */
  std::string _tfMapFrameId;
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
  rclcpp::Duration _waitForOdomTf;
  /**
   * Laser time stamp old
   */
  rclcpp::Time _stampLaserOld;
  /**
   * Scan passed in clockwise rotation (mathematically negative increment)
   */
  bool _reverseScan;
  /**
   * Minimum laser range
   */
  double _lasMinRange;

  geometry_msgs::msg::TransformStamped _tfFrameSensorMount;
};

} /* namespace ohm_tsd_slam_ref */
