/*
 * ThreadLocalize.cpp
 *
 *  Created on: Nov 6, 2018
 *      Refactured by: jasmin
 */

#include "ThreadLocalize.h"
#include "SlamNode.h"
#include "ThreadMapping.h"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <limits>
#include <memory>
#include <obcore/math/linalg/linalg.h>
#include <obcore/base/Logger.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <boost/bind.hpp>
#include <cstring>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unistd.h>

///todo whats TRACE kommt unten nochmal
//#define TRACE

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, const std::shared_ptr<rclcpp::Node>& node, 
                               const std::string& robot_name, const double xOffset, const double yOffset)
  : ThreadSLAM(*grid),
		_node(node),
		_mapper(*mapper),
		_sensor(nullptr),
		_initialized(false),
		_gridWidth(grid->getCellsX() * grid->getCellSize()),
		_gridHeight(grid->getCellsY() * grid->getCellSize()),
		_gridOffSetX(-(grid->getCellsX() * grid->getCellSize() * 0.5 + xOffset)),
		_gridOffSetY(-(grid->getCellsY() * grid->getCellSize() * 0.5 + yOffset)),
		_xOffset(xOffset),
		_yOffset(yOffset),
		_robotName(robot_name),
		_stampLaser(node->get_clock()->now()),
    _waitForOdomTf(rclcpp::Duration::from_seconds(1.0))
{
  // ThreadLocalize* threadLocalize = NULL;		///todo wofür hab ich das hier eig. variable wird nicht benutzt?

  double distFilterMax  		= 0.0;
  double distFilterMin			= 0.0;
  int icpIterations				= 0;
  double durationWaitForOdom	= 0.0;
  ///RandomMatcher options
  int trials					= 0;
  int sizeControlSet			= 0;
  double epsThresh				= 0.0;
  double zhit					= 0.0;
  double zphi					= 0.0;
  double zshort					= 0.0;
  double zmax					= 0.0;
  double zrand					= 0.0;
  double percentagePointsInC	= 0.0;
  double rangemax				= 0.0;
  double sigphi					= 0.0;
  double sighit					= 0.0;
  double lamshort				= 0.0;
  double maxAngleDiff			= 0.0;
  double maxAnglePenalty		= 0.0;
  int paramInt					= 0;
  int iVar 						= 0;

  /*** Read parameters from ros parameter server. Use namespace if provided. Multirobot use. ***/
  _robotName = robot_name;
  std::string::iterator it = _robotName.end() - 1;		//stores last symbol of nameSpace
  if(*it != '/' && _robotName.size()>0)
    _robotName += "/";
  const std::string node_name = std::string(node->get_name()) + "/";
  const std::string name_space = node_name + _robotName;
  _nameSpace = name_space;
  const std::string poseTopic = name_space + "estimated_pose";

  //ICP Options
  _node->declare_parameter<double>(_robotName + "dist_filter_max", DIST_FILT_MAX);
  _node->declare_parameter<double>(_robotName + "dist_filter_min", DIST_FILT_MIN);
  _node->declare_parameter<int>(_robotName + "icp_iterations", ICP_ITERATIONS);

  _node->declare_parameter<std::string>(_robotName + "tf_laser_frame", _robotName + "laser");
  _node->declare_parameter<std::string>(_robotName + "tf_odom_frame", _robotName + "odom");
  _node->declare_parameter<std::string>(_robotName + "tf_footprint_frame", _robotName + "base_footprint");

  try {
    // Only declare if not declared yet.
    // _node->declare_parameter<std::string>("tf_map_frame", "map")

    _node->declare_parameter<double>("reg_trs_max", TRNS_THRESH);
    _node->declare_parameter<double>("reg_sin_rot_max", ROT_THRESH);
    _node->declare_parameter<double>("max_velocity_lin", TRNS_VEL_MAX);
    _node->declare_parameter<double>("max_velocity_rot", ROT_VEL_MAX);
    _node->declare_parameter<bool>  ("ude_odom_rescue", false);
    _node->declare_parameter<double>("wait_for_odom_tf", 1.0);
    _node->declare_parameter<double>("laser_min_range", 0.0);
    _node->declare_parameter<int>   ("trials", 100);
    _node->declare_parameter<int>   ("sizeControlSet", 140);
    _node->declare_parameter<double>("epsThresh", 0.15);
    _node->declare_parameter<double>("zhit", 0.45);
    _node->declare_parameter<double>("zphi", 0);
    _node->declare_parameter<double>("zshort", 0.25);
    _node->declare_parameter<double>("zmax", 0.05);
    _node->declare_parameter<double>("zrand", 0.25);
    _node->declare_parameter<double>("percentagePointsInC", 0.9);
    _node->declare_parameter<double>("rangemax", 20);
    _node->declare_parameter<double>("sigphi", M_PI / 180.0 * 3);
    _node->declare_parameter<double>("sighit", 0.2);
    _node->declare_parameter<double>("lamshort", 0.08);
    _node->declare_parameter<double>("maxAngleDiff", 3.0);
    _node->declare_parameter<double>("maxAnglePenalty", 0.5);
  }
  catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& ex) {
    // All fine! In case of an multislam these paramters are declared multiple times.
  }

  _node->declare_parameter<int>(_robotName + "ransac_trials", RANSAC_TRIALS);
  _node->declare_parameter<double>(_robotName + "ransac_eps_thresh", RANSAC_EPS_THRESH);
  _node->declare_parameter<int>(_robotName + "ransac_ctrlset_size", RANSAC_CTRL_SET_SIZE);
  _node->declare_parameter<double>(_robotName + "ransac_phi_max", 30.0);
  _node->declare_parameter<int>(_robotName + "registration_mode", ICP);


  distFilterMax = _node->get_parameter(_robotName + "dist_filter_max").as_double();
  distFilterMin = _node->get_parameter(_robotName + "dist_filter_min").as_double();
  icpIterations = _node->get_parameter(_robotName + "icp_iterations").as_int();
  
  _tfLaserFrameId = _node->get_parameter(_robotName + "tf_laser_frame").as_string();
  _tfMapFrameId = _node->get_parameter("tf_map_frame").as_string();
  _tfOdomFrameId = _node->get_parameter(_robotName + "tf_odom_frame").as_string();
  _tfFootprintFrameId = _node->get_parameter(_robotName + "tf_footprint_frame").as_string();

  _trnsMax = _node->get_parameter("reg_trs_max").as_double();
  _rotMax = _node->get_parameter("reg_sin_rot_max").as_double();
  _trnsVelocityMax = _node->get_parameter("max_velocity_lin").as_double();
  _rotVelocityMax = _node->get_parameter("max_velocity_rot").as_double();
  _useOdomRescue = _node->get_parameter("ude_odom_rescue").as_bool();
  durationWaitForOdom = _node->get_parameter("wait_for_odom_tf").as_double();
  _lasMinRange = _node->get_parameter("laser_min_range").as_double();
  trials = _node->get_parameter("trials").as_int();
  sizeControlSet = _node->get_parameter("sizeControlSet").as_int();
  epsThresh = _node->get_parameter("epsThresh").as_double();
  zhit = _node->get_parameter("zhit").as_double();
  zphi = _node->get_parameter("zphi").as_double();
  zshort = _node->get_parameter("zshort").as_double();
  zmax = _node->get_parameter("zmax").as_double();
  zrand = _node->get_parameter("zrand").as_double();
  percentagePointsInC = _node->get_parameter("percentagePointsInC").as_double();
  rangemax = _node->get_parameter("rangemax").as_double();
  sigphi = _node->get_parameter("sigphi").as_double();
  sighit = _node->get_parameter("sighit").as_double();
  lamshort = _node->get_parameter("lamshort").as_double();
  maxAngleDiff = _node->get_parameter("maxAngleDiff").as_double();
  maxAnglePenalty = _node->get_parameter("maxAnglePenalty").as_double();

  //ransac options
  paramInt = _node->get_parameter(_robotName + "ransac_trials").as_int();
  _ranTrials = static_cast<unsigned int>(paramInt);

  _ranEpsThresh = _node->get_parameter(_robotName + "ransac_eps_thresh").as_double(); 
  paramInt = _node->get_parameter(_robotName + "ransac_ctrlset_size").as_int();
  _ranSizeCtrlSet = static_cast<unsigned int>(paramInt);

  _ranPhiMax = _node->get_parameter(_robotName + "ransac_phi_max").as_double();
  iVar = _node->get_parameter(_robotName + "registration_mode").as_int();
  _regMode = static_cast<EnumRegModes>(iVar);

  //Align laserscans
  switch(_regMode)
  {
  case ICP:
    //no instance needed
    break;
  case EXP:
    _RandomNormalMatcher 	= new obvious::RandomNormalMatching(trials, epsThresh, sizeControlSet);
    break;
  case PDF:
    _PDFMatcher 			= new obvious::PDFMatching(trials, epsThresh, sizeControlSet, zhit, zphi, zshort, zmax, zrand, percentagePointsInC, rangemax,
        sigphi, sighit, lamshort, maxAngleDiff, maxAnglePenalty);
    break;
  case TSD:
    _TSD_PDFMatcher 		= new obvious::TSD_PDFMatching(_grid, trials, epsThresh, sizeControlSet, zrand);
    break;
  default:
    RCLCPP_ERROR_STREAM(_node->get_logger(), "Unknown registration mode " << _regMode << " use default = ICP.");
  }

  //_odomAnalyzer 		= NULL;			///todo nullptr? unten auch?
  _tfBroadcaster    = std::make_unique<tf2_ros::TransformBroadcaster>(*_node);
  _tf_buffer = std::make_unique<tf2_ros::Buffer>(_node->get_clock());
  _tf_transform_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);
  _waitForOdomTf 		= rclcpp::Duration::from_seconds(durationWaitForOdom);
  _odomTfIsValid 		= false;
  _regMode 				  = static_cast<EnumRegModes>(iVar);
  _modelCoords			= nullptr;
  _modelNormals			= nullptr;
  _maskM				    = nullptr;
  _rayCaster			  = nullptr;			///todo wofür wird der genullt? wird gleich hier unten initialisiert
  _scene				    = nullptr;
  _maskS				    = nullptr;
  _lastPose				  = new obvious::Matrix(3, 3);
  _rayCaster			  = new obvious::RayCastPolar2D();
  _assigner				  = new obvious::FlannPairAssignment(2);
  _filterDist			  = new obvious::DistanceFilter(distFilterMax, distFilterMin, icpIterations - 10);
  _filterReciprocal	= new obvious::ReciprocalFilter();
  _estimator 			  = new obvious::ClosedFormEstimator2D();


  //configure ICP
  _filterBounds = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(icpIterations);
  _icp->setConvergenceCounter(icpIterations);

  _posePub = _node->create_publisher<geometry_msgs::msg::PoseStamped>(poseTopic, rclcpp::QoS(1).reliable()); // TODO: clarify OoS that should be used here.
  _poseStamped.header.frame_id	= _tfMapFrameId;
  _tf.header.frame_id	= _tfMapFrameId;
  _tf.child_frame_id = _robotName + _tfOdomFrameId;
  _reverseScan = false;

  //_odomAnalyzer = new OdometryAnalyzer(_grid);
}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _RandomNormalMatcher;
  delete _PDFMatcher;
  delete _TSD_PDFMatcher;

  _stayActive = false;
  _thread->join();
  _laserData.clear();
}

void ThreadLocalize::laserCallBack(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
  auto scanCopy = std::make_shared<sensor_msgs::msg::LaserScan>();
  scanCopy = scan;
  for(auto& iter : scanCopy->ranges)
  {
    if(iter < _lasMinRange)
      iter = 0.0;
  }
  if(!_initialized)
  {
    RCLCPP_INFO_STREAM(_node->get_logger(), "Localizer(" << _nameSpace << ") received first scan. Initialize node...");
    this->init(*scanCopy);
    RCLCPP_INFO_STREAM(_node->get_logger(), "Localizer(" << _nameSpace << ") initialized -> running...");

    //	    if(_useOdomRescue)
    //	        std::cout << __PRETTY_FUNCTION__ << "......................initialize OdometryAnalyzer.............. \n" << std::endl;
    //	    	_odomAnalyzer->odomRescueInit();
    _stampLaserOld = scan->header.stamp;
  }
  else
  {
    _dataMutex.lock();
    _laserData.push_front(scanCopy);
    _dataMutex.unlock();

    this->unblock();	//Method from ThreadSLAM to set a thread from sleep mode to run mode
  }
}

tf2::Transform ThreadLocalize::obviouslyMatrix3x3ToTf(obvious::Matrix& ob)
{
  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(ob(0,2), ob(1,2), 0.0));
  tf2::Quaternion rot;
  rot.setEuler(std::asin(ob(0,1)), 0.0, 0.0);
  tf.setRotation(rot);

  return tf;
}

obvious::Matrix ThreadLocalize::tfToObviouslyMatrix3x3(const tf2::Transform& tf)
{
  obvious::Matrix ob(3,3);
  ob.setIdentity();
  const auto q = tf.getRotation();
  // https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
  double theta = std::atan2(2.0*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  //sin() returns -0.0 -> avoid with +0.0
  ob(0,0 ) = cos(theta) + 0.0;
  ob(0,1 ) = -sin(theta) + 0.0;
  ob(0, 2) = x + 0.0;
  ob(1, 0) = sin(theta) + 0.0;
  ob(1, 1) = cos(theta) + 0.0;
  ob(1, 2) = y + 0.0;

  return ob;
}

void ThreadLocalize::eventLoop(void)
{
  _sleepCond.wait(_sleepMutex);
  while(_stayActive)
  {
    if(!_laserData.size())
    {
      _sleepCond.wait(_sleepMutex);
    }
    _dataMutex.lock();

    vector<float> ranges = _laserData.front()->ranges;
    if(_reverseScan)
      std::reverse(ranges.begin(),ranges.end());

    _stampLaserOld = _stampLaser;
    _stampLaser = _laserData.front()->header.stamp;

    _sensor->setRealMeasurementData(ranges);
    _sensor->setStandardMask();

    _laserData.clear();
    _dataMutex.unlock();

    //odom rescue
    //    if(_useOdomRescue)
    //    	_odomAnalyzer->odomRescueUpdate();

    //std::cout << __PRETTY_FUNCTION__ << "_odomTfIsValid = " << _odomTfIsValid << std::endl;

    const unsigned int measurementSize = _sensor->getRealMeasurementSize();

    if(!_scene)	//first call, initialize buffers
    {
      _scene		= new double[measurementSize * 2];
      _maskS		= new bool[measurementSize];
      _modelCoords	= new double[measurementSize * 2];
      _modelNormals	= new double[measurementSize * 2];
      _maskM		= new bool[measurementSize];
      *_lastPose	= _sensor->getTransformation();
    }

    //reconstruction
    unsigned int validModelPoints = _rayCaster->calcCoordsFromCurrentViewMask(&_grid, _sensor, _modelCoords, _modelNormals, _maskM);
    if(validModelPoints == 0)
    {
      RCLCPP_ERROR_STREAM(_node->get_logger(), "Localizer (" << _nameSpace << ") error! Raycasting found no coordinates!");
      continue;
    }

    //get current scan
    const unsigned int validScenePoints = _sensor->dataToCartesianVectorMask(_scene, _maskS);

    /**
     * Create Point Matrices with structure [x1 y1; x2 y2; ..]
     * M, N and S are matrices that preserve the ray model of a laser scanner
     * Xvalid matrices are matrices that do not preserve the ray model but contain only valid points (mask applied)
     * T transformation
     */
    obvious::Matrix M(measurementSize, 2, _modelCoords);
    obvious::Matrix N(measurementSize, 2, _modelNormals);
    obvious::Matrix S(measurementSize, 2, _scene);
    obvious::Matrix Mvalid = maskMatrix(&M, _maskM, measurementSize, validModelPoints);
    obvious::Matrix Nvalid = maskMatrix(&N, _maskM, measurementSize, validModelPoints);
    obvious::Matrix Svalid = maskMatrix(&S, _maskS, measurementSize, validScenePoints);
    obvious::Matrix T(3,3);

    T = doRegistration(_sensor, &M, &Mvalid, &N, &Nvalid, &S, &Svalid);

    //analyze registration result
    _tf.header.stamp = _node->get_clock()->now();
    const bool regErrorT = isRegistrationError(&T, _trnsMax, _rotMax);

    if(regErrorT)
    {
      RCLCPP_ERROR_STREAM(_node->get_logger(), "Localizer(" << _nameSpace << ") registration error!");
      sendNanTransform();
    }
    else 		//transformation valid -> transform sensor and publish new sensor pose
    {
//      obvious::Matrix Tbla(3, 3);
//        Tbla = this->tfToObviouslyMatrix3x3(_tfFrameSensorMount);
//        std::cout << __PRETTY_FUNCTION__ << "tbla" << std::endl;
//        Tbla.print();
//        std::cout << std::endl;
       //Tbla.invert();
       //T = Tbla * T;
      _sensor->transform(&T);
      //_sensor->transform(&Tbla);
      obvious::Matrix curPose = _sensor->getTransformation();
      sendTransform(&curPose);
      //update map if necessary
      if(this->isPoseChangeSignificant(_lastPose, &curPose))
      {
        *_lastPose = curPose;
        _mapper.queuePush(_sensor);
      }
    }
  }
}

void ThreadLocalize::init(const sensor_msgs::msg::LaserScan& scan)
{
  double localXoffset			= 0.0;
  double localYoffset			= 0.0;
  double localYawOffset			= 0.0;
  double maxRange				= 0.0;
  double minRange				= 0.0;
  double lowReflectivityRange	= 0.0;
  double footPrintWidth			= 0.0;
  double footPrintHeight		= 0.0;
  double footPrintXoffset		= 0.0;
  //std::string frameSensorMount;

  _node->declare_parameter<double>(_nameSpace + "local_offset_x", 0.0);
  _node->declare_parameter<double>(_nameSpace + "local_offset_y", 0.0);
  _node->declare_parameter<double>(_nameSpace + "local_offset_yaw", 0.0);
  _node->declare_parameter<double>(_nameSpace + "max_range", 30.0);
  _node->declare_parameter<double>(_nameSpace + "min_range", 0.001);
  _node->declare_parameter<double>(_nameSpace + "low_reflectivity_range", 2.0);
  _node->declare_parameter<double>(_nameSpace + "footprint_width", 1.0);
  _node->declare_parameter<double>(_nameSpace + "footprint_height", 1.0);
  _node->declare_parameter<double>(_nameSpace + "footprint_x_offset", 0.28);

  localXoffset = _node->get_parameter(_nameSpace + "local_offset_x").as_double();
  localYoffset = _node->get_parameter(_nameSpace + "local_offset_y").as_double();
  localYawOffset = _node->get_parameter(_nameSpace + "local_offset_yaw").as_double();
  maxRange = _node->get_parameter(_nameSpace + "max_range").as_double();
  minRange = _node->get_parameter(_nameSpace + "min_range").as_double();
  lowReflectivityRange = _node->get_parameter(_nameSpace + "low_reflectivity_range").as_double();
  footPrintWidth = _node->get_parameter(_nameSpace + "footprint_width").as_double();
  footPrintHeight = _node->get_parameter(_nameSpace + "footprint_height").as_double();
  footPrintXoffset = _node->get_parameter(_nameSpace + "footprint_x_offset").as_double();

  //prvNh.param<std::string>(_nameSpace + "frame_sensor_mount", frameSensorMount, "base_link");

//  if(!_tfListener.waitForTransform(scan.header.frame_id, frameSensorMount, ros::Time::now(), ros::Duration(0.5)))
//  {
//    std::cout << __PRETTY_FUNCTION__ << "looking up sensor mount frame failed " << std::endl;
//    _tfFrameSensorMount.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//    _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
//  }
//  else
//  {
//    try
//    {
//      _tfListener.lookupTransform(scan.header.frame_id, frameSensorMount, ros::Time(0), _tfFrameSensorMount);
//    }
//    catch(tf::TransformException& ex)
//    {
//      std::cout << __PRETTY_FUNCTION__ << "looking up sensor mount frame failed " << ex.what() << std::endl;
//      _tfFrameSensorMount.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//      _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
//    }
//  }

  const double phi		= localYawOffset;
  const double startX	= _gridWidth * 0.5 + _xOffset + localXoffset;
  const double startY	= _gridHeight * 0.5 + _yOffset + localYoffset;
  double tf[9] 			= {std::cos(phi), 	-std::sin(phi), 	startX,
                       std::sin(phi), 	 std::cos(phi),		startY,
                                   0,				         0,				1};
//  obvious::Matrix Tbla(3, 3);
//  Tbla = this->tfToObviouslyMatrix3x3(_tfFrameSensorMount);
  //Tbla.invert();
//  std::cout << __PRETTY_FUNCTION__ << " inverted\n";
//  Tbla.print();
  //std::cout << std::endl;



  obvious::Matrix Tinit(3,3);
  Tinit.setData(tf);
//  Tinit =  Tbla * Tinit;

  //std::cout << __PRETTY_FUNCTION__ << "Tinit = \n " << Tinit << std::endl;

  double inc 			= scan.angle_increment;
  double angle_min		= scan.angle_min;
  vector<float> ranges 	= scan.ranges;

  if(scan.angle_increment < 0.0 && scan.angle_min > 0)
  {
    _reverseScan 	= true;
    inc			= -inc;
    angle_min		= -angle_min;
    std::reverse(ranges.begin(), ranges.end());
  }
  _sensor = new obvious::SensorPolar2D(ranges.size(), inc, angle_min, maxRange, minRange, lowReflectivityRange);
  _sensor->setRealMeasurementData(ranges, 1.0);

  _sensor->setStandardMask();
  _sensor->transform(&Tinit);
  obfloat t[2] = {startX + footPrintXoffset, startY};
  if(!_grid.freeFootprint(t, footPrintWidth, footPrintHeight))
    RCLCPP_ERROR_STREAM(_node->get_logger(), "Localizer (" << _nameSpace << ") warning! Footprint could not be freed!");
  if(!_mapper.initialized())
    _mapper.initPush(_sensor);
  _initialized = true;
  //_sensor->transform()
  this->unblock();			//Method from ThreadSLAM to set a thread from sleep mode to run mode
}

obvious::Matrix ThreadLocalize::doRegistration(obvious::SensorPolar2D* sensor,
    obvious::Matrix* M,
    obvious::Matrix* Mvalid,
    obvious::Matrix* N,
    obvious::Matrix* Nvalid,
    obvious::Matrix* S,
    obvious::Matrix* Svalid)
{
  obvious::Matrix T44(4,4);
  T44.setIdentity();
  obvious::Matrix T(3,3);

  //wofür ist das?
#ifdef TRACE
  obvious::Matrix T_old(3,3);
#endif

  //RANSAC pre-registration (rough)
  switch(_regMode)
  {
  case ICP:
    //no pre-registration
    break;
  case EXP:
    //todo: check normals N for matching function (Daniel Ammon, Tobias Fink)
    T = _RandomNormalMatcher->match(M, _maskM, NULL, S, _maskS, obvious::deg2rad(_ranPhiMax), _trnsMax, sensor->getAngularResolution());
    T44(0, 0) = T(0, 0);
    T44(0, 1) = T(0, 1);
    T44(0, 3) = T(0, 2);
    T44(1, 0) = T(1, 0);
    T44(1, 1) = T(1, 1);
    T44(1, 3) = T(1, 2);
    break;
  case PDF:
    //todo: check normals N for matching function (Daniel Ammon, Tobias Fink)
    T = _PDFMatcher->match(M, _maskM, NULL, S, _maskS, obvious::deg2rad(_ranPhiMax), _trnsMax, sensor->getAngularResolution());
    T44(0, 0) = T(0, 0);
    T44(0, 1) = T(0, 1);
    T44(0, 3) = T(0, 2);
    T44(1, 0) = T(1, 0);
    T44(1, 1) = T(1, 1);
    T44(1, 3) = T(1, 2);
    break;
  case TSD:
    //todo: check normals N for matching function (Daniel Ammon, Tobias Fink
    T = _TSD_PDFMatcher->match(sensor->getTransformation(), M, _maskM, NULL, S, _maskS, obvious::deg2rad(_ranPhiMax), _trnsMax, sensor->getAngularResolution());
    T44(0, 0) = T(0, 0);
    T44(0, 1) = T(0, 1);
    T44(0, 3) = T(0, 2);
    T44(1, 0) = T(1, 0);
    T44(1, 1) = T(1, 1);
    T44(1, 3) = T(1, 2);
    break;
  default:
    //no pre-registration
    break;
  }

  _icp->reset();
  obvious::Matrix P = sensor->getTransformation();
  _filterBounds->setPose(&P);

  _icp->setModel(Mvalid, Nvalid);
  _icp->setScene(Svalid);
  double rms 			= 0.0;
  unsigned int pairs	= 0;
  unsigned int it		= 0;
  _icp->iterate(&rms, &pairs, &it, &T44);
  T = _icp->getFinalTransformation();

//  std::cout << __PRETTY_FUNCTION__ << " _useOdomRescue = " << _useOdomRescue << std::endl;
//  std::cout << __PRETTY_FUNCTION__ << " _odomTfIsValid = " << _odomTfIsValid << std::endl;
//  std::cout << "slam Transformation before odomRescueCheck: T_slam = \n" << T << std::endl;
  //if(_useOdomRescue && _odomTfIsValid)
  //  _odomAnalyzer->odomRescueCheck(T);
  //_odomAnalyzer->odomRescueCheck(T);

  return T;
}

bool ThreadLocalize::isRegistrationError(obvious::Matrix* T, const double trnsMax, const double rotMax)
{
  const double deltaX 		= (*T)(0, 2);
  const double deltaY 		= (*T)(1, 2);
  const double trnsAbs 		= std::sqrt(deltaX * deltaX + deltaY * deltaY);
  const double deltaPhi		= this->calcAngle(T);
  return (trnsAbs > trnsMax) || (std::abs(std::sin(deltaPhi)) > rotMax);
}

// static tf

void ThreadLocalize::sendTransform(obvious::Matrix* T)
{
  const double curTheta		= this->calcAngle(T);
  const double posX			= (*T)(0, 2) + _gridOffSetX;
  const double posY			= (*T)(1, 2) + _gridOffSetY;

  tf2::Quaternion orientation;
  orientation.setEuler(0.0, 0.0, curTheta);
  tf2::Transform pose;
  pose.setOrigin(tf2::Vector3(posX, posY, 0.0));
  pose.setRotation(orientation);
  _tf.child_frame_id = _tfOdomFrameId;
  _tf.header.frame_id = _tfMapFrameId;

  // Correction of laser to base_footprint.
  try {
    geometry_msgs::msg::TransformStamped tf_transform = _tf_buffer->lookupTransform(
      _tfLaserFrameId, _tfFootprintFrameId, tf2::TimePointZero
    );
    
    // Transform is available. Add to pose the transformation between laser and base_footprint.
    tf2::Transform t_laser_to_base_link;
    tf2::fromMsg(tf_transform.transform, t_laser_to_base_link);
    tf2::Transform t_map_base_link;
    // t_map_base_link.mult(pose, t_laser_to_base_link.inverse());
    t_map_base_link.mult(pose, t_laser_to_base_link);    
    pose = t_map_base_link;    
    // _tf.child_frame_id = _tfFootprintFrameId;
    // _tf.header.frame_id = _tfMapFrameId;
  }
  catch (const tf2::TransformException & ex) {
    // No transform available. Skip transforming into base_footprint.
    RCLCPP_INFO(_node->get_logger(), "No transfrom from laser to footprint available.");
    RCLCPP_INFO(_node->get_logger(), "Exception = %s", ex.what());

    // _tf.child_frame_id = _tfLaserFrameId;
    // return;
  }

  // Correction of odom.
  try {
    geometry_msgs::msg::TransformStamped tf_transform = _tf_buffer->lookupTransform(
      _tfFootprintFrameId, _tfOdomFrameId, tf2::TimePointZero
    );

    // Transform is available. Coorect odom frame by publishing map --> odom.
    tf2::Transform t_base_link_to_odom;
    tf2::fromMsg(tf_transform.transform, t_base_link_to_odom);
    tf2::Transform t_map_odom;
    t_map_odom.mult(pose, t_base_link_to_odom);
    pose = t_map_odom;
    _tf.child_frame_id = _tfOdomFrameId;
    _tf.header.frame_id = _tfMapFrameId;
    _tf.transform = tf2::toMsg(pose);        
  }
  catch (const tf2::TransformException & ex) {
    // No transform available. Skip transforming into base_footprint.
    RCLCPP_INFO(_node->get_logger(), "No transfrom from footprint to odom available.");
    RCLCPP_INFO(_node->get_logger(), "Exception = %s", ex.what());
    // return;
  }

  //_poseStamped.header.stamp = ros::Time::now();
  _poseStamped.header.stamp		= _stampLaser;
  _poseStamped.pose.position.x	= posX;
  _poseStamped.pose.position.y	= posY;
  _poseStamped.pose.position.z	= 0.0;

  tf2::Quaternion quat;
  quat.setEuler(0.0, 0.0, curTheta);
  _poseStamped.pose.orientation.w	= quat.w();
  _poseStamped.pose.orientation.x	= quat.x();
  _poseStamped.pose.orientation.y	= quat.y();
  _poseStamped.pose.orientation.z	= quat.z();
  //  _tf.stamp_ = ros::Time::now();

  // tf2::Transform transfrom;
  // transfrom.setRotation(quat);
  // transfrom.setOrigin(tf2::Vector3(posX, posY, 0.0));
  _tf.header.stamp = _stampLaser;
  // _tf.header.stamp = _node->get_clock()->now();


  _posePub->publish(_poseStamped);
  _tfBroadcaster->sendTransform(_tf);
}

void ThreadLocalize::sendNanTransform()
{
  _poseStamped.header.stamp 	 = _node->get_clock()->now();		//todo warum setzen wir hier ros time now und oben in sendtransform den stamplaser?
  _poseStamped.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  _poseStamped.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  _poseStamped.pose.position.z = std::numeric_limits<double>::quiet_NaN();

  tf2::Quaternion quat;
  quat.setEuler(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
  _poseStamped.pose.orientation.w	= quat.w();
  _poseStamped.pose.orientation.x	= quat.x();
  _poseStamped.pose.orientation.y	= quat.y();
  _poseStamped.pose.orientation.z	= quat.z();

  tf2::Transform transfrom;
  transfrom.setRotation(quat);
  transfrom.setOrigin(tf2::Vector3(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));
  _tf.header.stamp = _node->get_clock()->now(); //todo warum setzen wir hier ros time now und oben in sendtransform den stamplaser?
  _tf.transform = tf2::toMsg(transfrom);

  _posePub->publish(_poseStamped);
  _tfBroadcaster->sendTransform(_tf);
}

double ThreadLocalize::calcAngle(obvious::Matrix* T)
{
  double angle			= 0.0;
  const double ARCSIN	= asin((*T)(1, 0));
  const double ARCSINEG	= asin((*T)(0, 1));
  const double ARCOS	= acos((*T)(0, 0));
  if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
    angle = ARCOS;
  else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
    angle = 2.0 * M_PI - ARCOS;
  return(angle);
}

bool ThreadLocalize::isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose)
{
  const double deltaX		= (*curPose)(0, 2) - (*lastPose)(0, 2);
  const double deltaY		= (*curPose)(1, 2) - (*lastPose)(1, 2);
  double deltaPhi			= this->calcAngle(curPose) - this->calcAngle(lastPose);
  deltaPhi					= fabs(sin(deltaPhi));
  const double trnsAbs		= sqrt(deltaX * deltaX + deltaY * deltaY);
  return(deltaPhi > ROT_MIN || trnsAbs > TRNS_MIN);		//todo muss da keine klammer drum
}

obvious::Matrix ThreadLocalize::maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints)
{
  assert(Mat->getRows() == maskSize);
  assert(Mat->getCols() == 2);
  obvious::Matrix retMat(validPoints, 2);
  unsigned int cnt = 0;

  for(unsigned int i= 0; i < maskSize; i++)
  {
    if(mask[i])
    {
      retMat(cnt, 0) = (*Mat)(i, 0);
      retMat(cnt, 1) = (*Mat)(i, 1);
      cnt++;
    }
  }
  return retMat;
}

//todo maybe obsolete with pca matching, definitely not nice (author unknown)
void ThreadLocalize::reduceResolution(bool* const maskIn, obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
    const unsigned int pointsIn, const unsigned int pointsOut, const unsigned int reductionFactor)
{
  assert(pointsIn > pointsOut);
  //todo we only support scan with even number of points like 1080. if a scan has 1081 points is notl usable for subsampling here!
  const unsigned int factor = pointsIn / pointsOut;
  assert(factor == reductionFactor);

  unsigned int cnt = 0;
  for(unsigned int i = 0; i < pointsIn; i++)
  {
    if(!(i % factor))		//i % factor == 0
    {
      cnt++;
      if(maskIn[i])
      {
        maskOut[i/factor] = true;
        (*matOut)(i/factor, 0) = (*matIn)(i, 0);
        (*matOut)(i/factor, 1) = (*matIn)(i, 1);
      }
      else
      {
        maskOut[i/factor] = false;
      }
    }
  }
  assert(cnt == pointsOut);
}

} /* namespace ohm_tsd_slam_ref */
