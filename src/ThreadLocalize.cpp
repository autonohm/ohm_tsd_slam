/*
 * ThreadLocalize.cpp
 *
 *  Created on: Nov 6, 2018
 *      Refactured by: jasmin
 */

#include "ThreadLocalize.h"
#include "SlamNode.h"
#include "ThreadMapping.h"
#include "obcore/base/Logger.h"
#include "obcore/math/linalg/linalg.h"
#include "utilities.h"
#include <boost/bind.hpp>
#include <cstring>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <unistd.h>

/// todo whats TRACE kommt unten nochmal
//#define TRACE

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, const double xOffset, const double yOffset, const std::string& nameSpace)
    : ThreadSLAM(*grid)
    , _nameSpace(nameSpace)
    , _mapper(*mapper)
    , _sensor(nullptr)
    , _initialized(false)
    , _gridWidth(grid->getCellsX() * grid->getCellSize())
    , _gridHeight(grid->getCellsY() * grid->getCellSize())
    , _gridOffSetX(-1.0 * (grid->getCellsX() * grid->getCellSize() * 0.5 + xOffset))
    , _gridOffSetY(-1.0 * (grid->getCellsY() * grid->getCellSize() * 0.5 + yOffset))
    , _offsetInitial(Eigen::Vector2d(xOffset, yOffset))
    , _stampLaser(ros::Time::now())
    , _modelCoords(nullptr)
    , _modelNormals(nullptr)
    , _maskM(nullptr)
    , _maskS(nullptr)
    , _lastPose(std::make_unique<obvious::Matrix>(3, 3))
    , _rayCaster(std::make_unique<obvious::RayCastPolar2D>()),
    _reverseScan(false)
{
  std::string topicPose;
  std::string topicPoseCov;
  std::string topicLaser;
  std::string topicStartStopSLAM;

  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");

  prvNh.param(_nameSpace + "topic_pose", topicPose, std::string("default_ns/pose"));
  prvNh.param<std::string>(_nameSpace + "topic_pose_cov", topicPoseCov, "default_ns/pose_cov");
  prvNh.param("tf_base_frame", _tfBaseFrameId, std::string("/map"));
  prvNh.param<double>("laser_min_range", _lasMinRange, 0.0); 
  prvNh.param<double>(_nameSpace + "thresh_min_pose_change_lin", _threshMinPoseChangeLin, 0.05); // TODO: this should not be a launch file parameter
  prvNh.param<double>(_nameSpace + "thresh_min_pose_change_ang", _threshMinPoseChangeAng, 0.03); // TODO: this should not be a launch file parameter
  prvNh.param<std::string>(_nameSpace + "topic_laser", topicLaser, _nameSpace + "/scan");
  prvNh.param<std::string>(_nameSpace + "topic_start_stop_slam", topicStartStopSLAM, _nameSpace + "/start_stop_slam");

  prvNh.param<double>(_nameSpace + "cov_matched", _covMatched, 1e-17);
  prvNh.param<double>(_nameSpace + "cov_error", _covError, 100.0);

  _subsLaser     = nh.subscribe(topicLaser, 1, &ThreadLocalize::callBackLaser, this);
  _startStopSLAM = nh.advertiseService(topicStartStopSLAM, &ThreadLocalize::callBackStartStopSLAM, this);

  _posePub                     = nh.advertise<geometry_msgs::PoseStamped>(topicPose, 1);
  

  _pubPoseStCov                   = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topicPoseCov, 1);
  _poseStampedCov.header.frame_id = _tfBaseFrameId;
  _poseStamped.header.frame_id = _tfBaseFrameId;
  _tf.frame_id_                = _tfBaseFrameId;
  
}

ThreadLocalize::~ThreadLocalize()
{
  for(std::deque<sensor_msgs::LaserScan*>::iterator iter = _laserData.begin(); iter < _laserData.end(); iter++)
    delete *iter;
  _stayActive = false;
  _thread->join();
  _laserData.clear();
}

void ThreadLocalize::callBackLaser(const sensor_msgs::LaserScan& scan)
{
  sensor_msgs::LaserScan* scanCopy = new sensor_msgs::LaserScan;
  *scanCopy                        = scan;
  for(auto& iter : scanCopy->ranges)
  {
    if(iter < _lasMinRange)
      iter = 0.0;
    // if((std::isnan(iter)) || (std::isinf(iter)))
    //   iter = 0.0;
  }
  // std::reverse(scanCopy->ranges.begin(), scanCopy->ranges.end());
  //_reverseScan = true;
  if(!_initialized)
  {
    ROS_INFO_STREAM("Localizer(" << _nameSpace << ") received first scan. Initialize node...\n");
    this->init(*scanCopy);
    ROS_INFO_STREAM("Localizer(" << _nameSpace << ") initialized -> running...\n");

    //	    if(_useOdomRescue)
    //	        std::cout << __PRETTY_FUNCTION__ << "......................initialize OdometryAnalyzer.............. \n" << std::endl;
    //	    	_odomAnalyzer->odomRescueInit();

    //_stampLaserOld = scan.header.stamp;
  }
  else
  {
    _dataMutex.lock();
    _laserData.push_front(scanCopy);
    _dataMutex.unlock();

    this->unblock();
  }
}

bool ThreadLocalize::callBackStartStopSLAM(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if(req.data)
  {
    ros::NodeHandle nh;
    std::cout << __PRETTY_FUNCTION__ << " stored toppic is " << _subsLaser.getTopic() << std::endl;
    _subsLaser  = nh.subscribe(_subsLaser.getTopic(), 1, &ThreadLocalize::callBackLaser, this);
    res.message = "started topic " + _subsLaser.getTopic();
  }
  else
  {
    _subsLaser.shutdown();
    res.message = "stopped topic " + _subsLaser.getTopic();
  }
  res.success = true;
  return true;
}

void ThreadLocalize::eventLoop(void)
{
  std::cout << __PRETTY_FUNCTION__ << " loop started but no data yet, sleeping" << std::endl;
  _sleepCond.wait(_sleepMutex);
  std::cout << __PRETTY_FUNCTION__ << " first data element, start endless loop " << std::endl;
  while(_stayActive)
  {
    if(!_laserData.size())
    {
      _sleepCond.wait(_sleepMutex);
    }
    _dataMutex.lock();
    vector<float> ranges = _laserData.front()->ranges;
    if(_reverseScan)
      std::reverse(ranges.begin(), ranges.end());

    _stampLaser = _laserData.front()->header.stamp;

    _sensor->setRealMeasurementData(ranges);
    _sensor->setStandardMask();

    for(std::deque<sensor_msgs::LaserScan*>::iterator iter = _laserData.begin(); iter < _laserData.end(); iter++)
      delete *iter;
    _laserData.clear();
    _dataMutex.unlock();

    const unsigned int measurementSize = _sensor->getRealMeasurementSize();

    // reconstruction
    unsigned int validModelPoints = _rayCaster->calcCoordsFromCurrentViewMask(&_grid, _sensor.get(), _modelCoords, _modelNormals, _maskM);
    if(validModelPoints == 0)
    {
      ROS_ERROR_STREAM("Localizer (" << _nameSpace << ") error! Raycasting found no coordinates! \n");
      continue;
    }

    // get current scan
    const unsigned int validScenePoints = _sensor->dataToCartesianVectorMask(_coordsScene, _maskS);

    obvious::Matrix T(3, 3);

    //std::cout << __PRETTY_FUNCTION__ << " here? " << std::endl;
    const bool regErrorT = _registration->doRegistration(T, _modelCoords, _modelNormals, _maskM, validModelPoints, _coordsScene, _maskS);
    //std::cout << __PRETTY_FUNCTION__ << " no" << std::endl;

    _tf.stamp_ = ros::Time::now();

    if(regErrorT)
    {
      ROS_ERROR_STREAM("Localizer(" << _nameSpace << ") registration error! \n");
      sendNanTransform();
    }
    else // transformation valid -> transform sensor and publish new sensor pose
    {
      _sensor->transform(&T);
      obvious::Matrix curPose = _sensor->getTransformation();
      sendTransform(&curPose);
      // update map if necessary
      if(utilities::isPoseChangeSignificant(_lastPose.get(), &curPose, _threshMinPoseChangeLin, _threshMinPoseChangeAng))
      {
        *_lastPose = curPose;
        _mapper.queuePush(_sensor.get());
      }
    }
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  double localXoffset         = 0.0;
  double localYoffset         = 0.0;
  double localYawOffset       = 0.0;
  double maxRange             = 0.0;
  double minRange             = 0.0;
  double lowReflectivityRange = 0.0;
  double footPrintWidth       = 0.0;
  double footPrintHeight      = 0.0;
  double footPrintXoffset     = 0.0;
  // std::string frameSensorMount;

  ros::NodeHandle prvNh("~");
  std::string     fileConfigRobot;
  prvNh.param<std::string>(_nameSpace + "file_config_robot", fileConfigRobot, "/home/phil/workspace/ros/src/ohm_tsd_slam/config/config_robot_default.xml");

  tinyxml2::XMLDocument doc;
  tinyxml2::XMLError    loadState = doc.LoadFile(fileConfigRobot.c_str());
  if(loadState != tinyxml2::XML_SUCCESS)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " load config file " << fileConfigRobot << " failed with state " << loadState);
    throw "Invalid robot config file";
  }
  tinyxml2::XMLElement* rootNode = doc.RootElement();
  if(!rootNode)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " no node in xml file: " << fileConfigRobot.c_str());
    throw "Invalid robot config file";
  }
  const tinyxml2::XMLElement* element = nullptr;
  element                             = utilities::getTinyxmlChildElement(std::string("local_offset"), rootNode);
  utilities::loadTinyXmlAttribute(localXoffset, std::string("x"), *element);
  utilities::loadTinyXmlAttribute(localYoffset, std::string("y"), *element);
  utilities::loadTinyXmlAttribute(localYawOffset, std::string("yaw"), *element);

  element = utilities::getTinyxmlChildElement(std::string("range_thresh"), rootNode);
  utilities::loadTinyXmlAttribute(maxRange, std::string("max"), *element);
  utilities::loadTinyXmlAttribute(minRange, std::string("min"), *element);
  utilities::loadTinyXmlAttribute(lowReflectivityRange, std::string("low_reflectivity"), *element);

  element = utilities::getTinyxmlChildElement(std::string("footprint"), rootNode);
  utilities::loadTinyXmlAttribute(footPrintWidth, std::string("width"), *element);
  utilities::loadTinyXmlAttribute(footPrintHeight, std::string("height"), *element);
  utilities::loadTinyXmlAttribute(footPrintXoffset, std::string("x_offset"), *element);

  prvNh.param(_nameSpace + "tf_child_frame", _tfChildFrameId, std::string("default_ns/laser"));
  
  const double phi    = localYawOffset;
  const double startX = _gridWidth * 0.5 + _offsetInitial.x() + localXoffset;
  const double startY = _gridHeight * 0.5 + _offsetInitial.y() + localYoffset;
  double       tf[9]  = {std::cos(phi), -std::sin(phi), startX, std::sin(phi), std::cos(phi), startY, 0, 0, 1};

  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);

  double        inc       = scan.angle_increment;
  double        angle_min = scan.angle_min;
  vector<float> ranges    = scan.ranges;

  if(scan.angle_increment < 0.0 && scan.angle_min > 0)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " localizer automatically set to reverse mode ");
    _reverseScan = true;
    inc          = -inc;
    angle_min    = -angle_min;
    std::reverse(ranges.begin(), ranges.end());
  }
  _sensor = std::make_unique<obvious::SensorPolar2D>(ranges.size(), inc, angle_min, maxRange, minRange, lowReflectivityRange);
  _sensor->setRealMeasurementData(ranges, 1.0);

  _sensor->setStandardMask();
  _sensor->transform(&Tinit);
  obfloat t[2] = {startX + footPrintXoffset, startY};
  if(!_grid.freeFootprint(t, footPrintWidth, footPrintHeight))
    ROS_ERROR_STREAM("Localizer (" << _nameSpace << ") warning! Footprint could not be freed! \n");
  if(!_mapper.initialized())
    _mapper.initPush(_sensor.get());
  _initialized = true;
  //_sensor->transform()
  _registration                      = std::make_unique<Registration>(_grid, *_sensor); // TODO: registration mode
  const unsigned int measurementSize = _sensor->getRealMeasurementSize();
  _coordsScene                       = new double[measurementSize * 2];
  _maskS                             = new bool[measurementSize];
  _modelCoords                       = new double[measurementSize * 2];
  _modelNormals                      = new double[measurementSize * 2];
  _maskM                             = new bool[measurementSize];
  *_lastPose                         = _sensor->getTransformation();

  if(_tfListener.waitForTransform(scan.header.frame_id, _tfChildFrameId, ros::Time::now(), ros::Duration(3.0)))
  {
    try
    {
      std::cout << __PRETTY_FUNCTION__ << " looking up " << scan.header.frame_id << " " <<_tfChildFrameId << std::endl;
      _tfListener.lookupTransform(scan.header.frame_id, _tfChildFrameId, ros::Time(0), _tfFrameSensorMount);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error looking up static transform " << ex.what() << " The node will use the sensor frame.");
      _tfFrameSensorMount.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
      //_tfChildFrameId = scan.header.frame_id;
    }
    _tfFrameSensorMount.getOrigin().setZ(0.0);//   (tf::Vector3(_tfChildFrameId.  getOrigin().x(), _tfChildFrameId.getOrigin().y(), 0.0));
    _poseStamped.header.frame_id = _tfChildFrameId;
  }
  else
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Time out waiting for static transform from " << scan.header.frame_id << " to " << _tfChildFrameId);
    _tfFrameSensorMount.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    _tfChildFrameId = scan.header.frame_id;
  }
  std::cout << __PRETTY_FUNCTION__ << " tfchildframe " << _tfChildFrameId << std::endl;
  _tf.child_frame_id_ = _tfChildFrameId;
  
  this->unblock(); // Method from ThreadSLAM to set a thread from sleep mode to run mode
}

void ThreadLocalize::sendTransform(obvious::Matrix* T)
{
  const double curTheta = utilities::calcAngle(T);
  const double posX     = (*T)(0, 2) + _gridOffSetX;
  const double posY     = (*T)(1, 2) + _gridOffSetY;
  _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //toDO: this is here bc of problems with gazebo. Should be removed later

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, curTheta);
  _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
  _tf.setRotation(quat);
  
  _tf.setData(_tf * _tfFrameSensorMount);

  //_poseStamped.header.stamp = ros::Time::now();
  _poseStamped.header.stamp    = _stampLaser;
  _poseStamped.pose.position.x = _tf.getOrigin().x();
  _poseStamped.pose.position.y = _tf.getOrigin().y();
  _poseStamped.pose.position.z = 0.0;

  _poseStamped.pose.orientation.w = _tf.getRotation().w();
  _poseStamped.pose.orientation.x = _tf.getRotation().x();
  _poseStamped.pose.orientation.y = _tf.getRotation().y();
  _poseStamped.pose.orientation.z = _tf.getRotation().z();
    _tf.stamp_ = _stampLaser;

  _posePub.publish(_poseStamped);
  //std::cout << __PRETTY_FUNCTION__ << "real frame id " << _tf.child_frame_id_ << std::endl;
  _tfBroadcaster.sendTransform(_tf);

  _poseStampedCov.header.stamp    = _stampLaser;
  _poseStampedCov.pose.pose       = _poseStamped.pose;
  _poseStampedCov.pose.covariance = {_covMatched, 0.0, 0.0,         0.0, 0.0,         0.0, 0.0, _covMatched, 0.0, 0.0,         0.0, 0.0,
                                     0.0,         0.0, _covMatched, 0.0, 0.0,         0.0, 0.0, 0.0,         0.0, _covMatched, 0.0, 0.0,
                                     0.0,         0.0, 0.0,         0.0, _covMatched, 0.0, 0.0, 0.0,         0.0, 0.0,         0.0, _covMatched};
  _pubPoseStCov.publish(_poseStampedCov);
}

void ThreadLocalize::sendNanTransform()
{
  _poseStamped.header.stamp    = _stampLaser;
  _poseStamped.pose.position.x = NAN;
  _poseStamped.pose.position.y = NAN;
  _poseStamped.pose.position.z = NAN;
  tf::Quaternion quat;
  quat.setEuler(NAN, NAN, NAN);
  _poseStamped.pose.orientation.w = quat.w();
  _poseStamped.pose.orientation.x = quat.x();
  _poseStamped.pose.orientation.y = quat.y();
  _poseStamped.pose.orientation.z = quat.z();

  _tf.stamp_ = _stampLaser;
  _tf.setOrigin(tf::Vector3(NAN, NAN, NAN));
  _tf.setRotation(quat);

  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);
  _poseStampedCov.header.stamp = _stampLaser;
  _poseStampedCov.pose.pose    = _poseStamped.pose;

  _poseStampedCov.pose.covariance = {_covError, 0.0, 0.0,       0.0, 0.0,       0.0, 0.0, _covError, 0.0, 0.0,       0.0, 0.0,
                                     0.0,       0.0, _covError, 0.0, 0.0,       0.0, 0.0, 0.0,       0.0, _covError, 0.0, 0.0,
                                     0.0,       0.0, 0.0,       0.0, _covError, 0.0, 0.0, 0.0,       0.0, 0.0,       0.0, _covError};
  _pubPoseStCov.publish(_poseStampedCov);
}

} // namespace ohm_tsd_slam
