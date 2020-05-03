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
#include "utilities.h"

/// todo whats TRACE kommt unten nochmal
//#define TRACE

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, const double xOffset, const double yOffset,
                                const std::string& nameSpace)
    : ThreadSLAM(*grid)
    , _nameSpace(nameSpace)
    , _nh(nh)
    , _mapper(*mapper)
    , _sensor(NULL)
    , /// todo nullptr?
    _initialized(false)
    , _gridWidth(grid->getCellsX() * grid->getCellSize())
    , _gridHeight(grid->getCellsY() * grid->getCellSize())
    , _gridOffSetX(-1.0 * (grid->getCellsX() * grid->getCellSize() * 0.5 + xOffset))
    , _gridOffSetY(-1.0 * (grid->getCellsY() * grid->getCellSize() * 0.5 + yOffset))
    , _xOffset(xOffset)
    , _yOffset(yOffset)
    , _offsetInitial(Eigen::Vector2d(xOffset, yOffset))
    , _stampLaser(ros::Time::now())
{
  std::string topicPose;
  std::string topicPoseCov;
  int         iVar = 0;

  std::string topicLaser;
  std::string topicStartStopSLAM;

  // std::string::iterator it = _nameSpace.end() - 1; // stores last symbol of nameSpace
  // if(*it != '/' && _nameSpace.size() > 0)
  //   _nameSpace += "/";

  ros::NodeHandle prvNh("~");

  

  prvNh.param(_nameSpace + "topic_pose", topicPose, std::string("default_ns/pose"));
  prvNh.param<std::string>(_nameSpace + "topic_pose_cov", topicPoseCov, "default_ns/pose_cov");
  prvNh.param("tf_base_frame", _tfBaseFrameId, std::string("/map"));
  prvNh.param(_nameSpace + "tf_child_frame", _tfChildFrameId, std::string("default_ns/laser"));
  prvNh.param<double>("laser_min_range", _lasMinRange, 0.0); // TODO: this parameter applied in the push of obviously already
  prvNh.param<int>(_nameSpace + "registration_mode", iVar, 4);

  prvNh.param<double>(_nameSpace + "thresh_min_pose_change_lin", _threshMinPoseChangeLin, 0.05); // TODO: this should not be a launch file parameter
  prvNh.param<double>(_nameSpace + "thresh_min_pose_change_ang", _threshMinPoseChangeAng, 0.03); // TODO: this should not be a launch file parameter

  prvNh.param<std::string>(_nameSpace + "topic_laser", topicLaser, _nameSpace + "/scan");
  prvNh.param<std::string>(_nameSpace + "topic_start_stop_slam", topicStartStopSLAM, _nameSpace + "/start_stop_slam");

  prvNh.param<double>(_nameSpace + "cov_matched", _covMatched, 1e-17);
  prvNh.param<double>(_nameSpace + "cov_error", _covError, 100.0);

  _subsLaser     = _nh->subscribe(topicLaser, 1, &ThreadLocalize::callBackLaser, this);
  _startStopSLAM = _nh->advertiseService(topicStartStopSLAM, &ThreadLocalize::callBackStartStopSLAM, this);

  _modelCoords  = NULL;
  _modelNormals = NULL;
  _maskM        = NULL;

  //_scene     = NULL;
  _maskS     = NULL;
  _lastPose  = new obvious::Matrix(3, 3);
  _rayCaster = new obvious::RayCastPolar2D();

  _posePub                     = _nh->advertise<geometry_msgs::PoseStamped>(topicPose, 1);
  _poseStamped.header.frame_id = _tfBaseFrameId;
  _tf.frame_id_                = _tfBaseFrameId;
  _tf.child_frame_id_          = _nameSpace + _tfChildFrameId;

  _pubPoseStCov                   = _nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(topicPoseCov, 1);
  _poseStampedCov.header.frame_id = _tfBaseFrameId;

  _reverseScan = false;
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
  //std::reverse(scanCopy->ranges.begin(), scanCopy->ranges.end());
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
    std::cout << __PRETTY_FUNCTION__ << " stored toppic is " << _subsLaser.getTopic() << std::endl;
    _subsLaser  = _nh->subscribe(_subsLaser.getTopic(), 1, &ThreadLocalize::callBackLaser, this);
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
    unsigned int validModelPoints = _rayCaster->calcCoordsFromCurrentViewMask(&_grid, _sensor, _modelCoords, _modelNormals, _maskM);
    if(validModelPoints == 0)
    {
      ROS_ERROR_STREAM("Localizer (" << _nameSpace << ") error! Raycasting found no coordinates! \n");
      continue;
    }

    // get current scan
    const unsigned int validScenePoints = _sensor->dataToCartesianVectorMask(_coordsScene, _maskS);

    obvious::Matrix T(3, 3);

    std::cout << __PRETTY_FUNCTION__ << " here? " << std::endl;
    const bool regErrorT = _registration->doRegistration(T, _modelCoords, _modelNormals, _maskM, validModelPoints, _coordsScene, _maskS);
    std::cout << __PRETTY_FUNCTION__ << " no" << std::endl;

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
      if(utilities::isPoseChangeSignificant(_lastPose, &curPose, _threshMinPoseChangeLin, _threshMinPoseChangeAng))
      {
        *_lastPose = curPose;
        _mapper.queuePush(_sensor);
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
  tinyxml2::XMLError loadState = doc.LoadFile(fileConfigRobot.c_str());
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
  // if(rootNode->Value() != )  //toDo: put the namespace to this check wether the opened xml contains the correct root node
  tinyxml2::XMLElement* element = nullptr;
  element                       = rootNode->FirstChildElement("local_offset");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error malformed config. local_offset is missing");
    throw "Invalid robot config file";
  }
  //utilities::loadTyniXmlParameter(localXoffset, "x", element);

  tinyxml2::XMLError result = element->QueryDoubleAttribute("x", &localXoffset);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. local offset x loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " local x offset " << localXoffset);

  result = element->QueryDoubleAttribute("y", &localYoffset);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. local offset y loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " local y offset " << localYoffset);

  result = element->QueryDoubleAttribute("yaw", &localYawOffset);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. local offset yaw loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " local yaw offset " << localYawOffset);

  element = nullptr;
  element = rootNode->FirstChildElement("range_thresh");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error malformed config. range thresh is missing");
    throw "Invalid robot config file";
  }

  result = element->QueryDoubleAttribute("max", &maxRange);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. Max range loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " maxrange " << maxRange);

  result = element->QueryDoubleAttribute("min", &minRange);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. min range loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " min range " << minRange);

  result = element->QueryDoubleAttribute("low_reflectivity", &lowReflectivityRange);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. Low_reflectivity range loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " low reflceljdf  " << lowReflectivityRange);
  

  element = nullptr;
  element = rootNode->FirstChildElement("footprint");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error malformed config. Footprint is missing");
    throw "Invalid robot config file";
  }
  result = element->QueryDoubleAttribute("width", &footPrintWidth);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. Footprint width loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " footprint width  " << footPrintWidth);

  result = element->QueryDoubleAttribute("height", &footPrintHeight);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. Footprint height loading failed" << std::endl;
    throw "Invalid robot config file";
  }
else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " footprintheight " << footPrintHeight);

  result = element->QueryDoubleAttribute("x_offset", &footPrintXoffset);
  if(result != tinyxml2::XML_SUCCESS)
  {
    std::cout << __PRETTY_FUNCTION__ << " error malformed config. Footprint x offset loading failed" << std::endl;
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " footprint x offset " << footPrintXoffset);

  element = nullptr;
  element = rootNode->FirstChildElement("tf_frame");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error malformed config. Tf_frame is missing");
    throw "Invalid robot config file";
  }
  
  const char* var = element->Attribute("val");
  if(!var)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " hallali");
    throw "Invalid robot config file";
  }
  std::string tfFrame(var);
  std::cout << __PRETTY_FUNCTION__ << " frame is " << tfFrame << std::endl;

  std::cout << localXoffset << " " << localYoffset << " " << localYawOffset << " " << maxRange << " " << minRange << " " << lowReflectivityRange << " "
            << footPrintWidth << " " << footPrintHeight << " " << footPrintXoffset << std::endl;

  // prvNh.param<double>(_nameSpace + "local_offset_x", localXoffset, 0.0);
  // prvNh.param<double>(_nameSpace + "local_offset_y", localYoffset, 0.0);
  // prvNh.param<double>(_nameSpace + "local_offset_yaw", localYawOffset, 0.0);
  // prvNh.param<double>(_nameSpace + "max_range", maxRange, 30.0);
  // prvNh.param<double>(_nameSpace + "min_range", minRange, 0.001);
  // prvNh.param<double>(_nameSpace + "low_reflectivity_range", lowReflectivityRange, 2.0);
  // prvNh.param<double>(_nameSpace + "footprint_width", footPrintWidth, 1.0);
  // prvNh.param<double>(_nameSpace + "footprint_height", footPrintHeight, 1.0);
  // prvNh.param<double>(_nameSpace + "footprint_x_offset", footPrintXoffset, 0.28);

  std::cout << localXoffset << " " << localYoffset << " " << localYawOffset << " " << maxRange << " " << minRange << " " << lowReflectivityRange << " "
            << footPrintWidth << " " << footPrintHeight << " " << footPrintXoffset << std::endl;

  const double phi    = localYawOffset;
  const double startX = _gridWidth * 0.5 + _xOffset + localXoffset;
  const double startY = _gridHeight * 0.5 + _yOffset + localYoffset;
  double       tf[9]  = {std::cos(phi), -std::sin(phi), startX, std::sin(phi), std::cos(phi), startY, 0, 0, 1};

  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);

  double        inc       = scan.angle_increment;
  double        angle_min = scan.angle_min;
  vector<float> ranges    = scan.ranges;
  std::cout << __PRETTY_FUNCTION__ << " min inc " << angle_min << " " << inc << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " angle min + size * incr = " << angle_min + static_cast<double>(scan.ranges.size() - 1) * inc << std::endl;
  std::cout << __PRETTY_FUNCTION__ << " angle max " << scan.angle_max << std::endl;



  if(scan.angle_increment < 0.0 && scan.angle_min > 0)
  {
    std::cout << __PRETTY_FUNCTION__ << " reverse this shit" << std::endl;
    _reverseScan = true;
    inc          = -inc;
    angle_min    = -angle_min;
    std::reverse(ranges.begin(), ranges.end());
  }
  _sensor = new obvious::SensorPolar2D(ranges.size(), inc, angle_min, maxRange, minRange, lowReflectivityRange);
  _sensor->setRealMeasurementData(ranges, 1.0);

  _sensor->setStandardMask();
  _sensor->transform(&Tinit);
  obfloat t[2] = {startX + footPrintXoffset, startY};
  if(!_grid.freeFootprint(t, footPrintWidth, footPrintHeight))
    ROS_ERROR_STREAM("Localizer (" << _nameSpace << ") warning! Footprint could not be freed! \n");
  if(!_mapper.initialized())
    _mapper.initPush(_sensor);
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

  if(_tfListener.waitForTransform(scan.header.frame_id, tfFrame, ros::Time::now(), ros::Duration(3.0)))
  {
    try
    {
      _tfListener.lookupTransform(scan.header.frame_id, tfFrame, ros::Time(0), _tfFrameSensorMount);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error looking up static transorm " << ex.what() << " The node will use the sensor frame.");
      _tfFrameSensorMount.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    }
    _poseStamped.header.frame_id = tfFrame;
  }
  else
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Time out waiting for static transform from " << scan.header.frame_id << " to " << tfFrame);
    _tfFrameSensorMount.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    _poseStamped.header.frame_id = scan.header.frame_id;
  }

  this->unblock(); // Method from ThreadSLAM to set a thread from sleep mode to run mode
}

void ThreadLocalize::sendTransform(obvious::Matrix* T)
{
  const double curTheta = utilities::calcAngle(T);
  const double posX     = (*T)(0, 2) + _gridOffSetX;
  const double posY     = (*T)(1, 2) + _gridOffSetY;
  _tfFrameSensorMount.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, curTheta);
  _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
  _tf.setRotation(quat);
  _tf.child_frame_id_ = _tfFrameSensorMount.child_frame_id_;
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
  //  _tf.stamp_ = ros::Time::now();

  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);

  _poseStampedCov.header.stamp = _stampLaser;
  _poseStampedCov.pose.pose = _poseStamped.pose;
  _poseStampedCov.pose.covariance = {_covMatched,         0.0,         0.0,         0.0,         0.0,          0.0,
                                             0.0, _covMatched,         0.0,         0.0,         0.0,          0.0,
                                             0.0,         0.0, _covMatched,         0.0,         0.0,          0.0,
                                             0.0,         0.0,         0.0, _covMatched,         0.0,          0.0,
                                             0.0,         0.0,         0.0,         0.0, _covMatched,          0.0,
                                             0.0,         0.0,         0.0,         0.0,         0.0,  _covMatched};
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
  _poseStampedCov.pose.pose = _poseStamped.pose;

  _poseStampedCov.pose.covariance = {_covError,         0.0,         0.0,         0.0,         0.0,          0.0,
                                             0.0, _covError,         0.0,         0.0,         0.0,          0.0,
                                             0.0,         0.0, _covError,         0.0,         0.0,          0.0,
                                             0.0,         0.0,         0.0, _covError,         0.0,          0.0,
                                             0.0,         0.0,         0.0,         0.0, _covError,          0.0,
                                             0.0,         0.0,         0.0,         0.0,         0.0,  _covError};
 _pubPoseStCov.publish(_poseStampedCov);
}

} // namespace ohm_tsd_slam
