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

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace, const double xOffset,
                               const double yOffset)
    : ThreadSLAM(*grid)
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
    , _nameSpace(nameSpace)
    , _stampLaser(ros::Time::now())
{
  std::string poseTopic;
  std::string topicPoseStampedCov;
  int         iVar = 0;

  /*** Read parameters from ros parameter server. Use namespace if provided. Multirobot use. ***/
  _nameSpace               = nameSpace;
  std::string::iterator it = _nameSpace.end() - 1; // stores last symbol of nameSpace
  if(*it != '/' && _nameSpace.size() > 0)
    _nameSpace += "/";

  ros::NodeHandle prvNh("~");

  prvNh.param(_nameSpace + "pose_topic", poseTopic, std::string("default_ns/pose"));
  prvNh.param<std::string>(_nameSpace + "topic_pose_stamped_cov", topicPoseStampedCov, "default_ns/pose_stamped_cov");
  prvNh.param("tf_base_frame", _tfBaseFrameId, std::string("/map"));
  prvNh.param(_nameSpace + "tf_child_frame", _tfChildFrameId, std::string("default_ns/laser"));
  prvNh.param("tf_footprint_frame", _tfFootprintFrameId, std::string("base_footprint"));
  prvNh.param<double>("laser_min_range", _lasMinRange, 0.0);
  prvNh.param<int>(_nameSpace + "registration_mode", iVar, 4);
  prvNh.param<double>(_nameSpace + "thresh_min_pose_change_lin", _threshMinPoseChangeLin, 0.05);
  prvNh.param<double>(_nameSpace + "thresh_min_pose_change_ang", _threshMinPoseChangeAng, 0.03);
  double _threshMinPoseChangeLin;

  double _threshMinPoseChangeAng;
  _regMode = static_cast<RegModes>(iVar);

  _modelCoords  = NULL;
  _modelNormals = NULL;
  _maskM        = NULL;

  _scene     = NULL;
  _maskS     = NULL;
  _lastPose  = new obvious::Matrix(3, 3);
  _rayCaster = new obvious::RayCastPolar2D();

  _posePub                     = _nh->advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
  _poseStamped.header.frame_id = _tfBaseFrameId;
  _tf.frame_id_                = _tfBaseFrameId;
  _tf.child_frame_id_          = _nameSpace + _tfChildFrameId;

  _pubPoseStCov                   = _nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(topicPoseStampedCov, 1);
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

void ThreadLocalize::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  sensor_msgs::LaserScan* scanCopy = new sensor_msgs::LaserScan;
  *scanCopy                        = scan;
  for(auto& iter : scanCopy->ranges)
  {
    if(iter < _lasMinRange)
      iter = 0.0;
  }
  if(!_initialized)
  {
    ROS_INFO_STREAM("Localizer(" << _nameSpace << ") received first scan. Initialize node...\n");
    this->init(*scanCopy);
    ROS_INFO_STREAM("Localizer(" << _nameSpace << ") initialized -> running...\n");

    //	    if(_useOdomRescue)
    //	        std::cout << __PRETTY_FUNCTION__ << "......................initialize OdometryAnalyzer.............. \n" << std::endl;
    //	    	_odomAnalyzer->odomRescueInit();

    _stampLaserOld = scan.header.stamp;
  }
  else
  {
    _dataMutex.lock();
    _laserData.push_front(scanCopy);
    _dataMutex.unlock();

    this->unblock();
  }
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
      std::reverse(ranges.begin(), ranges.end());

    _stampLaserOld = _stampLaser;
    _stampLaser    = _laserData.front()->header.stamp;

    _sensor->setRealMeasurementData(ranges);
    _sensor->setStandardMask();

    for(std::deque<sensor_msgs::LaserScan*>::iterator iter = _laserData.begin(); iter < _laserData.end(); iter++)
      delete *iter;
    _laserData.clear();
    _dataMutex.unlock();

    const unsigned int measurementSize = _sensor->getRealMeasurementSize();

    if(!_scene) // first call, initialize buffers  //TODO: move to init
    {
      _scene        = new double[measurementSize * 2];
      _maskS        = new bool[measurementSize];
      _modelCoords  = new double[measurementSize * 2];
      _modelNormals = new double[measurementSize * 2];
      _maskM        = new bool[measurementSize];
      *_lastPose    = _sensor->getTransformation();
    }

    // reconstruction
    unsigned int validModelPoints = _rayCaster->calcCoordsFromCurrentViewMask(&_grid, _sensor, _modelCoords, _modelNormals, _maskM);
    if(validModelPoints == 0)
    {
      ROS_ERROR_STREAM("Localizer (" << _nameSpace << ") error! Raycasting found no coordinates! \n");
      continue;
    }

    // get current scan
    const unsigned int validScenePoints = _sensor->dataToCartesianVectorMask(_scene, _maskS);

    obvious::Matrix T(3, 3);

    const bool regErrorT = _registration->doRegistration(T, _modelCoords, _modelNormals, _maskM, validModelPoints, _scene, _maskS);

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

  prvNh.param<double>(_nameSpace + "local_offset_x", localXoffset, 0.0);
  prvNh.param<double>(_nameSpace + "local_offset_y", localYoffset, 0.0);
  prvNh.param<double>(_nameSpace + "local_offset_yaw", localYawOffset, 0.0);
  prvNh.param<double>(_nameSpace + "max_range", maxRange, 30.0);
  prvNh.param<double>(_nameSpace + "min_range", minRange, 0.001);
  prvNh.param<double>(_nameSpace + "low_reflectivity_range", lowReflectivityRange, 2.0);
  prvNh.param<double>(_nameSpace + "footprint_width", footPrintWidth, 1.0);
  prvNh.param<double>(_nameSpace + "footprint_height", footPrintHeight, 1.0);
  prvNh.param<double>(_nameSpace + "footprint_x_offset", footPrintXoffset, 0.28);

  const double phi    = localYawOffset;
  const double startX = _gridWidth * 0.5 + _xOffset + localXoffset;
  const double startY = _gridHeight * 0.5 + _yOffset + localYoffset;
  double       tf[9]  = {std::cos(phi), -std::sin(phi), startX, std::sin(phi), std::cos(phi), startY, 0, 0, 1};

  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);

  double        inc       = scan.angle_increment;
  double        angle_min = scan.angle_min;
  vector<float> ranges    = scan.ranges;

  if(scan.angle_increment < 0.0 && scan.angle_min > 0)
  {
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
  _registration = std::make_unique<Registration>(_regMode, _grid, *_sensor); // TODO: registration mode
  this->unblock();                                                           // Method from ThreadSLAM to set a thread from sleep mode to run mode
}

void ThreadLocalize::sendTransform(obvious::Matrix* T)
{
  const double curTheta = utilities::calcAngle(T);
  const double posX     = (*T)(0, 2) + _gridOffSetX;
  const double posY     = (*T)(1, 2) + _gridOffSetY;

  //_poseStamped.header.stamp = ros::Time::now();
  _poseStamped.header.stamp    = _stampLaser;
  _poseStamped.pose.position.x = posX;
  _poseStamped.pose.position.y = posY;
  _poseStamped.pose.position.z = 0.0;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, curTheta);
  _poseStamped.pose.orientation.w = quat.w();
  _poseStamped.pose.orientation.x = quat.x();
  _poseStamped.pose.orientation.y = quat.y();
  _poseStamped.pose.orientation.z = quat.z();
  //  _tf.stamp_ = ros::Time::now();
  _tf.stamp_ = _stampLaser;
  _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
  _tf.setRotation(quat);

  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);
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
}

} // namespace ohm_tsd_slam
