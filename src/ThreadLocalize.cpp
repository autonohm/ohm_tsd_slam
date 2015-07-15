/*
 * ThreadLocalize.cpp
 *
 *  Created on: 28.10.2014
 *      Author: phil
 */

#include "ThreadLocalize.h"

#include "SlamNode.h"
#include "ThreadMapping.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"
#include "obvision/registration/ransacMatching/RansacMatching.h"
#include "obvision/registration/ransacMatching/RandomNormalMatching.h"

#include <boost/bind.hpp>

#include <cstring>
#include <unistd.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
    const double xOffFactor, const double yOffFactor):
        ThreadSLAM(*grid),
        _nh(nh),
        _mapper(*mapper),
        _sensor(NULL),
        _newScan(false),
        _initialized(false),
        _nameSpace(nameSpace),
        _maskLaser(NULL),
        _gridOffSetX(-1.0 * grid->getCellsX() * grid->getCellSize() * xOffFactor),
        _gridOffSetY(-1.0 * grid->getCellsY()* grid->getCellSize() * yOffFactor),
        _scene(NULL),
        _modelCoords(NULL),
        _modelNormals(NULL),
        _maskM(NULL),
        _maskS(NULL),
        _gridWidth(grid->getCellsX() * grid->getCellSize()),
        _gridHeight(grid->getCellsY() * grid->getCellSize()),
        _xOffFactor(xOffFactor),
        _yOffFactor(yOffFactor)

{
  ros::NodeHandle prvNh("~");

  /*** Read parameters from ros parameter server. Use namespace if provided ***/
  _nameSpace = nameSpace;
  if(_nameSpace.size())   //given namespace
    _nameSpace += "/";

  //pose
  std::string poseTopic;
  prvNh.param(_nameSpace + "pose_topic", poseTopic, std::string("default_ns/pose"));
  poseTopic = _nameSpace + poseTopic;

  //frames
  std::string tfBaseFrameId;
  prvNh.param("tf_base_frame", tfBaseFrameId, std::string("/map"));

  std::string tfChildFrameId;
  prvNh.param(_nameSpace + "tf_child_frame", tfChildFrameId, std::string("default_ns/laser"));

  double distFilterMax = 0.0;
  double distFilterMin = 0.0;
  int icpIterations = 0;

  //reduce size for ransac
  int iVar = 0;
  prvNh.param<int>(_nameSpace + "ransac_reduce_factor", iVar , 1);
  _ransacReduceFactor = static_cast<unsigned int>(iVar);

  //ICP Options
  prvNh.param<double>(_nameSpace + "dist_filter_min", distFilterMin, 0.2);
  prvNh.param<double>(_nameSpace + "dist_filter_max", distFilterMax, 1.0);
  prvNh.param<int>(_nameSpace + "icp_iterations", icpIterations, 25);

  //Maximum allowed offset between to aligned scans
  prvNh.param<double>("reg_trs_max", _trnsMax, TRNS_THRESH);
  prvNh.param<double>("reg_sin_rot_max", _rotMax, ROT_THRESH);

  int paramInt = 0;
  prvNh.param<int>(nameSpace + "ransac_trials", paramInt, 50);
  _ranTrials = static_cast<unsigned int>(paramInt);
  prvNh.param<double>(nameSpace + "ransac_eps_thresh", _ranEpsThresh, 0.15);
  prvNh.param<int>(nameSpace + "ransac_ctrlset_size", paramInt, 180);
  _ranSizeCtrlSet = static_cast<unsigned int>(paramInt);

  iVar = 0;
  prvNh.param<int>(_nameSpace + "registration_mode", iVar, ICP);
  _regMode = static_cast<EnumRegModes>(iVar);

  prvNh.param<double>("depth_discontinuity_thresh", _depthDiscontinuityThresh, 3.0);

  /** Initialize member modules **/
  _lastPose         = new obvious::Matrix(3, 3);
  _rayCaster        = new obvious::RayCastPolar2D();
  _assigner         = new obvious::FlannPairAssignment(2);
  _filterDist       = new obvious::DistanceFilter(distFilterMax, distFilterMin, icpIterations - 10);
  _filterReciprocal = new obvious::ReciprocalFilter();
  _estimator        = new obvious::ClosedFormEstimator2D();

  //configure ICP
  _filterBounds     = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(icpIterations);
  _icp->setConvergenceCounter(icpIterations);


  _posePub = _nh->advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
  _poseStamped.header.frame_id = tfBaseFrameId;
  _tf.frame_id_                = tfBaseFrameId;
  _tf.child_frame_id_          = _nameSpace + tfChildFrameId;
}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _maskLaser;
}

void ThreadLocalize::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(_newScan)
    //if(!_dataMutex.try_lock())
    return;

  if(!_initialized)
  {
    ROS_INFO_STREAM("Localizer(" << _nameSpace << ") received first scan. Initialize node...\n");
    this->init(scan);
    ROS_INFO_STREAM("Localizer(" << _nameSpace << ") initialized -> running...\n");
    //_dataMutex.unlock();
    return;
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
    _maskLaser[i]=!isnan(scan.ranges[i]);
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_maskLaser);
  _sensor->maskDepthDiscontinuity(obvious::deg2rad(_depthDiscontinuityThresh));
  _sensor->maskZeroDepth();
  _newScan = true;
  //_dataMutex.unlock();
  this->unblock();

}

void ThreadLocalize::eventLoop(void)
{
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);
    //_dataMutex.lock();
    const unsigned int measurementSize = _sensor->getRealMeasurementSize();

    if(!_scene)   //first call, initialize buffers
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
      ROS_ERROR_STREAM("Localizer(" << _nameSpace <<") error! Raycasting found no coordinates!\n");
      return;
    }

    //get current scan
    unsigned int validScenePoints = 0;
    validScenePoints = _sensor->dataToCartesianVectorMask(_scene, _maskS);

    /**
     *  Create Point Matrixes with structure [x1 y1; x2 y2; ..]
     *  M, N, and S are matrices that preserve the ray model of a laser scanner
     *  Xvalid matrices are matrices that do not preserve the ray model but contain only valid points
     */
    obvious::Matrix M(measurementSize, 2, _modelCoords);
    obvious::Matrix N(measurementSize, 2, _modelNormals);
    obvious::Matrix Mvalid = maskMatrix(&M, _maskM, measurementSize, validModelPoints);

    unsigned int size = _sensor->dataToCartesianVector(_scene);
    obvious::Matrix S(measurementSize, 2, _scene);
    obvious::Matrix Svalid = maskMatrix(&S, _maskS, measurementSize, validScenePoints);

    obvious::Matrix T(3, 3);

    /** Align Laser scans */
    if(_regMode == ICP)
      T = doRegistration(_sensor, &M, &Mvalid, &N, NULL, &S, &Svalid, false);  //3x3 Transformation Matrix

    else if(_regMode == EXP)
      T = doRegistration(_sensor, &M, &Mvalid, &N, NULL, &S, &Svalid, true);  //3x3 Transformation Matrix

    /** analyze registration result */
    _tf.stamp_ = ros::Time::now();
    const bool regErrorT = isRegistrationError(&T, _trnsMax, _rotMax);

    if(regErrorT && _regMode == ICP_EXP_RSC) //rescue with ransac pre- registration
    {
      ROS_INFO_STREAM("Localizer(" << _nameSpace << ") registration error! Trying to recapture with" <<
                      "experimental registration approach...\n");
      obvious::Matrix secondT = doRegistration(_sensor, &M, &Mvalid, &N, NULL, &S, &Svalid, true);  //3x3 Transformation Matrix
      if(isRegistrationError(&secondT,_trnsMax * 1.5, _rotMax * 1.5)) //toDo: launch file parameters
      {
        ROS_ERROR_STREAM("Localizer(" << _nameSpace << ") could not recapture registration\n");
        sendNanTransform();
        continue;
      }
      else
      {
        ROS_INFO_STREAM("Localizer(" << _nameSpace << ") Recaptured!" << std::endl);
        _sensor->transform(&secondT);
        obvious::Matrix curPose = _sensor->getTransformation();
        sendTransform(&curPose);
        if(this->isPoseChangeSignificant(_lastPose, &curPose))
        {
          *_lastPose = curPose;
          _mapper.queuePush(_sensor);
        }
      }
    }
    else if(regErrorT)
    {
      ROS_ERROR_STREAM("Localizer(" << _nameSpace << ") registration error! \n");
      sendNanTransform();
      continue;
    }
    else //transformation valid -> transform sensor and publish new sensor pose
    {
      _sensor->transform(&T);
      obvious::Matrix curPose = _sensor->getTransformation();

      sendTransform(&curPose);
      /** Update MAP if necessary */
      if(this->isPoseChangeSignificant(_lastPose, &curPose))// && !_noPush)  toDo: integrate into release?
      {
        *_lastPose = curPose;
        _mapper.queuePush(_sensor);
      }
    }
    //_dataMutex.unlock();
    _newScan = false;
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  double xOffset   = 0.0;
  double yOffset= 0.0;
  double yawOffset= 0.0;
  double maxRange = 0.0;
  double minRange = 0.0;
  double lowReflectivityRange = 0.0;
  double footPrintWidth= 0.0;
  double footPrintHeight= 0.0;
  double footPrintXoffset= 0.0;

  ros::NodeHandle prvNh("~");

  prvNh.param<double>(_nameSpace + "/x_offset"              , xOffset             , 0.0);
  prvNh.param<double>(_nameSpace + "/y_offset"              , yOffset             , 0.0);
  prvNh.param<double>(_nameSpace + "/yaw_offset"            , yawOffset           , 0.0);
  prvNh.param<double>(_nameSpace + "/max_range"             , maxRange            , 30.0);
  prvNh.param<double>(_nameSpace + "/min_range"             , minRange            , 0.001);
  prvNh.param<double>(_nameSpace + "/low_reflectivity_range", lowReflectivityRange, 2.0);
  prvNh.param<double>(_nameSpace + "/footprint_width"       , footPrintWidth      , 0.0);
  prvNh.param<double>(_nameSpace + "/footprint_height"      , footPrintHeight     , 0.0);
  prvNh.param<double>(_nameSpace + "/footprint_x_offset"    , footPrintXoffset    , 0.28);

  double phi    = yawOffset;
  double startX = _gridWidth * _xOffFactor + xOffset;
  double startY = _gridWidth * _yOffFactor + yOffset;
  double tf[9]  = {std::cos(phi), -std::sin(phi), startX,
      std::sin(phi),  std::cos(phi), startY,
      0,              0,      1};
  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);

  _sensor = new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, maxRange, minRange, lowReflectivityRange);
  _sensor->setRealMeasurementData(scan.ranges, 1.0);

  _maskLaser = new bool[scan.ranges.size()];
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {
    _maskLaser[i]=!isnan(scan.ranges[i]);
  }
  _sensor->setRealMeasurementMask(_maskLaser);
  _sensor->transform(&Tinit);
  obfloat t[2] = {startX + footPrintXoffset, startY};
  if(!_grid.freeFootprint(t, footPrintWidth, footPrintHeight))
    ROS_ERROR_STREAM("Localizer(" << _nameSpace << ") warning! Footprint could not be freed!\n");
  if(!_mapper.initialized())
    _mapper.initPush(_sensor);
  _initialized = true;
}

obvious::Matrix ThreadLocalize::doRegistration(obvious::SensorPolar2D* sensor,
    obvious::Matrix* M,
    obvious::Matrix* Mvalid,
    obvious::Matrix* N,
    obvious::Matrix* Nvalid,
    obvious::Matrix* S,
    obvious::Matrix* Svalid,
    const bool experimental
)
{
  const unsigned int measurementSize = sensor->getRealMeasurementSize();
  obvious::Matrix T44(4, 4);
  T44.setIdentity();

  // RANSAC pre-registration (rough)
  if(experimental)
  {
    const unsigned int factor = _ransacReduceFactor;
    const unsigned int reducedSize = measurementSize / factor; // e.g.: 1080 -> 270
    obvious::Matrix Sreduced(reducedSize, 2);
    obvious::Matrix Mreduced(reducedSize, 2);
    bool* maskSRed = new bool[reducedSize];
    bool* maskMRed = new bool[reducedSize];
    if( factor != 1) {
      reduceResolution(_maskS, S, maskSRed, &Sreduced, measurementSize, reducedSize, factor);
      reduceResolution(_maskM, M, maskMRed, &Mreduced, measurementSize, reducedSize, factor);
    }

    //RansacMatching ransac(_ranTrials, _ranEpsThresh, _ranSizeCtrlSet);
    obvious::RandomNormalMatching ransac(_ranTrials, _ranEpsThresh, _ranSizeCtrlSet);
    const double phiMax = _rotMax;
    obvious::Matrix T(3, 3);
    //obvious::Matrix T = ransac.match(&M, _maskM, &N, &S, _maskS, phiMax, _trnsMax, sensor->getAngularResolution());
    if(factor == 1)
      T = ransac.match(M, _maskM, N, S, _maskS, phiMax, _trnsMax, sensor->getAngularResolution());
    else
      T = ransac.match(&Mreduced, maskMRed, N, &Sreduced, maskSRed, phiMax,
          _trnsMax, sensor->getAngularResolution() * (double) factor);

    T.invert();
    T44(0, 0) = T(0, 0);
    T44(0, 1) = T(0, 1);
    T44(0, 3) = T(0, 2);
    T44(1, 0) = T(1, 0);
    T44(1, 1) = T(1, 1);
    T44(1, 3) = T(1, 2);
  }

  _icp->reset();
  obvious::Matrix P = sensor->getTransformation();
  _filterBounds->setPose(&P);
  _icp->setModel(Svalid, NULL);
  _icp->setScene(Mvalid);
  double rms = 0.0;
  unsigned int pairs = 0;
  unsigned int it = 0;
  _icp->iterate(&rms, &pairs, &it, &T44);
  obvious::Matrix T = _icp->getFinalTransformation();
  T.invert();
  return T;
}

bool ThreadLocalize::isRegistrationError(obvious::Matrix* T, const double trnsMax, const double rotMax)
{
  double deltaX = (*T)(0, 2);
  double deltaY = (*T)(1, 2);
  double trnsAbs = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  double deltaPhi = this->calcAngle(T);

  return (trnsAbs > trnsMax) || (std::abs(std::sin(deltaPhi)) > rotMax);
}

void ThreadLocalize::sendTransform(obvious::Matrix* T)
{
  double curTheta = this->calcAngle(T);
  double posX = (*T)(0, 2) + _gridOffSetX; //Fixme this is not the right place for this
  double posY = (*T)(1, 2) + _gridOffSetY;
  _poseStamped.header.stamp = ros::Time::now();
  _poseStamped.pose.position.x = posX;
  _poseStamped.pose.position.y = posY;
  _poseStamped.pose.position.z = 0.0;
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, curTheta);
  _poseStamped.pose.orientation.w = quat.w();
  _poseStamped.pose.orientation.x = quat.x();
  _poseStamped.pose.orientation.y = quat.y();
  _poseStamped.pose.orientation.z = quat.z();

  _tf.stamp_ = ros::Time::now();
  _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
  _tf.setRotation(quat);

  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);
}

void ThreadLocalize::sendNanTransform()
{
  _poseStamped.header.stamp = ros::Time::now();
  _poseStamped.pose.position.x = NAN;
  _poseStamped.pose.position.y = NAN;
  _poseStamped.pose.position.z = NAN;
  tf::Quaternion quat;
  quat.setEuler(NAN, NAN, NAN);
  _poseStamped.pose.orientation.w = quat.w();
  _poseStamped.pose.orientation.x = quat.x();
  _poseStamped.pose.orientation.y = quat.y();
  _poseStamped.pose.orientation.z = quat.z();

  _tf.stamp_ = ros::Time::now();
  _tf.setOrigin(tf::Vector3(NAN, NAN, NAN));
  _tf.setRotation(quat);

  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);
}

double ThreadLocalize::calcAngle(obvious::Matrix* T)
{
  double angle          = 0.0;
  const double ARCSIN   = asin((*T)(1,0));
  const double ARCSINEG = asin((*T)(0,1));
  const double ARCOS    = acos((*T)(0,0));
  if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
    angle = ARCOS;
  else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
    angle = 2.0 * M_PI - ARCOS;
  return(angle);
}

bool ThreadLocalize::isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose)
{
  double deltaX   = (*curPose)(0, 2) - (*lastPose)(0, 2);
  double deltaY   = (*curPose)(1, 2) - (*lastPose)(1, 2);
  double deltaPhi = this->calcAngle(curPose) - this->calcAngle(lastPose);
  deltaPhi        = fabs(sin(deltaPhi));
  double trnsAbs  = sqrt(deltaX * deltaX + deltaY * deltaY);

  return (deltaPhi > ROT_MIN) || (trnsAbs > TRNS_MIN);
}

obvious::Matrix ThreadLocalize::maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints)
{
  assert(Mat->getRows() == maskSize);
  assert(Mat->getCols() == 2);
  obvious::Matrix retMat(validPoints, 2);
  unsigned int cnt = 0;
  for(int i = 0; i < maskSize; i++)
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

void ThreadLocalize::reduceResolution(bool* const maskIn, const obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
    const unsigned int pointsIn, const unsigned int pointsOut, const unsigned int reductionFactor)
{
  assert(pointsIn > pointsOut);
  //fixme we only support scan with even number of points like 1080. if a scan has 1081 points is not usable for subsampling here!
  const unsigned int factor = pointsIn / pointsOut;
  assert(factor == reductionFactor);

  unsigned int cnt = 0;
  for(unsigned int i = 0; i < pointsIn; i++)
  {
    if(!(i % factor)) // i % factor == 0
    {
      cnt++;
      if (maskIn[i]) {
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

} /* namespace ohm_tsd_slam */
