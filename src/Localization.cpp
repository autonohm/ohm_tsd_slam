#include "Localization.h"
#include "MultiSlamNode.h"
#include "ThreadMapping.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"
#include "obvision/registration/ransacMatching/RansacMatching.h"

#include <boost/bind.hpp>

#include <cstring>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

Localization::Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, const double xOffFactor, const double yOffFactor, 
    std::string nameSpace):
                _gridOffSetX(-1.0 * grid->getCellsX() * grid->getCellSize() * xOffFactor),
                _gridOffSetY(-1.0 * grid->getCellsY()* grid->getCellSize() * yOffFactor)
{
  _nh = &nh;
  ros::NodeHandle prvNh("~");
  // _pubMutex         = pubMutex;
  _mapper           = mapper;
  _grid             = grid;

  _scene            = NULL;
  _modelCoords      = NULL;
  _modelNormals     = NULL;
  _maskM = NULL;
  _maskS = NULL;

  /*** Read parameters from ros parameter server. Use namespace if provided ***/
  if(nameSpace.size())   //given namespace
    nameSpace += "/";

  //pose
  std::string poseTopic;
  prvNh.param(nameSpace + "pose_topic", poseTopic, std::string("default_ns/pose"));
  poseTopic = nameSpace + poseTopic;

  //base frame
  std::string tfBaseFrameId;
  prvNh.param("tf_base_frame", tfBaseFrameId, std::string("/map"));

  //child frame
  std::string tfChildFrameId;
  prvNh.param(nameSpace + "tf_child_frame", tfChildFrameId, std::string("default_ns/laser"));

  //use icpsac?
  prvNh.param<bool>(nameSpace + "use_icpsac", _ransac, false);

  //reduce size for ransac
  int iVar = 0;
  prvNh.param<int>(nameSpace + "ransac_reduce_factor", iVar, 1);
  _ransacReduceFactor = static_cast<unsigned int>(iVar);

  _noPush = false;   //start in slam mode (nopush = false)

  std::string togglePushServiceTopic;
  prvNh.param<std::string>(nameSpace + "toggle_push", togglePushServiceTopic, "toggle_push");
  _togglePushService = _nh->advertiseService(togglePushServiceTopic, &Localization::togglePushServiceCallBack, this);

  //ICP Options
  double distFilterMax = 0.0;
  double distFilterMin = 0.0;
  int icpIterations = 0;
  prvNh.param<double>(nameSpace + "dist_filter_min", distFilterMin, 0.2);
  prvNh.param<double>(nameSpace + "dist_filter_max", distFilterMax, 1.0);
  prvNh.param<int>(nameSpace+"icp_iterations", icpIterations, 25);

  //Maximum allowed offset between to aligned scans
  prvNh.param<double>("reg_trs_max", _trnsMax, TRNS_THRESH);
  prvNh.param<double>("reg_sin_rot_max", _rotMax, ROT_THRESH);

  /** Initialize member modules **/
  _lastPose         = new obvious::Matrix(3, 3);
  _xOffFactor       = xOffFactor;
  _yOffFactor       = yOffFactor;

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
  _tf.child_frame_id_          = nameSpace + tfChildFrameId;
}

Localization::~Localization()
{
  delete _rayCaster;
  delete _assigner;
  delete _filterBounds;
  delete _filterDist;
  delete _filterReciprocal;
  delete _estimator;
  delete _icp;
  delete _lastPose;
  if(_modelCoords)
    delete [] _modelCoords;
  if(_modelNormals)
    delete [] _modelNormals;
  if(_scene)
    delete [] _scene;
}

void Localization::localize(obvious::SensorPolar2D* sensor)
{
  const unsigned int measurementSize = sensor->getRealMeasurementSize();

  if(!_scene)
  {
    _scene 				= new double[measurementSize * 2];
    _maskS 				= new bool[measurementSize];
    _modelCoords 	= new double[measurementSize * 2];
    _modelNormals = new double[measurementSize * 2];
    _maskM 				= new bool[measurementSize];
    *_lastPose 		= sensor->getTransformation();
  }

  // reconstruction
  unsigned int validModelPoints = _rayCaster->calcCoordsFromCurrentViewMask(_grid, sensor, _modelCoords, _modelNormals, _maskM);
  if(validModelPoints == 0)
  {
    std::cout << __PRETTY_FUNCTION__ << " Error! Raycasting found no coordinates!\n";
    return;
  }

  //get current scan
  unsigned int validScenePoints = 0;
  validScenePoints = sensor->dataToCartesianVectorMask(_scene, _maskS);

  /**
   *  Create Point Matrixes with structure [x1 y1; x2 y2; ..]
   *  M, N, and S are matrices that preserve the ray model of a laser scanner
   *  Xvalid matrices are matrices that do not preserve the ray model but contain only valid points
   */
  obvious::Matrix M(measurementSize, 2, _modelCoords);
  obvious::Matrix N(measurementSize, 2, _modelNormals);
  obvious::Matrix Mvalid = maskMatrix(&M, _maskM, measurementSize, validModelPoints);

  for(unsigned int i = 0; i < validScenePoints; i++)
    _maskS[i] = sensor->getRealMeasurementMask()[i];
  for(unsigned int i = validScenePoints; i < measurementSize; i++)
    _maskS[i] = false;

  unsigned int size = sensor->dataToCartesianVector(_scene);
  obvious::Matrix S(measurementSize, 2, _scene);
  obvious::Matrix Svalid = maskMatrix(&S, _maskS, measurementSize, validScenePoints);

  /** Align Laser scans */
  obvious::Matrix T = doRegistration(sensor, &M, &Mvalid, &N, NULL, &S, &Svalid, _ransac);  //3x3 Transformation Matrix
  T.print();

  /** analyze registration result */
  _tf.stamp_ = ros::Time::now();
  const bool regErrorT = isRegistrationError(&T);
  
//  if(regErrorT && !_ransac) //icp only, we can use the ransac to get it back
//  {
//    std::cout << __PRETTY_FUNCTION__ << "regError! Trying to recapture with ICPSac\n";
//    obvious::Matrix secondT = doRegistration(sensor, &M, &Mvalid, &N, NULL, &S, &Svalid, true);  //3x3 Transformation Matrix
//    if(isRegistrationError(&secondT))
//    {
//      std::cout << __PRETTY_FUNCTION__ << "Could not recapture \n";
//      sendNanTransform();
//    }
//    else
//      std::cout << __PRETTY_FUNCTION__ << "Lucky You! Got back in place\n"; /todo push
//
//  }
//  else

  if(regErrorT)    //already used ransac, we can do nothing to get the pose right. Let's hope for the next scan
  {
    std::cout << __PRETTY_FUNCTION__ << "regError! \n";
    sendNanTransform();
  }
  else //transformation valid -> transform sensor and publish new sensor pose
  {
    sensor->transform(&T);
    obvious::Matrix curPose = sensor->getTransformation();

    sendTransform(&curPose);
    /** Update MAP if necessary */
    if(this->isPoseChangeSignificant(_lastPose, &curPose) && !_noPush)
    {
         *_lastPose = curPose;
         _mapper->queuePush(sensor);
    }
  }
}

obvious::Matrix Localization::doRegistration(obvious::SensorPolar2D* sensor,
                                              obvious::Matrix* M,
                                              obvious::Matrix* Mvalid,
                                              obvious::Matrix* N,
                                              obvious::Matrix* Nvalid,
                                              obvious::Matrix* S,
                                              obvious::Matrix* Svalid,
                                              const bool useRansac
                                              )
{

    const unsigned int measurementSize = sensor->getRealMeasurementSize();
    obvious::Matrix T44(4, 4);
    T44.setIdentity();

    // RANSAC pre-registration (rough)
    if(useRansac)
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

      //    maskToOneDegreeRes(_maskS, sensor->getAngularResolution(), measurementSize);
      //      maskToOneDegreeRes(_maskM, sensor->getAngularResolution(), measurementSize);
      RansacMatching ransac(50, 0.15, 180); //toDo: launch parameters
      const double phiMax = _rotMax;
      obvious::Matrix T(3, 3);
      if(factor == 1)
        T = ransac.match(M, _maskM, S, _maskS, phiMax, _trnsMax, sensor->getAngularResolution());
      else
        T = ransac.match(&Mreduced, maskMRed, &Sreduced, maskSRed, phiMax,
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
    _icp->setModel(Svalid, Nvalid);
    _icp->setScene(Mvalid);
    double rms = 0.0;
    unsigned int pairs = 0;
    unsigned int it = 0;
    _icp->iterate(&rms, &pairs, &it, &T44);
    obvious::Matrix T = _icp->getFinalTransformation();
    T.invert();

    return T;

}

bool Localization::isRegistrationError(obvious::Matrix* T)
{
  double deltaX = (*T)(0, 2);
  double deltaY = (*T)(1, 2);
  double trnsAbs = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  double deltaPhi = this->calcAngle(T);

  return (trnsAbs > _trnsMax) || (std::fabs(std::sin(deltaPhi)) > _rotMax);
}

void Localization::sendTransform(obvious::Matrix* T)
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

  //_pubMutex->lock();   //toDo: test if this mutex is necessary (other threads use this too)
  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);
  //_pubMutex->unlock();

}


void Localization::sendNanTransform()
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

  //_pubMutex->lock();
  _posePub.publish(_poseStamped);
  _tfBroadcaster.sendTransform(_tf);
  //_pubMutex->unlock();
}

double Localization::calcAngle(obvious::Matrix* T)
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

bool Localization::isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose)
{
  double deltaX   = (*curPose)(0, 2) - (*lastPose)(0, 2);
  double deltaY   = (*curPose)(1, 2) - (*lastPose)(1, 2);
  double deltaPhi = this->calcAngle(curPose) - this->calcAngle(lastPose);
  deltaPhi        = fabs(sin(deltaPhi));
  double trnsAbs  = sqrt(deltaX * deltaX + deltaY * deltaY);

  return (deltaPhi > ROT_MIN) || (trnsAbs > TRNS_MIN);
}

obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints)
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

bool Localization::togglePushServiceCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  _noPush = !_noPush;
  return true;
}

void maskToOneDegreeRes(bool* const mask, const double resolution, const unsigned int maskSize)
{
  const double desResRad = 1.0 * M_PI / 180.0;
  const unsigned int factor = static_cast<unsigned int>(desResRad / resolution + 0.5);
  for(unsigned int i = 0; i < maskSize; i++)
  {
    if((!(i % factor)) && mask[i])
      mask[i] = true;
    else
      mask[i] = false;
  }
}

void reduceResolution(bool* const maskIn, const obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
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

} /* namespace */
