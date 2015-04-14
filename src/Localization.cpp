#include "Localization.h"
#include "SlamNode.h"
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

Localization::Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, const double xOffFactor, const double yOffFactor, const bool ransac) :
    _gridOffSetX(-1.0 * grid->getCellsX() * grid->getCellSize() * xOffFactor), _gridOffSetY(-1.0 * grid->getCellsY()
        * grid->getCellSize() * yOffFactor)
{
  ros::NodeHandle prvNh("~");
  _mapper = mapper;
  _grid = grid;
  _ransac = ransac;
  _scene = NULL;
  _modelCoords = NULL;
  _modelNormals = NULL;

  double distFilterMax = 0.0;
  double distFilterMin = 0.0;
  int icpIterations = 0;

  prvNh.param<double>("dist_filter_max", distFilterMax, 0.2);
  prvNh.param<double>("dist_filter_min", distFilterMin, 0.01);
  prvNh.param<int>("icp_iterations", icpIterations, 20);

  _rayCaster = new obvious::RayCastPolar2D();
  _assigner = new obvious::FlannPairAssignment(2);
  _filterDist = new obvious::DistanceFilter(distFilterMax, distFilterMin, icpIterations - 10);
  _filterReciprocal = new obvious::ReciprocalFilter();
  _estimator = new obvious::ClosedFormEstimator2D();
  //_estimator = new obvious::PointToLine2DEstimator();
  prvNh.param<double>("reg_trs_max", _trnsMax, TRNS_THRESH);
  prvNh.param<double>("reg_sin_rot_max", _rotMax, ROT_THRESH);
  _lastPose = new obvious::Matrix(3, 3);
  _xOffFactor = xOffFactor;
  _yOffFactor = yOffFactor;

  //configure ICP
  _filterBounds = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(icpIterations);
  _icp->setConvergenceCounter(icpIterations);

  std::string poseTopic;
  std::string tfBaseFrameId;
  std::string tfChildFrameId;

  prvNh.param("pose_topic", poseTopic, std::string("pose"));
  prvNh.param("tf_base_frame", tfBaseFrameId, std::string("/map"));
  prvNh.param("tf_child_frame", tfChildFrameId, std::string("laser"));

  _posePub = nh.advertise<geometry_msgs::PoseStamped>(poseTopic, 1);

  _poseStamped.header.frame_id = tfBaseFrameId;
  _tf.frame_id_ = tfBaseFrameId;
  _tf.child_frame_id_ = tfChildFrameId;
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
    delete[] _modelCoords;
  if(_modelNormals)
    delete[] _modelNormals;
  if(_scene)
    delete[] _scene;
  if(_maskS)
    delete[] _maskS;
  if(_maskM)
    delete[] _maskM;
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

void Localization::localize(obvious::SensorPolar2D* sensor)
{
  unsigned int measurementSize = sensor->getRealMeasurementSize();
  bool* measurementMask = sensor->getRealMeasurementMask();
  if(!_scene)
  {
    _scene        = new double[measurementSize * 2];
    _maskS        = new bool[measurementSize];
    _modelCoords  = new double[measurementSize * 2];
    _modelNormals = new double[measurementSize * 2];
    _maskM        = new bool[measurementSize];
    *_lastPose    = sensor->getTransformation();
  }

  // reconstruction

  unsigned int validModelPoints = _rayCaster->calcCoordsFromCurrentViewMask(_grid, sensor, _modelCoords, _modelNormals, _maskM);
  if(validModelPoints == 0)
  {
    std::cout << __PRETTY_FUNCTION__ << " Error! Raycasting found no coordinates!\n";
    return;
  }

  unsigned int validScenePoints = 0;

  validScenePoints = sensor->dataToCartesianVectorMask(_scene, _maskS);

  obvious::Matrix M(measurementSize, 2, _modelCoords);
  obvious::Matrix N(measurementSize, 2, _modelNormals);
  obvious::Matrix Mvalid = maskMatrix(&M, _maskM, measurementSize, validModelPoints);
  //obvious::Matrix Nvalid = maskMatrix(&N, _maskM, measurementSize, validModelPoints);

  for(unsigned int i=0; i<validScenePoints; i++)
    _maskS[i] = measurementMask[i];
  for(unsigned int i=validScenePoints; i<measurementSize; i++)
    _maskS[i] = false;

  obvious::Matrix S(measurementSize, 2, _scene);
  obvious::Matrix Svalid = maskMatrix(&S, _maskS, measurementSize, validScenePoints);

  obvious::Matrix T44(4, 4);
  T44.setIdentity();

  // RANSAC pre-registration (rough)
  RansacMatching ransac;
  double phiMax = M_PI / 3.0;
  if(_ransac)
  {
    //ransac.activateTrace();
    obvious::Matrix T = ransac.match(&M, _maskM, &S, _maskS, phiMax, _trnsMax, sensor->getAngularResolution());
    T.invert();
    T44(0, 0) = T(0, 0);
    T44(0, 1) = T(0, 1);
    T44(0, 3) = T(0, 2);
    T44(1, 0) = T(1, 0);
    T44(1, 1) = T(1, 1);
    T44(1, 3) = T(1, 2);
  }

  _icp->reset();
  //_icp->activateTrace();
  obvious::Matrix P = sensor->getTransformation();
  _filterBounds->setPose(&P);
  _icp->setModel(&Svalid, NULL);
  _icp->setScene(&Mvalid);
  double rms = 0.0;
  unsigned int pairs = 0;
  unsigned int it = 0;
  _icp->iterate(&rms, &pairs, &it, &T44);
  obvious::Matrix T = _icp->getFinalTransformation();
  T.invert();

  // analyze registration result
  double deltaX = T(0, 2);
  double deltaY = T(1, 2);
  double trnsAbs = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  double deltaPhi = this->calcAngle(&T);
  _tf.stamp_ = ros::Time::now();

  //cout << "Registration: " << deltaY << " trnsAbs=" << trnsAbs << " sin(deltaPhi)=" << sin(deltaPhi) << endl;

  if(abs(deltaY) > 0.5 || (trnsAbs > _trnsMax) || std::fabs(std::sin(deltaPhi)) > phiMax)
  {
    cout << "Registration error - deltaY=" << deltaY << " trnsAbs=" << trnsAbs << " sin(deltaPhi)=" << sin(deltaPhi) << endl;
    // ransac.serializeTrace("/tmp/ransac/");

    // localization error broadcast invalid tf

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

    // Try circumferential matching
//    RansacMatching ransac(200, 0.15, 180);
//    double phiMax = M_PI/2.0;
//    obvious::Matrix T = ransac.match(&M, _maskM, &S, _maskS, phiMax, sensor->getAngularResolution());
//    //T.invert();
//    sensor->transform(&T);
  }
  else            //transformation valid -> transform sensor
  {
    sensor->transform(&T);
    obvious::Matrix curPose = sensor->getTransformation();
    double curTheta = this->calcAngle(&curPose);
    double posX = curPose(0, 2) + _gridOffSetX;
    double posY = curPose(1, 2) + _gridOffSetY;
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
    if(this->isPoseChangeSignificant(_lastPose, &curPose))
    {
      *_lastPose = curPose;
      _mapper->queuePush(sensor);
    }
  }
}

double Localization::calcAngle(obvious::Matrix* T)
{
  double angle = 0.0;
  const double ARCSIN = asin((*T)(1, 0));
  const double ARCSINEG = asin((*T)(0, 1));
  const double ARCOS = acos((*T)(0, 0));
  if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
    angle = ARCOS;
  else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
    angle = 2.0 * M_PI - ARCOS;
  return (angle);
}

bool Localization::isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose)
{
  double deltaX = (*curPose)(0, 2) - (*lastPose)(0, 2);
  double deltaY = (*curPose)(1, 2) - (*lastPose)(1, 2);
  double deltaPhi = this->calcAngle(curPose) - this->calcAngle(lastPose);
  deltaPhi = fabs(sin(deltaPhi));
  double trnsAbs = sqrt(deltaX * deltaX + deltaY * deltaY);

  return (deltaPhi > ROT_MIN) || (trnsAbs > TRNS_MIN);
}

} /* namespace */
