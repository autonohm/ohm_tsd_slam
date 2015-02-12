#include "Localization.h"
#include "SlamNode.h"
#include "ThreadMapping.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"

#include "obvision/ransacMatching/RansacMatching.h"

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

  _posePub = nh.advertise < geometry_msgs::PoseStamped > (poseTopic, 1);

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
}

void Localization::localize(obvious::SensorPolar2D* sensor)
{
  unsigned int size = sensor->getRealMeasurementSize();

  if(!_scene)
  {
    _scene = new double[size * 2];
    _modelCoords = new double[size * 2];
    _modelNormals = new double[size * 2];
    *_lastPose = sensor->getTransformation();
  }

  // reconstruction
  unsigned int modelSize = 0;
  _rayCaster->calcCoordsFromCurrentView(_grid, sensor, _modelCoords, _modelNormals, &modelSize);
  if(modelSize == 0)
  {
    std::cout << __PRETTY_FUNCTION__ << " Error! Raycasting found no coordinates!\n";
    return;
  }

  _icp->reset();
  obvious::Matrix P = sensor->getTransformation();
  _filterBounds->setPose(&P);

  obvious::Matrix M(modelSize / 2, 2, _modelCoords);
  obvious::Matrix N(modelSize / 2, 2, _modelNormals);

  size = sensor->dataToCartesianVector(_scene);

  obvious::Matrix S(size / 2, 2, _scene);

  obvious::Matrix T44(4, 4);
  T44.setIdentity();

  // RANSAC pre-registration (rough)
  if(_ransac)
  {
    RansacMatching ransac;
    obvious::Matrix T = ransac.match(&M, &S, M_PI / 4, sensor->getAngularResolution());
    T.invert();
    T44(0, 0) = T(0, 0);
    T44(0, 1) = T(0, 1);
    T44(0, 3) = T(0, 2);
    T44(1, 0) = T(1, 0);
    T44(1, 1) = T(1, 1);
    T44(1, 3) = T(1, 2);
  }

  _icp->setModel(&S, NULL);
  _icp->setScene(&M);
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

  if(deltaY > 0.5 || (trnsAbs > _trnsMax) || std::fabs(std::sin(deltaPhi)) > _rotMax)
  {
    cout << "Registration error - deltaY=" << deltaY << " trnsAbs=" << trnsAbs << " sin(deltaPhi)=" << sin(deltaPhi)
        << endl;
    // localization error broadcast invalid tf

    _poseStamped.header.stamp = ros::Time::now();
    _poseStamped.pose.position.x = NAN;
    _poseStamped.pose.position.y = NAN;
    _poseStamped.pose.position.z = NAN;
    tf::Quaternion quat(NAN, NAN, NAN);
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
    tf::Quaternion quat(0.0, 0.0, curTheta);
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
