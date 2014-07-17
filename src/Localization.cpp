#include "Localization.h"
#include "SlamNode.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"

#include <boost/bind.hpp>

#include <cstring>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

Localization::Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, boost::mutex* pubMutex)
{
  _pubMutex         = pubMutex;
  _mapper           = mapper;
  _grid             = grid;

  _scene            = NULL;
  _modelCoords      = NULL;
  _modelNormals     = NULL;

  _rayCaster        = new obvious::RayCastPolar2D();
  _assigner         = new obvious::FlannPairAssignment(2);
  _filterDist       = new obvious::DistanceFilter(2.0, 0.01, ITERATIONS - 3);
  _filterReciprocal = new obvious::ReciprocalFilter();
  //_estimator        = new obvious::PointToLine2DEstimator();
  _estimator        = new obvious::ClosedFormEstimator2D();
  _trnsMax          = TRNS_THRESH;
  _rotMax           = ROT_THRESH;
  _lastPose         = new obvious::Matrix(3, 3);

  //configure ICP
  _filterBounds     = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(ITERATIONS);
  _icp->setConvergenceCounter(ITERATIONS);
  _icp->activateTrace();

  std::string strVar;
  ros::NodeHandle prvNh("~");
  prvNh.param("pose_topic", strVar, std::string("pose"));
  _posePub = nh.advertise<geometry_msgs::PoseStamped>(strVar, 1);

  _poseStamped.header.frame_id = "/map";

  _tf.frame_id_       = "/map";    //toDo: launch parameters
  _tf.child_frame_id_ = "base_footprint";
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

  if(_modelCoords) delete [] _modelCoords;
  if(_modelNormals) delete [] _modelNormals;
  if(_scene) delete [] _scene;
}

void Localization::localize(obvious::SensorPolar2D* sensor)
{
  LOGMSG(DBG_DEBUG, "localize");

  unsigned int size = sensor->getRealMeasurementSize();

  if(!_scene)
  {
    _scene        = new double[size * 2];
    _modelCoords  = new double[size * 2];
    _modelNormals = new double[size * 2];
    *_lastPose    = sensor->getTransformation();
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

  obvious::Matrix M( modelSize / 2, 2, _modelCoords);
  obvious::Matrix N( modelSize / 2, 2, _modelNormals);
  _icp->setModel(&M, &N);

  size = sensor->dataToCartesianVector(_scene);
  obvious::Matrix S(size / 2, 2, _scene);
  _icp->setScene(&S);

  double rms = 0.0;
  unsigned int pairs = 0;
  unsigned int it = 0;
  _icp->iterate(&rms, &pairs, &it);
  obvious::Matrix T = _icp->getFinalTransformation();

  // analyze registration result
  double deltaX = T(0,2);
  double deltaY = T(1,2);
  double trnsAbs = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  double deltaPhi = this->calcAngle(&T);

  if(deltaY > 0.1 || (trnsAbs > _trnsMax) || std::fabs(std::sin(deltaPhi)) > _rotMax)
  {
    /*LOGMSG(DBG_DEBUG, "invalid transformation dX: " << deltaX << " dY: " << deltaY << " dPhi: " << deltaPhi);
    cout << "Pose: " << endl;
    P.print();
    cout << "T_icp: " << endl;
    T.print();
    _icp->serializeTrace("trace", 50);

    abort();*/

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

    _pubMutex->lock();
    _posePub.publish(_poseStamped);
    _tfBroadcaster.sendTransform(_tf);
    _pubMutex->unlock();
  }
  else            //transformation valid -> transform sensor
  {
    sensor->transform(&T);
    obvious::Matrix curPose = sensor->getTransformation();
    double curTheta = this->calcAngle(&curPose);
    double posX=curPose(0, 2) + std::cos(curTheta) * LAS_OFFS_X -_grid->getCellsX()*_grid->getCellSize()*S_X_F;   //toDo: Member variables and launch file parameters
    double posY=curPose(1, 2) + std::sin(curTheta) * LAS_OFFS_X -_grid->getCellsY()*_grid->getCellSize()*S_Y_F;
    _poseStamped.header.stamp = ros::Time::now();
    _poseStamped.pose.position.x = posX;
    _poseStamped.pose.position.y = posY;
    _poseStamped.pose.position.z = 0.0;
    tf::Quaternion quat(0.0, 0.0, curTheta);  //toDo: make change in obvious::estimator -> obsolete
    _poseStamped.pose.orientation.w = quat.w();
    _poseStamped.pose.orientation.x = quat.x();
    _poseStamped.pose.orientation.y = quat.y();
    _poseStamped.pose.orientation.z = quat.z();

    _tf.stamp_ = ros::Time::now();
    _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
    _tf.setRotation(quat);

    _pubMutex->lock();
    LOGMSG(DBG_DEBUG, "publish");
    _posePub.publish(_poseStamped);
    _tfBroadcaster.sendTransform(_tf);
    LOGMSG(DBG_DEBUG, "publishing done");
    _pubMutex->unlock();

    if(this->isPoseChangeSignificant(_lastPose, &curPose))
    {
 //     std::cout << "pose change is significant ... initiating push" << std::endl;
      *_lastPose = curPose;
      _mapper->queuePush(sensor);
    }
  }
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

} /* namespace */
