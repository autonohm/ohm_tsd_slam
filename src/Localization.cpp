#include "Localization.h"
#include "MultiSlamNode.h"
#include "ThreadMapping.h"

#include "obcore/math/linalg/linalg.h"
#include "obcore/base/Logger.h"
#include "obcore/math/linalg/MatrixFactory.h"

#include <boost/bind.hpp>

#include <cstring>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

Localization::Localization(obvious::TsdGrid* grid, ThreadMapping* mapper, boost::mutex* pubMutex, const double xOffFactor, const double yOffFactor, std::string nameSpace)
{
  _pubMutex         = pubMutex;
  _mapper           = mapper;
  _grid             = grid;

  _scene            = NULL;
  _modelCoords      = NULL;
  _modelNormals     = NULL;

  ros::NodeHandle prvNh("~");
  double multiIteratorNeg = 0.0;
  double multiIteratorPos = 0.0;
  double icpIterations    = 0.0;

  prvNh.param<double>("multi_iter_neg_angle", multiIteratorNeg, -3.14 / 180 * 5.0);
  prvNh.param<double>("multi_iter_pose_anle", multiIteratorPos, 3.14 / 180 * 5.0);
  prvNh.param<double>("icp_iterations", icpIterations, ITERATIONS);
  prvNh.param<int>("multi_push", _multiPush, 0);

  std::vector<obvious::Matrix> trafoVector;
  obvious::MatrixFactory matFak;

  //obvious::Matrix negTrafo = matFak.TransformationMatrix33(multiIteratorNeg, 0.0, 0.0);
  obvious::Matrix negTrafo = matFak.TransformationMatrix44(multiIteratorNeg, 0.0, 0.0, 0.0, 0.0, 0.0);
  obvious::Matrix posdblTrafo = matFak.TransformationMatrix44(2.0 * multiIteratorPos, 0.0, 0.0, 0.0, 0.0, 0.0);
  obvious::Matrix negdblTrafo = matFak.TransformationMatrix44(2.0 * multiIteratorNeg, 0.0, 0.0, 0.0, 0.0, 0.0);
  obvious::Matrix posTrafo = matFak.TransformationMatrix44(multiIteratorPos, 0.0, 0.0, 0.0, 0.0, 0.0);
  obvious::Matrix ident    = matFak.TransformationMatrix44(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  obvious::Matrix transX   = matFak.TranslationMatrix44(1.0, 0.0, 0.0);

  //obvious::Matrix posTrafo = matFak.TransformationMatrix33(multiIteratorPos, 0.0, 0.0);
  //obvious::Matrix ident    =  matFak.TransformationMatrix33(0.0, 0.0, 0.0);

  trafoVector.push_back(transX);
  trafoVector.push_back(negTrafo);
  trafoVector.push_back(ident);
  //    trafoVector.push_back(posTrafo);
  //    trafoVector.push_back(posdblTrafo);
  //    trafoVector.push_back(negdblTrafo);


  _rayCaster        = new obvious::RayCastPolar2D();
  _assigner         = new obvious::FlannPairAssignment(2);
  _filterDist       = new obvious::DistanceFilter(3.0, 0.01, icpIterations - 3);
  _filterReciprocal = new obvious::ReciprocalFilter();
  _estimator        = new obvious::ClosedFormEstimator2D();
  //_estimator        = new obvious::PointToLine2DEstimator();
  _trnsMax          = TRNS_THRESH;   //toDo: config file
  _rotMax           = ROT_THRESH;    //toDo: config file
  _lastPose         = new obvious::Matrix(3, 3);
  _xOffFactor       = xOffFactor;
  _yOffFactor       = yOffFactor;



  //configure ICP
  _filterBounds     = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _multiIcp = new obvious::IcpMultiInitIterator(trafoVector);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(icpIterations);
  _icp->setConvergenceCounter(icpIterations);
  _icp->activateTrace();


  if(nameSpace.size())   //given namespace
    nameSpace += "/";

  std::string poseParamServer;
  poseParamServer = nameSpace + "pose_topic";
  std::string poseTopic;
  prvNh.param(poseParamServer, poseTopic, std::string("default_ns/pose"));
  poseTopic = nameSpace + poseTopic;

  std::string tfBaseFrameId;
  prvNh.param("tf_base_frame", tfBaseFrameId, std::string("/map"));

  std::string tfChildParamServer;
  tfChildParamServer = nameSpace + "tf_child_frame";
  std::string tfChildFrameId;
  //std::cout << __PRETTY_FUNCTION__ << " looking up " << tfChildParamServer << std::endl;
  prvNh.param(tfChildParamServer, tfChildFrameId, std::string("default_ns/base_footprint"));
  //std::cout << __PRETTY_FUNCTION__ << " found " << tfChildFrameId << std::endl;

  //double sensorStaticXoffset = 0.0;
  std::string sensorStaticXoffsetParamServer;
  sensorStaticXoffsetParamServer = nameSpace + "/sensor_static_offset_x";
  prvNh.param<double>(sensorStaticXoffsetParamServer, _lasXOffset, 0.0);

  _posePub = _nh.advertise<geometry_msgs::PoseStamped>(poseTopic, 1);
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
//  obvious::Matrix N( modelSize / 2, 2, _modelNormals);
  _icp->setScene(&M);//, &N);

  size = sensor->dataToCartesianVector(_scene);
  obvious::Matrix S(size / 2, 2, _scene);
  _icp->setModel(&S);

  double rms = 0.0;
  unsigned int pairs = 0;
  unsigned int it = 0;

  //std::cout << __PRETTY_FUNCTION__ << "premultiiteratoricp " << std::endl;
  //obvious::Matrix T = _multiIcp->iterate(_icp);
  //std::cout << __PRETTY_FUNCTION__ << "postmultiiteratoricp " << std::endl;
  _icp->iterate(&rms, &pairs, &it);
  obvious::Matrix T = _icp->getFinalTransformation();
  T.invert();

  // analyze registration result
  double deltaX = T(0,2);
  double deltaY = T(1,2);
  double trnsAbs = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  double deltaPhi = this->calcAngle(&T);
  _tf.stamp_ = ros::Time::now();

  if(deltaY > 0.1 || (trnsAbs > _trnsMax) || std::fabs(std::sin(deltaPhi)) > _rotMax)
  {
    // localization error broadcast invalid tf

    sensor->transform(&T);
    obvious::Matrix curPose = sensor->getTransformation();
    double curTheta = this->calcAngle(&curPose);
    double posX=curPose(0, 2) + std::cos(curTheta) * _lasXOffset -_grid->getCellsX() * _grid->getCellSize() * _xOffFactor;
    double posY=curPose(1, 2) + std::sin(curTheta) * _lasXOffset -_grid->getCellsY() * _grid->getCellSize() * _yOffFactor;
    _poseStamped.header.stamp = ros::Time::now();
    _poseStamped.pose.position.x = posX;
    _poseStamped.pose.position.y = posY;
    _poseStamped.pose.position.z = 0.0;
    tf::Quaternion quat(0.0, 0.0, curTheta);
    _poseStamped.pose.orientation.w = quat.w();
    _poseStamped.pose.orientation.x = quat.x();
    _poseStamped.pose.orientation.y = quat.y();
    _poseStamped.pose.orientation.z = quat.z();
    _icp->serializeTrace("/tmp/icp", 100);
    //    std::cout << __PRETTY_FUNCTION__ << "regError!\n";
    //    _poseStamped.header.stamp = ros::Time::now();
    //    _poseStamped.pose.position.x = NAN;
    //    _poseStamped.pose.position.y = NAN;
    //    _poseStamped.pose.position.z = NAN;
    //    tf::Quaternion quat(NAN, NAN, NAN);
    //    _poseStamped.pose.orientation.w = quat.w();
    //    _poseStamped.pose.orientation.x = quat.x();
    //    _poseStamped.pose.orientation.y = quat.y();
    //    _poseStamped.pose.orientation.z = quat.z();
    //
    //    _tf.stamp_ = ros::Time::now();
    //    _tf.setOrigin(tf::Vector3(NAN, NAN, NAN));
    //    _tf.setRotation(quat);

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
    double posX=curPose(0, 2) + std::cos(curTheta) * _lasXOffset -_grid->getCellsX() * _grid->getCellSize() * _xOffFactor;
    double posY=curPose(1, 2) + std::sin(curTheta) * _lasXOffset -_grid->getCellsY() * _grid->getCellSize() * _yOffFactor;
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

    //_pubMutex->lock();   //toDo: test if this mutex is necessary (other threads use this too)
    _posePub.publish(_poseStamped);
    _tfBroadcaster.sendTransform(_tf);
    //_pubMutex->unlock();
    if(this->isPoseChangeSignificant(_lastPose, &curPose))
    {
      *_lastPose = curPose;
      for(int i = 0; i < _multiPush; i++)
      {
        _mapper->queuePush(sensor);
      }
    }
  }
}

void Localization::multiScanLocalize(const std::vector<obvious::SensorPolar2D*> multiScans, obvious::SensorPolar2D* sensor)
{
  unsigned int size = sensor->getRealMeasurementSize();
  if(!_scene)
  {
    _scene        = new double[size * 2];
    _modelCoords  = new double[size * 2];
    _modelNormals = new double[size * 2];
    *_lastPose    = sensor->getTransformation();
  }
//  obvious::Matrix Tcum(3, 3);
//  Tcum.setIdentity();
  _mapper->queuePush(multiScans[0]);
  for (unsigned int i = 0; i <= multiScans.size() - 2; i++)
  {
    _icp->reset();
    obvious::Matrix P = multiScans[i]->getTransformation();
    _filterBounds->setPose(&P);
    size = multiScans[i]->dataToCartesianVector(_modelCoords);
    obvious::Matrix M(size / 2, 2, _modelCoords);
    _icp->setModel(&M);//, &N);
    size = multiScans[i + 1]->dataToCartesianVector(_scene);
    obvious::Matrix S(size / 2, 2, _scene);
    _icp->setScene(&S);

    double rms = 0.0;
    unsigned int pairs = 0;
    unsigned int it = 0;
    _icp->iterate(&rms, &pairs, &it);
    obvious::Matrix T = _icp->getFinalTransformation();
    multiScans[i + 1]->transform(&T);
    _mapper->queuePush(multiScans[i + 1]);    //toDo: validate transformation and remove the fowl data set
    //Tcum
  }
  unsigned int modelSize = 0;
  _rayCaster->calcCoordsFromCurrentView(_grid, multiScans[multiScans.size() - 1], _modelCoords, _modelNormals, &modelSize);
  if(modelSize == 0)
  {
    std::cout << __PRETTY_FUNCTION__ << " Error! Raycasting found no coordinates!\n";
    return;
  }
  _icp->reset();
   obvious::Matrix P = multiScans[multiScans.size() - 1]->getTransformation();
   _filterBounds->setPose(&P);

   obvious::Matrix M( modelSize / 2, 2, _modelCoords);
   //obvious::Matrix N( modelSize / 2, 2, _modelNormals);
   _icp->setModel(&M);//, &N);

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
    _tf.stamp_ = ros::Time::now();

    if(deltaY > 0.1 || (trnsAbs > _trnsMax) || std::fabs(std::sin(deltaPhi)) > _rotMax)
    {
      // localization error broadcast invalid tf

      sensor->transform(&T);
      obvious::Matrix curPose = sensor->getTransformation();
      double curTheta = this->calcAngle(&curPose);
      double posX=curPose(0, 2) + std::cos(curTheta) * _lasXOffset -_grid->getCellsX() * _grid->getCellSize() * _xOffFactor;
      double posY=curPose(1, 2) + std::sin(curTheta) * _lasXOffset -_grid->getCellsY() * _grid->getCellSize() * _yOffFactor;
      _poseStamped.header.stamp = ros::Time::now();
      _poseStamped.pose.position.x = posX;
      _poseStamped.pose.position.y = posY;
      _poseStamped.pose.position.z = 0.0;
      tf::Quaternion quat(0.0, 0.0, curTheta);
      _poseStamped.pose.orientation.w = quat.w();
      _poseStamped.pose.orientation.x = quat.x();
      _poseStamped.pose.orientation.y = quat.y();
      _poseStamped.pose.orientation.z = quat.z();
      _icp->serializeTrace("/tmp/icp");
      //    std::cout << __PRETTY_FUNCTION__ << "regError!\n";
      //    _poseStamped.header.stamp = ros::Time::now();
      //    _poseStamped.pose.position.x = NAN;
      //    _poseStamped.pose.position.y = NAN;
      //    _poseStamped.pose.position.z = NAN;
      //    tf::Quaternion quat(NAN, NAN, NAN);
      //    _poseStamped.pose.orientation.w = quat.w();
      //    _poseStamped.pose.orientation.x = quat.x();
      //    _poseStamped.pose.orientation.y = quat.y();
      //    _poseStamped.pose.orientation.z = quat.z();
      //
      //    _tf.stamp_ = ros::Time::now();
      //    _tf.setOrigin(tf::Vector3(NAN, NAN, NAN));
      //    _tf.setRotation(quat);

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
      double posX=curPose(0, 2) + std::cos(curTheta) * _lasXOffset -_grid->getCellsX() * _grid->getCellSize() * _xOffFactor;
      double posY=curPose(1, 2) + std::sin(curTheta) * _lasXOffset -_grid->getCellsY() * _grid->getCellSize() * _yOffFactor;
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

      //_pubMutex->lock();   //toDo: test if this mutex is necessary (other threads use this too)
      _posePub.publish(_poseStamped);
      _tfBroadcaster.sendTransform(_tf);
      //_pubMutex->unlock();
      if(this->isPoseChangeSignificant(_lastPose, &curPose))
      {
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
