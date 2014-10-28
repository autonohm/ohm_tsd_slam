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

#include <boost/bind.hpp>

#include <cstring>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace ohm_tsd_slam
{

ThreadLocalize::ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle& nh, boost::mutex* pubMutex, SlamNode& parentNode)
{
  _initialized      = false;
  _newScan          = false;
  _sensor           = NULL;
  _mask             = NULL;
  _minRange         = MIN_RANGE;
  _maxRange         = MAX_RANGE;
  _xOffset          = 0.0;                     //toDo Launch parameter for robots starting position
  _yOffset          = 0.0;
  _yawOffset        = 0.0;
  _rangeFilter      = true;


  _mapper           = mapper;
  _grid             = grid;

  _scene            = NULL;
  _modelCoords      = NULL;
  _modelNormals     = NULL;

  _rayCaster        = new obvious::RayCastPolar2D();
  _assigner         = new obvious::FlannPairAssignment(2);
  _filterDist       = new obvious::DistanceFilter(2.0, 0.01, ITERATIONS - 3);
  _filterReciprocal = new obvious::ReciprocalFilter();
  _estimator        = new obvious::ClosedFormEstimator2D();
  _trnsMax          = TRNS_THRESH;   //toDo: config file
  _rotMax           = ROT_THRESH;    //toDo: config file
  _lastPose         = new obvious::Matrix(3, 3);
  _xOffFactor       = parentNode.xOffFactor();  //toDo: obsolete
  _yOffFactor       = parentNode.yOffFactor();
  _pubMutex         = pubMutex;

  //configure ICP
  _filterBounds     = new obvious::OutOfBoundsFilter2D(grid->getMinX(), grid->getMaxX(), grid->getMinY(), grid->getMaxY());
  _assigner->addPreFilter(_filterBounds);
  _assigner->addPostFilter(_filterDist);
  _assigner->addPostFilter(_filterReciprocal);
  _icp = new obvious::Icp(_assigner, _estimator);
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(ITERATIONS);
  _icp->setConvergenceCounter(ITERATIONS);
  //_icp->activateTrace();

  std::string poseTopic;
  std::string tfBaseFrameId;
  std::string tfChildFrameId;
  ros::NodeHandle prvNh("~");
  prvNh.param("pose_topic", poseTopic, std::string("pose"));
  prvNh.param("tf_base_frame", tfBaseFrameId, std::string("/map"));
  prvNh.param("tf_child_frame", tfChildFrameId, std::string("base_footprint"));
  prvNh.param<double>("sensor_static_offset_x", _lasXOffset, -0.19);

  _posePub = nh.advertise<geometry_msgs::PoseStamped>(poseTopic, 1);

  _poseStamped.header.frame_id = tfBaseFrameId;

  _tf.frame_id_                = tfBaseFrameId;
  _tf.child_frame_id_          = tfChildFrameId;

}

ThreadLocalize::~ThreadLocalize()
{
  delete _sensor;
  delete _mask;
  delete _rayCaster;
  delete _assigner;
  delete _filterDist;
  delete _filterReciprocal;
  delete _estimator;
  delete _lastPose;
  delete _scene;
}

void ThreadLocalize::localize(obvious::SensorPolar2D* sensor)
{

}

void ThreadLocalize::eventLoop(void)
{
  while(_stayActive)
  {
    ros::spinOnce();
    if(_newScan)
    {
      this->localize(_sensor);
    }
    else
    {
      //maybe publish pose only ?
    }
    _newScan = false;
  }
}

void ThreadLocalize::init(const sensor_msgs::LaserScan& scan)
{
  _mask = new bool[scan.ranges.size()];
  for(unsigned int i=0; i < scan.ranges.size(); i++)
  {
    _mask[i] = !isnan(scan.ranges[i]) && !isinf(scan.ranges[i])&&(std::abs(scan.ranges[i])>10e-6);
  }

  _sensor=new obvious::SensorPolar2D(scan.ranges.size(), scan.angle_increment, scan.angle_min, static_cast<double>(_maxRange));
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);

  double phi        = _yawOffset;
  double gridWidth  =_grid->getCellsX() * _grid->getCellSize();
  double gridHeight =_grid->getCellsY() * _grid->getCellSize();
  _xOffset = gridWidth / 2.0;   //toDo: for testing start in the middle of the grid change that!
  _yOffset = gridWidth / 2.0;
  double tf[9] = {cos(phi), -sin(phi), _xOffset,
      sin(phi),  cos(phi), _yOffset,
      0,         0,               1};
  obvious::Matrix Tinit(3, 3);
  Tinit.setData(tf);
  _sensor->transform(&Tinit);

  unsigned int size = _sensor->getRealMeasurementSize();

  _scene        = new double[size * 2];
  _modelCoords  = new double[size * 2];
  _modelNormals = new double[size * 2];
  *_lastPose    = _sensor->getTransformation();
}

void ThreadLocalize::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << " received first scan. Initailize node...\n";   //toDo: print ID of referring robot
    this->init(scan);
    std::cout << __PRETTY_FUNCTION__ << " initialized -> running...\n";
  }
  for(unsigned int i=0;i<scan.ranges.size();i++)
  {

    _mask[i] = !isnan(scan.ranges[i]) && !isinf(scan.ranges[i]) && (fabs(scan.ranges[i]) > 10e-6);
    if((_rangeFilter)&&_mask[i])
      _mask[i]=(scan.ranges[i]>_minRange)&&(scan.ranges[i]<_maxRange);
  }
  _sensor->setRealMeasurementData(scan.ranges, 1.0);
  _sensor->setRealMeasurementMask(_mask);
  _newScan = true;
}

} /* namespace ohm_tsd_slam */
