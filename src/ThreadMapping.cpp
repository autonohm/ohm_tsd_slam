#include "ThreadMapping.h"
#include "SlamNode.h"

#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

namespace ohm_tsd_slam
{

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid)
{
  _grid = grid;
  _initialized = false;
}

ThreadMapping::~ThreadMapping()
{
  _thread->join();
}

bool ThreadMapping::initialized(void)
{
  bool var = false;
  _pushMutex.lock();
  var = _initialized;
  _pushMutex.unlock();
  return var;
}

void ThreadMapping::initPush(obvious::SensorPolar2D* sensor)
{
  _pushMutex.lock();
  for(unsigned int i = 0; i < INIT_PSHS; i++)
    _grid->push(sensor);
  _pushMutex.unlock();
}

void ThreadMapping::eventLoop(void)
{
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);
    while(_stayActive && !_sensors.empty())
    {
      obvious::SensorPolar2D* sensor = _sensors.front();
      _grid->push(sensor);
      _pushMutex.lock();
      delete _sensors.front();
      _sensors.pop();
      _initialized = true;
      _pushMutex.unlock();
    }
  }
}

void ThreadMapping::queuePush(obvious::SensorPolar2D* sensor)
{
  _pushMutex.lock();
  obvious::SensorPolar2D* sensorLocal = new obvious::SensorPolar2D(sensor->getRealMeasurementSize(), sensor->getAngularResolution(), sensor->getPhiMin(), sensor->getMaximumRange());
  sensorLocal->setTransformation(sensor->getTransformation());
  sensorLocal->setRealMeasurementData(sensor->getRealMeasurementData());
  sensorLocal->setRealMeasurementMask(sensor->getRealMeasurementMask());
  sensorLocal->maskDepthDiscontinuity(obvious::deg2rad(3.0));
  _sensors.push(sensorLocal);
  _pushMutex.unlock();
  this->unblock();
}

void ThreadMapping::posePush(obvious::Matrix& pose, const ros::Time& timeStamp, const std::string& nameSpace)
{
	_poseMutex.lock();
	StampedPose* poseVar = new StampedPose;
	poseVar->_pose = pose;
	poseVar->_stamp = timeStamp;
	poseVar->_nameSpace = nameSpace;
	_robotPoses.push_back(poseVar);
	_poseMutex.unlock();
}

void ThreadMapping::multiRobotPoseFilter(obvious::SensorPolar2D& sensor, const std::string& nameSpace)
{
  StampedPose* curData = NULL;
  double curWidth      = 0.0;
  double curHeight     = 0.0;
  for(std::vector<StampedPose*>::iterator iter = _robotPoses.begin(); iter < _robotPoses.end(); iter++)
  {
    curData = *iter;
    if(curData->_nameSpace == nameSpace)
      continue;
    curWidth  = curData->_width;
    curHeight = curData->_height;
    obvious::Matrix corners(4, 4);
    corners(0, 0) = curWidth  / 2.0;
    corners(0, 1) = curHeight / 2.0;
  }
}

} /* namespace */
