#include "ThreadMapping.h"
#include "SlamNode.h"

#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include <cmath>

namespace ohm_tsd_slam
{

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid):
    ThreadSLAM(*grid),
    _initialized(false)
{
  ros::NodeHandle prvNh("~");
  prvNh.param<double>("depth_discontinuity_thresh", _depthDiscontinuityThresh, 3.0);
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
  if(this->initialized())
    return;
  _pushMutex.lock();
  for(unsigned int i = 0; i < INIT_PSHS; i++)
    _grid.push(sensor);
  _initialized = true;
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
      _grid.push(sensor);
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
  obvious::SensorPolar2D* sensorLocal = new obvious::SensorPolar2D(sensor->getRealMeasurementSize(), sensor->getAngularResolution(), sensor->getPhiMin(),
                                                                   sensor->getMaximumRange(), sensor->getMinimumRange(), sensor->getLowReflectivityRange());
  sensorLocal->setTransformation(sensor->getTransformation());
  sensorLocal->setRealMeasurementData(sensor->getRealMeasurementData());
  sensorLocal->setRealMeasurementMask(sensor->getRealMeasurementMask());
  sensorLocal->maskDepthDiscontinuity(obvious::deg2rad(_depthDiscontinuityThresh));
  _sensors.push(sensorLocal);
  _pushMutex.unlock();
  this->unblock();
}

} /* namespace */
