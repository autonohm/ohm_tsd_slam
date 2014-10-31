#include "ThreadMapping.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

namespace ohm_tsd_slam
{

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid)
{
  _grid = grid;
}

ThreadMapping::~ThreadMapping()
{
  _thread->join();
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
      _pushMutex.unlock();
    }
  }
}

void ThreadMapping::queuePush(obvious::SensorPolar2D* sensor)
{
  _pushMutex.lock();
  obvious::SensorPolar2D* sensorLocal = new obvious::SensorPolar2D(sensor->getRealMeasurementSize(), sensor->getAngularResolution(), sensor->getPhiMin(), sensor->getMaximumRange(), sensor->getMinimumRange(), sensor->getLowReflectivityRange());
  sensorLocal->setTransformation(sensor->getTransformation());
  sensorLocal->setRealMeasurementData(sensor->getRealMeasurementData());
  sensorLocal->resetMask();
  sensorLocal->maskDepthDiscontinuity(obvious::deg2rad(3.0));
  _sensors.push(sensorLocal);
  _pushMutex.unlock();

  this->unblock();
}

} /* namespace */
