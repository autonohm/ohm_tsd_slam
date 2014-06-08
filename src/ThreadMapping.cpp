#include "ThreadMapping.h"
#include "obcore/base/Logger.h"

namespace ohm_tsd_slam
{

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid)
{
  _grid = grid;
}

ThreadMapping::~ThreadMapping()
{

}

void ThreadMapping::eventLoop(void)
{
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);

    LOGMSG(DBG_DEBUG, "thread awake");
    while(_stayActive && !_sensors.empty())
    {
      LOGMSG(DBG_DEBUG, "push");
      obvious::SensorPolar2D* sensor = _sensors.front();
      _grid->push(sensor);
      LOGMSG(DBG_DEBUG, "pushing done");
      _pushMutex.lock();
      LOGMSG(DBG_DEBUG, "remove from queue");
      delete _sensors.front();
      _sensors.pop();
      LOGMSG(DBG_DEBUG, "removing done");
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
  _sensors.push(sensorLocal);
  _pushMutex.unlock();

  this->unblock();
}

} /* namespace */
