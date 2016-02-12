#include "ThreadMapping.h"
#include "SlamNode.h"

#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"

#include <cmath>

namespace ohm_tsd_slam
{

static std::vector<ros::Duration> _iterTimes;

ThreadMapping::ThreadMapping(obvious::TsdGrid* grid):
        ThreadSLAM(*grid),
        _initialized(false)
{
}

ThreadMapping::~ThreadMapping()
{
  _thread->join();
//  std::ofstream stream;
//    stream.open("/tmp/mapiter.log", std::ofstream::out);
//    if(!stream.is_open())
//      std::cout << __PRETTY_FUNCTION__ << "Error opening file" << "/tmp/mapiter.log" <<std::endl;
//    for(std::vector<ros::Duration>::iterator iter = _iterTimes.begin(); iter < _iterTimes.end(); iter++)
//    {
//      stream << iter->toNSec() * 10e-9 << std::endl;
//    }
//    stream.close();
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
      ros::Time timer;
      timer = ros::Time::now();
      _pushMutex.lock();
      obvious::SensorPolar2D* sensor = _sensors.back();
      _sensors.pop_back();
      _pushMutex.unlock();

      _grid.push(sensor);
      _pushMutex.lock();
      //cout << "Queue size: " << _sensors.size() << endl;
      delete sensor;
      _initialized = true;
      _pushMutex.unlock();
      _iterTimes.push_back(ros::Time::now() - timer);
    }
  }
//  std::ofstream stream;
//  stream.open("/tmp/mapiter.log", std::ofstream::out);
//  if(!stream.is_open())
//    std::cout << __PRETTY_FUNCTION__ << "Error opening file" << "/tmp/mapiter.log" <<std::endl;
//  for(std::vector<ros::Duration>::iterator iter = _iterTimes.begin(); iter < _iterTimes.end(); iter++)
//  {
//    stream << iter->toNSec() * 10e-9 << std::endl;
//  }
//  stream.close();
}

void ThreadMapping::queuePush(obvious::SensorPolar2D* sensor)
{
  _pushMutex.lock();
  obvious::SensorPolar2D* sensorLocal = new obvious::SensorPolar2D(sensor->getRealMeasurementSize(), sensor->getAngularResolution(), sensor->getPhiMin(),
      sensor->getMaximumRange(), sensor->getMinimumRange(), sensor->getLowReflectivityRange());
  sensorLocal->setTransformation(sensor->getTransformation());
  sensorLocal->setRealMeasurementData(sensor->getRealMeasurementData());
  sensorLocal->setStandardMask();
  _sensors.push_back(sensorLocal);
  _pushMutex.unlock();
  this->unblock();
}

void ThreadMapping::terminateThread(void)
{
  _pushMutex.lock();
  std::cout << __PRETTY_FUNCTION__ << "saving mapping iter times" << std::endl;
  std::ofstream stream;
     stream.open("/tmp/mapiter.log", std::ofstream::out);
     if(!stream.is_open())
       std::cout << __PRETTY_FUNCTION__ << "Error opening file" << "/tmp/mapiter.log" <<std::endl;
     for(std::vector<ros::Duration>::iterator iter = _iterTimes.begin(); iter < _iterTimes.end(); iter++)
     {
       stream << iter->toSec() << std::endl;
     }
     stream.close();
     _pushMutex.unlock();
  _stayActive = false;
  this->unblock();
}

} /* namespace */
