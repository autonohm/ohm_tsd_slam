#ifndef THREADMAPPING_H_
#define THREADMAPPING_H_

#include "ThreadSLAM.h"
#include "SlamNode.h"

#include "sensor_msgs/LaserScan.h"

#include "obvision/reconstruct/grid/SensorPolar2D.h"

#include <boost/thread.hpp>
#include <queue>

namespace ohm_tsd_slam
{

/**
 * @class ThreadMapping
 * @author Philipp Koch, Stefan May
 * @date 08.06.2014
 */
class ThreadMapping : public ThreadSLAM
{
public:

  ThreadMapping(obvious::TsdGrid* grid);

  virtual ~ThreadMapping();

  void queuePush(obvious::SensorPolar2D* sensor);

protected:

  virtual void eventLoop(void);

private:

  obvious::TsdGrid* _grid;

  std::queue<obvious::SensorPolar2D*> _sensors;

  boost::mutex _pushMutex;
};

} /* namespace */

#endif /* THREADMAPPING_H_ */
