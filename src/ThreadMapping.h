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
 * @brief Implements a thread updating an obvious::TsdGrid
 * @author Philipp Koch, Stefan May
 */
class ThreadMapping : public ThreadSLAM
{
public:

  /**
   * Constructor
   * @param grid Representation
   */
  ThreadMapping(obvious::TsdGrid* grid);

  /**
   * Destructor
   */
  virtual ~ThreadMapping();

  /**
   * queuePush
   * Method to add a top be pushed sensor to the queue (synchronized)
   * @param sensor New data
   */
  void queuePush(obvious::SensorPolar2D* sensor);

protected:

  /**
   * eventLoop
   * Thread event loop
   */
  virtual void eventLoop(void);

private:

  /**
   * Representation
   */
  obvious::TsdGrid* _grid;

  /**
   * Sensor queue
   */
  std::queue<obvious::SensorPolar2D*> _sensors;

  /**
   * Push mutex for queue
   */
  boost::mutex _pushMutex;
};

} /* namespace */

#endif /* THREADMAPPING_H_ */
