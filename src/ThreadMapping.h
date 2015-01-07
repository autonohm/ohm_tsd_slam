#ifndef THREADMAPPING_H_
#define THREADMAPPING_H_

#include "ThreadSLAM.h"

#include "sensor_msgs/LaserScan.h"

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"

#include <boost/thread.hpp>
#include <queue>

namespace ohm_tsd_slam
{

class SlamNode;

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

  bool initialized(void);

  /**
   * initPush
   * method to init the grid from a certain pose. Is done by the CALLING thread
   * @param sensor initial data
   */
  void initPush(obvious::SensorPolar2D* sensor);

  void posePush(obvious::Matrix& pose, const ros::Time& timeStamp, const std::string& nameSpace);

protected:

  /**
   * eventLoop
   * Thread event loop
   */
  virtual void eventLoop(void);

private:

  void multiRobotPoseFilter(obvious::SensorPolar2D& sensor, const std::string& nameSpace);

  struct StampedPose
  {
	  obvious::Matrix _pose;
	  ros::Time       _stamp;
	  std::string     _nameSpace;
	  double          _height;
	  double          _width;
  };

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

  bool _initialized;

  std::vector<StampedPose*> _robotPoses;

  boost::mutex _poseMutex;
};

} /* namespace */

#endif /* THREADMAPPING_H_ */
