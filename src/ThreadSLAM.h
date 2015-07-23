#ifndef THREADSLAM_H_
#define THREADSLAM_H_

#include <boost/thread.hpp>
#include "obvision/reconstruct/grid/TsdGrid.h"

#include <ros/ros.h>

namespace ohm_tsd_slam
{

/**
 * @class ThreadSLAM
 * @brief Base class implementing boost thread functionality
 * @author Philipp Koch, Stefan May
 */
class ThreadSLAM
{
public:

  /**
   * Constructor
   */
  ThreadSLAM(obvious::TsdGrid& grid);

  /**
   * Destructor
   */
  virtual ~ThreadSLAM();

  /**
   * unblock
   * Method to set the thread from sleep to run mode
   */
  void unblock(void);

  /**
   * alive
   * Method to determine the state of the thread. Function tries to call the thread until the given time runs out
   * @param ms Wait time
   * @return success
   */
  bool alive(unsigned int ms);

  /**
   * terminateThread
   * Function to terminate the thread
   */
  void terminateThread(void);
protected:

  /**
   * eventLoop
   * Abstract method connected to the boost threading functionality
   */
  virtual void eventLoop(void) = 0;

  /**
   * Boost threading object
   */
  boost::thread* _thread;

  /**
   * Boost sleeping mutex
   */
  boost::mutex _sleepMutex;

  /**
   * Boost condition variable for sleeping mode
   */
  boost::condition_variable_any _sleepCond;

  /**
   * Shutdown flag
   */
  bool _stayActive;

  /**
   * Reference to representation
   */
  obvious::TsdGrid& _grid;
};

} /* namespace ohm_tsd_slam */

#endif /* THREADSLAM_H_ */
