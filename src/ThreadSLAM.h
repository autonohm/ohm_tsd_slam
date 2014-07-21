#ifndef THREADSLAM_H_
#define THREADSLAM_H_

#include <boost/thread.hpp>
//#include "obvision/reconstruct/grid/TsdGrid.h"

namespace ohm_tsd_slam
{

/**
 * @class ThreadSLAM
 * @author Philipp Koch, Stefan May
 * @date 08.06.2014
 */
class ThreadSLAM
{
public:
  ThreadSLAM();
  virtual ~ThreadSLAM();
  void unblock(void);
  bool alive(unsigned int ms);
  void terminateThread(void);
protected:
  virtual void eventLoop(void) = 0;

  boost::thread* _thread;
  boost::mutex _sleepMutex;
  boost::condition_variable_any _sleepCond;
  bool _stayActive;
};

} /* namespace */

#endif /* THREADSLAM_H_ */
