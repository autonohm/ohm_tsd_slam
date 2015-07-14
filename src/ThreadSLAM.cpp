#include "ThreadSLAM.h"

namespace ohm_tsd_slam
{

ThreadSLAM::ThreadSLAM(obvious::TsdGrid& grid):
        _stayActive(true),
        _thread(new boost::thread(&ThreadSLAM::eventLoop, this)),
        _grid(grid)
{

}

ThreadSLAM::~ThreadSLAM()
{
  delete _thread;
}

void ThreadSLAM::unblock(void)
{
  _sleepCond.notify_all();
}

bool ThreadSLAM::alive(unsigned int ms)
{
  return(_thread->timed_join(boost::posix_time::milliseconds(ms)));
}

void ThreadSLAM::terminateThread(void)
{
  _stayActive = false;
  this->unblock();
}

} /* namespace */
