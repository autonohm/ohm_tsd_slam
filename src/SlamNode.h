#ifndef SLAMNODE_H_
#define SLAMNODE_H_

#include <ros/ros.h>

#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include "ThreadLocalize.h"
#include "ohm_tsd_slam/StartStopSLAM.h"

#define INIT_PSHS 1      //number of initial pushes into the grid
#define THREAD_TERM_MS 1   //time in ms waiting for thread to terminate




namespace ohm_tsd_slam
{
class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;
//class ThreadLocalize;

struct TaggedSubscriber
{
  TaggedSubscriber(std::string topic, ThreadLocalize& localizer, ros::NodeHandle& nh):
    _topic(topic),
    _localizer(&localizer),
    _nh(&nh)
  {}
  TaggedSubscriber(const TaggedSubscriber& subs):
    _subs(subs._subs),
    _topic(subs._topic),
    _localizer(subs._localizer),
    _nh(subs._nh)
  {}
  TaggedSubscriber(void):
    _localizer(NULL),
    _nh(NULL)
  {}
  bool topic(const std::string topic)
  {
    return topic == _topic;
  }
  void switchOff(void)
  {
    _subs.shutdown();
  }
  void switchOn(void)
  {
    _subs = _nh->subscribe(_topic, 1, &ThreadLocalize::laserCallBack, _localizer);
  }
  ros::Subscriber _subs;
  std::string _topic;
  ThreadLocalize* _localizer;
  ros::NodeHandle* _nh;
};

/**
 * @class SlamNode
 * @brief Main node management of the 2D SLAM
 * @author Philipp Koch
 */
class SlamNode
{
public:

  /**
   * Default constructor
   */
  SlamNode(void);

  /**
   * Destructor
   */
  virtual ~SlamNode();

  /**
   * start
   * Method to start the SLAM
   */
  void start(void){this->run();}
private:

  /**
   * run
   * Main SLAM method
   */
  void run(void);

  /**
   * timedGridPub
   * Enables occupancy grid thread with certain frequency
   */
  void timedGridPub(void);

  bool callBackServiceStartStopSLAM(ohm_tsd_slam::StartStopSLAM::Request& req, ohm_tsd_slam::StartStopSLAM::Response& res);

  /**
   * Main node handle
   */
  ros::NodeHandle _nh;

  /**
   * Representation
   */
  obvious::TsdGrid* _grid;

  /**
   * Mapping thread instance
   */
  ThreadMapping* _threadMapping;

  /**
   * Grid thread instance
   */
  ThreadGrid* _threadGrid;

  /**
   * X starting offset factor
   */
  double _xOffFactor;

  /**
   * Y starting offset factor
   */
  double _yOffFactor;

  /**
   * Rate used for occupancy grid generation
   */
  ros::Duration* _gridInterval;

  /**
   * Desired loop rate
   */
  ros::Rate* _loopRate;

  /**
   * Ros laser subscriber
   */
  std::vector<TaggedSubscriber> _subsLaser;

  /**
   * Localizing threads
   */
  std::vector<ThreadLocalize*> _localizers;

  ros::ServiceServer _serviceStartStopSLAM;
};

} /* namespace ohm_tsd_slam */

#endif /* SLAMNODE_H_ */
