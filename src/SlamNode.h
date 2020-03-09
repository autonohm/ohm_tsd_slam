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
   * Localizing threads
   */
  std::vector<ThreadLocalize*> _localizers;  //TODO: unique ptrs

 // ros::ServiceServer _serviceStartStopSLAM;
};

} /* namespace ohm_tsd_slam */

#endif /* SLAMNODE_H_ */
