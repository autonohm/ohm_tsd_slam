#ifndef THREADGRID_H_
#define THREADGRID_H_

#include "ThreadSLAM.h"

#include "obvision/reconstruct/grid/TsdGrid.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

namespace ohm_tsd_slam
{

/**
 * @class ThreadGrid
 * @brief Class implementing a thread generating an occupancy grid
 * @author Philipp Koch, Stefan May
 */
class ThreadGrid : public ThreadSLAM
{
public:

  /**
   * Constructor
   * @param grid Representation
   * @param nh Ros nodehandle
   * @param parentNode Pointer to main mapping instance
   */
  ThreadGrid(obvious::TsdGrid* grid, ros::NodeHandle* const nh, const double xOffset, const double yOffset);

  /**
   * Destructor
   */
  virtual ~ThreadGrid();

protected:

  /**
   * eventLoop
   * Thread event loop
   */
  virtual void eventLoop(void);

private:

  /**
   * getMapServCallBack
   * Ros service callback method for the get map service
   * @param req Request
   * @param res Response
   * @return true in case of success
   */
  bool getMapServCallBack(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  /**
   * Occupancy grid
   */
  nav_msgs::OccupancyGrid* _occGrid;

  /**
   * Ros get map service
   */
  ros::ServiceServer _getMapServ;

  /**
   * Buffer for occupancy grid content
   */
  char* _occGridContent;

  /**
   * Buffer for grid coordinates
   */
  double* _gridCoords;

  /**
   * Grid dimension
   */
  unsigned int _width;

  /**
   * Grid dimension
   */
  unsigned int _height;

  /**
   * Grid resolution
   */
  double _cellSize;

  /**
   * Occupancy grid publisher
   */
  ros::Publisher _gridPub;

  /**
   * Object inflation factor
   */
  unsigned int _objInflateFactor;

  /**
   * Object inflation control flag
   */
  bool _objectInflation;
};

} /* namespace ohm_tsd_slam */

#endif /* THREADGRID_H_ */
