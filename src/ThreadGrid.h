#ifndef THREADGRID_H_
#define THREADGRID_H_

#include "ThreadSLAM.h"
#include "SlamNode.h"

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
   * @param pubMutex Publising mutex publishing mutex
   * @param parentNode Pointer to main mapping instance
   */
  ThreadGrid(obvious::TsdGrid* grid, ros::NodeHandle nh, boost::mutex* pubMutex, SlamNode& parentNode);

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
   * @return success
   */
  bool getMapServCallBack(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  /**
   * freeInitialArea
   * Method to empty footprint of robot
   */
  void freeInitialArea(void);

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
   * Representation
   */
  obvious::TsdGrid* _grid;

  /**
   * Publishing mutex
   */
  boost::mutex* _pubMutex;

  /**
   * Object inflation factor
   */
  unsigned int _objInflateFactor;

  /**
   * Object inflation control flag
   */
  bool _objectInflation;

  /**
   * Length of area marked as free around start position (cells)
   */
  unsigned int _robotLength;

  /**
   * Width of area marked as free around start position (cells)
   */
  unsigned int _robotWidth;

  /**
   * X-coordinate of initial position (cells)
   */
  unsigned int _initialX;

  /**
   * Y-coordinate of initial position (cells)
   */
  unsigned int _initialY;
};

} /* namespace ohm_tsd_slam */

#endif /* THREADGRID_H_ */
