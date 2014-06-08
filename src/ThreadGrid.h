#ifndef THREADGRID_H_
#define THREADGRID_H_

#include "ThreadSLAM.h"

#include "obvision/reconstruct/grid/TsdGrid.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace ohm_tsd_slam
{

/**
 * @class ThreadGrid
 * @author Philipp Koch, Stefan May
 * @date 08.06.2014
 */
class ThreadGrid : public ThreadSLAM
{
public:

  ThreadGrid(obvious::TsdGrid* grid, ros::NodeHandle nh, boost::mutex* pubMutex);

  virtual ~ThreadGrid();

protected:

  virtual void eventLoop(void);

private:

  nav_msgs::OccupancyGrid* _occGrid;

  char* _occGridContent;

  double* _gridCoords;

  unsigned int _width;

  unsigned int _height;

  double _cellSize;

  ros::Publisher _gridPub;

  obvious::TsdGrid* _grid;

  boost::mutex* _pubMutex;
};

} /* namespace */

#endif /* THREADGRID_H_ */
