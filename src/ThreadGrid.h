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

  bool getMapServCallBack(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  nav_msgs::OccupancyGrid* _occGrid;

  ros::ServiceServer _getMapServ;

  char* _occGridContent;

  double* _gridCoords;

  unsigned int _width;

  unsigned int _height;

  double _cellSize;

  ros::Publisher _gridPub;

  obvious::TsdGrid* _grid;

  boost::mutex* _pubMutex;

  unsigned int _objInflateFactor; //< Factor to inflate the objects 0 = no inflation, 1 = 1 cell radius
};

} /* namespace */

#endif /* THREADGRID_H_ */
