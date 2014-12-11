#include "ThreadGrid.h"
#include "SlamNode.h"

#include <string>

#include "obvision/reconstruct/grid/RayCastAxisAligned2D.h"
#include "obcore/base/Logger.h"


namespace ohm_tsd_slam
{

ThreadGrid::ThreadGrid(obvious::TsdGrid* grid, ros::NodeHandle nh, const double xOffFactor, const double yOffFactor)
{
  _grid           = grid;
  _occGridContent = new char[grid->getCellsX() * grid->getCellsY()];
  _gridCoords     = new double[grid->getCellsX() * grid->getCellsY()];
  _width          = grid->getCellsX();
  _height         = grid->getCellsY();
  _cellSize       = grid->getCellSize();
  for(unsigned int i = 0; i < _grid->getCellsX() * _grid->getCellsY(); ++i)
    _occGridContent[i] = -1;

  _occGrid = new nav_msgs::OccupancyGrid;
  _occGrid->info.resolution = static_cast<double>(_grid->getCellSize());
  _occGrid->info.width = _grid->getCellsX();
  _occGrid->info.height = _grid->getCellsY();
  _occGrid->info.origin.orientation.w = 0.0;
  _occGrid->info.origin.orientation.x = 0.0;
  _occGrid->info.origin.orientation.y = 0.0;
  _occGrid->info.origin.orientation.z = 0.0;
  _occGrid->info.origin.position.x = 0.0 - _grid->getCellsX() * _grid->getCellSize() * xOffFactor;
  _occGrid->info.origin.position.y = 0.0 - _grid->getCellsY() * _grid->getCellSize() * yOffFactor;
  _occGrid->info.origin.position.z = 0.0;
  _occGrid->data.resize(_grid->getCellsX() * _grid->getCellsY());

  ros::NodeHandle prvNh("~");
  std::string mapTopic;
  std::string getMapTopic;
  int intVar         = 0;

  prvNh.param("map_topic", mapTopic, std::string("map"));
  prvNh.param("get_map_topic", getMapTopic, std::string("map"));
  prvNh.param<int>("object_inflation_factor", intVar, 2);
  prvNh.param<bool>("use_object_inflation", _objectInflation, false);

  _gridPub          = nh.advertise<nav_msgs::OccupancyGrid>(mapTopic, 1);
  _getMapServ       = nh.advertiseService(getMapTopic, &ThreadGrid::getMapServCallBack, this);
  _objInflateFactor = static_cast<unsigned int>(intVar);
}

ThreadGrid::~ThreadGrid()
{
  _thread->join();
  delete _occGrid;
  delete _occGridContent;
  delete _gridCoords;
}

void ThreadGrid::eventLoop(void)
{
  unsigned int frameId = 0;
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);
    unsigned int mapSize = 0;
    obvious::RayCastAxisAligned2D raycasterMap;
    raycasterMap.calcCoords(_grid, _gridCoords, NULL, &mapSize, _occGridContent);
    if(mapSize == 0)
    {
      std::cout << __PRETTY_FUNCTION__ << " error! Raycasting returned with no coordinates!\n";
    }
    _occGrid->header.stamp       = ros::Time::now();
    _occGrid->header.seq         = frameId++;
    _occGrid->info.map_load_time = ros::Time::now();
    unsigned int gridSize        = _width * _height;

    for(unsigned int i = 0; i < gridSize ; ++i)
    {
      _occGrid->data[i] = _occGridContent[i];
    }
    for(unsigned int i = 0; i < mapSize / 2; i++)
    {
      double x       = _gridCoords[2*i];
      double y       = _gridCoords[2*i+1];
      unsigned int u = static_cast<int>(x / _cellSize);
      unsigned int v = static_cast<int>(y / _cellSize);
      if(u > 0 && u < _width && v > 0 && v < _height)
      {
        _occGrid->data[v * _width + u] = 100;               //set grid cell to occupied
        if(_objectInflation)
        {
          for(unsigned int i = v-_objInflateFactor; i < v + _objInflateFactor; i++)
          {
            for(unsigned int j = u - _objInflateFactor; j < u + _objInflateFactor; j++)
            {
              if((u >= _width) || (v >= _height))
                continue;
              _occGrid->data[i * _width + j] = 100;
            }
          }
        }
      }
    }
    _gridPub.publish(*_occGrid);
  }
}

bool ThreadGrid::getMapServCallBack(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
  static unsigned int frameId=0;
  res.map=*_occGrid;
  res.map.header.stamp=ros::Time::now();
  _occGrid->header.seq=frameId++;
  _occGrid->info.map_load_time=ros::Time::now();
  return(true);
}

} /* namespace */
