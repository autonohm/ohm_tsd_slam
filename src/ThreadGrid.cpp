#include "ThreadGrid.h"
#include "SlamNode.h"

#include <string>

#include "obvision/reconstruct/grid/RayCastAxisAligned2D.h"
#include "obcore/base/Logger.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


namespace ohm_tsd_slam
{
ThreadGrid::ThreadGrid(obvious::TsdGrid* grid, ros::NodeHandle* const nh, const double xOffset, const double yOffset):
        ThreadSLAM(*grid),
        _occGrid(new nav_msgs::OccupancyGrid),
        _occGridContent(new char[grid->getCellsX() * grid->getCellsY()]),
        _gridCoords(new double[grid->getCellsX() * grid->getCellsY()]),
        _width(grid->getCellsX()),
        _height(grid->getCellsY()),
        _cellSize(grid->getCellSize())

{
  for(unsigned int i = 0; i < _grid.getCellsX() * _grid.getCellsY(); ++i)
    _occGridContent[i] = -1;

  _occGrid->info.resolution           = static_cast<double>(_grid.getCellSize());
  _occGrid->info.width                = _grid.getCellsX();
  _occGrid->info.height               = _grid.getCellsY();
  _occGrid->info.origin.orientation.w = 0.0;
  _occGrid->info.origin.orientation.x = 0.0;
  _occGrid->info.origin.orientation.y = 0.0;
  _occGrid->info.origin.orientation.z = 0.0;
  _occGrid->info.origin.position.x    = 0.0 - (static_cast<double>(_grid.getCellsX()) * static_cast<double>(_grid.getCellSize()) * 0.5 + xOffset);
  _occGrid->info.origin.position.y    = 0.0 - (static_cast<double>(_grid.getCellsY()) * static_cast<double>(_grid.getCellSize()) * 0.5 + yOffset);
  _occGrid->info.origin.position.z    = 0.0;
  _occGrid->data.resize(_grid.getCellsX() * _grid.getCellsY());

  ros::NodeHandle prvNh("~");
  std::string mapTopic;
  std::string getMapTopic;
  std::string topicTsdColorMap;
  std::string tfBaseFrame;
  int intVar         = 0;
  double interval = 0;
  prvNh.param<bool>("pub_tsd_color_map", _pubTsdColorMap, true);
  prvNh.param("map_topic", mapTopic, std::string("map"));
  prvNh.param("get_map_topic", getMapTopic, std::string("map"));
  prvNh.param<std::string>("topic_tsd_color_map", topicTsdColorMap, "tsd");
  prvNh.param<std::string>("tf_base_frame", tfBaseFrame, "map");
  _occGrid->header.frame_id = tfBaseFrame;

  prvNh.param<int>("object_inflation_factor", intVar, 0);
 // prvNh.param<bool>("use_object_inflation", _objectInflation, false);  //toDo: exchange with if inflation > 0
  //_objectInflation
  prvNh.param<double>("map_publish_interval", interval, 0.5);

  _gridPub          = nh->advertise<nav_msgs::OccupancyGrid>(mapTopic, 1);
  _pubColorImage = nh->advertise<sensor_msgs::Image>(topicTsdColorMap, 1);
  _getMapServ       = nh->advertiseService(getMapTopic, &ThreadGrid::getMapServCallBack, this);
  _objInflateFactor = static_cast<unsigned int>(intVar);
  _timerMain = nh->createTimer(ros::Duration(interval), &ThreadGrid::callBackTimerMain, this);
}

ThreadGrid::~ThreadGrid()
{
  _stayActive = false;
  _thread->join();
  delete _occGrid;
  delete _occGridContent;
  delete _gridCoords;
}

void ThreadGrid::eventLoop(void)
{
  static unsigned int frameId = 0;
  sensor_msgs::Image image;
  image.header.frame_id = "map";
  unsigned char* colorBuffer = new unsigned char[_grid.getCellsX() * _grid.getCellsY() * 3];
  image.data.resize(_grid.getCellsX() * _grid.getCellsY() * 3);
  image.step = _grid.getCellsY() * 3;
  while(_stayActive)
  {
    _sleepCond.wait(_sleepMutex);
    unsigned int mapSize = 0;
    obvious::RayCastAxisAligned2D raycasterMap;
    raycasterMap.calcCoords(&_grid, _gridCoords, NULL, &mapSize, _occGridContent);
    if(mapSize == 0)
      ROS_INFO_STREAM("OccupancyGridThread: Warning! Raycasting returned with no coordinates, map contains no data yet!\n");

    _occGrid->header.stamp       = ros::Time::now();
    _occGrid->header.seq         = frameId++;
    _occGrid->info.map_load_time = ros::Time::now();
    const unsigned int gridSize        = _width * _height;

    for(unsigned int i = 0; i < gridSize ; ++i)
      _occGrid->data[i] = _occGridContent[i];

    for(unsigned int i = 0; i < mapSize / 2; i++)
    {
      double x       = _gridCoords[2*i];
      double y       = _gridCoords[2*i+1];
      unsigned int u = static_cast<unsigned int>(round(x / _cellSize));
      unsigned int v = static_cast<unsigned int>(round(y / _cellSize));
      if(u > 0 && u < _width && v > 0 && v < _height)
      {
        _occGrid->data[v * _width + u] = 100;               //set grid cell to occupied
        if(_objInflateFactor > 0)
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
    image.header.stamp = ros::Time::now();
    image.header.seq++;
    image.height = _occGrid->info.height;
    image.width = _occGrid->info.width;
    image.encoding = sensor_msgs::image_encodings::RGB8;

    _grid.grid2ColorImage(colorBuffer, _grid.getCellsX(), _grid.getCellsY());
    for(unsigned int i = 0; i < _grid.getCellsX() * _grid.getCellsY() * 3; i++)
    {
      image.data[i] = colorBuffer[i];
    }
    _pubColorImage.publish(image);
  }
  delete colorBuffer;
}

bool ThreadGrid::getMapServCallBack(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
  static unsigned int frameId = 0;
  res.map = *_occGrid;
  res.map.header.stamp = ros::Time::now();
  _occGrid->header.seq = frameId++;
  _occGrid->info.map_load_time = ros::Time::now();
  return(true);
}

void ThreadGrid::callBackTimerMain(const ros::TimerEvent& ev)
{
  this->unblock();
}

} /* namespace */
