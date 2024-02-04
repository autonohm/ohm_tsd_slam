#include "ThreadGrid.h"
// #include "SlamNode.h"

#include <functional>
#include <memory>
#include <string>

#include <sensor_msgs/image_encodings.hpp>

#include "obvision/reconstruct/grid/RayCastAxisAligned2D.h"
#include "obcore/base/Logger.h"


namespace ohm_tsd_slam
{
ThreadGrid::ThreadGrid(obvious::TsdGrid* grid, const std::shared_ptr<rclcpp::Node>& node, const double xOffset, const double yOffset):
        ThreadSLAM(*grid),
        _node(node),
        _occGrid(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
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
  _occGrid->info.origin.orientation.w = 1.0;
  _occGrid->info.origin.orientation.x = 0.0;
  _occGrid->info.origin.orientation.y = 0.0;
  _occGrid->info.origin.orientation.z = 0.0;
  _occGrid->info.origin.position.x    = -(static_cast<double>(_grid.getCellsX()) * static_cast<double>(_grid.getCellSize()) * 0.5 + xOffset);
  _occGrid->info.origin.position.y    = -(static_cast<double>(_grid.getCellsY()) * static_cast<double>(_grid.getCellSize()) * 0.5 + yOffset);
  _occGrid->info.origin.position.z    = 0.0;
  _occGrid->data.resize(_grid.getCellsX() * _grid.getCellsY());

  node->declare_parameter<bool>("pub_tsd_color_map", true);
  // node->declare_parameter<std::string>("map_topic", "map");
  // node->declare_parameter<std::string>("get_map_topic", "map");
  // node->declare_parameter<std::string>("topic_tsd_color_map", "tsd");
  node->declare_parameter<int>("object_inflation_factor", 2);
  node->declare_parameter<bool>("use_object_inflation", false);

  _occGrid->header.frame_id = node->get_parameter("tf_map_frame").as_string();

  _objectInflation = node->get_parameter("use_object_inflation").as_bool(); //toDo: exchange with if inflation > 0
  _objInflateFactor = static_cast<unsigned int>(node->get_parameter("object_inflation_factor").as_int());

  const std::string node_name = _node->get_name();
  _gridPub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(node_name + "/map", rclcpp::QoS(1).reliable().transient_local());
  _pubColorImage = node->create_publisher<sensor_msgs::msg::Image>(node_name + "/map/image", rclcpp::QoS(1).best_effort());

  _getMapServ = node->create_service<nav_msgs::srv::GetMap>(
    node_name + "/get_map",
    std::bind(&ThreadGrid::getMapServCallBack, this, std::placeholders::_1, std::placeholders::_2)
  );
}

ThreadGrid::~ThreadGrid()
{
  _stayActive = false;
  _thread->join();
  delete[] _occGridContent;
  delete[] _gridCoords;
}

void ThreadGrid::eventLoop()
{
  sensor_msgs::msg::Image image;
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
      
      RCLCPP_WARN(_node->get_logger(), "OccupancyGridThread: Warning! Raycasting returned with no coordinates, map contains no data yet!\n");

    _occGrid->header.stamp       = _node->get_clock()->now();
    _occGrid->info.map_load_time = _node->get_clock()->now();
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
    _gridPub->publish(*_occGrid);
    image.header.stamp = _occGrid->header.stamp;
    image.height = _occGrid->info.height;
    image.width = _occGrid->info.width;
    image.encoding = sensor_msgs::image_encodings::RGB8;

    _grid.grid2ColorImage(colorBuffer, _grid.getCellsX(), _grid.getCellsY());
    for(unsigned int i = 0; i < _grid.getCellsX() * _grid.getCellsY() * 3; i++)
    {
      image.data[i] = colorBuffer[i];
    }
    _pubColorImage->publish(image);
  }
  delete[] colorBuffer;
}

bool ThreadGrid::getMapServCallBack(const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
                                    std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
{
  res->map = *_occGrid;
  res->map.header.stamp = _node->get_clock()->now();
  _occGrid->info.map_load_time = _node->get_clock()->now();
  return(true);
}

} /* namespace */
