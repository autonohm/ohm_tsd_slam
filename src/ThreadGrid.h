/*
 * ThreadGrid.h
 *
 * Refactored by: Christian Wendt
 */
#pragma once

#include "ThreadSLAM.h"

#include "obvision/reconstruct/grid/TsdGrid.h"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

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
  ThreadGrid(obvious::TsdGrid* grid, const std::shared_ptr<rclcpp::Node>& node, const double xOffset, const double yOffset);

  /**
   * Destructor
   */
  virtual ~ThreadGrid();

protected:

  /**
   * eventLoop
   * Thread event loop
   */
  virtual void eventLoop();

private:

  /**
   * getMapServCallBack
   * Ros service callback method for the get map service
   * @param req Request
   * @param res Response
   * @return true in case of success
   */
  bool getMapServCallBack(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                          std::shared_ptr<nav_msgs::srv::GetMap::Response> res);

  /**
   * Node instance used for communication and parameter handling.
   */
  std::shared_ptr<rclcpp::Node> _node;

  /**
   * Occupancy grid
   */
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> _occGrid;

  /**
   * Ros get map service
   */
  std::shared_ptr<rclcpp::Service<nav_msgs::srv::GetMap>> _getMapServ;

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
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> _gridPub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pubColorImage;

  bool _pubTsdColorMap;

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
