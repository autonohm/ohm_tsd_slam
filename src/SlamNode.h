/*
 * SlamNode.h
 *
 * Refactored by: Christian Wendt
 */
#pragma once

#include <functional>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>

#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include "ThreadLocalize.h"
#include "ohm_tsd_slam/srv/start_stop_slam.hpp"

#define INIT_PSHS 1        //number of initial pushes into the grid
#define THREAD_TERM_MS 1   //time in ms waiting for thread to terminate




namespace ohm_tsd_slam
{
class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;
//class ThreadLocalize;

struct TaggedSubscriber
{
  TaggedSubscriber(std::string topic, ThreadLocalize& localizer, const std::shared_ptr<rclcpp::Node>& node)
  : _topic(topic),
    _localizer(&localizer),
    _node(node)
  {}
  TaggedSubscriber() = default;
  bool topic(const std::string topic)
  {
    return topic == _topic;
  }
  void switchOff()
  {
    _subs = nullptr;
  }
  void switchOn()
  {
    _subs = _node->create_subscription<sensor_msgs::msg::LaserScan>(
      _topic,
      rclcpp::QoS(2).best_effort(), 
      std::bind(&ThreadLocalize::laserCallBack, _localizer, std::placeholders::_1)
    );
  }
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> _subs;
  std::string _topic;
  ThreadLocalize* _localizer = nullptr;
  std::shared_ptr<rclcpp::Node> _node;
};

/**
 * @class SlamNode
 * @brief Main node management of the 2D SLAM
 * @author Philipp Koch
 */
class SlamNode : public rclcpp::Node
{
public:

  /**
   * Default constructor
   */
  SlamNode();

  /**
   * Destructor
   */
  virtual ~SlamNode();

  /**
   * @brief Initialize class. It can't be called in contructor.
   * 
   */
  void initialize();

private:

  /**
   * timedGridPub
   * Enables occupancy grid thread with certain frequency
   */
  void timedGridPub();

  bool callBackServiceStartStopSLAM(const std::shared_ptr<ohm_tsd_slam::srv::StartStopSLAM::Request> req,
                                    std::shared_ptr<ohm_tsd_slam::srv::StartStopSLAM::Response> res);

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
  std::shared_ptr<rclcpp::Duration> _gridInterval;

  /**
   * Desired loop rate
   */
  std::shared_ptr<rclcpp::Duration> _loopRate;
  std::shared_ptr<rclcpp::TimerBase> _timer;

  /**
   * Ros laser subscriber
   */
  std::vector<TaggedSubscriber> _subsLaser;

  /**
   * Localizing threads
   */
  std::vector<ThreadLocalize*> _localizers;

  std::shared_ptr<rclcpp::Service<ohm_tsd_slam::srv::StartStopSLAM>> _serviceStartStopSLAM;
};

} /* namespace ohm_tsd_slam */
