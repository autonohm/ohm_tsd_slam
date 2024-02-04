/*
 * OdometryAnalyzer.h
 *
 *  Created on: Dec 6, 2018
 *      Refactured by: jasmin
 *      Refactored by: Christian Wendt
 */
#pragma once

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include "obvision/reconstruct/grid/TsdGrid.h"

#include "ThreadLocalize.h"

namespace ohm_tsd_slam
{

/**
 * @class OdometryAnalyzer
 * @brief analyze and incorporate odometry as a rescue in case localization is lost
 * @todo add function call in ThreadLocalize if you want to use odometry for rescue purposes
 */
class OdometryAnalyzer
{
public:
  OdometryAnalyzer(obvious::TsdGrid& grid, const std::shared_ptr<rclcpp::Node>& node);

  virtual ~OdometryAnalyzer();

  /**
   * initialize OdometryAnalyzer module, get tf and map
   */
  void odomRescueInit();

  /**
   * updates odometry tf data if a new scan comes in
   */
  void odomRescueUpdate();

  /**
   * check if slam transformation is plausible; if not - overwrite transformation with odometry
   * @param T Transformation matrix to check and correct
   */
  void odomRescueCheck(obvious::Matrix& T);

private:
  /**
   * converts a 3x3 tf matrix to an obviously matrix
   * @param tf tf matrix to convert
   * @return obviously matrix
   */
  obvious::Matrix tfToObviouslyMatrix3x3(const tf2::Stamped<tf2::Transform>& tf);

  /**
   * Method to analyze 2D transformation matrix.
   * @param T Pointer to transformation matrix
   * @return Calculated angle
   */
  double calcAngle(obvious::Matrix* T);

  //reference to tsdgrid representation
  obvious::TsdGrid& _grid;

  //pointer to ThreadLocalize to use its calcAngle method

  std::unique_ptr<tf2_ros::TransformListener> _tfTransformListener;
  std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
  //Container for reading tfs
  geometry_msgs::msg::TransformStamped _tfReader;

  //ros tf frame ids
  std::string _tfFootprintFrameId;
  std::string _tfOdomFrameId;
  std::string _tfBaseFrameId;
  std::string _tfChildFrameId;

  //Transform from base footprint to laser
  tf2::Stamped<tf2::Transform> _tfLaser;

  //Odom Transforms
  tf2::Stamped<tf2::Transform> _tfOdomOld;
  tf2::Stamped<tf2::Transform> _tfOdom;
  tf2::Stamped<tf2::Transform> _tfRelativeOdom;

  //Laser time stamps
  rclcpp::Time _stampLaser;
  rclcpp::Time _stampLaserOld;

  //state of current odom tf
  bool _odomTfIsValid;

  //time to wait for synced odom tf
  rclcpp::Duration _waitForOdomTf;

  //ICP translation threshold
  double _trnsMax;
  double _trnsVelocityMax;

  //ICP rotation threshold
  double _rotMax;
  double _rotVelocityMax;

  std::shared_ptr<rclcpp::Node> _node;
};

} /* namespace ohm_tsd_slam_ref */
