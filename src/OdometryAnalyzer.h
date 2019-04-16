/*
 * OdometryAnalyzer.h
 *
 *  Created on: Dec 6, 2018
 *      Refactured by: jasmin
 */

#ifndef ODOMETRYANALYZER_H_
#define ODOMETRYANALYZER_H_

#include <ros/ros.h>

#include "ThreadLocalize.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>


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
  OdometryAnalyzer(obvious::TsdGrid& grid);

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
  obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf);

  /**
   * Method to analyze 2D transformation matrix.
   * @param T Pointer to transformation matrix
   * @return Calculated angle
   */
  double calcAngle(obvious::Matrix* T);

  //reference to tsdgrid representation
  obvious::TsdGrid& _grid;

  //pointer to ThreadLocalize to use its calcAngle method

  tf::TransformListener _tfListener;
  //Container for reading tfs
  tf::StampedTransform _tfReader;

  //ros tf frame ids
  std::string _tfFootprintFrameId;
  std::string _tfOdomFrameId;
  std::string _tfBaseFrameId;
  std::string _tfChildFrameId;

  //Transform from base footprint to laser
  tf::Transform _tfLaser;

  //Odom Transforms
  tf::Transform _tfOdomOld;
  tf::Transform _tfOdom;
  tf::Transform _tfRelativeOdom;

  //Laser time stamps
  ros::Time _stampLaser;
  ros::Time _stampLaserOld;

  //state of current odom tf
  bool _odomTfIsValid;

  //time to wait for synced odom tf
  ros::Duration _waitForOdomTf;

  //ICP translation threshold
  double _trnsMax;
  double _trnsVelocityMax;

  //ICP rotation threshold
  double _rotMax;
  double _rotVelocityMax;

};

} /* namespace ohm_tsd_slam_ref */

#endif /* ODOMETRYANALYZER_H_ */
