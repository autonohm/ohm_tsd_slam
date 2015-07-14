/*
 * SlamNode.h
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#ifndef SLAMNODE_H_
#define SLAMNODE_H_

#include "SlamBase.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <vector>

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obcore/base/Logger.h"

#include <boost/thread.hpp>

#define CELLSIZE 0.025   //toDo: Launch file parameters
#define TRUNCATION_RADIUS 3.0
#define MAX_RANGE 30.0
#define MIN_RANGE 0.01
#define THETA_INIT 0.0   //used in degrees
#define MAP_T 2.0        //time between map generations
#define INIT_PSHS 1      //number of initial pushes into the grid
#define LAS_OFFS_X -0.19 //offset of the laser scanner to the base footprint

namespace ohm_tsd_slam
{
class Localization;
class ThreadSLAM;
class ThreadMapping;
class ThreadGrid;
class ThreadLocalize;
class LaserCallBackObject;

/**
 * @class SlamNode
 * @brief Main node management of the 2D SLAM
 * @author Philipp Koch
 */
class SlamNode //: public SlamBase
{
public:

  /**
   * Default constructor
   */
  SlamNode(void);

  /**
   * Destructor
   */
  virtual ~SlamNode();

  /**
   * start
   * Method to start the SLAM
   */
  void start(void){this->run();}
  bool reset(void);
private:

  /**
   * initialize
   * Method to initialize the necessary parameters with the first received scan
   * @param initScan Initial scan
   */
//  void initialize(const sensor_msgs::LaserScan& initScan);

  /**
   * run
   * Main SLAM method
   */
  void run(void);

  /**
   * laserScanCallBack
   * Callback method to laser subscriber
   * @param scan Laser scan
   */
  //void laserScanCallBack(const sensor_msgs::LaserScan& scan);

//  bool pause(void);
//
//  bool unPause(void);

  void init(void);
    std::vector<ThreadLocalize*> _localizers;
    std::vector<LaserCallBackObject*> _laserCallBacks;

    void timedGridPub(void);
      bool resetGrid(void);
      ros::NodeHandle _nh;
      obvious::TsdGrid* _grid;
      ThreadMapping* _threadMapping;
      ThreadGrid* _threadGrid;
      boost::mutex _pubMutex;
      double _xOffFactor;
      double _yOffFactor;
      double _yawOffset;
      double _rateVar;
      ros::Rate* _loopRate;
      bool _initialized;
      ros::Duration* _gridInterval;
      //bool _icpSac;
      unsigned int _octaveFactor;
      double _truncationRadius;
      double _cellSize;

  /**
   * Main node handle
   */
  //ros::NodeHandle _nh;

  /**
   * Laser subscriber
   */
 // ros::Subscriber _laserSubs;

  /**
   * Initilized flag
   */
  //bool _initialized;

  /**
   * Representation
   */
  //obvious::TsdGrid* _grid;

  /**
   * obvious::Sensor instance containing data and pose
   */
//  obvious::SensorPolar2D* _sensor;

  /**
   * Mask for lasr data
   */
  //bool* _mask;

  /**
   * Localization instance
   */
//  Localization* _localizer;

  /**
   * Mapping thread instance
   */
  //ThreadMapping* _threadMapping;

  /**
   * Grid thread instance
   */
  //ThreadGrid* _threadGrid;

  /**
   * Publishing mutex
   */
  //boost::mutex _pubMutex;

  /**
   * X starting offset factor
   */
  //double _xOffFactor;

  /**
   * Y starting offset factor
   */
  //double _yOffFactor;

  /**
   * Starting yaw angle
   */
  //double _yawOffset;

  /**
   * Minimum range threshold
   */
  //float _minRange;

  /**
   * Maximum range threshold
   */
  double _maxRange;

  /**
   * Time interval between occupancy grid
   */
  //double _gridPublishInterval;

  /**
   * Desired loop rate
   */
  //double _loopRate;

  bool _pause;

  std::string _laserTopic;

  std::vector<ros::Subscriber> _subsLaser;

};

} /* namespace ohm_tsdSlam */

#endif /* SLAMNODE_H_ */
