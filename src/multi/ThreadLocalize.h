/*
 * ThreadLocalize.h
 *
 *  Created on: 28.10.2014
 *      Author: phil
 */

#ifndef THREADLOCALIZE_H_
#define THREADLOCALIZE_H_

#include "ThreadSLAM.h"

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"
#include "obvision/registration/icp/icp_def.h"

#include <string>

namespace ohm_tsd_slam
{

class MultiSlamNode;
class ThreadMapping;
class Localization;

class ThreadLocalize: public ThreadSLAM
{
  enum EnumRegModes
  {
    ICP = 0, ///< Registration with Icp only
    EXP,     ///< Experimental Registration scheme, use with caution
    ICP_EXP_RSC ///< Registration can be rescued by pre registration using ransac
  };

public:
  ThreadLocalize(obvious::TsdGrid* grid, ThreadMapping* mapper, ros::NodeHandle* nh, std::string nameSpace,
      const double xOffFactor, const double yOffFactor);
  virtual ~ThreadLocalize();
  bool setData(const sensor_msgs::LaserScan& scan);
protected:
  virtual void eventLoop(void);
private:
  void init(const sensor_msgs::LaserScan& scan);
  double calcAngle(obvious::Matrix* T);
  bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose);

  //bool togglePushServiceCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res); //toDo: not in release?
  obvious::Matrix doRegistration(obvious::SensorPolar2D* sensor,
        obvious::Matrix* M,
        obvious::Matrix* Mvalid,
        obvious::Matrix* N,
        obvious::Matrix* Nvalid,
        obvious::Matrix* S,
        obvious::Matrix* Svalid,
        const bool useRansac
    );
  bool isRegistrationError(obvious::Matrix* T, const double trnsMax, const  double rotMax);
  void sendTransform(obvious::Matrix* T);
  void sendNanTransform();

  ros::NodeHandle* _nh;
  ros::Subscriber _lasSubs;
  obvious::TsdGrid& _grid;
  ThreadMapping& _mapper;
  obvious::SensorPolar2D* _sensor;
  Localization* _localizer;
  bool _newScan;
  bool _initialized;
  std::string _nameSpace;
  double _gridWidth;
  double _gridHeight;
  double _xOffFactor;
  double _yOffFactor;
  bool* _mask;
  boost::mutex _dataMutex;
  const double _gridOffSetX;
  const double _gridOffSetY;

    /**
     * Buffer for model coordinates
     */
    double* _modelCoords;

    /**
     * Buffer for model normals
     */
    double* _modelNormals;

    /**
     * Mask of model
     */
    bool* _maskM;

    /**
     * reconstruction
     */
    obvious::RayCastPolar2D* _rayCaster;

    /**
     * Representation
     */

    /**
     * Buffer for scene coordinates
     */
    double* _scene;

    /**
     * Mask of scene
     */
    bool* _maskS;

    /**
     * ICP pair assigner
     */
    obvious::PairAssignment* _assigner;

    /**
     * ICP out of bounds filter
     */
    obvious::OutOfBoundsFilter2D* _filterBounds;

    /**
     * ICP distance filter
     */
    obvious::DistanceFilter* _filterDist;

    /**
     * ICP reciprogal filter
     */
    obvious::ReciprocalFilter* _filterReciprocal;

    /**
     * ICP transformation estimator
     */
    obvious::IRigidEstimator* _estimator;

    /**
     * ICP main icp instance
     */
    obvious::Icp* _icp;

    /**
     * ICP translation threshold
     */
    double _trnsMax;

    /**
     * ICP rotation threshold
     */
    double _rotMax;

    /**
     * Last pose
     */
    obvious::Matrix* _lastPose;

    /**
     * Ros pose publisher
     */
    ros::Publisher _posePub;

    /**
     * Ros current pose
     */
    geometry_msgs::PoseStamped _poseStamped;

    /**
     * Ros tf interface
     */
    tf::TransformBroadcaster _tfBroadcaster;

    /**
     * Ros current transform
     */
    tf::StampedTransform _tf;

    /**
     * Pointer to publishing mutex
     */
    //boost::mutex* _pubMutex;

    /**
     * Starting x offset
     */
    //double _xOffFactor;

    /**
     * Starting y offset
     */
    //double _yOffFactor;

    /**
     * RANSAC registration flag
     */
    //bool _ransac;
    bool _ranRescueActive;

    /*
     * RANSAC Reduction: Use to scale down the number of points for ransac
     * Example: Points in Scan = 1080
     *          ransacReduceFactor = 4
     *          -> points for ransac = 1080 /ransacReduceFactor = 270;
     */
    unsigned int _ransacReduceFactor;

    /**
     * Flag to disable push (active = no altering of the map)
     */
    bool _noPush;

    unsigned int _ranTrials;
    double _ranEpsThresh;
    unsigned int _ranSizeCtrlSet;

    /**
     * namespace for all topics and services
     */
    //std::string _nameSpace;

};

obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints);
void maskToOneDegreeRes(bool* const mask, const double resolution, const unsigned int maskSize);
void reduceResolution(bool* const maskIn, const obvious::Matrix* matIn, bool* const maskOut, obvious::Matrix* matOut,
    unsigned int pointsIn, unsigned int pointsOut, unsigned int reductionFactor);

} /* namespace ohm_tsd_slam */

#endif /* THREADLOCALIZE_H_ */
