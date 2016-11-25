/*
 * slam_3d_node.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: georg
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>

#include "obvision/reconstruct/reconstruct_defs.h"
#include "obvision/registration/icp/icp_def.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obvision/reconstruct/space/TsdSpace.h"
#include "obvision/reconstruct/space/SensorPolar3D.h"
#include "obvision/reconstruct/space/RayCast3D.h"
#include "obcore/base/tools.h"
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include "obcore/math/TransformationWatchdog.h"
#include "obvision/reconstruct/space/RayCastAxisAligned3D.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <cmath>

static tf::TransformListener* _listener;
static tf::TransformBroadcaster* _broadcaster;
static ros::Publisher _pubGlobPcl;
static ros::Subscriber _subPcl;
static obvious::TsdSpace* _space;
static obvious::RayCast3D* _rayCaster;
static obvious::SensorPolar3D* _sensor;
static obvious::Matrix* _T;
static obvious::Matrix _Tinit(4, 4);
static obvious::Icp* _icp;
static obvious::OutOfBoundsFilter3D* _filterBounds;
static double* _dist = NULL;
static bool*   _mask = NULL;
static double* _normals    = NULL;
static double* _coords     = NULL;
static obvious::TransformationWatchdog _TFwatchdog;

bool initialize(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
void resetMask(const unsigned int size);
bool callBackGenerateGlPcl(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

#define VXLDIM 0.05
#define LAYOUTPARTITION LAYOUT_64x64x64
#define LAYOUTSPACE LAYOUT_512x512x512


int main(int argc, char** argv)
{
  LOGMSG_CONF("mapper3D.log", Logger::screen_off | Logger::file_off, DBG_DEBUG, DBG_DEBUG);
  ros::init(argc, argv, "gen_2d_scan");
  ros::NodeHandle nh;
  _subPcl = nh.subscribe("velodyne_points", 1, callBackCloud);
  _pubGlobPcl = nh.advertise<sensor_msgs::PointCloud>("global_points", 1);
  ros::ServiceServer genGlblPcl = nh.advertiseService("gen_global_pcl",  callBackGenerateGlPcl);

  _listener = new tf::TransformListener;
  _broadcaster = new tf::TransformBroadcaster;

  _space = new TsdSpace(VXLDIM, LAYOUTPARTITION, LAYOUTSPACE);
  _space->setMaxTruncation(3.0 * VXLDIM);

  double tr[3];
  _space->getCentroid(tr);
  tr[2] = 3.0;
  double tf[16]={1,  0, 0, tr[0],
      0,  1, 0, tr[1],
      0,  0, 1, tr[2],
      0,  0, 0, 1};
  _Tinit.setData(tf);

  _TFwatchdog.setInitTransformation(_Tinit);
  _TFwatchdog.setRotationThreshold(0.08);
  _TFwatchdog.setTranslationThreshold(0.03);

  unsigned int maxIterations = 25;

  obvious::PairAssignment* assigner = (obvious::PairAssignment*)new obvious::FlannPairAssignment(3, 0.0, true);
  obvious::IRigidEstimator* estimator = (obvious::IRigidEstimator*)new obvious::PointToPlaneEstimator3D();
  _filterBounds = new obvious::OutOfBoundsFilter3D(_space->getMinX(), _space->getMaxX(), _space->getMinY(), _space->getMaxY(), _space->getMinZ(), _space->getMaxZ());
  _filterBounds->setPose(&_Tinit);
  assigner->addPreFilter(_filterBounds);

  obvious::IPostAssignmentFilter* filterD = (obvious::IPostAssignmentFilter*)new obvious::DistanceFilter(0.5, 0.01, maxIterations-3);
  obvious::IPostAssignmentFilter* filterR = (obvious::IPostAssignmentFilter*)new obvious::ReciprocalFilter();
  assigner->addPostFilter(filterD);
  assigner->addPostFilter(filterR);

  _icp = new obvious::Icp(assigner, estimator);

  // Deactivate early termination
  _icp->setMaxRMS(0.0);
  _icp->setMaxIterations(maxIterations);
  _icp->setConvergenceCounter(maxIterations);

  _sensor = NULL;
  _rayCaster = new RayCast3D();








  ros::spin();
  delete _listener;
  delete _broadcaster;
  delete _dist;
  delete _mask;
  delete _coords;
  delete _normals;
}


void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::cout << " callback " << std::endl;
  static bool initial = true;
  static unsigned int falseCtr = 0;
  pcl::PointCloud<pcl::PointXYZ> transformed;
  //  if(!pcl_ros::transformPointCloud("base", *msg, transformed, *_listener))
  //    return;
  if(initial)
  {
    initial = !initialize(msg);
    std::cout << " initialized " << std::endl;
    return;
  }
  ROS_INFO_STREAM("Current Transformation: ");
  Matrix T = _sensor->getTransformation();
  T.print();

  _filterBounds->setPose(&T);

  // Extract model from TSDF space
  unsigned int size = 0;
  _rayCaster->calcCoordsFromCurrentPose(_space, _sensor, _coords, _normals, NULL, &size);

  if(!size)
  {
    ROS_ERROR_STREAM("Error. Raycasting found no valid coordinates");
    if(falseCtr++ > 2)
      _subPcl.shutdown();
    return;
  }
  std::cout << " Raycaster returned " << size << " points " << std::endl;
  _icp->reset();
  _icp->setModel(_coords, _normals, size / 3, 0.1);

  unsigned int idx = 0;
  for(unsigned int i=0; i < msg->points.size(); i++)
  {
    _dist[i] = std::sqrt(msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z);
    if(_dist[i]>10e-3)
      _mask[i] = true;
    else
      _mask[i] = false;
    if(_mask[i])
    {
      _coords[3*idx    ] = msg->points[i].x;
      _coords[3*idx + 1] = msg->points[i].y;
      _coords[3*idx + 2] = msg->points[i].z;
      idx++;
    }
  }
  if(!idx)
  {
    ROS_ERROR_STREAM("Invalid scene");
    return;
  }
  _icp->setScene(_coords, NULL, idx, 0.04);
  double rms = 0;
  unsigned int pairs = 0;
  unsigned int iterations = 0;

  EnumIcpState state = _icp->iterate(&rms, &pairs, &iterations);
  if(((state == ICP_SUCCESS) && (rms < 0.1)) || ((state == ICP_MAXITERATIONS) && (rms < 0.1)))
  {
    // Obtain scene-to-model registration
    ROS_INFO_STREAM("Scene-to-model registration");
    Matrix T = _icp->getFinalTransformation();
    T.print();

    double Tdata[16];
    T.getData(Tdata);
    //_vScene->transform(Tdata);

    _sensor->transform(&T);
    ROS_INFO_STREAM("Current sensor transformation");
    Matrix Tmp = _sensor->getTransformation();
    Tmp.print();

    if(1)//_TFwatchdog.checkWatchdog(Tmp))
    {
      _sensor->setRealMeasurementData(_dist);
      _sensor->setRealMeasurementMask(_mask);
      _space->push(_sensor);
    }
    tf::Matrix3x3 R;
    for(unsigned int i = 0; i < 3; i++)
      R[i].setValue(T(i, 0), T(0, 1), T(i, 2));

    tf::Quaternion quat;
    R.getRotation(quat);

    tf::StampedTransform tf;
    tf.stamp_ = ros::Time::now();
    tf.child_frame_id_ = "velodyne";
    tf.frame_id_ = "map";
    tf.setRotation(quat);
    tf.setOrigin(tf::Vector3(T(0, 3), T(1, 3), T(2, 3)));
    _broadcaster->sendTransform(tf);
//      _poseStamped.pose.orientation.w = quat.w();
//      _poseStamped.pose.orientation.x = quat.x();
//      _poseStamped.pose.orientation.y = quat.y();
//      _poseStamped.pose.orientation.z = quat.z();
//
//    //  _tf.stamp_ = ros::Time::now();
//      _tf.stamp_ = _stampLaser;
//      _tf.setOrigin(tf::Vector3(posX, posY, 0.0));
//      _tf.setRotation(quat);
//
//      _posePub.publish(_poseStamped);



  }
  else
    ROS_ERROR_STREAM("Registration failed, RMS " << rms);


}

bool initialize(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::cout << __PRETTY_FUNCTION__ << " callback received cloud with " << msg->points.size() << " points " << std::endl;
//  resetMask(msg->points.size());
  if(!msg->points.size())
    return false;
  double thetaRes = obvious::deg2rad(1.3);
  double thetaMin = obvious::deg2rad(-120);
  //obvious::SensorPolar3D* sensor = new obvious::SensorPolar3D(thetaMin, thetaRes, 32, 0.0, obvious::deg2rad(0.16), 2250);
  _sensor = new obvious::SensorPolar3D(thetaMin, thetaRes, 32, 0.0, obvious::deg2rad(0.16), msg->points.size() / 32);
  _sensor->transform(&_Tinit);
  ROS_INFO_STREAM("Initial Pose");
    Matrix Tmp = _sensor->getTransformation();
    Tmp.print();


  _dist = new double[msg->points.size()];
  _mask = new bool[msg->points.size()];
  _coords = new double[msg->points.size() * 3];
  _normals = new double[msg->points.size() * 3];
  for(unsigned int i=0; i < msg->points.size(); i++)
  {
    _dist[i] = std::sqrt(msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z);
    //std::cout << _dist[i] << std::endl;
    if(_dist[i] > 10e-3)
      _mask[i] = true;
    else
      _mask[i] = false;
  }
  _sensor->setRealMeasurementData(_dist);
  _sensor->setRealMeasurementMask(_mask);
  //for(unsigned int i = 0; i < 20; i++)
    _space->push(_sensor);
    _space->serializeSliceImages(EnumSpaceAxis::Z);
    _space->serializeSliceImages(EnumSpaceAxis::X);

  return true;
}

void resetMask(const unsigned int size)
{
  for(unsigned int i = 0; i < size; i++)
    _mask[i] = false;
}

bool callBackGenerateGlPcl(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  const unsigned int maxSize = _space->getXDimension()*_space->getYDimension()*_space->getZDimension();
  double* cloud = new double[maxSize*3];
    double* normals = new double[maxSize*3];
  obvious::RayCastAxisAligned3D raca;
  unsigned int size = 0;
  //raca.calcCoords(_space, cloud, normals, NULL, &size);
  _rayCaster->calcCoordsFromCurrentPose(_space, _sensor, cloud, normals, NULL, &size);
  if(!size)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Raycaster found no valid coordinates");
    return false;
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " raycast found " << size / 3 << " points");
  sensor_msgs::PointCloud cloudRos;
  cloudRos.header.frame_id = "map";
  cloudRos.points.resize(size);
  for(unsigned int i = 0; i < size; i += 3)
  {
    geometry_msgs::Point32 point;
    point.x = static_cast<float>(cloud[i]);
    point.y = static_cast<float>(cloud[i + 1]);
    point.z = static_cast<float>(cloud[i + 2]);
    cloudRos.points[i / 3] = point;
  }
  _pubGlobPcl.publish(cloudRos);
  return true;
}
