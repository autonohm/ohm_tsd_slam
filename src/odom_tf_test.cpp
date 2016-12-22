/*
 * odom_tf_test.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: phil
 */


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include "UtilitiesTransform.h"

void callBackScan(const sensor_msgs::LaserScan& scan);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_tf_test");
  ros::NodeHandle nh;
  ros::Subscriber subScan = nh.subscribe("scan", 1, callBackScan);
  ros::spin();
}


void callBackScan(const sensor_msgs::LaserScan& scan)
{
  ros::Time timer = ros::Time::now();
  static tf::TransformListener listener;
  static ros::Time last = ros::Time::now();
  obvious::Matrix tf(3, 3);

  tf::StampedTransform tfLast;
  tf::StampedTransform tfCurrent;

  try
  {
    listener.lookupTransform("odom", "base_footprint", last, tfLast);
  }
  catch(tf::TransformException& ex)
  {
    std::cout << __PRETTY_FUNCTION__ << " last tf failed " << ex.what() << std::endl;
  }


  try
  {
    listener.lookupTransform("odom", "base_footprint", scan.header.stamp, tfCurrent);
  }
  catch(tf::TransformException& ex)
  {
    std::cout << __PRETTY_FUNCTION__ << " current tf failed" << ex.what() << std::endl;
  }
  ohm_tsd_slam::UtilitiesTransform tfUtilities;
  obvious::Matrix obvLast = tfUtilities.tfToObviouslyMatrix3x3(tfLast);
  obvious::Matrix obvCur  = tfUtilities.tfToObviouslyMatrix3x3(tfCurrent);
  obvLast.invert();
  tf = obvLast * obvCur;
  tf.print();
  last = ros::Time::now();
  std::cout << __PRETTY_FUNCTION__ << " cycle time = " << (ros::Time::now() - timer).toSec() << " (s)" << std::endl;
}
