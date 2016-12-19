/*
 * odom_tf_test.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: phil
 */


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

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
  static tf::TransformListener listener;
  tf::StampedTransform tf;
//  tf.frame_id_ = "odom";
//  tf.child_frame_id_ = "base_footprint";
//  tf.stamp_ = scan.header.stamp;

  try
  {
    listener.lookupTransform("odom", "base_footprint", scan.header.stamp, tf);
  }
  catch(tf::TransformException& ex)
  {
    std::cout << __PRETTY_FUNCTION__ << " failed" << ex.what() << std::endl;
  }
}
