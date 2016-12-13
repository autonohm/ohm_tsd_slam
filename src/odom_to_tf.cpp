/*
 * odom_to_tf.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: phil
 */


#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void callBackOdometry(const nav_msgs::Odometry& odom);



#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");
  std::string topicOdom;
  std::string frameChild;

  prvNh.param<std::string>("topic_odom", topicOdom, "odom");
  prvNh.param<std::string>("frame_child", frameChild, "odom");

  ros::Subscriber subsOdom = nh.subscribe(topicOdom, 1, callBackOdometry);
  ros::spin();
}

void callBackOdometry(const nav_msgs::Odometry& odom)
{
  static  tf::TransformBroadcaster broadCaster;
  tf::StampedTransform tf;
  tf.stamp_ = odom.header.stamp;
  tf.frame_id_ = odom.header.frame_id;
  tf.child_frame_id_ = odom.child_frame_id;
  const geometry_msgs::Pose& pose = odom.pose.pose;

  tf::Vector3 origin(pose.position.x, pose.position.y, pose.position.z);
  tf::Quaternion quat;
  quat.setX(pose.orientation.x);
  quat.setY(pose.orientation.y);
  quat.setZ(pose.orientation.z);
  quat.setW(pose.orientation.w);

  tf.setOrigin(origin);
  tf.setRotation(quat);

  broadCaster.sendTransform(tf);
}
