/*
 * tf_test.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: phil
 */

#include "obcore/math/linalg/linalg.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>

#include "UtilitiesTransform.h"

void callbackTimer(const ros::TimerEvent&);

static ros::Publisher _poses;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_test");

  ros::NodeHandle nh;
  ros::Timer timer1 = nh.createTimer(ros::Duration(1 / 50.0), callbackTimer);
  _poses = nh.advertise<geometry_msgs::PoseArray>("poses", 1);
  ros::spin();
}

void callbackTimer(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster bc;
  ohm_tsd_slam::UtilitiesTransform utils;
  obvious::Matrix mat1(3, 3);
  obvious::Matrix mat2(3, 3);
  mat1.setIdentity();
  mat1(1, 2) = 1.0;
  mat2.setIdentity();
  mat2(1, 2) = 2.0;
  tf::Transform tf;
  tf.setRotation(tf::createQuaternionFromYaw(0.8));
  tf.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
  mat1 = utils.tfToObviouslyMatrix3x3(tf);
  bc.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "tf1"));
  obvious::Matrix point(1, 3);
  point.setIdentity();
  point(0, 0) = 1.0;
  point = mat1 * point;
  geometry_msgs::PoseStamped

  tf.setRotation(tf::createQuaternionFromYaw(1.2));
  tf.setOrigin(tf::Vector3(2.0, 0.0, 0.0));
  mat2 = utils.tfToObviouslyMatrix3x3(tf);
  bc.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "tf2"));


  obvious::Matrix matDiff(3, 3);
  matDiff.setIdentity();
  matDiff = mat2 * mat1.getInverse();
  matDiff.print();
  tf = utils.obviouslyMatrix3x3ToTf(matDiff);
  //bc.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "tf2", "tf1"));

  obvious::Matrix matSum(3, 3);
  matSum.setIdentity();
  matSum = matDiff * mat1;
  // bc.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "tf2"));
}

