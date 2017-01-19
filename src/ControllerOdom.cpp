/*
 * ControllerOdom.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: phil
 */

#include "ControllerOdom.h"
#include "UtilitiesTransform.h"

namespace ohm_tsd_slam
{

ControllerOdom::ControllerOdom(void)//tf::TransformListener& listener):
//  _listenerTf(listener)
{
  // TODO Auto-generated constructor stub

}

ControllerOdom::~ControllerOdom()
{
  // TODO Auto-generated destructor stub
}

bool ControllerOdom::getOdomTf(const ros::Time& last, const ros::Time& current, obvious::Matrix* const tf,
    const std::string& parentFrame, const std::string& childFrame)
{
  tf::StampedTransform tfLast;
  tf::StampedTransform tfCurrent;
  const double delay = 0.01;
 if(!_listenerTf.waitForTransform(parentFrame, childFrame, last, ros::Duration(delay)))
 {
   std::cout << __PRETTY_FUNCTION__ << " wait for last tf failed " << std::endl;
   return false;
 }
  try
  {
    _listenerTf.lookupTransform(parentFrame, childFrame, last, tfLast);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " tf last failed: " << ex.what());
    return false;
  }
  if(!_listenerTf.waitForTransform(parentFrame, childFrame, current, ros::Duration(delay)))
  {
    std::cout << __PRETTY_FUNCTION__ << " wait for current tf failed " << std::endl;
    return false;
  }
  try
  {
    _listenerTf.lookupTransform(parentFrame, childFrame, current, tfCurrent);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " tf current failed: " << ex.what());
    return false;
  }
  UtilitiesTransform tfUtilities;
//  std::cout << "here?\n";
  obvious::Matrix obvLast = tfUtilities.tfToObviouslyMatrix3x3(tfLast);
  obvious::Matrix obvCur  = tfUtilities.tfToObviouslyMatrix3x3(tfCurrent);
//  std::cout << "not\n";
  obvLast.invert();
  *tf = obvLast * obvCur;
  return true;
}

} /* namespace ohm_tsd_slam */
