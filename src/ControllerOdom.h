/*
 * ControllerOdom.h
 *
 *  Created on: Dec 12, 2016
 *      Author: phil
 */

#ifndef SRC_CONTROLLERODOM_H_
#define SRC_CONTROLLERODOM_H_

#include <tf/transform_listener.h>
#include <string>

#include "obcore/math/linalg/linalg.h"

namespace ohm_tsd_slam
{

class ControllerOdom
{
public:
  ControllerOdom(void);//tf::TransformListener& listener);
  virtual ~ControllerOdom();
  bool getOdomTf(const ros::Time& last, const ros::Time& current, obvious::Matrix* const tf,
                 const std::string& parentFrame, const std::string& childFrame);
private:
  tf::TransformListener _listenerTf;

};

} /* namespace ohm_tsd_slam */

#endif /* SRC_CONTROLLERODOM_H_ */
