/*
 * UtilitiesTransform.h
 *
 *  Created on: Dec 12, 2016
 *      Author: phil
 */

#ifndef SRC_UTILITIESTRANSFORM_H_
#define SRC_UTILITIESTRANSFORM_H_

#include <ros/ros.h>
#include <tf/tf.h>

#include "obcore/math/linalg/linalg.h"

namespace ohm_tsd_slam
{

class UtilitiesTransform
{
public:
  UtilitiesTransform();
  virtual ~UtilitiesTransform();
  tf::Transform obviouslyMatrix3x3ToTf(obvious::Matrix& ob);
  obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf);
};

} /* namespace ohm_tsd_slam */

#endif /* SRC_UTILITIESTRANSFORM_H_ */
