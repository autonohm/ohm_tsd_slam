/*
 * LaserCallBackObject.h
 *
 *  Created on: Jan 15, 2015
 *      Author: phil
 */

#ifndef SRC_MULTI_LASERCALLBACKOBJECT_H_
#define SRC_MULTI_LASERCALLBACKOBJECT_H_

#include <string>
#include "ThreadLocalize.h"
#include <sensor_msgs/LaserScan.h>

namespace ohm_tsd_slam
{

class LaserCallBackObject
{
public:
  LaserCallBackObject(ThreadLocalize* localizeThread);
  virtual ~LaserCallBackObject();
  void laserCallBack(const sensor_msgs::LaserScan& scan);
private:
  ThreadLocalize* _localizeThread;
};

} /* namespace ohm_tsd_slam */

#endif /* SRC_MULTI_LASERCALLBACKOBJECT_H_ */
