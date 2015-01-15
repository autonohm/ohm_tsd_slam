/*
 * LaserCallBackObject.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: phil
 */

#include "LaserCallBackObject.h"

namespace ohm_tsd_slam
{

LaserCallBackObject::LaserCallBackObject(ThreadLocalize* localizeThread):
    _localizeThread(localizeThread)
{


}

LaserCallBackObject::~LaserCallBackObject()
{

}

void LaserCallBackObject::laserCallBack(const sensor_msgs::LaserScan& scan)
{
  if(_localizeThread->setData(scan))
    _localizeThread->unblock();
}

} /* namespace ohm_tsd_slam */
