/*
 * SlamBase.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: phil
 */

#include "SlamBase.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"

#include <string>

namespace ohm_tsd_slam
{

SlamBase::SlamBase()
{
  ros::NodeHandle prvNh("~");
  std::string strVar;
  int octaveFactor = 0;
  double cellside = 0.0;
  double dVar      = 0;
  int iVar         = 0;
  double truncationRadius = 0.0;

  prvNh.param<double>("x_off_factor", _xOffFactor, 0.5);
  prvNh.param<double>("y_off_factor", _yOffFactor, 0.5);
  prvNh.param<double>("yaw_start_offset", _yawOffset, 0.0);
  prvNh.param<int>("cell_octave_factor", octaveFactor, 10);
  prvNh.param<double>("cellsize", cellside, 0.025);
  prvNh.param<int>("truncation_radius", iVar, 3);
  truncationRadius = static_cast<double>(iVar);
  prvNh.param<double>("occ_grid_time_interval", _gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", _rateVar, 40.0);
  _loopRate = new ros::Rate(_rateVar);
  unsigned int uiVar = static_cast<unsigned int>(octaveFactor);
  if(uiVar > 15)
  {
    std::cout << __PRETTY_FUNCTION__ << " error! Unknown cell_octave_factor -> set to default!" << std::endl;
    uiVar = 10;
  }

  _grid = new obvious::TsdGrid(cellside, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(uiVar));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellside);

  unsigned int cellsPerSide = pow(2, uiVar);
  std::cout << __PRETTY_FUNCTION__ << " creating representation with " << cellsPerSide << "x" << cellsPerSide;
  double sideLength = static_cast<double>(cellsPerSide) * cellside;
  std::cout << " cells, representating "<< sideLength << "x" << sideLength << "m^2" << std::endl;

  _threadGrid = NULL;
  _threadMapping = NULL;

}

SlamBase::~SlamBase()
{
  delete _loopRate;
}

} /* namespace ohm_tsd_slam */
