/*
 * RobotFootprintFilter.cpp
 *
 *  Created on: 20.09.2013
 *      Author: chris
 */


#include "RobotFootprintFilter.h"

namespace obvious
{

RobotFootprintFilter::RobotFootprintFilter(unsigned int dim, double maxRadius)
{
  _dim = dim;
  _maxRadius = maxRadius;
  /*_offset = NULL;
  _x_lim  = NULL;
  _y_lim  = NULL;
  _z_lim  = NULL;*/
};

/*RobotFootprintFilter::RobotFootprintFilter(const double* x_lim, const double* y_lim, const double* z_lim)
{
  //_minRadius = 0.0;
  //_maxRadius = 0.0;
  //_offset = NULL;
  //_x_lim  = NULL;
  //_y_lim  = NULL;
  //_z_lim  = NULL;
}*/

RobotFootprintFilter::~RobotFootprintFilter()
{

}

void RobotFootprintFilter::filter(double** scene, unsigned int size, bool* mask)
{
  if(_dim==2)
  {
    double thresh = _maxRadius * _maxRadius;
    for (unsigned int i=0 ; i<size ; i++)
    {
      if ((scene[i][0]*scene[i][0] + scene[i][1]*scene[i][1]) < thresh)
        mask[i] = false;
    }
  }
  else if(_dim==3)
  {
    for (unsigned int i=0 ; i<size ; i++)
    {
      double p[3] = {scene[i][0], scene[i][1], scene[i][2]};
      if (abs3D<double>(p) < _maxRadius)
        mask[i] = false;
    }
  }
}

}
