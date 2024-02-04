#include "DistanceFilter.h"

#include <algorithm>
#include <vector>
#include <math.h>

namespace obvious
{


DistanceFilter::DistanceFilter(double maxdist, double mindist, unsigned int iterations)
{
  _maxDistSqr = maxdist * maxdist;
  _minDistSqr = mindist * mindist;
  _distSqr = _maxDistSqr;
  double it = (double)(iterations - 1);
  if(iterations < 1) it = 1.0;
  // i'th root
  _multiplier = pow((mindist / maxdist), 1.0 / it);
};

DistanceFilter::~DistanceFilter()
{

};

void DistanceFilter::reset()
{
  _distSqr = _maxDistSqr;
}

void DistanceFilter::filter(double** model,
                            double** scene,
                            vector<StrCartesianIndexPair>* pairs,
                            vector<double>* distancesSqr,
                            vector<StrCartesianIndexPair>* fpairs,
                            vector<double>* fdistancesSqr,
                            vector<unsigned int>* nonPairs)
{
  if(!_active)
  {
    *fpairs = *pairs;
    *fdistancesSqr = *distancesSqr;
    return;
  }

  fpairs->clear();
  fdistancesSqr->clear();

  for(unsigned int p=0; p<pairs->size(); p++)
  {
    if((*distancesSqr)[p] <= _distSqr)
    {
      fpairs->push_back((*pairs)[p]);
      fdistancesSqr->push_back((*distancesSqr)[p]);
    }
    else
    {
      nonPairs->push_back((*pairs)[p].indexSecond);
    }
  }
  _distSqr *= _multiplier;
  if(_distSqr < _minDistSqr) _distSqr = _minDistSqr;
}

}
