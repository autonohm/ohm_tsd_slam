#ifndef DISTANCEFILTER_H
#define DISTANCEFILTER_H

#include "obvision/registration/icp/assign/filter/IPostAssignmentFilter.h"

namespace obvious
{

/**
 * @class DistanceFilter
 * @brief A filter for point pair rejection
 * @author Stefan May
 */
class DistanceFilter : public IPostAssignmentFilter
{
public:
  /**
   * Default constructor
   */
  DistanceFilter(double maxdist, double mindist, unsigned int iterations);

  /**
   * Destructor
   */
  ~DistanceFilter();

  virtual void filter(double** model, double** scene,
                      vector<StrCartesianIndexPair>* pairs,
                      vector<double>* distancesSqr,
                      vector<StrCartesianIndexPair>* fpairs,
                      vector<double>* fdistancesSqr,
                      vector<unsigned int>* nonPairs);

  virtual void reset();
private:
  double _maxDistSqr;
  double _minDistSqr;
  double _distSqr;
  double _multiplier;
};

}

#endif /*DISTANCEFILTER_H*/
