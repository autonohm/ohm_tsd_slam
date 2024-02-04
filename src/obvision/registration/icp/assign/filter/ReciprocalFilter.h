#ifndef RECIPROCALFILTER_H
#define RECIPROCALFILTER_H

#include "obvision/registration/icp/assign/filter/IPostAssignmentFilter.h"
#include <vector>

namespace obvious
{

/**
 * @class ReciprocalFilter
 * @brief A filter for point pair rejection
 * @author Stefan May
 */
class ReciprocalFilter : public IPostAssignmentFilter
{
public:
  /**
   * Default constructor
   */
  ReciprocalFilter();

  /**
   * Destructor
   */
  ~ReciprocalFilter();

  virtual void filter(double** model, double** scene, vector<StrCartesianIndexPair>* pairs, vector<double>* distancesSqr, vector<StrCartesianIndexPair>* fpairs, vector<double>* fdistancesSqr, vector<unsigned int>* nonPairs);

private:
  unsigned int _unOverlap;
};

}

#endif /*ReciprocalFilter_H*/
