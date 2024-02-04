#ifndef TRIMMEDFILTER_H
#define TRIMMEDFILTER_H

#include "obvision/registration/icp/assign/filter/IPostAssignmentFilter.h"

namespace obvious
{

/**
 * @class TrimmedFilter
 * @brief A filter for point pair rejection
 * @author Stefan May
 */
class TrimmedFilter : public IPostAssignmentFilter
{
public:
  /**
   * Default constructor
   */
  TrimmedFilter(unsigned int unOverlap);

  /**
   * Destructor
   */
  ~TrimmedFilter();

  virtual void filter(double** model,
                      double** scene,
                      vector<StrCartesianIndexPair>* pairs,
                      vector<double>* distancesSqr,
                      vector<StrCartesianIndexPair>* fpairs,
                      vector<double>* fdistancesSqr,
                      vector<unsigned int>* nonPairs);

private:
  unsigned int _unOverlap;
};

}

#endif /*TRIMMEDFILTER_H*/
