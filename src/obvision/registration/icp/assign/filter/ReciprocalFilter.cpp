#include "ReciprocalFilter.h"

#include <algorithm>
#include <vector>

namespace obvious
{

struct StrReciprocalPair
{
    unsigned int unIdx;
    unsigned int unModelIndex;
    double dDistanceSqr;
};

bool operator < (const StrReciprocalPair& first, const StrReciprocalPair& second)
{
    if (first.unModelIndex < second.unModelIndex) return true;
    else if ( (first.unModelIndex == second.unModelIndex) && (first.dDistanceSqr < second.dDistanceSqr)  ) return true;
    return false;
}

ReciprocalFilter::ReciprocalFilter()
{
};

ReciprocalFilter::~ReciprocalFilter()
{

};

void ReciprocalFilter::filter(double** model, double** scene, vector<StrCartesianIndexPair>* pairs, vector<double>* distancesSqr, vector<StrCartesianIndexPair>* fpairs, vector<double>* fdistancesSqr, vector<unsigned int>* nonPairs)
{
  if(!_active)
  {
    *fpairs        = *pairs;
    *fdistancesSqr = *distancesSqr;
    return;
  }

  fpairs->clear();
  fdistancesSqr->clear();

  // sort pairs w.r.t model point index (and distance if model point indices are equal)
  std::vector<StrReciprocalPair> vReciprocalPairs;
  unsigned int i=0;
  for( i=0; i < pairs->size(); ++i)
  {
      StrReciprocalPair pair;
      pair.unIdx = i;
      pair.unModelIndex = (*pairs)[i].indexFirst;
      pair.dDistanceSqr = (*distancesSqr)[i];
      vReciprocalPairs.push_back(pair);
  }

  if(vReciprocalPairs.size()==0) return;

  std::sort(vReciprocalPairs.begin(), vReciprocalPairs.end());

  unsigned int unModelIndexLast = vReciprocalPairs[0].unModelIndex; // remember model point index of first pair
  fpairs->push_back((*pairs)[vReciprocalPairs[0].unIdx]); // store first pair in result vector
  for ( i=1; i < vReciprocalPairs.size(); ++i )
  {
      // ignore all pairs containing the same model point index
      if ( vReciprocalPairs[i].unModelIndex ==  unModelIndexLast)
      {
        unsigned int idx = vReciprocalPairs[i].unIdx;
        nonPairs->push_back((*pairs)[idx].indexSecond);
      }
      else
      {
        unModelIndexLast = vReciprocalPairs[i].unModelIndex;
        unsigned int idx = vReciprocalPairs[i].unIdx;
        fpairs->push_back((*pairs)[idx]);
        fdistancesSqr->push_back((*distancesSqr)[idx]);
      }
  }
}

}
