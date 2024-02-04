#include "TrimmedFilter.h"

#include <algorithm>
#include <vector>

namespace obvious
{

struct StrObvisionTrimPair
{
  unsigned int unIdx;
  double dDistanceSqr;
};

bool operator < (const StrObvisionTrimPair& first, const StrObvisionTrimPair& second)
{
  if (first.dDistanceSqr < second.dDistanceSqr) return true;
  return false;
}

TrimmedFilter::TrimmedFilter(unsigned int unOverlap)
{
  _unOverlap = unOverlap;
};

TrimmedFilter::~TrimmedFilter()
{

};

void TrimmedFilter::filter(double** model,
                           double** scene,
                           vector<StrCartesianIndexPair>* pairs,
                           vector<double>* distancesSqr,
                           vector<StrCartesianIndexPair>* fpairs,
                           vector<double>* fdistancesSqr,
                           vector<unsigned int>* nonPairs)
{
  if(!_active)
  {
    *fpairs        = *pairs;
    *fdistancesSqr = *distancesSqr;
    return;
  }

  unsigned int unPairs = pairs->size();
  unsigned int unConsideredPairs = (unPairs * this->_unOverlap) / 100;

  fpairs->clear();
  fdistancesSqr->clear();

  vector<StrObvisionTrimPair> vTrimmedPairs;

  unsigned int p=0;
  for(p=0; p<unPairs; p++)
  {
    StrObvisionTrimPair trimmedPair;
    trimmedPair.unIdx = p;
    trimmedPair.dDistanceSqr = (*distancesSqr)[p];
    vTrimmedPairs.push_back(trimmedPair);
  }
  std::sort(vTrimmedPairs.begin(), vTrimmedPairs.end());

  // ensure in-bound access
  unsigned int unRetVals = ( unPairs > unConsideredPairs ? unConsideredPairs : unPairs);

  for(p=0; p<unRetVals; p++)
  {
    unsigned int unIdx = vTrimmedPairs[p].unIdx;
    StrCartesianIndexPair pair = (*pairs)[unIdx];
    fpairs->push_back(pair);
    fdistancesSqr->push_back((*distancesSqr)[unIdx]);
  }
  for(p=unRetVals; p<unPairs; p++)
  {
    unsigned int unIdx = vTrimmedPairs[p].unIdx;
    nonPairs->push_back((*pairs)[unIdx].indexSecond);
  }
}

}
