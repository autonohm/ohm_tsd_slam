#ifndef _RANDOMMATCHING_H_
#define _RANDOMMATCHING_H_

#include <vector>
#include "obcore/math/linalg/linalg.h"
#include "obvision/registration/Trace.h"

namespace obvious
{

class RandomMatching
{
public:
  RandomMatching(unsigned int sizeControlSet);

  virtual ~RandomMatching();

  void activateTrace();

  void deactivateTrace();

  void serializeTrace(const char* folder);

protected:

  // extract valid indices from matrix giving a validity mask
  std::vector<unsigned int> extractSamples(obvious::Matrix* M, const bool* mask, unsigned int searchRange);

  // pick control set for RANSAC in-/outlier detection
  obvious::Matrix* pickControlSet(obvious::Matrix* M, std::vector<unsigned int> idxValid, std::vector<unsigned int> &idxControl);

  void calcNormals(Matrix* M, Matrix* N, const bool* maskIn, bool* maskOut, int searchRadius);

  void calcPhi(Matrix* N,  const bool* mask, double* phi);

  void subsampleMask(bool* mask, unsigned int size, double probability);

  unsigned int _sizeControlSet;

  // Trace module
  Trace* _trace;

};

} /* namespace obvious */

#endif /* _RANDOMMATCHING_H_ */
