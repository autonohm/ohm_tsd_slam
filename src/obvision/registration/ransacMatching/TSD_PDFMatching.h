#ifndef _TSD_PDFMatching_H_
#define _TSD_PDFMatching_H_

#include <math.h>
#include "obcore/base/Logger.h"
#include "obcore/base/types.h"
#include "obcore/math/mathbase.h"
#include <omp.h>
#include <cmath>

#include "obvision/registration/ransacMatching/RandomMatching.h"
#include "obvision/reconstruct/grid/TsdGrid.h"

namespace obvious
{

/**
 * @class TSD_PDFMatching
 * @brief Matching algorithm with rating based on probability density function (PDF)
 * @author Daniel Ammon, Tobias Fink, Stefan May
 * @date 23.12.2015
 **/
class TSD_PDFMatching : public RandomMatching
{
public:

  TSD_PDFMatching( TsdGrid& grid,
                   unsigned int trials = 30,
                   double epsThresh = 0.15,
                   unsigned int sizeControlSet = 360,
                   double zrand = 0.05);

  virtual ~TSD_PDFMatching();

  /**
   * Matching method
   * @param TSensor sensor pose as transformation matrix
   * @param M Matrix for model points. Points are accessed by rows. e.g. x = M(p, 0) y= M(p,1)
   * @param maskM Mask for matrix M. Valid points are identified with true-value. It has the same size as M.getCols()
   * @param NM Matrix with normals for model data set
   * @param S Matrix for scene points
   * @param maskS Mask for matrix S
   * @param phiMax Maximum allowed rotation as output
   * @param transMax Maximum allowed translation
   * @param resolution Angular resolution of the laser scan
   * @return 3x3 registration matrix
   */
  obvious::Matrix match( obvious::Matrix TSensor,
                         obvious::Matrix* M,
                         const bool* maskM,
                         obvious::Matrix* NM,
                         obvious::Matrix* S,
                         const bool* maskS,
                         double phiMax = M_PI / 4.0,
                         const double transMax = 1.5,
                         const double resolution = 0.0);

private:

  // probability model variable
  double _zrand;

  // squared distance threshold
  double _scaleDistance;

  // normalization weight for orientation rating
  double _scaleOrientation;

  // number of trials
  unsigned int _trials;

  // Number of samples investigated for PCA in local neighborhood
  int _pcaSearchRange;

  // tsd grid representation
  TsdGrid& _grid;

};

} /* namespace obvious */

#endif /* _TSD_PDFMatching_H_ */
