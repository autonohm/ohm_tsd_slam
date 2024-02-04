#ifndef _PDFMATCHING_H_
#define _PDFMATCHING_H_

#include <math.h>
#include "obcore/base/Logger.h"
#include "obcore/math/mathbase.h"
#include <omp.h>
#include <cmath>

#include "obvision/registration/ransacMatching/RandomMatching.h"

namespace obvious
{

/**
 * @class PDFMatching
 * @brief Matching algorithm with rating based on probability density function (PDF)
 * @author Daniel Ammon, Tobias Fink
 **/
class PDFMatching : public RandomMatching
{
public:

  PDFMatching(
      unsigned int trials = 100,
      double epsThresh = 0.15,
      unsigned int sizeControlSet = 140,
      double zhit = 0.45,
      double zphi = 0.0,
      double zshort = 0.25,
      double zmax = 0.05,
      double zrand = 0.25,
      double percentagePointsInC = 0.9,
      double rangemax = 20,
      double sigphi = M_PI / 180.0 * 3,
      double sighit = 0.2,
      double lamshort = 0.08,
      double maxAngleDiff = 3.0,
      double maxAnglePenalty = 0.5);

  virtual ~PDFMatching();

  /**
   * Matching method
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
  obvious::Matrix match(obvious::Matrix* M,
              const bool* maskM,
              obvious::Matrix* NM,
              obvious::Matrix* S,
              const bool* maskS,
              double phiMax = M_PI / 4.0,
              const double transMax = 1.5,
              const double resolution = 0.0);

  /**
  * get probability of two single rangefinder scans (of two rays)
  * @param m distance of model point
  * @param s distance of scene/control point
  * @param phiDiff difference of the two angles
  */
  double probabilityOfTwoSingleScans(double m, double s, double phiDiff);

private:

  // probability model variables
  double _zhit;
  double _zphi;
  double _zshort;
  double _zmax;
  double _zrand;

  double _percentagePointsInC;

  double _rangemax;
  double _sigphi;
  double _sigpphi; // factor for multiplication in probability function
  double _sighit;
  double _sigphit; // factor for multiplication in probability function
  double _lamshort;

  double _maxAngleDiff;
  double _maxAnglePenalty;

  // squared distance threshold
  double _scaleDistance;

  // normalization weight for orientation rating
  double _scaleOrientation;

  // number of trials
  unsigned int _trials;

  // approximate control set
  unsigned int _sizeControlSet;

  // Number of samples investigated for PCA in local neighborhood
  int _pcaSearchRange;

};

} /* namespace obvious */

#endif /* _PDFMATCHING_H_ */
