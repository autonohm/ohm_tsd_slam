#ifndef RANDOMNORMALMATCHING_H_
#define RANDOMNORMALMATCHING_H_

#include <flann/flann.hpp>
#include "obvision/registration/ransacMatching/RandomMatching.h"
#include "obvision/registration/icp/PointToLineEstimator2D.h"
#include "omp.h"

namespace obvious
{

/**
 * @class RandomNormalMatching
 * @brief Matching algorithm with PCA alignment and RANSAC scheme
 * @author Stefan May
 **/
class RandomNormalMatching : public RandomMatching
{
public:
  /**
   * Constructor
   * @param trials number of trials / matching guesses
   * @param epsThresh threshold for rating good matches
   * @param phiMax maximum rotation
   * @param sizeControlSet approximate set of control set
   */
  RandomNormalMatching(unsigned int trials = 50, double epsThresh = 0.15, unsigned int sizeControlSet = 180);

  /**
   * Destructor
   */
  virtual ~RandomNormalMatching();

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

private:

  // init kd-tree for fast NN search in model
  void initKDTree(obvious::Matrix* M, vector<unsigned int> valid);

  // squared distance threshold
  double _scaleDistance;

  // normalization weight for orientation rating
  double _scaleOrientation;

  // number of trials
  unsigned int _trials;

  // tree for accelerating NN search
  flann::Index<flann::L2<double> >* _index;
  flann::Matrix<double>* _model;

  // Number of samples investigated for PCA in local neighborhood
  int _pcaSearchRange;

};

}

#endif /* RANDOMNORMALMATCHING_H_ */
