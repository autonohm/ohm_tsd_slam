#ifndef REGISTRATION_H_
#define REGISTRATION_H_

#include "IPreMatcher.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/registration/icp/icp_def.h"

#include <memory>

enum class RegModes
{
  ICP = 0, ///< Registration with ICP only
  EXP,
  PDF,
  TSD
};

class Registration
{
public:
  Registration(const RegModes& mode, obvious::TsdGrid& grid, obvious::SensorPolar2D& sensor);
  virtual ~Registration();
  // bool doRegistration(obvious::Matrix& T, obvious::Matrix* M, obvious::Matrix* Mvalid, obvious::Matrix* N, obvious::Matrix* Nvalid, obvious::Matrix* S,
  //                     obvious::Matrix* Svalid);
  bool doRegistration(obvious::Matrix& T, double* coordsModel, double* normalsModel, bool* maskModel, const unsigned int nPointsModel, 
                      double* coordsScene, bool* maskScene);                      
private:
  bool                         init(const std::string& configFileXml);
  bool isRegistrationError(obvious::Matrix* T, const double trnsMax, const double rotMax);
  /**
   * Method to analyze 2D transformation matrix.
   * @param T Pointer to transformation matrix
   * @return Calculated angle
   */
  //double calcAngle(obvious::Matrix* T);

   /**
   * Method to remove certain values in a matrix using a given mask.
   * @param Mat Input matrix data
   * @param mask Value mask
   * @param maskSize Amount of values in the mask
   * @param validPoints Value determining the number of values in the output matrix
   * @return Filtered matrix of type obvious::Matrix
   */
  //obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints);
  obvious::TsdGrid&            _grid;
  obvious::SensorPolar2D& _sensor;
  std::unique_ptr<IPrematcher> _matcherPre;

  std::unique_ptr<obvious::PairAssignment> _assigner;
  /**
   * ICP out of bounds filter
   */
  std::unique_ptr<obvious::OutOfBoundsFilter2D> _filterBounds;

  /**
   * ICP reciprocal filter
   * @todo what is this?
   */
  std::unique_ptr<obvious::ReciprocalFilter> _filterReciprocal;
  /**
   * ICP transformation estimator
   */
  std::unique_ptr<obvious::IRigidEstimator> _estimator;

  std::unique_ptr<obvious::DistanceFilter> _filterDist;
  /**
   * ICP main instance
   */
  std::unique_ptr<obvious::Icp> _icp;
  const RegModes                _regMode;

  double       _distFilterThreshMin;
  double       _distFilterThreshMax;
  unsigned int _icpIterations;
  double       _threshErrorAng;
  double       _threshErrorLin;
  double _icpThreshRms;
};

#endif