#ifndef SENSOR_POLAR_2D_H
#define SENSOR_POLAR_2D_H

#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class SensorPolar2D
 * @brief Generic class for 2D measurement units using polar sampling
 * @author Stefan May
 */
class SensorPolar2D : public Sensor
{
public:

  /**
   * Standard constructor
   * @param[in] beams number of beams
   * @param[in] angularRes angular resolution, i.e. angle between beams in rad
   * @param[in] phiMin minimum angle from which beams are counted positive counter-clockwisely (rad)
   * @param[in] maxRange maximum range
   * @param[in] minRange minimum range
   */
  SensorPolar2D(unsigned int beams, double angularRes, double phiMin, double maxRange=INFINITY, double minRange=0.0, double lowReflectivityRange=INFINITY);

  /**
   * Destructor
   */
  ~SensorPolar2D();

  /**
   * Set standard measurement mask (measurement data need to be set before)
   * Parameter for depth discontinuity is 1/60*M_PI
   */
  void setStandardMask();

  /**
   * Mask measurements with acute angles to neighbors
   * @param[in] thresh threshold in rad (meaningful values < 1/36*M_PI)
   */
  void maskDepthDiscontinuity(double thresh);

  /**
   * Assign an arbitrary 2D coordinate to a measurement beam
   * @param[in] coordinate vector
   * @return beam index, negative values are invalid, -1 -> exceeded upper bound, -2 -> exceeded lower bound
   */
  int backProject(double data[2]);

  /**
   * Parallel version of back projection
   * @param[in] M matrix of homogeneous 2D coordinates
   * @param[out] indices vector of beam indices, negative values are invalid, -1 -> exceeded upper bound, -2 -> exceeded lower bound
   * @param[in] T temporary transformation matrix of coordinates
   */
  void backProject(Matrix* M, int* indices, Matrix* T=NULL);

  /**
   * Get angular resolution
   * @return angular resolution
   */
  double getAngularResolution() const { return _angularRes;};

  /**
   * Get the minimum angle
   * @return minimum angle
   */
  double getPhiMin() const { return _phiMin; };

  /**
   * Get lower bound of field of view
   * @return lower bound
   */
  double getPhiLowerBound(void) const { return _phiLowerBound; };

  /**
   * Get upper bound of field of view
   * @return upper bound
   */
  double getPhiUpperBound(void) const { return _phiUpperBound; };

private:

  double _angularRes;

  double _phiMin;

  double _phiLowerBound;

  double _phiUpperBound;
};

}

#endif
