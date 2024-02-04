#ifndef RAYCASTPOLAR2D_H
#define RAYCASTPOLAR2D_H

#include <vector>
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

namespace obvious
{

/**
 * @class RayCastPolar2D
 * @brief
 * @author Stefan May
 */
class RayCastPolar2D
{
public:

  /**
   * Constructor
   */
  RayCastPolar2D();

  /**
   * Destructor
   */
  ~RayCastPolar2D();

  /**
   * Perform raycasting, i.e. calculate coordinates obtainable from the sensor's field of vision
   * @param grid grid instance
   * @param sensor sensor instance
   * @param coords coordinates of intersection between beam and surface
   * @param normals normals of surfaces
   * @param ctr number of points
   */
  void calcCoordsFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, unsigned int* ctr);

  /**
   * Perform raycasting, i.e. calculate coordinates obtainable from the sensor's field of vision
   * @param grid grid instance
   * @param sensor sensor instance
   * @param coords coordinates of intersection between beam and surface
   * @param normals normals of surfaces
   * @param mask validity mask
   * @return number of valid points, i.e., having mask[i]==true
   */
  unsigned int calcCoordsFromCurrentViewMask(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, bool* mask);

private:

  bool rayCastFromCurrentView(TsdGrid* grid, obfloat tr[2], obfloat ray[2], obfloat coordinates[2], obfloat normal[2]);

  void calcRayFromCurrentView(const unsigned int beam, double dirVec[2]);

  double _xmin;
  double _ymin;

  double _xmax;
  double _ymax;

  obfloat _idxMin;
  obfloat _idxMax;
};

}

#endif
