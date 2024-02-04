#ifndef OUTOFBOUNDSFILTER2D_H
#define OUTOFBOUNDSFILTER2D_H

#include "obvision/registration/icp/assign/filter/IPreAssignmentFilter.h"
#include "obcore/math/linalg/linalg.h"

namespace obvious
{

/**
 * @class OutOfBoundsFilter2D
 * @brief Bounding box filtering
 * @author Stefan May
 */
class OutOfBoundsFilter2D : public IPreAssignmentFilter
{
public:
  /**
   * Default constructor
   */
  OutOfBoundsFilter2D(double xmin, double xmax, double ymin, double ymax);

  /**
   * Destructor
   */
  ~OutOfBoundsFilter2D();

  void setPose(Matrix* T);

  virtual void filter(double** scene, unsigned int size, bool* mask);

private:
  double _xMin;
  double _xMax;
  double _yMin;
  double _yMax;
  Matrix* _T;
};

}

#endif /*OUTOFBOUNDSFILTER2D_H*/
