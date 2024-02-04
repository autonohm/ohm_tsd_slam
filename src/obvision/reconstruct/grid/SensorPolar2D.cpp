#include "SensorPolar2D.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

#include <math.h>
#include <string.h>

namespace obvious
{

SensorPolar2D::SensorPolar2D(unsigned int size, double angularRes, double phiMin, double maxRange, double minRange, double lowReflectivityRange) : Sensor(2, maxRange, minRange, lowReflectivityRange)
{
  _width = size;
  _height = 1;
  _size = size;

  _data = new double[_size];
  _mask = new bool[_size];
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;

  _angularRes = angularRes;
  _phiMin = phiMin;

  // smallest angle that lies in bounds (must be negative)
  _phiLowerBound = -0.5*_angularRes + _phiMin;

  // if angle is too large, it might be projected with modulo 2 PI to a valid index
  // upper bound -> phiMin + (size-1 + 0.5) * resolution
  _phiUpperBound = _phiMin + (((double)size)-0.5)*_angularRes;

  if(_phiMin>=180.0)
  {
    LOGMSG(DBG_ERROR, "Valid minimal angle < 180 degree");
  }

  _rays = new Matrix(2, _size);

  for(unsigned int i=0; i<_size; i++)
  {
    const double phi = _phiMin + ((double)i) * _angularRes;
    (*_rays)(0, i) = cos(phi);
    (*_rays)(1, i) = sin(phi);
  }

  _raysLocal = new Matrix(2, size);
  *_raysLocal = *_rays;
}

SensorPolar2D::~SensorPolar2D()
{
  delete [] _data;
  delete [] _mask;

  delete _rays;
  delete _raysLocal;
}

void SensorPolar2D::setStandardMask()
{
  resetMask();
  maskZeroDepth();
  maskInvalidDepth();
  maskDepthDiscontinuity(deg2rad(3.0));
}

void SensorPolar2D::maskDepthDiscontinuity(double thresh)
{
  int radius = 1;
  double cosphi;
  double sinphi;
  sincos(_angularRes, &sinphi, &cosphi);
  for(int i=radius; i<((int)_size)-radius; i++)
  {
    double betamin = M_PI;
    double a = _data[i];
    if(isinf(a)) continue;
    for(int j=-radius; j<=radius; j++)
    {
      const double b = _data[i+j];
      if(isinf(b)) continue;
      // law of cosines
      double c = sqrt(a*a+b*b-2*a*b*cosphi);

      if(a>b)
      {
        // law of sines
        const double beta  = asin(b/c*sinphi);

        if(beta<betamin)
          betamin = beta;
      }
    }

    if(betamin<thresh)
      _mask[i] = false;
  }
}

int SensorPolar2D::backProject(double data[2])
{
  Matrix xh(3, 1);
  xh(0,0) = data[0];
  xh(1,0) = data[1];
  xh(2,0) = 1.0;
  Matrix PoseInv = getTransformation();
  PoseInv.invert();
  xh = PoseInv * xh;

  const double phi = atan2(xh(1,0), xh(0,0));
  // ensure angle to lie in valid bounds
  if(phi<=_phiLowerBound) return -2;
  if(phi>=_phiUpperBound) return -1;
  return round((phi-_phiMin) /_angularRes);
}

void SensorPolar2D::backProject(Matrix* M, int* indices, Matrix* T)
{
  Timer t;
  Matrix PoseInv = getTransformation();
  PoseInv.invert();
  if(T)
    PoseInv *= *T;

  Matrix coords2D = Matrix::multiply(PoseInv, *M, false, true);

  const double angularResInv = 1.0 / _angularRes;
  for(unsigned int i=0; i<M->getRows(); i++)
  {
    const double phi = atan2(coords2D(1,i), coords2D(0,i));
    if(phi<=_phiLowerBound) indices[i] = -2;
    else if(phi>=_phiUpperBound) indices[i] = -1;
    else indices[i] = round((phi-_phiMin) * angularResInv);
  }
}

}
