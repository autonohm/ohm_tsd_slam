#include "RayCastPolar2D.h"

#include <string.h>

#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/base/Timer.h"
#include "obcore/base/Logger.h"

namespace obvious
{

RayCastPolar2D::RayCastPolar2D()
{
  _xmin   = NAN;
  _ymin   = NAN;

  _xmax   = NAN;
  _ymax   = NAN;
}

RayCastPolar2D::~RayCastPolar2D()
{

}

void RayCastPolar2D::calcCoordsFromCurrentView(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, unsigned int* cnt)
{
  Timer t;
  t.start();
  *cnt = 0;

  Matrix T = sensor->getTransformation();
  T.invert();

  Matrix* R = sensor->getNormalizedRayMap(grid->getCellSize());
  unsigned int count = sensor->getRealMeasurementSize();

  obfloat tr[2];
  sensor->getPosition(tr);

  if(grid->isInsideGrid(sensor))
  {
    _xmin   = -10e9;
    _ymin   = -10e9;

    _xmax   = 10e9;
    _ymax   = 10e9;
  }
  else
  {
    // prevent rays to be casted parallel to a plane outside of space
    _xmin   = 10e9;
    _ymin   = 10e9;

    _xmax   = -10e9;
    _ymax   = -10e9;

    LOGMSG(DBG_WARN, "Sensor is outside of grid");
  }

  _idxMin = sensor->getMinimumRange() / grid->getCellSize();
  _idxMax = sensor->getMaximumRange() / grid->getCellSize();

#pragma omp parallel
{
  obfloat c[2];
  obfloat n[2];
  Matrix M(3,1);
  Matrix N(3,1);
  M(2,0) = 1.0;
  N(2,0) = 0.0; // no translation for normals
  unsigned int size_tmp = 0;
  double* c_tmp         = new double[count*2];
  double* n_tmp         = new double[count*2];

#pragma omp for schedule(dynamic)
  for (unsigned int beam = 0; beam < count; beam++)
  {
    obfloat ray[2];
    ray[0] = (*R)(0, beam);
    ray[1] = (*R)(1, beam);
    if (rayCastFromCurrentView(grid, tr, ray, c, n))
    {
      M(0,0)  = c[0];
      M(1,0)  = c[1];
      N(0,0)  = n[0];
      N(1,0)  = n[1];
      M       = T * M;
      N       = T * N;
      for (unsigned int i = 0; i < 2; i++)
      {
        c_tmp[size_tmp] = M(i,0);
        n_tmp[size_tmp++] = N(i,0);
      }
    }
  }

#pragma omp critical
  {
      memcpy(&coords[*cnt],  c_tmp, size_tmp*sizeof(double));
      memcpy(&normals[*cnt], n_tmp, size_tmp*sizeof(double));
      *cnt += size_tmp;
  }
    delete[] c_tmp;     c_tmp = NULL;
    delete[] n_tmp;     n_tmp = NULL;
}

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.elapsed() << "s");
  LOGMSG(DBG_DEBUG, "Ray casting finished! Found " << *cnt << " coordinates");
}

unsigned int RayCastPolar2D::calcCoordsFromCurrentViewMask(TsdGrid* grid, SensorPolar2D* sensor, double* coords, double* normals, bool* mask)
{
  Timer t;
  t.start();

  unsigned int cnt = 0;

  Matrix T = sensor->getTransformation();
  T.invert();

  Matrix* R = sensor->getNormalizedRayMap(grid->getCellSize());

  obfloat tr[2];
  sensor->getPosition(tr);

  if(grid->isInsideGrid(sensor))
  {
    _xmin   = -10e9;
    _ymin   = -10e9;

    _xmax   = 10e9;
    _ymax   = 10e9;
  }
  else
  {
    // prevent rays to be casted parallel to a plane outside of space
    _xmin   = 10e9;
    _ymin   = 10e9;

    _xmax   = -10e9;
    _ymax   = -10e9;

    LOGMSG(DBG_WARN, "Sensor is outside of grid");
  }

  _idxMin = sensor->getMinimumRange() / grid->getCellSize();
  _idxMax = sensor->getMaximumRange() / grid->getCellSize();

#pragma omp parallel
{
  obfloat c[2];
  obfloat n[2];
  Matrix M(3,1);
  Matrix N(3,1);
  M(2,0) = 1.0;
  N(2,0) = 0.0; // no translation for normals

#pragma omp for schedule (dynamic) reduction(+:cnt)
  for (unsigned int beam = 0; beam < sensor->getRealMeasurementSize(); beam++)
  {
    obfloat ray[2];
    ray[0] = (*R)(0, beam);
    ray[1] = (*R)(1, beam);
    if (rayCastFromCurrentView(grid, tr, ray, c, n))
    {
      M(0,0)            = c[0];
      M(1,0)            = c[1];
      N(0,0)            = n[0];
      N(1,0)            = n[1];
      M                 = T * M;
      N                 = T * N;
      coords[2*beam]    = M(0,0);
      coords[2*beam+1]  = M(1,0);
      normals[2*beam]   = N(0,0);
      normals[2*beam+1] = N(1,0);
      mask[beam] = true;
      cnt++;
    }
    else
    {
      mask[beam] = false;
    }
  }
}

  LOGMSG(DBG_DEBUG, "Elapsed TSDF projection: " << t.elapsed() << "s");
  LOGMSG(DBG_DEBUG, "Ray casting finished!");

  return cnt;
}

bool RayCastPolar2D::rayCastFromCurrentView(TsdGrid* grid, obfloat tr[2], obfloat ray[2], obfloat coordinates[2], obfloat normal[2])
{
  int xDim = grid->getCellsX();
  int yDim = grid->getCellsY();
  double cellSize = grid->getCellSize();

  obfloat position[2];

  // Interpolation weight
  obfloat interp;

  obfloat xmin   = _xmin;
  obfloat ymin   = _ymin;
  if(fabs(ray[0])>10e-6) xmin = ((obfloat)(ray[0] > 0.0 ? 0 : (xDim-1)*cellSize) - tr[0]) / ray[0];
  if(fabs(ray[1])>10e-6) ymin = ((obfloat)(ray[1] > 0.0 ? 0 : (yDim-1)*cellSize) - tr[1]) / ray[1];
  obfloat idxMin = max(xmin, ymin);
  idxMin        = max(idxMin, TSDZERO);

  obfloat xmax   = _xmax;
  obfloat ymax   = _ymax;
  if(fabs(ray[0])>10e-6) xmax = ((obfloat)(ray[0] > 0.0 ? (xDim-1)*cellSize : 0) - tr[0]) / ray[0];
  if(fabs(ray[1])>10e-6) ymax = ((obfloat)(ray[1] > 0.0 ? (yDim-1)*cellSize : 0) - tr[1]) / ray[1];
  obfloat idxMax = min(xmax, ymax);

  idxMin = max(idxMin, _idxMin);
  idxMax = min(idxMax, _idxMax);

  if (idxMin >= idxMax) return false;

  // Traverse partitions roughly to clip minimum index
  obfloat partitionSize = grid->getPartitionSize();
  for(obfloat i=idxMin; i<idxMax; i+=partitionSize)
  {
    obfloat tsd_tmp;
    position[0] = tr[0] + i * ray[0];
    position[1] = tr[1] + i * ray[1];
    EnumTsdGridInterpolate retval = grid->interpolateBilinear(position, &tsd_tmp);
    if(retval!=INTERPOLATE_EMPTYPARTITION && retval!=INTERPOLATE_INVALIDINDEX)
      break;
    else
      idxMin = i;
  }

  obfloat tsd_prev;
  position[0] = tr[0] + idxMin * ray[0];
  position[1] = tr[1] + idxMin * ray[1];
  if(grid->interpolateBilinear(position, &tsd_prev)!=INTERPOLATE_SUCCESS)
    tsd_prev = NAN;

  bool found = false;
  for(obfloat i=idxMin; i<=idxMax; i+=1.0)
  {
    position[0] += ray[0];
    position[1] += ray[1];

    obfloat tsd = NAN;
    if (grid->interpolateBilinear(position, &tsd)!=INTERPOLATE_SUCCESS)
    {
      tsd_prev = tsd;
      continue;
    }

    // Check sign change
    if(tsd_prev > 0 && tsd < 0)
    {
      interp = tsd_prev / (tsd_prev - tsd);
      found = true;
      break;
    }
    else if(tsd_prev < 0 && tsd > 0)
    {
      found = false;
      break;
    }

    tsd_prev = tsd;
  }

  if(!found)
  {
    return false;
  }

  coordinates[0] = position[0] + ray[0] * (interp-1.0);
  coordinates[1] = position[1] + ray[1] * (interp-1.0);

  return grid->interpolateNormal(coordinates, normal);
}

}

