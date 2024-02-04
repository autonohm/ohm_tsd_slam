#ifndef SENSOR_H
#define SENSOR_H

#include "obcore/base/Logger.h"
#include "Sensor.h"
#include <string.h>

namespace obvious
{

Sensor::Sensor(unsigned int dim, double maxRange, double minRange, double lowReflectivityRange)
{
  _dim = dim;
  _maxRange = maxRange;
  _minRange = minRange;
  _lowReflectivityRange = lowReflectivityRange;

  _rgb      = NULL;
  _accuracy = NULL;
  _typeID   = NULL;

  _rayNorm = 1.0;

  _T = new Matrix(_dim+1, _dim+1);
  _T->setIdentity();
}

Sensor::~Sensor()
{
  delete _T;
  if(_rgb)      delete [] _rgb;
  if(_accuracy) delete [] _accuracy;
  if(_typeID)   delete [] _typeID;
}

Matrix* Sensor::getNormalizedRayMap(double norm)
{
  if(norm != _rayNorm)
  {
    for(unsigned int i=0; i<_size; i++)
    {
      for(unsigned int j=0; j<_dim; j++)
        (*_rays)(j, i) *= (norm/_rayNorm);
    }
    _rayNorm = norm;
  }
  return _rays;
}

void Sensor::transform(Matrix* T)
{
  Matrix R(*T, 0, 0, _dim, _dim);

  (*_rays) = R * (*_rays);

  // Transform sensor in his own coordinate system
  // P' = P T = [Rp | tp] [R | t] = [Rp R | Rp t + tp]
  // , where P is the old and P' is the new pose
  (*_T) = (*_T) * *T;
}

void Sensor::translate(double* tr)
{
  for(unsigned int r=0; r<_dim; r++)
    (*_T)(r, _dim) += tr[r];
}

unsigned int Sensor::getWidth()
{
  return _width;
}

unsigned int Sensor::getHeight()
{
  if(_dim<3)
  {
    LOGMSG(DBG_ERROR, "Sensor does not provide a two-dimensional measurement array");
    return 0;
  }
  return _height;
}

double Sensor::getMaximumRange()
{
  return _maxRange;
}

double Sensor::getMinimumRange()
{
  return _minRange;
}

double Sensor::getLowReflectivityRange()
{
  return _lowReflectivityRange;
}

Matrix Sensor::getTransformation()
{
  Matrix T = *_T;
  return T;
}

void Sensor::setTransformation(Matrix T)
{
  *_T = T;
}

void Sensor::resetTransformation()
{
  _T->setIdentity();
}

void Sensor::getPosition(obfloat* tr)
{
  for(unsigned int i=0; i<_dim; i++)
    tr[i] = (*_T)(i, _dim);
}

unsigned int Sensor::getRealMeasurementSize()
{
  return _size;
}

void Sensor::setRealMeasurementData(double* data, double scale)
{
  if(scale==1.0)
    memcpy(_data, data, _size*sizeof(*data));
  else
  {
    for(unsigned int i=0; i<_size; i++)
      _data[i] = data[i] * scale;
  }
}

void Sensor::setRealMeasurementData(vector<float> data, float scale)
{
  if(data.size()!=_size)
  {
    LOGMSG(DBG_WARN, "Size of measurement array wrong, expected " << _size << " obtained: " << data.size());
  }

  for(unsigned int i=0; i<data.size(); i++)
    _data[i] = (double)(data[i] * scale);
}

double* Sensor::getRealMeasurementData()
{
  return _data;
}

unsigned int Sensor::dataToCartesianVector(double* &coords)
{
  unsigned int cnt = 0;
  for(unsigned int i=0; i<_size; i++)
  {
    if(!isinf(_data[i]) && _mask[i])
    {
      for(unsigned int j=0; j<_dim; j++)
      {
        coords[cnt++] = (*_raysLocal)(j, i) * _data[i];
      }
    }
  }
  return cnt;
}

unsigned int Sensor::dataToCartesianVectorMask(double* &coords, bool* &validityMask)
{
  unsigned int cnt = 0;
  unsigned int validPoints = 0;
  for(unsigned int i = 0; i < _size; i++)
  {
    if(!isinf(_data[i]) && _mask[i])
    {
      for(unsigned int j = 0; j < _dim; j++)
      {
        coords[cnt++] = (*_raysLocal)(j, i) * _data[i];
      }
      validPoints++;
      validityMask[i] = true;
    }
    else
    {
      cnt+=_dim;
      validityMask[i] = false;
    }
  }
  return validPoints;
}

unsigned int Sensor::removeInvalidPoints(double* inPoints, bool* mask, unsigned int sizeMask, double* outPoints)
{
  unsigned int cnt = 0;
  for(unsigned int i = 0; i < sizeMask; i++)
  {
    if(mask[i])
    {
      outPoints[cnt*2]   = inPoints[i*2];
      outPoints[cnt*2+1] = inPoints[i*2+1];
      cnt++;
    }
  }
  return cnt;
}

Matrix Sensor::dataToHomogeneousCoordMatrix()
{
  Matrix M(_dim+1, _size);
  for(unsigned int i=0; i<_size; i++)
  {
    for(unsigned int j=0; j<_dim; j++)
      M(j, i) = (*_raysLocal)(j, i) * _data[i];
    M(_dim, i) = 1.0;
  }
  return M;
}

void Sensor::setRealMeasurementAccuracy(double* accuracy)
{
  if(!_accuracy) _accuracy = new double[_size];
  memcpy(_accuracy, accuracy, _size*sizeof(*accuracy));
}

double* Sensor::getRealMeasurementAccuracy()
{
  return _accuracy;
}

bool Sensor::hasRealMeasurementAccuracy()
{
  return (_accuracy!=NULL);
}

void Sensor::setRealMeasurementMask(bool* mask)
{
  memcpy(_mask, mask, _size*sizeof(*mask));
}

void Sensor::setRealMeasurementMask(vector<unsigned char> mask)
{
  for(unsigned int i=0; i<mask.size(); i++)
    _mask[i] = mask[i];
}

void Sensor::resetMask()
{
  for(unsigned int i=0; i<_size; i++)
    _mask[i] = true;
}

void Sensor::maskZeroDepth()
{
  for(unsigned int i=0; i<_size; i++)
    _mask[i] =  _mask[i] && (_data[i]!=0.0);
}

void Sensor::maskInvalidDepth()
{
  for(unsigned int i=0; i<_size; i++)
  {
    if(_data[i]>_maxRange) _data[i] = INFINITY;

    if(isnan(_data[i]))
    {
      _mask[i] = false;
      _data[i] = INFINITY;
    }

    //_mask[i] =  _mask[i] && (!isinf(_data[i]));
  }
}

bool* Sensor::getRealMeasurementMask()
{
  return _mask;
}

bool Sensor::hasRealMeasurementRGB()
{
  return (_rgb!=NULL);
}

void Sensor::setRealMeasurementRGB(unsigned char* rgb)
{
  if(!_rgb) _rgb = new unsigned char[_size*3];
  memcpy(_rgb, rgb, _size*3*sizeof(*rgb));
}

unsigned char* Sensor::getRealMeasurementRGB()
{
  return _rgb;
}

void Sensor::setRealMeasurementTypeID(int* typeID)
{
  if(!_typeID) _typeID = new int[_size];
  memcpy(_typeID, typeID, _size*sizeof(*typeID));
}

int* Sensor::getRealMeasurementTypeID()
{
  return _typeID;
}

}

#endif
