#include "CartesianCloud.h"

#include <string.h>
#include <obcore/math/mathbase.h>

namespace obvious
{
CartesianCloud3D::CartesianCloud3D(unsigned int size, double* coords, unsigned char* rgb, double* normals)
{
  _coords  = NULL;
  _colors  = NULL;
  _normals = NULL;
  _size    = size;
  bool withInfo = (rgb!=NULL);
  init(size, withInfo);

  _coords->setData(coords);

  if(normals)
  {
    _normals->setData(normals);
    _hasNormals = 1;
  }

  if(rgb)
  {
    memcpy(_colors, rgb, 3*size*sizeof(*_colors));
    _hasColors = true;
  }

}

CartesianCloud3D::CartesianCloud3D(unsigned int size, bool withInfo)
{
  init(size, withInfo);
}

CartesianCloud3D::CartesianCloud3D(CartesianCloud3D* cloud)
{
  init(cloud->size(), cloud->hasInfo());
  *_coords = *(cloud->_coords);
  *_normals= *(cloud->_normals);
  if(cloud->hasInfo())
  {
    memcpy(_colors, cloud->_colors, cloud->size()*3*sizeof(*_colors));
    memcpy(_attributes, cloud->_attributes, cloud->size()*sizeof(*_attributes));
    memcpy(_indices, cloud->_indices, cloud->size()*sizeof(*_indices));
  }
}

void CartesianCloud3D::init(unsigned int size, bool withInfo)
{
  _hasInfo    = 0;
  _hasNormals = 0;
  _coords   = new Matrix(size, 3);
  _normals  = new Matrix(size, 3);

  if (withInfo)
  {
    _hasInfo   = 1;
    _colors     = new unsigned char[size*3];
    _attributes = new int[size];
    _indices    = new int[size];

    for (unsigned int i = 0; i < size; i++)
    {
      _colors[i*3]   = 255;
      _colors[i*3+1] = 255;
      _colors[i*3+2] = 255;

      _attributes[i] = ePointAttrValid;

      _indices[i]    = i;
    }
  }
}

CartesianCloud3D::~CartesianCloud3D()
{
  _mSourceInfo.clear();

  if(_hasInfo)
  {
    delete[] _colors;
    delete[] _attributes;
    delete[] _indices;
  }

  delete _coords;
  delete _normals;
}

/*double* CartesianCloud3D::operator [](unsigned int i)
{
  return (*_coords)[i];
}*/

Matrix* CartesianCloud3D::getCoords()
{
  return _coords;
}

void CartesianCloud3D::setNormals(Matrix* normals)
{
  *_normals = *normals;
}

Matrix* CartesianCloud3D::getNormals()
{
  return _normals;
}

unsigned char* CartesianCloud3D::getColors()
{
  return _colors;
}

int* CartesianCloud3D::getAttributes()
{
  return _attributes;
}

int* CartesianCloud3D::getIndices()
{
  return _indices;
}

int CartesianCloud3D::hasInfo()
{
  return _hasInfo;
}

int CartesianCloud3D::hasSourceInfo()
{
  return (!_mSourceInfo.empty());
}

void CartesianCloud3D::addSourceInfo(EnumSourceInfo eSourceInfo, long lValue)
{
  _mSourceInfo[eSourceInfo] = lValue;
}

int CartesianCloud3D::getSourceInfo(EnumSourceInfo eSourceInfo, long* plValue)
{
  int nRetval = 0;
  if (_mSourceInfo.find(eSourceInfo) != _mSourceInfo.end())
  {
    nRetval = 1;
    *plValue = _mSourceInfo[eSourceInfo];
  }
  return nRetval;
}

void CartesianCloud3D::clearSourceInfo()
{
  _mSourceInfo.clear();
}

void CartesianCloud3D::maskPoints(bool* mask)
{
  for (unsigned int i=0; i<_coords->getRows(); i++)
  {
    if(!mask[i])_attributes[i] &= ~ePointAttrValid;
  }
}

void CartesianCloud3D::maskEmptyNormals()
{
  for (unsigned int i=0; i<_coords->getRows(); i++)
  {
    //double len = (*_normals)[i][0]*(*_normals)[i][0] + (*_normals)[i][1]*(*_normals)[i][1] + (*_normals)[i][2]*(*_normals)[i][2];
    double len = (*_normals)(i,0)*(*_normals)(i,0)+(*_normals)(i,1)*(*_normals)(i,1)+(*_normals)(i,2)*(*_normals)(i,2);
    if(len<10e-6)_attributes[i] &= ~ePointAttrValid;
  }
}

void CartesianCloud3D::removeInvalidPoints()
{

  if(!_hasInfo) return;

  unsigned int i;
  int cnt = 0;

  for (i=0; i<_coords->getRows(); i++)
  {
    if(_attributes[i] & ePointAttrValid)
    {
      cnt++;
    }
  }
  Matrix C(cnt, 3);
  Matrix N(cnt, 3);
  cnt = 0;

  for (i=0; i<_coords->getRows(); i++)
  {
    if(_attributes[i] & ePointAttrValid)
    {
      C(cnt,0) = (*_coords)(i,0);
      C(cnt,1) = (*_coords)(i,1);
      C(cnt,2) = (*_coords)(i,2);
      N(cnt,0) = (*_normals)(i,0);
      N(cnt,1) = (*_normals)(i,1);
      N(cnt,2) = (*_normals)(i,2);
      _colors[3*cnt]   = _colors[3*i];
      _colors[3*cnt+1] = _colors[3*i+1];
      _colors[3*cnt+2] = _colors[3*i+2];
      _indices[cnt]    = _indices[i];
      _attributes[cnt] = _attributes[i];
      cnt++;
    }
  }

  delete _coords;
  delete _normals;
  _coords = new Matrix(C);
  _normals = new Matrix(N);
}

void CartesianCloud3D::subsample(unsigned int step)
{
  for (unsigned int i=0; i<_coords->getRows(); i++)
  {
    if(i%step!=0)
      _attributes[i] &= ~ePointAttrValid;
  }
  removeInvalidPoints();
}

unsigned int CartesianCloud3D::size()
{
  return _coords->getRows();
}

void CartesianCloud3D::transform(Matrix* T)
{
  // Apply rotation
  Matrix R(*T, 0, 0, 3, 3);
  _coords->multiplyRight(R, false, true);

  // Apply translation
  _coords->addToColumn(0, (*T)(0,3));
  _coords->addToColumn(1, (*T)(1,3));
  _coords->addToColumn(2, (*T)(2,3));
}

void CartesianCloud3D::transform(double T[16])
{
  Matrix M(4, 4, T);
  transform(&M);
}

void CartesianCloud3D::createProjection(unsigned char* pImage, unsigned char* pMask, Matrix* P, int nW, int nH)
{
  Vector xi(4);
  xi(3) = 1.0;

  memset(pImage, 0, 3*nW*nH*sizeof(unsigned char));
  memset(pMask, 0, nW*nH*sizeof(unsigned char));

  for(unsigned int i=0; i<_coords->getRows(); i++)
  {
    if(_attributes[i] & ePointAttrValid)
    {
      xi(0) = (*_coords)(i,0);
      xi(1) = (*_coords)(i,1);
      xi(2) = (*_coords)(i,2);

      Vector ni = Matrix::multiply(*P, xi, false);
      double du = ni(0);
      double dv = ni(1);
      double dw = ni(2);
      if(dw > 10e-6)
      {
        int u = nW-1-(int)( du / dw + 0.5);
        int v = (int)(dv / dw + 0.5);

        if((u>=0) && (u<nW) && (v>=0) && (v<nH))
        {
          pImage[(v*nW+u)*3]   = _colors[3*i];
          pImage[(v*nW+u)*3+1] = _colors[3*i+1];
          pImage[(v*nW+u)*3+2] = _colors[3*i+2];;
          pMask[v*nW+u]        = 255;
        }
      }
    }
  }
}


void CartesianCloud3D::createZBuffer(unsigned char* pImage, double* zbuffer, Matrix* P, int nW, int nH)
{
  if(!_hasInfo) return;

  Vector xi(4);

  memset(pImage, 0, 3*nW*nH*sizeof(unsigned char));
  for(int i=0; i<nW*nH; i++)
    zbuffer[i] = 0.0;

  for(unsigned int i=0; i<_coords->getRows(); i++)
  {
    if(_attributes[i] & ePointAttrValid)
    {
      xi(0) = (*_coords)(i,0);
      xi(1) = (*_coords)(i,1);
      xi(2) = (*_coords)(i,2);

      Vector ni = Matrix::multiply(*P, xi, false);
      double du = ni(0);
      double dv = ni(1);
      double dw = ni(2);
      if(dw > 10e-6)
      {
        int u = nW-1-(int)( du / dw + 0.5);
        int v = (int)(dv / dw + 0.5);

        if((u>=0) && (u<nW) && (v>=0) && (v<nH))
        {
          pImage[(v*nW+u)*3]   = _colors[3*i];
          pImage[(v*nW+u)*3+1] = _colors[3*i+1];
          pImage[(v*nW+u)*3+2] = _colors[3*i+2];
          zbuffer[v*nW+u]      = xi(2);
        }
      }
    }
  }
}

void CartesianCloud3D::setData(const unsigned int size, double* coords, double* normals, const unsigned char* const rgb)
{
  delete _coords;
  _coords = new Matrix(size, 3, coords);

  if(_normals)
  {
    delete _normals;
    _normals = NULL;
  }
  if(normals)
  {
    _normals = new Matrix(size, 3, normals);
    _hasNormals = 1;
  }

  _mSourceInfo.clear();
  if(_hasInfo)
  {
    delete[] _colors;
    delete[] _attributes;
    delete[] _indices;
  }
  if(rgb)
  {
    _hasInfo   = 1;
    _colors     = new unsigned char[size*3];
    _attributes = new int[size];
    _indices    = new int[size];

    for (unsigned int i = 0; i < size; i++)
    {
      _attributes[i] = ePointAttrValid;
      _indices[i]    = i;
    }
    memcpy(_colors, rgb, size*3*sizeof(*_colors));
  }
}

}
