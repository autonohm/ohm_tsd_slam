#include "ClosedFormEstimator2D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"

using namespace obvious;

namespace obvious
{

ClosedFormEstimator2D::ClosedFormEstimator2D()
{
  _rms = 0.0;
  _cm[0] = 0.0;
  _cm[1] = 0.0;
  _cs[0] = 0.0;
  _cs[1] = 0.0;
  _pairs = NULL;
}

ClosedFormEstimator2D::~ClosedFormEstimator2D()
{

}

void ClosedFormEstimator2D::setModel(double** model, unsigned int size, double** normals)
{
  _model = model;
}

void ClosedFormEstimator2D::setScene(double** scene, unsigned int size, double** normals)
{
  _scene = scene;
}

void ClosedFormEstimator2D::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;

  _rms = 0.0;

  // Compute centroids
  _cm[0] = 0.0;
  _cm[1] = 0.0;
  _cs[0] = 0.0;
  _cs[1] = 0.0;

  unsigned int size = pairs->size();

  for (unsigned int i = 0; i < size; i++)
  {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel = _model[pair.indexFirst];
    double* pointScene = _scene[pair.indexSecond];
    _cm[0] += pointModel[0];
    _cm[1] += pointModel[1];
    _cs[0] += pointScene[0];
    _cs[1] += pointScene[1];
    _rms += distSqr2D(pointModel, pointScene);
  }
  double sizeInv = 1.0 / (double) size;
  _rms   *= sizeInv;
  _cm[0] *= sizeInv;
  _cm[1] *= sizeInv;
  _cs[0] *= sizeInv;
  _cs[1] *= sizeInv;
}

double ClosedFormEstimator2D::getRMS()
{
  return _rms;
}

void ClosedFormEstimator2D::estimateTransformation(Matrix* T)
{
  double nominator = 0.0, denominator = 0.0;

  for (unsigned int i = 0; i < (*_pairs).size(); i++)
  {
    double xFCm = _model[(*_pairs)[i].indexFirst][0]  - _cm[0];
    double yFCm = _model[(*_pairs)[i].indexFirst][1]  - _cm[1];
    double xSCs = _scene[(*_pairs)[i].indexSecond][0] - _cs[0];
    double ySCs = _scene[(*_pairs)[i].indexSecond][1] - _cs[1];
    nominator   += yFCm * xSCs - xFCm * ySCs;
    denominator += xFCm * xSCs + yFCm * ySCs;
  }

  // compute rotation
  double deltaTheta = atan2(nominator, denominator);

  double cosDeltaTheta = cos(deltaTheta);
  double sinDeltaTheta = sin(deltaTheta);

  // compute translation
  double deltaX = (_cm[0] - (cosDeltaTheta * _cs[0] - sinDeltaTheta * _cs[1]));
  double deltaY = (_cm[1] - (cosDeltaTheta * _cs[1] + sinDeltaTheta * _cs[0]));

  // fill result matrix
  T->setIdentity();
  (*T)(0,0) = cosDeltaTheta;
  (*T)(0,1) = -sinDeltaTheta;
  (*T)(0,3) = deltaX;

  (*T)(1,0) = sinDeltaTheta;
  (*T)(1,1) = cosDeltaTheta;
  (*T)(1,3) = deltaY;

  (*T)(2,3) = 0;
}

}
