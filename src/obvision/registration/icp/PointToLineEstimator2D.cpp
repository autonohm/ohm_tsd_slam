#include "PointToLineEstimator2D.h"
#include <iostream>
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include "obcore/math/linalg/linalg.h"

using namespace std;
using namespace obvious;

enum Coords
{
  x = 0,
  y = 1,
  z = 2
};

namespace obvious
{

PointToLine2DEstimator ::PointToLine2DEstimator ()
{
  _model        = NULL;
  _scene        = NULL;
  _normals      = NULL;
  _pairs        = NULL;
  _rms          = 10000.0;
  _iterations   = 0;

  for (unsigned int i=0 ; i<=2 ; i++)
  {
    _cs[i] = 0.0;
    _cm[i] = 0.0;
  }
}

PointToLine2DEstimator::~PointToLine2DEstimator (void)
{

}

void PointToLine2DEstimator::setModel(double** model, unsigned int size, double** normals)
{
  _model   = model;
  _normals = normals;
}

void PointToLine2DEstimator::setScene(double** scene, unsigned int size, double** normals)  // normals are ignored in this class
{
  _scene = scene;
}

void PointToLine2DEstimator::setPairs(std::vector<StrCartesianIndexPair>* pairs)
{
  _pairs = pairs;
  _rms = 0.0;

  unsigned int size = pairs->size();

  for(unsigned int i=0; i<size; i++)
  {
    StrCartesianIndexPair pair = (*pairs)[i];
    double* pointModel         = _model[pair.indexFirst];
    double* pointScene         = _scene[pair.indexSecond];
    double* pointNorm          = _normals[pair.indexFirst];
    //Equation 7 from the paper below
    //((scene point - model point) * normal)²
    // Note: scene point is already transformed
    double* MtoSVec = new double[2];
    MtoSVec[0] = pointScene[0] - pointModel[0];
    MtoSVec[1] = pointScene[1] - pointModel[1];
    _rms += fabs( MtoSVec[0] * pointNorm[0] + MtoSVec[1] * pointNorm[1]);
  }
  _rms /= (double)size;
}

double PointToLine2DEstimator::getRMS()
{
  return _rms;
}

/**
 * Estimation based on paper: Sickel, Konrad; Bubnik, Vojtech, Iterative Closest Point Algorithm for Rigid Registration of Ear Impressions,
 * In: Bauman Moscow State Technical University (Eds.) Proceedings of the 6-th Russian-Bavarian Conference on Bio-Medical Engineering
 * (6th Russian Bavarian Conference on Bio-Medical Engineering Moscow, Russia 8-12.11.2010) 2010, pp. 142-145
 */
void PointToLine2DEstimator::estimateTransformation(Matrix* T)
{
  if(_normals==NULL)
  {
    cout << "WARNING (" << __PRETTY_FUNCTION__ << "): Normals not set." << endl;
    return;
  }
  _iterations++;
  unsigned int size = _pairs->size();

  Matrix A(3, 3);
  A.setZero();
  double b[3] = {0.0, 0.0, 0.0};

  for(unsigned int i=0 ; i< size ; i++)
  {
    StrCartesianIndexPair pair = (*_pairs)[i];

    double* q  = _model[  pair.indexFirst];
    double* p  = _scene[  pair.indexSecond];
    double* n  = _normals[pair.indexFirst];

    double a[3];
    a[x] = 0;
    a[y] = 0;
    a[z] = p[x]*n[y] - p[y]*n[x];

    A(0,0) += a[z]*a[z];             A(0,1) += a[z]*n[x];             A(0,2) += a[z]*n[y];
    A(1,0) += a[z]*n[x];             A(1,1) += n[x]*n[x];             A(1,2) += n[x]*n[y];
    A(2,0) += a[z]*n[y];             A(2,1) += n[y]*n[x];             A(2,2) += n[y]*n[y];

    double pmq[3];
    pmq[x] = p[x]-q[x];
    pmq[y] = p[y]-q[y];
    pmq[z] = 0;

    double tmp = pmq[0]*n[0] + pmq[1]*n[1];

    b[0]-= a[z]*tmp;
    b[1]-= n[x]*tmp;
    b[2]-= n[y]*tmp;
  }

  double x[3];
  A.solve(b, x);

  const double psi   = x[0];
  const double theta = 0.0;
  const double phi   = 0.0;

  double cph = cos(phi);
  double cth = cos(theta);
  double cps = cos(psi);
  double sph = sin(phi);
  double sth = sin(theta);
  double sps = sin(psi);

  (*T)(0,0) = cth*cps;
  (*T)(0,1) = -cph*sps+sph*sth*cps;
  (*T)(0,2) = sph*sth+cph*sth*cps;

  (*T)(1,0) = cth*sps;
  (*T)(1,1) = cph*cps+sph*sth*sps;
  (*T)(1,2) = -sph*cps+cph*sth*sps;

  (*T)(2,0) = -sth;
  (*T)(2,1) = sph*cth;
  (*T)(2,2) = cph*cth;

  (*T)(0,3) = x[1];
  (*T)(1,3) = x[2];
}

unsigned int PointToLine2DEstimator::getIterations(void)
{
  return(_iterations);
}

}
