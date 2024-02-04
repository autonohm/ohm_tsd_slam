#include "ProjectivePairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

void ProjectivePairAssignment::init(double* P, unsigned int width, unsigned int height, unsigned int dim)
{
  if(dim!=3)
  {
    cout << "WARNING: ProjectivePairAssignment not implemented for other dimensions than 3!" << endl;
    return;
  }
  _w     = width;
  _h     = height;
  _idx_m = new unsigned int[_w*_h];
  _P     = new double[12];
  memcpy(_P, P, 12 * sizeof(*P));
}

ProjectivePairAssignment::~ProjectivePairAssignment()
{
  delete [] _idx_m;
  delete [] _P;
}

void ProjectivePairAssignment::setModel(double** model, int size)
{
  memset(_idx_m, 0, _w*_h*sizeof(*(_idx_m)));

  for(unsigned int i=0; i<(unsigned int)size; i++)
  {
    double dw = _P[8] * model[i][0] + _P[9] * model[i][1] + _P[10] * model[i][2] + _P[11];
    if(fabs(dw)>1e-9)
    {
      double du  = (_P[0] * model[i][0] + _P[1] * model[i][1] + _P[2] * model[i][2] + _P[3]) / dw;
      double dv  = (_P[4] * model[i][0] + _P[5] * model[i][1] + _P[6] * model[i][2] + _P[7]) / dw;
      int u   = (int)(du + 0.5);
      int v   = (int)(dv + 0.5);

      int idx = v*_w + u;
      if((u<0) || (v<0) || (u>=(int)_w) || (v>=(int)_h)) continue;
      _idx_m[idx] = i;
    }
  }

  _model = model;

}

void ProjectivePairAssignment::determinePairs(double** scene, bool* mask, int size)
{
  for(unsigned int i=0; i<(unsigned int)size; i++)
  {
    if(mask[i])
    {
      double dw = _P[8] * scene[i][0] + _P[9] * scene[i][1] + _P[10] * scene[i][2] + _P[11];
      if(fabs(dw)>1e-9)
      {
        double du  = (_P[0] * scene[i][0] + _P[1] * scene[i][1] + _P[2] * scene[i][2] + _P[3]) / dw;
        double dv  = (_P[4] * scene[i][0] + _P[5] * scene[i][1] + _P[6] * scene[i][2] + _P[7]) / dw;
        int u   = (int)(du + 0.5);
        int v   = (int)(dv + 0.5);

        if((u<0) || (v<0) || (u>=(int)_w) || (v>=(int)_h))
        {
          addNonPair(i);
          continue;
        }
        int idx_img = v*_w + u;
        int idx_m = _idx_m[idx_img];
        if(idx_m==0)
        {
          addNonPair(i);
          continue;
        }
        double* ps = scene[i];
        double* pm = _model[idx_m];
        double dist  = distSqr3D<double>(ps,pm);
        addPair(idx_m, i, dist);
      }
      else
      {
        addNonPair(i);
      }
    }
    else
    {
      addNonPair(i);
    }
  }
}

}
