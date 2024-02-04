#include "OcclusionFilter.h"

#include "obcore/base/System.h"
#include "obcore/base/tools.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

namespace obvious
{

OcclusionFilter::OcclusionFilter(double* P, unsigned int width, unsigned int height)
{
  System<double>::allocate(height, width, _zbuffer);
  System<int>::allocate(height, width, _map);

  memcpy(_P, P, 12*sizeof(*_P));

  _w = width;
  _h = height;
};

OcclusionFilter::~OcclusionFilter()
{
  System<double>::deallocate(_zbuffer);
  System<int>::deallocate(_map);
};

void OcclusionFilter::update(double* P)
{
  memcpy(_P, P, 12*sizeof(*_P));
}

void OcclusionFilter::filter(double** scene, unsigned int size, bool* mask)
{
  if(!_active) return;

#ifndef NDEBUG
  unsigned char* proj = new unsigned char[3*_w * _h];
  memset(proj, 0, 3*_w*_h*sizeof(*proj));
#endif

  for(unsigned int i=0; i<_w*_h; i++)
  {
	_map[0][i] = -1;
	_zbuffer[0][i] = 10e6;
  }
  for(unsigned int i=0; i<size; i++)
  {
    double dw = _P[8] * scene[i][0] + _P[9] * scene[i][1] + _P[10] * scene[i][2] + _P[11];

    if(fabs(dw)>1e-12 && scene[i][2] > 0)
    {
      double du = (_P[0] * scene[i][0] + _P[1] * scene[i][1] + _P[2]  * scene[i][2] + _P[3]) / dw;
      double dv = (_P[4] * scene[i][0] + _P[5] * scene[i][1] + _P[6]  * scene[i][2] + _P[7]) / dw;
      int u = (int)(du + 0.5);
      int v = _h-1-(int)(dv + 0.5);
      if((u>=0) && (u<(int)_w) && (v>=0) && (v<(int)_h))
      {
    	  int mval = _map[v][u];
    	  // Two points intersect
    	  if(mval>-1)
        {
    		  if((_zbuffer[v][u] - scene[i][2]) > 1e-3)
    		  {
    		     mask[mval] = 0;
    		     _map[v][u] = i;
    		     _zbuffer[v][u] = scene[i][2];
    		  }
    		  else if((_zbuffer[v][u] - scene[i][2]) < -1e-3)
    		  {
    		    mask[i] = 0;
    		  }
        }
    	  else
    	  {
    		  _map[v][u] = i;
    		  _zbuffer[v][u] = scene[i][2];
    	  }
#ifndef NDEBUG
    	  int idx = v*_w + u;
        proj[3*idx]=255;
        proj[3*idx+1]=255;
        proj[3*idx+2]=255;
#endif
    	}
      }
  }
#ifndef NDEBUG
  serializePPM("/tmp/projo.ppm", proj, _w, _h);
  delete [] proj;
#endif
}

}
