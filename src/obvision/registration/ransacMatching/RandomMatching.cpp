#include "RandomMatching.h"

#include "obcore/base/Logger.h"
#include <math.h>

namespace obvious
{

RandomMatching::RandomMatching(unsigned int sizeControlSet)
{
  _sizeControlSet   = sizeControlSet;
}

RandomMatching::~RandomMatching()
{
  if(_trace)
    delete _trace;
  _trace = NULL;
}

void RandomMatching::activateTrace()
{
  if(!_trace)
    _trace = new Trace(2);
}

void RandomMatching::deactivateTrace()
{
  if(_trace) delete _trace;
  _trace = NULL;
}

void RandomMatching::serializeTrace(const char* folder)
{
  if(_trace)
    _trace->serialize(folder);
  else
    LOGMSG(DBG_ERROR, "Trace not activated");
}

std::vector<unsigned int> RandomMatching::extractSamples(obvious::Matrix* M, const bool* mask, unsigned int searchRange)
{
  std::vector<unsigned int> validIndices;
  for(unsigned int i=searchRange; i<M->getRows()-searchRange; i++)
  {
    if(mask[i])
      validIndices.push_back(i);
  }
  return validIndices;
}

obvious::Matrix* RandomMatching::pickControlSet(obvious::Matrix* M, std::vector<unsigned int> idxValid, std::vector<unsigned int> &idxControl)
{
  unsigned int sizeControlSet = _sizeControlSet;
  if((idxValid.size()) < sizeControlSet)
  {
    LOGMSG(DBG_DEBUG, "Size of scene smaller than control set ... reducing size to " << idxValid.size());
    sizeControlSet = idxValid.size();
  }
  obvious::Matrix* C = new obvious::Matrix(3, sizeControlSet);
  std::vector<unsigned int> idxTemp = idxValid;
  unsigned int ctr = 0;
  while(idxControl.size() < sizeControlSet)
  {
    unsigned int r = rand() % idxTemp.size();
    unsigned int idx = idxTemp[r];
    idxControl.push_back(idx);
    idxTemp.erase(idxTemp.begin() + r);

    (*C)(0, ctr)   = (*M)(idx, 0);
    (*C)(1, ctr)   = (*M)(idx, 1);
    (*C)(2, ctr++) = 1.0;
  }
  return C;
}

void RandomMatching::calcNormals(Matrix* M, Matrix* N, const bool* maskIn, bool* maskOut, int searchRadius)
{
  int points = M->getRows();

  // mask borders at which we cannot calculate normal vectors
  for(int i=0; i<searchRadius; i++)
    maskOut[i] = false;
  for(int i=points-searchRadius; i<points; i++)
    maskOut[i] = false;

  for(int i=searchRadius; i<points-searchRadius; i++)
  {
    if(maskIn[i])
    {
      unsigned int cnt = 0;

      for(int j=-searchRadius; j<searchRadius; j++)
        if(maskIn[i+j]) cnt++;

      // check for minimum points necessary to perform PCA
      if(cnt>3)
      {
        Matrix A(cnt, 2);
        cnt = 0;
        for(int j=-searchRadius; j<searchRadius; j++)
        {
          if(maskIn[i+j])
          {
            A(cnt, 0) = (*M)(i+j, 0);
            A(cnt, 1) = (*M)(i+j, 1);
            cnt++;
          }
        }
        Matrix* Axes = A.pcaAnalysis();
        // longer axis
        double xLong  = (*Axes)(0,1)-(*Axes)(0,0);
        double yLong  = (*Axes)(0,3)-(*Axes)(0,2);
        // shorter axis
        double xShort = (*Axes)(1,1)-(*Axes)(1,0);
        double yShort = (*Axes)(1,3)-(*Axes)(1,2);
        // rate axes lengths -> main axis needs to be twice as long as second axis
        double lenLongSqr  = xLong*xLong + yLong*yLong;
        double lenShortSqr = xShort*xShort + yShort*yShort;

        if(lenShortSqr>1e-6 && (lenLongSqr/lenShortSqr)<4.0)
        {
          maskOut[i] = false;
          continue;
        }

        // shorter axis is normal
        double len = sqrt(lenShortSqr);
        if(((*M)(i,0)*xShort+(*M)(i,1)*yShort)<0.0)
        {
          (*N)(i, 0) = xShort / len;
          (*N)(i, 1) = yShort / len;
        }
        else
        {
          (*N)(i, 0) = -xShort / len;
          (*N)(i, 1) = -yShort / len;
        }

        delete Axes;
      }
      else
        maskOut[i] = false;
    }
  }
}

void RandomMatching::calcPhi(Matrix* N,  const bool* mask, double* phi)
{
  if(mask==NULL)
  {
    for(unsigned int i=0; i<N->getRows(); i++)
      phi[i] = atan2((*N)(i,1), (*N)(i, 0));
  }
  else
  {
    for(unsigned int i=0; i<N->getRows(); i++)
    {
      if(mask[i])
      {
        phi[i] = atan2((*N)(i,1), (*N)(i, 0));
      }
      else
      {
        phi[i] = -1e6;
      }
    }
  }
}

void RandomMatching::subsampleMask(bool* mask, unsigned int size, double probability)
{
  if(probability>1.0) probability = 1.0;
  if(probability<0.0) probability = 0.0;
  int probability_thresh = (int)(1000.0 - probability * 1000.0 + 0.5);
  for(unsigned int i=0; i<size; i++)
  {
    if((rand()%1000)<probability_thresh)
    {
      mask[i] = 0;
    }
  }
}

} /* namespace obvious */
