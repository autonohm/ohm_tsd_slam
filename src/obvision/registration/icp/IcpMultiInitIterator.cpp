#include "obvision/registration/icp/IcpMultiInitIterator.h"
#include "obcore/base/Logger.h"

namespace obvious
{

IcpMultiInitIterator::IcpMultiInitIterator(vector<Matrix> Tinit)
{
  if(Tinit.size()==0)
  {
    LOGMSG(DBG_ERROR, "Initialization vector of length zero passed");
    abort();
  }

  for(vector<Matrix>::iterator it=Tinit.begin(); it!=Tinit.end(); it++)
    _Tinit.push_back(*it);

  _Tlast = NULL;
}

IcpMultiInitIterator::~IcpMultiInitIterator()
{
  delete _Tlast;
}

inline bool assignBetterSolution(double &rmsBest, unsigned int &pairsBest, unsigned int &iterationsBest, Matrix &TBest, double rms, unsigned int pairs, unsigned int iterations, Matrix T)
{
  if(pairs > pairsBest)
  {
    rmsBest = rms;
    pairsBest = pairs;
    iterationsBest = iterations;
    TBest = T;
    return true;
  }

  return false;
}

Matrix IcpMultiInitIterator::iterate(Icp* icp)
{
  double rmsBest = 10e12;
  unsigned int pairsBest = 0;
  unsigned int iterationsBest = 10e4;
  Matrix TBest = icp->getFinalTransformation();

  // Temporary results
  double rms;
  unsigned int pairs;
  unsigned int iterations;

  //unsigned int cnt = 0;

  for(vector<Matrix>::iterator it=_Tinit.begin(); it!=_Tinit.end(); it++)
  {
    icp->reset();
    icp->iterate(&rms, &pairs, &iterations, &(*it));
    assignBetterSolution(rmsBest, pairsBest, iterationsBest, TBest, rms, pairs, iterations, icp->getFinalTransformation());

    /*char folder[32];
    sprintf(folder, "trace_%d_%d", cnt++, retval);
    icp->serializeTrace(folder, 50);*/
  }

  if(_Tlast)
  {
    // Apply matching with last transformation
    icp->reset();
    icp->iterate(&rms, &pairs, &iterations);
    assignBetterSolution(rmsBest, pairsBest, iterationsBest, TBest, rms, pairs, iterations, *_Tlast);

    /*char folder[32];
    sprintf(folder, "trace_%d_%d", cnt++, retval);
    icp->serializeTrace(folder, 50);*/
  }
  else
  {
    _Tlast = new Matrix(TBest.getRows(), TBest.getCols());
  }

  (*_Tlast) = TBest;

  return TBest;
}

}
