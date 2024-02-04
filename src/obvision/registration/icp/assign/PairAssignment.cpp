#include "PairAssignment.h"
#include <math.h>
#include "obcore/base/Timer.h"
namespace obvious
{

PairAssignment::PairAssignment()
{
	_dimension   = DEFAULTDIMENSION;
	_pairs        = &_initPairs;
	_distancesSqr = &_initDistancesSqr;
}

PairAssignment::PairAssignment(int dimension)
{
	_dimension   = dimension;
	_pairs        = &_initPairs;
	_distancesSqr = &_initDistancesSqr;
}

PairAssignment::~PairAssignment()
{
	_initPairs.clear();
	_nonPairs.clear();
	_initDistancesSqr.clear();
}

void PairAssignment::addPreFilter(IPreAssignmentFilter* filter)
{
  _vPrefilter.push_back(filter);
}

void PairAssignment::addPostFilter(IPostAssignmentFilter* filter)
{
  _vPostfilter.push_back(filter);
}

void PairAssignment::determinePairs(double** scene, int size)
{
  unsigned int i;
  bool* mask = new bool[size];
  memset(mask, 1, size * sizeof(*mask));
  for(i=0; i<_vPrefilter.size(); i++)
  {
    IPreAssignmentFilter* filter = _vPrefilter[i];
    filter->filter(scene, size, mask);
  }
  clearPairs();
  determinePairs(scene, mask, size);

  _pairs        = &_initPairs;
  _distancesSqr = &_initDistancesSqr;

  bool hasPrecedingFilter = false;
  for(i=0; i<_vPostfilter.size(); i++)
  {
    IPostAssignmentFilter* filter = _vPostfilter[i];

    if(!filter->_active) continue;

    if(hasPrecedingFilter)
    {
      _initPairs.clear();
      _initDistancesSqr.clear();
      _initPairs         = _filteredPairs;
      _initDistancesSqr  = _filteredDistancesSqr;
    }

    filter->filter(_model,
                   scene,
                   &_initPairs,
                   &_initDistancesSqr,
                   &_filteredPairs,
                   &_filteredDistancesSqr,
                   &_nonPairs);
    _pairs = &_filteredPairs;
    _distancesSqr = &_filteredDistancesSqr;

    hasPrecedingFilter = true;
  }

  delete[] mask;

}

vector<StrCartesianIndexPair>* PairAssignment::getPairs()
{
	return _pairs;
}

vector<double>* PairAssignment::getDistancesSqr()
{
	return _distancesSqr;
}

vector<unsigned int>* PairAssignment::getNonPairs()
{
	return &_nonPairs;	
}

void PairAssignment::addPair(unsigned int indexModel, unsigned int indexScene, double distanceSqr)
{
	StrCartesianIndexPair pair;
	pair.indexFirst = indexModel;
	pair.indexSecond = indexScene;
	_initPairs.push_back(pair);
	_initDistancesSqr.push_back(distanceSqr);
}

void PairAssignment::addNonPair(unsigned int indexScene)
{
	_nonPairs.push_back(indexScene);
}

int PairAssignment::getDimension()
{
	return _dimension;
}

void PairAssignment::clearPairs()
{
	_initPairs.clear();
	_filteredPairs.clear();
	_nonPairs.clear();
	_initDistancesSqr.clear();
	_filteredDistancesSqr.clear();
}

void PairAssignment::reset()
{
  clearPairs();

  for(unsigned int i=0; i<_vPostfilter.size(); i++)
  {
    IPostAssignmentFilter* filter = _vPostfilter[i];
    filter->reset();
  }
}

}	
