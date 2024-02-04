#include "AnnPairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

void AnnPairAssignment::init()
{
	_tree = NULL;
}

AnnPairAssignment::~AnnPairAssignment()
{
	if(_tree)
	{
		delete _tree;
		_tree = NULL;
		annClose();
	}
}

void AnnPairAssignment::setMaxVisitPoints(unsigned int visitPoints)
{
	annMaxPtsVisit(visitPoints);
}

void AnnPairAssignment::setModel(double** model, int size)
{
	
	if(_tree)
	{
		delete _tree;
		_tree = NULL;
	}
	_model = model;
	_tree = new ANNkd_tree(			// build search structure
				 			model,  	      // the data points
				 			size,		        // number of points
				 			_dimension);   // dimension of space
}

void AnnPairAssignment::determinePairs(double** scene, bool* mask, int size)
{
  ANNidxArray anIdx;          // near neighbor indices
  ANNdistArray adDists;       // near neighbor distances
  anIdx = new ANNidx[1];
  adDists = new ANNdist[1];

  double dErr = 0.0;

  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {

      _tree->annkSearch(
            scene[i],         // query point
            1,                // number of near neighbors to find
            anIdx,            // nearest neighbor array (modified)
            adDists,          // dist to near neighbors (modified)
            dErr);            // error bound
      addPair(anIdx[0], i, adDists[0]);
    }
    else
    {
      addNonPair(i);
    }
  }
  delete[] anIdx;
  delete[] adDists;
}

}
