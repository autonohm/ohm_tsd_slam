#include "FlannPairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

FlannPairAssignment::FlannPairAssignment(int dimension, double eps, bool parallelSearch) : PairAssignment(dimension)
{
  _useParallelVersion = parallelSearch;
  init(eps);
};

FlannPairAssignment::~FlannPairAssignment()
{
  if(_dataset)
  {
    delete _dataset;
    _dataset = NULL;
    delete _index;
    _index = NULL;
  }
}

void FlannPairAssignment::init(double eps)
{
  _dataset = NULL;
  _eps     = eps;
}

void FlannPairAssignment::setModel(double** model, int size)
{
  if(_dataset)
  {
    delete _dataset;
    _dataset = NULL;
    delete _index;
    _index = NULL;
    _model = NULL;
  }

  if(size<1)
  {
    cout << "Error: Model of size 0 passed" << endl;
    return;
  }

  _model = model;

  _dataset = new flann::Matrix<double>(&model[0][0], size, _dimension);
  flann::KDTreeSingleIndexParams p;
  _index = new flann::Index<flann::L2<double> >(*_dataset, p);
  _index->buildIndex();
}

void FlannPairAssignment::determinePairs(double** scene, bool* mask, int size)
{
  if(_useParallelVersion)
    determinePairsParallel(scene, mask, size);
  else
    determinePairsSequential(scene, mask, size);
}

void FlannPairAssignment::determinePairsSequential(double** scene, bool* mask, int size)
{
  if(size<1)
  {
    cout << "Error: Scene of size 0 passed" << endl;
    return;
  }

  flann::Matrix<int> indices(new int[1], 1, 1);
  flann::Matrix<double> dists(new double[1], 1, 1);
  flann::SearchParams p(-1, _eps);

  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      flann::Matrix<double> query(&scene[i][0], 1, _dimension);
      _index->knnSearch(query, indices, dists, 1, p);
      addPair(indices[0][0], i, dists[0][0]);
    }
    else
    {
      addNonPair(i);
    }
  }

  delete[] indices.ptr();
  delete[] dists.ptr();
}

void FlannPairAssignment::determinePairsParallel(double** scene, bool* mask, int size)
{
#pragma omp parallel
{
  flann::Matrix<int> indices(new int[1], 1, 1);
  flann::Matrix<double> dists(new double[1], 1, 1);
  flann::SearchParams p(-1, _eps);
  vector<double> vDist;
  vector<int> vIndicesM;
  vector<int> vIndicesS;
  vector<int> vNonPair;
#pragma omp for schedule(dynamic)
  for(int i = 0; i < size; i++)
  {
    if(mask[i]==1)
    {
      flann::Matrix<double> query(&scene[i][0], 1, _dimension);
      int count = _index->knnSearch(query, indices, dists, 1, p);
      if(count > 0)
      {
        vIndicesM.push_back(indices[0][0]);
        vIndicesS.push_back(i);
        vDist.push_back(dists[0][0]);
      }
      else
      {
        cout << "Error: no data found" << endl;
      }
    }
    else
    {
      vNonPair.push_back(i);
    }
  }

#pragma omp critical
{
  for(unsigned int i=0; i<vDist.size(); i++)
    addPair(vIndicesM[i], vIndicesS[i], vDist[i]);
}

#pragma omp critical
{
  for(unsigned int i=0; i<vNonPair.size(); i++)
    addNonPair(vNonPair[i]);
}

  delete[] indices.ptr();
  delete[] dists.ptr();
}
}

}
