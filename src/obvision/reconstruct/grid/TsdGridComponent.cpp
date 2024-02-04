#include "TsdGridComponent.h"
#include "obcore/math/mathbase.h"
#include <cmath>

namespace obvious
{

TsdGridComponent::TsdGridComponent(bool isLeaf)
{
  _isLeaf = isLeaf;
}

TsdGridComponent::~TsdGridComponent()
{

}

obfloat TsdGridComponent::getComponentSize()
{
  return _componentSize;
}

obfloat* TsdGridComponent::getCentroid()
{
  return _centroid;
}

obfloat TsdGridComponent::getCircumradius()
{
  return _circumradius;
}

Matrix* TsdGridComponent::getEdgeCoordsHom()
{
  return _edgeCoordsHom;
}

bool TsdGridComponent::isLeaf()
{
  return _isLeaf;
}

bool TsdGridComponent::isInRange(obfloat pos[2], Sensor* sensor, obfloat maxTruncation)
{
  // Centroid-to-sensor distance
  obfloat distance = euklideanDistance<obfloat>(pos, _centroid, 2);

  // closest possible distance of any voxel in partition
  double closestVoxelDist = distance - _circumradius - maxTruncation;

  // check if partition is out of range
  if(closestVoxelDist > sensor->getMaximumRange()) return false;

  // farthest possible distance of any voxel in partition
  double farthestVoxelDist = distance + _circumradius + maxTruncation;

  // check if partition is too close
  if(farthestVoxelDist < sensor->getMinimumRange()) return false;

  if(_isLeaf)
  {
    double* data = sensor->getRealMeasurementData();
    bool* mask = sensor->getRealMeasurementMask();
    int measurements = sensor->getRealMeasurementSize();

    int idxEdge[4];
    sensor->backProject(_edgeCoordsHom, idxEdge);

    // Index of -2 is lower bound
    // Index of -1 is upper bound
    bool isAnyEdgeVisible = false;
    bool areAllEdgesVisible = true;
    for(int i=0; i<4; i++)
    {
      if(idxEdge[i]==-1)
      {
        idxEdge[i] = measurements-1;
        areAllEdgesVisible = false;
      }
      else if(idxEdge[i]==-2)
      {
        idxEdge[i] = 0;
        areAllEdgesVisible = false;
      }
      else isAnyEdgeVisible = true;
    }

    // Check whether non of the cells are in the field of view
    if(!isAnyEdgeVisible) return false;

    int minIdx;
    int maxIdx;
    minmaxArray<int>(idxEdge, 4, &minIdx, &maxIdx);

    // Check if any cell comes closer than the truncation radius
    bool isVisible = false;
    for(int j=minIdx; j<=maxIdx; j++)
    {
        isVisible = isVisible || ((data[j] > closestVoxelDist) && mask[j]);
    }

    if(!isVisible) return false;

    if(areAllEdgesVisible)
    {
      bool isEmpty = true;
      for(int j=minIdx; j<=maxIdx; j++)
      {
        //isEmpty = isEmpty && (data[j] > farthestVoxelDist) && mask[j];
        if(isinf(data[j]))
          isEmpty = isEmpty && (distance < sensor->getLowReflectivityRange());
        else
          isEmpty = isEmpty && (data[j] > farthestVoxelDist) && mask[j];
      }

      if(isEmpty)
      {
        increaseEmptiness();
        return false;
      }
    }
  }
  return true;
}

} // namespace
