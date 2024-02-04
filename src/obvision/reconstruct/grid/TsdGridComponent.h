#ifndef TSDGRIDCOMPONENT_H
#define TSDGRIDCOMPONENT_H

#include "obcore/base/types.h"
#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/reconstruct_defs.h"
#include "obvision/reconstruct/Sensor.h"

namespace obvious
{

/**
 * @class TsdGridComponent
 * @brief Abstract component for quadtree implementation
 * @author Stefan May
 */
class TsdGridComponent
{
public:
  /**
   * Constructor
   * @brief Abstraction layer for leafs and branches
   * @param isLeaf discriminate leaf from branch
   */
  TsdGridComponent(bool isLeaf);

  /**
   * Destructor
   */
  virtual ~TsdGridComponent();

  /**
   * Edge length of component
   * @return edge length
   */
  obfloat getComponentSize();

  /**
   * Get coordinates of centroid
   * @return coordinate array
   */
  obfloat* getCentroid();

  /**
   * Get circumradius, i.e., diagonal length of component
   * @return circumradius
   */
  obfloat getCircumradius();

  /**
   * Get homogeneous coordinates of edges
   * @return coordinate array
   */
  Matrix* getEdgeCoordsHom();

  /**
   * Discriminate leaf from branch
   * @return discrimination flag
   */
  bool isLeaf();

  /**
   * Check whether component is in range from a certain sensor position
   * @param pos sensor position
   * @param sensor sensor
   * @param maxTruncation maximum truncation radius
   * @return visibility flag
   */
  bool isInRange(obfloat pos[2], Sensor* sensor, obfloat maxTruncation);

  /**
   * Increase emptiness of component (for visible cells, in which every measurement ray passes through without hitting a surface)
   */
  virtual void increaseEmptiness() = 0;

protected:

  obfloat _componentSize;

  bool _isLeaf;

  obfloat _centroid[2];

  Matrix* _edgeCoordsHom;

  obfloat _circumradius;

};

}

#endif
