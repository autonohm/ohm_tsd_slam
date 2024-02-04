#ifndef RAYCASTAXISALIGNED2D_H_
#define RAYCASTAXISALIGNED2D_H_

#include "obcore/base/types.h"
#include "obvision/reconstruct/grid/TsdGrid.h"

namespace obvious {

class RayCastAxisAligned2D {
public:
  RayCastAxisAligned2D();
  virtual ~RayCastAxisAligned2D();
  void calcCoords(TsdGrid* grid, obfloat* coords, obfloat* normals, unsigned int* cnt, char* occupiedGrid = NULL);
};

}

#endif /* RAYCASTAXISALIGNED2D_H_ */
