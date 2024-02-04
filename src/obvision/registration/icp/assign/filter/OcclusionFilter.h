#ifndef OCCLUSIONFILTER_H
#define OCCLUSIONFILTER_H

#include "obcore/base/CartesianCloud.h"
#include "obvision/registration/icp/assign/filter/IPreAssignmentFilter.h"
#include <vector>

using namespace obvious;

namespace obvious
{

/**
 * @class OcclusionFilter
 * @brief
 * @author Stefan May
 */
class OcclusionFilter : public IPreAssignmentFilter
{
public:
  /**
   * Default constructor
   */
	OcclusionFilter(double* P, unsigned int width, unsigned int height);

  /**
   * Destructor
   */
  ~OcclusionFilter();

  void update(double* P);

  virtual void filter(double** scene, unsigned int size, bool* mask);

private:
  double _P[12];
  int** _map;
  double** _zbuffer;
  unsigned int _w;
  unsigned int _h;
};

}

#endif /*OCCLUSIONFILTER_H*/
