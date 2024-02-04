#ifndef ICPMULTIINITITERATOR_H_
#define ICPMULTIINITITERATOR_H_

#include <iostream>
using namespace std;

#include "obvision/registration/icp/Icp.h"

using namespace obvious;

namespace obvious
{

/**
 * @class IcpMultiInitIterator
 * @brief Iterates through vector of initialization matrices and tests for the best matching result.
 *        This class is to be used for registration under fast motion
 * @author Stefan May
 **/
class IcpMultiInitIterator
{
public:
	/**
	 * Standard constructor
	 * @param assigner pair assignment strategy for point clouds
	 * @param estimator transformation estimator
	 * @param pDistanceThreshold thresholding strategy
	 */
  IcpMultiInitIterator(vector<Matrix> Tinit);
		 
	/**
	 * Destructor
	 */
	~IcpMultiInitIterator();
	
	/**
   * Start iteration for all initialization matrices. Take best result.
   * @param icp Instance of iterative closest point class
   */
  Matrix iterate(Icp* icp);

private:
	
  vector<Matrix> _Tinit;

  Matrix* _Tlast;
};

}

#endif /*ICPMULTIINITITERATOR_H_*/
