#ifndef FLANNPAIRASSIGNMENT_H
#define FLANNPAIRASSIGNMENT_H

#include <flann/flann.hpp>
#include "obcore/math/mathbase.h"
#include "obvision/registration/icp/assign/PairAssignment.h"


using std::vector;

namespace obvious
{

/**
 * @class FlannPairAssignment
 * @brief Encapsulates neighbor searching with approximate nearest neighbor algorithm (FLANN)
 * @author Stefan May
 **/
class FlannPairAssignment : public PairAssignment
{
public:
	
	/**
	 * Standard constructor
	 * @param dimension dimensionality of dataset
	 * @param eps used for searching eps-approximate neighbors
	 **/
	FlannPairAssignment(int dimension, double eps = 0.0, bool parallelSearch=false);
	
	/**
	 * Destructor
	 **/
	~FlannPairAssignment();

	/**
	 * Set model as matching base
	 * @param model array of xy values
	 * @param size number of points
	 **/
	void setModel(double** model, int size);
	
  /**
   * Determine point pairs
   * @param scene scene to be compared
   * @param msk validity mask
   * @param size nr of points in scene
   */
	void determinePairs(double** scene, bool* msk, int size);
	
private:

	/**
	 * Private initialization routine called by constructors
	 */
	void init(double eps);

	/**
	 * Sequential version of NN-search
	 */
	void determinePairsSequential(double** scene, bool* mask, int size);

	/**
   * OpenMP-accelerated version of NN-search
   */
	void determinePairsParallel(double** scene, bool* mask, int size);

	flann::Matrix<double>* _dataset;
	flann::Index<flann::L2<double> >* _index;

	double _eps;

	bool _useParallelVersion;
};

}

#endif
