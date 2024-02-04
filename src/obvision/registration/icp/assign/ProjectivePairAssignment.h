#ifndef PROJECTIVEPAIRASSIGNMENT_H
#define PROJECTIVEPAIRASSIGNMENT_H

#include "obcore/math/mathbase.h"
#include "obvision/registration/icp/assign/PairAssignment.h"


using std::vector;

namespace obvious
{

/**
 * @class ProjectivePairAssignment
 * @brief Encapsulates neighbor searching based on projective projection
 * @author Stefan May
 **/
class ProjectivePairAssignment : public PairAssignment
{
public:

	/**
	 * Standard constructor
	 * @param P projection matrix
	 * @param width width of underlying image
	 * @param height height of underlying image
	 * @param dimension dimensionality of dataset
	 **/
	ProjectivePairAssignment(double* P, unsigned int width, unsigned int height, int dimension=3) : PairAssignment(dimension) {init(P, width, height, dimension);};
	
	/**
	 * Standard destructor
	 **/
	~ProjectivePairAssignment();

	/**
	 * Set model as matching base
	 * @param model array of xy values
	 * @param size number of points
	 **/
	void setModel(double** model, int size);
	
	/**
	 * Determine point pairs (nearest neighbors)
	 * @param scene scene to be compared
	 * @param size nr of points in scene
	 * @param pairs return value of neighbors
	 * @param nonPairs return value of points with no neighbors
	 */
	void determinePairs(double** scene, bool* msk, int size);
	
private:

	 /**
	   * Private initialization routine called by constructors
	   */
	  void init(double* P, unsigned int width, unsigned int height, unsigned int dim);

	  double* _P;
	  unsigned int _w;
	  unsigned int _h;
	  unsigned int* _idx_m;
	  double** _model;
};

}

#endif
