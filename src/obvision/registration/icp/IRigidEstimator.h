#ifndef IRIGIDESTIMATOR_H_
#define IRIGIDESTIMATOR_H_

#include <math.h>

#include "obvision/registration/icp/assign/assignbase.h"
#include "obcore/math/linalg/linalg.h"
#include <vector>

namespace obvious
{

/**
 * @class IRigidEstimator
 * @brief An estimator for registering points clouds into one common coordinate system.
 * @author Stefan May
 */
class IRigidEstimator
{
	public:
		/**
		 * Default constructor
		 */
		IRigidEstimator(){};

		/**
		 * Destructor
		 */
		virtual ~IRigidEstimator(){};
	
		/**
		 * Setting internal pointer to model array. The model is seen as the ground truth against which the scene has to be registered.
		 * @param model Pointer to 3 dimensional model array
		 */
		virtual void setModel(double** model, unsigned int size, double** normals=NULL) = 0;
	
		/**
		 * Setting internal pointer to scene array. (See commend for setModel)
		 * @param scene Pointer to 3 dimensional scene array
		 */	
		virtual void setScene(double** scene, unsigned int size, double** normals=NULL) = 0;
		
		/**
		 * Setting assigned point pairs. You can use a pair assigner for this purpose. The deviation, i.e. the mean distance, between those pairs is also determined within this method.
		 * @param vPairs Vector of pairs of indices. Each index pair references a scene and a model point.
		 */
		virtual void setPairs(std::vector<StrCartesianIndexPair>* vPairs) = 0;
		
		/**
		 * Access the RMS error that has been calculated by the setPairs method.
		 * @return RMS error
		 */	
		virtual double getRMS() = 0;

		/**
		 * Determine the transformation matrix that registers the scene to the model.
		 * @param transformation matrix as return parameter 
		 */
		virtual void estimateTransformation(Matrix* T) = 0;
};

}

#endif /*IRIGIDESTIMATOR_H_*/
