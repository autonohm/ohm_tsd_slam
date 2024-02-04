#ifndef CLOSEDFORMESTIMATOR2D_H_
#define CLOSEDFORMESTIMATOR2D_H_

#include "obvision/registration/icp/IRigidEstimator.h"

namespace obvious
{

/**
 * @class ClosedFormEstimator2D
 * @brief An closed-form estimator to enregister 2D points clouds into one common coordinate system.
 * @author Stefan May
 */
class ClosedFormEstimator2D : public IRigidEstimator
{
	public:
		/**
		 * Default constructor
		 */
		ClosedFormEstimator2D();
		
		/**
		 * Destructor
		 */
		~ClosedFormEstimator2D();

		/**
		 * Setting internal pointer to model array. The model is seen as the ground truth against which the scene has to be registered.
		 * @param model Pointer to 3 dimensional model array
		 */
		virtual void setModel(double** model, unsigned int size, double** normals=NULL);
		
		/**
		 * Setting internal pointer to scene array. (See commend for setModel)
		 * @param scene Pointer to 3 dimensional scene array
		 */
		virtual void setScene(double** scene, unsigned int size, double** normals=NULL);
		
		/**
		 * Setting assigned point pairs. You can use a pair assigner for this purpose. The deviation, i.e. the mean distance, between those pairs is also determined within this method.
		 * @param pairs Vector of pairs of indices. Each index pair references a scene and a model point.
		 */
		virtual void setPairs(std::vector<StrCartesianIndexPair>* pairs);
		
		/**
		 * Access the RMS error that has been calculated by the setPairs method.
		 * @return RMS error
		 */
		virtual double getRMS();

		/**
		 * Determine the transformation matrix that registers the scene to the model.
		 * @param T transformation matrix as return parameter
		 */
		virtual void estimateTransformation(Matrix* T);
		
	private:
	
		/**
		 *  Centroid of model
		 */
		double _cm[2];
		
		/**
		 * Model
		 */
		double** _model;
		
		/**
		 * Centroid of scene
		 */
		double _cs[2];
		
		/**
		 * Scene
		 */
		double** _scene;
		
		/**
		 *  Index pairs
		 */
		std::vector<StrCartesianIndexPair>* _pairs;
		
		/**
		 *  Deviation
		 */
		double _rms;

		unsigned int _iterations;

};

}

#endif /*CLOSEDFORMESTIMATOR2D_H_*/
