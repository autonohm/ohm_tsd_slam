#ifndef PointToLine2DEstimator_H
#define PointToLine2DEstimator_H

#include "obvision/registration/icp/IRigidEstimator.h"

namespace obvious
{

/**
 * @class PointToLine2D
 * @brief An closed-form estimator to enregister 2D points clouds into one common coordinate system.
 * @author Christian Pfitzner
 */
class PointToLine2DEstimator : public IRigidEstimator
{
public:
  /**
   * Default constructor
   */
  PointToLine2DEstimator();
  /**
   * Destructor
   */
  ~PointToLine2DEstimator ();
  /**
   * Setting internal pointer to model array. The model is seen as the ground truth against which the scene has to be registered.
   * @param model Pointer to 3 dimensional model array
   */
  virtual void setModel(double** model, unsigned int size, double** normals);
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

  unsigned int getIterations(void);

private:
  double**                            _model;     //!< model
  double**                            _scene;     //!< scene
  double**                            _normals;   //!< pointer to normals
  std::vector<StrCartesianIndexPair>* _pairs;     //!< index pairs
  double                              _rms;       //!< deviation
  unsigned int                        _iterations;
  double                              _cm[3];
  double                              _cs[3];
};

}

#endif /*PointToLine2DEstimator_H */
