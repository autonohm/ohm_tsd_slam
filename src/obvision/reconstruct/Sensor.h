#ifndef SENSOR_H_
#define SENSOR_H_

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/reconstruct_defs.h"
#include <vector>
#include <cmath>

namespace obvious
{

/**
 * @class Sensor
 * @brief Abstract class for 2D and 3D measurement units
 * @author Stefan May
 */
class Sensor
{
public:

  /**
   * Standard constructor
   * @param[in] dim dimensionality of representation
   * @param[in] maxRange maximum range
   * @param[in] minRange minimum range, i.e. a dead zone
   * @param[in] lowReflectivityRange range for objects with low remission
   */
  Sensor(unsigned int dim, double maxRange, double minRange, double lowReflectivityRange);

  /**
   * Destructor
   */
  virtual ~Sensor();

  /**
   * Get size of first dimension
   * @return size
   */
  virtual unsigned int getWidth();

  /**
   * Get size of second dimension
   * @return size
   */
  virtual unsigned int getHeight();

  /**
   * Get maximum range
   * @return maximum range
   */
  virtual double getMaximumRange();

  /**
   * Get minimum range
   * @return minimum range
   */
  virtual double getMinimumRange();

  /**
   * Get range of sensor when objects of low reflectivity are present
   * @return maximum range
   */
  virtual double getLowReflectivityRange();

  /**
   * Access matrix of measurement rays with parameterizable normalization
   * @param norm normalization value
   * @return Ray matrix, i.e. R(dimensionality, measurement size)
   */
  virtual Matrix* getNormalizedRayMap(double norm);

  /**
   * Transform current sensor pose in his own coordinate system
   * P' = P T = [Rp | tp] [R | t] = [Rp R | Rp t + tp], where P is the old and P' is the new pose
   * @param[in] T transformation matrix
   */
  virtual void transform(Matrix* T);

  /**
   * Translate current sensor pose
   * @param tr[in] tr translation vector
   */
  virtual void translate(double* tr);

  /**
   * Accessor to sensor transformation
   * @return (dim+1)x(dim+1) transformation matrix (R | t; 0 1)
   */
  Matrix getTransformation();

  /**
   * Set transformation matrix
   * @param T transformation matrix
   */
  void setTransformation(Matrix T);

  /**
   * Reset sensor pose to identity
   */
  void resetTransformation();

  /**
   * Accessor to sensor translation
   * @param[out] tr translation vector
   */
  virtual void getPosition(obfloat* tr);

  /**
   * Get size of measurement vector
   * @return number of beams
   */
  virtual unsigned int getRealMeasurementSize();

  /**
   * Copy measurement data to internal buffer
   * @param data source with 2D coordinates
   * @param scale scale factor to multiply distances
   */
  virtual void setRealMeasurementData(double* data, double scale = 1.0);

  /**
   * Copy measurement data to internal buffer
   * @param data source with 2D coordinates
   * @param scale scale factor to multiply distances
   */
  virtual void setRealMeasurementData(vector<float> data, float scale = 1.f);

  /**
   * Get measurement vector
   * @return vector of distance data
   */
  virtual double* getRealMeasurementData();

  /**
   * Convert distance data in measurement array to Cartesian coordinates in sensor coordinate system
   * @param coords Output array of size dim*size. Output is grouped in n-tuples [x1 y1 ....]
   */
  unsigned int dataToCartesianVector(double* &coords);

  /**
   * Convert distance data in measurement array to Cartesian coordinates in sensor coordinate system
   * @param coords Output array of size dim*size. Output is grouped in n-tuples [x1 y1 ....]
   * @param mask Validity mask
   * @return number of valid points, i.e., having mask[i]==true
   */
  unsigned int dataToCartesianVectorMask(double* &coords, bool* &validityMask);

  /**
    * Removes points from a double vector [x1 y1 ....] according to a given mask.
    * @param inPoints A 2d double array that contains the point to be filtered
    * @param mask  The mask that describes which points are valid (true) or invalid (false)
    * @param sizeMask Size of the mask array. The inPoints array must contain the same amount of points (not double fields!)
    * @param outPoints An initialized 2d double array with the size of validPoints*2. This size is an output of dataToCartesianVectorMask.
    * @return Number of valid Points again. Test if this is equal to the output of dataToCartesianVectorMask.
    */
  static unsigned int removeInvalidPoints(double* inPoints, bool* mask, unsigned int sizeMask, double* outPoints);

  /**
   * Convert distance measurements to Cartesian coordinates represented as matrix
   * @return Cartesian coordinate matrix
   */
  Matrix dataToHomogeneousCoordMatrix();

  /**
   * Set measurement accuracy
   * @param accuracy array
   */
  virtual void setRealMeasurementAccuracy(double* accuracy);

  /**
   * Access measurement accuracy (if available, see hasRealMeasurmentAccuracy)
   * @return accuracy array
   */
  virtual double* getRealMeasurementAccuracy();

  /**
   * Instance contains accuracy values
   * @return accuracy flag
   */
  virtual bool hasRealMeasurementAccuracy();

  /**
   * Copy measurement mask
   * @param mask source mask
   */
  virtual void setRealMeasurementMask(bool* mask);

  /**
   * Copy measurement mask
   * @param mask source mask
   */
  virtual void setRealMeasurementMask(vector<unsigned char> mask);

  /**
   * Set mask for every pixel to true
   */
  virtual void resetMask();

  /**
   * Mask measurements having depth==0.0
   */
  virtual void maskZeroDepth();

  /**
   * Mask measurements having depth==NAN || INFINITY
   */
  virtual void maskInvalidDepth();

  /**
   * Get validity mask
   * @return validity mask vector. True signals a valid measurement
   */
  virtual bool* getRealMeasurementMask();

  /**
   * @return flag indicate availability of RGB data
   */
  virtual bool hasRealMeasurementRGB();

  /**
   * Copy rgb data to internal buffer
   * @param rgb color/texture data
   */
  virtual void setRealMeasurementRGB(unsigned char* rgb);

  /**
   * Get color data
   * @return pointer to internal color data (may be null for certain sensors)
   */
  virtual unsigned char* getRealMeasurementRGB();

  /**
   * Set user-defined metadata, e.g., point classifier
   * @param typeID user-defined type identified
   */
  void setRealMeasurementTypeID(int* typeID);

  /**
   * Get user-defined metadata, e.g., point classifier
   * @return user-defined type identified
   */
  int* getRealMeasurementTypeID();

  /**
   * Project coordinate back to sensor index
   * @param[in] M matrix of coordinates (homogeneous)
   * @param[out] indices vector of projection results (must be allocated outside)
   * @param[in] T temporary transformation matrix of coordinates
   */
  virtual void backProject(Matrix* M, int* indices, Matrix* T = NULL) = 0;

protected:

  // Pose
  Matrix* _T;

  // Dimensionality
  unsigned int _dim;

  // The sensor's minimal range
  double _minRange;

  // The sensor's maximal range
  double _maxRange;

  // The sensor's minimal range measuring against objects of low reflectivity
  double _lowReflectivityRange;

  // Number of measurement samples, i.e. _width x _height
  unsigned int _size;

  // Measurement data
  double* _data;

  // Accuracy of measurement samples (if determinable)
  double* _accuracy;

  // Validity of measurement samples
  bool* _mask;

  // RGB data (if available)
  unsigned char* _rgb;

  // Type identifier
  int* _typeID;

  // Size of first dimension, i.e., # of samples of a 2D scan or width of image sensor
  unsigned int _width;

  // Size of second dimension, i.e., 1 for a 2D scan or height of image sensor
  unsigned int _height;

  // Normalizing factor for normal vectors
  double _rayNorm;

  // Ray matrix in world coordinate frame
  Matrix* _rays;

  // Ray matrix in sensor coordinate frame
  Matrix* _raysLocal;

};

}

#endif
