#ifndef PAIRASSIGNMENT_H
#define PAIRASSIGNMENT_H

#include <vector>
#include <map>
#include "obvision/registration/icp/assign/assignbase.h"
#include "obvision/registration/icp/assign/filter/IPreAssignmentFilter.h"
#include "obvision/registration/icp/assign/filter/IPostAssignmentFilter.h"
#include <iostream>

using namespace std;

#define DEFAULTDIMENSION 2

namespace obvious
{

/**
 * @class PairAssignment
 * @brief Encapsulates neighbor searching
 * @author Stefan May
 **/
class PairAssignment
{
public:
  /**
   * Default constructor
   */
  PairAssignment();

  /**
   * Standard constructor
   **/
  PairAssignment(int dimension);

  /**
   * Standard destructor
   **/
  virtual ~PairAssignment();

  void addPreFilter(IPreAssignmentFilter* filter);
  void addPostFilter(IPostAssignmentFilter* filter);

  /**
   * Access internal container of assigned pair indices
   * @return pointer to pair index vector
   */
  virtual vector<StrCartesianIndexPair>* getPairs();

  /**
   * Access squared distances between index pairs
   * @return point to pair distance vector
   */
  virtual vector<double>* getDistancesSqr();

  /**
   * Access internal container of non-assigned point indices
   * @return pointer to non-pair index vector
   */
  virtual vector<unsigned int>* getNonPairs();

  /**
   * Dimension of space
   * @return dimension of space
   */
  virtual int getDimension();

  /**
   * Set model as matching base
   * @param model array of xy values
   * @param size number of points
   **/
  virtual void setModel(double** model, int size) = 0;

  /**
   * Determine point pairs (generic implementation)
   * @param scene scene to be compared
   * @param size nr of points in scene
   */
  void determinePairs(double** scene, int size);

  /**
   * Determine point pairs (concrete implementation)
   * @param scene scene to be compared
   * @param msk validity mask
   * @param size nr of points in scene
   */
  virtual void determinePairs(double** scene, bool* mask, int size) = 0;

  /**
   * clear vector of Cartesian point pairs and reset post assignment filters
   */
  void reset();

protected:
  /**
   * add assigned point to internal vector
   * @param indexModel index of model point
   * @param indexScene index of scene point
   * @param distanceSqr squared distance between model and scene point
   */
  virtual void addPair(unsigned int indexModel, unsigned int indexScene, double distanceSqr);

  /**
   * add non-assigned point to internal vector
   * @param indexScene index of scene point
   */
  virtual void addNonPair(unsigned int indexScene);

  /**
   * Dimension of space
   */
  int _dimension;

  /**
   * the 2D or 3D model
   */
  double** _model;

  /**
   * model size / number of points
   */
  int _sizeModel;


  vector<IPreAssignmentFilter*> _vPrefilter;
  vector<IPostAssignmentFilter*> _vPostfilter;

  void clearPairs();

private:
  /**
   * Vector of Cartesian pairs
   */
  vector<StrCartesianIndexPair> _initPairs;
  vector<StrCartesianIndexPair> _filteredPairs;
  vector<StrCartesianIndexPair>* _pairs;

  /**
   * Vector of squared distances of pairs
   */
  vector<double> _initDistancesSqr;
  vector<double> _filteredDistancesSqr;
  vector<double>* _distancesSqr;

  /**
   * Vector of Cartesian points (scene points, that could not be assigned to model points)
   */
  vector<unsigned int> _nonPairs;

};

}

#endif
