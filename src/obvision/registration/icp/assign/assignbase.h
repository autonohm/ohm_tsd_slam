#ifndef ASSIGNBASE_H
#define ASSIGNBASE_H

#include <stdlib.h>
#include <string.h>
#include <iostream>

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @struct TdCartesianPoint
 * @brief Represents a single point in Cartesian space
 * @author Stefan May and Dirk Holz
 **/
typedef double* TdCartesianPoint;

/**
 * @struct StrCartesianPair
 * @brief Represents a point pair in Cartesian space
 * @author Stefan May and Dirk Holz
 **/
struct StrCartesianPair
{
  /** first point's coordinates */
  TdCartesianPoint first;
  /** second point's coordinates */
  TdCartesianPoint second;
};

/**
 * @struct StrCartesianIndexPair
 * @brief Representation of one pair of point indices
 * @author Stefan May
 */
struct StrCartesianIndexPair
{
  /** index of first point */
  unsigned int indexFirst;
  /** index of second point */
  unsigned int indexSecond;
};

/**
 * @struct StrCartesianIndexDistancePair
 * @brief Represents a point pair and the euclidian distance in between
 * @author Markus Kühn
 */
struct StrCartestianIndexDistancePair
{
  /** index of first point*/
  unsigned int indexFirst;
  /** index of second point*/
  unsigned int indexSecond;
  /** absolute distance */
  double dist;

  /** Compare function according to distance, First is smaller second*/
  static bool pairCompareDist(const StrCartestianIndexDistancePair& firstElem, const StrCartestianIndexDistancePair& secondElem) {
    return firstElem.dist < secondElem.dist;

  }
};

}

#endif /* ASSIGNBASE_H */
