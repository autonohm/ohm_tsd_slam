#ifndef TSDGRID_H
#define TSDGRID_H

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "TsdGridPartition.h"

namespace obvious
{

enum EnumTsdGridLayout { LAYOUT_1x1=0,
  LAYOUT_2x2=1,
  LAYOUT_4x4=2,
  LAYOUT_8x8=3,
  LAYOUT_16x16=4,
  LAYOUT_32x32=5,
  LAYOUT_64x64=6,
  LAYOUT_128x128=7,
  LAYOUT_256x256=8,
  LAYOUT_512x512=9,
  LAYOUT_1024x1024=10,
  LAYOUT_2048x2048=11,
  LAYOUT_4096x4096=12,
  LAYOUT_8192x8192=13,
  LAYOUT_16384x16384=14,
  LAYOUT_36768x36768=15};

enum EnumTsdGridInterpolate { INTERPOLATE_SUCCESS=0,
  INTERPOLATE_INVALIDINDEX=1,
  INTERPOLATE_EMPTYPARTITION=2,
  INTERPOLATE_ISNAN=3};

enum EnumTsdGridPartitionIdentifier{ UNINITIALIZED = 0,
  EMPTY = 1,
  CONTENT = 2};

enum EnumTsdGridLoadSource{ FILE_SOURCE = 0,
  STRING_SOURCE = 1
};

/**
 * @class TsdGrid
 * @brief Grid on the basis of true signed distance functions
 * @author Stefan May, Philipp Koch
 */
class TsdGrid
{
public:

  /**
   * Standard constructor
   * Allocates and initializes space and matrices
   * @param[in] cellSize Size of cell in meters
   * @param[in] layoutPartition Partition layout, i.e., cells in partition
   * @param[in] layoutGrid Grid layout, i.e., partitions in grid
   */
  TsdGrid(const obfloat cellSize, const EnumTsdGridLayout layoutPartition, const EnumTsdGridLayout layoutGrid);

  /**
   * Constructor
   * Loads the grid data out of a given file. File has to be correct it is not being checked.
   * @param[in] path path to the data file
   */
  TsdGrid(const std::string& data, const EnumTsdGridLoadSource source = FILE_SOURCE);

  /**
   * Destructor
   */
  virtual ~TsdGrid();

  /**
   * Reset TSD grid (delete and re-instantiate cells)
   */
  void reset();

  /**
   * Access truncated signed distance at specific cell. This method does not check validity of indices.
   * The specific cell might not be instantiated.
   * @param y y coordinate
   * @param x x coordinate
   * @return truncated signed distance
   */
  obfloat& operator () (unsigned int y, unsigned int x) const
  {
    // Partition index
    unsigned int px = x / _dimPartition;
    // Cell index
    unsigned int cx = x % _dimPartition;
    // Partition index
    unsigned int py = y / _dimPartition;
    // Cell index
    unsigned int cy = y % _dimPartition;

    return (*_partitions[py][px])(cy, cx);
  }

  /**
   * Get number of cells in x-dimension
   * @return number of cells
   */
  unsigned int getCellsX() const { return _cellsX; }

  /**
   * Get number of cells in y-dimension
   * @return number of cells
   */
  unsigned int getCellsY() const { return _cellsY; }

  /**
   * Get size of cell in meters
   * @return size
   */
  obfloat getCellSize() const { return _cellSize; }

  /**
   * Get minimum for x-coordinate
   * @return x-coordinate
   */
  obfloat getMinX() const { return _minX; }

  /**
   * Get maximum for x-coordinate
   * @return x-coordinate
   */
  obfloat getMaxX() const { return _maxX; }

  /**
   * Get centroid of grid
   * @param[out] centroid centroid coordinates
   */
  void getCentroid(double centroid[2]);

  /**
   * Get minimum for y-coordinate
   * @return y-coordinate
   */
  obfloat getMinY() const { return _minY; }

  /**
   * Get maximum for y-coordinate
   * @return y-coordinate
   */
  obfloat getMaxY() const { return _maxY; }

  /**
   * Get number of cells along edge
   * @return number of cells
   */
  unsigned int getPartitionSize() const { return _partitions[0][0]->getWidth(); }

  /**
   * Set maximum truncation radius
   * @param[in] val truncation radius
   */
  void setMaxTruncation(const double val);

  /**
   * Get maximum truncation radius
   * @return truncation radius
   */
  double getMaxTruncation() const { return _maxTruncation; }

  /**
   * Push current measurement from sensor
   * @param[in] virtual 2D measurement unit
   */
  void push(SensorPolar2D* sensor);
  void pushTree(SensorPolar2D* sensor);

  bool containsData();

  /**
   * Create color image from tsdf grid
   * @param[out] color image (3-channel)
   */
  void grid2ColorImage(unsigned char* image, unsigned int width, unsigned int height);


  void getData(std::vector<double>& data);

  /**
   * Calculates normal of plain element hit by a ray caster
   * @param[out] coordinates
   * @param[out] normal vector
   */
  bool interpolateNormal(const obfloat coord[2], obfloat normal[2]);

  /**
   * interpolate bilinear
   * @param coordinates pointer to coordinates of intersection
   * @param[out] tsdf interpolated TSD value
   */
  EnumTsdGridInterpolate interpolateBilinear(obfloat coord[2], obfloat* tsd);

  /**
   * Convert arbitrary coordinate to grid coordinates
   * @param[in] coord 2D query coordinates
   * @param[out] x x-index
   * @param[out] y y-index
   * @param[out] dx x-coordinate of cell-center in metric space
   * @param[out] dy y-coordinate of cell-center in metric space
   */
  bool coord2Cell(obfloat coord[2], int* p, int* x, int* y, obfloat* dx, obfloat* dy);

  /**
   * Get pointer to internal partition space
   * @return pointer to 3D partition space
   */
  TsdGridPartition*** getPartitions() const { return _partitions; }

  /**
   * Determine whether sensor is in grid
   * @param sensor
   */
  bool isInsideGrid(Sensor* sensor);

  /**
   * Method to write the content of the grid into a given file
   * @param path Path where data file is created
   * @return True in case of success
   */
  bool storeGrid(const std::string& path);

  /**
   * Method to set the grid in a certain area as empty
   * @param centerCoords footprint center
   * @param width width of footprint
   * @param height height of footprint
   * @return true in case of success (valid coordinates)
   */
  bool freeFootprint(const obfloat centerCoords[2], const obfloat width, const obfloat height);

private:

  void init(const double cellSize, const EnumTsdGridLayout layoutPartition, const EnumTsdGridLayout layoutGrid);

  /**
   * De-Initialization
   */
  void deinit();

  void pushRecursion(SensorPolar2D* sensor, obfloat pos[2], TsdGridComponent* comp, vector<TsdGridPartition*> &partitionsToCheck);

  void propagateBorders();

  TsdGridComponent* _tree;

  int _cellsX;

  int _cellsY;

  int _sizeOfGrid;

  obfloat _cellSize;

  obfloat _invCellSize;

  obfloat _maxTruncation;

  obfloat _minX;

  obfloat _maxX;

  obfloat _minY;

  obfloat _maxY;

  TsdGridPartition*** _partitions;

  int _dimPartition;

  int _partitionsInX;

  int _partitionsInY;

  EnumTsdGridLayout _layoutPartitions;

  EnumTsdGridLayout _layoutGrid;

  bool _initialPushAccomplished;

};

inline EnumTsdGridInterpolate TsdGrid::interpolateBilinear(obfloat coord[2], obfloat* tsd)
{
  int p;
  int x;
  int y;
  obfloat dx;
  obfloat dy;

  if(!coord2Cell(coord, &p, &x, &y, &dx, &dy)) return INTERPOLATE_INVALIDINDEX;
  if(!_partitions[0][p]->isInitialized()) return INTERPOLATE_EMPTYPARTITION;

  const double wx = fabs((coord[0] - dx) * _invCellSize);
  const double wy = fabs((coord[1] - dy) * _invCellSize);

  *tsd = _partitions[0][p]->interpolateBilinear(x, y, wx, wy);

  if(isnan(*tsd))
    return INTERPOLATE_ISNAN;

  return INTERPOLATE_SUCCESS;
}

inline bool TsdGrid::coord2Cell(obfloat coord[2], int* p, int* x, int* y, obfloat* dx, obfloat* dy)
{
  // Get cell indices
  const obfloat dCoordX = coord[0] * _invCellSize;
  const obfloat dCoordY = coord[1] * _invCellSize;

  int xIdx = floor(dCoordX);
  int yIdx = floor(dCoordY);

  // Get center point of current cell
  *dx = (obfloat(xIdx) + 0.5) * _cellSize;
  *dy = (obfloat(yIdx) + 0.5) * _cellSize;

  // Ensure that query point has 4 neighbors for bilinear interpolation
  if (coord[0] < *dx)
  {
    xIdx--;
    (*dx) -= _cellSize;
  }
  if (coord[1] < *dy)
  {
    yIdx--;
    (*dy) -= _cellSize;
  }

  // Check boundaries
  if ((xIdx >= _cellsX) || (xIdx < 0) || (yIdx >= _cellsY) || (yIdx < 0))
    return false;

  *p = yIdx / _dimPartition * _partitionsInX + xIdx / _dimPartition;
  *x = xIdx % _dimPartition;
  *y = yIdx % _dimPartition;

  return true;
}

inline bool TsdGrid::isInsideGrid(Sensor* sensor)
{
  obfloat coord[2];
  sensor->getPosition(coord);
  return (coord[0]>_minX && coord[0]<_maxX && coord[1]>_minY && coord[1]<_maxY);
}

}

#endif
