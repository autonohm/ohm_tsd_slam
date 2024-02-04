#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"
#include "TsdGrid.h"
#include "TsdGridBranch.h"
#include "obcore/base/tools.h"

#include <cstring>
#include <cmath>
#include <omp.h>
#include <istream>
#include <assert.h>

namespace obvious
{

#define MAXWEIGHT 32.0

TsdGrid::TsdGrid(const obfloat cellSize, const EnumTsdGridLayout layoutPartition, const EnumTsdGridLayout layoutGrid)
{
  this->init(cellSize, layoutPartition, layoutGrid);
}

TsdGrid::TsdGrid(const std::string& data, const EnumTsdGridLoadSource source)
{
  /* Data file:
   * Cell dimensions
   * Grid dimensions
   * Grid data partition[row][col].cells[row][col]
   */
  std::istream* istream=NULL;
  std::ifstream inFile;
  std::stringstream ss;

  if(source == FILE_SOURCE)
  {
    inFile.open(data.c_str(), std::fstream::in);
    if(!inFile.is_open())
    {
      LOGMSG(DBG_ERROR, " error opening file " << data << "\n");
      std::exit(2);
    }
    istream = &inFile;
  }
  else if(source == STRING_SOURCE)
  {
    ss << data;
    LOGMSG(DBG_DEBUG, "loaded " << ss.str().size() << " characters into stringstream\n");
    istream = &ss;
  }

  const double cellSize = getDoubleLine(*istream);
  const EnumTsdGridLayout layoutPartition = static_cast<EnumTsdGridLayout>(getIntLine(*istream));
  const EnumTsdGridLayout layoutGrid = static_cast<EnumTsdGridLayout>(getIntLine(*istream));
  if( (layoutGrid < 0) || (layoutPartition < 0) || (layoutGrid > 15) || (layoutPartition > 15) )
  {
    LOGMSG(DBG_ERROR, " error! Partition or Gridlayout invalid!\n");
    if(source == FILE_SOURCE)
      inFile.close();
    std::exit(3);
  }

  const double maxTruncation = getDoubleLine(*istream);
  //inFile.getline(buffer, 1000);
  //maxTruncation=std::atof(buffer);
  this->init(cellSize, layoutPartition, layoutGrid);
  this->setMaxTruncation(maxTruncation);
  for(int y = 0; y < _partitionsInY; y++)
  {
    for(int x = 0; x < _partitionsInX; x++)
    {
      EnumTsdGridPartitionIdentifier id=static_cast<EnumTsdGridPartitionIdentifier>(getIntLine(*istream));
      TsdGridPartition* curPart=_partitions[y][x];
      if(id == UNINITIALIZED)
      {
        continue;
      }
      else if(id == EMPTY)
      {
        curPart->_initWeight=static_cast<obfloat>(getDoubleLine(*istream));
        curPart->_initWeight = std::min(curPart->_initWeight, TSDGRIDMAXWEIGHT);
      }
      else if(id == CONTENT)
      {
        curPart->init(_maxTruncation);
        for(unsigned int py=0; py < curPart->getHeight(); py++)
        {
          for(unsigned int px = 0; px < curPart->getWidth(); px++)
          {
              curPart->_grid[py][px].tsd=static_cast<obfloat>(getDoubleLine(*istream));
              curPart->_grid[py][px].weight=static_cast<obfloat>(getDoubleLine(*istream));
          }
        }
      }
      else
      {
        LOGMSG(DBG_ERROR, "Unknown partition identifier for partition(" << x << "/" << y << ")");
        inFile.close();
        std::exit(3);
      }
    }
  }
  if(source == FILE_SOURCE)
  {
  inFile.close();
  if(inFile.is_open())
    LOGMSG(DBG_DEBUG, "Closing of file " << data << " failed.");
  }
}

void TsdGrid::init(const double cellSize, const EnumTsdGridLayout layoutPartition, const EnumTsdGridLayout layoutGrid)
{
  cout << "init" << endl;
  _initialPushAccomplished = false;
  _cellSize = cellSize;
  _invCellSize = 1.0 / _cellSize;

  _cellsX = 1u << layoutGrid;
  _cellsY = _cellsX; //toDo: What if the grid is not quadratic?

  _dimPartition = 1u << layoutPartition;

  if(_dimPartition > _cellsX)
  {
    LOGMSG(DBG_ERROR, "Insufficient partition size : " << _dimPartition << "x" << _dimPartition << " in "
        << _cellsX << "x" << _cellsY << " grid");
    return;
  }

  _partitionsInX = _cellsX/_dimPartition;
  _partitionsInY = _cellsY/_dimPartition;

  _sizeOfGrid = _cellsY * _cellsX;

  _maxTruncation = 2.0*cellSize;

  LOGMSG(DBG_DEBUG, "Grid dimensions: " << _cellsX << "x" << _cellsY << " cells"
      << " = " << ((double)_cellsX)*cellSize << "x" << ((double)_cellsY)*cellSize << " sqm");

  _minX = 0.0;
  _maxX = ((obfloat)_cellsX + 0.5) * _cellSize; // TODO: Check
  _minY = 0.0;
  _maxY = ((obfloat)_cellsY + 0.5) * _cellSize;

  LOGMSG(DBG_DEBUG, "Allocating " << _partitionsInX << "x" << _partitionsInY << " partitions");
  System<TsdGridPartition*>::allocate(_partitionsInY, _partitionsInX, _partitions);

  for(int py=0; py<_partitionsInY; py++)
  {
    for(int px=0; px<_partitionsInX; px++)
    {
      _partitions[py][px] = new TsdGridPartition(px*_dimPartition, py*_dimPartition, _dimPartition, _dimPartition, cellSize);
    }
  }

  int depthTree = layoutGrid-layoutPartition;
  if(depthTree == 0)
  {
    _tree = _partitions[0][0];
  }
  else
  {
    TsdGridBranch* tree = new TsdGridBranch((TsdGridComponent***)_partitions, 0, 0, depthTree);
    _tree = tree;
  }
  _layoutPartitions = layoutPartition;
  _layoutGrid = layoutGrid;
}

TsdGrid::~TsdGrid(void)
{
  deinit();
}

void TsdGrid::deinit()
{
  for(int py=0; py<_partitionsInY; py++)
  {
    for(int px=0; px<_partitionsInX; px++)
    {
      delete _partitions[py][px];
    }
  }
  System<TsdGridPartition*>::deallocate(_partitions);
  int depthTree = _layoutGrid-_layoutPartitions;
  if(depthTree != 0)
  {
    delete _tree;
    _tree = NULL;
  }
}

void TsdGrid::reset()
{
  deinit();
  init(_cellSize, _layoutPartitions, _layoutGrid);
}

void TsdGrid::getCentroid(double centroid[2])
{
  centroid[0] = (_minX + _maxX) * 0.5;
  centroid[1] = (_minY + _maxY) * 0.5;
}

void TsdGrid::setMaxTruncation(double val)
{
  if(val < 2 * _cellSize)
  {
    LOGMSG(DBG_WARN, "Truncation radius must be at least 2 x cell size. Setting minimum size.");
    val = 2 * _cellSize;
  }

  _maxTruncation = val;
}

void TsdGrid::push(SensorPolar2D* sensor)
{
  Timer t;
  t.start();
  const double* data     = sensor->getRealMeasurementData();
  const bool* mask       = sensor->getRealMeasurementMask();
  obfloat tr[2];
  sensor->getPosition(tr);

  unsigned int partSize = (_partitions[0][0])->getSize();

#pragma omp parallel
  {
    int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
    for(unsigned int i=0; i<(unsigned int)(_partitionsInX*_partitionsInY); i++)
    {
      TsdGridPartition* part = _partitions[0][i];
      if(!part->isInRange(tr, sensor, _maxTruncation)) continue;

      part->init(_maxTruncation);

      const obfloat* partCentroid = part->getCentroid();
      obfloat distCentroid = sqrt((partCentroid[0]-tr[0])*(partCentroid[0]-tr[0])+(partCentroid[1]-tr[1])*(partCentroid[1]-tr[1]));
      if(distCentroid > sensor->getMaximumRange()) distCentroid = sensor->getMaximumRange();
      obfloat partWeight = (sensor->getMaximumRange()-distCentroid)/sensor->getMaximumRange();
      partWeight *= partWeight;

      Matrix* partCoords = part->getPartitionCoords();
      Matrix* cellCoordsHom = part->getCellCoordsHom();
      sensor->backProject(cellCoordsHom, idx);
      const double lowReflectivityRange = sensor->getLowReflectivityRange();

      for(unsigned int c=0; c<partSize; c++)
      {
        // Index of laser beam
        const int index = idx[c];

        if(index>=0)
        {
          if(mask[index])
          {
            if(!isinf(data[index]))
            {
              // calculate signed distance, i.e., measurement minus distance of current cell to sensor
              const double sd = data[index] - sqrt( ((*cellCoordsHom)(c,0)-tr[0]) * ((*cellCoordsHom)(c,0)-tr[0]) + ((*cellCoordsHom)(c,1)-tr[1]) * ((*cellCoordsHom)(c,1)-tr[1]));

              part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), sd, partWeight);
            }
            else
            {
              const double dist = sqrt( ((*cellCoordsHom)(c,0)-tr[0]) * ((*cellCoordsHom)(c,0)-tr[0]) + ((*cellCoordsHom)(c,1)-tr[1]) * ((*cellCoordsHom)(c,1)-tr[1]));
              if(dist<lowReflectivityRange)
                part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), _maxTruncation, partWeight);
            }
          }
        }
      }
    }
    delete [] idx;
  }

  propagateBorders();

  LOGMSG(DBG_DEBUG, "Elapsed push: " << t.elapsed() << "s");

  _initialPushAccomplished = true;
}

void TsdGrid::pushTree(SensorPolar2D* sensor)
{
  Timer t;
  t.start();
  double* data     = sensor->getRealMeasurementData();

  obfloat tr[2];
  sensor->getPosition(tr);

  TsdGridComponent* comp = _tree;
  vector<TsdGridPartition*> partitionsToCheck;
  pushRecursion(sensor, tr, comp, partitionsToCheck);

  LOGMSG(DBG_DEBUG, "Partitions to check: " << partitionsToCheck.size());

#pragma omp parallel
  {
    unsigned int partSize = (_partitions[0][0])->getSize();
    int* idx = new int[partSize];
#pragma omp for schedule(dynamic)
    for(unsigned int i=0; i<partitionsToCheck.size(); i++)
    {
      TsdGridPartition* part = partitionsToCheck[i];
      part->init(_maxTruncation);

      obfloat* partCentroid = part->getCentroid();
      obfloat distCentroid = sqrt((partCentroid[0]-tr[0])*(partCentroid[0]-tr[0])+(partCentroid[1]-tr[1])*(partCentroid[1]-tr[1]));
      if(distCentroid > sensor->getMaximumRange()) distCentroid = sensor->getMaximumRange();
      obfloat partWeight = (sensor->getMaximumRange()-distCentroid)/sensor->getMaximumRange();
      partWeight *= partWeight;

      Matrix* partCoords = part->getPartitionCoords();
      Matrix* cellCoordsHom = part->getCellCoordsHom();
      sensor->backProject(cellCoordsHom, idx);

      for(unsigned int c=0; c<partSize; c++)
      {
        // Index of laser beam
        int index = idx[c];

        if(index>=0)
        {
          if(!isinf(data[index]))
          {
            // calculate signed distance, i.e. measurement minus distance of current cell to sensor
            double sd = data[index] - sqrt( ((*cellCoordsHom)(c,0)-tr[0]) * ((*cellCoordsHom)(c,0)-tr[0]) + ((*cellCoordsHom)(c,1)-tr[1]) * ((*cellCoordsHom)(c,1)-tr[1]));

            part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), sd, partWeight);
          }
          else
          {
            double dist = sqrt( ((*cellCoordsHom)(c,0)-tr[0]) * ((*cellCoordsHom)(c,0)-tr[0]) + ((*cellCoordsHom)(c,1)-tr[1]) * ((*cellCoordsHom)(c,1)-tr[1]));
            if(dist<sensor->getLowReflectivityRange())
              part->addTsd((*partCoords)(c, 0), (*partCoords)(c, 1), _maxTruncation, partWeight);
          }
        }
      }
    }
    delete [] idx;
  }

  propagateBorders();

  LOGMSG(DBG_DEBUG, "Elapsed pushTree: " << t.elapsed() << "s");
}

bool TsdGrid::containsData()
{
  return _initialPushAccomplished;
}

void TsdGrid::pushRecursion(SensorPolar2D* sensor, obfloat pos[2], TsdGridComponent* comp, vector<TsdGridPartition*> &partitionsToCheck)
{
  if(comp->isInRange(pos, sensor, _maxTruncation))
  {
    if(comp->isLeaf())
      partitionsToCheck.push_back((TsdGridPartition*)comp);
    else
    {
      vector<TsdGridComponent*> children = ((TsdGridBranch*)comp)->getChildren();
      for(unsigned int i=0; i<children.size(); i++)
        pushRecursion(sensor, pos, children[i], partitionsToCheck);
    }
  }
}

void TsdGrid::propagateBorders()
{
  unsigned int width  = _partitions[0][0]->getWidth();
  unsigned int height = _partitions[0][0]->getHeight();

  // Copy valid tsd values of neighbors to borders of each partition.
  // Skip outmost partitions for the moment, they are negligible.
  for(int py=0; py<_partitionsInY; py++)
  {
    for(int px=0; px<_partitionsInX; px++)
    {
      TsdGridPartition* partCur       = _partitions[py][px];

      if(!partCur->isInitialized()) continue;

      if(px<(_partitionsInX-1))
      {
        TsdGridPartition* partRight     = _partitions[py][px+1];
        if(partRight->isInitialized())
        {
          // Copy right border
          for(unsigned int i=0; i<height; i++)
          {
            partCur->_grid[i][width].tsd = partRight->_grid[i][0].tsd;
            partCur->_grid[i][width].weight = partRight->_grid[i][0].weight;
          }
        }
      }

      if(py<(_partitionsInY-1))
      {
        TsdGridPartition* partUp        = _partitions[py+1][px];
        if(partUp->isInitialized())
        {
          // Copy upper border
          for(unsigned int i=0; i<width; i++)
          {
            partCur->_grid[height][i].tsd = partUp->_grid[0][i].tsd;
            partCur->_grid[height][i].weight = partUp->_grid[0][i].weight;
          }
        }
      }

      if(px<(_partitionsInX-1) && py<(_partitionsInY-1))
      {
        TsdGridPartition* partUpRight   = _partitions[py+1][px+1];
        if(partUpRight->isInitialized())
        {
          // Copy upper right corner
          partCur->_grid[height][width].tsd = partUpRight->_grid[0][0].tsd;
          partCur->_grid[height][width].weight = partUpRight->_grid[0][0].weight;
        }
      }
    }
  }
}

void TsdGrid::grid2ColorImage(unsigned char* image, unsigned int width, unsigned int height)
{
  unsigned char rgb[3];

  obfloat stepW = getMaxX() / (obfloat)width;
  obfloat stepH = getMaxY() / (obfloat)height;

  obfloat py = 0.0;
  unsigned int i = 0;
  for(unsigned int h=0; h<height; h++)
  {
    obfloat px = 0.0;
    for(unsigned int w=0; w<width; w++, i++)
    {
      obfloat coord[2];
      coord[0] = px;
      coord[1] = py;
      int p, x, y;
      obfloat dx, dy;
      obfloat tsd = NAN;
      bool isEmpty = false;

      if(coord2Cell(coord, &p, &x, &y, &dx, &dy))
      {
        if(_partitions[0][p]->isInitialized())
          tsd = _partitions[0][p]->_grid[y][x].tsd;

        isEmpty = _partitions[0][p]->isEmpty();
      }

      if(tsd>0.0)
      {
        rgb[0] = static_cast<unsigned char>(tsd * 255.0);
        rgb[1] = 255;
        rgb[2] = static_cast<unsigned char>(tsd * 255.0);
      }
      else if(tsd<0.0)
      {
        rgb[0] = static_cast<unsigned char>((1.0+tsd) * 255.0);
        rgb[1] = 0.0;
        rgb[2] = 0.0;
      }
      else if(isEmpty)
      {
        rgb[0] = 255;
        rgb[1] = 255;
        rgb[2] = 255;
      }
      else
      {
        rgb[0] = 0;
        rgb[1] = 0;
        rgb[2] = 0;
      }
      memcpy(&image[3*i], rgb, 3*sizeof(unsigned char));
      px += stepW;
    }
    py += stepH;
  }
}

void TsdGrid::getData(std::vector<double>& data)
{
  obfloat coordVar[2] = {0.0};
  int p = 0;
  int x = 0;
  int y = 0;
  obfloat dx = 0.0;
  obfloat dy = 0.0;
  obfloat tsd = 0.0;
  for(coordVar[1] = 0.0; coordVar[1] < _maxY; coordVar[1] += _cellSize)
  {
    for(coordVar[0] = 0.0; coordVar[0] < _maxX; coordVar[0] += _cellSize)
    {
      if(this->coord2Cell(coordVar, &p, &x, &y, &dx, &dy))
      {
        if(_partitions[0][p]->isInitialized())
          tsd = _partitions[0][p]->_grid[y][x].tsd;
        else
          tsd = NAN;
      }
      else
        tsd = NAN;
      data.push_back(tsd);
    }
  }
}

bool TsdGrid::interpolateNormal(const obfloat* coord, obfloat* normal)
{
  obfloat neighbor[3];
  obfloat depthInc = 0;
  obfloat depthDec = 0;

  neighbor[0] = coord[0] + _cellSize;
  neighbor[1] = coord[1];
  if(interpolateBilinear(neighbor, &depthInc)!=INTERPOLATE_SUCCESS) return false;

  neighbor[0] = coord[0] - _cellSize;
  // neighbor[1] = coord[1];
  if(interpolateBilinear(neighbor, &depthDec)!=INTERPOLATE_SUCCESS) return false;

  normal[0] = depthInc - depthDec;

  neighbor[0] = coord[0];
  neighbor[1] = coord[1] + _cellSize;
  if(interpolateBilinear(neighbor, &depthInc)!=INTERPOLATE_SUCCESS) return false;

  // neighbor[0] = coord[0];
  neighbor[1] = coord[1] - _cellSize;
  if(interpolateBilinear(neighbor, &depthDec)!=INTERPOLATE_SUCCESS) return false;

  normal[1] = depthInc - depthDec;

  norm2<obfloat>(normal);

  return true;
}

bool TsdGrid::storeGrid(const std::string& path)
{
  if(!path.size())
  {
    LOGMSG(DBG_ERROR, " error! Path invalid!\n");
    return(false);
  }
  std::fstream outFile;
  outFile.open(path.c_str(), std::fstream::out);
  if(!outFile.is_open())
  {
    LOGMSG(DBG_ERROR, " error opening file " << path << "\n");
    return(false);
  }
  //store grid layout and cellsize
  outFile << _cellSize << "\n" << _layoutPartitions << "\n" << _layoutGrid << "\n" << _maxTruncation << "\n";

  //read the grid partition wise and fill the cell data into the file
  //loop over all partitions
  for(int y = 0; y < _partitionsInY; y++)
  {
    for(int x = 0; x < _partitionsInX; x++)
    {
      //generate partition identifier
      TsdGridPartition* curPart = _partitions[y][x];
      EnumTsdGridPartitionIdentifier id;
      if(curPart->isInitialized())
      {
        id = CONTENT;
        outFile << id << "\n";
        for(unsigned int py=0; py < curPart->getHeight(); py++)
        {
          for(unsigned int px = 0; px < curPart->getWidth(); px++)
          {
            outFile << curPart->_grid[py][px].tsd << "\n";
            outFile << curPart->_grid[py][px].weight << "\n";
          }
        }
      }
      else //partition is not initialized
      {
        if(curPart->isEmpty()) //partition is not initialized but empty
        {
          id = EMPTY;
          outFile << id << "\n";
          outFile << curPart->_initWeight << "\n";
        }
        else
        {
          id = UNINITIALIZED;
          outFile << id << "\n";
        }
      }
    }
  }
  outFile.close();
  if(outFile.is_open())
    LOGMSG(DBG_DEBUG, " warning! File " << path << " is still open!\n");
  return(true);
}

bool TsdGrid::freeFootprint(const obfloat centerCoords[2], const obfloat width, const obfloat height)
{
  unsigned int minX = static_cast<unsigned int>((centerCoords[0] - width * 0.5) / _cellSize + 0.5);
  unsigned int maxX = static_cast<unsigned int>((centerCoords[0] + width * 0.5) / _cellSize + 0.5);
  unsigned int minY = static_cast<unsigned int>((centerCoords[1] - height * 0.5) / _cellSize + 0.5);
  unsigned int maxY = static_cast<unsigned int>((centerCoords[1] + height * 0.5) / _cellSize + 0.5);

  //check whether indices are in bounds
  if((minX > static_cast<unsigned int>(_cellsX)) || (maxX > static_cast<unsigned int>(_cellsX)) ||
      (minY > static_cast<unsigned int>(_cellsY)) || (maxY > static_cast<unsigned int>(_cellsY)))
  {
    LOGMSG(DBG_ERROR, " Error indices out of bounds\n");
    return false;
  }
  unsigned int dimPartition = static_cast<unsigned int>(_dimPartition);
  for(unsigned int rows = minY; rows < maxY; rows++)
  {
    for(unsigned int cols = minX; cols < maxX; cols++)
    {
      unsigned int py = rows / dimPartition;
      unsigned int px = cols / dimPartition;
      if(!_partitions[py][px]->isInitialized())   //partition uninitialized -> initialize
        _partitions[py][px]->init(_maxTruncation);
      unsigned int cy = rows % dimPartition;
      unsigned int cx = cols % dimPartition;
      (*_partitions[py][px])(cy, cx) = TSDINC;
    }
  }
  return true;
}

}
