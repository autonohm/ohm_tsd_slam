#include "CartesianCloudFactory.h"
#include <iostream>
#include <fstream>

#include "obcore/base/Logger.h"

namespace obvious
{

CartesianCloud3D* loadAscii(char* filename);
void serializeAscii(char* filename, CartesianCloud3D* cloud);

CartesianCloud3D* CartesianCloudFactory::load(char* filename, EnumFileFormat format)
{
  switch (format)
  {
  case eFormatAscii:
    return loadAscii(filename);
    break;
  default:
    LOGMSG(DBG_ERROR, "File format unknown");
    return NULL;
    break;
  }
}

void CartesianCloudFactory::serialize(char* filename, CartesianCloud3D* cloud, EnumFileFormat format)
{
  switch (format)
  {
  case eFormatAscii:
    return serializeAscii(filename, cloud);
    break;
  }
}

void serializeAscii(char* filename, CartesianCloud3D* cloud)
{
  ofstream file;
  file.open(filename, ios::out);

  // Number of points
  unsigned int cnt = cloud->size();
  unsigned char* colors = cloud->getColors();

  for(unsigned int i=0; i<cnt; i++)
  {
    file << (*(cloud->getCoords()))(i,0) << " " << (*(cloud->getCoords()))(i,1) << " " << (*(cloud->getCoords()))(i,2) << " ";
    if(colors) file << (unsigned int)colors[0] << " " << (unsigned int)colors[1] << " " << (unsigned int)colors[2] << endl;
  }

  file.close();
}

CartesianCloud3D* loadAscii(char* filename)
{
  ifstream file;
  file.open(filename, ios::in);

  // Determine number of points
  unsigned int size = 0;
  string line;
  while (getline(file, line))
    size++;
  file.clear();
  file.seekg(0);

  CartesianCloud3D* cloud = new CartesianCloud3D(size, true);
  double x, y, z, r, g, b;
  int cnt = 0;
  unsigned char* colors = cloud->getColors();
  int* attributes = cloud->getAttributes();
  int* sourceidx = cloud->getIndices();

  file >> x >> y >> z >> r >> g >> b;
  while (file.good())
  {
    (*(cloud->getCoords()))(cnt,0) = x;
    (*(cloud->getCoords()))(cnt,1) = y;
    (*(cloud->getCoords()))(cnt,2) = z;
    colors[3 * cnt] = r;
    colors[3 * cnt + 1] = g;
    colors[3 * cnt + 2] = b;
    attributes[cnt] = 0;
    if (z > 0) attributes[cnt] |= ePointAttrValid;
    sourceidx[cnt] = cnt;
    cnt++;
    file >> x >> y >> z >> r >> g >> b;
  }

  return cloud;
}

}
