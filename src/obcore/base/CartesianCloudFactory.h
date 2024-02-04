#ifndef CARTESIANCLOUDFACTORY_H
#define CARTESIANCLOUDFACTORY_H

#include "obcore/base/CartesianCloud.h"

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

enum EnumFileFormat { eFormatAscii };

/**
 * @class CartesianCloudFactory
 * @brief
 * @author Stefan May
 **/
class CartesianCloudFactory
{
public:
	
  static CartesianCloud3D* load(char* filename, EnumFileFormat format);

  static void serialize(char* filename, CartesianCloud3D* cloud, EnumFileFormat format);
};

}

#endif /*CARTESIANCLOUDFACTORY_H*/
