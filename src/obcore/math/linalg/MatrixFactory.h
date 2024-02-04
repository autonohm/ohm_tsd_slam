#ifndef MATRIXFACTORY_H__
#define MATRIXFACTORY_H__

using namespace std;

#include "obcore/base/types.h"

/**
 * @namespace obvious
 */
namespace obvious
{

class Matrix;

/**
 * @class MatrixFactory
 * @author Stefan May
 */
class MatrixFactory
{
public:

  /**
   * Instantiate a 2x2 roation matrix
   * @param phi rotation about z-axis
   * @return matrix instance
   */
  static Matrix RotationMatrix22(obfloat phi);

  /**
   * Instantiate a 3x3 roation matrix
   * @param psi rotation about x-axis
   * @return matrix instance
   */
  static Matrix RotationMatrix33AboutX(obfloat psi);

  /**
   * Instantiate a 3x3 roation matrix
   * @param theta rotation about y-axis
   * @return matrix instance
   */
  static Matrix RotationMatrix33AboutY(obfloat theta);

  /**
   * Instantiate a 3x3 roation matrix
   * @param phi rotation about z-axis
   * @return matrix instance
   */
  static Matrix RotationMatrix33AboutZ(obfloat phi);

  /**
   * Instantiate a 4x4 translation matrix, i.e. identity with last column set to translational input
   * @param tx x-component of translation
   * @param ty y-component of translation
   */
  static Matrix TranslationMatrix33(obfloat tx, obfloat ty);

  /**
   * Instantiate a 4x4 translation matrix, i.e. identity with last column set to translational input
   * @param tx x-component of translation
   * @param ty y-component of translation
   * @param tz z-component of translation
   */
  static Matrix TranslationMatrix44(obfloat tx, obfloat ty, obfloat tz);

  /**
   * Instantiate a 3x3 transformation matrix
   * @param phi rotation about z-axis
   * @param tx x-component of translation
   * @param ty y-component of translation
   */
  static Matrix TransformationMatrix33(obfloat phi, obfloat tx, obfloat ty);


  /**
   * Instantiate a 4x4 transformation matrix
   * @param phi rotation about z-axis
   * @param theta rotation about y-axis
   * @param psi rotation about x-axis
   * @param tx x-component of translation
   * @param ty y-component of translation
   * @param tz z-component of translation
   */
  static Matrix TransformationMatrix44(obfloat phi, obfloat theta, obfloat psi, obfloat tx=0, obfloat ty=0, obfloat tz=0);

private:

};

}

#endif //MATRIXFACTORY_H
