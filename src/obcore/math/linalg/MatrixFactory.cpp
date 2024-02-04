#include "MatrixFactory.h"
#include <math.h>
#include "linalg.h"

namespace obvious
{

Matrix MatrixFactory::RotationMatrix22(obfloat phi)
{
  Matrix M(2, 2);
  obfloat cp = cos(phi);
  obfloat sp = sin(phi);
  M(0,0) =  cp;
  M(0,1) = -sp;
  M(1,0) =  sp;
  M(1,1) =  cp;
  return M;
}

Matrix MatrixFactory::RotationMatrix33AboutX(obfloat psi)
{
  Matrix M(3, 3);
  obfloat cp = cos(psi);
  obfloat sp = sin(psi);
  M(0,0) = 1.0;
  M(0,1) = 0.0;
  M(0,2) = 0.0;
  M(1,0) = 0.0;
  M(1,1) =  cp;
  M(1,2) = -sp;
  M(2,0) = 0.0;
  M(2,1) = sp;
  M(2,2) = cp;
  return M;
}

Matrix MatrixFactory::RotationMatrix33AboutY(obfloat theta)
{
  Matrix M(3, 3);
  obfloat ct = cos(theta);
  obfloat st = sin(theta);
  M(0,0) =  ct;
  M(0,1) = 0.0;
  M(0,2) =  st;
  M(1,0) = 0.0;
  M(1,1) = 1.0;
  M(1,2) = 0.0;
  M(2,0) = -st;
  M(2,1) = 0.0;
  M(2,2) =  ct;
  return M;
}

Matrix MatrixFactory::RotationMatrix33AboutZ(obfloat phi)
{
  Matrix M(3, 3);
  obfloat cp = cos(phi);
  obfloat sp = sin(phi);
  M(0,0) =  cp;
  M(0,1) = -sp;
  M(0,2) = 0.0;
  M(1,0) =  sp;
  M(1,1) =  cp;
  M(1,2) = 0.0;
  M(2,0) = 0.0;
  M(2,1) = 0.0;
  M(2,2) = 1.0;
  return M;
}

Matrix MatrixFactory::TranslationMatrix33(obfloat tx, obfloat ty)
{
  Matrix M(3, 3);
  M.setIdentity();
  M(0,2) = tx;
  M(1,2) = ty;
  return M;
}

Matrix MatrixFactory::TranslationMatrix44(obfloat tx, obfloat ty, obfloat tz)
{
  Matrix M(4, 4);
  M.setIdentity();
  M(0,3) = tx;
  M(1,3) = ty;
  M(2,3) = tz;
  return M;
}

Matrix MatrixFactory::TransformationMatrix33(obfloat phi, obfloat tx, obfloat ty)
{
  Matrix M = MatrixFactory::TranslationMatrix33(tx, ty);
  obfloat cphi   = cos(phi);
  obfloat sphi   = sin(phi);
  M(0,0) = cphi;     M(0,1) = -sphi;
  M(1,0) = sphi;     M(1,1) = cphi;
  return M;
}

Matrix MatrixFactory::TransformationMatrix44(obfloat phi, obfloat theta, obfloat psi, obfloat tx, obfloat ty, obfloat tz)
{
  Matrix M = MatrixFactory::TranslationMatrix44(tx, ty, tz);
  obfloat cphi   = cos(phi);
  obfloat ctheta = cos(theta);
  obfloat cpsi   = cos(psi);
  obfloat sphi   = sin(phi);
  obfloat stheta = sin(theta);
  obfloat spsi   = sin(psi);
  M(0,0) = cphi*ctheta;     M(0,1) = cphi*stheta*spsi-sphi*cpsi;    M(0,2) = cphi*stheta*cpsi + sphi*spsi;
  M(1,0) = sphi*ctheta;     M(1,1) = sphi*stheta*spsi+cphi*cpsi;    M(1,2) = sphi*stheta*cpsi - cphi*spsi;
  M(2,0) = -stheta;         M(2,1) = ctheta*spsi;                   M(2,2) = ctheta*cpsi;
  return M;
}

}
