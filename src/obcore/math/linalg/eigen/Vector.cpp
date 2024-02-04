#include "Vector.h"

namespace obvious
{

Vector::Vector(unsigned int size, double* data)
{
  _V.resize(size);
  if(data)
  {
    for(unsigned int i=0; i<size; i++)
      _V(i) = data[i];
  }
}

Vector::Vector(const Vector &V)
{
  _V = V._V;
}

Vector::~Vector()
{

}

Vector& Vector::operator =  (const Vector &V)
{
  _V = V._V;
  return *this;
}

double& Vector::operator () (unsigned int i)
{
  return _V(i);
}

unsigned int Vector::getSize()
{
  return _V.count();
}

void Vector::setData(const double* array)
{
  for(unsigned int i=0; i<_V.count(); i++)
    _V[i] = array[i];
}

void Vector::setZero()
{
  _V.setZero();
}

void Vector::print()
{
  cout << _V;
  cout << endl;
}

}
