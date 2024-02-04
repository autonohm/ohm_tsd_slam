#include "Vector.h"

namespace obvious
{
Vector::Vector(unsigned int size, double* data)
{
  _V = gsl_vector_alloc(size);
  if(data)
    setData(data);
}

Vector::Vector(const Vector &V)
{
  _V = gsl_vector_alloc(V._V->size);
  gsl_vector_memcpy(_V, V._V);
}

Vector::~Vector()
{
  gsl_vector_free(_V);
}

Vector& Vector::operator =  (const Vector &V)
{
  gsl_vector_memcpy(_V, V._V);
  return *this;
}

double& Vector::operator () (unsigned int i)
{
  return *gsl_vector_ptr(_V, i);
}

unsigned int Vector::getSize()
{
  return _V->size;
}

void Vector::setData(const double* array)
{
  gsl_vector_const_view varray = gsl_vector_const_view_array(array, _V->size);
  gsl_vector_memcpy(_V, &varray.vector);
}

void Vector::setZero()
{
  gsl_vector_set_zero(_V);
}

void Vector::print()
{
  for(unsigned int i=0; i<_V->size; i++)
    cout << gsl_vector_get(_V, i);
  cout << endl;
}

}
