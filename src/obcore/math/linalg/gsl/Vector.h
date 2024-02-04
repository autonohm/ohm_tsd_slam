#ifndef VECTOR_H__
#define VECTOR_H__

#include <gsl/gsl_vector.h>

#include <iostream>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Vector
 * @brief Vector abstraction layer of GSL
 * @author Stefan May
 */
class Vector
{
  friend class Matrix;
  friend class VectorView;

public:
	/**
	 * Constructor
	 * @param size number of vector elements
	 * @param data data buffer to be copied
	 */
	Vector(unsigned int size, double* data = NULL);

	/**
	 * Copy constructor
	 * @param V vector to be copied
	 */
	Vector(const Vector &V);

	/**
	 * Destructor
	 */
	~Vector();

	/**
	 * Assignment operator
	 * @param V vector assigned to this one
	 * @return this vector instance
	 */
	Vector  &operator =  (const Vector &V);

	/**
	 * Element accessor
	 * @param i element index
	 * @return element
	 */
	double& operator () (unsigned int i);

	/**
	 * Property accessor
	 * @return number of vector elements
	 */
	unsigned int getSize();

	/**
	 * Set data from array
	 * @param array source array
	 */
	void setData(const double* array);

	/**
	 * Set all vector elements to zero
	 */
	void setZero();

	/**
	 * Print matrix to output stream
	 */
	void print();

private:

	/**
	 * Internal GSL representation
	 */
	gsl_vector* _V;

};

}

#endif //VECTOR_H
