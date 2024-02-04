#ifndef MATRIX_H__
#define MATRIX_H__

#include <eigen3/Eigen/Dense>
#include <vector>

#include "Vector.h"

#include <iostream>

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class Matrix
 * @brief Matrix abstraction layer of Eigen
 * @author Stefan May
 */
class Matrix
{
friend class MatrixView;

public:
	/**
	 * Constructor
	 * @param rows number of matrix rows
	 * @param cols number of matrix columns
	 * @param data data buffer to be copied
	 */
	Matrix(unsigned int rows, unsigned int cols, double* data = NULL);

	/**
	 * Copy constructor
	 * @param M matrix to be copied
	 */
	Matrix(const Matrix &M);

	/**
	 * Copy constructor of submatrix
	 * @param M source matrix
	 * @param i start row
	 * @param j start column
	 * @param rows number of rows
	 * @param cols number of columns
	 */
	Matrix(Matrix M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols);

	/**
	 * Destructor
	 */
	~Matrix();

	/**
	 * Assignment operator, copies elements of source to this one
	 * @param M matrix assigned to this one
	 * @return this matrix instance
	 */
	Matrix& operator =  (const Matrix &M);

	/**
	 * Multiplication operator
	 * @param M matrix multiplied with this one
	 * @return this matrix instance
	 */
	Matrix& operator *= (const Matrix &M);

	/**
	 * Subtraction operator
	 * @param M matrix subtracted from this one
	 * @return this matrix instance
	 */
	Matrix& operator -= (const Matrix &M);

  /**
   * Addition operator
   * @param M matrix added to this one
   * @return this matrix instance
   */
  Matrix& operator += (const Matrix &M);

  /**
   * Addition operator
   * @param scalar scalar value added to this one
   * @return this matrix instance
   */
  Matrix& operator += (const double scalar);

  /**
   * Add constant to column of matrix
   * @param column column index
   * @param scalar scalar value
   */
  void addToColumn(unsigned int column, const double scalar);

  /**
   * Add constant to row of matrix
   * @param row row index
   * @param scalar scalar value
   */
  void addToRow(unsigned int row, const double scalar);

	/**
	 * Row accessor
	 * @param i row index
	 * @return row elements as array
	 */
  double& operator () (unsigned int row, unsigned int col);

	/**
	 * Multiplication operator
	 * @param M1 1st matrix of product
	 * @param M2 2nd matrix of product
	 * @return matrix product
	 */
	friend Matrix operator * (const Matrix &M1, const Matrix &M2);

  /**
   * Multiply matrix from right to this one (A), i.e. A = A * M
   * @param M matrix instance
   * @param transposeArg1 transpose matrix A
   * @param transposeArg2 transpose matrix M
   */
  void multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2);

	/**
	 * Stream operator
	 * @param os output stream
	 * @param M matrix to be streamed, e.g. printed out
	 */
	friend ostream& operator <<(ostream &os, Matrix &M);

	/**
	 * Data accessor
	 * @param array buffer to copy data into (must be instantiated outside)
	 */
	void getData(double* array) const;

	/**
	 * Data mutator
	 * @param array array to copy data from
	 */
	void setData(double* array);

	/**
	 * Property accessor
	 * @return number of matrix rows
	 */
	unsigned int getRows() const;

	/**
	 * Property accessor
	 * @return number of matrix columns
	 */
	unsigned int getCols() const ;

	/**
	 * Set matrix to identity
	 */
	void setIdentity();

	/**
	 * Set all matrix elements to zero
	 */
	void setZero();

	/**
	 * Instantiate an inverse of the present matrix
	 * @return inverse matrix as new instance
	 */
	Matrix getInverse();

	/**
	 * Invert present matrix
	 */
	void invert();

	/**
	 * Instantiate the transpose matrix of present matrix
	 * @return transposed matrix as new instance
	 */
	Matrix getTranspose();

	/**
	 * Transpose current matrix
	 */
	void transpose();

	/**
	 * Calculate trace of matrix
	 * @return trace
	 */
	double trace();

	/**
	 * perform principle component analysis
	 * @return matrix in layout [x1_from x1_to y1_from y1_to z1_from z1_to; x2...]
	 */
	Matrix* pcaAnalysis();

	/**
	 * perform singular value decomposition A = U S V'
	 * @param U orthogonal matrix U
	 * @param s singular values
	 * @param V orthogonal square matrix
	 */
	void svd(Matrix* U, double* s, Matrix* V);

	/**
	 * Solve A x = b
	 * @param[in] b b must have elements = number of rows of current matrix
	 * @param[out] x x must have elements = number of rows of current matrix
	 */
	void solve(double* b, double* x);

  void leastSquares(double* b, double* x);

	/**
   * Transform current matrix, i.e. with homogeneous transformation
   * @param[in] T transformation matrix
   * @return transformed matrix instance
   */
	Matrix createTransform(Matrix T);

	/**
	 * Transform current matrix, i.e. with homogeneous transformation
	 * @param[in] T transformation matrix
	 */
	void transform(Matrix T);

	/**
	 * Print matrix to output stream
	 */
	void print() const;
	void print();

  static Matrix multiply(const Matrix &M1, const Matrix &M2, bool transposeArg1, bool transposeArg2);

  static Vector multiply(const Matrix &M, const Vector &V, bool transpose);

  static void multiply(const Matrix &M1, double* array, unsigned int rows, unsigned int cols);

private:

	/**
	 * Internal matrix representation
	 */
	Eigen::MatrixXd _M;

};

}

#endif //MATRIX_H
