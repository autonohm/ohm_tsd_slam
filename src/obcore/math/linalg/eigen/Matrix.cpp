#include "Matrix.h"
#include "obcore/base/Logger.h"

#include <math.h>

namespace obvious
{

Matrix::Matrix(unsigned int rows, unsigned int cols, double* data)
{
  if(data)
  {
    Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > M(data, rows, cols);
    _M = M;
  }
  else
  {
    _M.resize(rows, cols);
  }
}

Matrix::Matrix(const Matrix &M)
{
  _M = Eigen::MatrixXd(M._M);
}

Matrix::Matrix(Matrix M, unsigned int i, unsigned int j, unsigned int rows, unsigned int cols)
{
  _M.resize(rows, cols);
  _M = M._M.block(i, j, rows, cols);
}

Matrix::~Matrix()
{

}

Matrix&  Matrix::operator =  (const Matrix &M)
{
  _M = M._M;
  return *this;
}

Matrix&  Matrix::operator *=  (const Matrix &M)
{
  _M *= M._M;
  return *this;
}

Matrix& Matrix::operator -= (const Matrix &M)
{
  _M -= M._M;
  return *this;
}

Matrix& Matrix::operator += (const Matrix &M)
{
  _M += M._M;
  return *this;
}

Matrix& Matrix::operator += (const double scalar)
{
  _M.array() += scalar;
  return *this;
}

void Matrix::addToColumn(unsigned int column, const double scalar)
{
  _M.col(column).array() += scalar;
}

void Matrix::addToRow(unsigned int row, const double scalar)
{
  _M.row(row).array() += scalar;
}

double& Matrix::operator () (unsigned int row, unsigned int col)
{
  return _M(row, col);
}

Matrix operator * (const Matrix &M1, const Matrix &M2)
{
  Matrix M(M1._M.rows(), M2._M.cols());
  M._M = M1._M * M2._M;
  return M;
}

void Matrix::multiplyRight(const Matrix &M, bool transposeArg1, bool transposeArg2)
{
  Eigen::MatrixXd Mleft = _M;
  Eigen::MatrixXd Mright = M._M;
  if(transposeArg1 && transposeArg2)
    _M.noalias() = Mleft.transpose() * Mright.transpose();
  else if(transposeArg1)
    _M.noalias() = Mleft.transpose() * Mright;
  else if(transposeArg2)
    _M.noalias() = Mleft * Mright.transpose();
  else
    _M.noalias() = Mleft * Mright;
}

ostream& operator <<(ostream &os, Matrix &M)
{
  os << M;
  return os;
}

void Matrix::getData(double* array) const
{
  int i=0;
  for(int r=0; r<_M.rows(); r++)
    for(int c=0; c<_M.cols(); c++, i++)
      array[i] = _M(r,c);
}

void Matrix::setData(double* array)
{
  Eigen::Map< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > > M(array, _M.rows(), _M.cols());
  _M = M;
}

unsigned int Matrix::getRows() const
{
  return _M.rows();
}

unsigned int Matrix::getCols() const
{
  return _M.cols();
}

void Matrix::setIdentity()
{
  _M.setIdentity();
}

void Matrix::setZero()
{
  _M.setZero(_M.rows(), _M.cols());
}

Matrix Matrix::getInverse()
{
  Matrix Mi = *this;
  Mi.invert();
  return Mi;
}

void Matrix::invert()
{
  _M = _M.inverse();
}

void Matrix::transpose()
{
  _M.transposeInPlace();
}

Matrix Matrix::getTranspose()
{
  Matrix M = *this;
  M.transpose();
  return M;
}

double Matrix::trace()
{
  return _M.trace();
}

Matrix* Matrix::pcaAnalysis()
{
  Eigen::MatrixXd M = _M;
  Eigen::VectorXd centroid =  M.colwise().mean();
  Eigen::MatrixXd centered = M.rowwise() - M.colwise().mean();
  Eigen::MatrixXd cov = centered.adjoint() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);

  unsigned int dim = M.cols();
  Matrix* axes = new Matrix(dim, 2*dim);

  //cout << "eigenvalues" << endl;
  //cout << eig.eigenvalues() << endl;

  //cout << "V" << endl;
  //cout << eig.eigenvectors().rightCols(3) << endl;

  Eigen::MatrixXd Vtmp = eig.eigenvectors();
  for(unsigned int i=0; i<Vtmp.cols(); i++)
    if(eig.eigenvalues()(i) < 0) Vtmp.col(i) *= -1;

  Eigen::MatrixXd V = Vtmp;
  for(unsigned int i=0; i<V.cols(); i++)
    V.col(i) = Vtmp.col(V.cols()-i-1);

  //cout << "V2" << endl;
  //cout << V << endl;

  Eigen::MatrixXd P = V.transpose() * centered.transpose();

  //cout << "P" << endl;
  //cout << P << endl;

  for(unsigned int i=0; i<dim; i++)
  {
    // corresponding eigenvector in original coordinate system
    double max = P.row(i).maxCoeff();
    double min = P.row(i).minCoeff();
    double ext = max - min;
    double align = 0.0;
    if(ext > 1e-6)
    {
      align = (max + min)/2.0;
    }

    for(size_t j=0; j<dim; j++)
    {
      double e = V(j, i)*align;
      centroid(j) += e;
    }

    //cout << "max/min/ext/align: " << max << " " << min << " " << ext << " " << align << endl;
  }

  for(unsigned int i=0; i<dim; i++)
  {
    //cout << "cent: " << centroid(i) << endl;
    // extends of axis in orientation of eigenvector i
    double ext = P.row(i).maxCoeff() - P.row(i).minCoeff();

    // coordinates of axis j
    for(size_t j=0; j<dim; j++)
    {
      double e = V.col(i)(j)*ext/2.0;
      // axis coordinates with respect to center of original coordinate system
      (*axes)(i, 2*j) = centroid(j) - e;
      (*axes)(i, 2*j+1) = centroid(j) + e;
    }
  }

  return axes;
}

void Matrix::svd(Matrix* U, double* s, Matrix* V)
{
  if(U->getCols() != getCols() || U->getRows() != getRows())
  {
    LOGMSG(DBG_ERROR, "Matrix U must have same dimension");
    return;
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svdOfM = _M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  U->_M = svdOfM.matrixU();
  V->_M = svdOfM.matrixV();
  Eigen::VectorXd singular = svdOfM.singularValues();
  for(int i=0; i<singular.count(); i++)
    s[i] = singular(i);
}

void Matrix::solve(double* b, double* x)
{
  Eigen::Map<Eigen::MatrixXd> vb(b, _M.rows(), 1);
  Eigen::Map<Eigen::MatrixXd> vx(x, _M.rows(), 1);
  vx = _M.lu().solve(vb);
}

void Matrix::leastSquares(double* b, double* x)
{
  cout << "not implemented yet" << endl;
  abort();
}

Matrix Matrix::createTransform(Matrix T)
{
  unsigned int dim = getCols();

  if(dim!=2 && dim!=3)
  {
    LOGMSG(DBG_ERROR, "Matrix dimension invalid");
    abort();
  }

  Matrix M = *this;
  M.transform(T);
  return M;
}

void Matrix::transform(Matrix T)
{
  unsigned int dim = getCols();

  if(dim!=2 && dim!=3)
  {
    LOGMSG(DBG_ERROR, "Matrix dimension invalid");
    abort();
  }

  // Apply rotation
  Matrix Rt(dim, dim);
  for(unsigned int r=0; r<dim; r++)
    for(unsigned int c=0; c<dim; c++)
      Rt(c, r) = T(r, c);

  _M *= Rt._M;
  addToColumn(0, T(0, dim));
  addToColumn(1, T(1, dim));
  if(dim==3)
    addToColumn(2, T(2, dim));
}

void Matrix::print() const
{
  cout << _M;
  cout << endl;
}

void Matrix::print()
{
  cout << _M;
  cout << endl;
}

Matrix Matrix::multiply(const Matrix &M1, const Matrix &M2, bool transposeArg1, bool transposeArg2)
{
  Eigen::MatrixXd Mleft = M1._M;
  Eigen::MatrixXd Mright = M2._M;
  Eigen::MatrixXd X;
  if(transposeArg1 && transposeArg2)
    X = Mleft.transpose() * Mright.transpose();
  else if(transposeArg1)
    X = Mleft.transpose() * Mright;
  else if(transposeArg2)
    X = Mleft * Mright.transpose();
  else
    X = Mleft * Mright;
  Matrix M(X.rows(), X.cols());
  M._M = X;
  return M;
}

Vector Matrix::multiply(const Matrix &M, const Vector &V, bool transpose)
{
  Vector V2(M._M.rows());
  if(transpose)
    V2._V = M._M.transpose() * V._V;
  else
    V2._V = M._M * V._V;
  return V2;
}

typedef Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > MapType;
void Matrix::multiply(const Matrix &M1, double* array, unsigned int rows, unsigned int cols)
{
  MapType map(array, rows, cols);
  map = map * M1._M.transpose();
}

}
