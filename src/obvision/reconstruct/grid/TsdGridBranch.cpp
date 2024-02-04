#include "TsdGridBranch.h"
#include <math.h>
#include <iostream>

using namespace std;

namespace obvious
{

TsdGridBranch::TsdGridBranch(TsdGridComponent*** leafs, int x, int y, int level) : TsdGridComponent(false)
{
  _edgeCoordsHom = new Matrix(4, 3);

  TsdGridComponent* branch;
  TsdGridComponent* branchRight;
  TsdGridComponent* branchDown;
  TsdGridComponent* branchDownRight;

  if(level==1)
  {
    branch          = leafs[y][x];
    branchRight     = leafs[y][x+1];
    branchDown      = leafs[y+1][x];
    branchDownRight = leafs[y+1][x+1];
  }
  else
  {
    _isLeaf = true;
    level--;
    int step = 1 << level;
    branch          = new TsdGridBranch(leafs, x,      y, level);
    branchRight     = new TsdGridBranch(leafs, x+step, y, level);
    branchDown      = new TsdGridBranch(leafs, x,      y+step, level);
    branchDownRight = new TsdGridBranch(leafs, x+step, y+step, level);
  }

  _children.push_back(branch);
  _children.push_back(branchRight);
  _children.push_back(branchDown);
  _children.push_back(branchDownRight);

  // Calculate mean of centroids
  obfloat* c        = branch->getCentroid();
  obfloat* cR       = branchRight->getCentroid();
  obfloat* cD       = branchDown->getCentroid();
  obfloat* cDR      = branchDownRight->getCentroid();

  _centroid[0]     = (c[0] + cR[0] + cD[0] + cDR[0]) / 4.0;
  _centroid[1]     = (c[1] + cR[1] + cD[1] + cDR[1]) / 4.0;

  // Get outer bounds of leafs
  (*_edgeCoordsHom)(0, 0) = (*(branch->getEdgeCoordsHom()))(0,0);
  (*_edgeCoordsHom)(0, 1) = (*(branch->getEdgeCoordsHom()))(0,1);
  (*_edgeCoordsHom)(0, 2) = (*(branch->getEdgeCoordsHom()))(0,2);

  (*_edgeCoordsHom)(1, 0) = (*(branchRight->getEdgeCoordsHom()))(1,0);
  (*_edgeCoordsHom)(1, 1) = (*(branchRight->getEdgeCoordsHom()))(1,1);
  (*_edgeCoordsHom)(1, 2) = (*(branchRight->getEdgeCoordsHom()))(1,2);

  (*_edgeCoordsHom)(2, 0) = (*(branchDown->getEdgeCoordsHom()))(2,0);
  (*_edgeCoordsHom)(2, 1) = (*(branchDown->getEdgeCoordsHom()))(2,1);
  (*_edgeCoordsHom)(2, 2) = (*(branchDown->getEdgeCoordsHom()))(2,2);

  (*_edgeCoordsHom)(3, 0) = (*(branchDownRight->getEdgeCoordsHom()))(3,0);
  (*_edgeCoordsHom)(3, 1) = (*(branchDownRight->getEdgeCoordsHom()))(3,1);
  (*_edgeCoordsHom)(3, 2) = (*(branchDownRight->getEdgeCoordsHom()))(3,2);

  _componentSize = 2.0 * branch->getComponentSize();
  _circumradius = 2.0 * branch->getCircumradius();
}

TsdGridBranch::~TsdGridBranch()
{
  if(!_children[0]->isLeaf())
  {
    for(int i=0; i<4; i++)
    {
      delete _children[i];
    }
  }

  _children.clear();

  delete _edgeCoordsHom;
}

vector<TsdGridComponent*> TsdGridBranch::getChildren()
{
  return _children;
}

void TsdGridBranch::increaseEmptiness()
{
  for(int i=0; i<4; i++)
    _children[i]->increaseEmptiness();
}

static int level = 0;
void TsdGridBranch::print()
{
  for(int i=0; i<level; i++)
    cout << "   ";

  level++;

  obfloat* c = getCentroid();
  cout << "(" << c[0] << " " << c[1] << ")" << endl;

  if(_children[0]->isLeaf())
  {
    for(int i=0; i<level; i++)
      cout << "   ";

    for(int i=0; i<4; i++)
    {
      obfloat* c        = _children[i]->getCentroid();
      cout << "(" << c[0] << " " << c[1] << ") ";
    }
    cout << endl;
  }
  else
  {
    for(int i=0; i<4; i++)
      ((TsdGridBranch*)_children[i])->print();
  }

  level--;
}

void TsdGridBranch::printEdges()
{
  cout << "#" << level << endl;

  level++;

  Matrix* M        = getEdgeCoordsHom();
  M->print();

  if(_children[0]->isLeaf())
  {
    for(int i=0; i<4; i++)
    {
      Matrix* M        = _children[i]->getEdgeCoordsHom();
      M->print();
    }
    cout << endl;
  }
  else
  {
    for(int i=0; i<4; i++)
      ((TsdGridBranch*)_children[i])->printEdges();
  }

  level--;
}

}
