#ifndef TSDGRIDBRANCH_H
#define TSDGRIDBRANCH_H

#include "obvision/reconstruct/grid/TsdGridComponent.h"

namespace obvious
{

class TsdGridBranch : public TsdGridComponent
{
public:
  TsdGridBranch(TsdGridComponent*** leafs, int x, int y, int level);

  virtual ~TsdGridBranch();

  vector<TsdGridComponent*> getChildren();

  virtual void increaseEmptiness();

  void print();

  void printEdges();

private:

  vector<TsdGridComponent*> _children;

};

}

#endif
