/*
 * MultiSlamNode.h
 *
 *  Created on: Oct 30, 2014
 *      Author: phil
 */

#ifndef SRC_MULTI_MULTISLAMNODE_H_
#define SRC_MULTI_MULTISLAMNODE_H_

#include "SlamBase.h"

#include <vector>

namespace ohm_tsd_slam
{

class ThreadLocalize;

class MultiSlamNode : public SlamBase
{
public:
  MultiSlamNode();
  virtual ~MultiSlamNode();
 // void start(void);
private:
  void run(void);
  void init(void);
  std::vector<ThreadLocalize*> _localizers;
};

}

#endif /* SRC_MULTI_MULTISLAMNODE_H_ */
