#ifndef IPREMATCHER_H_
#define IPREMATCHER_H_

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

#include <string>

class IPrematcher
{
public:
  IPrematcher(){}
  virtual ~IPrematcher(){}

  virtual bool init(const std::string& configXml)                                             = 0;
  // virtual bool match(obvious::SensorPolar2D& sensor,
  //                                       obvious::Matrix* M,
  //                                       const bool* maskM,
  //                                       obvious::Matrix* NM,
  //                                       obvious::Matrix* S,
  //                                       const bool* maskS,
  //                                       double phiMax,
  //                                       const double transMax,
  //                                       const double resolution, obvious::Matrix& T) = 0;
  virtual bool match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T) = 0;
};

#endif