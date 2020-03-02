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
protected:
  virtual bool init(const std::string& configXml)                                             = 0;
  virtual bool doRegistration(obvious::SensorPolar2D* sensor, obvious::Matrix* M, obvious::Matrix* Mvalid, obvious::Matrix* N, obvious::Matrix* Nvalid,
                              obvious::Matrix* S, obvious::Matrix* Svalid, obvious::Matrix& T) = 0;
};

#endif