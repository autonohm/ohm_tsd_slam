#ifndef TSDPDFMATCHER_H_
#define TSDPDFMATCHER_H_
#include "IPreMatcher.h"

#include <string>
#include <tinyxml2.h>

class TsdPdfMatcher : public IPrematcher
{
public:
  TsdPdfMatcher();
  virtual ~TsdPdfMatcher();
  virtual bool init(const std::string& configXml) override;
  virtual bool doRegistration(obvious::SensorPolar2D* sensor, obvious::Matrix* M, obvious::Matrix* Mvalid, obvious::Matrix* N, obvious::Matrix* Nvalid,
                              obvious::Matrix* S, obvious::Matrix* Svalid, obvious::Matrix& T) override;

private:
  std::string _configXml;
  int         _trials;
  int         _sizeControlSet;
  double      _epsThresh;
  double      _zrand;
};

#endif