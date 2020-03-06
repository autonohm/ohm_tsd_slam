#ifndef TSDPDFMATCHER_H_
#define TSDPDFMATCHER_H_

#include "IPreMatcher.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/registration/ransacMatching/TSD_PDFMatching.h"
#include <memory>
#include <string>
#include <tinyxml2.h>

class TsdPdfMatcher : public IPrematcher
{
public:
  TsdPdfMatcher(obvious::SensorPolar2D& sensor, obvious::TsdGrid& grid, const std::string& nameSpace = "");
  virtual ~TsdPdfMatcher();
  virtual bool init(const std::string& configXml) override;
virtual bool match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T)override;
private:
  const std::string& _nameSpace;
  std::string                               _configXml;
  double                                    _ranPhiMax; // TODO: base class
  double                                    _transMax;  // TODO: base class
  std::unique_ptr<obvious::TSD_PDFMatching> _matcher;
  std::shared_ptr<obvious::SensorPolar2D>   _sensor; // TODO base class
  obvious::TsdGrid& _grid;
};
#endif