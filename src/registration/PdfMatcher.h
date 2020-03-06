#ifndef PDFPREMATCHER_H_
#define PDFPREMATCHER_H_

#include "IPreMatcher.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"
#include "obvision/registration/ransacMatching/PDFMatching.h"
#include <memory>

class PdfMatcher : public IPrematcher
{
public:
  PdfMatcher(obvious::SensorPolar2D& sensor, const std::string& nameSpace = "");
  virtual ~PdfMatcher();
  virtual bool init(const std::string& configXml) override;
  virtual bool match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T) override;

private:
  const std::string _nameSpace;
  std::unique_ptr<obvious::PDFMatching>   _matcher;
  std::shared_ptr<obvious::SensorPolar2D> _sensor; // TODO base class
  double _ranPhiMax = 0.0;
  double _regTrnsMax = 0.0;
};

#endif