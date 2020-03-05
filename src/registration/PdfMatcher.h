#ifndef PDFPREMATCHER_H_
#define PDFPREMATCHER_H_

#include "IPreMatcher.h"
#include <memory>
#include "obvision/registration/ransacMatching/PDFMatching.h"
#include "obvision/reconstruct/grid/SensorPolar2D.h"

class PdfMatcher : public IPrematcher
{
  public: 
    PdfMatcher(obvious::SensorPolar2D& sensor);
    virtual ~PdfMatcher();
    virtual bool init(const std::string& configXml) override;
    virtual bool match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T)override;
    private:
      std::unique_ptr<obvious::PDFMatching> _matcher;
      std::shared_ptr<obvious::SensorPolar2D>   _sensor; // TODO base class
};

#endif