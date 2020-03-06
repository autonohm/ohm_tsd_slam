#include "IPreMatcher.h"
#include "obvision/registration/ransacMatching/RandomNormalMatching.h"
#include <memory>

class RandomNormalMatcher : public IPrematcher
{
public:
  RandomNormalMatcher(const std::string& nameSpace = "");
  virtual ~RandomNormalMatcher();
  virtual bool init(const std::string& configXml) override;
  virtual bool match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T) override;

private:
const std::string _nameSpace;
  std::unique_ptr<obvious::RandomNormalMatching> _matcher;
  double                                         _ranPhiMax = 0.0;
  double                                         _regTrnsMax = 0.0;
};