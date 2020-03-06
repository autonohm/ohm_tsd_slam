#include "PdfMatcher.h"
#include "utilities.h"
#include <ros/ros.h>
#include <tinyxml2.h>

PdfMatcher::PdfMatcher(obvious::SensorPolar2D& sensor)
    : _sensor(&sensor)
{
}

PdfMatcher::~PdfMatcher() {}

bool PdfMatcher::init(const std::string& configXml)
{
  tinyxml2::XMLDocument doc;

  doc.LoadFile(configXml.c_str());
  tinyxml2::XMLNode* rootNode = doc.RootElement();

  if(!rootNode)
  {
    ROS_ERROR_STREAM("no node in xml file " << configXml << " .");
    return false;
  }
  std::string rootNodeName = rootNode->ToElement()->Name();
  if(rootNodeName != std::string("pdf_config"))
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Wrong config file. Root node must be " << std::string("pdf_config") << std::endl);
    return false;
  }
  unsigned int trials              = 0;
  double       epsThresh           = 0.0;
  unsigned int sizeControlSet      = 0;
  double       zhit                = 0.0;
  double       zphi                = 0.0;
  double       zShort              = 0.0;
  double       zMax                = 0.0;
  double       zRand               = 0.0;
  double       percentagePointsInc = 0.0;
  double       rangeMax            = 0.0;
  double       sigPhi              = 0.0;
  double       sigHit              = 0.0;
  double       lamShort            = 0.0;
  double       maxAngleDiff        = 0.0;
  double       maxAnglePenalty     = 0.0;

  if(!utilities::loadTyniXmlParameter(trials, std::string("trials"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(epsThresh, std::string("epsthresh"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(sizeControlSet, std::string("sizeControlSet"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(zhit, std::string("sizeControlSet"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(zphi, std::string("zphi"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(zShort, std::string("zshort"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(zMax, std::string("zmax"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(zRand, std::string("zrand"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(percentagePointsInc, std::string("percentage_points_inc"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(rangeMax, std::string("rangemax"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(sigPhi, std::string("sigphi"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(sigHit, std::string("sighit"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(lamShort, std::string("lamshort"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(maxAngleDiff, std::string("max_angle_diff"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(maxAnglePenalty, std::string("max_angle_penalty"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_ranPhiMax, std::string("ranphimax"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_regTrnsMax, std::string("reg_trs_max"), *rootNode))
    return false;

  _matcher = std::make_unique<obvious::PDFMatching>(trials, epsThresh, sizeControlSet, zhit, zphi, zShort, zMax, zRand, percentagePointsInc, rangeMax,
                                                    sigPhi, sigHit, lamShort, maxAngleDiff, maxAnglePenalty);
  return true;
}

bool PdfMatcher::match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T)
{
  if(!_matcher)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Not initilized. Call init() first" << std::endl);
    return false;
  }
  T = _matcher->match(M, maskM, NULL, S, maskS, obvious::deg2rad(_ranPhiMax), _regTrnsMax, sensor.getAngularResolution());
}