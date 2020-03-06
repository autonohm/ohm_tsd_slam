#include "RandomNormalMatcher.h"
#include "obcore/math/mathbase.h"
#include "utilities.h"
#include <ros/ros.h>
#include <tinyxml2.h>

RandomNormalMatcher::RandomNormalMatcher(const std::string& nameSpace):
_nameSpace(nameSpace) 
{
  std::string configXml;
  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>(nameSpace + "/random_normal_matcher/config_file", configXml,
                           "/home/phil/workspace/ros/src/ohm_tsd_slam/config/"
                           "config_random_normal_matcher.xml");
  if(!configXml.size())
    throw "config not found";
  if(!this->init(configXml))
    throw "config invalid";
}

RandomNormalMatcher::~RandomNormalMatcher() {}

bool RandomNormalMatcher::init(const std::string& configXml)
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
  if(rootNodeName != std::string("config_random_normal_matcher"))
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Wrong config file. Root node must be " << std::string("config_random_normal_matcher") << std::endl);
    return false;
  }
  unsigned int trials         = 0;
  double       epsThresh      = 0.0;
  unsigned int sizeControlSet = 0;

  if(!utilities::loadTyniXmlParameter(trials, std::string("trials"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(epsThresh, std::string("epsthresh"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(sizeControlSet, std::string("epsthresh"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_ranPhiMax, std::string("ranphimax"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_regTrnsMax, std::string("reg_trs_max"), *rootNode))
    return false;
   _matcher = std::make_unique<obvious::RandomNormalMatching>(trials, epsThresh, sizeControlSet);
  return true;
}

bool RandomNormalMatcher::match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T)
{
  if(!_matcher)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Call init() first " << std::endl);
    return false;
  }
  T = _matcher->match(M, maskM, NULL, S, maskS, obvious::deg2rad(_ranPhiMax), _regTrnsMax, sensor.getAngularResolution());
}