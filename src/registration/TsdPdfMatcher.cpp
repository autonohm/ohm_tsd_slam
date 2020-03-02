#include "TsdfPdfMatcher.h"
#include <ros/ros.h>
TsdPdfMatcher::TsdPdfMatcher()
    : _trials(0)
    , _sizeControlSet(0)
    , _epsThresh(0.0)
    , _zrand(0.0)
{
  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>("tsd_pdf/config_file", _configXml, "/home/phil/workspace/ros/src/ohm_tsd_slam/config/config_tsdf_pdf_matcher.xml");
  if(!_configXml.size())
    throw "config not found";
  this->init(_configXml);
}
TsdPdfMatcher::~TsdPdfMatcher() {}

bool TsdPdfMatcher::init(const std::string& configXml)
{

  tinyxml2::XMLDocument doc;

  doc.LoadFile(configXml.c_str());
  tinyxml2::XMLNode* rootNode = doc.RootElement();

  if(!rootNode)
  {
    ROS_ERROR_STREAM("no node in xml file " << _configXml << " .");
    return false;
  }
  std::string rootNodeName = rootNode->ToElement()->Name();
  if(rootNodeName != std::string("tsdf_pdf_config"))
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Wrong config file. Root node must be " << std::string("tsdf_pdf_config") << std::endl);
    return false;
  }

  tinyxml2::XMLElement* element = rootNode->FirstChild()->ToElement();
  element                       = rootNode->FirstChildElement("trials");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter trials " << std::endl);
    return false;
  }
  else
  {
    int val = 0;
    element->QueryIntText(&val);
    std::cout << __PRETTY_FUNCTION__ << " trials " << val << std::endl;
  }
  element = rootNode->FirstChildElement("epsthresh");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error reading paramer epsthresh " << std::endl);
    return false;
  }
  else
  {
    float val = 0;
    element->QueryFloatText(&val);
    std::cout << __PRETTY_FUNCTION__ << " epsthresh " << val << std::endl;
  }
  element = rootNode->FirstChildElement("sizeControlSet");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "error reading paramer sizeControlSet " << std::endl);
    return false;
  }
  else
  {
    int val = 0;
    element->QueryIntText(&val);
    std::cout << __PRETTY_FUNCTION__ << " sizeControlSet " << val << std::endl;
  }
  element = rootNode->FirstChildElement("zrand");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error reading paramer zrand " << std::endl);
    return false;
  }
  else
  {
    float val = 0;
    element->QueryFloatText(&val);
    std::cout << __PRETTY_FUNCTION__ << " zrand " << val << std::endl;
  }
  return true;
}
bool TsdPdfMatcher::doRegistration(obvious::SensorPolar2D* sensor, obvious::Matrix* M, obvious::Matrix* Mvalid, obvious::Matrix* N, obvious::Matrix* Nvalid,
                                   obvious::Matrix* S, obvious::Matrix* Svalid, obvious::Matrix& T)
{

T = _PDFMatcher->match(M, _maskM, NULL, S, _maskS, obvious::deg2rad(_ranPhiMax), _trnsMax, sensor->getAngularResolution());

}