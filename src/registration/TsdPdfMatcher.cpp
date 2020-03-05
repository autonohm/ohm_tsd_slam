#include "TsdfPdfMatcher.h"
#include "utilities.h"
#include <ros/ros.h>
TsdPdfMatcher::TsdPdfMatcher(obvious::SensorPolar2D& sensor, obvious::TsdGrid& grid)
    : _trials(0)
    , _sizeControlSet(0)
    , _epsThresh(0.0)
    , _zrand(0.0)
    , _sensor(&sensor)
    , _grid(grid)
{
  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>("tsd_pdf/config_file", _configXml,
                           "/home/phil/workspace/ros/src/ohm_tsd_slam/config/"
                           "config_tsdf_pdf_matcher.xml");
  if(!_configXml.size())
    throw "config not found";
  if(!this->init(_configXml))
    throw "config invalid";
}
TsdPdfMatcher::~TsdPdfMatcher()
{
  // delete _maskM;
  // delete _maskM;
}

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
  if(!utilities::loadTyniXmlParameter(_trials, std::string("trials"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_epsThresh, std::string("epsthresh"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_sizeControlSet, std::string("sizeControlSet"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_zrand, std::string("zrand"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_ranPhiMax, std::string("ranphimax"), *rootNode))
    return false;
  if(!utilities::loadTyniXmlParameter(_transMax, std::string("reg_trs_max"), *rootNode))
    return false;

  _matcher = std::make_unique<obvious::TSD_PDFMatching>(_grid, _trials, _epsThresh, _sizeControlSet, _zrand);
  return true;
}
bool TsdPdfMatcher::match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T)
// bool match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, obvious::Matrix* S, obvious::Matrix& T)
{

  if(!_matcher)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " matcher uninitialized call init first " << std::endl);
    return false;
  }
  obvious::Matrix Ttemp =
      _matcher->match(sensor.getTransformation(), M, maskM, NULL, S, maskS, obvious::deg2rad(_ranPhiMax), _transMax, sensor.getAngularResolution());
}