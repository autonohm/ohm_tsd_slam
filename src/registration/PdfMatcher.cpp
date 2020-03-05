#include "PdfMatcher.h"
#include <tinyxml2.h>
#include <ros/ros.h>


PdfMatcher::PdfMatcher(obvious::SensorPolar2D& sensor):
_sensor(&sensor)
{

}

PdfMatcher::~PdfMatcher()
{

}

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
    element->QueryIntText(&_trials);
    std::cout << __PRETTY_FUNCTION__ << " trials " << val << std::endl;
  }
}

bool PdfMatcher::match(obvious::SensorPolar2D& sensor, obvious::Matrix* M, bool* maskM, obvious::Matrix* S, bool* maskS, obvious::Matrix& T)
{

}