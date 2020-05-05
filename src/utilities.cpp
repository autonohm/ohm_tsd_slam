#include "utilities.h"
#include <assert.h>
#include <cmath>
#include <ros/ros.h>

namespace utilities
{

double calcAngle(obvious::Matrix* T)
{
  double       angle    = 0.0;
  const double ARCSIN   = std::asin((*T)(1, 0));
  const double ARCSINEG = std::asin((*T)(0, 1));
  const double ARCOS    = std::acos((*T)(0, 0));
  if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
    angle = ARCOS;
  else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
    angle = 2.0 * M_PI - ARCOS;
  return (angle);
}

bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose, const double threshLin, const double threshRot)
{
  const double deltaX   = (*curPose)(0, 2) - (*lastPose)(0, 2);
  const double deltaY   = (*curPose)(1, 2) - (*lastPose)(1, 2);
  double       deltaPhi = calcAngle(curPose) - calcAngle(lastPose);
  deltaPhi              = fabs(sin(deltaPhi));
  const double trnsAbs  = sqrt(deltaX * deltaX + deltaY * deltaY);
  return (deltaPhi > threshRot || trnsAbs > threshLin); // todo muss da keine klammer drum
}

obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints)
{
  assert(Mat->getRows() == maskSize);
  assert(Mat->getCols() == 2);
  obvious::Matrix retMat(validPoints, 2);
  unsigned int    cnt = 0;

  for(unsigned int i = 0; i < maskSize; i++)
  {
    if(mask[i])
    {
      retMat(cnt, 0) = (*Mat)(i, 0);
      retMat(cnt, 1) = (*Mat)(i, 1);
      cnt++;
    }
  }
  return retMat;
}

tf::Transform obviouslyMatrix3x3ToTf(obvious::Matrix& ob)
{
  tf::Transform tf;
  tf.setOrigin(tf::Vector3(ob(0, 2), ob(1, 2), 0.0));
  tf.setRotation(tf::createQuaternionFromYaw(asin(ob(0, 1))));
  return tf;
}

obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf)
{
  obvious::Matrix ob(3, 3);
  ob.setIdentity();

  double theta = tf::getYaw(tf.getRotation());
  double x     = tf.getOrigin().getX();
  double y     = tf.getOrigin().getY();

  ob(0, 0) = std::cos(theta) + 0.0;
  ob(0, 1) = -std::sin(theta) + 0.0;
  ob(0, 2) = x + 0.0;
  ob(1, 0) = std::sin(theta) + 0.0;
  ob(1, 1) = std::cos(theta) + 0.0;
  ob(1, 2) = y + 0.0;

  return ob;
}

const tinyxml2::XMLElement* getTinyxmlChildElement(const std::string& tag, const tinyxml2::XMLElement* rootNode)
{
  const tinyxml2::XMLElement* element = rootNode->FirstChildElement(tag.c_str());
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error malformed config. local_offset is missing");
    throw "Invalid robot config file";
  }
  return element;
}

bool loadTyniXmlParameter(unsigned int& param, const std::string& tag, tinyxml2::XMLElement& rootNode)
{
  tinyxml2::XMLElement* element = rootNode.FirstChildElement(tag.c_str());
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter " << tag << std::endl);
    return false;
  }
    int val = 0;
    element->QueryUnsignedText(&param);
    std::cout << __PRETTY_FUNCTION__ << " " << tag << " : " << param << std::endl;
    return true;
}

bool loadTyniXmlParameter(int& param, const std::string& tag, tinyxml2::XMLElement& rootNode)
{
  tinyxml2::XMLElement* element = rootNode.FirstChildElement(tag.c_str());
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter " << tag << std::endl);
    return false;
  }
  element->QueryIntText(&param);
  std::cout << __PRETTY_FUNCTION__ << " " << tag << " : " << param << std::endl;
  return true;
}
bool loadTyniXmlParameter(float& param, const std::string& tag, tinyxml2::XMLElement& rootNode)
{
  tinyxml2::XMLElement* element = rootNode.FirstChildElement(tag.c_str());
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter " << tag << std::endl);
    return false;
  }
  element->QueryFloatText(&param);
  std::cout << __PRETTY_FUNCTION__ << " " << tag << " : " << param << std::endl;
  return true;
}
bool loadTyniXmlParameter(double& param, const std::string& tag, tinyxml2::XMLElement& rootNode)
{
  tinyxml2::XMLElement* element = rootNode.FirstChildElement(tag.c_str());
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter " << tag << std::endl);
    return false;
  }
  element->QueryDoubleText(&param);
  std::cout << __PRETTY_FUNCTION__ << " " << tag << " : " << param << std::endl;
  return true;
}

void loadTinyXmlAttribute(double& param, const std::string& tag, const tinyxml2::XMLElement& element)
{
  tinyxml2::XMLError result = element.QueryDoubleAttribute(tag.c_str(), &param);
  if(result != tinyxml2::XML_SUCCESS)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__  << " error malformed config. " << element.Value() << "->" << tag << " loading failed ");
    throw "Invalid robot config file";
  }
  else
    ROS_INFO_STREAM(__PRETTY_FUNCTION__<< tag << "  " << param);
}

} // namespace utilities