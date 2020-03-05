#include "Registration.h"
#include "TsdfPdfMatcher.h"
#include "obcore/math/linalg/linalg.h"
#include <cmath>
#include <ros/ros.h>
#include <tinyxml2.h>
#include "utilities.h"

Registration::Registration(const RegModes& mode, obvious::TsdGrid& grid, obvious::SensorPolar2D& sensor)
    : _grid(grid)
    , _sensor(sensor)
    , _regMode(mode)
    , _assigner(std::make_unique<obvious::FlannPairAssignment>(2))
    , _filterBounds(std::make_unique<obvious::OutOfBoundsFilter2D>(_grid.getMinX(), _grid.getMaxX(), _grid.getMinY(), _grid.getMaxY()))
    , _filterReciprocal(std::make_unique<obvious::ReciprocalFilter>())
    , _estimator(std::make_unique<obvious::ClosedFormEstimator2D>())
{
  std::cout << __PRETTY_FUNCTION__ << " huhu" << std::endl;
  ros::NodeHandle prvNh("~");
  std::string     configFileXml;
  prvNh.param<std::string>("registration/config_file", configFileXml, "/home/phil/workspace/ros/src/ohm_tsd_slam/config/config_icp.xml");
  if(!configFileXml.size())
    throw "config not found";
  if(!this->init(configFileXml))
    throw "Error initilizing registration module";

  _filterDist = std::make_unique<obvious::DistanceFilter>(_distFilterThreshMin, _distFilterThreshMax, _icpIterations - 10);

  _assigner->addPreFilter(_filterBounds.get());
  _assigner->addPostFilter(_filterDist.get());
  _assigner->addPostFilter(_filterReciprocal.get());

  _icp = std::make_unique<obvious::Icp>(_assigner.get(), _estimator.get());

  _icp->setMaxRMS(_icpThreshRms);
  _icp->setMaxIterations(_icpIterations);
  _icp->setConvergenceCounter(_icpIterations);

  // switch(mode)
  // {
  // case RegModes::ICP:
  //   break;
  // case RegModes::TSD:
  _matcherPre = std::make_unique<TsdPdfMatcher>(_sensor, _grid);

  
  //   break;
  // default:
  //   break;
  //}
}

Registration::~Registration() {}

bool Registration::init(const std::string& configFileXml)
{
  tinyxml2::XMLDocument doc;

  doc.LoadFile(configFileXml.c_str());
  tinyxml2::XMLNode* rootNode = doc.RootElement();

  if(!rootNode)
  {
    ROS_ERROR_STREAM("no node in xml file " << configFileXml << " .");
    return false;
  }
  std::string rootNodeName = rootNode->ToElement()->Name();
  if(rootNodeName != std::string("config_icp"))
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Wrong config file. Root node must be " << std::string("config_icp") << std::endl);
    return false;
  }

  tinyxml2::XMLElement* element = rootNode->FirstChild()->ToElement();
  element                       = rootNode->FirstChildElement("dist_filter_max");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter dist_filter_max " << std::endl);
    return false;
  }
  else
  {
    element->QueryDoubleText(&_distFilterThreshMax);
    std::cout << __PRETTY_FUNCTION__ << " dist_filter_max " << _distFilterThreshMax << std::endl;
  }

  element = rootNode->FirstChildElement("dist_filter_min");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter dist_filter_min " << std::endl);
    return false;
  }
  else
  {
    element->QueryDoubleText(&_distFilterThreshMin);
    std::cout << __PRETTY_FUNCTION__ << " dist_filter_max " << _distFilterThreshMin << std::endl;
  }

  element = rootNode->FirstChildElement("icp_iterations");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter icp_iterations " << std::endl);
    return false;
  }
  else
  {
    element->QueryUnsignedText(&_icpIterations);
    std::cout << __PRETTY_FUNCTION__ << " icp_iterations " << _icpIterations << std::endl;
  }

  element = rootNode->FirstChildElement("icp_thresh_rms");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter icp_thresh_rms " << std::endl);
    return false;
  }
  else
  {
    element->QueryDoubleText(&_icpThreshRms);
    std::cout << __PRETTY_FUNCTION__ << " icp_iterations " << _icpThreshRms << std::endl;
  }

  element = rootNode->FirstChildElement("thresh_error_ang");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter thresh_error_ang " << std::endl);
    return false;
  }
  else
  {
    element->QueryDoubleText(&_threshErrorAng);
    std::cout << __PRETTY_FUNCTION__ << " thresh_error_ang " << _threshErrorAng << std::endl;
  }

  element = rootNode->FirstChildElement("thresh_error_lin");
  if(!element)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error parameter thresh_error_lin " << std::endl);
    return false;
  }
  else
  {
    element->QueryDoubleText(&_threshErrorLin);
    std::cout << __PRETTY_FUNCTION__ << " thresh_error_lin " << _threshErrorLin << std::endl;
  }

  return true;
}

bool Registration::doRegistration(obvious::Matrix& T, double* coordsModel, double* normalsModel, bool* maskModel, const unsigned int nPointsModel,
                                  double* coordsScene, bool* maskScene)
{

  const unsigned int measurementSize = _sensor.getRealMeasurementSize();

 /**
     * Create Point Matrices with structure [x1 y1; x2 y2; ..]
     * M, N and S are matrices that preserve the ray model of a laser scanner
     * Xvalid matrices are matrices that do not preserve the ray model but contain only valid points (mask applied)
     * T transformation
     */
  obvious::Matrix M(measurementSize, 2, coordsModel);
  obvious::Matrix N(measurementSize, 2, normalsModel);
  obvious::Matrix S(measurementSize, 2, coordsScene);
  obvious::Matrix Mvalid = utilities::maskMatrix(&M, maskModel, measurementSize, nPointsModel);
  obvious::Matrix Nvalid = utilities::maskMatrix(&N, maskModel, measurementSize, nPointsModel);
  obvious::Matrix Svalid = utilities::maskMatrix(&S, maskModel, measurementSize, measurementSize);

  obvious::Matrix T44(4, 4);
  T44.setIdentity();
  obvious::Matrix Ttemp(3, 3);
  Ttemp.setIdentity();

  // RANSAC pre-registration (rough)
  if(_regMode != RegModes::ICP)
  {
    _matcherPre->match(_sensor, &M, maskModel, &S, maskScene, Ttemp);
    T44(0, 0) = Ttemp(0, 0);
    T44(0, 1) = Ttemp(0, 1);
    T44(0, 3) = Ttemp(0, 2);
    T44(1, 0) = Ttemp(1, 0);
    T44(1, 1) = Ttemp(1, 1);
    T44(1, 3) = Ttemp(1, 2);
  }

  _icp->reset();
  obvious::Matrix P = _sensor.getTransformation();
  _filterBounds->setPose(&P);

  _icp->setModel(&Mvalid, &Nvalid);
  _icp->setScene(&Svalid);
  double       rms   = 0.0;
  unsigned int pairs = 0;
  unsigned int it    = 0;
  _icp->iterate(&rms, &pairs, &it, &T44);
  T = _icp->getFinalTransformation();

  const bool regErrorT = isRegistrationError(&T, _threshErrorLin, _threshErrorAng);
  
  return regErrorT;
}

// obvious::Matrix Registration::maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints)
// {
//   assert(Mat->getRows() == maskSize);
//   assert(Mat->getCols() == 2);
//   obvious::Matrix retMat(validPoints, 2);
//   unsigned int    cnt = 0;

//   for(unsigned int i = 0; i < maskSize; i++)
//   {
//     if(mask[i])
//     {
//       retMat(cnt, 0) = (*Mat)(i, 0);
//       retMat(cnt, 1) = (*Mat)(i, 1);
//       cnt++;
//     }
//   }
//   return retMat;
// }

bool Registration::isRegistrationError(obvious::Matrix* T, const double trnsMax, const double rotMax)
{
  const double deltaX   = (*T)(0, 2);
  const double deltaY   = (*T)(1, 2);
  const double trnsAbs  = std::sqrt(deltaX * deltaX + deltaY * deltaY);
  const double deltaPhi = utilities::calcAngle(T);
  return (trnsAbs > trnsMax) || (std::abs(std::sin(deltaPhi)) > rotMax);
}

// double Registration::calcAngle(obvious::Matrix* T)
// {
//   double       angle    = 0.0;
//   const double ARCSIN   = std::asin((*T)(1, 0));
//   const double ARCSINEG = std::asin((*T)(0, 1));
//   const double ARCOS    = std::acos((*T)(0, 0));
//   if((ARCSIN > 0.0) && (ARCSINEG < 0.0))
//     angle = ARCOS;
//   else if((ARCSIN < 0.0) && (ARCSINEG > 0.0))
//     angle = 2.0 * M_PI - ARCOS;
//   return (angle);
// }