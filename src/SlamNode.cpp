/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#include "SlamNode.h"
#include "ThreadGrid.h"
#include "ThreadMapping.h"
#include "obcore/math/mathbase.h"
#include <tinyxml2.h>

namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  ros::NodeHandle prvNh("~");
  int             iVar             = 0;
  double          truncationRadius = 0.0;
  double          cellSize         = 0.0;
  unsigned int    octaveFactor     = 0;
  double          xOffset          = 0.0;
  double          yOffset          = 0.0;

  prvNh.param<double>("x_offset", xOffset, 0.0);
  prvNh.param<double>("y_offset", yOffset, 0.0);
  prvNh.param<int>("map_size", iVar, 10);
  octaveFactor = static_cast<unsigned int>(iVar);
  prvNh.param<double>("cellsize", cellSize, 0.025);
  prvNh.param<int>("truncation_radius", iVar, 3);
  truncationRadius = static_cast<double>(iVar);

  if(octaveFactor > 15)
  {
    ROS_ERROR_STREAM("Error! Unknown map size -> set to default (10)!" << std::endl);
    octaveFactor = 10;
  }
  // instanciate representation
  _grid = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(octaveFactor)); // obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellSize);
  unsigned int cellsPerSide = pow(2, octaveFactor);
  double       sideLength   = static_cast<double>(cellsPerSide) * cellSize;
  ROS_INFO_STREAM("Creating representation with " << cellsPerSide << "x" << cellsPerSide << "cells, representating " << sideLength << "x" << sideLength
                                                  << "m^2" << std::endl);
  // instanciate mapping threads
  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid, &_nh, xOffset, yOffset);

  // std::string nameSpace = "";
  std::string fileConfigXMLSLam;
  prvNh.param<std::string>("file_config_xml_slam", fileConfigXMLSLam, "/home/phil/workspace/ros/src/ohm_tsd_slam/config/config_slam.xml");
  tinyxml2::XMLDocument doc;
  doc.LoadFile(fileConfigXMLSLam.c_str());

  tinyxml2::XMLNode* rootNode = doc.RootElement();

  if(!rootNode)
  {
    ROS_ERROR_STREAM("no node in xml file: " << fileConfigXMLSLam.c_str());
    throw "Invalid SLAM config file";
  }
  std::string rootNodeName = rootNode->ToElement()->Name();
  if(rootNodeName != std::string("config_slam"))
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Wrong config file. Root node must be " << std::string("config_slam") << " " << (rootNodeName)
                                         << std::endl);
    throw "Invalid SLAM config file";
  }

  for(tinyxml2::XMLNode* node = doc.FirstChild()->NextSibling()->FirstChild(); node; node = node->NextSibling())
  {
    tinyxml2::XMLElement* element = node->ToElement();

    if(!element)
      continue;
    if(element->Name() == std::string("robot"))
    {
      _localizers.push_back(new ThreadLocalize(_grid, _threadMapping, element->DoubleAttribute("offset_x"), element->DoubleAttribute("offset_y"),
                                                element->Attribute("namespace")));
    }
    else
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " unknown tag detagted ;-) " << element->Name() << std::endl);
      continue;
    }
  }
  if(_localizers.size() > 1)
  {
    ROS_INFO_STREAM("Multi SLAM started with robots :");
    for(auto& iter : _localizers)
      std::cout << " " << iter->nameSpace() << " pos(delta) (" << iter->offsetInitial().x() << "/" << iter->offsetInitial().y() << ")" << std::endl;
  }
  else if(_localizers.size() == 1)
    ROS_INFO_STREAM("Single SLAM started");
  else
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " no robots found. Check config file ");
}

SlamNode::~SlamNode()
{
  // stop all localization threads
  for(std::vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
  // stop mapping threads
  _threadGrid->terminateThread();
  while(_threadGrid->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadGrid;
  _threadMapping->terminateThread();
  while(_threadMapping->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadMapping;
  delete _grid;
}

void SlamNode::run(void)
{
  ROS_INFO_STREAM("Waiting for first laser scan to start node...");
  ros::spin();
}

} /* namespace ohm_tsd_slam */
