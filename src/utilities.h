#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "obcore/math/linalg/linalg.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tinyxml2.h>
#include <string>

namespace utilities
{
/**
 * Method to analyze 2D transformation matrix.
 * @param T Pointer to transformation matrix
 * @return Calculated angle
 */
double calcAngle(obvious::Matrix* T);

/**
 * Method to remove certain values in a matrix using a given mask.
 * @param Mat Input matrix data
 * @param mask Value mask
 * @param maskSize Amount of values in the mask
 * @param validPoints Value determining the number of values in the output matrix
 * @return Filtered matrix of type obvious::Matrix
 */
obvious::Matrix maskMatrix(obvious::Matrix* Mat, bool* mask, unsigned int maskSize, unsigned int validPoints);

/**
 * Method to determine whether the localized sensor has been moved significantly (value higher than thresh).
 * A map update is only initiated in case this method returns true.
 * @param lastPose Pointer to last known pose
 * @param curPose Pointer to current pose
 * @return True in case of significant pose change
 */
bool isPoseChangeSignificant(obvious::Matrix* lastPose, obvious::Matrix* curPose, const double threshLin, const double threshRot);

/**
 * Method to convert an 3x3 obvious::Matrix to a tf matrix
 * @param ob obvious::Matrix input to be converted
 * @return tf matrix tf::Transform
 */
tf::Transform obviouslyMatrix3x3ToTf(obvious::Matrix& ob); // TODO: utilities

/**
 * Converts a tf matrix to a 3x3 obvious::Matrix
 * @param tf matrix to be converted
 * @return converted obvious::Matrix
 */
obvious::Matrix tfToObviouslyMatrix3x3(const tf::Transform& tf); // TODO: utilities

const tinyxml2::XMLElement* getTinyxmlChildElement(const std::string& tag, const tinyxml2::XMLElement* rootNode);
bool loadTyniXmlParameter(unsigned int& param, const std::string& tag, tinyxml2::XMLElement& rootNode);
bool loadTyniXmlParameter(int& param, const std::string& tag, tinyxml2::XMLElement& rootNode);
bool loadTyniXmlParameter(float& param, const std::string& tag, tinyxml2::XMLElement& rootNode);
bool loadTyniXmlParameter(double& param, const std::string& tag, tinyxml2::XMLElement& rootNode); //toDO: find a better name than parameter

void loadTinyXmlAttribute(double& param, const std::string& tag, const tinyxml2::XMLElement& element);

} // namespace utilities

#endif