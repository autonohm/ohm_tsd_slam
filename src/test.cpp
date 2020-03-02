#include <ros/ros.h>
#include "registration/TsdfPdfMatcher.h"
#include <tinyxml2.h>

int main(int argc, char** argv)
{
ros::init(argc, argv, "teset");
ros::NodeHandle nh;
tinyxml2::XMLDocument xmlDoc;
tinyxml2::XMLNode* pRoot = xmlDoc.NewElement("tsdf_pdf_config");
xmlDoc.InsertFirstChild(pRoot);
tinyxml2::XMLElement* pElement = xmlDoc.NewElement("IntValue");
pElement->SetText(100);
pRoot->InsertEndChild(pElement);
tinyxml2::XMLElement* pElement2 = xmlDoc.NewElement("FloatValue");
pElement2->SetText(0.15f);
pRoot->InsertEndChild(pElement2);
tinyxml2::XMLError eResult = xmlDoc.SaveFile("SavedData.xml");

TsdPdfMatcher matcher;
return 0;
}