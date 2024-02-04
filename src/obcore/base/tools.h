#ifndef TOOLS_H__
#define TOOLS_H__

#include <istream>

namespace obvious
{

void rgb2gray(unsigned char* src, unsigned char* dst, unsigned int width, unsigned int height);

int serializePPM(const char* szFilename, void* pvBuffer, unsigned int nWidth, unsigned int nHeight, bool bInc = 1);

int serializePGM(const char* szFilename, void* pvBuffer, unsigned int nWidth, unsigned int nHeight, bool bInc);

int serializePBM(const char* szFilename, void* pvBuffer, unsigned int nWidth, unsigned int nHeight, bool bInc = 1);

double getDoubleLine(std::istream& source);

int getIntLine(std::istream& source);

}

#endif
