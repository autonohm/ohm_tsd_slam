#include "tools.h"
#include "obcore/math/mathbase.h"

#include <stdio.h>
#include <iostream>

using namespace std;

namespace obvious
{

/**
 * @brief Conversion of RGB image to grayscale image
 * Conversion algorithm taken from OpenCV library
 */
#define FIX(x,n)                 (int)((x)*(1 << (n)) + 0.5)
#define DESCALE(x,n)             ( ( (x) + ( 1 << ((n) - 1) ) ) >> (n))
void rgb2gray(unsigned char* src, unsigned char* dst, unsigned int width, unsigned int height)
{
  // HDTV standard
  // I = 0.212671 * R + 0.715160 * G + 0.072169 * B
  int nMaskR = FIX(0.212671,10);
  int nMaskG = FIX(0.715160,10);
  int nMaskB = FIX(0.072169,10);

  for(unsigned int i=0; i<width*height; ++i)
  {
    int val  = src[0] * nMaskR + src[1] * nMaskG + src[2] * nMaskB;
    dst[i] = DESCALE(val,10);
    src += 3;
  }
}

/**
 * @brief Serialization routine
 * Algorithms taken from openvolksbot library
 */
int serializePPM(const char* szFilename, void* pvBuffer, unsigned int nWidth, unsigned int nHeight, bool bInc)
{

  int nRowInc=0;
  int nRowStart=0;
  unsigned int i=0;
  unsigned int j=0;

  // increasing order
  if (bInc)
  {
    nRowStart = (nWidth * 3 * (nHeight - 1));
    nRowInc = -nWidth * 3;
  }
  FILE *pFile;

  // Open file
  pFile = fopen(szFilename, "wb");
  if (pFile == NULL) return 0;

  // Write header
  fprintf(pFile, "P6\n%d %d\n255\n", nWidth, nHeight);

  // Write pixel data
  unsigned char* buffer = (unsigned char*) pvBuffer;

  /* Go to the end of the buffer */
  buffer += nRowStart;
  for (i=0; i<nHeight; i++)
  {
    for (j=0; j<nWidth; j++)
    {
      putc(buffer[0], pFile); /* b */
      putc(buffer[1], pFile); /* g */
      putc(buffer[2], pFile); /* r */
      buffer += 3;
    }
    /* Move to next line */
    buffer += 2 * nRowInc;
  }

  // Close file
  fclose(pFile);

  return 1;
}

/**
 * @brief Serialization routine
 * Algorithms taken from openvolksbot library
 */
int serializePGM(const char* szFilename, void* pvBuffer, unsigned int nWidth, unsigned int nHeight, bool bInc)
{

  int nRowInc=0;
  int nRowStart=0;
  unsigned int i=0;
  unsigned int j=0;

  // increasing order
  if (bInc)
  {
    nRowStart = (nWidth * (nHeight - 1));
    nRowInc = -nWidth;
  }
  FILE *pFile;

  // Open file
  pFile = fopen(szFilename, "wb");
  if (pFile == NULL) return 0;

  // Write header
  fprintf(pFile, "P5\n%d %d\n255\n", nWidth, nHeight);

  // Write pixel data
  unsigned char* buffer = (unsigned char*) pvBuffer;

  /* Go to the end of the buffer */
  buffer += nRowStart;
  for (i=0; i<nHeight; i++)
  {
    for (j=0; j<nWidth; j++)
    {
      putc(buffer[0], pFile); /* b */
      buffer++;
    }
    /* Move to next line */
    buffer += 2 * nRowInc;
  }

  // Close file
  fclose(pFile);

  return 1;
}

/**
 * @brief Serialization routine
 * Algorithms taken from openvolksbot library
 */
int serializePBM(const char* szFilename, void* pvBuffer, unsigned int nWidth, unsigned int nHeight, bool bInc)
{

  int nRowInc=0;
  int nRowStart=0;
  unsigned int i=0;
  unsigned int j=0;

  // increasing order
  if (bInc)
  {
    nRowStart = (nWidth * (nHeight - 1));
    nRowInc = -nWidth;
  }
  FILE *pFile;

  // Open file
  pFile = fopen(szFilename, "w");
  if (pFile == NULL) return 0;

  // Write header
  fprintf(pFile, "P1\n%d %d\n", nWidth, nHeight);

  // Write pixel data
  unsigned char* buffer = (unsigned char*) pvBuffer;

  /* Go to the end of the buffer */
  buffer += nRowStart;
  for (i=0; i<nHeight; i++)
  {
    for (j=0; j<nWidth; j++)
    {
      if(j>0) fputs(" ", pFile);
      if(*buffer++) fputs("0", pFile);
      else fputs("1", pFile);
    }
    fputs("\n", pFile);
    /* Move to next line */
    buffer += 2 * nRowInc;
  }

  // Close file
  fclose(pFile);

  return 1;
}

/**@brief Load one line from istream source and convert it into double
 *
 * @param source istream reference (ifstream, sstream, ...)
 * @return computed double value
 */
double getDoubleLine(std::istream& source)
{
  std::string lineVar;
  std::getline(source, lineVar);
  if(!lineVar.size())
    return NAN;
  else
  {
    return std::strtod(lineVar.c_str(), NULL);
  }
}

/**@brief Load one line from istream source and convert it into integer
 *
 * @param source istream reference (ifstream, sstream, ...)
 * @return computed integer value
 */
int getIntLine(std::istream& source)
{
  std::string lineVar;
  std::getline(source, lineVar);
  if(!lineVar.size())
    return 0;
  else
    return std::atoi(lineVar.c_str());
}
//#include "jpeglib.h"
//#include <string.h>
//
///**
// *  This defines a new destination manager to store images in memory
// */
//typedef struct {
//  /** public fields */
//  struct jpeg_destination_mgr pub;
//  /** start of buffer */
//  JOCTET *buffer;
//  /** buffer size */
//  int bufsize;
//  /** finale data size */
//  int datacount;
//} memory_destination_mgr;
//
//typedef memory_destination_mgr *mem_dest_ptr;
//
///**
// * Initialize destination --- called by jpeg_start_compress before any data is actually written.
// * @param cinfo Jpeg compression information (include image size, colorspace and more)
// */
//METHODDEF(void) init_destination (j_compress_ptr cinfo)
//{
//  mem_dest_ptr dest = (mem_dest_ptr) cinfo->dest;
//  dest->pub.next_output_byte = dest->buffer;
//  dest->pub.free_in_buffer = dest->bufsize;
//  dest->datacount=0;
//}
//
///**
// * Empty the output buffer --- called whenever buffer fills up.
// * @param cinfo Jpeg compression information (include image size, colorspace and more)
// */
//METHODDEF(boolean) empty_output_buffer (j_compress_ptr cinfo)
//{
//  mem_dest_ptr dest = (mem_dest_ptr) cinfo->dest;
//  dest->pub.next_output_byte = dest->buffer;
//  dest->pub.free_in_buffer = dest->bufsize;
//
//  return TRUE;
//}
//
///**
// * Terminate destination --- called by jpeg_finish_compress after all data has been written.  Usually needs to flush buffer.
// * @param cinfo Jpeg compression information (include image size, colorspace and more)
// */
//METHODDEF(void) term_destination (j_compress_ptr cinfo)
//{
//  /* expose the finale compressed image size */
//  mem_dest_ptr dest = (mem_dest_ptr) cinfo->dest;
//  dest->datacount = dest->bufsize - dest->pub.free_in_buffer;
//}
//
//GLOBAL(void) jpeg_memory_dest(j_compress_ptr cinfo, JOCTET *buffer,int bufsize)
//{
//  mem_dest_ptr dest;
//  if (cinfo->dest == NULL) {    /* first time for this JPEG object? */
//    cinfo->dest = (struct jpeg_destination_mgr *)
//      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
//                                  sizeof(memory_destination_mgr));
//  }
//
//  dest = (mem_dest_ptr) cinfo->dest;
//  dest->bufsize=bufsize;
//  dest->buffer=buffer;
//  dest->pub.init_destination = init_destination;
//  dest->pub.empty_output_buffer = empty_output_buffer;
//  dest->pub.term_destination = term_destination;
//}
//
//int jpeg_compress(char *dst, char *src, int width, int height, int dstsize, int quality)
//{
//  struct jpeg_compress_struct cinfo;
//  struct jpeg_error_mgr jerr;
//  unsigned char *dataRGB = (unsigned char *)src;
//  JSAMPROW row_pointer=(JSAMPROW)dataRGB;
//  JOCTET *jpgbuff;
//  mem_dest_ptr dest;
//  int csize=0;
//
//  /* zero out the compresion info structures and
//     allocate a new compressor handle */
//  memset (&cinfo,0,sizeof(cinfo));
//  cinfo.err = jpeg_std_error(&jerr);
//  jpeg_create_compress(&cinfo);
//
//  /* Setup JPEG datastructures */
//  cinfo.image_width = width;      /* image width and height, in pixels */
//  cinfo.image_height = height;
//  cinfo.input_components = 3;   /* # of color components per pixel=3 RGB */
//  cinfo.in_color_space = JCS_RGB;
//  jpgbuff = (JOCTET*)dst;
//
//  /* Setup compression and do it */
//  jpeg_memory_dest(&cinfo,jpgbuff,dstsize);
//  jpeg_set_defaults(&cinfo);
//  jpeg_set_quality (&cinfo, quality, TRUE);
//  jpeg_start_compress(&cinfo, TRUE);
//  /* compress each scanline one-at-a-time */
//  while (cinfo.next_scanline < cinfo.image_height) {
//    row_pointer = (JSAMPROW)(dataRGB+(cinfo.next_scanline*3*width));
//    jpeg_write_scanlines(&cinfo, &row_pointer, 1);
//  }
//  jpeg_finish_compress(&cinfo);
//  /* Now extract the size of the compressed buffer */
//  dest=(mem_dest_ptr)cinfo.dest;
//  csize=dest->datacount; /* the actual compressed datasize */
//  /* destroy the compressor handle */
//  jpeg_destroy_compress(&cinfo);
//  return csize;
//}

}
