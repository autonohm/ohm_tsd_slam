/**
 * Definition of basic mathematics
 * This header file collects methods for convenience. Some methods are adopted from other libraries (see comments below).
 * @author Stefan May
 * @date 22.08.2012
 */

#ifndef OBVIOUSMATHBASE_H
#define OBVIOUSMATHBASE_H

#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;

#ifdef WIN32
#	ifndef M_PI
#		define M_PI        3.14159265358979323846
#	endif
#	ifdef max
#		undef max
#	endif
#	ifdef min
#		undef min
#	endif
#	define NOMINMAX
#endif

namespace obvious {

  /**
   * @function max
   * @brief maximum template function
   * @return maximum of a or b
   **/
  template <class T>
  static inline T max(const T a, const T b)
  {
    return ((a >= b) ? a : b);
  }

  /**
   * @function min
   * @brief minimumn template function
   * @return minimum of a or b
   **/
  template <class T>
  static inline T min(const T a, const T b)
  {
    return ( (a <= b) ? a : b );
  }

  template <class T>
  static inline void minmaxArray(const T* a, const unsigned int size, T* min, T* max)
  {
    *min = *max = a[0];
    for(unsigned int i=1; i<size; i++)
    {
      if(*min > a[i]) *min = a[i];
      else if (*max < a[i]) *max = a[i];
    }
  }

  template <class T>
  void sort2( T* v )
  {
      if ( v[0] > v[1] )
          swap( v[0], v[1] );
  }

  template <class T>
  void sort3( T* v )
  {
      if ( v[0] > v[1] )  swap( v[0], v[1] );
      if ( v[0] > v[2] )  swap( v[0], v[2] );
      if ( v[1] > v[2] )  swap( v[1], v[2] );
  }

  template <class T>
  void sort4( T* v )
  {
      if ( v[0] > v[1] )  swap( v[0], v[1] );
      if ( v[0] > v[2] )  swap( v[0], v[2] );
      if ( v[0] > v[3] )  swap( v[0], v[3] );
      sort3( &v[1] );
  }


  /**
   * @function abs2D
   * @brief template function calculating length of 2D vector
   * @param a 2D vector
   * @return length
   */
  template <class T>
  static inline T abs2D(T a[2])
  {
    T abs=0;
    for(unsigned int i=0; i<2; i++)
    {
        abs+=a[i]*a[i];
    }
    return(sqrt(abs));
  }

  /**
   * @function abs3D
   * @brief template function calculating length of 3D vector
   * @param a 3D vector
   * @return length
   */
  template <class T>
  static inline T abs3D(T a[3])
  {
    T abs=0;
    for(unsigned int i=0; i<3; i++)
    {
        abs+=a[i]*a[i];
    }
    return(sqrt(abs));
  }

  /**
   * Calculate the l2-norm of an arbitrarily sized vector
   * @param coords coordinate vector
   * @param size size of coordinate vector
   * @return l2-norm
   */
  template <class T>
  inline float l2Norm(T* coords, int size)
  {
    T sqr = coords[0] * coords[0];
    for(int i=1; i<size; i++)
    {
      sqr += coords[i] * coords[i];
    }
    return sqrt(sqr);
  }

  /**
   * @function Dist2_2D
   * @return Squared distance between two points in 2D space
   **/
  template <class T>
  static inline T distSqr2D(const T *x1, const T *x2)
  {
    T dx = x2[0] - x1[0];
    T dy = x2[1] - x1[1];

    return dx*dx + dy*dy;
  }

  /**
   * @function Dist2
   * @return Squared distance between two points in 3D space
   **/
  template <class T>
  static inline T distSqr3D(const T *x1, const T *x2)
  {
    T dx = x2[0] - x1[0];
    T dy = x2[1] - x1[1];
    T dz = x2[2] - x1[2];

    return dx*dx + dy*dy + dz*dz;
  }

  /**
   * @function rad
   * @brief degree to rad conversion
   * @param deg degree value
   * @return rad rad value
   **/
  static inline double deg2rad(const double deg)
  {
    return ( (M_PI * deg) / 180.0 );
  }

  /**
   * @function deg
   * @brief rad to degree conversion
   * @param rad rad value
   * @return deg degree value
   **/
  static inline double rad2deg(const double rad)
  {
    return ( (rad * 180.0) / M_PI );
  }

  template <class T>
  static inline void cross3(T* n, const T* u, const T* v)
  {
    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];
  }

  template <class T>
  static inline T dot2(const T* u, const T* v)
  {
    return u[0]*v[0] + u[1]*v[1];
  }

  template <class T>
  static inline T dot3(const T* u, const T* v)
  {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
  }

  template <class T>
  static inline void norm2(T* n)
  {
    T len = sqrt(n[0]*n[0] + n[1]*n[1]);
    if(fabs(len)<=10e-6) return;
    n[0] /= len;
    n[1] /= len;
  }

  template <class T>
  static inline void norm3(T* n)
  {
    T len = sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    if(fabs(len)<=10e-6) return;
    n[0] /= len;
    n[1] /= len;
    n[2] /= len;
  }

  /**
   * Shift angle to [-PI, PI] range
   * @param dAngle rad angle with range [-2*PI, 2*PI]
   * @return rad angle with valid range
   */
  static inline double getPiCorrectedAngle(const double angle) {
    double correctedAngle = angle;
    if (correctedAngle < -M_PI)
      correctedAngle += 2*M_PI;
    else if (correctedAngle > M_PI)
      correctedAngle -= 2*M_PI;
    return correctedAngle;
  }

  /**
   * Create lookup table for sqare root function (speedup purpose)
   * @param lut pointer to lookup table (must already be instanciated)
   * @param size size of lookup table
   */
  template<class T>
  void createSqRtLut(T* lut, T size)
  {
    for(T t=0; t<size; ++t)
      lut[t] = (T)( sqrt((float)t) + 0.5 );
  }

  // CubeRoot implementation from http://www.worldserver.com/turk/computergraphics/CubeRoot.pdf
  // The OpenCV improvements are included.
  // Note: Determine, if this function is faster than pow! Some compiler make better code for the pow function.
  typedef union obvious32suf
  {
    int i;
    unsigned u;
    float f;
  }
  obvious32suf;

  inline float cbRt24bit(float value)
  {
    float fr;
    obvious32suf v, m;
    int ix, s;
    int ex, shx;

    v.f = value;
    ix = v.i & 0x7fffffff;
    s = v.i & 0x80000000;
    ex = (ix >> 23) - 127;
    shx = ex % 3;
    shx -= shx >= 0 ? 3 : 0;
    ex = (ex - shx) / 3; /* exponent of cube root */
    v.i = (ix & ((1<<23)-1)) | ((shx + 127)<<23);
    fr = v.f;

    /* 0.125 <= fr < 1.0 */
    /* Use quartic rational polynomial with error < 2^(-24) */
    fr = ((((45.2548339756803022511987494 * fr +
        192.2798368355061050458134625) * fr +
        119.1654824285581628956914143) * fr +
        13.43250139086239872172837314) * fr +
        0.1636161226585754240958355063)
        /
        ((((14.80884093219134573786480845 * fr +
            151.9714051044435648658557668) * fr +
            168.5254414101568283957668343) * fr +
            33.9905941350215598754191872) * fr +
            1.0);

    /* fr *= 2^ex * sign */
    m.f = value;
    v.f = fr;
    v.i = (v.i + (ex << 23) + s) & (m.i*2 != 0 ? -1 : 0);
    return v.f;
  }

  inline float cbRt6bit(float value)
  {
    float fr;
    obvious32suf v, m;
    int ix, s;
    int ex, shx;

    v.f = value;
    ix = v.i & 0x7fffffff;
    s = v.i & 0x80000000;
    ex = (ix >> 23) - 127;
    shx = ex % 3;
    shx -= shx >= 0 ? 3 : 0;
    ex = (ex - shx) / 3; /* exponent of cube root */
    v.i = (ix & ((1<<23)-1)) | ((shx + 127)<<23);
    fr = v.f;

    /* 0.125 <= fr < 1.0 */
    /* Use polynomial with error < 2^(-6) */
    fr = -0.46946116 * fr * fr +1.072302 * fr + 0.3812513;

    /* fr *= 2^ex * sign */
    m.f = value;
    v.f = fr;
    v.i = (v.i + (ex << 23) + s) & (m.i*2 != 0 ? -1 : 0);
    return v.f;
  }

  /**
   * Cube root calculation based on an approximation (see upper)
   * @param value input value
   * @return cube root of input value
   */
  inline float cubeRoot(float value)
  {
    return cbRt24bit(value);
  }

  /**
   * Calculate the squared Euklidean distance between two n-dimensional points
   * @param coords1 first coordinate vector
   * @param coords2 second coordinate vector
   * @param size size of coordinate vectors (must both be equally sized)
   * @return squared Euklidean distance
   */
  inline float euklidianDistanceSqr(float* coords1, float* coords2, int size)
  {
    float sqr = 0.0f;
    for(int i=0; i<size; i++)
      {
        float tmp = coords1[i] - coords2[i];
        sqr += tmp * tmp;
      }
    return sqr;
  }

  /**
   * Calculate the Euklidean distance between two n-dimensional points
   * @param coords1 first coordinate vector
   * @param coords2 second coordinate vector
   * @param size size of coordinate vectors (must both be equally sized)
   * @return Euklidean distance
   */
  template <class T>
  inline T euklideanDistance(T* coords1, T* coords2, int size)
  {
	  T sqr = 0.0;
    for(int i=0; i<size; i++)
    {
      T tmp = coords1[i] - coords2[i];
      sqr += tmp * tmp;
    }
	  return sqrt(sqr);
	}

 /* double euklideanDistanceVecD(vector<double>::iterator *vec1,vector<double>::iterator *vec2)
  {
	  double abs=0;
	  vector<double>::iterator iter1=*vec1;
	  if(!vec2)
	  {
		  for(unsigned int i=0;i<3;i++)
		  	{
		  		abs+=*iter1*(*iter1);
		  		iter1++;
		  	}
		}
	  return(sqrt(abs));
  }*/

  /**
   * Calculate the L1 distance between two n-dimensional points
   * @param pfCoords1 first coordinate vector
   * @param pfCoords2 second coordinate vector
   * @param size size of coordinate vectors (must both be equally sized)
   * @return L1 distance
   */
  inline float l1Distance(float* coords1, float* coords2, int size)
  {
    float dist = 0.0f;
    for(int i=0; i<size; i++)
      {
        float tmp = coords1[i] - coords2[i];
        dist += fabs(tmp);
      }
    return dist;
  }

  /**
   * Calculates the empirical correlation coefficient of two data vectors
   * @param features1 first data vector
   * @param features2 second data vector
   * @param size size of data vectors (must be be equally sized)
   * @return empirical correlation coefficient
   */
  inline float empiricalCorrelation(float* features1, float* features2, int size)
  {
    if(size < 1) return 0.0f;
    if(size == 1)
      {
        if((features2[0] == 0.0f) && (features1[0] == 0.0f)) return 1.0f;

        if((features2[0] == 0.0f) || (features1[0] == 0.0f)) return 0.0f;

        float ratio = features1[0] / features2[0];
        if(ratio > 1.0f) ratio = 1.0f / ratio;
        return ratio;
      }

    // calculate the empirical correlation coefficient between the feature vectors of both pixels
    float x = 0.0f;
    float y = 0.0f;
    float xx = 0.0f;
    float yy = 0.0f;
    float xy = 0.0f;

    float fSize = (float) size;

    for(int i=0; i<size; ++i)
      {
        float xTmp = features1[i];
        float yTmp = features2[i];;
        // Zero values are not significant
        if((xTmp > 0.0f)||(yTmp > 0.0f))
          {
            x += xTmp;
            y += yTmp;
            xx += xTmp * xTmp;
            yy += yTmp * yTmp;
            xy += xTmp * yTmp;
          }
        else
          fSize -= 1.0f;
      }

    // No significant values passed
    if(fSize < 1.0f) return 0.0f;

    float corr;
    if(fSize == 1.0f)
      {
        if((y == 0.0f) && (x == 0.0f)) return 1.0f;

        if((y == 0.0f) || (x == 0.0f)) return 0.0f;

        corr = x / y;

        if(corr > 1.0f) corr = 1.0f / corr;
      }
    else
      {
        corr = xy - 1.0f/fSize * x * y;
        float fArg = (xx - 1.0f/fSize * x*x) * ( yy - 1.0f/fSize * y*y);

        if(fArg<0.1)
          corr = 0.0f;
        else
          corr /= sqrt( fArg );
      }

    return corr;
  }

  /**
   * @function  Calculates angle between two vectors
   * @param     vec1 vector one
   * @param     vec2 vector two
   * @return    smallest angle between vectors
   */
  template <class T>
  inline double getAngleBetweenVec(const T* vec1, const T* vec2)
  {
    T scalar = dot3<T>(vec1, vec2);
    T v1[3] = {vec1[0], vec1[1], vec1[2]};
    T v2[3] = {vec2[0], vec2[1], vec2[2]};
    T absValue = abs3D<double>(v1) * abs3D<double>(v2);
    double angle    = (double)acos(scalar / absValue);
    return(angle);
  }

} // namespace

#endif //OBVIOUSMATHBASE_H
