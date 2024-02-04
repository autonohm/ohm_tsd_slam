#ifndef CARTESIANCLOUD_H
#define CARTESIANCLOUD_H

#include <vector>
#include <map>
#include <math.h>
#include <string>

#include "obcore/math/linalg/linalg.h"

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

  enum EnumPointAttribute { ePointAttrValid = 0x1};

  enum EnumSourceInfo{eSOURCEROWS=0, eSOURCECOLS=1};

  /**
   * @class CartesianCloud3D
   * @brief Represents a cloud of points in 3D space
   * @author Stefan May
   **/
  class CartesianCloud3D
  {
  public:
    /**
     * Constructor;
     * @param size Number of points in the cloud
     */
    CartesianCloud3D(unsigned int size, double* coords, unsigned char* rgb, double* normals);

    /**
     * Constructor;
     */
    CartesianCloud3D(unsigned int size, bool withInfo = false);

    /**
     * Destructor
     */
    ~CartesianCloud3D();

    /**
     * Copy constructor
     */
    CartesianCloud3D(CartesianCloud3D* cloud);

    /**
     * Accessor to each point via array indices.
     * @param i point index
     */
    //double* operator [] (unsigned int i);

    /**
     * Get access to array of coordinates
     * @return pointer to matrix instance
     */
    Matrix* getCoords();

    void setNormals(Matrix* normals);

    Matrix* getNormals();

    /**
     * Get access to array of colors
     * @return pointer to matrix instance
     */
    unsigned char* getColors();

    /**
     * Get access to array of attributes
     * @return pointer pointer to array of attributes
     */
    int* getAttributes();

    /**
     * Get access to array of indices. Indices are related to the time of instantiation.
     * The cloud might be modified through a method, e.g., removeInvalidPoints, though, indices
     * are not necessarily in ascending order.
     * @return pointer pointer to array of indices
     */
    int* getIndices();

    /**
     * Shows presence of additional point info
     * @return flag of presence
     */
    int hasInfo();

    /**
     * Shows, whether the cloud has normals or not
     * @return flag
     */
    int hasNormals(void)const{return(_hasNormals);}

    /**
     * Shows, whether the cloud has colors or not
     * @return flag
     */
    bool hasColors(void)const{return(_hasColors);}

    /**
     * Add additional source information
     * @param eSourceInfo source information identifier
     * @param lValue the info
     */
    void addSourceInfo(EnumSourceInfo eSourceInfo, long lValue);

    /**
     * Shows presence of additional source information. This is e.g. a row and column width.
     * @return flag of presence
     */
    int hasSourceInfo();

    /**
     * Accessor to additional source information
     * @param eSourceInfo source information identifier
     * @param plValue the info
     * @return success
     */
    int getSourceInfo(EnumSourceInfo eSourceInfo, long* plValue);

    /**
     * Clear source info map
     */
    void clearSourceInfo();

    void maskPoints(bool* mask);
    void maskEmptyNormals();

    /**
     * Remove points which have an invalid flag in attributes
     */
    void removeInvalidPoints();

    /**
     * Sub-sample point cloud
     * @param step subsampling step
     */
    void subsample(unsigned int step);

    /**
     * Get size of CartesianCloud
     * @return number of points
     */
    unsigned int size();

    /**
     * Transform the point cloud, i.e. translate and/or rotate it
     * @param matrix a 4x4 translation matrix
     */
    void transform(Matrix* T);
    void transform(double T[16]);

    /**
     * Create a perspective projection depending on a projection matrix
     * @param pImage projective image
     * @param pMask mask to be considered
     * @param P point coordinates
     * @param nW width of image
     * @param nH height of image
     */
    void createProjection(unsigned char* pImage, unsigned char* pMask, Matrix* P, int nW, int nH);

    /**
     * Create a z-Buffer depending on a projection matrix
     * @param pImage projective image
     * @param zbuffer z-Buffer
     * @param P point coordinates
     * @param nW width of image
     * @param nH height of image
     */
    void createZBuffer(unsigned char* pImage, double* zbuffer, Matrix* P, int nW, int nH);

    /**
     * Set identifier
     * @param id string identifier
     */
    void setId(const string& id){_id = id;}

    /**
     * get identifier
     * @return id string identifier
     */
    const string& id(void)const{return(_id);}

    /**
     * Method to refresh the data in the cloud. If size differs from the size in the cloud,
     * the buffer is reallocated
     * @param size Number of points to refreshed
     * @param coords Cartesian coordinates of the new points
     * @param normals Normals of the points
     * @param rgb Colors of the points
     */
    void setData(const unsigned int size, double* coords, double* normals, const unsigned char* const rgb);

  private:

    void init(unsigned int size, bool withInfo);

    /**
     * point container
     */
    Matrix* _coords;
    Matrix* _normals;
    unsigned char* _colors;
    int* _attributes;
    int* _indices;

    /**
     * Info flag signs presence of additional point information
     */
    int _hasInfo;

    /**
     * Info flag signs presence of additional point information
     */
    int _hasNormals;

    /**
     * Info flag signs presence of point color information
     */
    bool _hasColors;

    /**
     * Source info map
     */
    map<int, long> _mSourceInfo;

    /**
     * string identifier
     */
    string _id;

    /**
     * number of points in the cloud
     */
    unsigned int _size;
  };

}

#endif /*CARTESIANCLOUD_H*/
