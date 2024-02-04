#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <cstring>


/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class System
 * @brief This class encapsulates system specific calls for memory allocation
 * @author Stefan May
 */ 
template <class T>
class System
{
public:
    /**
     * Allocation of 2D arrays
     * @param rows number of rows
     * @param cols number of columns
     * @param array2D data array
     */
    static void allocate (unsigned int rows, unsigned int cols, T** &array2D);

    /**
     * Deallocation of 2D arrays. Pointers are set to null.
     * @param array2D data array
     */
    static void deallocate (T** &array2D);

    /**
     * Memcpy two-dimensional array
     * @param rows number of rows
     * @param cols number of columns
     * @param src source array
     * @param dst destination array
     */
    static void copy (unsigned int rows, unsigned int cols, T** &src, T** &dst);

    /**
     * Initialize array with zero values
     * @param rows number of rows
     * @param cols number of columns
     * @param buf array
     */
    static void initZero(unsigned int rows, unsigned int cols, T** buf);

    /**
     * Allocation of 3D arrays
     * @param rows number of rows
     * @param cols number of columns
     * @param slices number of slices
     * @param array3D data array
     */
    static void allocate (unsigned int rows, unsigned int cols, unsigned int slices, T*** &array3D);

    /**
     * Deallocation of 3D arrays. Pointers are set to null.
     * @param array3D data array
     */        
    static void deallocate (T*** &array3D);

    /**
     * Memcpy three-dimensional array
     * @param rows number of rows
     * @param cols number of columns
     * @param slices number of slices
     * @param src source array
     * @param dst destination array
     */
    static void copy (unsigned int rows, unsigned int cols, unsigned int slices,  T*** &src, T*** &dst);

    /**
     * Initialize array with zero values
     * @param rows number of rows
     * @param cols number of columns
     * @param slices number of slices
     * @param buf array
     */
    static void initZero(unsigned int rows, unsigned int cols, unsigned int slices, T*** buf);
};

#include "System.inl"

}

#endif /*SYSTEM_H_*/
