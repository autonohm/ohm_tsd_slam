template <class T>
void System<T>::allocate (unsigned int rows, unsigned int cols, T** &array2D)
{
    array2D = new T*[rows];
    array2D[0] = new T[rows*cols];
    for (unsigned int row = 1; row < rows; row++)
    {
        array2D[row] = &array2D[0][cols*row];
    }
}

template <class T>
void System<T>::deallocate (T**& array2D)
{
    delete[] array2D[0];
    delete[] array2D;
    array2D = 0;
}

template <class T>
void System<T>::copy (unsigned int rows, unsigned int cols, T** &src, T** &dst)
{
	memcpy(dst[0], src[0], rows*cols*sizeof(T));
}

template <class T>
void System<T>::initZero(unsigned int rows, unsigned int cols, T** buf)
{
    memset(buf[0], 0, rows*cols*sizeof(T));
}

template <class T>
void System<T>::allocate (unsigned int rows, unsigned int cols, unsigned int slices, T*** &array3D)
{
    array3D = new T**[rows];
    for (unsigned int row = 0; row < rows; row++)
    	System<T>::allocate(cols, slices, array3D[row]);
}

template <class T>
void System<T>::deallocate (T***& array3D)
{
    delete[] array3D[0][0];
    delete[] array3D[0];
    delete[] array3D;
    array3D = 0;
}

template <class T>
void System<T>::copy (unsigned int rows, unsigned int cols, unsigned int slices,  T*** &src, T*** &dst)
{
    for (unsigned int row = 0; row < rows; row++)
    	System<T>::memcpy(cols, slices, src[row], dst[row]);
}

template <class T>
void System<T>::initZero(unsigned int rows, unsigned int cols, unsigned int slices, T*** buf)
{
    for(unsigned int i=0; i<slices; i++)
       initZero(rows, cols, buf[i]);
}