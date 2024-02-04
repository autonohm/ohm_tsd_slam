#include "obcore/base/PointCloud.h"
#include "obcore/base/Point.h"

#include <Eigen/Geometry>

#include <iostream>

namespace obvious {

/* Implementation for point type Point. */
template <>
PointCloud<Point>::PointCloud(const std::size_t size)
    : _width(size),
      _height(1),
      _points(size)
{

}

template <>
PointCloud<Point>::PointCloud(const PointCloud& cloud)
    : _width(cloud._width),
      _height(cloud._height),
      _points(cloud._points)
{

}

template <>
PointCloud<Point>::PointCloud(const unsigned int width, const unsigned int height)
    : _width(width),
      _height(height),
      _points(width * width)
{

}

template <>
void PointCloud<Point>::rotate(const obfloat roll, const obfloat pitch, const obfloat yaw)
{
    Eigen::Map<Eigen::Matrix<obfloat, Eigen::Dynamic, 3, Eigen::RowMajor>, Eigen::Aligned>
        mat(reinterpret_cast<obfloat*>(&_points[0]), _points.size(), 3);


    Eigen::Quaternion<obfloat> quat(Eigen::AngleAxis<obfloat>(roll, Eigen::Matrix<obfloat, 3, 1>::UnitX()) *
                                    Eigen::AngleAxis<obfloat>(pitch, Eigen::Matrix<obfloat, 3, 1>::UnitY()) *
                                    Eigen::AngleAxis<obfloat>(yaw, Eigen::Matrix<obfloat, 3, 1>::UnitZ()));
    Eigen::Matrix<obfloat, 3, 3> rot(quat.matrix());

    mat *= rot;
}



/* Implementation for point type PointRgb. */
template <>
PointCloud<PointRgb>::PointCloud(const std::size_t size)
    : _width(size),
      _height(1),
      _points(size)
{

}

template <>
PointCloud<PointRgb>::PointCloud(const PointCloud& cloud)
    : _width(cloud._width),
      _height(cloud._height),
      _points(cloud._points)
{

}

template <>
PointCloud<PointRgb>::PointCloud(const unsigned int width, const unsigned int height)
    : _width(width),
      _height(height),
      _points(width * height)
{

}

template <>
void PointCloud<PointRgb>::rotate(const obfloat roll, const obfloat pitch, const obfloat yaw)
{
    const unsigned int s = sizeof(PointRgb) / sizeof(obfloat);

    Eigen::Map<Eigen::Matrix<obfloat, Eigen::Dynamic, 3, Eigen::RowMajor>, Eigen::Aligned, Eigen::OuterStride<s> >
        mat(reinterpret_cast<obfloat*>(&_points[0]), _points.size(), 3);
    Eigen::Quaternion<obfloat> quat(Eigen::AngleAxis<obfloat>(roll, Eigen::Matrix<obfloat, 3, 1>::UnitX()) *
                                    Eigen::AngleAxis<obfloat>(pitch, Eigen::Matrix<obfloat, 3, 1>::UnitY()) *
                                    Eigen::AngleAxis<obfloat>(yaw, Eigen::Matrix<obfloat, 3, 1>::UnitZ()));
    Eigen::Matrix<obfloat, 3, 3> rot(quat.matrix());

    mat *= rot;
}

} // end namespace obvious
