/*********************************************************************************
*
* Copyright (C) 2014 by TH-Nürnberg
* Written by Christian Merkl <christian.merkl@th-nuernberg.de>
* All Rights Reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*
*********************************************************************************/
#ifndef ___POINT_CLOUD_H___
#define ___POINT_CLOUD_H___

#include <cstdlib>
#include <vector>
#include <Eigen/StdVector>

#include "obcore/base/Point.h"

namespace obvious {

template <typename PointT>
class PointCloud
{
public:
    PointCloud(const std::size_t size = 0);
    PointCloud(const unsigned int width, const unsigned int height);
    PointCloud(const PointCloud& cloud);

    typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > Container;
    typedef typename Container::iterator iterator;
    typedef typename Container::const_iterator const_iterator;

    inline std::size_t size(void) const { return _points.size(); }
    inline iterator begin(void) { return _points.begin(); }
    inline const_iterator begin(void) const { return _points.begin(); }
    inline iterator end(void) { return _points.end(); }
    inline const_iterator end(void) const { return _points.end(); }
    inline PointT& operator[](const unsigned int index) { return _points[index]; }
    inline const PointT& operator[](const unsigned int index) const { return _points[index]; }
    inline unsigned int width(void) const { return _width; }
    inline unsigned int height(void) const { return _height; }
    inline bool isOrganized(void) const { return _height != 1; }
    inline void resize(const std::size_t size)
    {
        _width = size;
        _height = 1;
        _points.resize(size);
    }
    inline void resize(const unsigned int width, const unsigned int height)
    {
        _width = width;
        _height = height;
        _points.resize(width * height);
    }

    void rotate(const obfloat roll, const obfloat pitch, const obfloat yaw);

private:
    unsigned int _width;
    unsigned int _height;
    Container _points;
};

template class PointCloud<Point>;

} // end namespace obvious

#endif
