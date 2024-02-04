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
#ifndef ___POINT_H___
#define ___POINT_H___

#include "obcore/base/types.h"

#include <stdint.h>

namespace obvious {

struct Pixel
{
    unsigned int u;
    unsigned int v;
};

struct Point
{
    obfloat x;
    obfloat y;
    obfloat z;
};

struct Point2D
{
    obfloat x;
    obfloat y;
};

struct PointRgb : public Point
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

struct PointRgbT : public PointRgb
{
    uint16_t t;
};

} // end namespace obvious

#endif
