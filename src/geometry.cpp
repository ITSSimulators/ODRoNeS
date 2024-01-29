// 
//  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
//  
//  Copyright (c) 2023 Albert Solernou, University of Leeds.
// 
//  GTSmartActors is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  GTSmartActors is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// 
//  You should have received a copy of the GNU General Public License
//  along with ODRoNeS. If not, see <http://www.gnu.org/licenses/>.
// 
//  We would appreciate that if you use this software for work leading 
//  to publications you cite the package and its related publications. 
//

#include "geometry.h"

geometry::geometry()
{
    geometry::base();
    return;
}

geometry::~geometry()
{
    return;
}


geometry& geometry::operator=(const geometry& g)
{
    // clearMemory();
    assignInputGeomToThis(g);
    return *this;

}

void geometry::base()
{
    _origin = {0., 0.};
    _dest = {0., 0.};
    _shape = mvf::shape::unknown;
    _length = 0;
    _to = {0., 0.};
    _blc = {0., 0.};
    _trc = {0., 0.};
    _o = {0., 0.};
    _d = {0., 0.};
    _roadSo = 0;
    _ready = false;
}

scalar geometry::sl0(scalar s) const
{
    return s + _roadSo;
}

void geometry::assignInputGeomToThis(const geometry &g)
{
    _origin = g._origin;
    _dest = g._dest;
    _shape = g._shape;
    _length = g._length;
    _to = g._to;
    _blc = g._blc;
    _trc = g._trc;
    _o = g._o;
    _d = g._d;
    _roadSo = g._roadSo;
    _ready = g._ready;
}

void geometry::printOut() const
{
    std::cout << "orig: (" << _origin[0] << ", " << _origin[1] << ")"
              << ", dest: (" << _dest[0] << ", " << _dest[1] << ")"
              << ", length: " << _length << std::endl;
}
