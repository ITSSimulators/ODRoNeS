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

#include "vwStraight.h"

vwStraight::vwStraight(const Odr::geometry &odg, int sign, std::vector<Odr::offset> vwWidth,
                       std::vector<Odr::offset> off, scalar so, scalar se, scalar roadSo, bool print)
{
    vwNumerical::base();

    // Shape:
    _shape = mvf::shape::vwStraight;
    _vwOff = off;
    _vwWidth = vwWidth;
    _roadSo = roadSo;
    _print = print;

    // First the initial directions:
    _to = {std::cos(odg.hdg), std::sin(odg.hdg)};
    _no = {- sign * _to[1], sign * _to[0]};

    // Now the start and end of the reference lane:
    _l = odg.length;
    _o = {odg.x, odg.y};
    _d = {odg.x + _to[0] * _l, odg.y + _to[1] * _l};

    // Now so and se:
    // firstly make sure that they fall in range:
    if (! mvf::isInRangeLR(so, 0, _l))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (! mvf::isInRangeLR(se, 0, _l))
    if ((se > odg.length) || (se < 0))
        std::cerr << "se is out of bonds!!" << std::endl;

    // and secondly, store their values into _mint and _maxt
    _mint = so;
    _maxt = se;

    _origin = curvexy(_mint);
    _dest = curvexy(_maxt);

    // setup the numerical side:
    scalar ds = numerical::defaultDs(_l);
    vwNumerical::setup(ds);
    _length = numerical::maxS();

    // and now calculate the bounding box:
    nCalcBoundingBox(_blc, _trc);

    _ready = true;

}

arr2 vwStraight::curvexy_a(scalar t) const
{
    return {_o[0] + _to[0] * t + _no[0] * offset(_roadSo + t), _o[1] + _to[1] * t + _no[1] * offset(_roadSo + t)};
}

void vwStraight::printOut() const
{
    geometry::printOut();
}
