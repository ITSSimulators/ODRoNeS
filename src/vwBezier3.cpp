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

#include "vwBezier3.h"
using namespace odrones;


vwBezier3::vwBezier3(const Odr::geometry &odr, int sign, std::vector<Odr::offset> ioffset,
                     scalar so, scalar se, scalar roadSo, bool print)
{
    vwNumerical::base();

    //shape:
    _shape = mvf::shape::vwBezier3;
    _vwOff = ioffset;
    _roadSo = roadSo;
    _roadSe = roadSo + se - so;
    _sign = sign;

    // Set an internal Bezier3 to help with calculations:
    _bz0.set({odr.bz0x, odr.bz0y}, {odr.bz1x, odr.bz1y},
             {odr.bz2x, odr.bz2y}, {odr.bz3x, odr.bz3y});

    // Now the initial directions:
    _to = _bz0.to();
    _no = {-sign * _to[1], sign * _to[0]};

    // Now the start and end of the reference lane:
    _l = odr.length;
    _o = _bz0.origin();
    _d = _bz0.dest();

    // Now so and se:
    // firstly make sure that they fall in range:
    if (! mvf::isInRangeLR(so, 0, _l))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (! mvf::isInRangeLR(se, 0, _l))
    if ((se > odr.length) || (se < 0))
        std::cerr << "se is out of bonds!!" << std::endl;

    // offset is parametrised along s, we can't change that.
    _mint = so;
    _maxt = se;

    _origin = vwBezier3::curvexy_a(_mint);
    _dest = vwBezier3::curvexy_a(_maxt);

    // setup the numerical side:
    scalar ds = numerical::defaultDs(_l);
    vwNumerical::setup(ds);
    _length = numerical::maxS();

    // and now calculate the bounding box:
    nCalcBoundingBox(_blc, _trc);

    if (print)
    {
        std::cout << "bz0: "; _bz0.printOut();
        std::cout << "vwBezier3: "; printOut();
    }

    _ready = true;

}

arr2 vwBezier3::curvexy_a(scalar t) const
{
    scalar s = 0;
    if (! _bz0.getTgivenD(s, t))
    {
        std::cerr << "[ Error ] vwBezier3 could not find a t given d!" << std::endl;
        return {0., 0.};
    }

    arr2 cl = _bz0.curvexy(s);

    arr2 ti = _bz0.curvePxy(s);
    mvf::normalise(ti);
    arr2 ni = {-_sign * ti[1], _sign * ti[0]};

    scalar off = vwNumerical::offset(_roadSo + t);

    return {cl[0] + off * ni[0],
            cl[1] + off * ni[1]};
}

arr2 vwBezier3::l0xy_a(scalar t) const
{
    scalar s = 0;
    if (! _bz0.getTgivenD(s, t))
    {
        std::cerr << "[ Error ] vwBezier3 could not find a t given d: " << t << std::endl;
        return {0., 0.};
    }
    return _bz0.curvexy(s);
}
