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

#include "vwSpiral.h"


void vwSpiral::base()
{
    vwNumerical::base();

    _shape = mvf::shape::vwSpiral;
    _sign = 0;
    _l = 0;

    offsetP = std::bind(&vwSpiral::offsetP_a, this, std::placeholders::_1);
}

vwSpiral::vwSpiral(const Odr::geometry &odg, int sign, std::vector<Odr::offset> ioffset,
                           scalar so, scalar se, scalar roadSo, bool geomPrint)
{
    vwSpiral::base();

    _sign = sign;
    _vwOff = ioffset;
    _l = odg.length;
    _roadSo = roadSo;

    _o = {odg.x, odg.y};
    _to = {std::cos(odg.hdg), std::sin(odg.hdg)};

    _clothoid.build(_o[0], _o[1], odg.hdg, odg.curvStart, (odg.curvEnd - odg.curvStart) / odg.length, odg.length);

    // Check that so and se are within bounds:
    if (!mvf::isInRangeLR(so, 0, odg.length))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (!mvf::isInRangeLR(se, 0, odg.length))
        std::cerr << "se is out of bonds!!" << std::endl;

    _origin = vwSpiral::curvexy_a(so);
    _dest = vwSpiral::curvexy_a(se);

    // setup the numerical side:
    _mint = so;
    _maxt = se;
    scalar ds = numerical::defaultDs(_l);
    vwNumerical::setup(ds);
    _length = numerical::maxS();

    // and now use it to calculate the bounding box:
    nCalcBoundingBox(_blc, _trc);

    if (geomPrint)
    {
        geometry::printOut();
        /* std::cout << "_origin: (" << _origin[0] << ", " << _origin[1] << ") "; // << std::endl;
        std::cout << "_to: (" << _to[0] << ", " << _to[1] << "), "; // << std::endl;
        std::cout << "dest: (" << _dest[0] << ", " << _dest[1] << ") "; // << std::endl;
        std::cout << "normalised: " << std::to_string(_normalised) << " ";
        std::cout << "length: " << _length << ", odrLength: " << _l << " " << std::endl; */
    }
}


arr2 vwSpiral::clxy(scalar t) const
{
    arr2 p;
    _clothoid.eval(t, p[0], p[1]);
    return p;
}


arr2 vwSpiral::clPxy(scalar t) const
{
    arr2 p;
    _clothoid.eval_D(t, p[0], p[1]);
    return p;
}


arr2 vwSpiral::clP2xy(scalar t) const
{
    arr2 p;
    _clothoid.eval_DD(t, p[0], p[1]);
    return p;
}

arr2 vwSpiral::curvexy_a(scalar t) const
{
    return xy(t);
}

arr2 vwSpiral::l0xy_a(scalar t) const
{
    return clxy(t);
}


arr2 vwSpiral::xy(scalar t) const
{
    arr2 cl = clxy(t);
    arr2 ni = clNormal(t);

    scalar off = vwNumerical::offset(_roadSo + t);

    return {cl[0] + off * ni[0],
            cl[1] + off * ni[1]};
}

arr2 vwSpiral::xyP(scalar t) const
{
    arr2 clP = clPxy(t);
    arr2 ni = clNormal(t);
    arr2 niP = clNormalP(t);

    scalar off = offset(_roadSo + t);
    scalar offP = offsetP(_roadSo + t);


    return {clP[0] + offP * ni[0] + off * niP[0],
            clP[1] + offP * ni[1] + off * niP[1]};
}


arr2 vwSpiral::clNormal(scalar t) const
{
    arr2 ti;
    _clothoid.tg(t, ti[0], ti[1]);
    return {-_sign * ti[1],
             _sign * ti[0]};
}

arr2 vwSpiral::clNormalP(scalar t) const
{
    // That's the first derivative of the normal of the centre lane, which if we write clxy(t) = f(t) is:
    //           f'(t)    (  0  1 )
    //  n(t) = ---------- |       | * _sign
    //         ||f'(t)||  ( -1  0 )
    // and thus
    //              f''(t) * ||f'(t)|| - f'(t) * ( ||f'(t) ) '      / 0  1 \
    //  n'(t) = -------------------------------------------------   |      | * _sign
    //                        ||f'(t)||^2                           \ -1 0 /
    arr2 tid;
    _clothoid.tg_D(t, tid[0], tid[1]);
    return {- _sign * tid[1], _sign * tid[0]};

}


