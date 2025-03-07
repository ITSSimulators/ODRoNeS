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

#include "vwArc.h"
using namespace odrones;

vwArc::vwArc(const Odr::geometry &odg, int sign, std::vector<Odr::offset> off,
             scalar so, scalar se, scalar roadSo, bool print)
{
    vwNumerical::base();

    // Shape:
    _shape = mvf::shape::vwArc;
    _vwOff = off;
    _roadSo = roadSo;
    _roadSe = roadSo + se - so;
    _print = print;

    // Calculate the parameters of the central lane arc from arc with zero offset:
    // arc a = arc(odg, sign, 0, so, se);
    arc a = arc(odg, sign, 0, 0, odg.length, roadSo);
    _centre = a.centre();
    _alpha = odg.curvature * odg.length;
    _radiusOfCurvature = a.radiusOfCurvature();
    _co = a.co();
    _sign = sign;
    if (a.shape() == mvf::shape::clockwise)
        _sign = - _sign;

    // Now the initial directions:
    _to = a.to();
    _no = {- sign * _to[1], sign * _to[0]};

    // Now the start and end of the reference lane:
    _l = odg.length;
    _o = a.origin();
    _d = a.dest();

    // Now so and se:
    // firstly make sure that they fall in range:
    if (! mvf::isInRangeLR(so, 0, _l))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (! mvf::isInRangeLR(se, 0, _l))
    if ((se > odg.length) || (se < 0))
        std::cerr << "se is out of bonds!!" << std::endl;

    // offset is parametrised along s, we can't change that.
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

    if (print)
    {
        arc aAux = arc(odg, sign, off[0].a, so, se, roadSo);
        std::cout << "aAux: "; aAux.printOut();
        std::cout << "vwArc: "; printOut();
    }

    _ready = true;

}

arr2 vwArc::curvexy_a(scalar t) const
{
    scalar angle = t / _radiusOfCurvature;
    /*
    arr2 v = {std::abs(_radiusOfCurvature) * _co[0], std::abs(_radiusOfCurvature) * _co[1]};
    mvf::rotateVectorByAngle(v, angle);
    return {_centre[0] + v[0] + _no[0] * offset(t), _centre[1] + v[1] +  _no[1] * offset(t)};
    */
    arr2 ci = _co;
    mvf::rotateVectorByAngle(ci, angle);
    return {_centre[0] + ci[0] * (std::abs(_radiusOfCurvature) - _sign * offset(_roadSo + t)),
            _centre[1] + ci[1] * (std::abs(_radiusOfCurvature) - _sign * offset(_roadSo + t))};

}

arr2 vwArc::l0xy_a(scalar t) const
{
    scalar angle = t / _radiusOfCurvature;
    arr2 ci = _co;
    mvf::rotateVectorByAngle(ci, angle);
    return {_centre[0] + ci[0] * std::abs(_radiusOfCurvature),
            _centre[1] + ci[1] * std::abs(_radiusOfCurvature)};

}



void vwArc::printOut() const
{
    std::cout << "vwArc centre: (" << _centre[0] << ", " << _centre[1] << ")"
              << ", co: (" << _co[0] << ", " << _co[1] << ")"
              << ", _alpha: " << _alpha << ", rad: " << _radiusOfCurvature
              << ", offset(" << _mint << "): " << offset(_mint)
              << ", offset(" << _maxt << "): " << offset(_maxt);
    geometry::printOut();

}

scalar vwArc::l0Curvature() const
{
    return _radiusOfCurvature;
}
