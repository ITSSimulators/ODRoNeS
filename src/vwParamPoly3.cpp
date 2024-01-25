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

#include "vwParamPoly3.h"


void vwParamPoly3::base()
{
    vwNumerical::base();

    _shape = mvf::shape::vwParamPoly3;
    _u = {0., 0., 0., 0.};
    _v = {0., 0., 0., 0.};
    _normalised = false;
    _mint = 0;
    _maxt = 0;

    _sign = 0;
    _l = 0;

    _po = {0., 0.};
    _pto = {0., 0.};
}

vwParamPoly3::vwParamPoly3(const Odr::geometry &odg, int sign,
                           std::vector<Odr::offset> vwWidth,
                           std::vector<Odr::offset> ioffset,
                           scalar so, scalar se, scalar roadSo, bool geomPrint)
{
    vwParamPoly3::base();

    _sign = sign;
    _vwOff = ioffset;
    _vwWidth = vwWidth;
    _l = odg.length;
    _roadSo = roadSo;

    _u[0] = odg.aU;
    _u[1] = odg.bU;
    _u[2] = odg.cU;
    _u[3] = odg.dU;

    _v[0] = odg.aV;
    _v[1] = odg.bV;
    _v[2] = odg.cV;
    _v[3] = odg.dV;

    _o = {odg.x, odg.y};
    _to = {std::cos(odg.hdg), std::sin(odg.hdg)};

    _po = _o;
    _pto = _to;

    // Check that so and se are within bounds:
    if (!mvf::isInRangeLR(so, 0, odg.length))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (!mvf::isInRangeLR(se, 0, odg.length))
        std::cerr << "se is out of bonds!!" << std::endl;

    _mint = so; // used later in vwNumerical::setup
    _maxt = se;
    if (odg.pRange == Odr::Attr::ParamPoly3Range::normalized)
        _normalised = true;
    else
        _normalised = false;

    _origin = vwParamPoly3::curvexy_a(_mint);
    _dest = vwParamPoly3::curvexy_a(_maxt);

    // setup the numerical side:
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


arr2 vwParamPoly3::clxy(scalar t) const
{
    scalar u = _u[0] + _u[1] * t + _u[2] * t*t + _u[3] * t * t * t;
    scalar v = _v[0] + _v[1] * t + _v[2] * t*t + _v[3] * t * t * t;

    return { _po[0] + u * _pto[0] - v * _pto[1],
             _po[1] + u * _pto[1] + v * _pto[0]};
}


arr2 vwParamPoly3::clPxy(scalar t) const
{
    scalar up = _u[1] + 2 * _u[2] * t + 3 * _u[3] * t * t;
    scalar vp = _v[1] + 2 * _v[2] * t + 3 * _v[3] * t * t;

    return { up * _pto[0] - vp * _pto[1],
             up * _pto[1] + vp * _pto[0]};

}

arr2 vwParamPoly3::clP2xy(scalar t) const
{
    scalar up2 = 2 * _u[2] + 6 * _u[3] * t;
    scalar vp2 = 2 * _v[2] + 6 * _v[3] * t;

    return { up2 * _pto[0] - vp2 * _pto[1],
             up2 * _pto[1] + vp2 * _pto[0]};
}

arr2 vwParamPoly3::curvexy_a(scalar t) const
{
    scalar tScale = t;
    if (_normalised) tScale = t / _l;
    return vwpp3xy(tScale);
}


arr2 vwParamPoly3::vwpp3xy(scalar t) const
{
    arr2 cl = clxy(t);
    arr2 ni = clNormal(t);

    scalar off;
    if (_normalised)
        off = vwNumerical::offset(_roadSo + t * _l);
    else
        off = vwNumerical::offset(_roadSo + t);

    return {cl[0] + off * ni[0],
            cl[1] + off * ni[1]};
}

arr2 vwParamPoly3::vwpp3Pxy(scalar t) const
{
    arr2 clP = clPxy(t);
    arr2 ni = clNormal(t);
    arr2 niP = clNormalP(t);

    scalar off, offP;
    if (_normalised)
    {
        off = offset(_roadSo + t * _l);
        offP = offsetP(_roadSo + t * _l);
    }
    else
    {
        off = offset(_roadSo + t);
        offP = offsetP(_roadSo + t);
    }


    return {clP[0] + offP * ni[0] + off * niP[0],
            clP[1] + offP * ni[1] + off * niP[1]};
}


arr2 vwParamPoly3::clNormal(scalar t) const
{
    arr2 ti = clPxy(t);
    mvf::normalise(ti);
    return {-_sign * ti[1],
             _sign * ti[0]};
}

arr2 vwParamPoly3::clNormalP(scalar t) const
{
    // That's the first derivative of the normal of the centre lane, which if we write clxy(t) = f(t) is:
    //           f'(t)    (  0  1 )
    //  n(t) = ---------- |       | * _sign
    //         ||f'(t)||  ( -1  0 )
    // and thus
    //              f''(t) * ||f'(t)|| - f'(t) * ( ||f'(t) ) '      / 0  1 \
    //  n'(t) = -------------------------------------------------   |      | * _sign
    //                        ||f'(t)||^2                           \ -1 0 /
    arr2 ti = clPxy(t);
    scalar tiLength2 = mvf::sqrMagnitude(ti);
    scalar tiLength = std::sqrt(tiLength2);
    arr2 tiP = clP2xy(t);
    scalar tiLengthP = (tiP[0] * ti[0] + tiP[1] * ti[1]) / tiLength;

    arr2 npi = { (tiP[0] * tiLength - ti[0] * tiLengthP) / tiLength2,
                 (tiP[1] * tiLength - ti[1] * tiLengthP) / tiLength2 };

    return {- _sign * npi[1], _sign * npi[0]};

}


scalar vwParamPoly3::calcLength() const
{
    return 0;
}

