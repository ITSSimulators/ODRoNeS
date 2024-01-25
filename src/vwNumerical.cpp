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

#include "vwNumerical.h"

vwNumerical::vwNumerical()
{
    vwNumerical::base();
}

vwNumerical::~vwNumerical()
{
    clearMemory();
}

void vwNumerical::base()
{
    geometry::base();
    numerical::base();

    _no = {0., 0.};
    _l = 0;
    _mint = 0;
    _maxt = 0;

    oWidth = std::bind(&vwNumerical::oWidth_a, this, std::placeholders::_1);
    offset = std::bind(&vwNumerical::offset_a, this, std::placeholders::_1);
    offsetP = std::bind(&vwNumerical::offsetP_a, this, std::placeholders::_1);
    curvexy = std::bind(&vwNumerical::curvexy_a, this, std::placeholders::_1);

    _ahead = true;

    _pointsW = nullptr;
}

void vwNumerical::allocateMemory(uint pSize)
{
    _pointsW = new scalar[pSize]();
}

void vwNumerical::zeroPoints()
{
    std::fill(_pointsW, _pointsW + _pointsSize, 0);
}

void vwNumerical::clearMemory()
{
    if (_pointsSize > 0)
    {
        delete[] _pointsW;
        vwNumerical::base();
    }
}

vwNumerical::vwNumerical(const vwNumerical& vws)
{
    assignInputGeomToThis(vws);
}

void vwNumerical::assignInputGeomToThis(const vwNumerical &vws)
{
    _no = vws._no;
    _l = vws._l;
    _mint = vws._mint;
    _maxt = vws._maxt;
    _vwOff = vws._vwOff;
    _vwWidth = vws._vwWidth;

    if (vws._ahead)
    {
        oWidth = std::bind(&vwNumerical::oWidth_a, this, std::placeholders::_1);
        offset = std::bind(&vwNumerical::offset_a, this, std::placeholders::_1);
        curvexy = std::bind(&vwNumerical::curvexy_a, this, std::placeholders::_1);
    }
    else
    {
        oWidth = std::bind(&vwNumerical::oWidth_b, this, std::placeholders::_1);
        offset = std::bind(&vwNumerical::offset_b, this, std::placeholders::_1);
        curvexy = std::bind(&vwNumerical::curvexy_b, this, std::placeholders::_1);
    }
    _ahead = vws._ahead;

    if ((vws._pointsSize > 0) && (vws._pointsW))
    {
        allocateMemory(vws._pointsSize);
        for (uint i = 0; i < vws._pointsSize; ++i)
            _pointsW[i] = vws._pointsW[i];
    }
    else
        _pointsW = nullptr;

    geometry::assignInputGeomToThis(vws);
    numerical::assignInputToThis(vws);

}


vwNumerical& vwNumerical::operator=(const vwNumerical &vws)
{
    numerical::clearMemory();
    assignInputGeomToThis(vws);
    return *this;
}


scalar vwNumerical::oWidth_a(scalar t) const
{
    /* The range to check is _off[x].s and _off[x].se, margins included depending on lr */
    scalar w = 0.;
    for (uint i = 0; i < _vwWidth.size(); ++i)
    {
        if (_vwWidth[i].inRange(t))
        {
            scalar s = t - _vwWidth[i].s;
            scalar s2 = s * s;
            w += _vwWidth[i].a + _vwWidth[i].b * s + _vwWidth[i].c * s2 + _vwWidth[i].d * s * s2;
        }
    }

    return w;

}


scalar vwNumerical::oWidth_b(scalar t) const
{
    return oWidth_a(_l - t);
}


scalar vwNumerical::offset_a(scalar t) const
{
    /* The range to check is _off[x].s and _off[x].se, margins included depending on lr */
    scalar o = 0.;
    for (uint i = 0; i < _vwOff.size(); ++i)
    {
        if (_vwOff[i].inRange(t))
        {
            scalar s = t - _vwOff[i].s;
            scalar s2 = s * s;
            o += _vwOff[i].a + _vwOff[i].b * s + _vwOff[i].c * s2 + _vwOff[i].d * s * s2;
        }
    }

    return o;
}

scalar vwNumerical::offset_b(scalar t) const
{
    return offset_a(_l - t);
}

scalar vwNumerical::offsetP_a(scalar t) const
{
    /* The range to check is _off[x].s and _off[x].se */

    scalar oP = 0.;
    bool set = false;
    for (uint i = 0; i < _vwOff.size(); ++i)
    {
        if (_vwOff[i].inRange(t))
        {
            scalar s = t - _vwOff[i].s;
            oP += _vwOff[i].b  + 2 * _vwOff[i].c * s + 3 * _vwOff[i].d * s * s;
            set = true;
        }
    }

    if (!set)
    {
        scalar d = 1e12;
        for (uint i = 0; i < _vwOff.size(); ++i)
        {
            scalar dio = std::fabs(t - _vwOff[i].s);
            if (dio < d)
            {
                oP = _vwOff[i].b + 2 * _vwOff[i].c * _vwOff[i].s + 3 * _vwOff[i].d * _vwOff[i].s * _vwOff[i].s;
                d = dio;
            }
            scalar die = std::fabs(t - _vwOff[i].se);
            if (die < d)
            {
                oP = _vwOff[i].b + 2 * _vwOff[i].c * _vwOff[i].se + 3 * _vwOff[i].d * _vwOff[i].se * _vwOff[i].se;
                d = die;
            }
        }
    }

    return oP;
}

scalar vwNumerical::offsetP_b(scalar t) const
{
    return - offsetP_a(_l - t);
}



/*
arr2 vwNumerical::curvexy_a(scalar t) const
{
    return {_o[0] + _to[0] * t + _no[0] * offset(t), _o[1] + _to[1] * t + _no[1] * offset(t)};
}
*/


arr2 vwNumerical::curvexy_b(scalar t) const
{
    return curvexy_a(_l - t);
}

void vwNumerical::fillInSPoints(std::vector<scalar> &W, std::vector<scalar> &S, std::vector<arr2> &points,
                                scalar ds, scalar dt) const
{
    points.push_back(curvexy(_mint));
    W.push_back(oWidth(_roadSo + _mint));
    S.push_back(0);
    scalar t = _mint;
    while (t < _maxt)
    {
        scalar ds_i = 0;
        arr2 ie;
        while ((ds_i < ds) && (t < _maxt))
        {
           ie = curvexy(t);
           ds_i += mvf::distance(points.back(), ie);
           t += dt;
        }
        S.push_back(mvf::distance(ie, points.back()));
        points.push_back(ie);
        W.push_back(oWidth(_roadSo + t - dt));
    }
    S.push_back(mvf::distance(curvexy(_maxt), points.back()));
    points.push_back(curvexy(_maxt));
    W.push_back(oWidth(_maxt));
    return;
}

bool vwNumerical::setup(scalar ds)
{
    bool success = true;
    std::vector<scalar> S, W;
    std::vector<arr2> points;
    constexpr uint iParts = 10;
    scalar dt = ds / iParts;
    fillInSPoints(W, S, points, ds, dt);
    // Make sure there are enough points:
    if (points.size() < minPointsSize)
    {
        ds = S.back() / (minPointsSize - 1); // that should be enough
        dt = ds / iParts;
        S.clear();
        points.clear();
        fillInSPoints(W, S, points, ds, dt);
    }
    if (points.size() < minPointsSize)
    {
        std::cout << "[ Error ] vwNumerical::setup did not fill in enough points" << std::endl;
        success = false;
    }

    _pointsSize = static_cast<uint>(points.size());
    numerical::initialise(ds, _pointsSize);
    vwNumerical::allocateMemory(_pointsSize);
    vwNumerical::zeroPoints();
    _pointsX[0] = points[0][0];
    _pointsY[0] = points[0][1];
    _pointsS[0] = 0;
    _pointsW[0] = W[0];
    for (uint i = 1; i < _pointsSize; ++i)
    {
        _pointsX[i] = points[i][0];
        _pointsY[i] = points[i][1];
        _pointsS[i] = S[i] + _pointsS[i - 1];
        _pointsW[i] = W[i];
    }
    _approxDs = numerical::maxS() / (_pointsSize - 1); // now get a more accurate value for _pointsDs

    return success;
}

scalar vwNumerical::interpolateW(scalar d) const
{
    // Don't segfault, please:
    if (d > maxS())
    {
        if (!mvf::areCloseEnough(d, maxS(), 1e-8))
            std::cout << "numerical::interpolate got " << d << ", but maxs = " << maxS() << std::endl;
        d = maxS();
    }
    else if (d < 0)
    {
        if (!mvf::areCloseEnough(d, 0, 1e-8))
            std::cout << "numerical::interpolate got " << d << ", but s starts at 0!" << std::endl;
        d = 0;
    }

    // Take a good initial guess for ndx:
    uint ndx = std::floor((scalar) d / _approxDs);
    if (ndx > _pointsSize -1) ndx = _pointsSize -1; // this can very rarely.
    //  and now refine it:
    while ( (ndx > 0) && (_pointsS[ndx] > d) )
            ndx -= 1;
    while ( (ndx < _pointsSize - 1) && (_pointsS[ndx + 1] < d) )
            ndx += 1;


    if (ndx == _pointsSize -1)
        return _pointsW[_pointsSize - 1];

    scalar frac = (d - _pointsS[ndx]) / (_pointsS[ndx + 1] - _pointsS[ndx]);
    return _pointsW[ndx] * (1 - frac) + _pointsW[ndx+1] * frac;

}


void vwNumerical::nSetupPointsXYUniformly(scalar dl)
{
    // Memory has already been allocated!
    arr2 io = curvexy(_mint);
    _pointsX[0] = io[0];
    _pointsY[0] = io[1];


    // Estimate the length:
    constexpr uint iParts = 10;
    scalar dt = (_maxt - _mint) / (_pointsSize * iParts);
    scalar t = _mint;
    scalar realLength = 0;
    while (t < _maxt)
    {
        arr2 ie = curvexy(t);
        realLength += mvf::distance(io, ie);
        io = ie;
        t += dt;
    }
    realLength += mvf::distance(io, curvexy(_maxt));



    scalar l = dl;
    scalar integral = 0;
    io = {_pointsX[0], _pointsY[0]};
    t = _mint;
    bool lpm = false; // last point missing
    for (uint i = 1; i < _pointsSize; ++i)
    {
        if (l > realLength) // we rounded a bit too far.
        {
            lpm = true;
            if (i != _pointsSize -1)
            {
                // std::cout << "[ lane ] _pointsXY will likely go wrong... " << std::endl;
                _pointsSize = i + 1;
            }
            break;
        }

        while (integral < l)
        {
            t += dt;
            arr2 ie = curvexy(t);
            integral += mvf::distance(ie, io);
            io = ie;
            if (mvf::areCloseEnough(integral, l, mvf::absolutePrecision)) break;
        }
        _pointsX[i] = io[0];
        _pointsY[i] = io[1];
        l += dl;
    }

    if (lpm)
    {
        io = curvexy(_maxt);
        _pointsX[_pointsSize -1] = io[0];
        _pointsY[_pointsSize -1] = io[1];
    }

    return;
}


void vwNumerical::invert()
{
    // swap origin and destination
    arr2 tmp = _dest;
    _dest = _origin;
    _origin = tmp;

    // don't swap o and d:

    // swap curvexy and offset:
    if (_ahead)
    {
        offset = std::bind(&vwNumerical::offset_b, this, std::placeholders::_1);
        offsetP = std::bind(&vwNumerical::offsetP_b, this, std::placeholders::_1);
        curvexy = std::bind(&vwNumerical::curvexy_b, this, std::placeholders::_1);
    }
    else
    {
        offset = std::bind(&vwNumerical::offset_a, this, std::placeholders::_1);
        offsetP = std::bind(&vwNumerical::offsetP_a, this, std::placeholders::_1);
        curvexy = std::bind(&vwNumerical::curvexy_a, this, std::placeholders::_1);
    }
    _ahead = !_ahead;

    numerical::nInvert();

    std::cerr << "[ Unable to invert ] because we'd need to fix _pointsW, and we're changing that" << std::endl;

    _to = numerical::nGetTangentInPoint(_origin);
}


bool vwNumerical::isPointHere(const arr2 &p) const
{
    if (!mvf::isPointInBoxBLcTRcTol(p, _blc, _trc, mvf::absolutePrecision))
        return false;

    arr2 q;
    numerical::nProjectPointHere(q, p); /// And here we see that deriving both from numerical and parametric was not a great idea.
    if (mvf::sqrDistance(q, p) < mvf::distPrecision2) return true;
    return false;
}


arr2 vwNumerical::projectPointHere(const arr2 &p) const
{
    arr2 q;
    numerical::nProjectPointHere(q, p);
    return q;
}

arr2 vwNumerical::getTangentInPoint(const arr2 &p) const
{
    return numerical::nGetTangentInPoint(p);
}

scalar vwNumerical::distanceToTheEoL(const arr2 &p) const
{
    return numerical::nDistanceToTheEoL(p);
}

bool vwNumerical::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    return numerical::nGetPointAfterDistance(p, o, d);
}

bool vwNumerical::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    return numerical::nGetIntersectionPointFromOT(p, o, t);
}

scalar vwNumerical::getCurvature(const arr2 &p) const
{
    return numerical::nGetCurvature(p);
}

#ifdef QT_CORE_LIB
QPainterPath vwNumerical::getQPainterPath(uint n) const
{
    return nGetQPainterPath(n);
}
#endif
