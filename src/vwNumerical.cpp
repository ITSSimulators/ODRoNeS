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

void vwNumerical::base()
{
    geometry::base();
    numerical::base();

    _no = {0., 0.};
    _l = 0;
    _mint = 0;
    _maxt = 0;

    offset = std::bind(&vwNumerical::offset_a, this, std::placeholders::_1);
    offsetP = std::bind(&vwNumerical::offsetP_a, this, std::placeholders::_1);
    curvexy = std::bind(&vwNumerical::curvexy_a, this, std::placeholders::_1);

    _ahead = true;
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
    _off = vws._off;

    if (vws._ahead)
    {
        offset = std::bind(&vwNumerical::offset_a, this, std::placeholders::_1);
        curvexy = std::bind(&vwNumerical::curvexy_a, this, std::placeholders::_1);
    }
    else
    {
        offset = std::bind(&vwNumerical::offset_b, this, std::placeholders::_1);
        curvexy = std::bind(&vwNumerical::curvexy_b, this, std::placeholders::_1);
    }
    _ahead = vws._ahead;

    geometry::assignInputGeomToThis(vws);
    numerical::assignInputToThis(vws);
}

vwNumerical& vwNumerical::operator=(const vwNumerical &vws)
{
    numerical::clearMemory();
    assignInputGeomToThis(vws);
    return *this;
}



scalar vwNumerical::offset_a(scalar t) const
{
    /* The range to check is _off[x].s and _off[x].se */

    scalar o = 0.;
    bool set = false;
    for (uint i = 0; i < _off.size(); ++i)
    {
        bool inRange = false;
        if (_off[i].lr == Odr::offset::LR::RL)
            inRange = mvf::isInRangeLR(t, _off[i].s, _off[i].se);
        else if (_off[i].lr == Odr::offset::LR::L)
            inRange = mvf::isInRangeL(t, _off[i].s, _off[i].se);
        if (_off[i].lr == Odr::offset::LR::R)
            inRange = mvf::isInRangeR(t, _off[i].s, _off[i].se);

        if (inRange)
        {
            scalar s = t - _off[i].s;
            scalar s2 = s * s;
            o += _off[i].a + _off[i].b * s + _off[i].c * s2 + _off[i].d * s * s2;
            set = true;
        }
    }

    if (!set)
    {
        scalar d = 1e12;
        for (uint i = 0; i < _off.size(); ++i)
        {
            scalar dio = std::fabs(t - _off[i].s);
            if (dio < d)
            {
                o = _off[i].a + _off[i].b * _off[i].s + _off[i].c * _off[i].s * _off[i].s + _off[i].d * _off[i].s * _off[i].s * _off[i].s;
                d = dio;
            }
            scalar die = std::fabs(t - _off[i].se);
            if (die < d)
            {
                o = _off[i].a + _off[i].b * _off[i].se + _off[i].c * _off[i].se * _off[i].se + _off[i].d * _off[i].se * _off[i].se * _off[i].se;
                d = die;
            }
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
    for (uint i = 0; i < _off.size(); ++i)
    {
        bool inRange = false;
        if (_off[i].lr == Odr::offset::LR::RL)
            inRange = mvf::isInRangeLR(t, _off[i].s, _off[i].se);
        else if (_off[i].lr == Odr::offset::LR::L)
            inRange = mvf::isInRangeL(t, _off[i].s, _off[i].se);
        if (_off[i].lr == Odr::offset::LR::R)
            inRange = mvf::isInRangeR(t, _off[i].s, _off[i].se);

        if (inRange)
        {
            scalar s = t - _off[i].s;
            oP += _off[i].b  + 2 * _off[i].c * s + 3 * _off[i].d * s * s;
            set = true;
        }
    }

    if (!set)
    {
        scalar d = 1e12;
        for (uint i = 0; i < _off.size(); ++i)
        {
            scalar dio = std::fabs(t - _off[i].s);
            if (dio < d)
            {
                oP = _off[i].b + 2 * _off[i].c * _off[i].s + 3 * _off[i].d * _off[i].s * _off[i].s;
                d = dio;
            }
            scalar die = std::fabs(t - _off[i].se);
            if (die < d)
            {
                oP = _off[i].b + 2 * _off[i].c * _off[i].se + 3 * _off[i].d * _off[i].se * _off[i].se;
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

void vwNumerical::fillInSPoints(std::vector<scalar> &S, std::vector<arr2> &points, scalar ds, scalar dt) const
{
    points.push_back(curvexy(_mint));
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
    }
    S.push_back(mvf::distance(curvexy(_maxt), points.back()));
    points.push_back(curvexy(_maxt));
    return;
}

bool vwNumerical::setup(scalar ds)
{
    bool success = true;
    std::vector<scalar> S;
    std::vector<arr2> points;
    constexpr uint iParts = 10;
    scalar dt = ds / iParts;
    fillInSPoints(S, points, ds, dt);
    // Make sure there are enough points:
    if (points.size() < minPointsSize)
    {
        ds = S.back() / (minPointsSize - 1); // that should be enough
        dt = ds / iParts;
        S.clear();
        points.clear();
        fillInSPoints(S, points, ds, dt);
    }
    if (points.size() < minPointsSize)
    {
        std::cout << "[ Error ] vwNumerical::setup did not fill in enough points" << std::endl;
        success = false;
    }

    _pointsSize = static_cast<uint>(points.size());
    numerical::initialise(ds, _pointsSize);
    _pointsX[0] = points[0][0];
    _pointsY[0] = points[0][1];
    _pointsS[0] = 0;
    for (uint i = 1; i < _pointsSize; ++i)
    {
        _pointsX[i] = points[i][0];
        _pointsY[i] = points[i][1];
        _pointsS[i] = S[i] + _pointsS[i - 1];
    }
    _approxDs = numerical::maxS() / (_pointsSize - 1); // now get a more accurate value for _pointsDs

    return success;
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
