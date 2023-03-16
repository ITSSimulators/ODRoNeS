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

#include "numerical.h"

numerical::numerical()
{
    base();
}

numerical::numerical(const numerical& n)
{
    assignInputToThis(n);
}

numerical& numerical::operator=(const numerical &n)
{
    clearMemory();
    assignInputToThis(n);
    return *this;
}

numerical::~numerical()
{
    clearMemory();
}

void numerical::base()
{
    _approxDs = 0;
    _pointsSize = 0;
    _pointsX = nullptr;
    _pointsY = nullptr;
    _pointsS = nullptr;
}

void numerical::clearMemory()
{
    if (_pointsSize > 0)
    {
        delete[] _pointsX;
        delete[] _pointsY;
        delete[] _pointsS;
        base();
    }
}

scalar numerical::defaultDs(scalar length)
{
    return std::min(0.05, length / static_cast<scalar>(minPointsSize - 1)); // we need these many points to be able to differentiate.
}

uint numerical::pointsSize() const
{
    return _pointsSize;
}

scalar numerical::maxS() const
{
    return _pointsS[_pointsSize -1];
}

std::vector<arr2> numerical::points() const
{
    std::vector<arr2> v;
    for (uint i = 0; i < _pointsSize; ++i)
        v.push_back({_pointsX[i], _pointsY[i]});
    return v;
}

std::vector<scalar> numerical::S() const
{
    std::vector<scalar> s(_pointsSize);
    for (uint i = 0; i < _pointsSize; ++i)
        s[i] = _pointsS[i];
    return s;

}

void numerical::initialise(scalar ds, uint size)
{
    _approxDs = ds;
    allocateMemory(size);
    zeroPoints();
}

uint numerical::setup()
{
    uint err = 0;
    nSetupPointsXYUniformly(_approxDs);

    _pointsS[0] = 0;
    for (uint i = 1; i < _pointsSize; ++i)
        _pointsS[i] = _pointsS[i-1] +
                mvf::distance({_pointsX[i], _pointsY[i]}, {_pointsX[i-1], _pointsY[i-1]});

    return err;

}

void numerical::allocateMemory(uint pSize)
{
    _pointsSize = pSize;
    _pointsX = new scalar[pSize]();
    _pointsY = new scalar[pSize]();
    _pointsS = new scalar[pSize]();
}

void numerical::zeroPoints()
{
    std::fill(_pointsX, _pointsX + _pointsSize, 0);
    std::fill(_pointsY, _pointsY + _pointsSize, 0);
    std::fill(_pointsS, _pointsS + _pointsSize, 0);
}

void numerical::assignInputToThis(const numerical &n)
{
    _approxDs = n._approxDs;
    _pointsSize = n._pointsSize;
    if (_pointsSize > 0)
    {
        allocateMemory(_pointsSize);
        for (uint i = 0; i < _pointsSize; ++i)
        {
            _pointsX[i] = n._pointsX[i];
            _pointsY[i] = n._pointsY[i];
            _pointsS[i] = n._pointsS[i];
        }
    }
    else
        _pointsX = _pointsY = _pointsS = nullptr;
}


void numerical::nInvert()
{
    // Recalculate new points using interpolate()
    scalar *tmpX = new scalar[_pointsSize];
    scalar *tmpY = new scalar[_pointsSize];
    scalar *tmpS = new scalar[_pointsSize];
    tmpX[0] = _pointsX[_pointsSize - 1];
    tmpY[0] = _pointsY[_pointsSize - 1];
    tmpS[0] = 0;
    for (uint i = 1; i < _pointsSize - 1; ++i)
    {
        arr2 p = interpolate(maxS() - i * _approxDs);
        tmpX[i] = p[0];
        tmpY[i] = p[1];
        tmpS[i] = tmpS[i-1] + mvf::distance({tmpX[i], tmpY[i]}, {tmpX[i-1], tmpY[i-1]});
    }
    tmpX[_pointsSize - 1] = _pointsX[0];
    tmpY[_pointsSize - 1] = _pointsY[0];
    tmpS[_pointsSize - 1] = tmpS[_pointsSize -2] +
            mvf::distance({tmpX[_pointsSize -1], tmpY[_pointsSize -1]},
                          {tmpX[_pointsSize -2], tmpY[_pointsSize -2]});
    // Copy the data back to _pointsX,Y to preserve locality:
    std::copy(tmpX, tmpX + _pointsSize, _pointsX);
    std::copy(tmpY, tmpY + _pointsSize, _pointsY);
    std::copy(tmpS, tmpS + _pointsSize, _pointsS);

    // Remove the arrays
    delete[] tmpX;
    delete[] tmpY;
    delete[] tmpS;
}

bool numerical::interpolate(arr2 &p, scalar d) const
{
    if (_pointsSize == 0) return false;
    if ((d > maxS()) || (d < 0)) return false;
    p = interpolate(d);
    return true;
}

arr2 numerical::interpolate(scalar d) const
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
        return {_pointsX[_pointsSize - 1], _pointsY[_pointsSize - 1]};

    scalar frac = (d - _pointsS[ndx]) / (_pointsS[ndx + 1] - _pointsS[ndx]);
    return {_pointsX[ndx] * (1 - frac) + _pointsX[ndx+1] * frac,
            _pointsY[ndx] * (1 - frac) + _pointsY[ndx+1] * frac};

}


void numerical::nCalcBoundingBox(arr2 &blc, arr2 &trc)
{
    auto [ mx, Mx ] = std::minmax_element(_pointsX, _pointsX + _pointsSize);
    auto [ my, My ] = std::minmax_element(_pointsY, _pointsY + _pointsSize);
    blc = {*mx, *my};
    trc = {*Mx, *My};
    return;
}

scalar numerical::nProjectPointHere(arr2 &p, const arr2 &o) const
{
    uint minIdx = 0;
    scalar d2o = 1e12;
    uint prt = minPointsSize;
    uint parts = std::floor(_pointsSize / prt);
    if (parts < minPointsSize)
    {
        prt = 1;
        parts = _pointsSize -1;
    }
    for (uint i = 0; i <= parts; ++i)
    {
        uint idx = i * prt;
        if (idx > _pointsSize -1) // This has happened.
            idx = _pointsSize - 1;
        arr2 q = {_pointsX[idx], _pointsY[idx]};
        scalar d2i = mvf::sqrDistance(o, q);
        if (d2i < d2o)
        {
            minIdx = idx;
            d2o = d2i;
        }
    }

    scalar si;
    scalar so = _pointsS[minIdx]; // minIdx * _pointsDs;
    if (minIdx == _pointsSize -1) so = maxS();
    scalar ds = maxS() / parts;
    while (ds > mvf::distPrecision)
    {
        // look for a better s in the upper half of the interval:
        si = so + 0.5 * ds;
        if (si <= maxS())
        {
            arr2 q = interpolate(si);
            scalar d2i = mvf::sqrDistance(o, q);
            if (d2i < d2o)
            {
                d2o = d2i;
                so = si;
                ds = 0.5 * ds;
                continue;
            }
        }

        // look for a better s in the lower half of the interval:
        si = so - 0.5 * ds;
        if (si >= 0)
        {
            arr2 q = interpolate(si);
            scalar d2i = mvf::sqrDistance(o, q);
            if (d2i < d2o)
            {
                d2o = d2i;
                so = si;
                ds = 0.5 * ds;
                continue;
            }
        }

        // if we could not find anything better, half the interval
        ds = 0.5 * ds;
    }

    p = interpolate(so);
    return so;
}

scalar numerical::nDistanceToTheEoL(const arr2 &p) const
{
    arr2 q;
    return maxS() - numerical::nProjectPointHere(q, p);
}

bool numerical::nGetPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    if (d > maxS()) return false;
    arr2 q;
    scalar s = numerical::nProjectPointHere(q, o) + d;
    if (s > maxS())
        return false;
    p = interpolate(s);
    return true;
}

bool numerical::nGetPointAfterDistance(arr2 &p, scalar s, scalar d) const
{
    if (s + d > maxS()) return false;
    p = interpolate(s+d);
    return true;
}

bool numerical::nGetPointAtDistance(arr2 &p, scalar d) const
{
    if ((d < 0) || (d > maxS())) return false;
    p = interpolate(d);
    return true;
}

bool numerical::nGetIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    scalar *otX = new scalar[2];
    scalar *otY = new scalar[2];
    otX[0] = o[0];
    otY[0] = o[1];
    otX[1] = o[0] + 1000 * t[0];
    otY[1] = o[1] + 1000 * t[1];

    std::vector<arr2> intersections;
    mvf::numericalIntersections(intersections, _pointsX, _pointsY,
                                0, _pointsSize -1, otX, otY, 0, 1);

    delete[] otX;
    delete[] otY;

    if (!intersections.size()) return false;
    p = intersections[0];
    return true;
}

scalar numerical::nGetCurvature(const arr2 &p) const
{
    arr2 q;
    scalar s = nProjectPointHere(q, p);
    scalar xp, yp, xpp, ypp;
    scalar ds = 10* _approxDs; // 20* and 4th order was good too.
    if (s + ds > maxS())
    {
        // backward:
        arr2 b1, b2, b3;
        interpolate(b1, s - ds);
        interpolate(b2, s - 2*ds);
        interpolate(b3, s - 3*ds);

        xp = (0.5*b2[0] - 2*b1[0] + 1.5*p[0]) / ds;
        yp = (0.5*b2[1] - 2*b1[1] + 1.5*p[1]) / ds;
        xpp = ( -b3[0] + 4*b2[0] - 5*b1[0] + 2*p[0]) / (ds*ds);
        ypp = ( -b3[1] + 4*b2[1] - 5*b1[1] + 2*p[1]) / (ds*ds);
    }
    else if (s - ds < 0)
    {
        // forward:
        arr2 f1, f2, f3;
        interpolate(f1, s + ds);
        interpolate(f2, s + 2*ds);
        interpolate(f3, s + 3*ds);

        xp = (-1.5*p[0] + 2*f1[0] - 0.5*f2[0]) / ds;
        yp = (-1.5*p[1] + 2*f1[1] - 0.5*f2[1]) / ds;
        xpp = (2*p[0] - 5*f1[0] + 4*f2[0] - f3[0]) / (ds * ds);
        ypp = (2*p[1] - 5*f1[1] + 4*f2[1] - f3[1]) / (ds * ds);
    }
    else
    {
        arr2 f, b;
        interpolate(f, s + ds);
        interpolate(b, s - ds);

        // 2nd order should be decent enough
        xp = (f[0] - b[0]) / (2 * ds);
        yp = (f[1] - b[1]) / (2 * ds);
        xpp = (f[0] - 2*p[0] + b[0]) / (ds * ds);
        ypp = (f[1] - 2*p[1] + b[1]) / (ds * ds);

        // 4th order derivatives:
        // arr2 f2, b2;
        // interpolate(f2, s + 2*ds);
        // interpolate(b2, s - 2*ds);
        // xp = (0.25*b2[0] -2*b[0] + 2*f[0] - 0.25*f2[0]) / (3 * ds);
        // yp = (0.25*b2[1] -2*b[1] + 2*f[1] - 0.25*f2[1]) / (3 * ds);
        // xpp = (-0.25*b2[0] + 4*b[0] - 7.5*p[0] + 4*f[0] - 0.25*f2[0]) / (3*ds*ds);
        // ypp = (-0.25*b2[1] + 4*b[1] - 7.5*p[1] + 4*f[1] - 0.25*f2[1]) / (3*ds*ds);

    }
    scalar denom = (xp * xp + yp * yp);
    if (mvf::areCloseEnough(0., denom, 1e-12)) return std::numeric_limits<scalar>::max();
    scalar curvature = (xp * ypp - xpp * yp) / (denom * std::sqrt(denom));

    return curvature;
}


arr2 numerical::nGetTangentInPoint(const arr2 &p) const
{
    arr2 q;
    scalar s = nProjectPointHere(q, p);
    arr2 pp;
    scalar ds = _approxDs; // 10* _pointsDs;
    // 2nd order approximation for every case:
    if (s + ds > maxS())
    {
        // backward:
        arr2 b1, b2, b3;
        interpolate(b1, s - ds);
        interpolate(b2, s - 2*ds);
        interpolate(b3, s - 3*ds);

        pp[0] = (0.5*b2[0] - 2*b1[0] + 1.5*p[0]) / ds;
        pp[1] = (0.5*b2[1] - 2*b1[1] + 1.5*p[1]) / ds;
    }
    else if (s - ds < 0)
    {
        // forward:
        arr2 f1, f2, f3;
        interpolate(f1, s + ds);
        interpolate(f2, s + 2*ds);
        interpolate(f3, s + 3*ds);

        pp[0] = (-1.5*p[0] + 2*f1[0] - 0.5*f2[0]) / ds;
        pp[1] = (-1.5*p[1] + 2*f1[1] - 0.5*f2[1]) / ds;
    }
    else
    {
        arr2 f, b;
        interpolate(f, s + ds);
        interpolate(b, s - ds);

        pp[0] = (f[0] - b[0]) / (2 * ds);
        pp[1] = (f[1] - b[1]) / (2 * ds);
    }

    mvf::normalise(pp);
    return pp;
}


#ifdef QT_CORE_LIB
QPainterPath numerical::nGetQPainterPath(uint n) const
{
    QPainterPath qpp;

    qpp.moveTo(QPointF(ct::mToPix * _pointsX[0], - ct::mToPix * _pointsY[0]));
    scalar dl = maxS() / scalar(n);
    for (uint i = 0; i < n; ++i)
    {
        arr2 p = interpolate(i * dl);
        if ((!std::isfinite(p[0])) || (!std::isfinite(p[1])))
        {
            std::cout << "nan?" << std::endl;
        }
        qpp.lineTo(QPointF(ct::mToPix * p[0], - ct::mToPix * p[1]));
    }
    qpp.lineTo(QPointF(ct::mToPix * _pointsX[_pointsSize-1], - ct::mToPix * _pointsY[_pointsSize-1]));
    return qpp;
}
#endif // QT_CORE_LIB

