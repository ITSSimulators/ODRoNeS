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

#include "bezier.h"
using namespace odrones;

bezier::bezier()
{
    _degree = 0;
    _wx = nullptr;
    _wy = nullptr;
    geometry::base();
}

void bezier::base()
{
    _degree = 0;
    _wx = nullptr;
    _wy = nullptr;
}

bezier::bezier(const bezier &b)
{
    assignInputLaneToThis(b);
}

bezier::~bezier()
{
    clearMem();
}

bezier& bezier::operator =(const bezier& b)
{
    clearMem();
    assignInputLaneToThis(b);
    return *this;
}


void bezier::allocateMem(uint degree)
{
    _wx = new scalar[degree + 1];
    _wy = new scalar[degree + 1];
}


arr2 bezier::controlPoint(uint i) const
{
    if (i > _degree) return {0., 0.};
    return {_wx[i], _wy[i]};
}


void bezier::clearMem()
{
    if (_ready)
    {
        _degree = 0;
        delete[] _wx;
        delete[] _wy;
    }
}

void bezier::assignInputLaneToThis(const bezier &b)
{
    if (!b._ready) return;

    _degree = b._degree;
    allocateMem(_degree);
    for (size_t i = 0; i < _degree + 1; ++i)
    {
        _wx[i] = b._wx[i];
        _wy[i] = b._wy[i];
    }
    _length = b._length;
}

void bezier::invert()
{
    arr2 tmp = getTangentInPoint(_dest);
    _to = {-tmp[0], -tmp[1]};

    scalar* _tmpx = new scalar[_degree + 1];
    scalar* _tmpy = new scalar[_degree + 1];
    for (uint i = 0; i < _degree + 1; ++i)
    {
        _tmpx[_degree - i] = _wx[i];
        _tmpy[_degree - i] = _wy[i];
    }
    // we'll move the data again, hoping that the first array was allocated in a better place.
    for (uint i = 0; i < _degree + 1; ++i)
    {
        _wx[i] = _tmpx[i];
        _wy[i] = _tmpy[i];
    }
    delete[] _tmpx;
    delete[] _tmpy;

    // swap origin and destination
    tmp = _dest;
    _dest = _origin;
    _origin = tmp;

    // swap o and d:
    tmp = _d;
    _d = _o;
    _o = tmp;

    return;
}


bool bezier::distanceToTheEoL(scalar &d, const arr2 &p) const
{
    scalar t;
    if (!getTGivenXY(t, p)) return false;
    d = distanceToTheEoL(t);
    return true;
}

scalar bezier::distanceToTheEoL(const arr2 &p) const
{
    scalar t;
    if (!getTGivenXY(t, p)) return -1;
    return distanceToTheEoL(t);
}

bool bezier::distanceFromTheBoL(scalar &d, const arr2 &p) const
{
    scalar t;
    if (!getTGivenXY(t, p)) return false;
    d = calcLength(t);
    return true;
}

scalar bezier::distanceFromTheBoL(scalar t) const
{
    return calcLength(t);
}

scalar bezier::distanceToTheEoL(scalar t) const
{
    scalar d = _length - calcLength(t);
    if (d < 0) d = 0;
    return d;
}

scalar bezier::distanceBetween(scalar t1, scalar t2) const
{
    return std::abs(calcLength(t2) - calcLength(t1));
}

scalar bezier::tangentSize(scalar t) const
{
    arr2 dr = curvePxy(t);
    return std::sqrt(dr[0] * dr[0] + dr[1] * dr[1]);
}

bool bezier::getTgivenD(scalar &t, scalar d, scalar epsilon) const
{
    constexpr bool rungeKutta = false;
    if (rungeKutta) return getTgivenDRungeKutta(t, d, epsilon);
    else return getTgivenDNewtonRoot(t, d, epsilon); // this one does not quite work yet.
}

bool bezier::getTgivenDRungeKutta(scalar &t, scalar d, scalar epsilon) const
{
    if (mvf::areSameValues(0, d))
    {
        t = 0;
        return true;
    }
    if (mvf::areSameValues(_length,d))
    {
        t = 1;
        return true;
    }
    if ((d > _length) || (d < 0)) return false;

    scalar erro = 10;
    uint nIter = 20;
    while (erro > epsilon)
    {
        scalar h = d / nIter;
        t = 0;
        for (uint i=0; i<nIter; ++i)
        {
            scalar k1 = h / tangentSize(t);
            scalar k2 = h / tangentSize(t + k1 / 2.);
            scalar k3 = h / tangentSize(t + k2 / 2.);
            scalar k4 = h / tangentSize(t + k3);
            t += (k1 + 2*(k2 + k3) + k4) * ct::oneSixth;
        }
        scalar erri = std::abs(d - distanceFromTheBoL(t));
        if (std::abs(erri-erro)/erro < 1e-12) return false;
        erro = erri;
        nIter *= 2;
    }
    return true;
}

bool bezier::getTgivenDNewtonRoot(scalar &t, scalar d, scalar epsilon) const
{
    t = d / _length;
    scalar tmin = 0;
    scalar tmax = 1;
    uint nIter = 500;
    for (uint i = 0; i < nIter; ++i)
    {
        scalar F = distanceFromTheBoL(t) - d;
        if (std::abs(F) <= epsilon)
        {
            return true;
        }
        scalar dFdt = tangentSize(t);

        bool dFdt_zero = false;
        scalar ti = t;
        if (mvf::areCloseEnough(dFdt, 0., 1e-16))
            dFdt_zero = true;
        else
            ti = t - F/dFdt;

        if (F > 0)
        {
            tmax = t;
            if ((ti <= tmin) || (dFdt_zero)) t = (tmax + tmin) / 2.;
            else t = ti;
        }
        else
        {
            tmin = t;
            if ((ti >= tmax) || (dFdt_zero)) t = (tmax + tmin) / 2.;
            else t = ti;
        }
    }

    return false;

}

bool bezier::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    if (mvf::areSameValues(d, 0))
    {
        p = {o[0], o[1]};
        return true;
    }

    scalar to;
    if (!getTGivenXY(to, o))
    {
        std::cerr << "[ bezier ] o: " << o[0] << ", " << o[1] << " is not in this line." << std::endl;
        return false;
    }

    scalar so = distanceFromTheBoL(to);
    if (so + d > _length) return false; // that means the point would be after the end of this line.

    return getPointAtDistance(p, so + d);
}


bool bezier::getPointAtDistance(arr2 &p, scalar d) const
{
    if (d > _length) return false;

    if (mvf::areSameValues(d, 0))
    {
        p = origin();
        return true;
    }

    // get the parametre t (te) after d:
    scalar te = 0;
    if (!getTgivenD(te, d)) return false; // and if false, then it was too long.

    p = curvexy(te);
    return true;
}



scalar bezier::getLength() const
{
    return _length;
}

scalar bezier::getCurvature(scalar t) const
{
    return pGetCurvature(t);
}

scalar bezier::getCurvature(const arr2 &p) const
{
    return pGetCurvature(p);
}


arr2 bezier::projectPointHere(const arr2 &o) const
{
    arr2 p;
    projectPointHereT(p, o);
    return p;
}


bool bezier::isPointHere(const arr2 &p) const
{
    scalar t = 0;
    return getTGivenXY(t, p);
}

arr2 bezier::getTangentInPoint(const arr2 &p) const
{
    scalar t;
    getTGivenXY(t, p);
    arr2 tangent = curvePxy(t);
    mvf::normalise(tangent);
    return tangent;
}

#ifdef QT_CORE_LIB
QPainterPath bezier::getQPainterPath(uint n) const
{
    QPainterPath qpp;
    qpp.moveTo(QPointF(ct::mToPix * curvex(0), - ct::mToPix * curvey(0)));
    for (uint i = 0; i <= n; ++i)
    {
        scalar t = i / scalar(n);
        auto [x, y] = curvexy(t);
        qpp.lineTo(QPointF(ct::mToPix * x, - ct::mToPix * y));
    }
    qpp.lineTo(QPointF(ct::mToPix * curvex(1), - ct::mToPix * curvey(1)));
    return qpp;
}
#endif

scalar bezier::arcError() const
{
    arc ao = Arc();
    scalar d1 = mvf::distance(curvexy(0.25), ao.centre());
    scalar d2 = mvf::distance(curvexy(0.75), ao.centre());
    scalar r = std::abs(ao.radiusOfCurvature());

    return std::sqrt(0.5 * ( (d1-r)*(d1-r) + (d2-r)*(d2-r) ));
}

arc bezier::Arc() const
{
    arc ao;
    ao.setWith3Points(curvexy(0), curvexy(1), curvexy(0.5));
    return ao;
}
