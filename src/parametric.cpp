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

#include "parametric.h"

parametric::parametric() :
    _mint(0),
    _maxt(1)
{

}

uint parametric::rootPx(scalar &t1, scalar &t2) const
{
    t1 = t2 = 0;
    return 0;
}

uint parametric::rootPy(scalar &t1, scalar &t2) const
{
    t1 = t2 = 0;
    return 0;
}

bool parametric::rootPx(scalar &t) const
{
    t = 0;
    return false;
}

bool parametric::rootPy(scalar &t) const
{
    t = 0;
    return false;
}

bool parametric::isTinRange(scalar t) const
{
    if (mvf::isInRangeLR(t, _mint, _maxt)) return true;
    return false;
}


bool parametric::isTinRangeFix(scalar &t) const
{
    if ((t >= _mint) && (t <= _maxt)) return true;
    if (mvf::areSameValues(_mint, t))
    {
        t = _mint;
        return true;
    }
    if (mvf::areSameValues(_maxt, t))
    {
        t = _maxt;
        return true;
    }
    return false;
}


uint parametric::reorderTinRange(scalar &t1, scalar &t2) const
{
    uint answers = 0;
    if (isTinRange(t1))
    {
        answers += 1;
        if (mvf::areSameValues(t1, t2)) return answers;
    }

    if (isTinRange(t2))
    {
        answers += 1;
        if (answers == 1) t1 = t2;
    }

    return answers;
}


uint parametric::reorderTinRangeFix(scalar &t1, scalar &t2) const
{
    uint answers = 0;
    if (isTinRangeFix(t1))
    {
        answers += 1;
        if (mvf::areSameValues(t1, t2)) return answers;
    }

    if (isTinRangeFix(t2))
    {
        answers += 1;
        if (answers == 1) t1 = t2;
    }

    return answers;
}


uint parametric::reorderTinRange(scalar &t1, scalar &t2, scalar &t3) const
{
    uint answers = 0;
    if (isTinRange(t1))
    {
        answers += 1;
        if (isTinRange(t2))
        {
            answers += 1;
            if (isTinRange(t3))
                answers += 1;
        }
        else if (isTinRange(t3))
        {
            t2 = t3;
            answers += 1;
        }
    }
    else
    {
        if (isTinRange(t2))
        {
            answers += 1;
            t1 = t2;
            if (isTinRange(t3))
            {
                answers += 1;
                t2 = t3;
            }
        }
        else
        {
            if (isTinRange(t3))
            {
                answers += 1;
                t1 = t3;
            }
        }
    }

    return answers;
}


uint parametric::reorderTinRangeFix(scalar &t1, scalar &t2, scalar &t3) const
{
    uint answers = 0;
    if (isTinRangeFix(t1))
    {
        answers += 1;
        if (isTinRangeFix(t2))
        {
            answers += 1;
            if (isTinRangeFix(t3))
                answers += 1;
        }
        else if (isTinRangeFix(t3))
        {
            t2 = t3;
            answers += 1;
        }
    }
    else
    {
        if (isTinRangeFix(t2))
        {
            answers += 1;
            t1 = t2;
            if (isTinRangeFix(t3))
            {
                answers += 1;
                t2 = t3;
            }
        }
        else
        {
            if (isTinRangeFix(t3))
            {
                answers += 1;
                t1 = t3;
            }
        }
    }

    return answers;
}


scalar parametric::calcLengthGQ20() const
{
    uint order = 20;
    scalar l = 0;
    for (uint i = 0; i < order; ++i)
    {
        arr2 dr = curvePxy(0.5*((_maxt - _mint) * ct::gc::A20[i] + (_maxt + _mint)));
        l += ct::gc::W20[i] * std::sqrt(dr[0] * dr[0] + dr[1] * dr[1]);
    }
    l *= (_maxt - _mint) * 0.5;
    return l;
}

scalar parametric::calcLengthGQ30() const
{
    uint order = 30;
    scalar lg = 0;
    for (uint i = 0; i < order; ++i)
    {
        arr2 dr = curvePxy(0.5*((_maxt - _mint) * ct::gc::A30[i] + (_maxt + _mint)));
        lg += ct::gc::W30[i] * std::sqrt(dr[0] * dr[0] + dr[1] * dr[1]);
    }
    lg *= (_maxt - _mint) * 0.5;

    return lg;
}


void parametric::calcBoundingBoxO2(arr2 &blc, arr2 &trc) const
{
    auto [ ox, oy ] = curvexy(_mint);
    auto [ ex, ey ] = curvexy(_maxt);

    scalar minx, miny, maxx, maxy;
    if (ox < ex)
    {
        minx = ox;
        maxx = ex;
    }
    else
    {
        minx = ex;
        maxx = ox;
    }

    if (oy < ey)
    {
        miny = oy;
        maxy = ey;
    }
    else
    {
        miny = ey;
        maxy = oy;
    }

    scalar bp0tx; // t at which Bx' is zero
    scalar bp0ty; // t at which By' is zero

    if (rootPx(bp0tx))
    {
        auto [ rx, ry ] = curvexy(bp0tx);
        if (rx < minx) minx = rx;
        if (ry < miny) miny = ry;
        if (rx > maxx) maxx = rx;
        if (ry > maxy) maxy = ry;
    }

    if (rootPy(bp0ty))
    {
        auto [ rx, ry ] = curvexy(bp0ty);
        if (rx < minx) minx = rx;
        if (ry < miny) miny = ry;
        if (rx > maxx) maxx = rx;
        if (ry > maxy) maxy = ry;
    }

    blc = { minx, miny };
    trc = { maxx, maxy};
}

void parametric::calcBoundingBoxO3(arr2 &blc, arr2 &trc) const
{
    auto [ ox, oy ] = curvexy(_mint);
    auto [ ex, ey ] = curvexy(_maxt);

    scalar minx, miny, maxx, maxy;
    if (ox < ex)
    {
        minx = ox;
        maxx = ex;
    }
    else
    {
        minx = ex;
        maxx = ox;
    }

    if (oy < ey)
    {
        miny = oy;
        maxy = ey;
    }
    else
    {
        miny = ey;
        maxy = oy;
    }

    scalar tx1, tx2;
    uint xAnswers = rootPx(tx1, tx2);
    if (xAnswers >= 1) // consider tx1
    {
        auto [rx, ry] = curvexy(tx1);
        if (rx < minx) minx = rx;
        if (ry < miny) miny = ry;
        if (rx > maxx) maxx = rx;
        if (ry > maxy) maxy = ry;
    }
    if (xAnswers == 2) // consider also tx2
    {
        auto [rx, ry] = curvexy(tx2);
        if (rx < minx) minx = rx;
        if (ry < miny) miny = ry;
        if (rx > maxx) maxx = rx;
        if (ry > maxy) maxy = ry;
    }

    scalar ty1, ty2;
    uint yAnswers = rootPy(ty1, ty2);
    if (yAnswers >= 1) // consider ty1
    {
        auto [rx, ry] = curvexy(ty1);
        if (rx < minx) minx = rx;
        if (ry < miny) miny = ry;
        if (rx > maxx) maxx = rx;
        if (ry > maxy) maxy = ry;
    }
    if (yAnswers == 2) // consider also ty2
    {
        auto [rx, ry] = curvexy(ty2);
        if (rx < minx) minx = rx;
        if (ry < miny) miny = ry;
        if (rx > maxx) maxx = rx;
        if (ry > maxy) maxy = ry;
    }

    blc = {minx, miny};
    trc = {maxx, maxy};

    return;
}


scalar parametric::projectPointHereT(arr2 &p, const arr2 &o) const
{
    constexpr scalar ds = 0.10;
    uint parts = (std::max)(50, static_cast<int>(_length / ds));

    scalar dt = ( _maxt - _mint )/ parts; // continue here!!!
    uint minIdx = 0;
    scalar d2o = 1e12;
    for (uint i = 0; i <= parts; ++i)
    {
        arr2 q = curvexy(_mint + i * dt);
        scalar d2i = mvf::sqrDistance(o, q);
        if (d2i < d2o)
        {
            minIdx = i;
            d2o = d2i;
        }
    }

    scalar to = minIdx * dt;
    scalar ti;
    while (dt > mvf::distPrecision)
    {
        // look for a better t in the upper half of the interval:
        ti = to + 0.5 * dt;
        if (ti <= _maxt)
        {
            arr2 q = curvexy(ti);
            scalar d2i = mvf::sqrDistance(o, q);
            if (d2i < d2o)
            {
                d2o = d2i;
                to = ti;
                dt = 0.5 * dt;
                continue;
            }
        }

        // look for a better t in the lower half of the interval:
        ti = to - 0.5 * dt;
        if (ti >= _mint)
        {
            arr2 q = curvexy(ti);
            scalar d2i = mvf::sqrDistance(o, q);
            if (d2i < d2o)
            {
                d2o = d2i;
                to = ti;
                dt = 0.5 * dt;
                continue;
            }
        }

        // if we could not find anything better, half the interval
        dt = 0.5 * dt;
    }

    p = curvexy(to);
    return to;
}


scalar parametric::pGetCurvature(scalar t) const
{
    auto [xp, yp] = curvePxy(t);
    auto [xpp, ypp] = curveP2xy(t);
    scalar denom = (xp * xp + yp * yp);
    if (mvf::areCloseEnough(0, denom, 1e-12)) return std::numeric_limits<scalar>::max();

    return (xp * ypp - xpp * yp) / (denom * std::sqrt( denom ) );

}

scalar parametric::pGetCurvature(const arr2 &p) const
{
    scalar t;
    if (!getTGivenXY(t, p)) return -1;
    return pGetCurvature(t);
}

