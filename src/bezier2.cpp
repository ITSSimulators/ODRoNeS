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

#include "bezier2.h"

bezier2::bezier2()
{

}

/* bezier2::bezier2(const arr2 &p0, const arr2 &p1, const arr2 &p2)
{
    set(p0, p1, p2);
}*/


bezier2::bezier2(const bezier2& b)
{
    assignInputLaneToThis(b);
}

bezier2::~bezier2()
{

}

bezier2& bezier2::operator=(const bezier2 &b)
{
    clearMem();
    assignInputLaneToThis(b);
    return *this;
}

void bezier2::assignInputLaneToThis(const bezier2 &b)
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
    geometry::assignInputGeomToThis(b);
}

void bezier2::set(const arr2 &p0, const arr2 &p1, const arr2 &p2)
{
    _degree = 2;
    allocateMem(_degree);
    _wx[0] = p0[0];
    _wx[1] = p1[0];
    _wx[2] = p2[0];
    _wy[0] = p0[1];
    _wy[1] = p1[1];
    _wy[2] = p2[1];

    _origin = p0; // curvexy(0);
    _dest = p2; // curvexy(1);
    _shape = mvf::shape::bezier2;
    _length = calcLength();
    _to = mvf::tangent(p0, p1); // curvePxy(0);
    getBoundingBox(_blc, _trc);

    _ready = true;
}

scalar bezier2::curvex(scalar t) const
{
    return _wx[0] * (1-t) * (1-t) + _wx[1] * 2 * (1-t) * t + _wx[2] * t * t;
}

scalar bezier2::curvey(scalar t) const
{
    return _wy[0] * (1-t) * (1-t) + _wy[1] * 2 * (1-t) * t + _wy[2] * t * t;
}

arr2 bezier2::curvexy(scalar t) const
{
    scalar mt2 = (1-t) * (1-t);
    scalar t2 = t * t;
    scalar twice_tmt = 2 * t * (1-t);
    return { _wx[0] * mt2 + _wx[1] * twice_tmt + _wx[2] * t2,
             _wy[0] * mt2 + _wy[1] * twice_tmt + _wy[2] * t2};
}

scalar bezier2::curvePx(scalar t) const
{
    return 2 * ( (_wx[1] - _wx[0]) * (1-t) + (_wx[2] - _wx[1]) * t);
}

scalar bezier2::curvePy(scalar t) const
{
    return 2 * ( (_wy[1] - _wy[0]) * (1-t) + (_wy[2] - _wy[1]) * t);
}

arr2 bezier2::curvePxy(scalar t) const
{
    return { 2* ( (_wx[1] - _wx[0]) * (1-t) + (_wx[2] - _wx[1]) * t),
             2* ( (_wy[1] - _wy[0]) * (1-t) + (_wy[2] - _wy[1]) * t)};
}

scalar bezier2::curveP2x([[maybe_unused]] scalar t) const
{
    return 2 * ( _wx[0] - 2*_wx[1] + _wx[2] );
}

scalar bezier2::curveP2y([[maybe_unused]] scalar t) const
{
    return 2 * ( _wy[0] - 2*_wy[1] + _wy[2] );
}

arr2 bezier2::curveP2xy([[maybe_unused]] scalar t) const
{
    return {2 * ( _wx[0] - 2*_wx[1] + _wx[2] ),
            2 * ( _wy[0] - 2*_wy[1] + _wy[2] ) };
}

bool bezier2::rootPx(scalar &t) const
{
    scalar dnm = _wx[0] - 2 * _wx[1] + _wx[2];
    if (mvf::areSameValues(dnm, 0)) return false;
    t = (_wx[0] - _wx[1]) / dnm;
    if (!isTinRange(t)) return false;
    return true;
}

bool bezier2::rootPy(scalar &t) const
{
    scalar dnm = _wy[0] - 2 * _wy[1] + _wy[2];
    if (mvf::areSameValues(dnm, 0)) return false;
    t = (_wy[0] - _wy[1]) / dnm;
    if (!isTinRange(t)) return false;
    return true;
}



bezier2 bezier2::getStartingPart(scalar t) const
{
    arr2 p0 = {_wx[0], _wy[0]};
    arr2 p1 = {t*_wx[1] - (t-1)*_wx[0], t*_wy[1] - (t-1)*_wy[0]};
    arr2 p2 = {t*t*_wx[2] - 2*t*(t-1)*_wx[1] + (t-1)*(t-1)*_wx[0],
               t*t*_wy[2] - 2*t*(t-1)*_wy[1] + (t-1)*(t-1)*_wy[0]};
    bezier2 bz2;
    bz2.set(p0, p1, p2);
    return bz2;
}

bezier2 bezier2::getEndingPart(scalar t) const
{
    arr2 p0 = {t*t*_wx[2] - 2*t*(t-1)*_wx[1] + (t-1)*(t-1)*_wx[0],
               t*t*_wy[2] - 2*t*(t-1)*_wy[1] + (t-1)*(t-1)*_wy[0]};
    arr2 p1 = {t*_wx[2] - (t-1)*_wx[1], t*_wy[2] - (t-1)*_wy[1]};
    arr2 p2 = {_wx[2], _wy[2]};
    bezier2 bz2;
    bz2.set(p0, p1, p2);
    return bz2;
}


bool bezier2::getTGivenXY(scalar &t, const arr2 &xy) const
{
    // the Bezier equation can be put in this form:
    // B(t) = (w[2] - 2*w[1] + w[0])t**2 + 2(w[1] - w[0])t + w[0];
    // Thus we solve for B(t) - x = 0, and find t1, t2,
    scalar t1, t2;
    if (! (mvf::solve2ndOrderEq(t1, t2, _wx[2] - 2*_wx[1] + _wx[0],
                                2*(_wx[1] - _wx[0]), _wx[0] - xy[0])))
        return false;

    // std::cout << "[2] " << (_wx[2] - 2*_wx[1] + _wx[0])*t1*t1 + 2*(_wx[1] - _wx[0])*t1 + _wx[0] << " = " << xy[0] << std::endl;
    if ((!isTinRangeFix(t1)) && (!isTinRangeFix(t2))) return false;

    scalar t3, t4;
    if (! (mvf::solve2ndOrderEq(t3, t4, _wy[2] - 2*_wy[1] + _wy[0],
                                2*(_wy[1] - _wy[0]), _wy[0] - xy[1])))
        return false;

    if (isTinRange(t1))
    {
        if ((mvf::areSameValues(t1, t3)) || mvf::areSameValues(t1, t4))
        {
            t = t1;
            return true;
        }
    }

    if (isTinRange(t2))
    {
        if ((mvf::areSameValues(t2, t3)) || mvf::areSameValues(t2, t4))
        {
            t = t2;
            return true;
        }
    }
    return false;
}

bool bezier2::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    // Firstly put translate the curve, and o into (0, 0)
    constexpr uint d = 3;
    scalar wxo[d], wyo[d];
    for (uint i = 0; i < d; ++i)
    {
        wxo[i] = _wx[i] - o[0];
        wyo[i] = _wy[i] - o[1];
    }
    // And rotate it so that t is along the x axis:
    scalar theta = -std::atan2(t[1],t[0]);
    scalar wxr[d], wyr[d];
    for (uint i = 0; i < d; ++i)
    {
        wxr[i] =   wxo[i] * std::cos(theta) - wyo[i] * std::sin(theta);
        wyr[i] = + wxo[i] * std::sin(theta) + wyo[i] * std::cos(theta);
    }


    // Now these would be used to create a new Bezier curve.
    //   and we would look for those t values where By(t) == 0:
    scalar t1, t2;
    if (!mvf::solve2ndOrderEq(t1, t2, wyr[2] - 2*wyr[1] + wyr[0],2*(wyr[1] - wyr[0]), wyr[0]))
        return false;
    uint answers = reorderTinRangeFix(t1, t2);
    if (answers == 0) return false;


    // If there are more than one ts, we want the one that is closer to o, i e, with x positive and closer to 0
    bezier2 auxb2;
    auxb2.set({wxr[0], wyr[0]}, {wxr[1], wyr[1]}, {wxr[2], wyr[2]});
    scalar qx = auxb2.curvex(t1);
    if (qx > 0)
    {
        if (answers > 1)
        {
            scalar x = auxb2.curvex(t2);
            if ((x >= 0) && (x < qx))
            {
                p = curvexy(t2);
                return true;
            }
        }
        p = curvexy(t1);
        return true;
    }
    else if (answers > 1)
    {
        qx = auxb2.curvex(t2);
        if (qx > 0)
        {
            p = curvexy(t2);
            return true;
        }
    }
    return false;

}



void bezier2::getBoundingBox(arr2 &blc, arr2 &trc) const
{
    calcBoundingBoxO2(blc, trc);
}

bool bezier2::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
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

    bezier2 endB = getEndingPart(to);
    if (mvf::areSameValues(d, endB.getLength()))
    {
        p = curvexy(1);
        return true;
    }
    if (d > endB.getLength()) return false;  // that means the point would be after the end of this line.

    // get the parametre t (te) after d:
    scalar te;
    if (!endB.getTgivenD(te, d)) return false; // and if false, then it was too long.

    p = endB.curvexy(te);
    return true;
}


scalar bezier2::calcLength() const
{
    return  calcLengthGQ20();
}



scalar bezier2::calcLength(scalar t) const
{
    uint order = 20;
    scalar l = 0;
    for (uint i = 0; i < order; ++i)
    {
        arr2 dr = curvePxy(t*0.5*(ct::gc::A20[i]+1));
        l += ct::gc::W20[i] * std::sqrt(dr[0] * dr[0] + dr[1] * dr[1]);
    }
    l *= t*0.5;
    return l;
}
