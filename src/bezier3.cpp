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

#include "bezier3.h"
using namespace odrones;

bezier3::bezier3()
{

}

bezier3::bezier3(const arr2 &p0, const arr2 &p1, const arr2 &p2, const arr2 &p3)
{
    set(p0, p1, p2, p3);
}


bezier3::bezier3(const bezier3& b)
{
    assignInputLaneToThis(b);
}

bezier3::~bezier3()
{

}

bezier3& bezier3::operator=(const bezier3 &b)
{
    clearMem();
    assignInputLaneToThis(b);
    return *this;
}

void bezier3::assignInputLaneToThis(const bezier3 &b)
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

void bezier3::set(const arr2 &p0, const arr2 &p1, const arr2 &p2, const arr2 &p3)
{
    _degree = 3;
    allocateMem(_degree);
    _wx[0] = p0[0];
    _wx[1] = p1[0];
    _wx[2] = p2[0];
    _wx[3] = p3[0];
    _wy[0] = p0[1];
    _wy[1] = p1[1];
    _wy[2] = p2[1];
    _wy[3] = p3[1];

    _origin = p0; // curvexy(0);
    _dest = p3; // curvexy(1);
    _shape = mvf::shape::bezier3;
    _length = bezier3::calcLength();
    _to = mvf::tangent(p0, p1); //curvePxy(0);
    // mvf::normalise(_to);
    bezier3::getBoundingBox(_blc, _trc);

    _ready = true;
}


scalar bezier3::curvex(scalar t) const
{
    scalar t2 = t * t;
    scalar t3 = t2 * t;
    scalar mt = 1-t;
    scalar mt2 = mt * mt;
    scalar mt3 = mt2 * mt;
    return _wx[0]*mt3 + 3*_wx[1]*mt2*t + 3*_wx[2]*mt*t2 + _wx[3]*t3;
}

scalar bezier3::curvey(scalar t) const
{
    scalar t2 = t * t;
    scalar t3 = t2 * t;
    scalar mt = 1-t;
    scalar mt2 = mt * mt;
    scalar mt3 = mt2 * mt;
    return _wy[0]*mt3 + 3*_wy[1]*mt2*t + 3*_wy[2]*mt*t2 + _wy[3]*t3;
}

arr2 bezier3::curvexy(scalar t) const
{
    scalar t2 = t * t;
    scalar t3 = t2 * t;
    scalar mt = 1-t;
    scalar mt2 = mt * mt;
    scalar mt3 = mt2 * mt;
    return { _wx[0]*mt3 + 3*_wx[1]*mt2*t + 3*_wx[2]*mt*t2 + _wx[3]*t3,
             _wy[0]*mt3 + 3*_wy[1]*mt2*t + 3*_wy[2]*mt*t2 + _wy[3]*t3 };

}

scalar bezier3::curvePx(scalar t) const
{
    scalar t2 = t * t;
    scalar mt = 1-t;
    scalar mt2 = mt * mt;
    return 3*(_wx[1]-_wx[0])*mt2 + 6*(_wx[2]-_wx[1])*mt*t + 3*(_wx[3]-_wx[2])*t2;
}

scalar bezier3::curvePy(scalar t) const
{
    scalar t2 = t * t;
    scalar mt = 1-t;
    scalar mt2 = mt * mt;
    return 3*(_wy[1]-_wy[0])*mt2 + 6*(_wy[2]-_wy[1])*mt*t + 3*(_wy[3]-_wy[2])*t2;
}

arr2 bezier3::curvePxy(scalar t) const
{
    scalar t2 = t * t;
    scalar mt = 1-t;
    scalar mt2 = mt * mt;
    return { 3*(_wx[1]-_wx[0])*mt2 + 6*(_wx[2]-_wx[1])*mt*t + 3*(_wx[3]-_wx[2])*t2,
             3*(_wy[1]-_wy[0])*mt2 + 6*(_wy[2]-_wy[1])*mt*t + 3*(_wy[3]-_wy[2])*t2 };
}

scalar bezier3::curveP2x(scalar t) const
{
    return 6 * ( (_wx[1]-_wx[0])*(t-1) + (_wx[2]-_wx[1])*(1-2*t) + (_wx[3]-_wx[2])*t );
}

scalar bezier3::curveP2y(scalar t) const
{
    return 6 * ( (_wy[1]-_wy[0])*(t-1) + (_wy[2]-_wy[1])*(1-2*t) + (_wy[3]-_wy[2])*t );
}

arr2 bezier3::curveP2xy(scalar t) const
{
    return { 6 * ( (_wx[1]-_wx[0])*(t-1) + (_wx[2]-_wx[1])*(1-2*t) + (_wx[3]-_wx[2])*t ),
             6 * ( (_wy[1]-_wy[0])*(t-1) + (_wy[2]-_wy[1])*(1-2*t) + (_wy[3]-_wy[2])*t )};
}


uint bezier3::rootPx(scalar &t1, scalar &t2) const
{
    if (!mvf::solve2ndOrderEq(t1, t2, _wx[3] - 3*_wx[2] + 3*_wx[1] -_wx[0], 2*(_wx[2] - 2*_wx[1] + _wx[0]), _wx[1] - _wx[0])) return 0;
    return reorderTinRangeFix(t1, t2);
}

uint bezier3::rootPy(scalar &t1, scalar &t2) const
{
    if (!mvf::solve2ndOrderEq(t1, t2, _wy[3] - 3*_wy[2] + 3*_wy[1] -_wy[0], 2*(_wy[2] - 2*_wy[1] + _wy[0]), _wy[1] - _wy[0])) return 0;
    return reorderTinRangeFix(t1, t2);
}


bezier3 bezier3::getStartingPart(scalar t) const
{
    scalar t2 = t*t;
    scalar t3 = t2*t;
    scalar tm = (t-1);
    scalar tm2 = (t-1)*(t-1);
    scalar tm3 = tm2 * tm;

    arr2 p0 = {_wx[0], _wy[0]};
    arr2 p1 = {t*_wx[1] - (t-1)*_wx[0], t*_wy[1] - (t-1)*_wy[0]};
    arr2 p2 = {t2*_wx[2] - 2*t*(t-1)*_wx[1] + tm2*_wx[0],
               t2*_wy[2] - 2*t*(t-1)*_wy[1] + tm2*_wy[0]};
    arr2 p3 = {t3*_wx[3] - 3*t2*tm*_wx[2] +3*t*tm2*_wx[1] - tm3*_wx[0],
               t3*_wy[3] - 3*t2*tm*_wy[2] +3*t*tm2*_wy[1] - tm3*_wy[0]};

    bezier3 bz3;
    bz3.set(p0, p1, p2, p3);
    return bz3;
}

bezier3 bezier3::getEndingPart(scalar t) const
{

    scalar t2 = t*t;
    scalar t3 = t2*t;
    scalar tm = (t-1);
    scalar tm2 = (t-1)*(t-1);
    scalar tm3 = tm2 * tm;

    arr2 p0 = {t3*_wx[3] - 3*t2*tm*_wx[2] +3*t*tm2*_wx[1] - tm3*_wx[0],
               t3*_wy[3] - 3*t2*tm*_wy[2] +3*t*tm2*_wy[1] - tm3*_wy[0]};
    arr2 p1 = {t2*_wx[3] - 2*t*tm*_wx[2] + tm2*_wx[1],
               t2*_wy[3] - 2*t*tm*_wy[2] + tm2*_wy[1]};
    arr2 p2 = {t*_wx[3] - tm*_wx[2], t*_wy[3] - tm*_wy[2]};
    arr2 p3 = {_wx[3], _wy[3]};


    bezier3 bz3;
    bz3.set(p0, p1, p2, p3);
    return bz3;
}



bool bezier3::getTGivenXY(scalar &t, const arr2 &xy) const
{
    if (mvf::areSamePoints(xy, _origin))
    {
        t = 0;
        return true;
    }
    else if (mvf::areSamePoints(xy, _dest))
    {
        t = _maxt;
        return true;
    }

    // The Bezier equation can be written in this form:
    // B(t) = (w[3] - 3w[2] + 3w[1] - w[0])t**3 + 3(w[2] - 2w[1] + w[0])t**2 + 3(w[1] - w[0])t + w[0]
    // and so we solve for B(t) - x = 0, and find t1, t2 & t3, and then for B(t) - y = 0 and find t4, t5, & t6.
    scalar tx[3];
    // scalar t1, t2, t3;
    uint nx = mvf::solve3rdOrderEq(tx[0], tx[1], tx[2],
            _wx[3] - 3*_wx[2] + 3*_wx[1] - _wx[0],
            3*(_wx[2] - 2*_wx[1] + _wx[0]),
            3*(_wx[1] - _wx[0]),
            _wx[0] - xy[0]);
    if (nx == 0) return false;

    if ((!isTinRangeFix(tx[0])) && (!isTinRangeFix(tx[1])) && (!isTinRangeFix(tx[2]))) return false;

    scalar ty[3];
    uint ny = mvf::solve3rdOrderEq(ty[0], ty[1], ty[2], _wy[3] - 3*_wy[2] + 3*_wy[1] - _wy[0],
                                3*(_wy[2] - 2*_wy[1] + _wy[0]), 3*(_wy[1] - _wy[0]), _wy[0] - xy[1]);
    if (ny == 0) return false;

    if ((!isTinRange(ty[0])) && (!isTinRange(ty[1])) && (!isTinRange(ty[2]))) return false;

    for (uint i = 0; i < nx; ++i)
    {
        if (!isTinRange(tx[i])) continue;
        for (uint j = 0; j < ny; ++j)
        {
            if (mvf::areCloseEnough(tx[i], ty[j], mvf::absolutePrecision))
            {
                t = tx[i];
                return true;
            }
        }
    }
    return false;

}


bool bezier3::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{

    // Firstly put translate the curve, and o into (0, 0)
    constexpr uint d = 4;
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
    scalar t1, t2, t3;
    uint answers = mvf::solve3rdOrderEq(t1, t2, t3,
                                        wyr[3] - 3*wyr[2] + 3*wyr[1] - wyr[0],
                                        3*(wyr[2] - 2*wyr[1] + wyr[0]), 3*(wyr[1] - wyr[0]), wyr[0]);

    if (answers == 0) return false;
    bezier3 auxb3;
    auxb3.set({wxr[0], wyr[0]}, {wxr[1], wyr[1]}, {wxr[2], wyr[2]}, {wxr[3], wyr[3]});
    if (answers == 1)
    {
        if (!isTinRangeFix(t1)) return false;
        if (auxb3.curvex(t1) < 0) return false;
        p = curvexy(t1);
        return true;
    }

    if (answers == 2)
    {
        uint vAnswers = reorderTinRangeFix(t1, t2);
        if (vAnswers == 0) return false;
        scalar qx = auxb3.curvex(t1);
        if (qx > 0)
        {
            if (vAnswers > 1)
            {
                scalar x = auxb3.curvex(t2);
                if ((x >= 0) && (x < qx))
                {
                    p = curvexy(t2);
                    return true;
                }
            }
            p = curvexy(t1);
            return true;
        }
        else if (vAnswers > 1)
        {
            qx = auxb3.curvex(t2);
            if (qx > 0)
            {
                p = curvexy(t2);
                return true;
            }
        }
        return false;
    }

    uint vAnswers = reorderTinRangeFix(t1, t2, t3);
    if (vAnswers == 0) return false;
    else if (vAnswers == 1)
    {
        if (auxb3.curvex(t1) < 0) return false;
        p = curvexy(t1);
        return true;
    }
    else if (vAnswers == 2)
    {
        scalar qx = auxb3.curvex(t1);
        if (qx > 0)
        {
            scalar x = auxb3.curvex(t2);
            if ((x >= 0) && (x < qx))
            {
                p = curvexy(t2);
                return true;
            }
            p = curvexy(t1);
            return true;
        }
        else if (auxb3.curvex(t2) > 0)
        {
            p = curvexy(t2);
            return true;
        }
        return false;
    }
    else if (vAnswers == 3)
    {
        scalar qx = auxb3.curvex(t1);
        if (qx > 0)
        {
            scalar rx = auxb3.curvex(t2);
            scalar x = auxb3.curvex(t3);
            if ((rx > 0) && (rx < qx))
            {
                if ((x >= 0) && (x < rx))
                {
                    p = curvexy(t3);
                    return true;
                }
                p = curvexy(t2);
                return true;
            }
            else if ((x > 0) && (x < qx))
            {
                p = curvexy(t3);
                return true;
            }
            p = curvexy(t1);
            return true;
        }
        else // now it's just t2 or t3:
        {
            scalar rx = auxb3.curvex(t2);
            scalar x = auxb3.curvex(t3);
            if (rx > 0)
            {
                if ((x >= 0) && (x < rx))
                {
                    p = curvexy(t3);
                    return true;
                }
                p = curvexy(t2);
                return true;
            }
            else if (x > 0)
            {
                p = curvexy(t3);
                return true;
            }
            return false;
        }
    }

    return false;
}


void bezier3::getBoundingBox(arr2 &blc, arr2 &trc) const
{
    calcBoundingBoxO3(blc, trc);
}


bool bezier3::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
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

    bezier3 endB = getEndingPart(to);
    if (mvf::areSameValues(d, endB.getLength()))
    {
        p = curvexy(1);
        return true;
    }
    else if (d > endB.getLength()) return false;  // that means the point would be after the end of this line.

    // get the parametre t (te) after d:
    scalar te;
    if (!endB.getTgivenD(te, d)) return false; // and if false, then it was too long.

    p = endB.curvexy(te);
    return true;
}


scalar bezier3::calcLength() const
{
    return calcLengthGQ30();
}


scalar bezier3::calcLength(scalar t) const
{
    uint order = 30;
    scalar l = 0;
    for (uint i = 0; i < order; ++i)
    {
        arr2 dr = curvePxy(t*0.5*(ct::gc::A30[i]+1));
        l += ct::gc::W30[i] * std::sqrt(dr[0] * dr[0] + dr[1] * dr[1]);
    }
    l *= t * 0.5;
    return l;
}

