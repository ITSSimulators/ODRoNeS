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

#include "straight.h"

straight::straight(const straight& s)
{
    assignInputGeomToThis(s);
}

straight::straight(const arr2& origin, const arr2& dest)
{
    _origin = origin;
    _dest = dest;
    _shape = mvf::shape::straight;
    _length = mvf::distance(origin, dest);
    mvf::tangent(_to, _origin, _dest);
    mvf::boundingBoxFromTwoPoints(_blc, _trc, _origin, _dest);

    _o = origin;
    _d = dest;

    _ready = true;
}

straight::straight(const OneVersion::segment &sgm)
{
     assignInputGeomToThis(straight({sgm.start.x, sgm.start.y}, {sgm.end.x, sgm.end.y}));

     if (! mvf::areSameValues(_length, sgm.length) )
     {
         std::cout << "Wrong length!" << std::endl;
     }

     if (! mvf::areCloseEnough(1, mvf::scalarProduct(_to, {sgm.start.tx, sgm.start.ty}), 1e-8) )
     {
         std::cout << "Wrong tangent!" << std::endl;
     }
}

straight::straight(const Odr::geometry &odg, int sign, scalar offsetA, scalar so, scalar se)
{
    // Firstly make sure that so and se fall in range:
    if (!mvf::isInRangeLR(so, 0, odg.length))
        std::cerr << "so is out of bonds!!" << std::endl;

    if (!mvf::isInRangeLR(se, 0, odg.length))
        std::cerr << "se is out of bonds!!" << std::endl;

    // Now _to, _o and _origin:
    _to = {std::cos(odg.hdg), std::sin(odg.hdg)};
    arr2 no = {- sign * _to[1], sign * _to[0]};
    _o = {odg.x, odg.y};
    _origin = {odg.x + _to[0] * so + no[0] * offsetA, odg.y + _to[1] * so + no[1] * offsetA};

    // and _d, _dest and the length:
    _d = {_o[0] + _to[0] * odg.length, _o[1] + _to[1] * odg.length};
    _dest = {odg.x + _to[0] * se + no[0] * offsetA, odg.y + _to[1] * se + no[1] * offsetA};
    // std::cout << "_origin: (" << g.origin[0] << ", " << g.origin[1] << ")"
    //           << "_dest: (" << g.dest[0] << ", " << g.dest[1] << ")" << std::endl;
    _length = se - so;

    _shape = mvf::shape::straight;
    mvf::boundingBoxFromTwoPoints(_blc, _trc, _origin, _dest);

    _ready = true;
}

void straight::invert()
{
    // swap origin and destination
    arr2 tmp = _dest;
    _dest = _origin;
    _origin = tmp;

    // swap o and d:
    tmp = _d;
    _d = _o;
    _o = tmp;

    _to = mvf::tangent(_origin, _dest);
}


bool straight::isPointHere(const arr2 &p) const
{
    return mvf::isPointOnSegment(p, _origin, _dest);
}


arr2 straight::projectPointHere(const arr2 &p) const
{
    return mvf::projectPointToSegment(p, _origin, _to, _length);
}

arr2 straight::getTangentInPoint([[maybe_unused]] const arr2 &p) const
{
    return _to;
}

scalar straight::distanceToTheEoL(const arr2 &p) const
{
    return mvf::distance(p, _dest);
}

bool straight::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    arr2 t = getTangentInPoint(o);
    p[0] = o[0] + d * t[0];
    p[1] = o[1] + d * t[1];
    if (isPointHere(p)) return true;
    return false;
}

bool straight::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    return mvf::getIntersectionPointToSegment(p, o, t, _origin, _to, _length);
}

scalar straight::getCurvature([[maybe_unused]] const arr2 &p) const
{
    return 0;
}

#ifdef QT_CORE_LIB
QPainterPath straight::getQPainterPath([[maybe_unused]] uint n) const
{
    QPainterPath qpp;
    qpp.moveTo(ct::mToPix * _origin[0], -ct::mToPix * _origin[1]);
    qpp.lineTo(ct::mToPix * _dest[0], -ct::mToPix * _dest[1]);
    return qpp;
}
#endif
