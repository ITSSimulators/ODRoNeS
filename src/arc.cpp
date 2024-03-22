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

#include "arc.h"
using namespace odrones;

arc::arc()
{
    geometry::base();
    arc::base();
}

arc::arc(const arc& a)
{
    assignInputGeomToThis(a);
}

void arc::base()
{
    _centre = {0., 0.};
    _co = {0., 0.};
    _cd = {0., 0.};
    _alpha = 0;
    _radiusOfCurvature = 0;
    _odrRoOR = 1;
    _pending = false;
}


arc::arc(const OneVersion::segment &sgm, scalar offset)
{

    _centre = {sgm.centreX, sgm.centreY};
    _o = {sgm.start.x, sgm.start.y};
    _d = {sgm.end.x, sgm.end.y};
    _to = {sgm.start.tx, sgm.start.ty};

    mvf::tangent(_co, _centre, _o);
    mvf::tangent(_cd, _centre, _d);

    arr2 no = { -_to[1], _to[0] };
    _origin = {_o[0] + offset * no[0], _o[1] + offset * no[1] };
    arr2 ne = { - sgm.end.ty, sgm.end.tx};
    _dest = {_d[0] + offset * ne[0], _d[1] + offset * ne[1] };

    _alpha = mvf::subtendedAngle(_co, _cd);
    _radiusOfCurvature = mvf::distance(_centre, _origin); // sgm.radius;

    if (_alpha < 0)
    {
        _shape = mvf::shape::clockwise;
        _radiusOfCurvature*= -1;
    }
    else
        _shape = mvf::shape::counterclockwise;

    _length = std::abs(_alpha * _radiusOfCurvature);

    mvf::boundingBoxForArc(_blc, _trc, _origin, _dest, _centre, mvf::distance(_origin, _centre), _shape);

    _odrRoOR = 1;

    _ready = true;
}

arc::arc(const Odr::geometry &odg, int sign, scalar offsetA, scalar so, scalar se)
{
    arr2 ti = {std::cos(odg.hdg), std::sin(odg.hdg)};
    _o = {odg.x, odg.y};

    arr2 coi = {ti[1], -ti[0]};
    _shape = mvf::shape::counterclockwise;
    int confusion = 1;
    if (odg.curvature < 0)
    {
        coi = {-ti[1], ti[0]};
        _shape = mvf::shape::clockwise;
        confusion = -1;
    }
    scalar curv = std::abs(odg.curvature);
    _centre = {_o[0] - coi[0] / curv,
               _o[1] - coi[1] / curv};

    scalar ao = odg.curvature * so;
    _co = coi;
    mvf::rotateVectorByAngle(_co, ao);
    _origin = {_centre[0] + _co[0] * ( 1. / curv - confusion * sign * offsetA),
               _centre[1] + _co[1] * ( 1. / curv - confusion * sign * offsetA)};

    scalar ae = odg.curvature * se;
    _cd = coi;
    mvf::rotateVectorByAngle(_cd, ae);
    _dest = {_centre[0] + _cd[0] * ( 1. / curv - confusion * sign * offsetA),
             _centre[1] + _cd[1] * ( 1. / curv - confusion * sign * offsetA)};

    _to = ti;
    mvf::rotateVectorByAngle(_to, ao);

    _alpha = odg.curvature * (se - so);

    _d = {_centre[0] + _cd[0] / curv,
          _centre[1] + _cd[1] / curv};

    // We should have g.radiusOfCurvature to be the real mvf::distance(g.origin, g.centre),
    //   and the real length, stored as alpha * radiusOfCurvature.
    _radiusOfCurvature = mvf::distance(_origin, _centre);
    if (_shape == mvf::shape::clockwise) _radiusOfCurvature = - _radiusOfCurvature;
    _length = std::abs(_alpha * _radiusOfCurvature);

    _odrRoOR = 1. / (curv * std::fabs(_radiusOfCurvature));

    mvf::boundingBoxForArc(_blc, _trc, _origin, _dest, _centre, mvf::distance(_origin, _centre), _shape);

    _ready = true;


    /* new, temporary variables
    arr2 ti = {std::cos(odg.hdg), std::sin(odg.hdg)};
    arr2 newO = {odg.x, odg.y};

    arr2 coi = {ti[1], -ti[0]};
    mvf::shape newShape = mvf::shape::counterclockwise;
    int confusion = 1;
    if (odg.curvature < 0)
    {
        coi = {-ti[1], ti[0]};
        newShape = mvf::shape::clockwise;
        confusion = -1;
    }
    scalar curv = std::abs(odg.curvature);
    arr2 newCentre = {newO[0] - coi[0] / curv,
               newO[1] - coi[1] / curv};

    scalar ao = odg.curvature * so;
    arr2 newCo = coi;
    mvf::rotateVectorByAngle(newCo, ao);
    arr2 newOrigin = {newCentre[0] + newCo[0] * ( 1. / curv - confusion * sign * offsetA),
                      newCentre[1] + newCo[1] * ( 1. / curv - confusion * sign * offsetA)};

    scalar ae = odg.curvature * se;
    arr2 newCd = coi;
    mvf::rotateVectorByAngle(newCd, ae);
    arr2 newDest = {newCentre[0] + newCd[0] * ( 1. / curv - confusion * sign * offsetA),
                    newCentre[1] + newCd[1] * ( 1. / curv - confusion * sign * offsetA)};

    arr2 newTo = ti;
    mvf::rotateVectorByAngle(newTo, ao);

    scalar newAlpha = odg.curvature * (se - so);

    arr2 newD = {newCentre[0] + newCd[0] / curv,
         newCentre[1] + newCd[1] / curv};

    // We should have g.radiusOfCurvature to be the real mvf::distance(g.origin, g.centre),
    //   and the real length, stored as alpha * radiusOfCurvature.
    scalar newRad = mvf::distance(newOrigin, newCentre);
    if (newShape == mvf::shape::clockwise) newRad = - newRad;
    scalar newLength = std::abs(newAlpha * newRad);
    */


    /* Classic
    _to = {std::cos(odg.hdg), std::sin(odg.hdg)};
    arr2 no = {- sign * _to[1], sign * _to[0]};
    _o = {odg.x, odg.y};
    _origin = {odg.x + no[0] * offsetA, odg.y + no[1] * offsetA};

    if (odg.curvature > 0)
    {
        _co = {_to[1], -_to[0]};
        _shape = mvf::shape::counterclockwise;
    }
    else
    {
        _co = {-_to[1], _to[0]};
        _shape = mvf::shape::clockwise;
    }
    curv = std::abs(odg.curvature);
    _centre = {_o[0] - _co[0] / curv,
              _o[1] - _co[1] / curv};

    _alpha = odg.curvature * odg.length;
    _cd = _co;
    mvf::rotateVectorByAngle(_cd, _alpha);

    _d = {_centre[0] + _cd[0] / curv,
         _centre[1] + _cd[1] / curv};

    arr2 te = _to;
    mvf::rotateVectorByAngle(te, _alpha);
    arr2 ne = {- sign * te[1], sign * te[0]};

    _dest = {_d[0] + ne[0] * offsetA, _d[1] + ne[1] * offsetA};

    // We should have g.radiusOfCurvature to be the real mvf::distance(g.origin, g.centre),
    //   and the real length, stored as alpha * radiusOfCurvature.
    _radiusOfCurvature = mvf::distance(_origin, _centre);
    if (_shape == mvf::shape::clockwise) _radiusOfCurvature = -_radiusOfCurvature;
    _length = std::abs(_alpha * _radiusOfCurvature);
    */

    /*
    int err = 0;
    if (!mvf::areSamePoints(_o, newO))
    {
        std::cerr << "wrong _o!" << std::endl;
        err = 1;
    }
    if (!mvf::areSamePoints(_d, newD))
    {
        std::cerr << "wrong _d!" << std::endl;
        err = 1;
    }
    if (!mvf::areSamePoints(_centre, newCentre))
    {
        std::cerr << "wrong _centre!" << std::endl;
        err = 1;
    }
    if (!mvf::areSamePoints(_origin, newOrigin))
    {
        std::cerr << "wrong _origin!" << std::endl;
        err = 1;
    }
    if (!mvf::areSamePoints(_dest, newDest))
    {
        std::cerr << "wrong _dest!" << std::endl;
        err = 1;
    }
    if (!mvf::areSameValues(_length, newLength))
    {
        std::cerr << "wrong _length!" << std::endl;
        err = 1;
    }
    if (!mvf::areSameValues(_alpha, newAlpha))
    {
        std::cerr << "wrong _alpha!" << std::endl;
        err = 1;
    }
    if (!mvf::areSamePoints(_cd, newCd))
    {
        std::cerr << "wrong _cd!" << std::endl;
        err = 1;
    }
    if (!mvf::areSamePoints(_co, newCo))
    {
        std::cerr << "wrong _co!" << std::endl;
        err = 1;
    }

    std::cout << "err: " << err << std::endl;
    */


}


arc::arc(const arr2& origin, const arr2& dest)
{
    geometry::base();
    _centre = {0., 0.};
    _co = {0., 0.};
    _cd = {0., 0.};
    _alpha = 0;
    _radiusOfCurvature = 0;
    _origin = origin;
    _dest = dest;
    _ready = false;
    _odrRoOR = 1;
    _pending = true;

}

arc::arc(const arr2& origin, const arr2& dest, const arr2& centre, mvf::shape s)
{
    _origin = origin;
    _dest = dest;
    _centre = centre;
    _shape = s;

    // 1 - set the tangent at the origin:
    scalar theta = std::atan2(_origin[1] - _centre[1], _origin[0] - _centre[0]);
    // here _to comes from the derivative of the parametric eqs., that grow ccw
    _to = {-std::sin(theta), std::cos(theta)};
    //  so it has to be rotated by pi if going cw:
    if (_shape == mvf::shape::clockwise) _to = { -_to[0], -_to[1] };


    // 2 - Get the radius of curvature:
    arr2 od = {_dest[0] - _origin[0], _dest[1] - _origin[1]};
    scalar m = mvf::magnitude(od);
    od[0] = od[0] / m;
    od[1] = od[1] / m;
    scalar sinPsi = - od[0]*_to[1] + od[1]*_to[0];
    _radiusOfCurvature = m / (2 * sinPsi);


    // 3 - find the total angle:
    //     and remember _co and _cd;
    scalar rm1 = 1. / std::fabs(_radiusOfCurvature);
    _co = {(_origin[0] - _centre[0]) * rm1, (_origin[1] - _centre[1]) * rm1};
    _cd = {(_dest[0] - _centre[0]) * rm1, (_dest[1] - _centre[1]) * rm1};
    _alpha = mvf::subtendedAngle4pi(_co, _cd, _shape);

    // 3.1 - and use the angle to get the length of the lane:
    _length = fabs( _alpha * _radiusOfCurvature );


    // 4 - checking and printouts:
    arr2 shouldBecd = {_co[0], _co[1]};
    mvf::rotateVectorByAngle(shouldBecd, _alpha);
    if (!mvf::areSamePoints(shouldBecd, _cd))
        std::cerr << " ERROR when constructing an arc" << std::endl;

    arr2 itShouldBeDest;
    itShouldBeDest[0] = _centre[0] + fabs(_radiusOfCurvature) * shouldBecd[0];
    itShouldBeDest[1] = _centre[1] + fabs(_radiusOfCurvature) * shouldBecd[1];
    if (!mvf::areSamePoints(itShouldBeDest, _dest))
        std::cerr << " ERROR when constructing an arc: err 2" << std::endl;


    mvf::boundingBoxForArc(_blc, _trc, _origin, _dest, _centre, mvf::distance(_origin, _centre), _shape);

    _o = _origin;
    _d = _dest;

    _odrRoOR = 1;

    _ready = true;
}

void arc::setTo(arr2 to)
{
    arc tmp(_origin, _dest, to);
    assignInputGeomToThis(tmp);
    return;
}

arc& arc::operator=(const arc &a)
{
    assignInputGeomToThis(a);
    return *this;
}

void arc::assignInputGeomToThis(const arc &a)
{
    _centre = a._centre;
    _alpha = a._alpha;
    _radiusOfCurvature = a._radiusOfCurvature;
    _co = a._co;
    _cd = a._cd;
    _odrRoOR = a._odrRoOR;
    geometry::assignInputGeomToThis(a);
}


void arc::invert()
{
    arr2 tmp = getTangentInPoint(_dest);
    _to = {-tmp[0], -tmp[1]};

    // swap origin and destination
    tmp = _dest;
    _dest = _origin;
    _origin = tmp;

    // swap o and d:
    tmp = _d;
    _d = _o;
    _o = tmp;

    // swap co and cd:
    tmp = _cd;
    _cd = _co;
    _co = tmp;

    // change the angle of alpha:
    _alpha = -_alpha;

    if (_shape == mvf::shape::clockwise)
    {
        _shape = mvf::shape::counterclockwise;
        _radiusOfCurvature = - _radiusOfCurvature;
    }
    else if (_shape == mvf::shape::counterclockwise)
    {
        _shape = mvf::shape::clockwise;
        _radiusOfCurvature = - _radiusOfCurvature;
    }

}

arc::arc(const arr2& origin, const arr2& dest, const arr2& to)
{
    _origin = origin;
    _dest = dest;
    _to = to;
    mvf::normalise(_to); // well, just in case.

    // 1 - Get the radius of curvature:
    arr2 od = {_dest[0] - _origin[0], _dest[1] - _origin[1]};
    scalar m = mvf::magnitude(od);
    od[0] = od[0] / m;
    od[1] = od[1] / m;
    scalar sinPsi = - od[0]*_to[1] + od[1]*_to[0];
    _radiusOfCurvature = m / (2 * sinPsi);
    // and the shape
    if (_radiusOfCurvature > 0) _shape = mvf::shape::counterclockwise;
    else _shape = mvf::shape::clockwise;


    // 2 - get the centre of the circle:
    // 2.1 - rotate _to by 90 degrees towards the centre of the circle:
    arr2 oc = {_to[0], _to[1]};
    mvf::rotateVectorByAngle(oc, constants::pi / 2);
    // 2.2 - go from the origin to the centre:
    _centre[0] = _origin[0] + _radiusOfCurvature * oc[0];
    _centre[1] = _origin[1] + _radiusOfCurvature * oc[1];


    // 3 - find the total angle:
    _co[0] = _origin[0] - _centre[0];
    _co[1] = _origin[1] - _centre[1];
    mvf::normalise(_co);
    _cd[0] = _dest[0] - _centre[0];
    _cd[1] = _dest[1] - _centre[1];
    mvf::normalise(_cd);
    _alpha = mvf::subtendedAngle4pi(_co, _cd, _shape);

    // 4 - and use the angle to get the length of the lane:
    _length = fabs( _alpha * _radiusOfCurvature );


    // 4 - checking and printouts:
    arr2 shouldBecd = {_co[0], _co[1]};
    mvf::rotateVectorByAngle(shouldBecd, _alpha);
    if (!mvf::areSamePoints(shouldBecd, _cd))
        std::cerr << " ERROR when constructing an arc" << std::endl;

    arr2 itShouldBeDest;
    itShouldBeDest[0] = _centre[0] + fabs(_radiusOfCurvature) * shouldBecd[0];
    itShouldBeDest[1] = _centre[1] + fabs(_radiusOfCurvature) * shouldBecd[1];
    if (!mvf::areSamePoints(itShouldBeDest, _dest))
        std::cerr << " ERROR when constructing an arc" << std::endl;


    mvf::boundingBoxForArc(_blc, _trc, _origin, _dest, _centre, mvf::distance(_origin, _centre), _shape);

    _o = _origin;
    _d = _dest;

    _odrRoOR = 1;

    _ready = true;

}


bool arc::isPointHere(const arr2 &p) const
{
    return mvf::isPointOnArc(p, _centre, _co, _radiusOfCurvature, _alpha);
}


arr2 arc::projectPointHere(const arr2 &p) const
{
    return mvf::projectPointToArc(p, _origin, _centre, _co, _alpha);
}

arr2 arc::getTangentInPoint(const arr2 &p) const
{
    // Set t as the starting tangent, and rotate it:
    arr2 t = _to;
    arr2 cp = {p[0] - _centre[0], p[1] - _centre[1]};
    mvf::normalise(cp);
    mvf::rotateVectorByAngle(t, mvf::subtendedAngle(_co, cp));
    return t;
}

scalar arc::distanceToTheEoL(const arr2 &p) const
{
    arr2 cp = {p[0] - _centre[0], p[1] - _centre[1]};
    mvf::normalise(cp);
    scalar beta = mvf::subtendedAngle(cp, _cd);
    return fabs ( beta * _radiusOfCurvature );

}

bool arc::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    if (d > _length) return false; // there's nothing to do, if d is too big.

    // get the angle given the distance:
    scalar beta = d / _radiusOfCurvature;
    // if (_shape == mvf::shape::clockwise) beta = -beta; // that may be a great disaster
    // rotate the vector centre to origin by this amount:
    arr2 ci = {o[0] - _centre[0], o[1] - _centre[1]};
    mvf::normalise(ci);
    mvf::rotateVectorByAngle(ci, beta);
    p[0] = _centre[0] + fabs(_radiusOfCurvature) * ci[0];
    p[1] = _centre[1] + fabs(_radiusOfCurvature) * ci[1];

    if (isPointHere(p)) return true;
    return false;

}

bool arc::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    return mvf::getIntersectionPointToArc(p, o, t, _centre, _co, _radiusOfCurvature, _alpha);
}

scalar arc::getCurvature([[maybe_unused]] const arr2 &p) const
{
    return 1./_radiusOfCurvature;
}

scalar arc::sl0(scalar s) const
{
    return s * _odrRoOR + _roadSo;
}

#ifdef QT_CORE_LIB
QPainterPath arc::getQPainterPath(uint n) const
{
    QPainterPath qpp;
    scalar dl = _length / scalar(n);
    qpp.moveTo(ct::mToPix * _origin[0], -ct::mToPix * _origin[1]);
    for (uint i = 0; i <= n; ++i)
    {
        arr2 p;
        getPointAfterDistance(p, _origin, i * dl);
        qpp.lineTo(QPointF(ct::mToPix * p[0], -ct::mToPix * p[1]));
    }
    qpp.lineTo(ct::mToPix * _dest[0], -ct::mToPix * _dest[1]);
    return qpp;
}
/* The approach below does not always align perfectly on long radius curves.
QPainterPath arc::getQPainterPath([[maybe_unused]] uint n) const
{
    QPainterPath qpp;
    qpp.moveTo(ct::mToPix * _origin[0], -ct::mToPix * _origin[1]);
    scalar roc = std::fabs(_radiusOfCurvature);
    QRectF rectangle(ct::mToPix * ( _centre[0] - roc ), - ct::mToPix * ( _centre[1] + roc ),
            ct::mToPix * ( 2 * roc ), ct::mToPix *( 2 * roc ));
    qreal startingFrom = static_cast<qreal>( - ct::rad2deg * atan2(- _co[1], _co[0]));
    qreal angle = static_cast<qreal>( ct::rad2deg * _alpha);
    qpp.arcTo(rectangle, startingFrom, angle);
    return qpp;
}
*/

#endif




