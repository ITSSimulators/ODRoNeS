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

#ifndef ODRONES_MATVEC_H
#define ODRONES_MATVEC_H


#include "constants.h"
#include <cmath>
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <tuple>

namespace odrones
{

// typedef odrones::scalar scalar;
typedef std::array<scalar, 2> arr2;
class segment {
public:
    segment(const std::array<scalar, 2> &a1, const std::array<scalar, 2> &a2)
    {
        p1 = a1; p2 = a2;
    }

public:
    arr2 p1 {0., 0.};
    arr2 p2 {0., 0.};
};

// typedef std::array<scalar, 3> arr3;

class mvf
{
public:
    // 0 - Stuff related to scalars:
    static constexpr scalar absolutePrecision = 1e-6;
    static constexpr scalar relativePrecision = 1e-5;
    static constexpr scalar distPrecision = 5e-4;
    static constexpr scalar distPrecision2 = distPrecision * distPrecision;

    static bool areSameValues(scalar a, scalar b);
    static bool areCloseEnough(scalar a, scalar b, scalar absPrec);
    static bool isInRangeLR(scalar a, scalar lower, scalar higher, scalar absPrec = absolutePrecision); ///< true if a > lower and a < higher within a range
    static bool isInRangeL(scalar a, scalar lower, scalar higher, scalar absPrec = 0); ///< true if a > lower and a < higher within a range
    static bool isInRangeR(scalar a, scalar lower, scalar higher, scalar absPrec = 0); ///< true if a > lower and a < higher within a range
    static bool isInRange0(scalar a, scalar lower, scalar higher);
    static int round(scalar a);
    /*! Will return min(a,b) that is greater than zero, or a negative number (a or b) if both are negative */
    static scalar minPositive(scalar a, scalar b);
    static scalar positiveZero(scalar a); ///< return 0 if a is closeEnough to zero;
    static scalar sqr(scalar a); ///< returns a*a;

    /*! solves a*x**2 + b*x + c */
    static bool solve2ndOrderEq(scalar &x1, scalar &x2, scalar a, scalar b, scalar c);
    /*! solves d*x**3 + a*x**2 + b*x + c */
    static uint solve3rdOrderEq(scalar &x1, scalar &x2, scalar &x3, scalar d, scalar a, scalar b, scalar c);

    /*! returns - atan2(x, y) in degrees */
    static scalar qtHeading(const arr2 &t);

    /*! return the quadrant in which the vector would fall in */
    static int quadrant(const arr2 &v);


    // 1 - Basic single vector operations:
    static scalar magnitude(const arr2 &v);
    static scalar magnitude(scalar x, scalar y);
    static scalar sqrMagnitude(const arr2 &v);
    static void normalise(arr2 &v);
    static void resize(arr2 &v, scalar m);
    // static void max(arr2 &v);

    // 2 - Basic two vector operations:
    //! distance between a and b:
    static scalar distance(const arr2 &a, const arr2 &b);
    static scalar sqrDistance(const arr2 &a, const arr2 &b);
    //! return true if distance(a,b) < absPrec
    static bool areCloseEnough(const arr2 &a, const arr2 &b, scalar absPrec);
    //! unit vector going from a to b: unit(b-a)
    static void tangent(arr2 &t, const arr2 &a, const arr2 &b);
    static arr2 tangent(const arr2 &a, const arr2 &b);
    static void substract1stTo2ndInto3rd(const arr2 &a, const arr2 &b, arr2 &c);
    static void add1stTo2ndInto3rd(const arr2 &a, const arr2 &b, arr2 &c);
    static scalar scalarProduct(const arr2 &a, const arr2 &b);
    static bool areSamePoints(const arr2 &a, const arr2 &b);

    // 3 - Geometry
    //! the shape of a segment:
    enum class shape {straight, vwStraight, clockwise, counterclockwise, vwArc, bezier2, bezier3, vwBezier3, paramPoly3, vwParamPoly3, vwSpiral, opendrive, oneversion, unknown};
    static std::string shapeString(shape s);
    enum class side {port, bow, starboard, unknown};
    static std::string sideString(side s);
    static side parseSide(const char* str);
    enum class intersectionToSegment {none, forward, backward};
    //! whether point p on the segment determined by a---b.
    static bool isPointOnSegment(const arr2 &p, const arr2 &a, const arr2 &b);
    //! whether point p is on the arc with centre c, unit vector from centre to origin co, radius R, length alpha, within some small but non-zero tolerance.
    static bool isPointOnArc(const arr2 &p, const arr2 &c, const arr2 &co, scalar R, scalar alpha, scalar tol = 1e-9);
    //! whether point p is on the "horizontal" box defined by tlc (top-left corner) and brc (bottom-right corner) points.
    static bool isPointInBoxTLcBRc(const arr2 &p, const arr2 &tlc, const arr2 &brc);
    //! whether point p is on the "horizontal" box defined by blc (bottom-left corner) and trc (top-right corner) points.
    static bool isPointInBoxBLcTRc(const arr2 &p, const arr2 &blc, const arr2 &trc);
    //! whether point p is on the "horizontal" box defined by blc (bottom-left corner) and trc (top-right corner) points, within some (positive) tolerance tol.
    static bool isPointInBoxBLcTRcTol(const arr2 &p, const arr2 &blc, const arr2 &trc, scalar tol);
    //! update the bounding box so that it contains the new bounding box:
    static void increaseBoxWithBox(arr2 &blc, arr2 &trc, const arr2 &bli, const arr2 &tri);
    //! get the bounding box as blc, and trc, defined by two points p and q:
    static void boundingBoxFromTwoPoints(arr2 &blc, arr2 &trc, const arr2 &p, const arr2 &q);
    //! get the bounding box as blc and trc for an arc circle with starting point o, ending point e, and centre c */
    static void boundingBoxForArc(arr2 &blc, arr2 &trc, const arr2 &o, const arr2 &e, const arr2 &c, scalar r, shape s);
    //! do the two bounding boxes overlap?
    static bool boxesOverlap(const arr2 &blci, const arr2 &trci, const arr2 &blcj, const arr2 &trcj);
    //! see if any pair of edges intersect.
    static bool figuresOverlap(const std::vector<segment> &fig1, const std::vector<segment> &fig2);
    //! rotate the 2D vector v around the z axis, by angle radians.
    static void rotateVectorByAngle(arr2 &v, scalar angle);
    //! return a new vector that results from rotating the point p around the cm by an angle (in radians)
    static arr2 rotatePointAroundCM(const arr2 &p, const arr2 &cm, scalar angle);
    //! subtended angle between 2 unit vectors, sign is v towards u:
    static scalar subtendedAngle(const arr2 &v, const arr2 &u);
    static scalar subtendedAngle4pi(const arr2 &v, const arr2 &u, shape s);

    //! project a point p onto a segment defined by an origin o, a direction at o to, and a length length:
    static arr2 projectPointToSegment(const arr2 &p, const arr2 &o, const arr2 &to, scalar length);
    //! project a point p onto an arc defined by
    //!  the origin of the arc o,
    //!  the centre of the circle defining the arc c,
    //!  a unit vector pointing from c to o, co,
    //!  and the total (signed) angle.
    static arr2 projectPointToArc(const arr2 &p, const arr2 &o, const arr2 &c, const arr2 &co, scalar alpha);

    //! outputs p, the intersection point of the line given by point o and tangent to,
    //!   and the straight segment starting at a towards ta with length abLength.
    //! returns none if there is no such intersection point, either because the ta and to are parallel, or because of the finite length of the abSegment (abLength).
    //!         forward if there to is pointing towards the segment
    //!         backwards if to is not pointing towards the segment.
    static intersectionToSegment getCodedIntersectionPointToSegment(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &a, const arr2 &ta, const scalar &abLength);
    //! calls getCodedIntersection and returns true if forwards, false otherwise.
    static bool getIntersectionPointToSegment(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &a, const arr2 &ta, const scalar &abLength);
    static intersectionToSegment getCodedIntersectionPointToArc(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &c, const arr2 &co, scalar R, scalar alpha);
    static bool getIntersectionPointToArc(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &c, const arr2 &co, scalar R, scalar alpha);
    //! return true if there is intersection between the segments p1-p2 and q1-q2, and the intersection point in x
    static bool intersectionBetweenSegments(arr2 &x, const arr2 &p1, const arr2 &p2, const arr2 &q1, const arr2 &q2);
    //! Calculate the intersection between straight lines defined by a point and a tangent,
    //!   return false if ta and tb are parallel,
    //!   return true otherwise together with d1 and d2 so that the intersection is in o1 + d1*to, and equally in a + d2*ta.
    static bool intersectionBetweenLines(scalar &d1, scalar &d2, const arr2 &o, const arr2 &to, const arr2 &a, const arr2 &ta);
    //! Recursive function to be called with two sets of points forming a curve, the initial index and final indices.
    //!   Call this function if the bounding boxes of pointsA and pointsB do overlap.
    static void numericalIntersections(std::vector<arr2> &intersections,
                                       const scalar *pointsAx, const scalar *pointsAy, uint ndxOA, uint ndxEA,
                                       const scalar *pointsBx, const scalar *pointsBy, uint ndxOB, uint ndxEB);

    // 4 - Kinematics:
    static scalar minimumTimeForDx(scalar Dx, scalar vo, scalar vmax, scalar a);
    // static scalar minimumTimeForDxVoNonNeg(scalar Dx, scalar vo, scalar vmax, scalar a);
};


class vec2
{

public:
    vec2(){ assign(0., 0.); };
    vec2( scalar t0, scalar t1 ) { assign(t0, t1); }
    vec2( const std::array<scalar,2> &p) { assign(p[0], p[1]); }
    // vec2( const arr2 &p) { assign( p[0], p[1]); }
    scalar& operator [](std::size_t i) { return _data[i]; }
    scalar operator*(const vec2 &b) const { return _data[0] * b._data[0] + _data[1] * b._data[1]; }
    vec2 operator*(scalar x) const { return vec2(x * _data[0], x * _data[1]); }
    // friend vec2 operator*(const scalar s, const vec2& v);
    vec2 operator+(const vec2 &b) const { return vec2(_data[0] + b._data[0], _data[1] + b._data[1]); }
    vec2 operator-(const vec2 &b) const { return vec2(_data[0] - b._data[0], _data[1] - b._data[1]); }
    void assign( scalar t0, scalar t1 ) { _data[0] = t0; _data[1] = t1; }

    scalar magnitude() const { return mvf::magnitude(_data); }
    void normalise() { mvf::normalise(_data); };
    scalar distance(vec2 &b) const { return mvf::distance(_data, b._data); }
    bool areSamePoints(vec2 &b) const { return mvf::areSamePoints(_data, b._data); }

    const std::array<scalar, 2> &data = _data;

private:
    std::array<scalar,2> _data={0., 0.};
};
/*
vec2 operator*(const double s, const vec2& v)
{
    return vec2(v._data[0] * s, v._data[1] * s);
}
*/


} // namespace odrones


#endif //  ODRONES_MATVEC_H
