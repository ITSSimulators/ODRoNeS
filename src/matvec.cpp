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


#include "matvec.h"

#include <cstring>

using namespace odrones;

////////////////////////////////////////
// -1 - Auxilliary functions         ///
////////////////////////////////////////
std::string mvf::shapeString(mvf::shape s)
{
    switch (s)
    {
    case mvf::shape::straight:
        return "straight";
    case mvf::shape::vwStraight:
        return "vwStraight";
    case mvf::shape::clockwise:
        return "arc_cw";
    case mvf::shape::counterclockwise:
        return "arc_ccw";
    case mvf::shape::vwArc:
        return "vwArc";
    case mvf::shape::bezier2:
        return "bezier";
    case mvf::shape::bezier3:
        return "bezier";
    case mvf::shape::vwBezier3:
        return "vwBezier";
    case mvf::shape::paramPoly3:
        return "paramPoly3";
    case mvf::shape::vwParamPoly3:
        return "vwParamPoly3";
    case mvf::shape::vwSpiral:
        return "vwSpiral";
    case mvf::shape::opendrive:
        return "opendrive";
    case mvf::shape::unknown:
        return "unknown";
    default:
        return "clueless!";
    }
}


std::string mvf::sideString(mvf::side s)
{
    switch (s)
    {
    case mvf::side::bow:
        return "bow";
    case mvf::side::port:
        return "port";
    case mvf::side::starboard:
        return "starboard";
    default:
        return "clueless!";
    }
}

odrones::mvf::side odrones::mvf::parseSide(const char* str)
{
    if (std::strcmp(str, "port") == 0)
    {
        return side::port;
    }
    else if (std::strcmp(str, "bow") == 0)
    {
        return side::bow;
    }
    else if (std::strcmp(str, "starboard") == 0)
    {
        return side::starboard;
    }

    return side::unknown;
}

////////////////////////////////////////
// 0 - Operations with scalars        //
////////////////////////////////////////

bool mvf::areSameValues(scalar a, scalar b)
{
    if ((fabs(a - b) < mvf::absolutePrecision) ||
            fabs(a - b) < mvf::relativePrecision * (std::max)(fabs(a), fabs(b)))
        return true;
    // if (fabs(a - b) < fabs(a) * mvf::relativePrecision) return true;
    else return false;
}

bool mvf::areCloseEnough(scalar a, scalar b, scalar absPrec)
{
    if (fabs(a - b) < absPrec) return true;
    return false;
}

bool mvf::isInRangeLR(scalar a, scalar lower, scalar higher, scalar absPrec)
{
    if (a < lower - absPrec) return false;
    if (a > higher + absPrec) return false;
    return true;
}

bool mvf::isInRangeL(scalar a, scalar lower, scalar higher, scalar absPrec)
{
    if ((a >= lower - absPrec) && (a < higher)) return true;
    return false;
}

bool mvf::isInRangeR(scalar a, scalar lower, scalar higher, scalar absPrec)
{
    if ((a > lower) && (a <= higher + absPrec)) return true;
    return false;
}

bool mvf::isInRange0(scalar a, scalar lower, scalar higher)
{
    if ((a > lower) && (a < higher)) return true;
    return false;
}

int mvf::round(scalar a)
{
    return static_cast<int>(std::round(a));
}

scalar mvf::minPositive(scalar a, scalar b)
{
    if (a < b)
    {
        if (a > 0) return a;
        else return b;
    }
    else
    {
        if (b > 0) return b;
        else return a;
    }
}

scalar mvf::positiveZero(scalar a)
{
    if (mvf::areCloseEnough(a, 0, absolutePrecision)) return 0;
    return a;
}


scalar mvf::sqr(scalar a)
{
    return a*a;
}

bool mvf::solve2ndOrderEq(scalar &x1, scalar &x2, scalar a, scalar b, scalar c)
{
    if (mvf::areSameValues(a, 0))
    {
        if (mvf::areSameValues(b, 0)) return false; // no solutions
        // linear solution;
        x1 = -c / b;
        x2 = x1; // everybody trusts there will be two or zero...
        return true;
    }
    scalar disc = b * b - 4*a*c;
    if (disc < 0) return false;
    disc = std::sqrt(disc);
    x1 = (-b + disc)/(2 * a);
    x2 = (-b - disc)/(2 * a);
    return true;
}

uint mvf::solve3rdOrderEq(scalar &x1, scalar &x2, scalar &x3, scalar d, scalar a, scalar b, scalar c)
{
    if (mvf::areSameValues(d, 0)) // this is not a cubic curve
    {
        if (mvf::areSameValues(a, 0)) // this is not quadratic
        {
            if (mvf::areSameValues(b,0)) return 0; // no solutions
            x1 = -c / b; // linear solution
            return 1;
        }
        if (solve2ndOrderEq(x2, x3, a, b, c)) return 2;
        else return 0;
    }

    a /= d;
    b /= d;
    c /= d;

    scalar p = (3*b - a*a) * constants::oneThird;
    scalar p3 = p * constants::oneThird;
    scalar q = (2*a*a*a - 9*a*b + 27*c)/27.;
    scalar q2 = 0.5 * q;
    scalar disc = q2*q2 + p3*p3*p3;

    // three possible real roots:
    if (disc < 0)
    {
        scalar mp3 = -p * constants::oneThird;
        scalar mp33 = mp3 * mp3 * mp3;
        scalar r = std::sqrt(mp33);
        scalar t = -q / (2*r);
        scalar cosphi = t<-1 ? -1 : t >1 ? 1 : t;
        scalar phi = std::acos(cosphi);
        scalar crtr = std::cbrt(r);
        scalar t1 = 2 * crtr;
        x1 = t1 * cos(phi * constants::oneThird) - a * constants::oneThird;
        x2 = t1 * cos((phi + 2*constants::pi) * constants::oneThird) - a * constants::oneThird;
        x3 = t1 * cos((phi + 4*constants::pi) * constants::oneThird) - a * constants::oneThird;
        return 3;
    }

    // three real roots, but two are equal:
    if (mvf::areSameValues(0, disc))
    {
        scalar u1 = q2 < 0 ? std::cbrt(-q2) : -std::cbrt(q2);
        x1 = 2 * u1 - a * constants::oneThird;
        x2 = -u1 -a * constants::oneThird;
        return 2;
    }

    // one real root, two complex roots:
    scalar sd = std::sqrt(disc);
    scalar u1 = std::cbrt(sd - q2);
    scalar v1 = std::cbrt(sd + q2);
    x1 = u1 - v1 - a * constants::oneThird;
    return 1;
}


scalar mvf::qtHeading(const arr2 &t)
{
    // return - constants::rad2deg * std::atan2(t[0], t[1]);
    return 180 + constants::rad2deg * std::atan2(t[0], t[1]);
}


////////////////////////////////////////
// 1 - Basic single vector operations //
////////////////////////////////////////
scalar mvf::magnitude(const arr2 &v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1]);
}

scalar mvf::magnitude(scalar x, scalar y)
{
    return std::sqrt(x * x + y * y);
}

scalar mvf::sqrMagnitude(const arr2 &v)
{
    return (v[0] * v[0] + v[1] * v[1]);
}

void mvf::normalise(arr2 &v)
{
    scalar m = magnitude(v);
    v[0] = v[0]/m;
    v[1] = v[1]/m;
}

void mvf::resize(arr2 &v, scalar m)
{
    v[0] = v[0] * m;
    v[1] = v[1] * m;
}

/*
void mvf::max(arr2 &v)
{
    v[0] = std::numeric_limits<scalar>::max();
    v[1] = std::numeric_limits<scalar>::max();
}
*/

int mvf::quadrant(const arr2 &v)
{
    // atan2 range is -pi to pi. Get first the quadrant for o and e:
    //          |
    //      Q2  |  Q1
    //      ---------
    //      Q3  |  Q4
    //          |
    scalar a = std::atan2(v[1], v[0]);

    int q = 0;
    const scalar halfPi = 0.5*constants::pi;
    if (a >= 0 && a < halfPi)
        q = 1;
    else if (a >= halfPi && a <= constants::pi)
        q = 2;
    else if (a >= -constants::pi && a < -halfPi)
        q = 3;
    else if (a >= -halfPi && a < 0)
        q = 4;

    return q;

}



////////////////////////////////////////
// 2 - Basic two-vector operations    //
////////////////////////////////////////
void mvf::tangent(arr2 &t, const arr2 &a, const arr2 &b)
{
    t[0] = b[0] - a[0];
    t[1] = b[1] - a[1];
    normalise(t);
}

arr2 mvf::tangent(const arr2 &a, const arr2 &b)
{
    arr2 t = {b[0] - a[0], b[1] - a[1]};
    normalise(t);
    return t;
}

scalar mvf::distance(const arr2 &a, const arr2 &b)
{
    // arr2 c = {b[0] - a[0], b[1] - a[1]};
    // return magnitude(c);
    return std::sqrt((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
}

scalar mvf::sqrDistance(const arr2 &a, const arr2 &b)
{
    // arr2 c = {b[0] - a[0], b[1] - a[1]};
    // return sqrMagnitude(c);
    return (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]);
}

bool mvf::areCloseEnough(const arr2 &a, const arr2 &b, scalar absPrec)
{
    if (sqrDistance(a, b) < absPrec * absPrec) return true;
    return false;
}

void mvf::substract1stTo2ndInto3rd(const arr2 &a, const arr2 &b, arr2 &c)
{
    c[0] = b[0] - a[0];
    c[1] = b[1] - a[1];
}

void mvf::add1stTo2ndInto3rd(const arr2 &a, const arr2 &b, arr2 &c)
{
    c[0] = b[0] + a[0];
    c[1] = b[1] + a[1];
}

scalar mvf::scalarProduct(const arr2 &a, const arr2 &b)
{
    return a[0]*b[0] + a[1]*b[1];
}

bool mvf::areSamePoints(const arr2 &a, const arr2 &b)
{
    if (areSameValues(0, distance(a, b))) return true;
    return false;
}

////////////////////////////////////////
// 3 - Geometry                       //
////////////////////////////////////////

bool mvf::isPointOnSegment(const arr2 &p, const arr2 &a, const arr2 &b)
{
    arr2 t;
    tangent(t, a, b);

    scalar d;
    // firstly check that the point is within the line,
    //   i. e., that there is a p[] = a[] + d * t[]:
    if (!mvf::areSameValues(t[0], 0))
    {
        d = (p[0] - a[0])/t[0];
        if (!areSameValues(a[1] + d * t[1], p[1])) return false;
    }
    else
    {
        d = (p[1] - a[1])/t[1];
        if (!areSameValues(a[0] + d * t[0], p[0])) return false;
    }

    // if (d > distance(a, b)) return false;
    if ( (d > distance(a, b)) || (d < 0) ) return false;

    return true;
}

bool mvf::isPointOnArc(const arr2 &p, const arr2 &c, const arr2 &co, scalar R, scalar alpha, scalar tol)
{
    // We'll need the variables v and r^2 at the first stage,
    //   and r on the second... so I did it like that.
    arr2 v = {p[0] - c[0], p[1] - c[1]};
    scalar r = mvf::magnitude(v);

    // First, check that the thing falls onto the circle:
    // if (! mvf::areSameValues(r, fabs(R)))
    if (! mvf::areCloseEnough(r, fabs(R), tol))
        return false;

    // Secondly, check that the angle falls between _co and _cd:
    v[0] = v[0] / r;
    v[1] = v[1] / r;
    scalar angle = mvf::subtendedAngle(co, v);
    scalar angleTol = tol / fabs(R);
    if (alpha > 0)
    {
        if (mvf::isInRangeLR(angle, 0, alpha, angleTol)) return true;
    }
    else
    {
        if (mvf::isInRangeLR(angle, alpha, 0, angleTol)) return true;
    }

    return false;

}

bool mvf::isPointInBoxTLcBRc(const arr2 &p, const arr2 &tlc, const arr2 &brc)
{
    if (p[0] < tlc[0]) return false;
    if (p[1] > tlc[1]) return false;
    if (p[0] > brc[0]) return false;
    if (p[1] < brc[1]) return false;
    return true;
}

bool mvf::isPointInBoxBLcTRc(const arr2 &p, const arr2 &blc, const arr2 &trc)
{
    if (p[0] < blc[0]) return false;
    if (p[1] < blc[1]) return false;
    if (p[0] > trc[0]) return false;
    if (p[1] > trc[1]) return false;
    return true;
}

bool mvf::isPointInBoxBLcTRcTol(const arr2 &p, const arr2 &blc, const arr2 &trc, scalar tol)
{
    if (p[0] < blc[0] - tol) return false;
    if (p[1] < blc[1] - tol) return false;
    if (p[0] > trc[0] + tol) return false;
    if (p[1] > trc[1] + tol) return false;
    return true;
}


void mvf::increaseBoxWithBox(arr2 &blc, arr2 &trc, const arr2 &bli, const arr2 &tri)
{
    for (int i = 0; i < 2; ++i)
    {
        if (bli[i] < blc[i]) blc[i] = bli[i];
        if (tri[i] > trc[i]) trc[i] = tri[i];
    }
}

void mvf::boundingBoxFromTwoPoints(arr2 &blc, arr2 &trc, const arr2 &p, const arr2 &q)
{
    blc = {(std::min)(p[0], q[0]), (std::min)(p[1], q[1])};
    trc = {(std::max)(p[0], q[0]), (std::max)(p[1], q[1])};
}

void mvf::boundingBoxForArc(arr2 &blc, arr2 &trc, const arr2 &o, const arr2 &e, const arr2 &c, scalar r, shape s)
{

    // Method 1 - Use the whole circle:
    // blc = {c[0] - r, c[1] - r};
    // trc = {c[0] + r, c[1] + r};
    // return;


    // Method 2 - Be more precise:
    if ((s != shape::clockwise) && (s != shape::counterclockwise))
        return;


    int qo = quadrant({o[0] - c[0], o[1] - c[1]});
    int qe = quadrant({e[0] - c[0], e[1] - c[1]});
    if (qo == qe)
    {
        boundingBoxFromTwoPoints(blc, trc, o, e);
        return;
    }

    arr2 N = {c[0], c[1] + r};
    arr2 S = {c[0], c[1] - r};
    arr2 E = {c[0] + r, c[1]};
    arr2 W = {c[0] - r, c[1]};

    arr2 north = o;
    arr2 south = o;
    arr2 east = o;
    arr2 west = o;

    if (s == shape::counterclockwise) //(!clockwise)
    {
        if (qo == 1)
        {
            if (qe == 2)
                north = N;
            else if (qe == 3)
            {
                north = N;
                west = W;
            }
            else if (qe == 4)
            {
                north = N;
                west = W;
                south = S;
            }
        }
        else if (qo == 2)
        {
            if (qe == 3)
                west = W;
            else if (qe == 4)
            {
                west = W;
                south = S;
            }
            else if (qe == 1)
            {
                west = W;
                south = S;
                east = E;
            }
        }
        else if (qo == 3)
        {
            if (qe == 4)
                south = S;
            else if (qe == 1)
            {
                south = S;
                east = E;
            }
            else if (qe == 2)
            {
                south = S;
                east = E;
                north = N;
            }
        }
        else if (qo == 4)
        {
            if (qe == 1)
                east = E;
            else if (qe == 2)
            {
                east = E;
                north = N;
            }
            else if (qe == 3)
            {
                east = E;
                north = N;
                west = W;
            }
        }
    }
    else
    {
        if (qo == 1)
        {
            if (qe == 4)
                east = E;
            else if (qe == 3)
            {
                east = E;
                south = S;
            }
            else if (qe == 2)
            {
                east = E;
                south = S;
                west = W;
            }
        }
        else if (qo == 2)
        {
            if (qe == 1)
                north = N;
            else if (qe == 4)
            {
                north = N;
                east = E;
            }
            else if (qe == 3)
            {
                north = N;
                east = E;
                south = S;
            }
        }
        else if (qo == 3)
        {
            if (qe == 2)
                west = W;
            else if (qe == 1)
            {
                west = W;
                north = N;
            }
            else if (qe == 4)
            {
                west = W;
                north = N;
                east = E;
            }
        }
        else if (qo == 4)
        {
            if (qe == 3)
                south = S;
            else if (qe == 2)
            {
                south = S;
                west = W;
            }
            else if (qe == 1)
            {
                south = S;
                west = W;
                north = N;
            }
        }
    }

    std::tie(blc[0], trc[0]) = std::minmax({o[0], e[0], north[0], south[0], east[0], west[0]});
    std::tie(blc[1], trc[1]) = std::minmax({o[1], e[1], north[1], south[1], east[1], west[1]});

    return;
}



bool mvf::boxesOverlap(const arr2 &blci, const arr2 &trci, const arr2 &blcj, const arr2 &trcj)
{
    // First, check if a corner is within the other box.
    if (isPointInBoxBLcTRc(blci, blcj, trcj)) return true;
    if (isPointInBoxBLcTRc(trci, blcj, trcj)) return true;

    if (isPointInBoxBLcTRc({blci[0], trci[1]}, blcj, trcj)) return true;
    if (isPointInBoxBLcTRc({trci[0], blci[1]}, blcj, trcj)) return true;


    if (isPointInBoxBLcTRc(blcj, blci, trci)) return true;
    if (isPointInBoxBLcTRc(trcj, blci, trci)) return true;

    if (isPointInBoxBLcTRc({blcj[0], trcj[1]}, blci, trci)) return true;
    if (isPointInBoxBLcTRc({trcj[0], blcj[1]}, blci, trci)) return true;

    // if no corner is within the other box, then it still may be that the boxes are:
    //        ---
    //        | |
    //   ------------
    //   |    | |   |
    //   ------------
    //        | |
    //        ---
    //   which means that there has to be double intersection, so it does not matter which one we check.
    arr2 tmp;
    if ( (intersectionBetweenSegments(tmp, blci, {trci[0], blci[1]}, blcj, {blcj[0], trcj[1]})) &&
         (intersectionBetweenSegments(tmp, blci, {trci[0], blci[1]}, {trcj[0], blcj[1]}, trcj)) )
        return true;

    if ( (intersectionBetweenSegments(tmp, blcj, {trcj[0], blcj[1]}, blci, {blci[0], trci[1]})) &&
         (intersectionBetweenSegments(tmp, blcj, {trcj[0], blcj[1]}, {trci[0], blci[1]}, trci)) )
        return true;

    return false;
}


void mvf::numericalIntersections(std::vector<arr2> &intersections,
                                 const scalar *pointsAx, const scalar *pointsAy, uint ndxOA, uint ndxEA,
                                 const scalar *pointsBx, const scalar *pointsBy, uint ndxOB, uint ndxEB)
{

    // 1 - split the A curve
    bool splitA = true;
    uint ndxOA1, ndxEA1, ndxOA2, ndxEA2;
    if ((ndxEA - ndxOA == 1) || (ndxEA - ndxOA == 2))
    {
        splitA = false;
        ndxOA1 = ndxOA;
        ndxEA1 = ndxEA;
    }
    else
    {
        ndxOA1 = ndxOA;
        ndxOA2 = static_cast<uint>(std::floor(0.5*(ndxOA + ndxEA)));

        ndxEA1 = ndxOA2 + 1;
        ndxEA2 = ndxEA;
    }

    // 2 - split the B curve
    bool splitB = true;
    uint ndxOB1, ndxEB1, ndxOB2, ndxEB2;
    if ((ndxEB - ndxOB == 1) || (ndxEB - ndxOB == 2))
    {
        splitB = false;
        ndxOB1 = ndxOB;
        ndxEB1 = ndxEB;
    }
    else
    {
        ndxOB1 = ndxOB;
        ndxOB2 = static_cast<uint>(std::floor(0.5*(ndxOB + ndxEB)));

        ndxEB1 = ndxOB2 + 1;
        ndxEB2 = ndxEB;
    }


    // If it is just two points per side, calculate the actual crossing point.
    if ( (!splitA) && (!splitB) )
    {
        arr2 x;
        if (!intersectionBetweenSegments(x, {pointsAx[ndxOA], pointsAy[ndxOA]},
                                            {pointsAx[ndxEA], pointsAy[ndxEA]},
                                            {pointsBx[ndxOB], pointsBy[ndxOB]},
                                            {pointsBx[ndxEB], pointsBy[ndxEB]}))
            return;
        intersections.push_back(x);
        return;
    }

    // Calculate the bounding box of A1, A2, B1 and B2
    auto [ mxA1, MxA1 ] = std::minmax_element(pointsAx+ndxOA1, pointsAx+ndxEA1);
    auto [ myA1, MyA1 ] = std::minmax_element(pointsAy+ndxOA1, pointsAy+ndxEA1);
    arr2 blA1 = {*mxA1, *myA1};
    arr2 trA1 = {*MxA1, *MyA1};


    const scalar *mxA2, *MxA2, *myA2, *MyA2;
    arr2 blA2, trA2;
    if (splitA)
    {
        std::tie(mxA2, MxA2) = std::minmax_element(pointsAx+ndxOA2, pointsAx+ndxEA2);
        std::tie(myA2, MyA2) = std::minmax_element(pointsAy+ndxOA2, pointsAy+ndxEA2);
        blA2 = {*mxA2, *myA2};
        trA2 = {*MxA2, *MyA2};
    }


    auto [ mxB1, MxB1 ] = std::minmax_element(pointsBx+ndxOB1, pointsBx+ndxEB1);
    auto [ myB1, MyB1 ] = std::minmax_element(pointsBy+ndxOB1, pointsBy+ndxEB1);
    arr2 blB1 = {*mxB1, *myB1};
    arr2 trB1 = {*MxB1, *MyB1};


    const scalar *mxB2, *MxB2, *myB2, *MyB2;
    arr2 blB2, trB2;
    if (splitB)
    {
        std::tie(mxB2, MxB2) = std::minmax_element(pointsBx+ndxOB2, pointsBx+ndxEB2);
        std::tie(myB2, MyB2) = std::minmax_element(pointsBy+ndxOB2, pointsBy+ndxEB2);
        blB2 = {*mxB2, *myB2};
        trB2 = {*MxB2, *MyB2};
    }


    if (boxesOverlap(blA1, trA1, blB1, trB1))
        numericalIntersections(intersections, pointsAx, pointsAy, ndxOA1, ndxEA1,
                                              pointsBx, pointsBy, ndxOB1, ndxEB1);

    if ((splitB) && (boxesOverlap(blA1, trA1, blB2, trB2)))
        numericalIntersections(intersections, pointsAx, pointsAy, ndxOA1, ndxEA1,
                                              pointsBx, pointsBy, ndxOB2, ndxEB2);

    if ((splitA) && (boxesOverlap(blA2, trA2, blB1, trB1)))
        numericalIntersections(intersections, pointsAx, pointsAy, ndxOA2, ndxEA2,
                                              pointsBx, pointsBy, ndxOB1, ndxEB1);

    if ((splitA) && (splitB) && (boxesOverlap(blA2, trA2, blB2, trB2)))
        numericalIntersections(intersections, pointsAx, pointsAy, ndxOA2, ndxEA2,
                                              pointsBx, pointsBy, ndxOB2, ndxEB2);
    return;

}


void mvf::rotateVectorByAngle(arr2 &v, scalar angle)
{
    arr2 vo = {v[0], v[1]};
    v[0] = vo[0]*cos(angle) - vo[1]*sin(angle);
    v[1] = vo[0]*sin(angle) + vo[1]*cos(angle);
}

arr2 mvf::rotatePointAroundCM(const arr2 &p, const arr2 &cm, scalar angle)
{
    arr2 q = {p[0] - cm[0], p[1] - cm[1]};
    rotateVectorByAngle(q, angle);
    return {q[0] + cm[0], q[1] + cm[1]};
}


scalar mvf::subtendedAngle(const arr2 &v, const arr2 &u)
{
    return atan2(v[0]*u[1] - v[1]*u[0], v[0]*u[0] + v[1]*u[1]);
}

scalar mvf::subtendedAngle4pi(const arr2 &v, const arr2 &u, shape s)
{
    scalar a = subtendedAngle(v, u);
    if (a < 0)
    {
        if (s == mvf::shape::counterclockwise) a = constants::pi/2 - a;
    }
    else // (a > 0)
    {
        if (s == mvf::shape::clockwise) a = - constants::pi/2 - a;
    }

    return a;
}


arr2 mvf::projectPointToSegment(const arr2 &p, const arr2 &o, const arr2 &to, scalar length)
{
    // have a look at both sides the segment, see if p + n intersect with the input segment (o, o + length * to):
    arr2 q;
    arr2 n = {-to[1], to[0]};
    if (getCodedIntersectionPointToSegment(q, p, n, o, to, length) != mvf::intersectionToSegment::none) return q;

    // If there's no intersection, it means that the projection is one of the edges:
    arr2 e = {o[0] + to[0] * length, o[1] + to[1] * length};
    scalar d2o = sqrDistance(p, o);
    scalar d2e = sqrDistance(p, e);
    if (d2o < d2e) return o;
    return e;
}


arr2 mvf::projectPointToArc(const arr2 &p, const arr2 &o, const arr2 &c, const arr2 &co, scalar a)
{
    scalar ds = 0.1;
    scalar r = mvf::distance(c, o);
    scalar length = std::abs(a) * r;
    uint parts = (std::max)(50, static_cast<int>(length / ds));

    scalar da = a / parts;
    scalar d2o = 1e12;
    uint minIdx = 0;
    for (uint i = 0; i <= parts; ++i)
    {
        arr2 coi = co;
        mvf::rotateVectorByAngle(coi, i * da);
        arr2 qi = {c[0] + r * coi[0], c[1] + r * coi[1]};
        scalar d2i = mvf::sqrDistance(p, qi);
        if (d2i < d2o)
        {
            d2o = d2i;
            minIdx = i;
        }
    }


    scalar ao = minIdx * da;
    scalar ai;
    int sign = 1;
    if (a < 0) sign = -1;
    while (ds > distPrecision)
    {
        ai = ao + 0.5 * da;
        if (ai * sign <= a * sign)
        {
            arr2 coi = co;
            mvf::rotateVectorByAngle(coi, ai);
            arr2 qi = {c[0] + r * coi[0], c[1] + r * coi[1]};
            scalar d2i = mvf::sqrDistance(p, qi);
            if (d2i < d2o)
            {
                d2o = d2i;
                ao = ai;
                da = 0.5 * da;
                ds = std::abs(da) * r;
                continue;
            }
         }

         ai = ao - 0.5 * da;
         if (ai * sign >= 0)
         {
            arr2 coi = co;
            mvf::rotateVectorByAngle(coi, ai);
            arr2 qi = {c[0] + r * coi[0], c[1] + r * coi[1]};
            scalar d2i = mvf::sqrDistance(p, qi);
            if (d2i < d2o)
            {
                d2o = d2i;
                ao = ai;
                da = 0.5 * da;
                ds = std::abs(da) * r;
                continue;
            }
         }

         da = 0.5 * da;
         ds = std::abs(da) * r;
    }

    arr2 coi = co;
    mvf::rotateVectorByAngle(coi, ao);
    return {c[0] + r * coi[0], c[1] + r * coi[1]};

}

bool mvf::getIntersectionPointToArc(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &c, const arr2 &co, scalar R, scalar alpha)
{
    if (getCodedIntersectionPointToArc(p, o, to, c, co, R, alpha) == mvf::intersectionToSegment::forward) return true;
    else return false;
}

mvf::intersectionToSegment mvf::getCodedIntersectionPointToArc(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &c, const arr2 &co, scalar R, scalar alpha)
{

    // The arc is represented by the circle equation R^2 = (x - c[0])^2 + (y - c[1])^2, and thus y = c[1] +- sqrt( R^2 - (x - c[0])^2 )
    // The segment to intersect with this arc has equation y = x * t[1] / t[0] + b;
    //   but if t[0] = 0, then we have a vertical line passing through o, which means that x = o[0]
    arr2 pA, pB; // the line can intersect the circle in two points. We'll figure out which one (if any) is suitable
    bool zeroes = false;
    if (mvf::areSameValues(0, to[0]))
    {
        zeroes = true;
        pA[0] = o[0];
        pB[0] = o[0];
        scalar disc = R * R - (o[0] - c[0])*(o[0] - c[0]);
        if (disc < 0) return mvf::intersectionToSegment::none;
        disc = sqrt(disc);
        pA[1] = c[1] + disc;
        pB[1] = c[1] - disc;
    }
    else
    {
        long double m = to[1] / to[0];
        long double b = o[1] - m * o[0];
        long double soepA = (m*m + 1);
        long double soepB = 2 * (m*(b-c[1]) - c[0]);
        long double soepC = (b - c[1])*(b - c[1]) - R*R + c[0]*c[0];

        long double disc = soepB * soepB - 4 * soepA * soepC;
        if (disc < 0) return mvf::intersectionToSegment::none;
        disc = std::sqrt(disc);
        pA[0] = (-soepB + disc) / (2 * soepA);
        pB[0] = (-soepB - disc) / (2 * soepA);
        pA[1] = m * pA[0] + b;
        pB[1] = m * pB[0] + b;
    }

    // Now look at whether pA and pB are backwards or forwards.
    //   pA = o + dA * to   and
    //   pB = o + dB * to,  so
    scalar dA, dB;
    if (zeroes)
    {
        // pA[0] = o[0]  --- pA[1] = o[1] + dA * to[1]
        // pB[0] = o[0]  --- pB[1] = o[1] + dB * to[1]
        dA = (pA[1] - o[1])/to[1];
        dB = (pB[1] - o[1])/to[1];
    }
    else
    {
        dA = (pA[0] - o[0])/to[0];
        dB = (pB[0] - o[0])/to[0];
    }
    dA = mvf::positiveZero(dA);
    dB = mvf::positiveZero(dB);
    // Try to return forwards:
    if (dA >= 0)
    {
        //  then dA would be preferred because of being closer, dB would be OK.
        if (dB > dA)
        {
            if (isPointOnArc(pA, c, co, R, alpha))
            {
                p = {pA[0], pA[1]};
                return mvf::intersectionToSegment::forward;
            }
            else if (isPointOnArc(pB, c, co, R, alpha))
            {
                p = {pB[0], pB[1]};
                return mvf::intersectionToSegment::forward;
            }
        }
        else
        {
            if (dB >= 0)
            {
                if (isPointOnArc(pB, c, co, R, alpha))
                {
                    p = {pB[0], pB[1]};
                    return mvf::intersectionToSegment::forward;
                }
            }
            else if (isPointOnArc(pA, c, co, R, alpha))
            {
                p = {pA[0], pA[1]};
                return mvf::intersectionToSegment::forward;
            }
        }
    }
    else if (dB >= 0)
    {
        if (isPointOnArc(pB, c, co, R, alpha))
        {
            p = {pB[0], pB[1]};
            return mvf::intersectionToSegment::forward;
        }
    }
    else // so both are backwards:
    {
        if (dA > dB) // then we'd prefer dA because of being closer (negative numbers)
        {
            if (isPointOnArc(pA, c, co, R, alpha))
            {
                p = {pA[0], pA[1]};
                return mvf::intersectionToSegment::backward;
            }
            else if (isPointOnArc(pB, c, co, R, alpha))
            {
                p = {pB[0], pB[1]};
                return mvf::intersectionToSegment::backward;
            }
        }
        else
        {
            if (isPointOnArc(pB, c, co, R, alpha))
            {
                p = {pB[0], pB[1]};
                return mvf::intersectionToSegment::backward;
            }
            else if (isPointOnArc(pA, c, co, R, alpha))
            {
                p = {pA[0], pA[1]};
                return mvf::intersectionToSegment::backward;
            }
        }

    }

    return mvf::intersectionToSegment::none;
}


bool mvf::getIntersectionPointToSegment(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &a, const arr2 &ta, const scalar &abLength)
{
    if (getCodedIntersectionPointToSegment(p, o, to, a, ta, abLength) == mvf::intersectionToSegment::forward) return true;
    return false;
}


bool mvf::intersectionBetweenLines(scalar &da, scalar &db, const arr2 &a, const arr2 &ta, const arr2 &b, const arr2 &tb)
{
    // 1 - Check that the vectors to and ta are not parallel:
    if (mvf::areSameValues(0, ta[0]*tb[1] - ta[1]*tb[0]))
        return false;

    da = -1, db = -1;

    bool zeroes = false;

    // 2 - Write the main equality in x,y components:
    //  a[0] + da * ta[0] = b[0]+ db * tb[0]
    //  a[1] + da * ta[1] = b[1]+ db * tb[1]
    if (mvf::areSameValues(ta[0], 0))
    {
        db = (a[0] - b[0]) / tb[0]; // tb[0] cannot be 0 because ta[0] is 0 and the lines are not parallel.
        da = (b[1] + db * tb[1] - a[1]) / ta[1]; // ta[1] cannot be 0 because ta[0] is 0, and to is a unit vector.
        zeroes = true;
    }
    else if (mvf::areSameValues(tb[0], 0))
    {
        da = (b[0] - a[0]) / ta[0];
        db = (a[1] + da * ta[1] - b[1]) / tb[1];
        zeroes = true;
    }
    // neither ta[0] nor tb[0] are zero, and thus we can write the slope form of the line equation:
    // y = mo * x + co and y = ma * x + ca, and find (x,y) the intersection point from here:
    if (! zeroes)
    {
        scalar mo = ta[1] / ta[0];
        scalar ma = tb[1] / tb[0];
        scalar co = a[1] - mo * a[0];
        scalar ca = b[1] - ma * b[0];
        arr2 p; // the intersection point;
        p[0] = (co - ca) / (ma - mo);
        p[1] = mo * p[0] + co;

        // now use the form p = a + d * ta, to check that the point is within the ta segment:
        // p[0] = a[0] + db * ta[0];
        db = (p[0] - b[0])/tb[0];
        // and check that we are on the correct side:
        // p[0] = o[0] + da * to[0];
        da = (p[0] - a[0])/ta[0];
    }

    return true;
}

bool mvf::intersectionBetweenSegments(arr2 &x, const arr2 &p1, const arr2 &p2, const arr2 &q1, const arr2 &q2)
{
    // constexpr scalar tol = 1e-8;
    if (areCloseEnough(p1, p2, absolutePrecision)) // if p1--p2 is not a segment but a single point
    {
        if (areCloseEnough(q1, q2, absolutePrecision)) // if q1--q2 is not a segment but a single point
            return areCloseEnough(p1, q1, absolutePrecision); // then return if they're the same point
        else
            return isPointOnSegment(p1, q1, q2); // otherwise check if the point p1 == p2 is on q1--q2.
    }
    else if (areCloseEnough(q1, q2, absolutePrecision))
        return isPointOnSegment(q1, p1, p2);

    scalar dp = -1, dq = -1;
    arr2 tp = tangent(p1, p2);
    arr2 tq = tangent(q1, q2);

    if (!intersectionBetweenLines(dp, dq, p1, tp, q1, tq)) return false;

    if ((dp < 0) || (dq < 0)) return false;
    if (dp > distance(p1, p2)) return false;
    if (dq > distance(q1, q2)) return false;

    x = {p1[0] + dp*tp[0], p1[1] + dp*tp[1]};
    // if (!isPointInSegment(x, p1, p2))
    //     std::cout << "shit" << std::endl;
    // if (!isPointInSegment(x, q1, q2))
    //     std::cout << "shit" << std::endl;

    return true;

}


mvf::intersectionToSegment mvf::getCodedIntersectionPointToSegment(arr2 &p, const arr2 &o, const arr2 &to, const arr2 &a, const arr2 &ta, const scalar &abLength)
{
    scalar d1 = -1, d2 = -1;

    if (!intersectionBetweenLines(d1, d2, o, to, a, ta)) return mvf::intersectionToSegment::none;

    p[0] = o[0] + d1 * to[0];
    p[1] = o[1] + d1 * to[1];

    // 3 - Check:
    if (d2 < 0) return mvf::intersectionToSegment::none;
    if (d2 > abLength) return mvf::intersectionToSegment::none;
    if (d1 < 0) return mvf::intersectionToSegment::backward;

    return mvf::intersectionToSegment::forward;

    /*
    // 1 - Check that the vectors to and ta are not parallel:
    if (mvf::areSameValues(0, to[0]*ta[1] - to[1]*ta[0]))
        return mvf::intersectionToSegment::none;


    bool zeroes = false;

    // 2 - Write the main equality in x,y components:
    //  o[0] + d1 * to[0] = a[0]+ d2 * ta[0]
    //  o[1] + d1 * to[1] = a[1]+ d2 * ta[1]
    if (mvf::areSameValues(to[0], 0))
    {
        d2 = (o[0] - a[0]) / ta[0]; // ta[0] cannot be 0 because to[0] is 0 and the lines are not parallel.
        d1 = (a[1] + d2 * ta[1] - o[1]) / to[1]; // to[1] cannot be 0 because to[0] is 0, and to is a unit vector.
        zeroes = true;
    }
    else if (mvf::areSameValues(ta[0], 0))
    {
        d1 = (a[0] - o[0]) / to[0];
        d2 = (o[1] + d1 * to[1] - a[1]) / ta[1];
        zeroes = true;
    }
    // neither to[0] nor ta[0] are zero, and thus we can write the slope form of the line equation:
    // y = mo * x + co and y = ma * x + ca, and find (x,y) the intersection point from here:
    if (! zeroes)
    {
        scalar mo = to[1] / to[0];
        scalar ma = ta[1] / ta[0];
        scalar co = o[1] - mo * o[0];
        scalar ca = a[1] - ma * a[0];
        p[0] = (co - ca) / (ma - mo);
        p[1] = mo * p[0] + co;

        // now use the form p = a + d * ta, to check that the point is within the ta segment:
        // p[0] = a[0] + d2 * ta[0];
        d2 = (p[0] - a[0])/ta[0];
        // and check that we are on the correct side:
        // p[0] = o[0] + d1 * to[0];
        d1 = (p[0] - o[0])/to[0];
    }
    else
    {

        p[0] = o[0] + d1 * to[0];
        p[1] = o[1] + d1 * to[1];
    }

    // 3 - Check:
    if (d2 < 0) return mvf::intersectionToSegment::none;
    if (d2 > abLength) return mvf::intersectionToSegment::none;
    if (d1 < 0) return mvf::intersectionToSegment::backward;

    return mvf::intersectionToSegment::forward;
    */

}

////////////////////////////////////////
// 4 - Kinematics                     //
////////////////////////////////////////

scalar mvf::minimumTimeForDx(scalar Dx, scalar vo, scalar vmax, scalar a)
// scalar mvf::minimumTimeForDxVoNonNeg(scalar Dx, scalar vo, scalar vmax, scalar a)
{
    // if we're already there, then t = 0;
    if (mvf::areSameValues(0, Dx)) return 0;

    // Calculate the time to cover Dx at constant acceleration a:
    scalar disc = vo*vo + 2 * a * Dx;
    // if there is no solution, it means we're decelerating, and that we'll stop before.
    if (disc < 0) return std::numeric_limits<scalar>::max();
    disc = sqrt(disc);
    scalar t = (-vo + disc) / a;
    scalar timeCtAcc2 = (-vo - disc) / a;

    // sort out which value we should take, the minimum that is > 0, and place it in t
    if (t < 0)
    {
        if (timeCtAcc2 < 0) return std::numeric_limits<scalar>::max();
        t = timeCtAcc2;
    }
    else if ((timeCtAcc2 > 0) && (timeCtAcc2 < t)) t = timeCtAcc2;


    // Calculate the time to vMax:
    scalar timeToVMax = (vmax - vo)/a;
    // If timeCtAcc > timeToVMax -> we would drive at a higher speed than vmax.
    //  Instead, we'd accelerate during timeToVMax, and drive the rest at constant speed:
    if ((t > timeToVMax) && (timeToVMax > 0))
    {
        scalar DxAcc = vo * timeToVMax + 0.5 * a * timeToVMax*timeToVMax;
        t = timeToVMax + (Dx - DxAcc) / vmax;
    }
    return t;

}

