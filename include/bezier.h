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

#ifndef ODRONES_LANEBEZIER_H
#define ODRONES_LANEBEZIER_H

#include <vector>
#include <limits>
#include <iostream>
#include <cstdarg>
#include <tuple>
#include "parametric.h"

#ifdef QT_CORE_LIB
#include <QPainterPath>
#endif

namespace odrones
{

class bezier : public parametric
{
public:

    // enum class component {x, y};

    /** will construct the lane from origin to dest, with width width,
     * ending condition end, going from orig to dest with some laneShape, and given some ID */
    bezier();
    bezier(const bezier& b);
    bezier& operator=(const bezier& b);
    virtual ~bezier() override;
    void assignInputLaneToThis(const bezier& b);

    void base() override;
    void invert() override;

    // virtual scalar curve(scalar t, component c) = 0; ///< returns the component of the curve at t.
    virtual scalar curvex(scalar t) const = 0;
    virtual scalar curvey(scalar t) const = 0;

    virtual scalar curvePx(scalar t) const = 0;
    virtual scalar curvePy(scalar t) const = 0;

    virtual scalar curveP2x(scalar t) const = 0;
    virtual scalar curveP2y(scalar t) const = 0;

    /*! returns sqrt( (dBx/dt)**2 + (dBy/dt)**2) */
    scalar tangentSize(scalar t) const;

    scalar getLength() const;
    virtual scalar calcLength(scalar t) const = 0; ///< calculate the length up to t

    /*! radius of curvature at t */
    scalar getCurvature(scalar t) const;
    scalar getCurvature(const arr2& p) const override; ///< get the curvature at p, assuming p is on the curve.


    /*! get t given a certain distance from the start of the line, and an error tolerance of epsilon */
    bool getTgivenD(scalar &t, scalar d, scalar epsilon = mvf::distPrecision) const;

    /*! return true if the point falls in the curve 0 <= t <= 1; */
    bool isPointHere(const arr2 &p) const override;

    //! returns "d" the distance from the point until the end of the line,
    //!     with True if the input point p was on the line, and False otherwise.
    scalar distanceToTheEoL(const arr2 &p) const override;
    bool distanceToTheEoL(scalar &d, const arr2 &p) const;
    bool distanceFromTheBoL(scalar &d, const arr2 &p) const;

    scalar distanceToTheEoL(scalar t) const;
    scalar distanceFromTheBoL(scalar t) const;
    scalar distanceBetween(scalar t1, scalar t2) const;

    //! returns a point that is d metres after o, and true if it is on this same lane.
    //!    The base class provides a valid implementation, individual classes may provide higher accuracy ones.
    virtual bool getPointAfterDistance(arr2& p, const arr2 &o, scalar d) const override;

    bool getPointAtDistance(arr2 &p, scalar d) const override;

    //! return the top left corner, length and height of the rectangle that aligned to the x,y axis bounds the curve.
    virtual void getBoundingBox(arr2& blc, arr2& trc) const = 0;

    //! returns true and sets the destination intersection point between arr2 origin and arr2 tangent if there is some intersection,
    //!   and false otherwise.
    // virtual bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const = 0;
    virtual bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override = 0;

    //! places in p the projection of o onto the bezier line, and returns the corresponding t.
    arr2 projectPointHere(const arr2& p) const override;

    //! a classic:
    arr2 getTangentInPoint(const arr2& p) const override;

    bool isArc() const override {return false;}
    bool isNumerical() const override {return false;}
    arr2 controlPoint(uint i) const; ///< Return the ith control point;

#ifdef QT_CORE_LIB
    QPainterPath getQPainterPath(uint n) const override;
#endif


protected:
    uint _degree;
    scalar* _wx; ///< an array with the x componets of the weights or control points;
    scalar* _wy; ///< an array with the x componets of the weights or control points;

    void allocateMem(uint degree); ///< allocate the arrays needed;
    void clearMem(); ///< deallocate all the arrays, if the bezier line is set;

    /*! length of a curve, using gaussian quadrature of order... 20! */
    virtual scalar calcLength() const = 0;

    bool getTgivenDRungeKutta(scalar &t, scalar d, scalar epsilon = mvf::distPrecision) const;
    bool getTgivenDNewtonRoot(scalar &t, scalar d, scalar epsilon = mvf::distPrecision) const;

};


} // namespace odrones;


#endif // ODRONES_LANEBEZIER_H
