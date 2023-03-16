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

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include "matvec.h"
#include "readOdr.h"

#ifdef QT_CORE_LIB
#include <QPainterPath>
#endif

class geometry
{
public:
    geometry();
    virtual geometry& operator=(const geometry& g);
    void assignInputGeomToThis(const geometry& g);
    virtual ~geometry();

    arr2 origin() const {return _origin;}
    arr2 dest() const {return _dest;}
    arr2 to() const {return _to;}
    arr2 blc() const {return _blc;}
    arr2 trc() const {return _trc;}
    scalar length() const {return _length;}
    mvf::shape shape() const {return _shape;}
    bool ready() const {return _ready;}



    virtual void base(); ///< initialise all the variables.
    virtual void invert() = 0; ///< invert, and go from end to origin.
    void printOut() const;

    virtual bool isArc() const = 0; ///< return whether it is arc or not;
    virtual bool isNumerical() const = 0; ///< return true if it derives from numerical too;
    virtual bool isPointHere(const arr2 &p) const = 0; ///< return true if p is exactly on this segment.
    virtual arr2 projectPointHere(const arr2 &p) const = 0; ///< project p onto this segment and return it.
    virtual arr2 getTangentInPoint(const arr2 &p) const = 0; ///< unsafely assuming that p is on geometry, calculate the tangent at p.
    virtual scalar distanceToTheEoL(const arr2 &p) const = 0;
    //! unsafely assuming that o is on geometry, it returns a point that is d metres after o, and true if it is on this same lane.
    virtual bool getPointAfterDistance(arr2& p, const arr2 &o, scalar d) const = 0;
    //! returns true and sets the destination intersection point between arr2 origin and arr2 tangent if there is some intersection, false otherwise
    virtual bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const = 0;
    virtual scalar getCurvature(const arr2 &p) const = 0; ///< unsafely assuming that p is on geometry, get the curvature at p.
#ifdef QT_CORE_LIB
    virtual QPainterPath getQPainterPath(uint n) const = 0; ///< get a QPainterPath with n points, or something better.
#endif


protected:
    arr2 _origin; ///< real shape of the lane
    arr2 _dest; ///< real dest of the lane
    mvf::shape _shape; ///< shape of the lane
    scalar _length; ///< the real length of this bit, rather than the length of lane zero.
    arr2 _to; ///< unit tangent at origin.
    arr2 _blc, _trc; ///< bounding box bottom left corner and top right corner.
    arr2 _o, _d; ///< arc; origin and destination of the "lane 0", before the offset [opendrive].
    bool _ready; ///< whether the geometry is ready or not.

};



#endif // GEOMETRY_H
