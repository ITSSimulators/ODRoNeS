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

#include "geometry.h"

#ifndef ODRONES_ARC_H
#define ODRONES_ARC_H

namespace odrones
{

class arc : public geometry
{
public:
    arc();
    arc(const Odr::geometry &odr, int sign, scalar offsetA, scalar so, scalar se, scalar roadSo);
    arc(const OneVersion::segment &sgm, scalar offset);
    arc(const arr2& origin, const arr2& dest, const arr2& centre, mvf::shape s);
    arc(const arr2& origin, const arr2& dest, const arr2& to);
    arc(const arr2& origin, const arr2& dest); ///< temporary constructor, it will need calling setTo() to finish the setup.
    arc(const arc& a);
    arc& operator=(const arc& a);
    void assignInputGeomToThis(const arc& a);

    void setTo(arr2 to);

    void base() override;
    void invert() override;
    bool isArc() const override {return true;}
    bool isNumerical() const override {return false;}
    scalar sl0(scalar s) const override;
    bool pending() const {return _pending;}

    bool isPointHere(const arr2 &p) const override;
    arr2 projectPointHere(const arr2 &p) const override;
    arr2 getTangentInPoint(const arr2 &p) const override;
    scalar distanceToTheEoL(const arr2 &p) const override;
    bool getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const override;
    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override;
    scalar getCurvature(const arr2 &p) const override;
#ifdef QT_CORE_LIB
    QPainterPath getQPainterPath(uint n) const override;
#endif // QT_CORE_LIB

public:
    scalar alpha() const {return _alpha;}
    arr2 centre() const {return _centre;}
    scalar radiusOfCurvature() const {return _radiusOfCurvature;}
    arr2 co() const {return _co;}
    arr2 cd() const {return _cd;}


private:
    arr2 _centre; ///< arc; centre of the circle of the curved lane
    scalar _alpha; ///< arc; the angle subtended by the arc that is the curved lane
    scalar _radiusOfCurvature; ///< arc; the real radius of curvature rather than the lane zero one; signed.
    scalar _odrRoOR; ///< arc; odr; radius of curvature of lane zero over the radius of curvature of this one.
    arr2 _co, _cd; ///< arc; unit vectors from the centre to the origin and destination
    bool _pending; ///< true when _origin and _dest have been set, but the configuration is not there yet.


};

}

#endif // ODRONES_ARC_H
