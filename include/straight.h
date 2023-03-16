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

#ifndef STRAIGHT_H
#define STRAIGHT_H

class straight : public geometry
{
public:

    straight(const Odr::geometry &odr, int sign, scalar offsetA, scalar so, scalar se);
    straight(const arr2 &origin, const arr2 &dest);
    straight(const straight& s);

    void invert() override;
    bool isArc() const override {return false;}
    bool isNumerical() const override {return false;}
    bool isPointHere(const arr2 &p) const override;
    arr2 projectPointHere(const arr2 &p) const override;
    arr2 getTangentInPoint(const arr2 &p) const override;
    scalar distanceToTheEoL(const arr2 &p) const override;
    bool getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const override;
    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override;
    scalar getCurvature(const arr2 &p) const override;
#ifdef QT_CORE_LIB
    QPainterPath getQPainterPath(uint n) const override;
#endif

};


#endif // STRAIGHT_H
