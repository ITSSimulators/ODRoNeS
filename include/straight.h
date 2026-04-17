//
//   This file is part of ODRoNeS (OpenDRIVE Road Network System).
//
//   Copyright (c) 2019-2026 Albert Solernou, University of Leeds.
//
//   The ODRoNeS package is free software; you can redistribute it and/or
//   modify it under the terms of the GNU Lesser General Public
//   License as published by the Free Software Foundation; either
//   version 3 of the License, or (at your option) any later version.
//
//   The ODRoNeS package is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//   Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public
//   License along with the ODRoNeS package; if not, see
//   <https://www.gnu.org/licenses/>.
//



#include "geometry.h"

#ifndef ODRONES_STRAIGHT_H
#define ODRONES_STRAIGHT_H

namespace odrones
{

class straight : public geometry
{
public:

    straight(const Odr::geometry &odr, int sign, scalar offsetA, scalar so, scalar se, scalar roadSo);
    straight(const OneVersion::segment &sgm, scalar offset);
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
    bool getPointAtDistance(arr2 &p, scalar d) const override;
    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override;
    scalar getCurvature(const arr2 &p) const override;
#ifdef QT_CORE_LIB
    QPainterPath getQPainterPath(uint n) const override;
#endif

};

} // namespace odrones

#endif // ODRONES_STRAIGHT_H
