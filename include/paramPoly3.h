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

#include <array>
#include "parametric.h"
#include "numerical.h"

#ifndef ODRONES_PARAMPOLY3_H
#define ODRONES_PARAMPOLY3_H

namespace odrones
{
class paramPoly3 : public parametric, public numerical
{
public:

    paramPoly3(const Odr::geometry &odr, int sign, scalar offsetA, scalar so, scalar se, scalar roadSo);
    paramPoly3(const paramPoly3& p3);
    paramPoly3& operator=(const paramPoly3& p3);
    void assignInputGeomToThis(const paramPoly3& p3);

    void base() override;
    void invert() override;

    void nSetupPointsXYUniformly(scalar ds) override;

    arr2 projectPointHere(const arr2 &p) const override;   ///< parametric
    bool isPointHere(const arr2 &p) const override;        ///< box, parametric & numerical
    arr2 getTangentInPoint(const arr2 &p) const override;  ///< parametric
    scalar distanceToTheEoL(const arr2 &p) const override; ///< numerical
    bool getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const override; ///< numerical
    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override; ///< numerical
    scalar getCurvature(const arr2 &p) const override;     ///< numerial

    arr2 clxy(scalar t) const; ///< returns the point of the central lane (lane 0, or reference lane) at t;
    arr2 clPxy(scalar t) const; ///< returns the first derivative of the central lane (lane 0, or reference lane) at t;
    arr2 clP2xy(scalar t) const; ///< returns the second derivative of the central lane (lane 0, or reference lane) at t;
    arr2 curvexy(scalar t) const override; ///< returns the point of the lane at t;
    arr2 curvePxy(scalar t) const override; ///< returns the first derivative of the lane at t;
    arr2 curveP2xy(scalar t) const override; ///< returns the second derivative of the lane at t;

    arr2 clNormal(scalar t) const; ///< return the normal at t;
    arr2 clNormalP(scalar t) const; ///< return the first derivative of the normal at t;

    scalar calcLength() const;

    bool getTGivenXY(scalar &t, const arr2 &xy) const override; ///< NOT IMPLEMENTED!

    bool isArc() const override {return false;}
    bool isNumerical() const override {return true;}
    scalar sl0(scalar s) const override;

#ifdef QT_CORE_LIB
    QPainterPath getQPainterPath(uint n) const override;
#endif // QT_CORE_LIB

private:

    std::array<scalar, 4> _u; ///< paramPoly3; u[a, b, c, d];
    std::array<scalar, 4> _v; ///< paramPoly3; v[a, b, c, d];
    arr2 _po; ///< a private origin for the centre of the lane that is unchanged on inversion
    arr2 _pto; ///< a private to for the centre of the lane that is unchanged on inversion.
    bool _normalised; ///< paramPoly3; whether _u and _v come normalised or at length.
    int _sign;
    scalar _offset; ///< constant offset from the center of lane 0; positive scalar.
    scalar _odrLength; ///< paramPoly3; the length of the reference lane (0).

};

} // namespace odrones;


#endif // ODRONES_PARAMPOLY3_H
