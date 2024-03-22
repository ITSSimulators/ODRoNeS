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
#include "numerical.h"
#include <functional>

#ifndef ODRONES_VWNUMERICAL_H
#define ODRONES_VWNUMERICAL_H

namespace odrones
{

class vwNumerical : public geometry, public numerical
{
public:
    vwNumerical();
    vwNumerical(const vwNumerical& vws);
    vwNumerical& operator=(const vwNumerical &vws);
    virtual ~vwNumerical();

    void base() override;
    void assignInputGeomToThis(const vwNumerical &vws);

    bool setup(scalar ds);
    void nSetupPointsXYUniformly(scalar ds) override; /// deprecated? We're now using fillInSPoints, which does S & points.

    void invert() override;
    bool isArc() const override {return false;}
    bool isNumerical() const override {return true;}
    bool isPointHere(const arr2 &p) const override;
    arr2 projectPointHere(const arr2 &p) const override;
    arr2 getTangentInPoint(const arr2 &p) const override;
    scalar distanceToTheEoL(const arr2 &p) const override;
    bool getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const override;
    bool getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const override;
    scalar getCurvature(const arr2 &p) const override;

    scalar sl0(scalar d) const override;

#ifdef QT_CORE_LIB
    QPainterPath getQPainterPath(uint n) const override;
#endif

protected:
    scalar offset_a(scalar t) const; ///< return the value of offset(s) ahead
    scalar offset_b(scalar t) const; ///< return the value of offset(s) backwards
    std::function<scalar(scalar)> offset; ///< offset will store either offset_a or offset_b;

    scalar offsetP_a(scalar t) const;  ///< return the value of offset'(s) ahead; t is non-normalised.
    scalar offsetP_b(scalar t) const; ///< return the value of offset'(s) backwards; t is non-normalised.
    std::function<scalar(scalar)> offsetP;  ///< offset will store either offsetP_a or offsetP_b; t is non-normalised.

    virtual arr2 curvexy_a(scalar t) const = 0; ///< return the value of curvexy(s) ahead
    arr2 curvexy_b(scalar t) const; ///< return the value of curvexy(s) backwards
    std::function<arr2(scalar)> curvexy; ///< curvexy will store either curvexy_a or curvexy_b

    virtual arr2 l0xy_a(scalar t) const = 0;
    arr2 l0xy_b(scalar t) const;
    std::function<arr2(scalar)> l0xy;

    //! Fill in the the array points (and S) with
    //!   points separated ds (and distance to the origin)
    //!   running small increment steps dt along curvexy(t) from _mint to _maxt
    void fillInT(std::vector<scalar> &T, scalar &maxSo, scalar &maxS, scalar ds, scalar dt) const;
protected:
    arr2 _no;
    scalar _l; ///< lane zero length;
    scalar _mint, _maxt; ///< vwNumerical could be also a parametric curve.
    std::vector<Odr::offset> _vwOff;
    bool _ahead; ///< auxilliary boolean to know whether we're using ahead _a or backwards _b functions.
};

} // namespace odrones;

#endif // ODRONES_VWNUMERICAL_H
