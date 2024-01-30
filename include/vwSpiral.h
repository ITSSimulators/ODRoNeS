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
#include <functional>
#include "vwNumerical.h"
#include "Clothoid.hh"

#ifndef VWSPIRAL_H
#define VWSPIRAL_H

class vwSpiral : public vwNumerical
{
public:

    vwSpiral(const Odr::geometry &odr, int sign, std::vector<Odr::offset> ioffset,
             scalar so, scalar se, scalar roadSo, bool geomPrint);

    void base() override;

    arr2 clxy(scalar t) const; ///< returns the point of the central lane (lane 0, or reference lane) at t;
    arr2 clPxy(scalar t) const; ///< returns the first derivative of the central lane (lane 0, or reference lane) at t;
    arr2 clP2xy(scalar t) const; ///< returns the second derivative of the central lane (lane 0, or reference lane) at t;

    arr2 xy(scalar t) const; ///< returns the point of the lane at t;
    arr2 xyP(scalar t) const; ///< returns the first derivative of the lane at t;

    arr2 clNormal(scalar t) const; ///< return the normal at t;
    arr2 clNormalP(scalar t) const; ///< return the first derivative of the normal at t;


private:
    arr2 curvexy_a(scalar t) const override; ///< normalised or not, here t is the distance in meters.
    arr2 l0xy_a(scalar t) const override; ///< normalised or not, here t is the distance in meters.

private:

    G2lib::ClothoidCurve _clothoid;
    int _sign;

};


#endif // VWSPIRAL_H
