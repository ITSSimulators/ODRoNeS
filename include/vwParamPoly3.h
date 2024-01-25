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

#ifndef VWPARAMPOLY3_H
#define VWPARAMPOLY3_H

class vwParamPoly3 : public vwNumerical
{
public:

    vwParamPoly3(const Odr::geometry &odr, int sign, std::vector<Odr::offset> vwWidth,
                 std::vector<Odr::offset> ioffset, scalar so, scalar se, scalar roadSo, bool geomPrint);

    void base() override;

    arr2 clxy(scalar t) const; ///< returns the point of the central lane (lane 0, or reference lane) at t;
    arr2 clPxy(scalar t) const; ///< returns the first derivative of the central lane (lane 0, or reference lane) at t;
    arr2 clP2xy(scalar t) const; ///< returns the second derivative of the central lane (lane 0, or reference lane) at t;
    arr2 vwpp3xy(scalar t) const; ///< returns the point of the lane at t;
    arr2 vwpp3Pxy(scalar t) const; ///< returns the first derivative of the lane at t;

    arr2 clNormal(scalar t) const; ///< return the normal at t;
    arr2 clNormalP(scalar t) const; ///< return the first derivative of the normal at t;

    scalar calcLength() const;

private:
    arr2 curvexy_a(scalar t) const override; ///< normalised or not, here t is the distance in meters.

private:

    std::array<scalar, 4> _u; ///< vwParamPoly3; u[a, b, c, d];
    std::array<scalar, 4> _v; ///< vwParamPoly3; v[a, b, c, d];
    arr2 _po; ///< a private origin for the centre of the lane that is unchanged on inversion
    arr2 _pto; ///< a private to for the centre of the lane that is unchanged on inversion.
    scalar _roadSo; ///< the starting point down the road at which this geometry starts.
    bool _normalised; ///< vwParamPoly3; whether _u and _v come normalised or at length.
    int _sign;

};


#endif // VWPARAMPOLY3_H
